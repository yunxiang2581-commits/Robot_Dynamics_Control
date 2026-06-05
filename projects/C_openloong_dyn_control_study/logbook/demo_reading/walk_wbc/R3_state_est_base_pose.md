# walk_wbc 第三轮阅读笔记

## 1. 本轮目标

第三轮只回答一个问题：

```text
为什么 MJ_Interface 已经从 MuJoCo 读到了 basePos / baseLinVel，
walk_wbc 还要再经过 StateEst，最后再覆盖 DataBus.q / dq 的 base 部分？
```

本轮重点不是 WBC，也不是动力学，而是：

- `StateEst::init(DataBus &Data)`
- `StateEst::set(DataBus &Data)`
- `StateEst::update()`
- `StateEst::get(DataBus &Data)`

## 2. 这一步为什么存在

第二轮已经确认：

- `MJ_Interface` 能从 MuJoCo 读到关节、IMU、base 姿态等原始观测
- `DataBus::updateQ()` 能先拼出一版初始 `q / dq`

但控制器真正需要的不是“原始观测直接拼起来的状态”，而是：

- 更平滑的姿态和角速度
- 经过支撑脚约束修正的 base 位置和速度
- 更适合后续运动学、动力学和 WBC 的 floating-base 状态

所以 `StateEst` 的职责是：

```text
把原始观测 + 腿部运动学关系
融合成一版更稳定的 base 状态估计，
然后写回 DataBus，覆盖 q / dq 里的 base 部分。
```

## 3. walk_wbc 中的调用位置

`walk_wbc.cpp` 主循环里的顺序是：

```cpp
mj_interface.updateSensorValues();
mj_interface.dataBusWrite(RobotState);

if (simTime > 1 && StateModule.flag_init)
    StateModule.init(RobotState);

StateModule.set(RobotState);
StateModule.update();
StateModule.get(RobotState);
```

这说明：

```text
MJ_Interface 先给 RobotState 填一版“原始状态”
-> StateEst 读入这些原始量
-> 内部估计
-> 再把估计结果写回 RobotState
```

所以第三轮的核心结论先提前说出来：

```text
DataBus::updateQ() 不是最终版 q / dq，
StateEst::get() 才会把最终用于后续控制的 base 状态写回去。
```

## 4. StateEst 内部到底在估计什么

从 `StateEst.cpp` 里的状态维度可以看出：

- `X` 是 `15 x 1`
- `A` 是 `15 x 15`
- `B` 是 `15 x 3`
- `C` 是 `14 x 15`
- `Y` 是 `14 x 1`

结合 `update()` 末尾的拆包：

```cpp
peW.block<3, 1>(0, 0) = X.segment<3>(6);
peW.block<3, 1>(0, 1) = X.segment<3>(9);
base_pos = X.segment<3>(0);
base_vel = X.segment<3>(3);
delta_acc = X.segment<3>(12);
```

可以读出状态向量的物理含义：

```text
X =
[ base_pos(3);
  base_vel(3);
  left_foot_pos_W(3);
  right_foot_pos_W(3);
  delta_acc(3) ]
```

也就是说，这个估计器主要在估：

1. base 位置
2. base 速度
3. 左脚世界系位置
4. 右脚世界系位置
5. 加速度修正项

这已经不是单纯“滤一下 IMU”，而是一个小型状态估计器。

## 5. init() 在做什么

`init()` 的作用是给 Kalman 状态一个初始值 `X0` 和协方差 `P0`。

关键代码：

```cpp
offYaw = Data.rpy[2];
R = eul2Rot(Data.base_rpy[0], Data.base_rpy[1], 0);
pWL = R * fe_l_pos_L;
pWR = R * fe_r_pos_L;
double zOff = -(pWR(2) + pWL(2)) / 2.0 + 0.07;
X0 << 0, 0, zOff,
      0, 0, 0,
      pWL(0), pWL(1), 0.07,
      pWR(0), pWR(1), 0.07,
      0, 0, 0;
```

这里做了三件事：

1. 记录一个 `offYaw`
2. 用当前腿部末端位置估一个初始 base 高度
3. 给左右脚世界系位置一个初始值

### 5.1 `offYaw` 的意义

`offYaw = Data.rpy[2]`

它不是说 yaw 不重要，而是说：

```text
估计器内部更关心“去掉全局航向零点后的姿态变化”
```

后面 `eul_woOff` 就是：

```text
eul_woOff = 当前 eul - 初始 yaw 偏置
```

这样可以减少全局航向角对局部估计的干扰。

### 5.2 为什么用脚来初始化 base 高度

如果双脚一开始站在地上，那么：

- 已知脚在 body 坐标系下的位置
- 已知脚底大致应该接近地面高度 `0.07`

就可以反过来估出 body 大概应在多高的位置。

所以这里的思想是：

```text
“站立时脚更可信”
-> 用脚的位置关系反推初始 base 高度
```

## 6. set() 读入了哪些观测

`set()` 不做 Kalman 更新，它主要是把 `DataBus` 里的当前观测搬进 `StateEst` 内部。

读入的核心量有：

- `acc <- Data.baseAcc`
- `eul <- Data.rpy`
- `omegaL <- Data.baseAngVel`
- `omegaW <- Data.base_omega_W`
- `phi <- Data.phi`
- `legState <- Data.legState`
- `fe_l_pos_L / fe_r_pos_L`
- `fe_l_vel_L / fe_r_vel_L`
- `fe_l_pos_W / fe_r_pos_W`

### 6.1 这里最关键的不是位置，而是“姿态 + IMU + 足端运动学”

这说明 `StateEst` 的输入来源有三类：

1. IMU：`baseAcc`, `baseAngVel`
2. 姿态：`rpy`
3. 运动学：左右脚相对 body 的位置和速度

估计器并不是盯着 MuJoCo 的 `xpos` 直接相信，而是重新融合这些量。

### 6.2 `eul_w_filter` 在这里先做了一轮滤波

代码里先执行：

```cpp
eul_w_filter.run(eul_woOff_ary, omegaL_ary);
eul_w_filter.getData(Eul_filtered, wL_filtered);
```

然后才更新：

```cpp
eul_woOff << Eul_filtered[0], Eul_filtered[1], Eul_filtered[2];
omegaL << wL_filtered[0], wL_filtered[1], wL_filtered[2];
omegaW = Rrpy_woOff * omegaL;
```

所以 `StateEst` 并不是原样用 `MJ_Interface` 给的姿态和角速度，而是先做了一轮滤波和平滑。

#### 6.2.1 这个滤波器到底在滤什么

`Eul_W_filter` 不是单纯低通，而是一个 6 维 Kalman 滤波器。

它内部维护的状态可以写成：

```text
x =
[ roll,
  pitch,
  yaw,
  wx,
  wy,
  wz ]
```

其中：

- 前 3 维是欧拉角
- 后 3 维是机体系角速度 `omegaL`

当前一拍的测量量是：

```text
z =
[ roll_meas,
  pitch_meas,
  yaw_meas,
  wx_meas,
  wy_meas,
  wz_meas ]
```

也就是说，它是把“姿态角”和“角速度”放在一起联合滤波，而不是各自独立平滑。

#### 6.2.2 它为什么需要“预测”

这里的预测值不是为了控制下一步动作，而是为了给“当前时刻状态”一个先验参考。

最适合当前阅读阶段的理解是：

```text
测量值负责告诉我们“这一拍看起来是多少”
预测值负责告诉我们“按照上一拍的运动趋势，这一拍本来大概应该是多少”
```

然后滤波器再把两者融合，得到更平滑、更连续、更抗噪的当前状态。

如果只信当前测量：

- 姿态会抖
- 角速度会抖
- 后续 `StateEst`、`Pin_KinDyn`、`WBC` 都会跟着抖

所以这里先加一层小滤波器，专门把姿态与角速度整理干净。

#### 6.2.3 这套滤波的数学骨架

定义状态：

```text
x_k = [roll, pitch, yaw, wx, wy, wz]^T
```

定义测量：

```text
z_k = [roll_m, pitch_m, yaw_m, wx_m, wy_m, wz_m]^T
```

观测方程非常简单：

```text
z_k = H x_k + v_k
```

在这份代码里：

```text
H = I
```

也就是：

```text
测量值就是状态本身，只是带测量噪声 v_k
```

预测模型的核心来自欧拉角运动学关系：

```text
eul_dot = T(eul) * omegaL
```

把它做一阶离散化后，可以写成：

```text
x_{k|k-1} = F_k x_{k-1|k-1} + w_k
```

其中：

- `F_k`：由当前 `roll/pitch` 决定的状态转移矩阵
- `w_k`：过程噪声

这就是源码里 `F(0,3) ... F(2,5)` 那些项的来源：它们在表达“角速度会如何让欧拉角在一小步时间 `dt` 内发生变化”。

#### 6.2.4 一拍滤波到底做了什么

把源码逻辑翻成人话，就是下面四步：

1. 先把当前测量装成 `z_k`
2. 根据上一拍最终状态，推一个当前预测状态 `x_{k|k-1}`
3. 比较“当前测量”和“当前预测”差多少
4. 按照当前可信度，把预测和测量融合成当前最终状态 `x_{k|k}`

对应公式是：

```text
预测：
x_{k|k-1} = F_k x_{k-1|k-1}
P_{k|k-1} = F_k P_{k-1|k-1} F_k^T + Q

校正：
y_k = z_k - H x_{k|k-1}
S_k = H P_{k|k-1} H^T + R
K_k = P_{k|k-1} H^T S_k^{-1}
x_{k|k} = x_{k|k-1} + K_k y_k
P_{k|k} = (I - K_k H) P_{k|k-1}
```

这里各符号的阅读定位是：

- `x`：当前估计状态
- `z`：当前测量
- `y`：测量和预测的差
- `P`：当前估计不确定度
- `Q`：预测模型噪声
- `R`：测量噪声
- `K`：这次融合时更信预测还是更信测量的权重

#### 6.2.5 一个最小 1 维直觉例子

如果先只看 `roll` 一个量：

- 预测值：`roll_pred = 0.10`
- 测量值：`roll_meas = 0.18`

则残差是：

```text
y = roll_meas - roll_pred = 0.08
```

如果这次融合权重近似为：

```text
K = 0.6
```

那校正后的结果就是：

```text
roll_new = roll_pred + K * y
         = 0.10 + 0.6 * 0.08
         = 0.148
```

这说明：

```text
Kalman 校正不是二选一，
而是“按权重把预测往测量方向拉一部分”
```

放回这份代码里，就是：

```text
上一拍最终姿态/角速度
-> 按欧拉角运动学预测这一拍
-> 用这一拍姿态与陀螺仪测量修正
-> 得到更平滑的 eul_woOff 和 omegaL
```

### 6.3 `freeAcc` 是什么

关键代码：

```cpp
freeAcc = eul2Rot(0.0, 0.0, offYaw).transpose() * accTmp;
```

可以先把它理解成：

```text
把 IMU 加速度按当前估计器使用的 yaw 参考系重表达，
作为预测模型里的输入加速度
```

这一轮先不深究它是否已经完全去除了重力项，先记它的角色：

- `freeAcc` 是 `update()` 预测步的输入

## 7. update() 是这一轮最核心的函数

`update()` 做的是标准的“预测 + 校正”。

## 7.1 先决定哪条腿当前更可信

代码：

```cpp
if (legState == DataBus::LSt) {
    leg_contact[0] = true;
    leg_contact[1] = false;
}
else if (legState == DataBus::RSt) {
    leg_contact[1] = true;
    leg_contact[0] = false;
}
else {
    leg_contact[0] = true;
    leg_contact[1] = true;
}
```

这里暂时没直接用估计接触力判定接触，而是先按步态状态机：

- 左支撑：左脚可信，右脚不可信
- 右支撑：右脚可信，左脚不可信
- 双支撑/站立：双脚都可信

这会直接影响后面 measurement 的权重。

## 7.2 `getTrustRegion_wt_h()` 在调权重

这个函数根据：

- `leg_contact`
- `legState`
- `phi`

生成：

- `Xi`
- `Xiv`
- `Xih`

它们本质上是在调不同测量项的置信度。

可以先粗记为：

```text
支撑脚阶段：脚位置/脚速度/脚高度更可信
摆动脚阶段：相关约束权重降低
```

## 7.3 预测步：用离散模型推进状态

关键代码：

```cpp
X = A * X + B * freeAcc;
P = A * P * A.transpose() + Q;
```

它对应的直觉是：

- 位置由速度积分推进
- 速度由加速度推进
- 脚位置和加速度偏置按模型推进

所以这一步是“只靠上一时刻状态和 IMU 输入，先猜这一时刻在哪里”。

## 7.4 校正步：用脚约束把 base 拉回来

这一段最重要：

```cpp
pbW = Rrpy_woOff * peB;
vbW.col(i) = leg_contact[i] * Rrpy_woOff * (velTmp.col(i) + omegaLVec.cross(peB.col(i)))
           + (1 - leg_contact[i]) * (-base_vel);
Y.segment<3>(0) = -pbW.block<3, 1>(0, 0);
Y.segment<3>(3) = -pbW.block<3, 1>(0, 1);
Y.segment<3>(6) = vbW.block<3, 1>(0, 0);
Y.segment<3>(9) = vbW.block<3, 1>(0, 1);
```

物理意思是：

### 脚位置约束

如果某只脚是支撑脚，那么它相对 body 的位置已知，经过姿态旋转后，就能给出：

```text
body 在世界系里不应随便漂
```

### 脚速度约束

如果某只脚正在支撑地面，那么它在世界系速度应该接近 0。

这会变成对 `base_vel` 的反向约束。

所以本质逻辑是：

```text
IMU 负责“推着状态往前预测”
支撑脚约束负责“把飘掉的 base 拉回来”
```

这就是为什么不能只直接用 MuJoCo 读出来的 base 线速度。

## 7.5 最后从 X 里拆出估计结果

`update()` 末尾：

```cpp
base_pos = X.segment<3>(0);
base_vel = X.segment<3>(3);
delta_acc = X.segment<3>(12);
fe_l_pos_W = peW.block<3, 1>(0, 0);
fe_r_pos_W = peW.block<3, 1>(0, 1);
```

也就是说，这一轮估计结果正式生成了：

- `base_pos`
- `base_vel`
- `delta_acc`
- 左右脚世界系位置

## 7.6 `update()` 完整小结

把 `StateEst::update()` 从头到尾压成一条主线，就是：

```text
1. 根据 legState 先决定当前哪只脚可信
2. 根据接触脚状态和相位 phi 调整本拍脚约束权重
3. 用上一拍状态 X 和 IMU 输入 freeAcc 做 prediction
4. 用脚位置 / 脚速度 / 脚高度关系构造 measurement Y
5. 用 Kalman correction 把“支撑脚踩地约束”融合回当前状态
6. 从 15 维状态向量 X 中拆出 base_pos / base_vel / feet_W / delta_acc
```

如果换成更物理的说法：

```text
IMU 告诉估计器“机器人大概怎么动了”
支撑脚告诉估计器“机器人不应该漂到哪里去”
update() 的工作就是把这两类信息融合起来
```

因此，`StateEst::update()` 的本质不是“重新读一次 base 状态”，而是：

```text
先用 IMU 做 base 的先验预测，
再用“支撑脚应当踩稳地面、不应乱滑”的几何与速度约束做校正，
最终得到更可信的 floating-base base 状态。
```

## 8. get() 为什么会覆盖 DataBus

`get()` 的关键不是“导出一些估计量”，而是：

```cpp
Data.base_pos = Data.base_pos_est;
Data.base_vel = Data.base_vel_est;
Data.base_rpy = Data.eul_est;
Data.base_omega_W = Data.omegaW_est;

Data.q.block<3, 1>(0, 0) = Data.base_pos;
Data.dq.block<3, 1>(0, 0) = Data.base_vel;
```

## 8.1 `get()` 的真正作用

`get()` 可以分成三层：

1. 先把内部结果写到 `*_est` 字段  
   例如：
   - `base_pos_est`
   - `base_vel_est`
   - `eul_est`
   - `omegaW_est`

2. 再把这些估计量提升为当前正式 base 状态  
   例如：
   - `base_pos`
   - `base_vel`
   - `base_rpy`
   - `base_omega_W`
   - `base_rot`

3. 最后直接覆盖 floating-base 的 `q / dq` 前段  
   例如：
   - `q[0:2] = base_pos`
   - `q[3:6] = quaternion(base_rpy)`
   - `dq[0:2] = base_vel`
   - `dq[3:5] = base_omega_W`

所以它不是“多存一份估计结果”，而是：

```text
把状态估计结果真正注入后续控制链
```

## 9. 四个主函数总表

| 函数 | 作用 | 输入主线 | 输出主线 |
|---|---|---|---|
| `init(DataBus &Data)` | 给估计器定初值 | `rpy`、`base_rpy`、双脚相对 body 位置 | `X0`、`P0`、`offYaw` |
| `set(DataBus &Data)` | 读入并预处理本拍观测 | IMU、姿态、角速度、步态状态、双脚位置速度 | `freeAcc`、`eul_woOff`、`omegaL/omegaW`、`fe_*` |
| `update()` | 做完整一拍 Kalman 递推 | 上一拍 `X/P` + 当前 `freeAcc` + 当前脚约束输入 | 更新后的 `X/P`、`base_pos/base_vel/feet_W/delta_acc` |
| `get(DataBus &Data)` | 把估计结果写回 DataBus 和 `q/dq` | `update()` 的输出 | 正式 `base_*` 字段和覆盖后的 floating-base `q/dq` |

## 10. 第三轮最终结论

第三轮读完后，可以把 `walk_wbc` 里这条链明确写成：

```text
MuJoCo 原始观测
-> MJ_Interface / DataBus::updateQ() 先拼一版原始 q/dq
-> StateEst::set() 读取并预处理观测
-> StateEst::update() 用 IMU prediction + 支撑脚约束 correction 做状态估计
-> StateEst::get() 把估计结果写回 base_* 和 q/dq
-> 后续 Pin_KinDyn / WBC / 控制器使用的是估计后的 floating-base 状态
```

因此，这一轮最重要的认识是：

```text
StateEst 不是可有可无的平滑器，
而是 walk_wbc 中 floating-base base 状态的正式来源。
```

## 11. 第四轮建议

第四轮建议进入：

```text
StateEst 输出的 q / dq
-> Pin_KinDyn::dataBusRead()
-> computeJ_dJ()
-> computeDyn()
-> dataBusWrite()
```

目标是回答：

```text
估计后的 floating-base 状态
如何变成 Jacobian、dJ、M、Nonlinear 等 WBC 输入。
```

## 12. 这一轮暂时不展开的点

先留到后面再深挖：

- `A / B / C` 每一块矩阵的严格数学意义
- `delta_acc` 是否对应 IMU 偏置或未建模项
- `freeAcc` 是否已经完全扣除重力
- `setF / updateF / getF` 的接触力估计链

所以本质逻辑是：

```text
IMU 负责“推着状态往前预测”
支撑脚约束负责“把飘掉的 base 拉回来”
```

这就是为什么不能只直接用 MuJoCo 读出来的 base 线速度。

## 7.5 最后从 X 里拆出估计结果

`update()` 末尾：

```cpp
base_pos = X.segment<3>(0);
base_vel = X.segment<3>(3);
delta_acc = X.segment<3>(12);
fe_l_pos_W = peW.block<3, 1>(0, 0);
fe_r_pos_W = peW.block<3, 1>(0, 1);
```

也就是说，这一轮估计结果正式生成了：

- `base_pos`
- `base_vel`
- `delta_acc`
- 左右脚世界系位置

## 7.6 `update()` 完整小结

把 `StateEst::update()` 从头到尾压成一条主线，就是：

```text
1. 根据 legState 先决定当前哪只脚可信
2. 根据接触脚状态和相位 phi 调整本拍脚约束权重
3. 用上一拍状态 X 和 IMU 输入 freeAcc 做 prediction
4. 用脚位置 / 脚速度 / 脚高度关系构造 measurement Y
5. 用 Kalman correction 把“支撑脚踩地约束”融合回当前状态
6. 从 15 维状态向量 X 中拆出 base_pos / base_vel / feet_W / delta_acc
```

如果换成更物理的说法：

```text
IMU 告诉估计器“机器人大概怎么动了”
支撑脚告诉估计器“机器人不应该漂到哪里去”
update() 的工作就是把这两类信息融合起来
```

因此，`StateEst::update()` 的本质不是“重新读一次 base 状态”，而是：

```text
先用 IMU 做 base 的先验预测，
再用“支撑脚应当踩稳地面、不应乱滑”的几何与速度约束做校正，
最终得到更可信的 floating-base base 状态。
```

## 8. get() 为什么会覆盖 DataBus

`get()` 的关键不是“导出一些估计量”，而是：

```cpp
Data.base_pos = Data.base_pos_est;
Data.base_vel = Data.base_vel_est;
Data.base_rpy = Data.eul_est;
Data.base_omega_W = Data.omegaW_est;

Data.q.block<3, 1>(0, 0) = Data.base_pos;
Data.dq.block<3, 1>(0, 0) = Data.base_vel;
...
Data.dq.block<3, 1>(3, 0) = Data.base_omega_W;
```

这几句说明：

```text
StateEst::get() 不是“额外保存一份估计结果”
而是直接把 DataBus 中 floating-base 的 base 部分改写掉
```

因此：

- 第二轮 `MJ_Interface + updateQ()` 拼出来的 `q/dq`
- 到第三轮这里被 `StateEst::get()` 覆盖

这就是为什么第二轮里看到：

```text
basePos / baseLinVel 没直接从 MJ_Interface 写进最终控制状态
```

因为最终版本要等 `StateEst`。

## 9. 第三轮核心结论

第三轮读完，可以把主结论写成一句话：

```text
StateEst 的作用不是重复读取 MuJoCo 状态，
而是用 IMU + 姿态 + 支撑脚运动学约束，
重新估计更稳定的 floating-base base 状态，
并覆盖 DataBus.q / dq 的 base 部分，供后续 Pin_KinDyn、WBC 和控制器使用。
```

## 10. 这一轮暂时不展开的点

先留到后面再深挖：

- `A / B / C` 每一块矩阵的严格数学意义
- `delta_acc` 是否对应 IMU 偏置或未建模项
- `freeAcc` 是否已经完全扣除重力
- `setF / updateF / getF` 的接触力估计链

## 11. 下一轮建议

第四轮建议只读这条链：

```text
StateEst 输出的 q / dq
-> Pin_KinDyn::dataBusRead()
-> computeJ_dJ()
-> computeDyn()
-> dataBusWrite()
```

目标是回答：

```text
估计后的 floating-base 状态
是怎么变成 Jacobian、dJ、M、Nonlinear 等 WBC 输入的。
```
