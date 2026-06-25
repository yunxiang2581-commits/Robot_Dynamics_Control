# R1: walk_mpc_wbc 主文件差异阅读

## 1. 本轮目标

本轮只阅读：

```text
external/open_source_repos/OpenLoong-Dyn-Control/demo/walk_mpc_wbc.cpp
```

重点是对比 `walk_wbc.cpp`，找出主文件层面的差异。

本轮不深入 `MPC::cal()` 的 QP 细节，只回答：

1. MPC 对象在哪里创建？
2. MPC 以什么频率运行？
3. MPC 在主循环中插在哪里？
4. WBC/PVT 输出链路有没有改变？
5. 哪些日志变量是 MPC 新增的？

## 2. 第 25-26 行：双时间尺度

源码：

```cpp
const double dt = 0.001;
const double dt_200Hz = 0.005;
```

这里定义了两个常量时间步。

主控制/仿真时间步：

$$
dt=0.001s
$$

对应频率：

$$
f=\frac{1}{0.001}=1000Hz
$$

MPC 时间步：

$$
dt_{MPC}=0.005s
$$

对应频率：

$$
f_{MPC}=\frac{1}{0.005}=200Hz
$$

所以：

```text
WBC / PVT / MuJoCo 主循环约 1000 Hz；
MPC 约 200 Hz。
```

二者关系是：

$$
\frac{dt_{MPC}}{dt}
=
\frac{0.005}{0.001}
=
5
$$

也就是主控制循环每跑 5 次，MPC 运行 1 次。

## 3. 第 39 行：创建 MPC 对象

源码：

```cpp
MPC MPC_solv(dt_200Hz);
```

这句创建了一个 MPC 控制器对象：

```text
类型: MPC
对象名: MPC_solv
构造参数: dt_200Hz
```

等价理解：

$$
dtIn = dt_{MPC}=0.005
$$

MPC 内部预测模型会按 5 ms 的离散步长工作。

它和 WBC 的关系不是并列输出电机力矩，而是上下游关系：

$$
MPC\_solv
\rightarrow
RobotState
\rightarrow
WBC\_solv
$$

其中：

```text
MPC_solv: 负责整体 base/质心预测和接触力前馈；
WBC_solv: 负责全身任务优先级、动力学一致性和关节力矩。
```

## 4. 第 105-109 行：新增 MPC 日志变量

源码：

```cpp
logger.addIterm("dX_cal", 12);
logger.addIterm("Ufe", 12);
logger.addIterm("Xd", 12);
logger.addIterm("X_cur", 12);
logger.addIterm("X_cal", 12);
```

这些是 `walk_wbc.cpp` 中没有的 MPC 中间量。

MPC 的核心状态维度是：

$$
X\in\mathbb{R}^{12}
$$

后续 `MPC::dataBusRead()` 会把它组织成：

$$
X=
\begin{bmatrix}
rpy\\
p\\
\omega\\
v
\end{bmatrix}
=
\begin{bmatrix}
roll\\
pitch\\
yaw\\
x\\
y\\
z\\
\omega_x\\
\omega_y\\
\omega_z\\
v_x\\
v_y\\
v_z
\end{bmatrix}
$$

各日志变量含义：

```text
X_cur: 当前 MPC 状态
Xd: MPC 期望状态
X_cal: MPC 预测/计算状态
dX_cal: MPC 计算出的状态变化率
Ufe: 足端接触 wrench / 接触控制量
```

注意日志注册中 `Ufe` 是 12 维，实际后面记录的是：

$$
RobotState.Fr\_{ff}
$$

也就是 WBC 使用的双脚接触 wrench 前馈。

## 5. 第 114 行：MPC_count

源码：

```cpp
int MPC_count = 0; // count for controlling the mpc running period
```

它是 MPC 调度计数器。

因为主循环是：

$$
dt=0.001s
$$

而 MPC 周期是：

$$
dt_{MPC}=0.005s
$$

所以需要计数器实现：

```text
主循环每跑一次，MPC_count 加 1；
累计到 5 次时，运行一次 MPC。
```

## 6. 第 140-159 行：上游目标生成基本不变

源码结构：

```cpp
jsInterp.setWzDesLPara(...);
jsInterp.setVxDesLPara(...);
jsInterp.step();
jsInterp.dataBusWrite(RobotState);

gaitScheduler.dataBusRead(RobotState);
gaitScheduler.step();
gaitScheduler.dataBusWrite(RobotState);

footPlacement.dataBusRead(RobotState);
footPlacement.getSwingPos();
footPlacement.dataBusWrite(RobotState);
```

这一段和 `walk_wbc.cpp` 基本相同。

这里有两个容易漏掉的小差别：

1. `walk_mpc_wbc.cpp` 里没有 `StateEst` 这一层，所以它走的是更直接的 `sensor -> DataBus -> kinDyn` 路径。
2. `jsInterp.step()` 在这里是每拍都跑；而 `walk_wbc.cpp` 把这一步放在 `startSteppingTime` 之后。

前者是结构差异，后者是命令更新时序差异。

它生成：

$$
js\_pos\_{des},\quad
js\_vel\_{des},\quad
js\_\omega\_{des}
$$

以及：

$$
legState,\quad
legStateNext,\quad
\phi
$$

和摆动脚目标位置。

区别在于：

```text
walk_wbc: 这些目标后面直接喂给 WBC；
walk_mpc_wbc: 这些目标先进入 MPC，再由 MPC 改写 WBC 的部分输入。
```

## 7. 第 161-168 行：MPC 插入点

源码：

```cpp
MPC_count = MPC_count + 1;
if (MPC_count > (dt_200Hz / dt - 1)) {
    MPC_solv.dataBusRead(RobotState);
    MPC_solv.cal();
    MPC_solv.dataBusWrite(RobotState);
    MPC_count = 0;
}
```

代入时间步：

$$
\frac{dt_{200Hz}}{dt}-1
=
\frac{0.005}{0.001}-1
=
4
$$

条件为：

$$
MPC\_count > 4
$$

所以当：

$$
MPC\_count=5
$$

时运行一次 MPC。

MPC 的数据流是：

$$
RobotState
\rightarrow
MPC::dataBusRead()
\rightarrow
MPC::cal()
\rightarrow
MPC::dataBusWrite()
\rightarrow
RobotState
$$

也就是说，MPC 不是直接输出电机力矩，而是通过 `RobotState` 把结果交给后续 WBC。

## 8. 第 170-175 行：WBC 调用保持不变

源码：

```cpp
WBC_solv.dataBusRead(RobotState);
WBC_solv.computeDdq(kinDynSolver);
WBC_solv.computeTau();
WBC_solv.dataBusWrite(RobotState);
```

这几句和 `walk_wbc.cpp` 基本一样。

但是在 `walk_mpc_wbc.cpp` 里，WBC 读取的 `RobotState` 可能已经被 MPC 更新过。

WBC 可能读到 MPC 写入的：

$$
F_{r,ff}
$$

$$
des\_\Delta q,\quad
des\_{\dot q},\quad
des\_{\ddot q}
$$

以及：

$$
base\_pos_{des},\quad
base\_rpy_{des}
$$

所以关键变化不是 WBC 调用形式变了，而是 WBC 的上游输入变了。

对比：

```text
walk_wbc:
手写 Fr_ff 和 base des -> WBC

walk_mpc_wbc:
MPC 输出 Fr_ff 和 base des -> WBC
```

## 9. 第 177-181 行：3 秒前保持初始姿态

源码：

```cpp
if (simTime <= startSteppingTime) {
    RobotState.motors_pos_des = eigen2std(resLeg.jointPosRes + resHand.jointPosRes);
    RobotState.motors_vel_des = motors_vel_des;
    RobotState.motors_tor_des = motors_tau_des;
}
```

其中：

$$
startSteppingTime=3s
$$

所以前 3 秒机器人不进入完整行走控制，而是保持初始化 IK 姿态：

$$
q_{j,des}=q_{IK,init}
$$

$$
\dot q_{j,des}=0
$$

$$
\tau_{j,des}=0
$$

这一步用于让机器人平稳进入初始站立姿态。

## 10. 第 182-195 行：启用 MPC 并设置权重

源码：

```cpp
MPC_solv.enable();
Eigen::Matrix<double, 1, nx>  L_diag;
Eigen::Matrix<double, 1, nu>  K_diag;
L_diag <<
       1.0, 1.0, 1.0,      // eul
        1.0, 200.0, 1.0,   // pCoM
        1e-7, 1e-7, 1e-7,  // w
        100.0, 10.0, 1.0;  // vCoM
K_diag <<
       1.0, 1.0, 1.0,      // fl
        1.0, 1.0, 1.0,
        1.0, 1.0, 1.0,     // fr
        1.0, 1.0, 1.0, 1.0;
MPC_solv.set_weight(1e-6, L_diag, K_diag);
```

`enable()` 内部只是：

$$
EN=true
$$

当：

$$
EN=true
$$

时，`MPC::cal()` 才执行真正的预测控制优化。

这里有一个时序细节：

```text
MPC 调用在 WBC 前面；
enable() 和 set_weight() 写在 WBC 后面的 else 分支中。
```

所以第一次超过 3 秒时，`enable()` 对后续 MPC 周期生效，存在一个“延后一拍”的效果。

### 10.1 L_diag

`L_diag` 是状态误差权重：

$$
L_{diag}
=
\begin{bmatrix}
1 & 1 & 1 &
1 & 200 & 1 &
10^{-7} & 10^{-7} & 10^{-7} &
100 & 10 & 1
\end{bmatrix}
$$

对应状态：

$$
X=
\begin{bmatrix}
rpy\\
p\\
\omega\\
v
\end{bmatrix}
$$

含义：

```text
rpy: 姿态跟踪权重为 1
pCoM: y 方向位置权重为 200，重视横向稳定
w: 角速度权重接近 0，几乎不惩罚角速度误差
vCoM: 前向速度权重 100，横向速度权重 10，竖直速度权重 1
```

### 10.2 K_diag

`K_diag` 是控制输入权重：

$$
K_{diag}\in\mathbb{R}^{13}
$$

前 12 维主要对应双脚 wrench：

$$
\begin{bmatrix}
f_L\\
\tau_L\\
f_R\\
\tau_R
\end{bmatrix}
\in
\mathbb{R}^{12}
$$

最后第 13 维会在后续 `MPC::cal()` 中继续阅读。

当前代码中 `K_diag` 全部设为 1，表示没有特别偏置某个输入通道。

### 10.3 set_weight()

调用：

```cpp
MPC_solv.set_weight(1e-6, L_diag, K_diag);
```

其中：

$$
\alpha=10^{-6}
$$

MPC 目标函数可以先粗略理解为：

$$
\min_U
(X-X_d)^T L (X-X_d)
+
\alpha U^T K U
$$

因为：

$$
nx=12,\quad mpc\_N=10
$$

所以：

$$
L\in\mathbb{R}^{120\times120}
$$

又因为：

$$
nu=13,\quad ch=3
$$

所以：

$$
K\in\mathbb{R}^{39\times39}
$$

## 11. 第 197-200 行：最终电机目标仍来自 WBC

源码：

```cpp
Eigen::VectorXd pos_des = kinDynSolver.integrateDIY(RobotState.q, RobotState.wbc_delta_q_final);
RobotState.motors_pos_des = eigen2std(pos_des.block(7, 0, model_nv - 6, 1));
RobotState.motors_vel_des = eigen2std(RobotState.wbc_dq_final);
RobotState.motors_tor_des = eigen2std(RobotState.wbc_tauJointRes);
```

这几行和 `walk_wbc.cpp` 基本一致。

WBC 输出：

$$
\Delta q_{\text{wbc}},\quad
\dot q_{\text{wbc}},\quad
\tau_{\text{wbc}}
$$

通过构型积分：

$$
q_{des}
=
q
\oplus
\Delta q_{\text{wbc}}
$$

只取关节部分：

$$
q_{j,des}
=
q_{des}[7:]
$$

最终：

$$
motors\_pos\_des=q_{j,des}
$$

$$
motors\_vel\_des=\dot q_{\text{wbc}}
$$

$$
motors\_tor\_des=\tau_{\text{wbc}}
$$

所以：

```text
MPC 不直接输出电机力矩。
MPC 改写 WBC 的上游输入；
WBC 输出最终关节位置、速度和力矩目标。
```

## 12. 第 203-219 行：PVT 与 MuJoCo 输出

源码：

```cpp
pvtCtr.dataBusRead(RobotState);
if (simTime <= 3) {
    pvtCtr.calMotorsPVT(...);
} else {
    pvtCtr.setJointPD(...);
    pvtCtr.calMotorsPVT();
}
pvtCtr.dataBusWrite(RobotState);
mj_interface.setMotorsTorque(RobotState.motors_tor_out);
```

这部分和 `walk_wbc.cpp` 作用一致。

PVT 读取：

$$
motors\_pos\_des,\quad
motors\_vel\_des,\quad
motors\_tor\_des
$$

以及当前关节状态，计算最终：

$$
motors\_tor\_out
$$

核心形式仍然是：

$$
\tau_{PD}
=
K_p(q_{des}-q)
+
K_d(\dot q_{des}-\dot q)
$$

再叠加 WBC 前馈：

$$
\tau_{link}
=
LPF(\tau_{PD})
+
\tau_{ff}
$$

最后输出到 MuJoCo：

$$
motors\_tor\_out
\rightarrow
mj\_data->ctrl
$$

这一层没有直接使用 MPC 的 `Ufe`。

## 13. 第 228-245 行：记录 MPC 中间量

源码：

```cpp
logger.recItermData("dX_cal", RobotState.dX_cal);
logger.recItermData("Ufe", RobotState.Fr_ff);
logger.recItermData("Xd", RobotState.Xd);
logger.recItermData("X_cur", RobotState.X_cur);
logger.recItermData("X_cal", RobotState.X_cal);
```

这些日志用于观察 MPC 行为。

其中：

$$
dX_{cal}
$$

是 MPC 计算出的状态变化率。

$$
RobotState.Fr_{ff}
$$

是 WBC 使用的接触 wrench 前馈，日志中命名为 `Ufe`。

$$
X_d
$$

是 MPC 期望状态。

$$
X_{cur}
$$

是 MPC 当前状态。

$$
X_{cal}
$$

是 MPC 预测/计算状态。

如果后续要调试 MPC，可以画：

$$
X_{cur},\quad X_d,\quad X_{cal}
$$

以及：

$$
F_{r,ff}
$$

来判断状态跟踪和接触力分配是否合理。

## 14. R1 总结

`walk_mpc_wbc.cpp` 相比 `walk_wbc.cpp` 的主文件差异可以归纳为：

```text
1. 引入 mpc.h
2. 新增 dt_200Hz = 0.005
3. 创建 MPC_solv(dt_200Hz)
4. 用 MPC_count 控制 MPC 每 5 个主循环运行一次
5. 在 WBC 前执行 MPC_solv.dataBusRead/cal/dataBusWrite
6. 3 秒后 enable MPC 并设置 L_diag / K_diag
7. 记录 X_cur / Xd / X_cal / dX_cal / Fr_ff 等 MPC 中间量
```

完整数据流：

$$
Joystick/Gait/FootPlacement
\rightarrow
MPC
\rightarrow
F_{r,ff},\ base\ des
\rightarrow
WBC
\rightarrow
wbc\_\tau
\rightarrow
PVT
\rightarrow
mj\_data->ctrl
$$

一句话总结：

```text
R1 读完可以确认：walk_mpc_wbc 的主文件没有改变 WBC/PVT 的最终输出链路，只是在 WBC 前增加了一个较低频的 MPC 上游参考生成层。
```

下一轮进入：

```cpp
MPC::dataBusRead(DataBus &Data)
```

重点拆：

$$
X_{cur}
=
\begin{bmatrix}
rpy\\
p\\
\omega\\
v
\end{bmatrix}
$$

以及：

$$
X_d
$$

如何从 `RobotState` 中生成。
