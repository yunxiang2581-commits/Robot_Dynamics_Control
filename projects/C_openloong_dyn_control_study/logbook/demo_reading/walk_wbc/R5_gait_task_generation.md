# walk_wbc 第五轮阅读笔记

## 1. 本轮目标

第五轮只回答一个问题：

```text
期望速度如何变成 motionState、legState、接触状态和摆动脚目标？
```

本轮关注的是 gait / task 生成层，而不是 WBC 求解层。

## 2. 本轮边界

本轮只读：

- `external/open_source_repos/OpenLoong-Dyn-Control/algorithm/joystick_interpreter.h`
- `external/open_source_repos/OpenLoong-Dyn-Control/algorithm/joystick_interpreter.cpp`
- `external/open_source_repos/OpenLoong-Dyn-Control/math/ramp_trajectory.h`
- `external/open_source_repos/OpenLoong-Dyn-Control/math/ramp_trajectory.cpp`
- `external/open_source_repos/OpenLoong-Dyn-Control/algorithm/gait_scheduler.h`
- `external/open_source_repos/OpenLoong-Dyn-Control/algorithm/gait_scheduler.cpp`
- 后续还要继续读 `foot_placement.cpp`

暂时不展开：

- `WBC_priority`
- `PVT_Ctr`

## 3. 本轮预期数据流

```text
desired base command
-> JoyStickInterpreter
-> GaitScheduler
-> FootPlacement
-> RobotState.motionState / legState / swing_fe_pos_des_W / swing_fe_rpy_des_W
```

这一层的作用是：

```text
把“我要怎么走”
翻译成 WBC 可以直接执行的任务目标。
```

更准确地说：

- `JoyStickInterpreter`：生成平滑的 base 期望运动
- `GaitScheduler`：推进步态相位，决定当前支撑腿 / 摆动腿
- `FootPlacement`：生成摆动脚目标落点与摆动轨迹

其中 `GaitScheduler` 虽然不求 WBC 和关节扭矩输出，但它会读取动力学量来估计接触力，用于换脚判定。

## 4. walk_wbc / walk_wbc_staircase 中的主链位置

主循环里当前这一层的关键调用是：

```cpp
if (simTime > startWalkingTime) {
    jsInterp.setWzDesLPara(0, 1);
    jsInterp.setVxDesLPara(xv_des, 2.0);
    RobotState.motionState = DataBus::Walk;
} else
    jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));

if (simTime >= startSteppingTime) {
    jsInterp.step();
    jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), stand_legLength + foot_height, RobotState.base_rpy(2));
    jsInterp.dataBusWrite(RobotState);

    gaitScheduler.start();
    RobotState.motionState = DataBus::Walk;
    gaitScheduler.dataBusRead(RobotState);
    gaitScheduler.step();
    gaitScheduler.dataBusWrite(RobotState);

    footPlacement.dataBusRead(RobotState);
    footPlacement.getSwingPos();
    footPlacement.dataBusWrite(RobotState);
}
```

这一段的主逻辑可以先压成：

```text
给速度目标
-> 速度目标平滑化与积分
-> 步态相位推进与换脚判断
-> 生成摆动脚目标
```

## 5. JoyStickInterpreter：把速度命令变成平滑的 base 参考

### 5.1 三个入口函数本身很薄

代码：

```cpp
void JoyStickInterpreter::setVxDesLPara(double vxDesLIn, double timeToReach) {
    vxLGen.setPara(vxDesLIn, timeToReach);
}

void JoyStickInterpreter::setVyDesLPara(double vyDesLIn, double timeToReach) {
    vyLGen.setPara(vyDesLIn, timeToReach);
}

void JoyStickInterpreter::setWzDesLPara(double wzDesLIn, double timeToReach) {
    wzLGen.setPara(wzDesLIn, timeToReach);
}
```

这三句本身不做积分，也不直接改 `RobotState`。

它们只是把：

- 目标前向速度 `vx`
- 目标侧向速度 `vy`
- 目标偏航角速度 `wz`

交给 3 个 `RampTrajectory` 发生器。

### 5.2 RampTrajectory 的作用

`RampTrajectory::setPara(yDesIn, timeToReach)` 的核心是：

```text
给定当前输出 yOld、目标 yDes、到达时间 T
-> 计算恒定斜率 k = (yDes - yOld) / T
```

后续每次 `step()`：

```text
y = yOld + k * dt
```

接近目标后直接收敛到 `yDes`。

所以它不是一步跳变，而是线性爬坡：

```text
目标速度
-> 平滑速度参考
```

### 5.3 JoyStickInterpreter::step() 的物理意义

`JoyStickInterpreter::step()` 的核心逻辑是：

1. 先从三个 ramp 中取出当前时刻的局部速度参考
2. 积分偏航角 `thetaZ`
3. 把局部速度 `(vx_L, vy_L)` 旋转到世界系 `(vx_W, vy_W)`
4. 再积分得到世界系位置 `(px_W, py_W)`

因此这一层不是在控制机器人，而是在构造一条“期望 base 运动轨迹”：

```text
局部速度参考
-> 世界系速度参考
-> 世界系位置参考
```

### 5.4 dataBusWrite() 写出的是什么

`JoyStickInterpreter::dataBusWrite()` 会把这些参考写入 `RobotState`：

- `js_pos_des`
- `js_vel_des`
- `js_eul_des`
- `js_omega_des`
- `base_pos_des`
- `base_rpy_des`
- `base_vel_des`
- `base_omega_des`

所以第五轮里，`JoyStickInterpreter` 的职责可以收成一句：

```text
把抽象的速度命令，变成平滑、连续、可积分的 base 期望轨迹。
```

## 6. GaitScheduler：步态相位推进与换脚判定

### 6.1 这个模块在干什么

`GaitScheduler` 的职责不是求扭矩，而是维护：

- 当前运动模式 `motionState`
- 当前支撑腿状态 `legState`
- 下一步目标状态 `legStateNext`
- 步态相位 `phi`
- 当前摆动脚起点 `swingStartPos_W`
- 当前支撑脚参考 `stanceStartPos_W`
- 摆动腿对应的髋位置 `posHip_W`
- 当前支撑脚位置 `posST_W`

也就是：

```text
“现在是哪条腿在支撑、哪条腿在摆、这一步走到第几成了”
```

### 6.2 dataBusRead() 读了哪些量

它会从 `RobotState` 中读取：

- 当前关节力矩 `motors_tor_cur`
- 动力学项 `dyn_M`, `dyn_Non`
- 左右脚 `J`, `dJ`
- `dq`
- 左右脚测力 `fL`, `fR`
- 左右脚、左右髋的世界系位置
- 当前 `motionState`

这说明：

```text
GaitScheduler 虽然属于 gait 层，
但它会读取动力学与 Jacobian 结果来辅助接触判定。
```

### 6.3 接触力反算代码

对应 [gait_scheduler.cpp](/home/ubuntu/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/algorithm/gait_scheduler.cpp:81)：

```cpp
Eigen::VectorXd tauAll;
tauAll = Eigen::VectorXd::Zero(model_nv);
tauAll.block(6, 0, model_nv - 6, 1) = torJoint;
FLest = -pseudoInv_SVD(J_l * dyn_M.inverse() * J_l.transpose()) * (J_l * dyn_M.inverse() * (tauAll - dyn_Non) + dJ_l * dq);
FRest = -pseudoInv_SVD(J_r * dyn_M.inverse() * J_r.transpose()) * (J_r * dyn_M.inverse() * (tauAll - dyn_Non) + dJ_r * dq);
```

这里算的不是传感器直接测得的力，而是：

```text
在“接触脚不动”的约束下，
根据当前动力学方程反算出的脚端接触力估计。
```

### 6.4 这个公式从哪来

先从 floating-base 动力学方程出发：

\[
M(q)\ddot q + h(q,\dot q) = \tau + J^\top F
\]

其中：

- \(M(q)\) 对应 `dyn_M`
- \(h(q,\dot q)\) 对应 `dyn_Non`
- \(\tau\) 对应 `tauAll`
- \(J\) 对应该脚 Jacobian
- \(F\) 对应该脚接触力

如果把脚看成稳定接触脚，那么脚端世界系加速度近似为 0：

\[
\ddot x_{foot} \approx 0
\]

而脚端速度关系是：

\[
\dot x_{foot} = J(q)\dot q
\]

对时间再求导：

\[
\ddot x_{foot} = \frac{d}{dt}(J\dot q) = J\ddot q + \dot J \dot q
\]

所以接触约束写成：

\[
J\ddot q + \dot J \dot q = 0
\]

把动力学里的

\[
\ddot q = M^{-1}(\tau + J^\top F - h)
\]

代进去，得到：

\[
J M^{-1}(\tau + J^\top F - h) + \dot J \dot q = 0
\]

整理：

\[
J M^{-1}J^\top F
=
-\left(J M^{-1}(\tau - h) + \dot J \dot q\right)
\]

于是：

\[
F
=
-\left(J M^{-1}J^\top\right)^+
\left(J M^{-1}(\tau - h) + \dot J \dot q\right)
\]

这就是代码里的：

```cpp
-pseudoInv_SVD(J * M^{-1} * J.transpose())
 * (J * M^{-1} * (tau - Non) + dJ * dq)
```

其中：

- `pseudoInv_SVD(...)` 是伪逆
- 前 6 维 `tau=0`，因为 floating-base 不是电机直接驱动

### 6.5 这个接触力估计拿来干什么

`GaitScheduler` 主要看：

- `FLest[2]`
- `FRest[2]`

也就是左右脚估计接触力的竖直分量。

它们被用来判断：

1. 摆动脚是否已经真正落地
2. 是否满足换脚条件
3. `Walk2Stand` 时是否已经能收口到双支撑

所以这一步的用途是：

```text
不是做精确力控制，
而是给步态状态机一个更物理的落脚判据。
```

### 6.6 `J ddq + dJ dq = 0` 这句的来源

这句来自 Jacobian 速度映射：

\[
x_{foot} = f(q)
\]

\[
\dot x_{foot} = J(q)\dot q
\]

再对时间求导：

\[
\ddot x_{foot} = \frac{d}{dt}(J\dot q) = J\ddot q + \dot J \dot q
\]

如果脚被视为稳定接触、相对地面不加速，那么：

\[
\ddot x_{foot} \approx 0
\]

于是得到：

\[
J\ddot q + \dot J \dot q \approx 0
\]

这不是恒等式，而是“接触脚不动”的约束假设。

### 6.7 `step()`：相位推进和第一次起步初始化

`step()` 中间这段先决定步态相位 `phi` 怎么走：

```cpp
double dPhi{0};

if (motionState == DataBus::Walk2Stand)
{
    enableNextStep = false;
    start_walk = false;
    if (touchDown)
        motionState = DataBus::Stand;
}

if (motionState == DataBus::Stand)
{
    dPhi = 0;
    phi = 0;
    isIni = false;
    enableNextStep = false;
    stepNumCur = 0;
}
else if (motionState == DataBus::Walk)
{
    enableNextStep = true;
    dPhi = 1.0 / tSwing * dt;
}
else if (motionState == DataBus::Walk2Stand)
    dPhi = 1.0 / tSwing * dt;

phi += dPhi;
```

这里的核心是：

```text
Stand      -> 不推进 phi，重置步态状态
Walk       -> 推进 phi，并允许连续迈下一步
Walk2Stand -> 当前这一步继续推进，但不再开启下一步
```

`phi` 是一步摆动周期的归一化相位：

```text
phi = 0   一步刚开始
phi = 0.5 摆动中段
phi = 1   这一脚应该结束/落地
```

如果 `tSwing = 0.4`，`dt = 0.001`，则：

```text
dPhi = dt / tSwing = 0.001 / 0.4 = 0.0025
```

大约 400 个控制周期后，`phi` 从 0 走到 1，刚好对应 0.4 秒摆动时间。

然后是第一次起步初始化：

```cpp
if (!isIni && start_walk)
{
    isIni = true;
    legState = firstleg;
    if (legState == DataBus::LSt)
    {
        swingStartPos_W = fe_r_pos_W;
        stanceStartPos_W = fe_l_pos_W;
    }
    else
    {
        swingStartPos_W = fe_l_pos_W;
        stanceStartPos_W = fe_r_pos_W;
    }
}
```

这里要注意 `legState` 的含义：

```text
LSt = left stance  = 左腿支撑，右腿摆动
RSt = right stance = 右腿支撑，左腿摆动
DSt = double stance / reserved = 双支撑或站立收尾
```

所以如果第一步是 `LSt`：

```text
左脚位置 -> stanceStartPos_W
右脚位置 -> swingStartPos_W
```

这一步不是生成摆动脚轨迹，而是给后面的 `FootPlacement` 准备“当前这一步从哪里开始摆”的起点。

### 6.8 `step()`：连续行走时的换脚判据

正常 `Walk` 时，`enableNextStep = true`，后面这段才允许真的切到下一步：

```cpp
if (legState == DataBus::LSt && FRest[2] >= 280 && phi >= 0.6)
{
    if (enableNextStep)
    {
        legState = DataBus::RSt;
        swingStartPos_W = fe_l_pos_W;
        stanceStartPos_W = fe_r_pos_W;
        phi = 0;
        stepNumCur++;
    }
}
else if (legState == DataBus::RSt && FLest[2] >= 280 && phi >= 0.6)
{
    if (enableNextStep)
    {
        legState = DataBus::LSt;
        swingStartPos_W = fe_r_pos_W;
        stanceStartPos_W = fe_l_pos_W;
        phi = 0;
        stepNumCur++;
    }
}
```

这段逻辑可以读成：

```text
当前左腿支撑时，看右脚估计接触力 FRest[2]
当前右腿支撑时，看左脚估计接触力 FLest[2]
```

换脚必须同时满足两个条件：

- 摆动脚竖直接触力足够大：`Fz >= 280`
- 摆动相位已经过了前半段：`phi >= 0.6`

`phi >= 0.6` 是一个防误触发条件。否则刚开始摆腿时，接触力估计的噪声可能会让状态机过早换脚。

换脚时会一起更新四件事：

- `legState`：新的支撑腿
- `swingStartPos_W`：下一步摆动脚的起点
- `stanceStartPos_W`：下一步支撑脚的起点
- `phi = 0`：新的一步从相位 0 开始

注释里保留了两个替代判据：

```cpp
// (FRest[2] >= 280 && phi >= 0.6) || (phi >= 0.99)
// phi >= 0.9
```

它们分别表示：

- 接触力判据失败时，用 `phi >= 0.99` 做保底换脚
- 完全按时间换脚，不看接触力

当前源码选择的是“接触力 + 最小相位”的事件触发方式，更贴近真实落脚。

### 6.9 `step()`：Walk2Stand 时的收口逻辑

当 `enableNextStep = false` 时，代码进入“当前这一步落地后就不再迈下一步”的逻辑：

```cpp
if (!enableNextStep)
{
    if (legState == DataBus::LSt && FRest[2] >= 200)
    {
        touchDown = true;
        stepNumCur++;
        legState = DataBus::DSt;
    }
    if (legState == DataBus::RSt && FLest[2] >= 200)
    {
        touchDown = true;
        stepNumCur++;
        legState = DataBus::DSt;
    }
}
```

这段主要服务于 `Walk2Stand`：

```text
当前这一步可以完成，
但完成后不要切换到下一条单腿支撑，
而是进入 DSt，准备回到 Stand。
```

这里接触力阈值是 `200`，比连续行走换脚的 `280` 低。可以先理解为：

```text
连续行走换脚要求落地更明确；
停止收口只需要确认摆动脚已经接触，可以进入双支撑。
```

前面 `Walk2Stand` 分支里还有：

```cpp
if (touchDown)
    motionState = DataBus::Stand;
```

所以停止链条是：

```text
Walk2Stand
-> enableNextStep = false
-> 摆动脚触地 touchDown = true
-> legState = DSt
-> 下一次 step() 把 motionState 切回 Stand
```

### 6.10 `step()`：给 FootPlacement 准备接口变量

最后这段把当前支撑腿状态转成 `FootPlacement` 需要的几何输入：

```cpp
if (legState == DataBus::LSt)
{
    posHip_W = hip_r_pos_W;
    posST_W = fe_l_pos_W;
    theta0 = -3.1415 * 0.5;
    legStateNext = DataBus::RSt;
    if (motionState == DataBus::Walk)
        legStateNext = DataBus::RSt;
    else if (motionState == DataBus::Walk2Stand)
        legStateNext = DataBus::DSt;
}
else if (legState == DataBus::RSt)
{
    posHip_W = hip_l_pos_W;
    posST_W = fe_r_pos_W;
    theta0 = 3.1415 * 0.5;
    legStateNext = DataBus::LSt;
    if (motionState == DataBus::Walk)
        legStateNext = DataBus::LSt;
    else if (motionState == DataBus::Walk2Stand)
        legStateNext = DataBus::DSt;
}
else
{
    posHip_W = hip_l_pos_W;
    posST_W = fe_r_pos_W;
    theta0 = 3.1415 * 0.5;
    legStateNext = DataBus::DSt;
}
```

左支撑时：

```text
左脚是支撑脚 -> posST_W = fe_l_pos_W
右腿是摆动腿 -> posHip_W = hip_r_pos_W
theta0 = -pi/2
```

右支撑时：

```text
右脚是支撑脚 -> posST_W = fe_r_pos_W
左腿是摆动腿 -> posHip_W = hip_l_pos_W
theta0 = +pi/2
```

这些量后面会被 `FootPlacement::dataBusRead()` 读走，用来计算摆动脚最终落点。

`theta0` 的作用是告诉落点规划：

```text
当前摆动腿在身体 yaw 方向的左侧还是右侧。
```

`legStateNext` 则是下一步状态提示：

```text
Walk      -> 下一步继续交替单腿支撑
Walk2Stand -> 下一步进入 DSt，不再继续迈步
```

### 6.11 `start()`：只打开起步标志

`GaitScheduler::start()` 很短：

```cpp
void GaitScheduler::start()
{
    start_walk = true;
}
```

它不直接修改 `phi`、`legState` 或脚位置，只是打开标志位：

```text
start()
-> start_walk = true
-> 下一次 step() 里满足 !isIni && start_walk
-> 初始化第一步支撑脚和摆动脚起点
```

所以 `start()` 是“允许起步”的开关，真正的初始化仍然发生在 `step()` 里。

### 6.12 本节结论

`GaitScheduler` 可以收成一句：

```text
根据当前动力学状态和脚接触估计，
维护“哪只脚支撑 / 哪只脚摆动 / 当前一步走到哪里了”。
```

更细一点说：

```text
motionState
-> enableNextStep / dPhi / phi
-> 接触力估计 FLest / FRest
-> legState 切换或 DSt 收口
-> swingStartPos_W / stanceStartPos_W / posHip_W / posST_W / theta0
```

这就是 `GaitScheduler` 到 `FootPlacement` 的接口。

## 7. 阶段检查：GaitScheduler 已读完

- `JoyStickInterpreter`
- `RampTrajectory`
- `GaitScheduler`
  - `dataBusRead()`：读取模型量、脚位置、当前 motionState
  - `step()`：估计接触力、推进相位、换脚、Walk2Stand 收口
  - `dataBusWrite()`：写回 `legState / phi / swingStartPos_W / posHip_W`
  - `start()`：打开第一次起步初始化标志

读完 `GaitScheduler` 后，第五轮还剩最后一段：

- `FootPlacement::dataBusRead()`
- `FootPlacement::getSwingPos()`
- `FootPlacement::dataBusWrite()`

到那一步，才能把：

```text
legState / phi / posHip_W / posST_W
-> swing_fe_pos_des_W / swing_fe_rpy_des_W
```

这条链完整收口。下面第 8 节就是这部分。

## 8. FootPlacement：把步态状态变成摆动脚目标

### 8.1 这个模块在主链里的位置

`FootPlacement` 接在 `GaitScheduler` 后面。

前一层 `GaitScheduler` 已经回答了：

```text
现在是哪条腿支撑？
当前相位 phi 到哪里了？
摆动脚从哪里开始？
支撑脚在哪里？
摆动腿对应的 hip 在哪里？
```

`FootPlacement` 要继续回答：

```text
这一拍摆动脚应该走到哪里？
这一整步最终应该落在哪里？
```

所以它不是在求 IK，也不是在算关节力矩。它生成的是 WBC 后面要跟踪的任务目标：

- `swing_fe_pos_des_W`
- `swing_fe_rpy_des_W`
- `swingDesPosCur_W`
- `swingDesPosFinal_W`

### 8.2 dataBusRead() 读了哪些输入

对应 [foot_placement.cpp](/home/ubuntu/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/algorithm/foot_placement.cpp:11)：

```cpp
posStart_W = robotState.swingStartPos_W;
desV_W = robotState.js_vel_des;
desWz_W = robotState.js_omega_des(2);
curV_W = robotState.dq.block<3, 1>(0, 0);
phi = robotState.phi;
hipPos_W = robotState.posHip_W;
STPos_W = robotState.posST_W;
base_pos = robotState.base_pos;
tSwing = robotState.tSwing;
theta0 = robotState.theta0;
yawCur = robotState.rpy[2];
omegaZ_W = robotState.base_omega_W(2);
hip_width = robotState.width_hips;
legState = robotState.legState;
```

这些输入可以分成四组：

1. 步态状态：

- `posStart_W`：当前摆动脚轨迹起点
- `phi`：当前摆动相位，通常在 `[0, 1]`
- `tSwing`：摆动周期
- `legState`：当前支撑腿状态

2. 高层速度目标：

- `desV_W`：期望 base 线速度，来自 `JoyStickInterpreter`
- `desWz_W`：期望 yaw 角速度，来自 `JoyStickInterpreter`

3. 当前真实运动状态：

- `curV_W`：当前 base 线速度
- `yawCur`：当前 yaw
- `omegaZ_W`：当前 yaw 角速度
- `base_pos`：当前 base 位置

4. 几何参考：

- `hipPos_W`：摆动腿 hip 的世界系位置
- `STPos_W`：支撑脚世界系位置
- `theta0`：摆动腿相对机身 yaw 的侧向偏置
- `hip_width`：髋宽

本节的学习重点是：

```text
FootPlacement 的输入不是 q / dq 全部模型量，
而是 gait 层整理过的“步态变量 + 当前速度 + 期望速度 + 几何参考”。
```

### 8.3 getSwingPos() 第一段：速度误差影响最终落点

对应 [foot_placement.cpp](/home/ubuntu/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/algorithm/foot_placement.cpp:43)：

```cpp
KP.setZero();
KP(0, 0) = kp_vx;
KP(1, 1) = kp_vy;
KP(2, 2) = 0;
Rz << cos(yawCur), -sin(yawCur), 0,
    sin(yawCur), cos(yawCur), 0,
    0, 0, 1;
KP = Rz * KP * Rz.transpose();
```

这里先在机身 yaw 方向下定义速度反馈增益，再旋转到世界系。

原因是：

```text
kp_vx / kp_vy 更像“机器人前向 / 侧向”的调参量，
但 posDes_W 是世界系落点，
所以需要用当前 yaw 把增益方向转到世界系。
```

然后计算最终落点：

```cpp
posDes_W = hipPos_W + KP * (desV_W - curV_W) * (-1) + 0.5 * tSwing * curV_W +
           curV_W * (1 - phi) * tSwing;
```

这句可以拆成四部分：

- `hipPos_W`：先把摆动腿 hip 当作落点基准
- `KP * (desV_W - curV_W) * (-1)`：速度误差修正项
- `0.5 * tSwing * curV_W`：用当前速度预测半步位移
- `curV_W * (1 - phi) * tSwing`：考虑当前相位下剩余摆动时间

这不是严格的动力学最优解，而是一个启发式落脚点公式：

```text
当前速度、期望速度和剩余摆动时间
共同决定脚最终应该落在 hip 前后/左右的哪个位置。
```

注意这里代码使用的是 `-(desV_W - curV_W)`，也就是 `curV_W - desV_W` 方向的反馈。阅读时先按“源码采用的调参约定”理解，不要直接把它当成唯一标准公式。

### 8.4 getSwingPos() 第二段：yaw 转向时修正左右脚落点

对应 [foot_placement.cpp](/home/ubuntu/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/algorithm/foot_placement.cpp:57)：

```cpp
thetaF = yawCur + theta0 + omegaZ_W * (1 - phi) * tSwing + 0.5 * omegaZ_W * tSwing + kp_wz * (omegaZ_W - desWz_W);
posDes_W(0) += 0.5 * hip_width * (cos(thetaF) - cos(yawCur + theta0));
posDes_W(1) += 0.5 * hip_width * (sin(thetaF) - sin(yawCur + theta0));
```

这段处理的是转弯时的脚步位置。

直观解释：

```text
如果机器人正在转 yaw，
左右脚相对身体中心的理想位置也应该跟着旋转，
不能只按直线行走的 x/y 落点来放脚。
```

其中：

- `yawCur`：当前身体 yaw
- `theta0`：左/右腿侧向偏置，左支撑时取 `-pi/2`，右支撑时取 `+pi/2`
- `omegaZ_W * (1 - phi) * tSwing`：剩余摆动时间内的 yaw 预测
- `0.5 * omegaZ_W * tSwing`：半步 yaw 预测
- `kp_wz * (omegaZ_W - desWz_W)`：yaw 角速度误差修正
- `0.5 * hip_width`：用半髋宽把“身体中心旋转”转换成“脚侧向位置变化”

一句话：

```text
这段是在让摆动脚落点跟上身体未来的转向趋势。
```

### 8.5 getSwingPos() 第三段：脚端固定偏置与高度

对应 [foot_placement.cpp](/home/ubuntu/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/algorithm/foot_placement.cpp:63)：

```cpp
double xOff_L = -0.07;
double yOff_L = 0.04;
double zOff_W = -0.035;
posDes_W(2) = base_pos(2) - legLength + zOff_W;
```

这几个量是工程调参项：

- `xOff_L`：脚端在机身 x 方向的固定偏置
- `yOff_L`：脚端在机身 y 方向的固定偏置
- `zOff_W`：脚端高度修正
- `legLength`：期望 base 到脚端的高度距离

`z` 方向的落点不是从地形模型显式查出来的，而是：

```text
posDes_W.z = 当前 base 高度 - 期望腿长 + 脚端高度偏置
```

在 `walk_wbc_staircase.cpp` 里还有额外的 staircase 逻辑：

```cpp
RobotState.base_pos_des(2) = stand_legLength + foot_height + (RobotState.basePos[0] - 0.025)*0.1;
```

也就是说 staircase demo 主要通过抬高 `base_pos_des(2)` 来适应台阶，而不是在 `FootPlacement` 里显式读取台阶高度图。

### 8.6 getSwingPos() 第四段：用 phi 生成当前摆动轨迹点

前面算出的 `posDes_W` 是这一整步的最终落点。

但 WBC 每个控制周期需要的是“当前这一拍”摆动脚应该在哪里，所以还要根据 `phi` 生成插值轨迹。

x/y 方向使用摆线轨迹：

```cpp
pDesCur[0] = posStart_W(0) + (posDes_W(0) - posStart_W(0)) / (2 * pi) * (2 * pi * phi - sin(2 * pi * phi));
pDesCur[1] = posStart_W(1) + (posDes_W(1) - posStart_W(1)) / (2 * pi) * (2 * pi * phi - sin(2 * pi * phi));
```

这个形式的好处是：

```text
phi = 0 和 phi = 1 附近速度比较平滑，
摆动脚起步和落脚不会突然跳变。
```

z 方向使用 `Trajectory(...)`：

```cpp
pDesCur[2] = posStart_W(2) + Trajectory(0.2, stepHeight, posDes_W(2) - posStart_W(2)) + zStretch;
```

它的含义是：

- 前段把脚抬到 `stepHeight`
- 后段逐渐落向最终高度差 `posDes_W(2) - posStart_W(2)`
- `zStretch` 在 `phi >= 0.98` 时逐步向下压，最多到 `-0.05`

这里 `zStretch` 的用途可以理解为：

```text
临近落脚时稍微向下探一点，
帮助摆动脚更容易接触地面/台阶。
```

### 8.7 dataBusWrite() 写给谁

对应 [foot_placement.cpp](/home/ubuntu/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/algorithm/foot_placement.cpp:29)：

```cpp
robotState.swingDesPosCur_W << pDesCur[0], pDesCur[1], pDesCur[2];
robotState.swingDesPosFinal_W = posDes_W;
robotState.swing_fe_rpy_des_W << 0, 0, robotState.base_rpy_des(2);
robotState.swing_fe_pos_des_W << pDesCur[0], pDesCur[1], pDesCur[2];
```

这里写回的是 WBC 任务目标：

- `swingDesPosCur_W`：当前摆动脚轨迹点，便于记录/调试
- `swingDesPosFinal_W`：本步最终落点，便于记录/调试
- `swing_fe_pos_des_W`：WBC 真正读取的摆动脚位置目标
- `swing_fe_rpy_des_W`：WBC 真正读取的摆动脚姿态目标

姿态目标目前很简单：

```text
roll = 0
pitch = 0
yaw = base_rpy_des.z
```

也就是摆动脚 yaw 跟随期望 base yaw。

### 8.8 FootPlacement 本节结论

`FootPlacement` 可以收成一句：

```text
根据当前支撑腿、步态相位、base 当前速度、base 期望速度和身体几何关系，
生成摆动脚当前目标点与最终落点。
```

完整链条现在闭合为：

```text
JoyStickInterpreter
  速度命令 -> 平滑 base 参考

GaitScheduler
  base/脚/动力学状态 -> legState, phi, swingStartPos_W, posHip_W

FootPlacement
  legState, phi, 速度目标, 几何参考 -> swing_fe_pos_des_W

WBC_priority
  base 目标 + 摆动脚目标 + 支撑脚约束 -> ddq / Fr / tau
```

## 9. R5 和 staircase demo 的关系

`walk_wbc.cpp` 和 `walk_wbc_staircase.cpp` 的 R5 主链基本一致：

```text
jsInterp.step()
jsInterp.dataBusWrite(RobotState)
gaitScheduler.dataBusRead/step/dataBusWrite
footPlacement.dataBusRead/getSwingPos/dataBusWrite
```

但 staircase demo 有几个重要参数变化：

- `xv_des = 0.7`：前进速度更快
- `footPlacement.stepHeight = 0.4`：摆脚高度明显更高，用于跨台阶
- `RobotState.base_pos_des(2)` 会随 `basePos[0]` 上升：让身体高度跟着前进位置增加

因此 staircase demo 的核心思路不是“重新发明步态调度器”，而是：

```text
沿用原来的 gait / foot-placement / WBC 链条，
通过更高摆脚高度和随 x 上升的 base 高度目标来适应台阶场景。
```

## 10. R5 总结

第五轮现在可以正式收口：

```text
期望速度如何变成 motionState、legState、接触状态和摆动脚目标？
```

答案是：

1. `JoyStickInterpreter`

```text
vx/vy/wz 目标
-> RampTrajectory 平滑
-> yaw 积分
-> 世界系 base 位置/速度/姿态参考
```

2. `GaitScheduler`

```text
motionState + 脚位置 + Jacobian/dJ/动力学项 + 当前关节力矩
-> 接触力估计
-> phi 推进
-> legState / legStateNext / swingStartPos_W / posHip_W
```

3. `FootPlacement`

```text
legState + phi + js_vel_des + dq + yaw + hip 几何
-> 最终落点 posDes_W
-> 当前摆动轨迹点 pDesCur
-> swing_fe_pos_des_W / swing_fe_rpy_des_W
```

这一层的输出会在下一轮进入 `WBC_priority`：

```text
WBC 不再关心“怎么生成步态目标”，
只关心“如何满足这些 base / foot / contact 任务”。
```
