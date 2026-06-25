# walk_wbc 总体数据流总览

## 1. 文件目的

这个文件不是替代 `R1 ~ R9` 的单轮阅读笔记，而是做一张跨轮次总图：

```text
MuJoCo
-> MJ_Interface / DataBus
-> StateEst
-> Pin_KinDyn
-> gait / task
-> WBC
-> PVT
-> MuJoCo
```

阅读 `walk_wbc` 时，每一轮只深挖一段；这个文件负责把所有段重新连起来，并用最关键的数学关系说明每段数据“为什么有用”。

## 2. 当前阅读进度

当前按轮次的完成情况：

- `R1`：已完成，主流程地图
- `R2`：已完成，`MuJoCo -> MJ_Interface -> DataBus::updateQ()`
- `R3`：已完成，`StateEst`
- `R4`：已完成，`Pin_KinDyn`
- `R5`：已完成，`JoyStickInterpreter / GaitScheduler / FootPlacement`
- `R6`：已完成，`WBC_priority::computeDdq() / PriorityTasks::computeAll()`
- `R7`：已完成，`WBC_priority::computeTau()`
- `R8`：已完成，`PVT_Ctr -> motors_tor_out -> mj_data->ctrl`
- `R9`：已完成，`walk_wbc.cpp` 完整闭环复盘
- `R10`：已完成，面试求职项目简述与详细项目解释
- `R11`：已完成，`walk_wbc` 完成后的复现推进结论与下一步计划

## 3. 一张总图

### 3.1 控制闭环总链

```text
上一轮电机力矩 tau_out(k)
-> 写入 mj_data->ctrl
-> mj_step(mj_model, mj_data)
-> MuJoCo 当前状态 x_mj(k+1)
-> MJ_Interface 读状态
-> DataBus.updateQ() 组装原始 q/dq
-> StateEst 修正 floating-base base 状态
-> Pin_KinDyn 计算 J / dJ / pose / dynamics
-> gait / task 生成 walking 任务目标
-> WBC computeDdq() 生成期望 delta_q / dq / ddq
-> WBC computeTau() 求接触力与关节力矩
-> PVT_Ctr 生成最终 motors_tor_out
-> 下一轮 mj_step()
```

### 3.2 最关键的两个“状态”

这个 demo 里最重要的不是 MuJoCo 的原始 `qpos/qvel`，而是控制链内部使用的 floating-base 状态：

```text
q  = [base position(3), base quaternion(4), joint positions]
dq = [base linear velocity(3), base angular velocity(3), joint velocities]
```

后续 `StateEst`、`Pin_KinDyn`、`WBC` 都围绕这两个量工作。

## 4. R1：主流程总览

### 4.1 这一轮回答的问题

```text
walk_wbc 每个仿真步怎样从 MuJoCo 状态走到 WBC/PVT 力矩，再写回 MuJoCo？
```

### 4.2 这一轮的核心收获

- `walk_wbc.cpp` 是闭环装配入口，不是单个算法实现文件。
- 主循环中的关键次序是：

```text
mj_step
-> updateSensorValues
-> StateEst
-> Pin_KinDyn
-> gait/task
-> WBC
-> PVT
-> setMotorsTorque
```

- 整个 demo 是标准离散闭环：

```text
u_k
-> simulator
-> x_{k+1}
-> controller
-> u_{k+1}
```

## 5. R2：MuJoCo -> MJ_Interface -> DataBus::updateQ()

### 5.1 数据流

```text
mj_data->qpos
-> motor_pos[i]
-> RobotState.motors_pos_cur[i]
-> q[7 + i]

mj_data->qvel
-> motor_vel[i]
-> RobotState.motors_vel_cur[i]
-> dq[6 + i]

mj_data->sensordata["baselink-quat"]
-> baseQuat
-> rpy
-> quaternion(rpy)
-> q[3:6]

mj_data->sensordata["baselink-gyro"]
-> baseAngVel
-> R(rpy) * baseAngVel
-> dq[3:5]
```

### 5.2 这一段的关键公式

原始 `updateQ()` 里，广义状态先被组装成：

```text
q(0:2)   = basePos
q(3:6)   = quat(rpy)
q(7:)    = motors_pos_cur

dq(0:2)  = baseLinVel
dq(3:5)  = base_omega_W
dq(6:)   = motors_vel_cur
```

### 5.3 这一轮的关键注意事项

`MJ_Interface` 虽然读出了：

```text
basePos
baseLinVel
```

但它没有直接把这两个量写进 `RobotState`，所以 `q[0:2]` 和 `dq[0:2]` 还不是最终可信的 base 平移状态。

## 6. R3：StateEst 修正 floating-base base 状态

### 6.1 这一轮回答的问题

```text
为什么不直接相信 MuJoCo 读出来的 basePos/baseLinVel？
StateEst 最后到底覆盖了 q/dq 的哪些维度？
```

### 6.2 状态估计器核心状态

`StateEst` 内部维护 15 维状态：

```text
X[0:2]    = base_pos
X[3:5]    = base_vel
X[6:8]    = left foot world pos
X[9:11]   = right foot world pos
X[12:14]  = delta_acc
```

### 6.3 这一段的关键公式

预测：

```text
X_{k+1|k} = A X_k + B freeAcc
P_{k+1|k} = A P_k A^T + Q
```

修正：

```text
K = P C^T (C P C^T + R)^{-1}
X = X + K (Y - C X)
```

### 6.4 对 `q/dq` 的覆盖

`StateEst::get()` 最重要的作用是重新写回：

```text
q[0:2]  = estimated base_pos
q[3:6]  = estimated base quaternion
dq[0:2] = estimated base_vel
dq[3:5] = estimated base_omega_W
```

所以第三轮的结论是：

```text
关节部分来自 MuJoCo；
floating-base 的 base 部分来自 StateEst 修正后的估计结果。
```

## 7. R4：Pin_KinDyn 把 q/dq 变成模型量

### 7.1 这一轮回答的问题

```text
StateEst 修正后的 q/dq，如何变成 FK、Jacobian、dJ、质量矩阵、重力项和非线性项？
```

### 7.2 两套 Pinocchio 模型

`Pin_KinDyn` 同时维护：

- `floating-base model`
  作用：用于全身 FK、Jacobian、动力学、WBC。
- `fixed-base model`
  作用：用于 body/baselink 相对量，如脚相对 base 的位置和速度。

### 7.3 这一段的关键公式

运动学：

```text
forwardKinematics(q)
J = J(q)
dJ = dJ(q, dq)
```

动力学：

```text
M(q) ddq + C(q,dq) dq + G(q) = generalized_forces
dyn_Non = C(q,dq) dq + G(q)
```

质心动量：

```text
h = Ag(q) dq
h_dot = Ag(q) ddq + dAg(q,dq) dq
```

### 7.4 这一轮输出了什么

写回 `RobotState` 的主要模型量：

- `J_l / J_r / J_base / J_hd_l / J_hd_r / J_hip_link`
- `dJ_l / dJ_r / dJ_base / dJ_hd_l / dJ_hd_r`
- `fe_*_pos_W / fe_*_rot_W`
- `fe_*_pos_L / fe_*_vel_L`
- `dyn_M / dyn_M_inv / dyn_C / dyn_G / dyn_Non`
- `dyn_Ag / dyn_dAg`
- `pCoM_W / Jcom_W`

### 7.5 当前实际使用状态

当前源码中：

- `dyn_M / dyn_M_inv / dyn_Non` 已被 `WBC_priority` 真实使用。
- `J_* / dJ_* / feet/base/CoM` 已被 `WBC_priority` 和后续 gait/task 真实使用。
- `dyn_Ag / dyn_dAg` 已被读入 `WBC_priority`，但当前版本尚未真正参与后续求解表达式。

## 8. R5：gait / task 生成层

### 8.1 本轮回答的问题

```text
期望速度如何变成 motionState、legState、接触状态和摆动脚目标？
```

### 8.2 实际数据流

```text
walk_wbc.cpp 中的 xv_des / wz_des
-> JoyStickInterpreter
-> RampTrajectory
-> GaitScheduler
-> FootPlacement
-> RobotState.base_*_des / legState / phi / swing_fe_pos_des_W / swing_fe_rpy_des_W
```

### 8.3 这一层的作用

这一层不解 WBC，不直接输出关节力矩。它负责把“我要怎么走”翻译成 WBC 可执行的 walking task。

更具体地说：

- `JoyStickInterpreter`：
  - 输入：`walk_wbc.cpp` 中设置的 `xv_des / wz_des`
  - 作用：用 `RampTrajectory` 平滑速度命令，积分 yaw 和世界系 base 位置
  - 输出：`js_vel_des / js_omega_des / base_pos_des / base_rpy_des / base_vel_des / base_omega_des`

- `GaitScheduler`：
  - 输入：`motionState`、脚/髋世界系位置、`J_l / J_r / dJ_l / dJ_r`、`dyn_M / dyn_Non`、当前关节力矩
  - 作用：估计左右脚接触力，推进 `phi`，判断换脚和 `Walk2Stand` 收口
  - 输出：`legState / legStateNext / phi / swingStartPos_W / posHip_W / posST_W / theta0`

- `FootPlacement`：
  - 输入：`GaitScheduler` 的步态变量、`JoyStickInterpreter` 的期望速度、当前 base 状态
  - 作用：先算本步最终落点 `posDes_W`，再按当前 `phi` 生成当前轨迹点 `pDesCur`
  - 输出：`swing_fe_pos_des_W = pDesCur`、`swing_fe_rpy_des_W`、`swingDesPosFinal_W = posDes_W`

### 8.4 本轮核心区别

`FootPlacement` 里有两个容易混淆的量：

```text
swingDesPosFinal_W = posDes_W
```

表示这一整步最终落点。

```text
swing_fe_pos_des_W = pDesCur
```

表示当前 `phi` 对应的摆动脚世界系目标点，也是 WBC 每个控制周期真正跟踪的目标。

所以 R5 的最终结论是：

```text
JoyStickInterpreter 负责“想往哪走”；
GaitScheduler 负责“哪只脚支撑、当前走到哪一步”；
FootPlacement 负责“当前相位下摆动脚目标在哪里”。
```

### 8.5 关键字段流

R5 里最重要的一条链路是“期望速度如何一路传到摆动脚任务”：

```text
walk_wbc.cpp
  xv_des / wz_des
      |
      v
JoyStickInterpreter::setVxDesLPara()
JoyStickInterpreter::setWzDesLPara()
      |
      v
RampTrajectory
  把速度命令从阶跃变成平滑变化
      |
      v
RobotState.js_vel_des
RobotState.js_omega_des
RobotState.base_pos_des
RobotState.base_rpy_des
      |
      v
FootPlacement::dataBusRead()
  desV_W / desWz_W
      |
      v
FootPlacement::getSwingPos()
  swing_fe_pos_des_W
```

这条线说明：`FootPlacement` 里的期望速度不是凭空来的，而是 `walk_wbc.cpp` 给出的高层速度命令，经过 `JoyStickInterpreter` 和 `RampTrajectory` 平滑后写入 `DataBus`。

### 8.6 `legState` 和 `phi`

`GaitScheduler` 给后续所有 walking task 提供两个核心离散/连续状态：

```text
legState:
  LSt = 左脚支撑，右脚摆动
  RSt = 右脚支撑，左脚摆动
  DSt = 双支撑或站立收口

phi:
  当前摆动相位，约从 0 推进到 1
```

它们后面会直接影响 WBC 里哪条腿作为接触约束，哪条腿作为摆动脚任务：

```text
LSt:
  Jc  = J_l
  Jsw = J_r

RSt:
  Jc  = J_r
  Jsw = J_l
```

所以 R5 不只是“生成脚步轨迹”，它还决定了 R6 里任务和约束的身份分配。

### 8.7 `FootPlacement` 的压缩公式复盘

`FootPlacement::getSwingPos()` 可以分成两步理解。

第一步：根据速度误差、当前速度、yaw 转向、脚底偏置，算本步最终落点：

```text
posDes_W
  = 髋部参考点
  + 速度反馈修正
  + 当前速度前瞻项
  + yaw 角速度修正
  + 足端固定偏置
  + 目标落脚高度
```

其中速度反馈的核心结构是：

```text
速度反馈修正 = - KP * (desV_W - curV_W)

KP = Rz(yawCur) * diag(kp_vx, kp_vy, 0) * Rz(yawCur)^T
```

第二步：根据当前 `phi`，把起点 `posStart_W` 平滑插值到最终落点 `posDes_W`：

```text
x / y:
  使用 cycloid 插值

z:
  使用 Trajectory(phase, stepHeight, final_height_delta)
  末端再叠加 zStretch 做触地前向下探测
```

最终写入 `DataBus` 的不是完整轨迹数组，而是当前控制周期的一个目标点：

```text
swing_fe_pos_des_W = pDesCur
```

连续控制循环里 `phi` 不断变化，`pDesCur` 连续更新，合起来才形成整段摆动脚轨迹。

### 8.8 R5 到 R6 的接口

R5 结束后，R6 的 WBC 主要读取这些任务输入：

```text
base_pos_des
base_rpy_des
base_vel_des
base_omega_des

legState
legStateNext
motionState
phi

stance_fe_pos_cur_W
stance_fe_rot_cur_W
stanceDesPos_W

swing_fe_pos_des_W
swing_fe_rpy_des_W
swingDesPosFinal_W
```

因此进入 R6 时，第一件事不是直接看求解公式，而是先看：

```text
WBC_priority::dataBusRead()
```

因为这里会把 R5 生成的 walking task 和 R4 生成的动力学/Jacobian 数据接起来，然后才进入 `computeDdq()`。

## 9. R6：WBC computeDdq()

### 9.1 本轮将回答的问题

$$
\text{walking / standing 任务如何变成期望 } \Delta q,\ \dot q,\ \ddot q\text{？}
$$

详细笔记：

```text
projects/C_openloong_dyn_control_study/logbook/demo_reading/walk_wbc/R6_wbc_compute_ddq_priority_tasks.md
```

### 9.2 本轮实际读完的内容

R6 读的是 WBC 的运动学任务求解层，不直接求关节力矩。

```text
WBC_priority::dataBusRead()
-> WBC_priority::computeDdq()
-> PriorityTasks::computeAll()
-> delta_q_final_kin / dq_final_kin / ddq_final_kin
```

其中：

```text
dataBusRead()
  把 R4 的 J/dJ/M/dyn_Non/q/dq 和 R5 的 base/swing/legState 读进 WBC

computeDdq()
  给 walk / stand 两套任务表填写 errX / derrX / J / dJ / kp / kd

PriorityTasks::computeAll()
  按人为指定的 taskOrder 逐层零空间求解
```

### 9.3 Walk 任务链

walking 模式实际启用的优先级链是：

$$
\text{static\_Contact}
\rightarrow
\text{PosRot}
\rightarrow
\text{SwingLeg}
\rightarrow
\text{RedundantJoints}
\rightarrow
\text{HandTrackJoints}
$$

含义：

- `static_Contact`：支撑脚静止。
- `PosRot`：控制 base 位置和姿态。
- `SwingLeg`：跟踪 R5 `FootPlacement` 生成的摆动脚目标。
- `RedundantJoints`：冗余关节回零。
- `HandTrackJoints`：手臂关节跟踪摆臂姿态。

核心约束：

$$
J_c\ddot q+\dot J_c\dot q=0
$$

$$
J_{base}\ddot q
=
\ddot x_{base,cmd}-\dot J_{base}\dot q
$$

$$
J_{sw}\ddot q
=
\ddot x_{sw,cmd}-\dot J_{sw}\dot q
$$

其中 `SwingLeg` 是 R5 到 R6 的关键接口：

$$
\text{swing\_fe\_pos\_des\_W}
\rightarrow
\text{SwingLeg.errX}
\rightarrow
J_{sw}\ddot q
$$

### 9.4 Stand 任务链

standing 模式实际启用的优先级链是：

$$
\text{static\_Contact}
\rightarrow
\text{CoMXY\_HipRPY}
\rightarrow
\text{Pz}
\rightarrow
\text{HandTrackJoints}
\rightarrow
\text{HeadRP}
$$

含义：

- `static_Contact`：双脚接触静止。
- `CoMXY_HipRPY`：控制质心 xy 和髋部姿态。
- `Pz`：控制 base 高度。
- `HandTrackJoints`：手臂固定姿态。
- `HeadRP`：头部 roll / pitch 姿态。

stand 与 walk 的区别：

$$
\text{Walk: 单脚支撑 + 摆动脚轨迹}
$$

$$
\text{Stand: 双脚支撑 + CoM 水平平衡}
$$

### 9.5 统一数学关系

每个任务先构造任务加速度命令：

$$
\ddot x_{cmd}
=
\ddot x_{des}
+K_p e
+K_d\dot e
$$

任务空间加速度关系是：

$$
\ddot x
=
J\ddot q+\dot J\dot q
$$

因此每个任务最终都转成：

$$
J\ddot q
=
\ddot x_{cmd}-\dot J\dot q
$$

### 9.6 逐层优先级求解

`buildPriority()` 人为指定任务顺序，`computeAll()` 从高到低沿 `childId` 求解。

第一层没有更高优先级任务：

$$
N_0=I
$$

后续任务只能在上一层任务的零空间里修正：

$$
N_i
=
N_{i-1}
\left(
I-(J_{i-1}^{pre})^\#J_{i-1}^{pre}
\right)
$$

当前任务在剩余自由度中的有效 Jacobian：

$$
J_i^{pre}=J_iN_i
$$

加速度递推：

$$
\ddot q_i
=
\ddot q_{i-1}
+(J_i^{pre})_{\text{dyn}}^\#
\left(
\ddot x_{cmd,i}
-\dot J_i\dot q
-J_i\ddot q_{i-1}
\right)
$$

最终输出：

$$
\Delta q_{\text{final,kin}},
\quad
\dot q_{\text{final,kin}},
\quad
\ddot q_{\text{final,kin}}
$$

R6 的结论：

$$
\text{R6 完成“任务目标 } \rightarrow \text{ 加速度层 WBC 解”的链路。}
$$

## 10. R7：WBC computeTau()

### 10.1 本轮将回答的问题

```text
已知期望 ddq，如何求接触力和关节力矩？
```

详细笔记：

```text
projects/C_openloong_dyn_control_study/logbook/demo_reading/walk_wbc/R7_wbc_compute_tau_qp.md
```

### 10.2 本轮实际读完的内容

R7 读的是 WBC 的动力学一致性和接触约束层：

```text
ddq_final_kin / Fr_ff
-> floating-base 动力学等式
-> 接触 wrench 不等式
-> qpOASES 求 delta_ddq_b / delta_F
-> tauJointRes
```

当前版本最关键的动力学关系是：

$$
M\ddot q + h = J_{fe}^T F + S^T\tau
$$

在代码变量里：

$$
h = \text{dyn\_Non}
$$

这里：

- `M = dyn_M`
- `Non = dyn_Non`
- `Jfe = [J_l; J_r]`
- `F` 是双脚 12 维接触 wrench
- `tau` 是关节力矩

由于 floating base 前 6 维没有电机，QP 重点约束：

$$
S_f
\left(
M\ddot q_{\text{opt}}
+h
-J_{fe}^{T}F_{\text{opt}}
\right)=0
$$

QP 变量是：

$$
x=
\begin{bmatrix}
\delta\ddot q_b\\
\delta F
\end{bmatrix}
\in\mathbb{R}^{18}
$$

其中：

```text
delta_ddq_b: 6 维 floating-base 加速度修正
delta_F:     12 维双脚接触 wrench 修正
```

约束行数：

```text
QP_nc = 22 = 6 + 16

前 6 行：floating-base 动力学等式
后 16 行：双脚接触 wrench 不等式
```

单脚 8 个接触不等式包括：

- 接触摩擦锥约束
- 法向力 `fz` 上下界
- 足端 `tau_x / tau_y / tau_z` 力矩界

walk 模式下，代码仍保留双脚 12 维接触 wrench，但会通过 `f_low / f_upp` 把摆动脚接触量限制到接近 0。

### 10.3 QP 数据结构

这里的 `QP_nv = 18` 不是 18 个关节，而是：

$$
x=
\begin{bmatrix}
\delta\ddot q_b\\
\delta F
\end{bmatrix}
\in\mathbb{R}^{18}
$$

其中：

$$
\delta\ddot q_b\in\mathbb{R}^{6}
$$

是 floating-base 加速度修正。

$$
\delta F\in\mathbb{R}^{12}
$$

是双脚接触 wrench 修正。

单脚 wrench 是：

$$
F_{\text{foot}}
=
\begin{bmatrix}
f_x\\
f_y\\
f_z\\
\tau_x\\
\tau_y\\
\tau_z
\end{bmatrix}
$$

双脚 wrench 是：

$$
F=
\begin{bmatrix}
F_L\\
F_R
\end{bmatrix}
\in\mathbb{R}^{12}
$$

QP 的总约束矩阵为：

$$
A_{\text{final}}
=
\begin{bmatrix}
A_1\\
A_2
\end{bmatrix}
=
\begin{bmatrix}
S_fMS_b^T & -S_fJ_{fe}^{T}\\
0_{16\times6} & W
\end{bmatrix}
\in\mathbb{R}^{22\times18}
$$

其中前 6 行是 floating-base 动力学等式：

$$
A_1x=eqRes
$$

后 16 行是接触 wrench 不等式：

$$
neqRes_{\text{low}}\le A_2x\le neqRes_{\text{upp}}
$$

上下界组合为：

$$
l_A=
\begin{bmatrix}
eqRes\\
neqRes_{\text{low}}
\end{bmatrix},
\quad
u_A=
\begin{bmatrix}
eqRes\\
neqRes_{\text{upp}}
\end{bmatrix}
$$

目标函数是：

$$
\min_x \frac{1}{2}x^THx+g^Tx
$$

当前代码中：

$$
g=0
$$

基础权重为：

$$
H=
\begin{bmatrix}
2\cdot10^7I_6 & 0\\
0 & 2\cdot10^1I_{12}
\end{bmatrix}
$$

所以 QP 倾向于：

```text
尽量少改 floating-base 加速度；
相对更愿意调整接触 wrench。
```

### 10.4 如何限制左右脚出力

因为：

$$
WF=
\begin{bmatrix}
W_{\text{foot}}F_L\\
W_{\text{foot}}F_R
\end{bmatrix}
$$

所以 `WF` 的行号含义是：

```text
0 ~ 7   行：左脚约束
8 ~ 15  行：右脚约束
```

每只脚 8 行依次表示：

$$
\begin{bmatrix}
f_x+\frac{\mu}{\sqrt{2}}f_z\\
-f_x+\frac{\mu}{\sqrt{2}}f_z\\
f_y+\frac{\mu}{\sqrt{2}}f_z\\
-f_y+\frac{\mu}{\sqrt{2}}f_z\\
f_z\\
\tau_x\\
\tau_y\\
\tau_z
\end{bmatrix}
$$

`LSt` 时左脚支撑、右脚摆动，代码通过设置右脚约束行 `8 ~ 15` 的上下界，使：

$$
F_R\approx0
$$

`RSt` 时右脚支撑、左脚摆动，代码通过设置左脚约束行 `0 ~ 7` 的上下界，使：

$$
F_L\approx0
$$

也就是说，源码不是直接写死某只脚的力，而是通过：

```text
双脚 wrench 的排列顺序
W 矩阵的行号
f_low / f_upp 的对应位置
```

来约束摆动脚不出力。

### 10.5 反算关节力矩

QP 不直接输出关节力矩，而是先得到：

$$
\ddot q_{\text{opt}}
=
\ddot q_{\text{kin}}
+
\begin{bmatrix}
\delta\ddot q_b\\
0
\end{bmatrix}
$$

$$
F_{\text{opt}}=F_{\text{ff}}+\delta F
$$

然后反算：

$$
\tau_{\text{all}}
=
M\ddot q_{\text{opt}}
+h
-J_{fe}^{T}F_{\text{opt}}
$$

最后取关节部分：

$$
\tau_{\text{joint}}=\tau_{\text{all}}[6:]
$$

对应代码输出：

```text
tauJointRes = tauRes.block(6, 0, model_nv - 6, 1)
```

### 10.6 写回细节

`dataBusWrite()` 有一个容易忽略的覆盖：

```text
先写：
  wbc_ddq_final = eigen_ddq_Opt

后写：
  wbc_ddq_final = ddq_final_kin
```

所以当前源码实际传给下游的是：

```text
wbc_delta_q_final = R6 运动学层 delta_q
wbc_dq_final      = R6 运动学层 dq
wbc_ddq_final     = R6 运动学层 ddq
wbc_tauJointRes   = R7 用 eigen_ddq_Opt / eigen_fr_Opt 反算出的关节力矩
wbc_FrRes         = R7 求出的最终接触 wrench
```

这个细节说明：

```text
QP 修正后的 eigen_ddq_Opt 没有作为 wbc_ddq_final 留给下游；
但它已经参与 tauJointRes 的计算。
```

### 10.7 当前代码现状

这版 WBC 已真实使用：

- `dyn_M`
- `dyn_Non`
- `J_l / J_r`
- `dJ_l / dJ_r`
- `J_base / Jcom / J_hip_link`

这版 WBC 尚未真实使用：

- `dyn_Ag`
- `dyn_dAg`

## 11. R8：PVT_Ctr 生成最终执行力矩

### 11.1 本轮将回答的问题

```text
WBC 的期望 pos/vel/torque，如何变成最终电机输出力矩？
```

详细笔记：

```text
projects/C_openloong_dyn_control_study/logbook/demo_reading/walk_wbc/R8_pvt_motor_output.md
```

### 11.2 当前已读到的数据流

```text
wbc_delta_q_final / wbc_dq_final / wbc_tauJointRes
-> integrateDIY()
-> motors_pos_des / motors_vel_des / motors_tor_des
-> PVT_Ctr
-> motors_tor_out
-> MJ_Interface::setMotorsTorque()
-> mj_data->ctrl
```

### 11.3 这一层的物理意义

这一层不是再做全身任务分配，而是把 WBC 给出的期望关节量整理成更稳定、可执行的关节力矩输出。

R8 的入口来自 R6/R7：

$$
\Delta q_{\text{wbc}}
=
\text{wbc\_delta\_q\_final}
$$

$$
\dot q_{\text{wbc}}
=
\text{wbc\_dq\_final}
$$

$$
\tau_{\text{ff}}
=
\text{wbc\_tauJointRes}
$$

其中 \(\tau_{\text{ff}}\) 是 R7 反算出的关节前馈力矩。

### 11.4 从 WBC 输出到关节命令

`walk_wbc.cpp` 中先积分位置修正：

$$
q_{\text{des}}
=
\operatorname{integrateDIY}
\left(
q,\ \Delta q_{\text{wbc}}
\right)
$$

完整构型 \(q\) 的前 7 维是 floating-base 位姿，所以电机位置命令取关节部分：

$$
q_{j,\text{des}}
=
q_{\text{des}}[7:]
$$

速度和力矩命令为：

$$
\dot q_{j,\text{des}}
=
\dot q_{\text{wbc}}
$$

$$
\tau_{\text{ff}}
=
\text{wbc\_tauJointRes}
$$

对应写入：

```text
motors_pos_des
motors_vel_des
motors_tor_des
```

### 11.5 PVT 控制律

对每个关节，PVT 先计算 PD 反馈：

$$
\tau_{PD,i}
=
K_{p,i}
\left(
q_{des,i}-q_i
\right)
+
K_{d,i}
\left(
\dot q_{des,i}-\dot q_i
\right)
$$

再低通滤波并加上 WBC 前馈力矩：

$$
\tau_{link,i}
=
LPF
\left(
\tau_{PD,i}
\right)
+
\tau_{ff,i}
$$

然后限幅：

$$
\left|
\tau_{link,i}
\right|
\le
\tau_{\max,i}
$$

最后通过减速比换成电机侧力矩：

$$
\tau_{motor,i}
=
\frac{\tau_{link,i}}{gear_i}
$$

所以写入仿真的不是单纯的 WBC 前馈力矩，而是：

$$
\tau_{motor,i}
=
\frac{
\operatorname{sat}
\left(
LPF
\left[
K_{p,i}(q_{des,i}-q_i)
+
K_{d,i}(\dot q_{des,i}-\dot q_i)
\right]
+
\tau_{ff,i}
\right)
}{gear_i}
$$

### 11.6 link-side 和 motor-side 的区别

`PVT_Ctr` 内部同时保存：

```text
motor_tor_out_link
motor_tor_out_motor
```

写回 `DataBus` 时：

```text
motors_tor_cur = motor_tor_out_link
motors_tor_out = motor_tor_out_motor
```

最后 `MJ_Interface::setMotorsTorque()` 使用：

$$
\text{mj\_data->ctrl}[i]
=
\text{motors\_tor\_out}[i]
$$

因此：

```text
motors_tor_cur  记录关节侧力矩
motors_tor_out  是写入 MuJoCo ctrl 的电机侧力矩
```

### 11.7 启动阶段的特殊处理

仿真前几秒会调用：

```text
calMotorsPVT(deltaP_Lim)
```

其作用是限制每个控制周期的位置命令变化量：

$$
\left|
q_{des,i}^{use}
-
q_{des,i}^{old}
\right|
\le
\Delta q_{\max}
$$

这样可以避免启动阶段位置命令突变，使 PD 项瞬间产生过大的力矩。

### 11.8 本轮最终结论

R8 逐函数读完了：

```text
PVT_Ctr::PVT_Ctr()
PVT_Ctr::dataBusRead()
PVT_Ctr::dataBusWrite()
PVT_Ctr::setJointPD()
PVT_Ctr::calMotorsPVT()
PVT_Ctr::calMotorsPVT(deltaP_Lim)
PVT_Ctr::sign()
PVT_Ctr::enablePV() / disablePV()
MJ_Interface::setMotorsTorque()
```

最终链路为：

$$
\Delta q_{\text{wbc}},\ \dot q_{\text{wbc}},\ \tau_{\text{wbc}}
\rightarrow
q_{j,des},\ \dot q_{j,des},\ \tau_{ff}
\rightarrow
\tau_{link}
\rightarrow
\tau_{motor}
\rightarrow
\text{mj\_data->ctrl}
$$

核心控制律为：

$$
\tau_{motor,i}
=
\frac{
\operatorname{sat}_{\tau_{\max,i}}
\left(
LPF_i
\left[
PV_i
\left(
K_{p,i}(q_{des,i}-q_i)
+
K_{d,i}(\dot q_{des,i}-\dot q_i)
\right)
\right]
+
\tau_{ff,i}
\right)
}{gear_i}
$$

其中：

```text
tau_ff       = wbc_tauJointRes
tau_link     = 关节侧力矩
tau_motor    = 电机侧力矩
motors_tor_out = tau_motor
mj_data->ctrl  = motors_tor_out
```

R8 也留下一个 R9 复盘检查点：

```text
motors_vel_des = wbc_dq_final
```

当前源码没有显式取 `wbc_dq_final[6:]`，而 `dq_final_kin` 在 WBC 内部按 `model_nv` 维初始化，因此 R9 可以专门复核这个速度命令的维度和索引是否与 `motors_vel_des` 完全一致。

## 12. R9：闭环总复盘

### 12.1 本轮将回答的问题

```text
这一整条链是否真正形成了稳定的 MuJoCo 闭环？
```

### 12.2 最后要能独立说出的总链

```text
上一轮 motors_tor_out
-> mj_data->ctrl
-> mj_step
-> MJ_Interface / DataBus
-> StateEst
-> Pin_KinDyn
-> gait / task
-> WBC computeDdq
-> WBC computeTau
-> PVT_Ctr
-> 新一轮 motors_tor_out
```

## 13. R10：面试求职项目简述

### 13.1 本轮回答的问题

```text
如何把 walk_wbc 源码学习成果整理成适合简历和面试表达的项目经历？
```

### 13.2 独立文档

详见：

```text
R10_interview_project_summary.md
```

### 13.3 面试一句话

```text
我系统阅读并复盘了 OpenLoong 人形机器人 walk_wbc 行走控制 demo，重点理解了 floating-base 人形机器人如何通过状态估计、Pinocchio 运动学动力学、步态与落脚点规划、WBC/QP 和 PVT 控制形成从 MuJoCo 状态到电机力矩输出的完整闭环。
```

### 13.4 面试最核心公式

floating-base 动力学一致性：

$$
S_f\left(M\ddot q+h-J^TF\right)=0
$$

关节力矩反算：

$$
\tau_j
=
\left[
M\ddot q_{\mathrm{opt}}
+h
-J^TF_{\mathrm{opt}}
\right]_{6:}
$$

PVT 输出：

$$
\tau_{\mathrm{motor},i}
=
\frac{
\mathrm{sat}
\left(
LPF\left[
K_{p,i}(q_{des,i}-q_i)
+
K_{d,i}(\dot q_{des,i}-\dot q_i)
\right]
+
\tau_{\mathrm{ff},i}
\right)
}{gear_i}
$$

## 14. R11：复现推进结论与下一步计划

### 14.1 本轮回答的问题

```text
walk_wbc 阅读完成后，复现项目应该怎么推进？
```

### 14.2 独立文档

详见：

```text
R11_reproduction_conclusion_and_next_plan.md
```

### 14.3 当前总判断

```text
walk_wbc 阅读主线已经完成。
下一阶段应从源码阅读转向最小复现。
复现不应直接全量复刻 walk_wbc，而应从 Pinocchio 模型量开始。
```

### 14.4 推荐复现路线

```text
阶段 A：最小 DataBus 数据流
阶段 B：Pinocchio floating-base 模型量
阶段 C：最小 WBC 运动学层
阶段 D：最小 WBC 动力学 QP
阶段 E：PVT 最小闭环
阶段 F：MuJoCo 最小仿真闭环
```

### 14.5 下一步最小任务

```text
写一个脚本，加载 floating-base URDF，输出 nq/nv、脚部 Jacobian、M、h，并生成复盘 txt。
```

## 15. 当前最重要的复习抓手

如果你现在只想保留 6 句话，建议保留这 6 句：

1. `walk_wbc` 是一个完整控制闭环装配文件，不是单一算法文件。
2. `StateEst` 决定了 floating-base `q/dq` 中 base 部分最终用什么状态。
3. `Pin_KinDyn` 把 `q/dq` 变成 `J / dJ / pose / dyn_* / CoM` 这些控制器真正需要的模型量。
4. `JoyStickInterpreter / GaitScheduler / FootPlacement` 把期望速度变成 `base_*_des / legState / phi / swing_fe_pos_des_W`。
5. `WBC` 当前真实依赖的是 `J_*`、`dJ_*`、`dyn_M`、`dyn_Non`、`Jcom_W`、`pCoM_W`，而不是质心动量矩阵 `dyn_Ag / dyn_dAg`。
6. `PVT_Ctr` 把全身层结果转成 MuJoCo 下一步真正执行的电机力矩。

## 16. 下一步建议

当前 `walk_wbc` 主阅读链已经完成，后续最合适进入的是复现和验证：

```text
1. 最小 Pinocchio floating-base 模型量检查
2. 验证 nq / nv / J / M / h 的维度
3. 再复现最小 WBC/QP demo
4. 验证 wbc_dq_final -> motors_vel_des 的维度一致性
5. 用日志画 base 速度、接触力、关节力矩曲线
```
