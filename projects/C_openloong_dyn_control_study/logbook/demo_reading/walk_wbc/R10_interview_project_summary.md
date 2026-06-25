# R10：面试求职项目简述

## 1. 文档目的

本轮不是继续逐行阅读源码，而是把 `walk_wbc` 学习成果整理成适合简历和面试表达的项目说明。

重点回答：

```text
我做了什么？
我理解了什么核心技术？
这个项目能体现哪些机器人控制能力？
面试时如何从一句话讲到完整技术链路？
```

## 2. 简历版项目描述

项目名称：基于 OpenLoong 的人形机器人全身动力学控制学习与源码解析

围绕 OpenLoong-Dyn-Control 的 `walk_wbc` 行走控制 demo，系统梳理了从 MuJoCo 仿真、状态估计、Pinocchio 运动学/动力学建模、步态规划、落脚点规划、WBC 全身控制到 PVT 电机力矩输出的完整闭环。重点分析 floating-base 人形机器人在行走中的状态流、任务流和动力学力矩生成过程。

深入阅读并复盘了 `walk_wbc.cpp` 主控制循环、`WBC_priority::computeDdq()`、`WBC_priority::computeTau()`、`PVT_Ctr`、`Pin_KinDyn`、`StateEst`、`GaitScheduler`、`FootPlacement` 和 MuJoCo 接口模块，理解 WBC 如何生成期望广义加速度，如何通过 QP 修正 floating-base 动力学一致性，并最终反算关节前馈力矩。

## 3. 简历关键词

```text
Humanoid Robot
Floating-base Dynamics
Whole-Body Control
Quadratic Programming
Pinocchio
MuJoCo
State Estimation
Gait Scheduling
Foot Placement
PVT Joint Control
```

## 4. 简历亮点条目

- 梳理人形机器人行走控制从仿真状态读取到电机力矩输出的完整实时闭环。
- 解析 WBC 运动学层任务优先级求解，以及动力学层 QP 修正逻辑。
- 理解 floating-base 无驱动约束：

$$
S_f\left(M\ddot q+h-J^TF\right)=0
$$

- 推导 WBC 动力学修正后的关节力矩反算：

$$
\tau_j
=
\left[
M\ddot q_{\mathrm{opt}}
+h
-J^TF_{\mathrm{opt}}
\right]_{6:}
$$

- 分析 PVT 控制器如何结合 PD 反馈、低通滤波、WBC 前馈力矩和减速比输出 MuJoCo actuator torque。
- 建立面向源码阅读的模块化学习笔记，形成 `MuJoCo -> StateEst -> Pinocchio -> Gait -> WBC -> PVT -> MuJoCo` 的完整数据流图。

## 5. 面试一句话版本

我系统阅读并复盘了 OpenLoong 人形机器人 `walk_wbc` 行走控制 demo，重点理解了 floating-base 人形机器人如何通过状态估计、Pinocchio 运动学动力学、步态与落脚点规划、WBC/QP 和 PVT 控制形成从 MuJoCo 状态到电机力矩输出的完整闭环。

## 6. 面试 30 秒版本

这个项目中，我不是只运行 demo，而是逐段阅读 OpenLoong 的 `walk_wbc` 源码，整理了人形机器人行走控制的完整数据流。每个仿真周期中，MuJoCo 先推进状态，`MJ_Interface` 读取关节和 IMU 数据，`StateEst` 修正 floating-base 状态，`Pin_KinDyn` 使用 Pinocchio 计算 Jacobian 和动力学项，步态与落脚点模块生成行走目标，WBC 计算期望运动和前馈关节力矩，最后 PVT 控制器输出电机力矩写回 `mj_data->ctrl`。

其中我重点推导了 WBC 动力学层 QP，理解了为什么 floating-base 前 6 维不能直接输出力矩，以及如何通过接触 wrench 修正和动力学一致性约束反算关节力矩。

## 7. 面试 2 分钟版本

这个项目围绕 OpenLoong-Dyn-Control 的 `walk_wbc.cpp` 展开。我按控制闭环拆解源码，而不是只看单个函数。

首先，MuJoCo 通过：

$$
mj\_step(mj\_model,mj\_data)
$$

推进仿真状态。随后 `MJ_Interface` 从 `mj_data` 中读取关节位置、关节速度、IMU、base 姿态和传感器信息，并写入统一数据总线 `RobotState`。

然后 `StateEst` 根据 IMU、足端状态和步态接触状态估计 floating-base 的位置、速度、姿态和角速度，并更新控制器内部使用的：

$$
q,\quad \dot q
$$

接着 `Pin_KinDyn` 使用 Pinocchio 计算当前构型下的运动学和动力学项：

$$
J,\quad \dot J,\quad M,\quad M^{-1},\quad h
$$

其中：

$$
h=C(q,\dot q)\dot q+g(q)
$$

行走目标由 `JoyStickInterpreter`、`GaitScheduler` 和 `FootPlacement` 生成。`JoyStickInterpreter` 生成 base 期望速度和位置，`GaitScheduler` 生成支撑腿、摆动腿和步态相位：

$$
\phi\in[0,1]
$$

`FootPlacement` 根据当前速度、期望速度、髋部位置和相位生成摆动脚目标轨迹。

WBC 分为两层。第一层 `computeDdq()` 是运动学层任务优先级求解，输出：

$$
\Delta q_{\mathrm{wbc}},\quad
\dot q_{\mathrm{wbc}},\quad
\ddot q_{\mathrm{wbc}}
$$

第二层 `computeTau()` 是动力学层 QP。它优化：

$$
x=
\begin{bmatrix}
\delta\ddot q_b\\
\delta F
\end{bmatrix}
$$

其中：

$$
\delta\ddot q_b\in\mathbb R^6
$$

是 floating-base 加速度修正，

$$
\delta F\in\mathbb R^{12}
$$

是双脚接触 wrench 修正。

QP 的核心约束是 floating-base 动力学一致性：

$$
S_f\left(M\ddot q+h-J^TF\right)=0
$$

这体现了 floating-base 人形机器人的关键特点：base 本身没有电机，不能直接输出力矩，只有关节可以被驱动。

求解后得到：

$$
\ddot q_{\mathrm{opt}}
=
\ddot q_{\mathrm{kin}}
+
\begin{bmatrix}
\delta\ddot q_b\\
0
\end{bmatrix}
$$

$$
F_{\mathrm{opt}}
=
F_{\mathrm{ff}}+\delta F
$$

再反算广义力：

$$
\tau_{\mathrm{res}}
=
M\ddot q_{\mathrm{opt}}
+h
-J^TF_{\mathrm{opt}}
$$

最终只取关节部分：

$$
\tau_j=\tau_{\mathrm{res}}[6:]
$$

最后，PVT 控制器把 WBC 的位置、速度和前馈力矩转成电机侧 torque：

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

并写入：

$$
mj\_data->ctrl
$$

形成下一轮 MuJoCo 仿真的控制输入。

## 8. 项目完整数据流

```text
上一轮 motors_tor_out
-> mj_data->ctrl
-> mj_step
-> MJ_Interface.updateSensorValues()
-> MJ_Interface.dataBusWrite()
-> DataBus.updateQ()
-> StateEst.set/update/get
-> Pin_KinDyn.dataBusRead()
-> Pin_KinDyn.computeJ_dJ()
-> Pin_KinDyn.computeDyn()
-> Pin_KinDyn.dataBusWrite()
-> JoyStickInterpreter / GaitScheduler / FootPlacement
-> WBC_priority.dataBusRead()
-> WBC_priority.computeDdq()
-> WBC_priority.computeTau()
-> WBC_priority.dataBusWrite()
-> integrateDIY()
-> motors_pos_des / motors_vel_des / motors_tor_des
-> PVT_Ctr.calMotorsPVT()
-> motors_tor_out
-> MJ_Interface.setMotorsTorque()
-> 下一轮 mj_step
```

## 9. 面试重点问题与回答抓手

### 9.1 你在这个项目里主要做了什么？

我把 OpenLoong 的 `walk_wbc` demo 按模块和数据流完整拆开，逐段阅读源码并推导关键公式，形成了从 MuJoCo 状态读取、floating-base 状态估计、Pinocchio 动力学计算、步态规划、WBC/QP 到 PVT 电机力矩输出的闭环理解。

### 9.2 WBC 为什么需要 QP？

运动学层 WBC 可以给出期望运动：

$$
\Delta q,\quad \dot q,\quad \ddot q
$$

但 floating-base 人形机器人必须满足无驱动 base 的动力学约束：

$$
S_f\left(M\ddot q+h-J^TF\right)=0
$$

所以动力学层 QP 用：

$$
\delta\ddot q_b,\quad \delta F
$$

修正 base 加速度和接触力，使运动学结果能被真实接触和关节力矩支持。

### 9.3 为什么最后只取关节力矩？

因为 floating-base 前 6 维不是电机自由度，没有实际 actuator。反算得到的广义力：

$$
\tau_{\mathrm{res}}\in\mathbb R^{6+n_j}
$$

中，前 6 维对应 base wrench，只用于检查动力学一致性；真正能输出给电机的是后面：

$$
\tau_j=\tau_{\mathrm{res}}[6:]
$$

### 9.4 PVT 和 WBC 的关系是什么？

WBC 输出的是全身层的期望关节运动和前馈关节力矩：

$$
q_{j,des},\quad \dot q_{j,des},\quad \tau_{ff}
$$

PVT 是低层关节控制器，用 PD 反馈补偿跟踪误差，同时叠加 WBC 前馈力矩，最终输出电机侧力矩：

$$
\tau_{\mathrm{motor}}
$$

### 9.5 这个项目体现的能力

- 能读懂 floating-base 人形机器人控制代码。
- 能把源码变量和动力学公式对应起来。
- 能理解 Pinocchio 中 `nq/nv`、free-flyer、Jacobian、CRBA、Coriolis、gravity 的作用。
- 能解释 WBC/QP 为什么不是直接求关节力矩，而是先修正 base 加速度和接触 wrench。
- 能说明从高层行走目标到低层 actuator torque 的完整工程链路。

### 9.6 五个高频面试题可背诵版

#### 问题 1：请用 1 分钟介绍这个项目

我主要围绕 OpenLoong-Dyn-Control 的人形机器人 `walk_wbc` demo 做了系统源码拆解和控制链路复盘，重点阅读了 `walk_wbc` 的完整实时控制闭环。

单个控制周期里，MuJoCo 先推进仿真状态，接口层读取关节、IMU 和接触信息，状态估计模块修正 floating-base 的 \(q,\dot q\)，Pinocchio 计算 Jacobian、质量矩阵和非线性项，步态与落脚点模块生成行走任务目标，最后 WBC 或 MPC+WBC 计算全身控制量，再通过 PVT 转成电机力矩写回 `mj_data->ctrl`。

这个项目的 demo 覆盖了普通行走、MPC+WBC 行走、joystick 交互、跳跃和楼梯场景。我重点理解的是状态估计、Pinocchio 模型量计算、WBC/QP 动力学一致性、MPC 接触力规划，以及 PVT 电机输出这几部分。

#### 问题 2：为什么 WBC 里有 floating-base 动力学约束？

对 floating-base 人形机器人，广义速度可以写成：

$$
\dot q=
\begin{bmatrix}
v_b\\
\dot q_j
\end{bmatrix}
$$

其中：

$$
v_b\in\mathbb R^6
$$

是 base 的线速度和角速度，\(\dot q_j\) 是关节速度。

完整动力学是：

$$
M\ddot q+h=J^TF+S^T\tau
$$

但 floating base 本身没有电机，前 6 维不能直接施加关节力矩，所以电机力矩只作用在关节维度。\(S_f\) 是选择矩阵：

$$
S_f=
\begin{bmatrix}
I_6 & 0
\end{bmatrix}
$$

它的作用是取出动力学方程前 6 行，要求 base 的动力学只能由惯性、重力和接触力平衡：

$$
S_f\left(M\ddot q+h-J^TF\right)=0
$$

在 QP 中，这个约束保证 WBC 运动学层给出的 \(\ddot q\) 和接触力 \(F\) 是动力学可实现的，不会产生一个需要 base 电机才能实现的虚假运动。

短版回答：

```text
Sf 是 floating-base 选择矩阵，用来取出广义动力学前 6 行。
因为人形机器人 base 是浮动的，没有 actuator，电机力矩只能作用在关节维度。
所以前 6 维必须满足无驱动动力学平衡。
QP 用这个约束修正 base 加速度和接触力，保证 WBC 的运动结果动力学可实现。
```

#### 问题 3：`computeDdq()` 和 `computeTau()` 分别做什么？

`computeDdq()` 是 WBC 的运动学层求解。它根据当前运动状态、支撑脚/摆动脚、base 姿态、摆动脚目标、关节姿态等任务，按优先级求出机器人下一步应该怎么运动，输出：

$$
\Delta q_{\text{wbc}},\quad
\dot q_{\text{wbc}},\quad
\ddot q_{\text{wbc}}
$$

也就是位置增量、速度和加速度层面的全身运动参考。

`computeTau()` 是动力学层求解。它在 `computeDdq()` 给出的运动学结果基础上，考虑 floating-base 无驱动约束和双脚接触力约束，通过 QP 修正：

$$
\delta\ddot q_b,\quad \delta F
$$

得到动力学一致的：

$$
\ddot q_{\text{opt}},\quad F_{\text{opt}}
$$

然后用动力学方程反算关节力矩：

$$
\tau_{\text{res}}
=
M\ddot q_{\text{opt}}+h-J^TF_{\text{opt}}
$$

最后取关节部分：

$$
\tau_j=\tau_{\text{res}}[6:]
$$

作为 WBC 输出给 PVT 的前馈关节力矩。

短版回答：

```text
computeDdq 解决“机器人应该怎么动”，是运动学任务优先级求解，输出 delta_q、dq、ddq。
computeTau 解决“这个运动怎么由接触力和关节力矩实现”，通过 QP 修正 base 加速度和接触力，再反算关节力矩 tau_j。
```

#### 问题 4：QP 变量为什么是 \(\delta\ddot q_b,\delta F\)，而不是关节力矩？

在 `computeTau()` 里，QP 不是直接优化关节力矩，而是优化：

$$
x=
\begin{bmatrix}
\delta\ddot q_b\\
\delta F
\end{bmatrix}
$$

\(\delta\ddot q_b\) 是 floating-base 前 6 维加速度修正，用来让运动学层给出的：

$$
\ddot q_{\text{kin}}
$$

满足 base 动力学约束。

\(\delta F\) 是双脚接触 wrench 的修正量。每只脚的 contact wrench 包括三维接触力和三维接触力矩：

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

双脚合起来是：

$$
F\in\mathbb R^{12}
$$

QP 求出修正后：

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
F_{\text{opt}}=F_{ff}+\delta F
$$

然后根据动力学方程反算：

$$
\tau_{\text{res}}
=
M\ddot q_{\text{opt}}+h-J^TF_{\text{opt}}
$$

最终只取关节部分：

$$
\tau_j=\tau_{\text{res}}[6:]
$$

所以这里不直接优化关节力矩，是因为关节力矩可以在 \(\ddot q\) 和接触力确定后由动力学反算出来；QP 更关注 floating-base 无驱动约束和接触力可行性。

短版回答：

```text
delta_ddq_b 修正 base 加速度，delta_F 修正双脚接触 wrench。
wrench 是三维力加三维力矩，不是位移和角度。
QP 先保证 base 动力学和接触力可行，再通过动力学方程反算关节力矩，所以不需要直接把关节力矩作为优化变量。
```

#### 问题 5：WBC 输出如何变成最终的 `mj_data->ctrl`？

WBC 输出后，代码先把全身层结果转换成 PVT 能用的三个输入：

$$
q_{j,des},\quad \dot q_{j,des},\quad \tau_{ff}
$$

其中 `wbc_delta_q_final` 不是绝对位置，而是当前构型上的增量，所以代码先调用：

$$
q_{des}=integrateDIY(q,\Delta q_{\text{wbc}})
$$

`integrateDIY()` 的作用是把当前完整构型 \(q\) 和 WBC 给出的增量 \(\Delta q\) 积分成新的期望构型。因为 floating-base 里有四元数，所以不能简单做普通加法。它会处理：

$$
p_b^{des}=p_b+\Delta p_b
$$

$$
Q_b^{des}=Q_b\oplus\Delta\theta_b
$$

$$
q_j^{des}=q_j+\Delta q_j
$$

然后代码只取关节部分：

$$
q_{j,des}=q_{des}[7:]
$$

写入：

```text
motors_pos_des
```

同时：

$$
motors\_vel\_des=wbc\_dq\_final
$$

$$
motors\_tor\_des=wbc\_tauJointRes
$$

其中 `motors_tor_des` 是 WBC 给 PVT 的前馈关节力矩，不是最终输出到 MuJoCo 的力矩。

接下来 `PVT_Ctr` 读取当前关节状态和这些目标，做低层关节控制。核心公式是：

$$
\tau_{PD,i}
=
K_{p,i}(q_{des,i}-q_i)
+
K_{d,i}(\dot q_{des,i}-\dot q_i)
$$

然后经过低通滤波并叠加 WBC 前馈力矩：

$$
\tau_{link,i}
=
\mathrm{sat}
\left(
LPF(\tau_{PD,i})
+
\tau_{ff,i}
\right)
$$

这里：

$$
\tau_{ff,i}=motors\_tor\_des
$$

再根据减速比 `gear` 转成电机侧力矩：

$$
\tau_{motor,i}
=
\frac{\tau_{link,i}}{gear_i}
$$

`gear` 是电机到关节之间的减速比。关节侧力矩和电机侧力矩近似满足：

$$
\tau_{link}=gear\cdot\tau_{motor}
$$

所以代码输出给 actuator 时要除以：

$$
gear
$$

最后：

$$
motors\_tor\_out=\tau_{motor}
$$

并通过：

```cpp
mj_interface.setMotorsTorque(RobotState.motors_tor_out);
```

写入：

$$
mj\_data->ctrl
$$

完整数据流是：

```text
wbc_delta_q -> integrateDIY -> motors_pos_des
wbc_dq -> motors_vel_des
wbc_tauJointRes -> motors_tor_des
motors_pos_des / motors_vel_des / motors_tor_des
-> PVT
-> motors_tor_out
-> mj_data->ctrl
```

短版回答：

```text
WBC 给出关节运动目标和前馈力矩。
integrateDIY 把构型增量变成期望关节位置。
PVT_Ctr 用 PD 反馈加 WBC 前馈力矩生成电机侧 torque。
最后 motors_tor_out 写入 MuJoCo 的 mj_data->ctrl。
```

## 10. 可继续深化的方向

后续如果要把这个项目进一步变成更强的求职作品，可以继续补充：

- 独立复现一个最小 WBC/QP demo。
- 对 `wbc_dq_final -> motors_vel_des` 的维度一致性做源码验证。
- 记录一组仿真日志，画出 base 速度、接触力、关节力矩曲线。
- 对比站立、原地踏步、前进走路三种状态下的任务权重和接触约束变化。
- 将 R1-R9 的源码阅读笔记整理成一页控制架构图。
