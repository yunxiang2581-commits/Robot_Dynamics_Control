# R0: walk_mpc_wbc Pipeline 总览

## 1. 本轮目标

本轮只建立 `walk_mpc_wbc.cpp` 的整体控制链路，并重点对比它和 `walk_wbc.cpp` 的差异。

这一轮不深入推导 `MPC::cal()` 里的 QP 矩阵，只先回答三个问题：

1. MPC 插在整个控制流程的哪个位置？
2. MPC 相比 `walk_wbc` 新增了哪些数据？
3. MPC 最终把哪些量写给 WBC？

## 2. 和 walk_wbc 的共同主线

`walk_mpc_wbc` 保留了 `walk_wbc` 的大部分闭环结构：

```text
MuJoCo simulation
-> MJ_Interface
-> DataBus
-> Pin_KinDyn
-> JoyStickInterpreter
-> GaitScheduler
-> FootPlacement
-> WBC_priority
-> PVT_Ctr
-> mj_data->ctrl
```

这些模块的作用和 `walk_wbc` 基本一致：

- `MJ_Interface` 从 MuJoCo 读取传感器、关节和 base 状态。
- `Pin_KinDyn` 计算运动学和动力学量，例如 Jacobian、质量矩阵、非线性项。
- `JoyStickInterpreter` 生成期望 base 速度和位姿参考。
- `GaitScheduler` 生成左右脚支撑/摆动状态。
- `FootPlacement` 生成摆动脚落脚点。
- `WBC_priority` 计算全身运动学任务和关节前馈力矩。
- `PVT_Ctr` 把 WBC 目标变成最终电机侧 torque。

所以本 demo 不是重写控制框架，而是在原有 WBC 控制链路中间插入 MPC。

## 3. walk_wbc 与 walk_mpc_wbc 的核心差异

`walk_wbc` 的关键链路可以概括为：

```text
Joystick / Gait / FootPlacement
-> WBC
-> PVT
-> MuJoCo actuator
```

`walk_mpc_wbc` 的关键链路变成：

```text
Joystick / Gait / FootPlacement
-> MPC
-> WBC
-> PVT
-> MuJoCo actuator
```

数学上可以理解为：

$$
\text{walk\_wbc:}
\quad
r_{\text{joy}},\ gait,\ footstep
\rightarrow
WBC
\rightarrow
\tau_j
$$

而：

$$
\text{walk\_mpc\_wbc:}
\quad
r_{\text{joy}},\ gait,\ footstep
\rightarrow
MPC
\rightarrow
\left(
F_{r,ff},
q_{b,des},
\dot q_{b,des},
\ddot q_{b,des}
\right)
\rightarrow
WBC
\rightarrow
\tau_j
$$

其中：

$$
F_{r,ff}
$$

是 MPC 计算出的双脚接触 wrench 前馈。

## 4. 新增 MPC 相关源码位置

主文件新增头文件：

```cpp
#include "mpc.h"
```

新增 MPC 控制周期：

```cpp
const double dt = 0.001;
const double dt_200Hz = 0.005;
```

含义是：

$$
dt=0.001s
$$

主仿真和主控制循环约为 1000 Hz。

$$
dt_{MPC}=0.005s
$$

MPC 以 200 Hz 运行。

新增 MPC 对象：

```cpp
MPC MPC_solv(dt_200Hz);
```

这说明 MPC 内部模型离散步长使用的是：

$$
dt_{MPC}
$$

而不是 MuJoCo 每一步的：

$$
dt
$$

## 5. MPC 在主循环中的插入位置

主循环中，MPC 位于 gait / foot placement 之后，WBC 之前：

```text
MJ_Interface
-> Pin_KinDyn
-> JoyStickInterpreter
-> GaitScheduler
-> FootPlacement
-> MPC
-> WBC
-> PVT
```

源码结构是：

```cpp
MPC_count = MPC_count + 1;
if (MPC_count > (dt_200Hz / dt - 1)) {
    MPC_solv.dataBusRead(RobotState);
    MPC_solv.cal();
    MPC_solv.dataBusWrite(RobotState);
    MPC_count = 0;
}
```

因为：

$$
\frac{dt_{MPC}}{dt}
=
\frac{0.005}{0.001}
=
5
$$

所以 MPC 每 5 个主循环运行一次。

这一段的数据流是：

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

随后 WBC 再读取更新后的 `RobotState`：

```cpp
WBC_solv.dataBusRead(RobotState);
WBC_solv.computeDdq(kinDynSolver);
WBC_solv.computeTau();
WBC_solv.dataBusWrite(RobotState);
```

所以核心顺序是：

$$
MPC
\rightarrow
RobotState
\rightarrow
WBC
$$

## 6. MPC 新增日志变量

`walk_mpc_wbc.cpp` 相比 `walk_wbc` 新增了 MPC 相关日志项：

```cpp
logger.addIterm("dX_cal", 12);
logger.addIterm("Ufe", 12);
logger.addIterm("Xd", 12);
logger.addIterm("X_cur", 12);
logger.addIterm("X_cal", 12);
```

这些变量含义是：

$$
X_{cur}
$$

当前 MPC 状态。

$$
X_d
$$

MPC 期望状态轨迹。

$$
X_{cal}
$$

MPC 预测/计算得到的状态。

$$
\dot X_{cal}
$$

MPC 计算得到的状态变化率。

$$
U_{fe}
$$

MPC 优化得到的足端接触力/力矩控制量。

这说明本 demo 不只是看最终电机力矩，也专门记录 MPC 的预测状态和接触力输出。

## 7. MPC 最终写给 WBC 的关键量

`MPC::dataBusWrite()` 是 `MPC -> WBC` 的接口。

最重要的一句是：

```cpp
Data.Fr_ff = Ufe.block<12, 1>(0, 0);
```

也就是：

$$
F_{r,ff}
=
U_{fe}[0:12]
$$

其中：

$$
F_{r,ff}
=
\begin{bmatrix}
f_L \\
\tau_L \\
f_R \\
\tau_R
\end{bmatrix}
\in
\mathbb{R}^{12}
$$

展开为：

$$
F_{r,ff}
=
\begin{bmatrix}
f_{Lx}\\
f_{Ly}\\
f_{Lz}\\
\tau_{Lx}\\
\tau_{Ly}\\
\tau_{Lz}\\
f_{Rx}\\
f_{Ry}\\
f_{Rz}\\
\tau_{Rx}\\
\tau_{Ry}\\
\tau_{Rz}
\end{bmatrix}
$$

MPC 还写入 base 运动目标：

$$
\ddot q_{b,des}
$$

$$
\dot q_{b,des}
$$

$$
\Delta q_{b,des}
$$

以及：

$$
base\_rpy_{des},
\quad
base\_pos_{des}
$$

所以 `walk_mpc_wbc` 的关键变化是：

```text
walk_wbc: 主循环直接给 WBC 设置 base 速度/加速度目标。
walk_mpc_wbc: MPC 根据预测模型优化出 Fr_ff 和 base 目标，再交给 WBC。
```

## 8. 本 demo 的控制层级理解

可以把 `walk_mpc_wbc` 理解成三层：

```text
高层参考:
JoyStickInterpreter / GaitScheduler / FootPlacement

中层预测控制:
MPC

全身动力学与低层输出:
WBC_priority / PVT_Ctr
```

数学数据流是：

$$
r_{\text{joy}},\ gait,\ footstep
\rightarrow
X_d
\rightarrow
MPC
\rightarrow
U_{fe},\ X_{cal},\ \dot X_{cal}
\rightarrow
F_{r,ff},\ q_{b,des},\dot q_{b,des},\ddot q_{b,des}
\rightarrow
WBC
\rightarrow
\tau_j
\rightarrow
PVT
\rightarrow
u_{\text{motor}}
$$

## 9. R0 结论

`walk_mpc_wbc` 的重点不是重新实现 WBC，而是在 WBC 前面加入一个基于整体动力学的 MPC。

MPC 的作用是：

1. 读取当前 base 状态、足端位置、步态状态和 joystick 参考。
2. 在未来预测窗口内优化接触力。
3. 输出双脚接触 wrench 前馈。
4. 输出 base 的位置、速度和加速度目标。
5. 让 WBC 在这些更合理的目标上继续做全身任务求解和关节力矩计算。

一句话总结：

```text
walk_wbc 主要是 WBC 直接跟踪行走目标；
walk_mpc_wbc 是 MPC 先规划整体质心/base 与接触力，再由 WBC 落实到全身关节控制。
```

## 10. 后续阅读轮次

后续按以下轮次继续：

- R1: `walk_mpc_wbc.cpp` 主循环差异逐段读。
- R2: `MPC::dataBusRead()`，读 MPC 输入和状态向量。
- R3: `MPC::cal()` 的状态空间模型。
- R4: `MPC::cal()` 的 QP 目标函数。
- R5: `MPC::cal()` 的约束构造。
- R6: `MPC::dataBusWrite()`，读 MPC 如何把结果交给 WBC。
- R7: 与 `walk_wbc` 的完整对比和面试总结。
