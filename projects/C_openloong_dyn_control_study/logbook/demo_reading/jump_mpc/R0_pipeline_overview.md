# R0: jump_mpc Pipeline 总览

## 1. 本轮目标

本轮先不拆具体函数，只建立 `jump_mpc.cpp` 的整体控制链路。

跳跃 demo 和行走 demo 的最大区别是：

```text
walk_mpc_wbc: 周期性换脚，重点是步态相位与摆脚轨迹
jump_mpc:     状态机驱动，重点是起跳、腾空、下落、触地恢复
```

这一轮只回答三个问题：

1. `jump_mpc` 的控制主线是什么？
2. MPC 在跳跃里什么时候启用、什么时候关闭？
3. MPC 的输出最后怎样进入电机命令？

## 2. 先看总流程

`jump_mpc.cpp` 的控制链路可以先记成：

```text
MuJoCo
-> MJ_Interface / DataBus
-> Pin_KinDyn
-> 跳跃状态机 jump_state
-> MPC
-> 接触力映射 / 关节力矩修正
-> PVT_Ctr
-> mj_data->ctrl
```

和 `walk_mpc_wbc` 相比，这里不再强调 `gaitScheduler` 和 `FootPlacement` 的周期性步态，而是靠 `jump_state` 来切换不同阶段的控制策略。

## 3. jump_mpc 的阶段结构

源码里最关键的是：

```cpp
uint16_t jump_state = 0;
double startJumpingTime = 8.5;
double prepareTime = 3;
```

所以跳跃过程可以粗分成：

```text
准备阶段
-> 起跳阶段
-> 上升 / 腾空
-> 下降
-> 触地恢复
```

大致对应源码里的状态：

- `jump_state == 0`：起跳前准备跳跃
- `jump_state == 3`：上升 / 腾空阶段
- `jump_state == 4`：下降 / 接近落地阶段
- `jump_state == 5`：恢复阶段

这和 walking 的周期性 `legState` 不一样，jump 的控制逻辑是显式状态机。

## 4. 跳跃 demo 的核心变量

这一版跳跃控制最重要的变量有：

```text
jump_state
jump_z
jump_vel_des
jump_acc_t
mpc_force
Uje
Jac_stand
FLest / FRest
fe_react_tau_cmd
```

先给出它们的直观意义：

- `jump_state`：当前跳跃阶段。
- `jump_z`：希望腾空高度，对应起跳速度尺度。
- `jump_vel_des`：期望起跳线速度。
- `jump_acc_t`：起跳加速持续时间。
- `mpc_force`：跳跃用的 MPC 控制器。
- `Uje`：把接触 wrench 映射到关节的 12 维力矩修正。
- `Jac_stand`：双脚支撑时的接触 Jacobian。
- `FLest / FRest`：根据关节力矩反推的左右脚接触力估计。

## 5. MPC 在 jump_mpc 里怎么插

`jump_mpc` 里和 `walk_mpc_wbc` 一样有 MPC，但调用方式更直接：

```cpp
mpc_force.dataBusRead(RobotState);
mpc_force.cal();
mpc_force.dataBusWrite(RobotState);
```

也就是说，跳跃 demo 里 MPC 仍然是：

```text
读状态
-> 求解
-> 写回结果
```

但它后面的使用方式和 walk 不同。

在 `jump_mpc.cpp` 里，MPC 的输出会被进一步拿去做：

```cpp
Uje = Jac_stand.transpose() * (-1.0) * RobotState.fe_react_tau_cmd.block<nu - 1, 1>(0, 0);
```

这说明跳跃 demo 更像：

```text
MPC 给出接触力前馈
-> 通过接触 Jacobian 映射成关节力矩修正
-> 再交给 PVT
```

## 6. 和 walk_mpc_wbc 的关键区别

### 6.1 walk_mpc_wbc

walking demo 的 MPC 主要服务于：

```text
接触力规划 + base 预测 + WBC 前馈
```

它更偏向周期步态下的接触分配。

### 6.2 jump_mpc

jump demo 的 MPC 更偏向：

```text
起跳推力规划 + 腾空/落地阶段切换
```

也就是说，jump 里的 MPC 更像“推一把”，目标是让身体获得足够的向上速度，而不是维持周期走路。

## 7. 为什么跳跃更适合用状态机

因为跳跃不是平稳周期行为，而是强阶段性行为。

例如：

```text
准备阶段：把姿态和脚位置收紧
起跳阶段：给出向上的速度命令
上升阶段：关闭或减弱接触推力
下降阶段：准备触地
恢复阶段：重新进入支撑与平衡
```

所以 jump demo 先靠状态机决定控制策略，再在某些阶段打开 MPC。

这和 walking 的逻辑正相反：

```text
walking: 先有周期步态，再用 MPC/WBC 跟踪
jumping: 先有阶段状态机，再在阶段内部启用 MPC
```

## 8. 跳跃 demo 的核心输出

和 walking 一样，最后都要写回：

```text
RobotState.motors_pos_des
RobotState.motors_vel_des
RobotState.motors_tor_des
RobotState.motors_tor_out
```

但 jump demo 里还额外强调：

```text
RobotState.fe_react_tau_cmd
RobotState.js_pos_des
RobotState.js_vel_des
RobotState.js_eul_des
```

这些量会共同决定：

```text
起跳前怎么压姿态
起跳时给多大向上速度
腾空后怎么收姿态
落地时怎么重新恢复支撑
```

## 9. 当前源码里可先抓住的控制逻辑

从源码看，`jump_mpc.cpp` 至少有三层控制逻辑：

### 9.1 预备阶段

```cpp
if (simTime <= prepareTime) { ... }
```

目的：

```text
锁定当前足端，建立初始姿态和 IK 解
```

### 9.2 起跳前缓冲

```cpp
else if (simTime < startJumpingTime && simTime > prepareTime) { ... }
```

目的：

```text
慢慢把 base / 脚位置拉到跳跃前站姿
```

### 9.3 跳跃主阶段

```cpp
else if (simTime >= startJumpingTime) { ... }
```

目的：

```text
按 jump_state 切换起跳、上升、下降、恢复
```

## 10. 这一轮先得到什么结论

先把 `jump_mpc` 粗略记成一句话：

```text
它是一个状态机驱动的跳跃控制 demo，
通过 MPC 规划起跳/恢复阶段的接触力与 base 运动，
再借助 Jacobian 和 PVT 把结果变成电机 torque。
```

## 11. 后续阅读顺序

后面可以按这个顺序继续：

1. `jump_mpc.cpp` 预备阶段和起跳阶段的状态机切换
2. MPC 在跳跃里如何启用/关闭
3. `Jac_stand.transpose()` 为什么能把 wrench 变成关节力矩
4. `FLest / FRest` 是怎么估计的
5. PVT 如何把关节 torque 发给 MuJoCo

## 12. 面试一句话

如果要用一句话讲 jump demo，可以先这么说：

```text
我阅读并复盘了 OpenLoong 的 jump_mpc 跳跃 demo，理解了它如何通过跳跃状态机切换起跳、腾空和落地阶段，并结合 MPC、Jacobian 映射和 PVT 实现从接触力前馈到关节力矩输出的完整闭环。
```

