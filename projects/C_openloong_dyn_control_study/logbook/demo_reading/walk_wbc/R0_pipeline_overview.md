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
- `R5`：未开始，`JoyStickInterpreter / GaitScheduler / FootPlacement`
- `R6`：未开始，`WBC_priority::computeDdq()`
- `R7`：未开始，`WBC_priority::computeTau()`
- `R8`：未开始，`PVT_Ctr -> motors_tor_out`
- `R9`：未开始，闭环总复盘

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

### 8.1 本轮将回答的问题

```text
期望速度如何变成 motionState、legState、接触状态和摆动脚目标？
```

### 8.2 预期数据流

```text
desired base command
-> JoyStickInterpreter
-> GaitScheduler
-> FootPlacement
-> RobotState.motionState / legState / swing_fe_pos_des_W / swing_fe_rpy_des_W
```

### 8.3 这一层的作用

这一层不解动力学，也不求扭矩。它负责把“我要怎么走”翻译成 WBC 可执行的任务目标。

## 9. R6：WBC computeDdq()

### 9.1 本轮将回答的问题

```text
walking / standing 任务如何变成期望 delta_q / dq / ddq？
```

### 9.2 预期数学关系

任务空间误差一般会组织成：

```text
errX
derrX
ddxDes
```

再利用：

```text
x_ddot = J ddq + dJ dq
```

构造任务优先级求解，得到：

```text
delta_q_final_kin
dq_final_kin
ddq_final_kin
```

## 10. R7：WBC computeTau()

### 10.1 本轮将回答的问题

```text
已知期望 ddq，如何求接触力和关节力矩？
```

### 10.2 预期数学关系

当前版本最关键的动力学等式是：

```text
M ddq + Non = J_c^T F + S^T tau
```

这里：

- `M = dyn_M`
- `Non = dyn_Non`
- `J_c` 由支撑脚 Jacobian 组成
- `F` 是接触力
- `tau` 是关节力矩

同时还会叠加：

- 接触摩擦锥约束
- 法向力上下界
- 足端力矩界

### 10.3 当前代码现状

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

### 11.2 预期数据流

```text
wbc_delta_q_final / wbc_dq_final / wbc_tauJointRes
-> integrateDIY()
-> motors_pos_des / motors_vel_des / motors_tor_des
-> PVT_Ctr
-> motors_tor_out
```

### 11.3 这一层的物理意义

这一层不是再做全身任务分配，而是把 WBC 给出的期望关节量整理成更稳定、可执行的关节力矩输出。

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

## 13. 当前最重要的复习抓手

如果你现在只想保留 5 句话，建议保留这 5 句：

1. `walk_wbc` 是一个完整控制闭环装配文件，不是单一算法文件。
2. `StateEst` 决定了 floating-base `q/dq` 中 base 部分最终用什么状态。
3. `Pin_KinDyn` 把 `q/dq` 变成 `J / dJ / pose / dyn_* / CoM` 这些控制器真正需要的模型量。
4. `WBC` 当前真实依赖的是 `J_*`、`dJ_*`、`dyn_M`、`dyn_Non`、`Jcom_W`、`pCoM_W`，而不是质心动量矩阵 `dyn_Ag / dyn_dAg`。
5. `PVT_Ctr` 把全身层结果转成 MuJoCo 下一步真正执行的电机力矩。

## 14. 下一步建议

当前最合适进入的是第五轮：

```text
JoyStickInterpreter
-> GaitScheduler
-> FootPlacement
```

目标不是马上推公式，而是先回答：

```text
RobotState 里哪些 walking task 字段，是这一层开始写进去的？
```
