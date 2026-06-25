# jump_mpc 第一轮阅读笔记

## 1. 本轮目标

只读 `external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp` 的主流程。

本轮回答一个核心问题：

```text
jump_mpc 每个仿真步怎样从 MuJoCo 状态走到跳跃状态机、MPC、PVT 力矩，再写回 MuJoCo？
```

## 2. 本轮边界

- 只读 `jump_mpc.cpp`。
- 不展开 `MPC::cal()`、`Pin_KinDyn`、`MJ_Interface`、`PVT_Ctr` 内部实现。
- 不细拆每个 `jump_state` 的控制公式，下一轮 R2 专门读状态机。
- 不运行 GUI，不修改 `external/` 源码。

## 3. 文件角色

`jump_mpc.cpp` 是跳跃 demo 的顶层控制装配入口。

它把这些模块串起来：

- MuJoCo 仿真
- `MJ_Interface` 状态读写
- `Pin_KinDyn` 运动学和动力学更新
- `jump_state` 跳跃状态机
- `MPC` 接触力前馈
- `Jac_stand.transpose()` 力到关节力矩映射
- `PVT_Ctr` 关节控制
- 日志与可视化

一句话总结：

```text
MuJoCo step
-> 状态读入 RobotState
-> Pin_KinDyn 更新 J / dJ / 动力学
-> 按 simTime 和 jump_state 生成跳跃目标
-> MPC 求接触力
-> 接触力通过 Jacobian 映射成腿部关节力矩
-> PVT 生成最终 motors_tor_out
-> torque 写回 MuJoCo
```

## 3.1 主流程摘要

`jump_mpc` 没有像 `walk_wbc` 那样使用完整的：

```text
StateEst -> GaitScheduler -> FootPlacement -> WBC_priority
```

它更直接：

```text
状态机 + IK + MPC + Jacobian torque injection + PVT
```

也就是说，这个 demo 的重点不是连续步态调度，而是让机器人按几个跳跃阶段执行一段预设动作。

## 4. main() 中创建的模块

`main()` 一开始实例化：

- `UIctr uiController`
- `MJ_Interface mj_interface`
- `Pin_KinDyn kinDynSolver`
- `MPC mpc_force`
- `PVT_Ctr pvtCtr`
- `DataBus RobotState`
- `DataLogger logger`

当前理解：

- `RobotState` 仍然是全局共享状态总线。
- `mpc_force` 只在某些跳跃阶段启用。
- `pvtCtr` 始终负责生成最终电机力矩。
- 本 demo 没有显式创建 `WBC_priority`，所以接触力到关节力矩是用 `Jac_stand.transpose()` 直接做的。

## 5. 初始化姿态和跳跃参数

主循环前先设置：

- 双脚目标位置 `fe_l_pos_L_des / fe_r_pos_L_des`
- 双脚目标姿态 `fe_l_rot_des / fe_r_rot_des`
- 双手目标位置和姿态
- 腿部 IK：`computeInK_Leg(...)`
- 手臂 IK：`computeInK_Hand(...)`

关键跳跃时间和状态：

```cpp
double startJumpingTime = 8.5;
double prepareTime = 3;
uint16_t jump_state = 0;
double stand_z = -0.8;
double jump_z = 0.2;
double simEndTime = 13;
```

当前理解：

- `prepareTime = 3`：前 3 秒用于准备姿态。
- `startJumpingTime = 8.5`：8.5 秒后进入真正跳跃状态机。
- `jump_state`：跳跃阶段编号。
- `stand_z`：起跳前双脚相对 base 的站立高度目标。
- `jump_z`：期望跳跃高度，用来计算起跳速度。

## 6. 双层循环结构

主循环仍然是 MuJoCo demo 常见的双层结构：

```text
外层：窗口没有关闭时持续运行
内层：每个显示帧内推进多个 MuJoCo 仿真小步
```

源码结构：

```cpp
while (!glfwWindowShouldClose(uiController.window)) {
    simstart = mj_data->time;
    while (mj_data->time - simstart < 1.0 / 60.0 && uiController.runSim) {
        ...
    }
    uiController.updateScene();
}
```

当前理解：

- `mj_model->opt.timestep` 是仿真的真实积分步长。
- `1.0 / 60.0` 是画面更新节奏。
- 一个渲染周期里可能执行多个控制/仿真小步。

## 7. 每个仿真步的主调用链

### 7.1 MuJoCo 先推进一步

```cpp
mj_step(mj_model, mj_data);
simTime = mj_data->time;
```

当前理解：

- 当前 `mj_step()` 用的是上一轮写入的 `mj_data->ctrl`。
- 这是标准离散闭环：

```text
u_k -> simulator -> x_{k+1} -> controller -> u_{k+1}
```

### 7.2 从 MuJoCo 读状态到 RobotState

```cpp
mj_interface.updateSensorValues();
mj_interface.dataBusWrite(RobotState);
```

当前理解：

- `MJ_Interface` 从 `mj_data` 和 sensor 里读取当前状态。
- 然后把关节、base、传感器等信息写入 `RobotState`。
- 这一轮先不展开 `MJ_Interface` 内部。

### 7.3 Pin_KinDyn 更新运动学和动力学

```cpp
kinDynSolver.dataBusRead(RobotState);
kinDynSolver.computeJ_dJ();
kinDynSolver.computeDyn();
kinDynSolver.dataBusWrite(RobotState);
```

当前理解：

- `computeJ_dJ()` 计算左右脚 Jacobian 和 `dJ`。
- `computeDyn()` 计算质量矩阵、非线性项等动力学量。
- 后面的 `Jac_stand`、`FLest/FRest`、MPC 输入都依赖这些模型量。

### 7.4 构造站立接触 Jacobian

源码把左右脚 Jacobian 的最后 12 个关节列取出来：

```cpp
Jac_stand.block(0, 0, 6, 12) = RobotState.J_l.block(0, model_nv - 12, 6, 12);
Jac_stand.block(6, 0, 6, 12) = RobotState.J_r.block(0, model_nv - 12, 6, 12);
```

所以：

```text
Jac_stand in R^{12 x 12}
```

它只关心双腿最后 12 个关节，因为后面要把 12 维足端 wrench 映射成 12 维腿部关节力矩。

本轮先记住公式：

```text
tau_leg = - J_stand^T F_fe
```

完整解释放到 R4。

### 7.5 估计左右脚接触力

源码根据当前关节力矩和动力学项反推：

```cpp
FLest
FRest
```

当前理解：

- `FLest`：左脚接触力估计。
- `FRest`：右脚接触力估计。
- 它们在 `jump_state == 4` 下降阶段用来判断是否触地。

关键判断：

```cpp
if (FLest(2) > 1000 && FRest(2) > 1000) {
    jump_state = 5;
}
```

也就是：

```text
左右脚竖直反力都足够大 -> 认为已经落地 -> 进入恢复阶段
```

### 7.6 按时间和 jump_state 生成目标

主流程先按时间分三大段：

```cpp
if (simTime <= prepareTime) {
    ...
} else if (simTime < startJumpingTime && simTime > prepareTime) {
    ...
} else if (simTime >= startJumpingTime) {
    ...
}
```

对应：

```text
0 ~ 3s       准备阶段
3s ~ 8.5s    起跳前站姿缓冲
8.5s 后      跳跃状态机
```

真正的跳跃阶段再由：

```cpp
jump_state == 0
jump_state == 3
jump_state == 4
jump_state == 5
```

继续细分。

本轮先不拆每个状态内部逻辑，R2 专门读。

### 7.7 MPC 统一在状态机之后求解

无论当前 MPC 是否启用，源码每一小步都会调用：

```cpp
mpc_force.dataBusRead(RobotState);
mpc_force.cal();
mpc_force.dataBusWrite(RobotState);
```

但真正是否计算有效输出，要看：

```cpp
mpc_force.enable();
mpc_force.disable();
mpc_force.get_ENA();
```

当前理解：

- `jump_state == 0` 起跳推力阶段：MPC 启用。
- `jump_state == 3` 上升阶段：MPC 关闭。
- `jump_state == 4` 下降阶段：MPC 关闭。
- `jump_state == 5` 落地恢复阶段：MPC 再次启用。

所以 jump demo 的 MPC 不是一直主导控制，而是在关键接触阶段提供接触力前馈。

### 7.8 MPC 输出映射成腿部关节力矩

如果 MPC 启用：

```cpp
Uje = Jac_stand.transpose() * (-1.0) * RobotState.fe_react_tau_cmd.block<nu - 1, 1>(0, 0);
```

对应：

```text
F_fe = RobotState.fe_react_tau_cmd[0:12]
Uje  = - J_stand^T F_fe
```

其中：

```text
F_fe in R^{12}
Uje  in R^{12}
```

随后源码会做关节力矩限幅：

```cpp
Limit(Uje(i), jTor_max[i], jTor_min[i]);
Limit(Uje(i + 6), jTor_max[i], jTor_min[i]);
```

然后把这些力矩写入最后 12 个腿部关节：

```cpp
pvtCtr.disablePV(model_nv - 6 - 12 + i);
RobotState.motors_tor_des[model_nv - 6 - 12 + i] = Uje(i);
```

当前理解：

- MPC 输出的是足端接触力。
- 代码直接把它映射成腿关节力矩。
- 对这些腿部关节，PVT 的位置/速度项会被关闭，让 torque 前馈直接起作用。

### 7.9 PVT 生成最终电机力矩

之后统一进入：

```cpp
pvtCtr.dataBusRead(RobotState);
if (simTime <= startJumpingTime) {
    pvtCtr.calMotorsPVT(110.0 / 1000.0 / 180.0 * 3.1415);
} else {
    pvtCtr.calMotorsPVT();
}
pvtCtr.dataBusWrite(RobotState);
```

当前理解：

- 起跳前用一个额外参数调用 `calMotorsPVT(...)`，像是更柔和的启动控制。
- 起跳后用默认 PVT。
- 最终输出写入 `RobotState.motors_tor_out`。

### 7.10 写回 MuJoCo 和日志

最终写回：

```cpp
mj_interface.setMotorsTorque(RobotState.motors_tor_out);
```

日志记录：

- `simTime`
- `motor_pos_des`
- `motor_pos_cur`
- `motor_vel_cur`
- `motor_tor_des`
- `motor_tor_out`
- `rpyVal`
- `gpsVal`
- `fe_l_pos_L_des`
- `fe_r_pos_L_des`
- `fe_l_pos_W`
- `fe_r_pos_W`
- `Ufe`

当前理解：

- `motor_tor_des` 是控制器希望的关节力矩。
- `motor_tor_out` 是 PVT 整形后的最终电机力矩。
- `Ufe` 记录的是 MPC 产生的足端 wrench 前馈。

## 8. 第一轮阅读结论

`jump_mpc` 的主流程可以先记成：

```text
MuJoCo
-> MJ_Interface
-> RobotState
-> Pin_KinDyn
-> jump_state 目标生成
-> MPC 接触力
-> -J^T F 映射腿部关节力矩
-> PVT_Ctr
-> motors_tor_out
-> MuJoCo
```

和 `walk_wbc` 的最大区别：

```text
walk_wbc:  gait/task -> WBC -> PVT
jump_mpc:  state machine -> MPC force -> Jacobian torque injection -> PVT
```

也就是说，jump demo 更像一个“状态机驱动的动作 demo”，而不是完整通用的全身控制框架。

## 9. 当前仍未展开的问题

后续还要继续拆：

- `jump_state == 0/3/4/5` 每个阶段到底设置了什么目标。
- `jump_vel_des` 和 `jump_acc_t` 如何决定起跳速度。
- 为什么 MPC 在起跳和落地恢复阶段启用，在腾空阶段关闭。
- `FLest / FRest` 的触地判断公式。
- `Jac_stand.transpose()` 映射力矩的符号和维度。

## 10. 建议的第二轮阅读顺序

R2 建议专门读：

```text
prepareTime / startJumpingTime / jump_state
```

重点回答：

```text
jump_mpc 是怎样从准备站立，切到起跳、上升、下降、落地恢复的？
```
