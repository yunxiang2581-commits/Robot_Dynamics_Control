# C05B walk_wbc 主循环源码追踪

## 1. 学习目标

本轮继续 Project C 的学习，重点从 `wbc_speed_test` 进入 `walk_wbc`：

- `wbc_speed_test` 已证明 WBC / PVT 数值链路可以在固定输入下跑通。
- `walk_wbc` 需要回答更关键的问题：这些模块如何接入 MuJoCo，形成传感器输入 -> 状态估计 -> Pinocchio 动力学 -> WBC -> PVT -> 电机力矩 -> MuJoCo 下一步的闭环。
- 这一步连接 Project C 后续的 simplified WBC demo，因为我们需要先知道官方闭环里哪些变量是真正的控制输入、控制输出和可记录指标。

## 2. 输入、输出与数学逻辑

输入：

- MuJoCo XML：`../models/scene_board.xml`
- Pinocchio URDF：`../models/AzureLoong.urdf`
- 当前仿真状态：`mj_data->qpos`、`mj_data->qvel`、传感器 quaternion / gyro / acc
- 用户速度目标：`xv_des = 0.7`
- 步态时间参数：`GaitScheduler(0.4, timestep)`

输出：

- `RobotState.motors_tor_out`
- `mj_data->ctrl[i]`
- `record/datalog.log`
- viewer 渲染画面

数学逻辑是否变化：

- 相对 C05A，WBC、PVT、Pin_KinDyn 的核心算法入口没有变化。
- 变化的是数据来源和闭环方向：C05A 使用固定采样状态；C05B 的 `walk_wbc` 使用 MuJoCo 每一步仿真状态，并把 PVT 输出力矩写回 MuJoCo。

改动参数：

- 本轮只读源码，不修改参数。

主要风险：

- GUI / OpenGL / X11 环境可能导致运行验证失败。
- `MJ_Interface::dataBusWrite()` 中 base position / base linear velocity 写入被注释，真实闭环依赖 `StateEst` 补充估计值。
- `fL/fR` 来自 `MJ_Interface::f3d`，但当前追踪还未看到接触力填充路径，需要继续确认。
- WBC 的 `qp_status`、`wbc_tauJointRes`、`wbc_FrRes` 没有进入 `walk_wbc` 的 datalog。

## 3. walk_wbc 主循环顺序

源码入口：

```text
external/open_source_repos/OpenLoong-Dyn-Control/demo/walk_wbc.cpp
```

主循环可以按下面顺序理解：

```text
MuJoCo mj_step
-> MJ_Interface.updateSensorValues
-> MJ_Interface.dataBusWrite
-> StateEst init / set / update / get
-> Pin_KinDyn dataBusRead / computeJ_dJ / computeDyn / dataBusWrite
-> StateEst force update
-> JoyStickInterpreter desired command
-> GaitScheduler contact schedule
-> FootPlacement swing foot target
-> WBC_priority computeDdq / computeTau
-> WBC output converted to desired joint pos / vel / torque
-> PVT_Ctr calMotorsPVT
-> MJ_Interface.setMotorsTorque
-> DataLogger record
-> UI scene update
```

关键区别在最后两步：

- `walk_wbc` 会调用 `mj_interface.setMotorsTorque(RobotState.motors_tor_out)`。
- `wbc_speed_test` 只计算并记录耗时，不把力矩反馈给 MuJoCo。

## 4. 模块角色解释

### MJ_Interface

`MJ_Interface` 是 MuJoCo 和 `DataBus` 之间的桥：

- 从 MuJoCo 读取关节位置和速度。
- 从传感器读取 base quaternion、gyro、acc。
- 把 quaternion 转成 RPY。
- 调用 `busIn.updateQ()`，得到 Pinocchio 需要的浮动基 `q` / `dq`。
- 把 PVT 最终输出的关节力矩写入 `mj_data->ctrl`。

注意：当前源码中 `basePos` 和 `baseLinVel` 写入 `DataBus` 的语句被注释，所以 `walk_wbc` 不是单纯依赖 MuJoCo 位置速度直写，而是还经过 `StateEst`。

### DataBus

`DataBus` 是官方控制链的共享状态对象。它把下面几类信息放在同一个结构里：

- 传感器与当前状态：`q`、`dq`、`rpy`、关节状态、base 状态。
- Pinocchio 结果：Jacobian、frame pose、动力学矩阵、重力项和非线性项。
- 指令：joystick command、MPC command、WBC desired acceleration / velocity / delta q。
- WBC 结果：`wbc_delta_q_final`、`wbc_dq_final`、`wbc_tauJointRes`、`wbc_FrRes`、`qp_status`。
- PVT 输出：`motors_tor_out`。

这和本仓库主学习线一致：

```text
URDF -> FK -> Jacobian -> dynamics -> WBC/QP -> actuator command
```

### Pin_KinDyn

`Pin_KinDyn` 对应 Pinocchio 学习里的模型和动力学计算：

- 读取 `RobotState.q` / `RobotState.dq`。
- 计算 frame 位姿、Jacobian、`dJ`。
- 计算质量矩阵、重力项、非线性项。
- 写回 `DataBus`，供 WBC 使用。

### WBC_priority

`WBC_priority` 分两步：

- `computeDdq()`：根据任务优先级求期望 `delta_q`、`dq`、`ddq`。
- `computeTau()`：根据动力学、接触约束和期望加速度求关节力矩与接触力。

这些结果写回：

```text
wbc_delta_q_final
wbc_dq_final
wbc_ddq_final
wbc_tauJointRes
wbc_FrRes
qp_status
```

### PVT_Ctr

`PVT_Ctr` 是执行层：

- 读取期望关节位置、速度和前馈力矩。
- 根据关节 PD 增益计算最终输出力矩。
- 写出 `motors_tor_out`。

在 `walk_wbc` 中，PVT 输出会真正进入 MuJoCo 控制数组；这是 C05A benchmark 没有验证的闭环关键点。

## 5. 和 C05A 的对照结论

| 项目 | C05A wbc_speed_test | C05B walk_wbc |
| --- | --- | --- |
| MuJoCo viewer | 无 | 有 |
| MuJoCo `mj_step` | 无 | 有 |
| 状态输入 | 固定 sample state | MuJoCo sensor + StateEst |
| Pin_KinDyn | 有 | 有 |
| GaitScheduler | 有，但仅 benchmark 语义 | 有，接入仿真时间 |
| FootPlacement | 有 | 有 |
| WBC computeDdq / computeTau | 有 | 有 |
| WBC 输出转关节命令 | 注释掉 | 启用 |
| PVT 输出写回 MuJoCo | 无 | 有 |
| datalog | runtime + 基础状态 | simTime + 基础状态 |
| 可代表完整闭环 | 否 | 是，需运行验证 |

## 6. 对后续 Project C 的意义

后续要实现 `C01_contact_force_allocation_demo` 和 `C02_simplified_wbc_qp_balance_demo`，不能只复刻 C05A 的固定输入 benchmark，而应抽象 C05B 的闭环结构：

```text
state feedback
-> kinematics/dynamics update
-> desired base/contact task
-> WBC/QP solve
-> joint torque command
-> simulator step
-> metrics/logging
```

因此，下一步建议是：

1. 做 `C05C MJ_Interface / StateEst / contact force trace`，确认 `fL/fR` 和 base state 在真实闭环中的来源。
2. 再做 `C06 walk_wbc GUI/runtime observation`，规划 X11/OpenGL/截图或录屏，不直接跳到 `walk_mpc_wbc`。
3. 最后把 C05B/C05C 的变量抽象成 Project C 自己的 simplified simulator 接口。

## 7. 边界记录

本轮只读源码并生成学习记录：

- 未修改 `external/open_source_repos/OpenLoong-Dyn-Control/`。
- 未运行 `walk_wbc`。
- 未运行 `walk_mpc_wbc`。
- 未打开 MuJoCo GUI。
- 未修改 WBC / PVT / MuJoCo 参数。
- 未生成视频。
