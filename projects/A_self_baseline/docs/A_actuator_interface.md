# A Actuator Interface

## 1. 定位

Actuator interface 回答一个问题：给定 q trajectory，如何进入 MuJoCo actuator tracking。

A07 是唯一规划 `data.ctrl` 和 `mujoco.mj_step` 控制闭环的层。当前 Step R-C 只保留 TODO skeleton。

## 2. ActuatorTrackingSpec

| 字段 | 含义 |
|---|---|
| enabled | 是否启用 actuator tracking |
| trajectory_source | q_traj 路径 |
| control_mode | position / velocity / torque |
| actuator_names | actuator 名称 |
| kp / kd | 后续 PD 参数 |
| metadata | 辅助信息 |

## 3. 与 A06 的边界

A06 只管理 target 和 viewer target。A06 可以做 kinematic preview 的规划，但不写 `data.ctrl`。

A07 才消费 `trajectory_source`，并在后续 R8 中实现 actuator tracking。

## 4. 与 mink 的关系

mink `arm_ur5e_actuators.py` 演示了 actuator 执行层。A 项目不调用 mink，而是在 `actuator_interface.py` 中保留学习型接口。

## 5. 验证标准

- trajectory_source 存在。
- q_traj shape 可解释。
- actuator names 与 `model.nu` 一致。
- ctrlrange violation 可记录。
- 不在 A04/A05/A06 中写 `data.ctrl`。

## 6. 当前不做

- 不执行 MuJoCo control loop。
- 不录 video。
- 不启动 viewer。
- 不实现真实 hardware control。
