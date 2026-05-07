# Step R-F A04-A07 Full TODO Tasks

## 1. 为什么要生成完整 TODO

Step R-F 的目标是让 A04/A05/A06/A07 wrapper 成为完整功能路线图。当前不实现算法，但必须把未来功能拆成可验证、可逐步补全的任务。

## 2. 为什么不是简化版

本步骤不是只写“TODO: 实现 IK”。每个 TODO 都包含要做什么、为什么、mink 对标概念、公式或数据流、输入、输出、推荐 API、验证标准和常见错误。

## 3. A04 TODO 总览

A04 覆盖：

- 前置产物读取。
- TargetDefinition 解析。
- MuJoCo model/data 加载。
- position-only DLS。
- pose_6d DLS。
- damping 稳定性。
- q integration。
- 终止条件。
- trajectory/log/figure/report。
- A04 边界。

## 4. A05 TODO 总览

A05 覆盖：

- target_definition_json。
- FrameTask。
- PostureTask。
- QP objective。
- VelocityLimit。
- JointPositionLimit / ConfigurationLimit。
- SciPy solver。
- OSQP solver。
- QP-IK outer loop。
- constraint log。
- full outputs。
- A05 边界。

## 5. A06 TODO 总览

A06 覆盖：

- fixed_pose。
- pose_sequence。
- mocap_placeholder。
- interactive_viewer_todo。
- derived MJCF planning。
- model.nmocap check。
- mouse drag target。
- keyboard movement。
- target pose feeds IK。
- kinematic IK follow。
- target logs/reports。
- A06 边界。

## 6. A07 TODO 总览

A07 覆盖：

- trajectory_source。
- actuator inspect。
- ctrl range。
- position actuator tracking。
- PD tracking。
- MuJoCo control loop future plan。
- tracking log。
- tracking figure。
- video placeholder。
- report。
- A07 边界。

## 7. 与 mink example 的对应关系

```text
mocap target
  -> FrameTask / PostureTask
  -> ConfigurationLimit / VelocityLimit / CollisionAvoidanceLimit
  -> solve_ik
  -> viewer.sync
  -> data.ctrl / mj_step
```

A 项目映射：

- A06 Target + Viewer interface: mocap target / viewer target。
- A04/A05 IK interface: FrameTask / PostureTask / solve_ik。
- A05 IK interface: VelocityLimit / ConfigurationLimit。
- A07 Actuator interface: data.ctrl / mj_step future location。

## 8. 当前未实现内容

- DLS / QP solver。
- viewer drag。
- keyboard movement。
- derived MJCF。
- kinematic IK follow。
- actuator tracking。
- video generation。

## 9. 后续 R1-R10

1. R1 TargetDefinition load/save/validate。
2. R2 A05 target_definition_json。
3. R3 A06 fixed_pose / pose_sequence。
4. R4 A06 mocap capability。
5. R5 A06 derived MJCF。
6. R6 A06 keyboard movement。
7. R7 A06 mouse drag。
8. R8 A06 kinematic IK follow。
9. R9 A07 actuator tracking。
10. R10 A10 video demo。

## 10. 验收清单

- [x] A04 TODO 覆盖 position / pose_6d DLS。
- [x] A05 TODO 覆盖 FrameTask / PostureTask / limits / SciPy / OSQP。
- [x] A06 TODO 覆盖 fixed_pose / pose_sequence / mocap / viewer / mouse / keyboard。
- [x] A07 TODO 覆盖 trajectory / actuator / ctrl range / PD / log / figure / video placeholder。
- [x] 未调用 mink。
- [x] 未写 data.ctrl。
- [x] 未启动 viewer。
- [x] 未生成 video。
- [ ] py_compile 通过。
