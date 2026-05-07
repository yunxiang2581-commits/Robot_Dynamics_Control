# A Four Interface Refactor Plan

## 1. 为什么参考 mink example

mink 的 UR5e 示例展示了一条清晰的数据流：

```text
mocap target -> FrameTask/PostureTask -> limits -> solve_ik -> viewer -> data.ctrl
```

A 项目不直接调用 mink，而是学习它背后的结构，把功能拆成四个接口，便于逐步实现和验证。

## 2. 四个接口

| 接口 | 负责什么 | 对应脚本 |
|---|---|---|
| Target interface | 目标从哪里来，如何表达 target pose | A06 |
| IK interface | 如何由 target 求 q_traj | A04 / A05 |
| Viewer interface | 如何规划 viewer target / mocap target | A06 |
| Actuator interface | 如何由 q_traj 进入 actuator tracking | A07 |

## 3. 统一 schema

核心 schema 位于 `src/robot_baseline/motion_types.py`：

- `TargetDefinition`: target pose 与来源。
- `IkRequest`: A04/A05 的统一 IK 请求。
- `IkResult`: IK 输出、误差日志和轨迹来源。
- `TrajectorySource`: A06/A07 之间的轨迹来源记录。
- `ViewerTargetSpec`: viewer/mocap target 规划。
- `ActuatorTrackingSpec`: A07 actuator tracking 规划。

四元数统一使用 `wxyz`。如果后续使用 SciPy `Rotation.from_quat`，需要转换成 `xyzw`。

## 4. A04/A05/A06/A07 映射

- A04 = `solver_type=dls` 的 IK wrapper。
- A05 = `solver_type=qp_scipy` / `qp_osqp` 的 IK wrapper。
- A06 = Target interface + Viewer interface wrapper。
- A07 = Actuator interface wrapper。

## 5. 当前只保留 TODO 的功能

- DLS 单步求解。
- QP-IK objective、bounds、solver backend。
- TargetDefinition 完整 load/save/validate。
- A06 fixed target 最小实现。
- 派生 MJCF。
- keyboard target movement。
- mouse drag target validation。
- kinematic IK follow。
- actuator tracking。
- video demo。

## 6. 后续实现顺序

1. R1: TargetDefinition load/save/validate。
2. R2: A05 target_definition_json。
3. R3: A06 fixed_pose / pose_sequence。
4. R4: A06 mocap capability。
5. R5: A06 derived MJCF。
6. R6: A06 keyboard movement。
7. R7: A06 mouse drag。
8. R8: A06 kinematic IK follow。
9. R9: A07 actuator tracking。
10. R10: A10 video demo。

## 7. Step R-F 完整 TODO skeleton

A04/A05/A06/A07 当前是完整功能版 TODO skeleton，不是简化版。当前入口：

- A04: `scripts/04_ik_dls_wrapper.py`
- A05: `scripts/05_ik_qp_wrapper.py`
- A06: `scripts/06_target_viewer_wrapper.py`
- A07: `scripts/07_actuator_wrapper.py`

对应完整计划文档：

- `A04_DLS_IK_TODO_full_plan.md`
- `A05_QP_IK_TODO_full_plan.md`
- `A06_Target_Viewer_TODO_full_plan.md`
- `A07_Actuator_TODO_full_plan.md`
