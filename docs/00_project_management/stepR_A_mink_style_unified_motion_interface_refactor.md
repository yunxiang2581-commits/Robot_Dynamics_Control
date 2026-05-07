# Step R-A - Mink-Style Unified Motion Interface Refactor

## 1. 为什么做这一步

A04/A05/A06/A07 已经开始出现重复 target、IK、trajectory 和边界说明。Step R-A 先建立统一接口，让后续实现沿着 `TargetDefinition -> IkRequest -> IkResult -> ViewerTarget TODO -> ActuatorTrackingSpec` 推进。

## 2. 本步骤只做 TODO skeleton

本步骤不运行完整 IK、不启动 viewer、不写 `data.ctrl`、不录 video、不调用 mink。A04/A05 旧的完整脚本逻辑被折叠为后续要迁移到 `ik_interface.py` 的 TODO。

## 3. 与 mink 的关系

mink example 的链路是：

```text
mocap target -> FrameTask/PostureTask -> limits -> solve_ik -> viewer -> data.ctrl
```

A 项目学习这个结构，但不直接调用 mink。A06 负责 target，A04/A05 负责 IK，A07 负责 actuator executor。

## 4. 修改文件清单

- `src/robot_baseline/motion_types.py`
- `src/robot_baseline/target_interface.py`
- `src/robot_baseline/ik_interface.py`
- `src/robot_baseline/trajectory_io.py`
- `src/robot_baseline/viewer_target_todo.py`
- `src/robot_baseline/actuator_interface.py`
- `scripts/04_dls_differential_ik.py`
- `scripts/05_task_limit_qp_ik.py`
- `scripts/06_target_mocap_tracking.py`
- `scripts/07_mujoco_actuator_tracking.py`
- `configs/motion_task.yaml`
- `configs/target_tracking.yaml`
- pipeline README/docs

## 5. 未实现内容

- 完整 DLS trajectory solver。
- 完整 QP solver dispatch。
- A06 target definition 真实输出。
- 派生 MJCF。
- viewer mouse/keyboard target。
- kinematic IK follow。
- A07 actuator tracking。
- video recording。

## 6. 下一步

1. 把 A04 DLS 已验证 helper 迁移到 `ik_interface.solve_ik_step`。
2. 把 A05 QP objective、limit 和 solver 迁移到 `ik_interface`。
3. 实现 A06 target definition 输出。
4. 再进入 viewer target TODO 的分步实现。
5. 最后进入 A07 actuator tracking。

## 7. 验收清单

- [x] 新增统一 motion dataclass。
- [x] 新增 target interface TODO skeleton。
- [x] 新增 IK interface TODO skeleton。
- [x] 新增 trajectory IO skeleton。
- [x] 新增 viewer target TODO skeleton。
- [x] 新增 actuator interface TODO skeleton。
- [x] A04/A05/A06/A07 变成薄 wrapper。
- [x] 未调用 mink。
- [x] 未写 data.ctrl。
- [x] 未启动 viewer。
- [x] 未生成 video。
- [x] 未修改 external/mink_upstream。
- [x] 未修改 legacy_imported。
- [x] py_compile 通过。
