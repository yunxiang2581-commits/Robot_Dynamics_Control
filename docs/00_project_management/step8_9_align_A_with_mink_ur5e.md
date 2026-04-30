# Step 8.9 Align A With Mink UR5e

## 1. 为什么做这次目标调整

原 A 项目主线偏向 H1 人形机器人和 Pinocchio/URDF 学习。H1 适合作为历史学习参考，但作为第一条完整 baseline，固定基 6-DOF 机械臂更容易形成可展示、可验证、可逐步实现的闭环。

因此 A 项目当前主线调整为：对标 `kevinzakka/mink` UR5e 示例的教学版 6-DOF 机械臂控制 baseline。

## 2. A 项目新目标

A 项目不是完整复刻 mink 库，也不是直接调用 mink 替代自己的实现。目标是逐步复现 mink UR5e 示例背后的核心链路：

- MuJoCo model inspect；
- configuration / site pose；
- site Jacobian；
- DLS differential IK；
- task + limit + QP-IK；
- target / mocap-style tracking；
- MuJoCo actuator tracking；
- optional collision avoidance TODO；
- comparison with mink。

## 3. A01-A09 新 Pipeline

```text
A00 reference and assets
  -> A01 model inspect
  -> A02 configuration / site pose
  -> A03 site Jacobian check
  -> A04 DLS differential IK
  -> A05 task + limit + QP-IK
  -> A06 target / mocap-style tracking
  -> A07 MuJoCo actuator tracking
  -> A08 collision avoidance TODO
  -> A09 comparison report
```

## 4. 与 mink 的对应关系

- A01 对应 MuJoCo model loading 和模型对象检查。
- A02 对应 `Configuration` 中从 `q` 查询 site/body pose。
- A03 对应 differential IK 所需的 site Jacobian。
- A04 对应 `solve_ik` 的最小无约束教学版。
- A05 对应 `FrameTask`、`PostureTask`、`ConfigurationLimit`、`VelocityLimit`。
- A06 对应 viewer target / mocap-style target。
- A07 对应 `arm_ur5e_actuators.py` 的 actuator tracking 思路。
- A08 对应 collision avoidance constraint 的后续 TODO。
- A09 对应与 mink 示例的逐项 comparison report。

## 5. 修改文件清单

- `projects/A_self_baseline/docs/A_pipeline_contract.md`
- `projects/A_self_baseline/README.md`
- `projects/A_self_baseline/docs/reference_mink_ur5e.md`
- `projects/A_self_baseline/docs/A_mink_alignment_plan.md`
- `projects/A_self_baseline/configs/robot.yaml`
- `projects/A_self_baseline/scripts/01_model_inspect.py`
- `projects/A_self_baseline/scripts/02_configuration_site_pose.py`
- `projects/A_self_baseline/scripts/03_site_jacobian_check.py`
- `projects/A_self_baseline/scripts/04_dls_differential_ik.py`
- `projects/A_self_baseline/scripts/05_task_limit_qp_ik.py`
- `projects/A_self_baseline/scripts/06_target_mocap_tracking.py`
- `projects/A_self_baseline/scripts/07_mujoco_actuator_tracking.py`
- `docs/00_project_management/step8_9_align_A_with_mink_ur5e.md`

## 6. 未实现算法说明

本步骤只修改 Markdown、配置模板、脚本 docstring 和 TODO 说明。未实现 FK、Jacobian、IK、QP、WBC、MuJoCo 控制、RL 或 collision avoidance。

## 7. H1 Legacy 的新定位

H1 legacy 是历史学习参考，继续保留在 `legacy_imported/` 中。它记录早期 Pinocchio/URDF/FK/Jacobian/IK 学习路径，但不再是当前 A 项目主线。

## 8. 下一步

下一步进入 A01 model inspect：先确认 UR5e/MJCF 模型资产路径，再最小实现模型检查报告。

## 9. 验收清单

- [ ] A 项目 README 明确 UR5e / mink-style 当前主线。
- [ ] pipeline 契约包含 A00-A09。
- [ ] `robot.yaml` 使用 UR5e/MJCF 配置模板。
- [ ] A01-A07 docstring 与 TODO 对齐 mink 概念。
- [ ] 未修改 legacy_imported。
- [ ] 未实现任何控制算法。
- [ ] `py_compile` 通过。
