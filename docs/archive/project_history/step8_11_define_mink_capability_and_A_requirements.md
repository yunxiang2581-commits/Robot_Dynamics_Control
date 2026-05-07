# Step 8.11 Define Mink Capability And A Requirements

## 1. 为什么需要这份需求文档

A 项目已经从 H1 历史学习参考转向对标 mink UR5e 示例的教学版 6-DOF 机械臂 baseline。为了避免后续实现范围发散，需要先明确 mink 能力、A 项目必须实现的内容、暂缓内容和最终展示物。

## 2. mink 能力拆解摘要

mink 的核心能力是：给定机器人当前 configuration 和一组 task-space objectives，在 respecting limits/constraints 的前提下，求一个局部最优 joint velocity。

重点能力包括：

- MuJoCo robot model loading；
- Configuration abstraction；
- Task abstraction；
- Limit / constraint abstraction；
- QP-based differential IK；
- UR5e examples；
- actuator tracking example；
- optional collision avoidance；
- viewer / target / mocap-style interaction。

## 3. A 项目需要实现的范围

A 项目需要自己实现最小模型检查、site pose、site Jacobian、DLS IK、最小 QP-IK、target tracking、actuator tracking、报告/日志/轨迹/视频输出和与 mink 的对比报告。

A 项目不完整复制 mink 源码，不直接调用 mink 替代自己的实现，不追求完整 API、多机器人 examples 或库级测试覆盖。

## 4. A00-A10 Pipeline

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
  -> A10 demo showcase / video recording
```

## 5. 最终视频 Demo 交付物

主展示目标是：

```text
outputs/videos/A07_ur5e_actuator_tracking_demo.mp4
```

它应配套 tracking log、tracking error figure、report，并可在 README 或 GitHub Release 中展示。

可选 demo：

- `A04_dls_ik_target_tracking_demo.mp4`
- `A05_qp_ik_joint_limit_demo.mp4`

## 6. 修改文件清单

- `projects/A_self_baseline/docs/mink_capability_vs_A_requirements.md`
- `projects/A_self_baseline/README.md`
- `projects/A_self_baseline/docs/A_pipeline_contract.md`
- `projects/A_self_baseline/docs/A_mink_alignment_plan.md`
- `projects/A_self_baseline/docs/reference_mink_ur5e.md`
- `docs/00_project_management/step8_11_define_mink_capability_and_A_requirements.md`

## 7. 未实现算法说明

本步骤只修改 Markdown 文档和 README 引用。未实现 FK、Jacobian、IK、QP、WBC、MuJoCo 控制或视频录制代码。

## 8. 下一步

Step 9C 先统一 A00-A10 脚本入口命名和 TODO 骨架。之后进入 A01 model inspect 最小实现，读取 `shared/robot_assets/models/mink_universal_robots_ur5e/scene.xml` 并输出模型对象摘要。

## 9. 验收清单

- [x] 创建 mink 能力与 A 需求总文档。
- [x] README 链接到需求总文档。
- [x] pipeline contract 包含 A10 demo showcase / video recording。
- [x] alignment plan 增加阶段 8。
- [x] reference 文档说明两个文档的分工。
- [x] 未修改 Python 代码。
- [x] 未修改 `external/mink_upstream`。

## 10. 当前项目状态

Step 8.11 已完成。A 项目已具备进入实现阶段的文档条件：

- A 当前主线：mink-style UR5e 6-DOF manipulator control baseline；
- 当前实现入口：A00-A10 mink-style TODO pipeline，其中下一步实现入口是 A01 model inspect；
- 当前模型资产：`shared/robot_assets/models/mink_universal_robots_ur5e/scene.xml`；
- 下一步任务：实现 A01 最小可运行 MJCF inspect，输出模型维度和对象清单；
- 仍不做：FK、Jacobian、IK、QP、WBC、RL 的跨阶段实现。
