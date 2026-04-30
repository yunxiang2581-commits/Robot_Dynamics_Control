# A 自研机器人运动控制基础系统

## 当前状态

本目录是 A 项目的文档入口。

当前 A 项目已从准备阶段进入实现阶段，主线是对标 mink UR5e 示例的教学版 6-DOF 机械臂控制 baseline。当前实现入口是 A01 `model inspect`，目标是先检查 `shared/robot_assets/models/mink_universal_robots_ur5e/scene.xml` 中的 MuJoCo 模型对象。

## 后续内容

| 文档 | 目标 | 验收标准 |
| --- | --- | --- |
| `mink_capability_vs_A_requirements.md` | mink 能力与 A 项目需求总表 | 明确 A00-A10、最终 demo 和实现边界 |
| `A_pipeline_contract.md` | A00-A10 pipeline 契约 | 每一步输入、输出、对标 mink 概念和验收标准清楚 |
| `reference_mink_ur5e.md` | 为什么选 mink UR5e | 明确 reference 与 copied assets 的边界 |
| `A_mink_alignment_plan.md` | 分阶段实现计划 | 明确从 A01 到 A10 的推进顺序 |
| `01_model_inspect.md` | A01 历史 URDF inspect 说明 | 后续适配为 A01 model inspect / MJCF inspect |

## 当前下一步

进入 A01 最小可运行 MJCF inspect，只补 A01 相关模型检查逻辑和报告输出，不实现后续 FK、Jacobian、IK 或 QP。
