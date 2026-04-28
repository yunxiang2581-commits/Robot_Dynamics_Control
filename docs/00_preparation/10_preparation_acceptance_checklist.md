# 准备阶段验收清单

## 阅读导航

| 上一篇 | 当前文档 | 下一篇 |
| --- | --- | --- |
| [09 风险与备选方案](09_risk_and_fallback_plan.md) | 准备阶段验收清单 | 后续进入 A/B/C 开发阶段 |

## 本文用途

本文件用于判断准备阶段是否完成。勾完本清单后，才能进入 A/B/C 三条主线的代码骨架和算法开发。

## 核心结论

| 验收维度 | 标准 |
| --- | --- |
| 文档 | 准备阶段文档齐全、结构统一、能互相跳转 |
| 仓库 | README、`.gitignore`、`external/`、`outputs/`、A/B/C 占位目录齐全 |
| 边界 | 明确准备阶段不实现算法 |
| 后续条件 | 可以进入 A/B/C 的代码骨架和文档模板阶段 |

## 文档验收

| 状态 | 检查项 | 路径 | 验收标准 |
| --- | --- | --- | --- |
| [ ] | 准备阶段目录已创建 | `docs/00_preparation/` | 目录存在 |
| [ ] | 准备阶段总览已完成 | `00_preparation_overview.md` | 说明目标、边界、产物 |
| [ ] | 求职能力矩阵已完成 | `01_job_target_and_skill_matrix.md` | 有岗位关键词 -> 能力 -> 项目任务表 |
| [ ] | 三项目范围说明已完成 | `02_three_project_scope.md` | A/B/C 范围明确 |
| [ ] | 准备任务表已完成 | `03_preparation_task_table.md` | 有编号、输入、输出、验收 |
| [ ] | 环境需求文档已完成 | `04_environment_requirements.md` | 能逐项检查环境 |
| [ ] | 仓库结构规划已完成 | `05_repository_structure_plan.md` | 每个目录职责明确 |
| [ ] | 开源项目计划已完成 | `06_external_project_plan.md` | 两个开源项目有目标、产物、验收 |
| [ ] | 机器人模型与资源计划已完成 | `07_robot_model_and_asset_plan.md` | 大文件管理规则明确 |
| [ ] | Codex 工作流规则已完成 | `08_codex_workflow_rules.md` | 明确不一次性实现复杂算法 |
| [ ] | 风险与备选方案已完成 | `09_risk_and_fallback_plan.md` | 每个高风险都有 fallback |
| [ ] | 验收清单已完成 | `10_preparation_acceptance_checklist.md` | 可用于进入开发阶段前检查 |

## 仓库验收

| 状态 | 检查项 | 验收标准 |
| --- | --- | --- |
| [ ] | `README.md` 已加入准备阶段入口 | 能跳转到 `docs/00_preparation/` |
| [ ] | `external/` 中只有开源项目说明 | 没有下载大型第三方源码 |
| [ ] | `outputs/` 已有目录 | `figures/`、`videos/`、`logs/`、`reports/` 存在 |
| [ ] | `.gitignore` 已包含大文件忽略规则 | 视频、日志、权重、缓存被忽略 |
| [ ] | 后续 A/B/C 三项目目录已占位 | `docs/01_self_baseline/` 等目录存在 |
| [ ] | `src/robot_baseline/` 只含占位文件 | 当前没有算法实现 |
| [ ] | `scripts/` 只含说明文件 | 当前没有运行脚本 |
| [ ] | `configs/` 只含说明文件 | 当前没有参数配置 |
| [ ] | `tests/` 只含说明文件 | 当前没有算法测试 |

## 进入开发阶段条件

| 条件 | 标准 |
| --- | --- |
| 文档完整 | 准备阶段文档齐全 |
| 边界清晰 | 所有文档明确准备阶段不实现算法 |
| 结构可用 | 后续 A/B/C 目录入口存在 |
| 风险可控 | 编译、训练、模型、ROS、大文件风险都有备选方案 |
| 工作流明确 | 后续 Codex 按 TODO 教学骨架逐步实现 |

