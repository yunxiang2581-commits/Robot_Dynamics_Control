# 准备阶段任务总表

## 阅读导航

| 上一篇 | 当前文档 | 下一篇 |
| --- | --- | --- |
| [02 三总项目范围说明](02_three_project_scope.md) | 准备阶段任务总表 | [04 环境需求文档](04_environment_requirements.md) |

## 本文用途

本文件把准备阶段拆成可执行任务。后续检查项目进度时，以本表为准。

## 核心结论

| 项目 | 说明 |
| --- | --- |
| 当前任务范围 | PREP-001 到 PREP-015 |
| 必做任务 | PREP-001 到 PREP-013 |
| 建议任务 | PREP-014、PREP-015 |
| 下一阶段前提 | 准备文档、目录、README、`.gitignore` 全部可检查 |

## 任务总表

为避免宽表格换行，本节按任务分组记录输出和验收标准。

### 基础文档任务

- PREP-001：创建 `docs/00_preparation/`，验收标准是准备阶段目录可见。
- PREP-002：编写 `00_preparation_overview.md`，验收标准是说明目标、边界和产物。
- PREP-003：编写 `01_job_target_and_skill_matrix.md`，验收标准是岗位关键词能映射到项目能力。
- PREP-004：编写 `02_three_project_scope.md`，验收标准是 A/B/C 范围清晰且不发散。
- PREP-005：编写 `03_preparation_task_table.md`，验收标准是任务有编号、输出和验收方式。

### 环境、结构和外部项目任务

- PREP-006：编写 `04_environment_requirements.md`，验收标准是环境可以逐项检查。
- PREP-007：编写 `05_repository_structure_plan.md`，验收标准是 monorepo 目录职责明确。
- PREP-008：编写 `06_external_project_plan.md`，验收标准是 B/C 外部项目有阅读、复现和限制说明。
- PREP-009：编写 `07_robot_model_and_asset_plan.md`，验收标准是 URDF、MJCF、mesh、outputs 和 Git 忽略策略清楚。

### 规则、风险和验收任务

- PREP-010：编写 `08_codex_workflow_rules.md`，验收标准是明确不要直接写完整复杂算法。
- PREP-011：编写 `09_risk_and_fallback_plan.md`，验收标准是每个高风险都有备选方案。
- PREP-012：编写 `10_preparation_acceptance_checklist.md`，验收标准是可用于进入开发阶段前检查。
- PREP-013：更新 `README.md`，验收标准是 README 能跳转到准备阶段文档和 A/B/C 项目入口。
- PREP-014：创建 A/B/C 项目容器，验收标准是 `projects/A_self_baseline/`、`projects/B_legged_control_study/`、`projects/C_unitree_rl_mjlab_study/` 存在。
- PREP-015：创建 `.gitignore` 草案，验收标准是不提交大视频、权重、缓存和运行日志。

## 优先级说明

| 优先级 | 含义 | 处理方式 |
| --- | --- | --- |
| 必做 | 准备阶段完成前必须存在 | 本阶段直接创建 |
| 建议 | 对仓库管理有帮助 | 本阶段尽量创建 |
| 后续 | 进入算法阶段再做 | 本阶段只保留入口或说明 |

## 输出产物

| 类型 | 路径 | 说明 |
| --- | --- | --- |
| 准备文档 | `docs/00_preparation/` | 当前阶段核心产物 |
| 占位文档 | `projects/A_self_baseline/docs/` 等 | 后续 A/B/C 阶段入口 |
| 开源项目说明 | `projects/B_legged_control_study/external/`、`projects/C_unitree_rl_mjlab_study/external/` | 只放说明，不放大型源码 |
| 忽略规则 | `.gitignore` | 控制缓存、大视频、权重、日志 |

## 使用方式

| 使用者 | 如何使用 |
| --- | --- |
| 学习者 | 按任务编号检查准备阶段是否完成 |
| Codex | 按“输入 -> 输出 -> 验收标准”逐项生成或修改文件 |
| 面试复盘 | 说明项目不是零散脚本，而是按阶段管理的求职 baseline |
