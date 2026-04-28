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

| 编号 | 任务名称 | 输入 | Codex 要做什么 | 输出 | 验收标准 | 优先级 |
| --- | --- | --- | --- | --- | --- | --- |
| PREP-001 | 创建准备阶段目录 | 当前仓库 | 创建 `docs/00_preparation/` | 目录存在 | `docs/00_preparation/` 可见 | 必做 |
| PREP-002 | 编写准备阶段总览 | 本需求 | 生成阶段说明文档 | `00_preparation_overview.md` | 说明目标、边界、产物 | 必做 |
| PREP-003 | 编写求职能力矩阵 | 岗位关键词 | 把岗位要求映射到项目模块 | `01_job_target_and_skill_matrix.md` | 有“岗位关键词 -> 能力 -> 项目任务”映射 | 必做 |
| PREP-004 | 编写三项目范围 | A/B/C 三条主线 | 明确每个项目做什么、不做什么 | `02_three_project_scope.md` | 不包含发散目标 | 必做 |
| PREP-005 | 编写准备任务表 | 本表 | 形成 Markdown 任务总表 | `03_preparation_task_table.md` | 有编号、输入、输出、验收 | 必做 |
| PREP-006 | 编写环境需求 | Ubuntu、Python、Docker 背景 | 生成环境检查表 | `04_environment_requirements.md` | 能逐项检查环境 | 必做 |
| PREP-007 | 编写仓库结构规划 | 目标仓库结构 | 生成目录职责表 | `05_repository_structure_plan.md` | 每个目录职责明确 | 必做 |
| PREP-008 | 编写开源项目计划 | `legged_control`、`unitree_rl_mjlab` | 写清阅读、复现、对照目标 | `06_external_project_plan.md` | 每个项目有目标、产物、验收 | 必做 |
| PREP-009 | 编写模型资源计划 | URDF、MJCF、mesh、outputs | 规定资源放置和 Git 忽略策略 | `07_robot_model_and_asset_plan.md` | 避免大文件污染仓库 | 必做 |
| PREP-010 | 编写 Codex 规则 | TODO 教学骨架偏好 | 生成 Codex 工作流规则文档 | `08_codex_workflow_rules.md` | 明确不要直接写完整复杂算法 | 必做 |
| PREP-011 | 编写风险计划 | 环境、编译、训练风险 | 制定 fallback | `09_risk_and_fallback_plan.md` | 每个高风险都有备选方案 | 必做 |
| PREP-012 | 编写验收清单 | 准备阶段所有产物 | 生成勾选表 | `10_preparation_acceptance_checklist.md` | 勾完即可进入开发阶段 | 必做 |
| PREP-013 | 更新总 README | 所有准备文档 | 在 README 增加准备阶段入口 | `README.md` | README 能跳转到准备阶段文档 | 必做 |
| PREP-014 | 创建后续阶段占位目录 | A/B/C 项目规划 | 创建 A/B/C 文档占位目录 | 目录结构 | 不写算法，只放 README 占位 | 建议 |
| PREP-015 | 创建 `.gitignore` 草案 | 大文件规则 | 补充输出、模型、日志忽略规则 | `.gitignore` | 不提交大视频、权重、缓存 | 建议 |

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
| 占位文档 | `docs/01_self_baseline/` 等 | 后续 A/B/C 阶段入口 |
| 开源项目说明 | `external/` | 只放说明，不放大型源码 |
| 忽略规则 | `.gitignore` | 控制缓存、大视频、权重、日志 |

## 使用方式

| 使用者 | 如何使用 |
| --- | --- |
| 学习者 | 按任务编号检查准备阶段是否完成 |
| Codex | 按“输入 -> 输出 -> 验收标准”逐项生成或修改文件 |
| 面试复盘 | 说明项目不是零散脚本，而是按阶段管理的求职 baseline |

