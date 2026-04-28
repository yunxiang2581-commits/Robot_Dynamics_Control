# 仓库结构规划

## 阅读导航

| 上一篇 | 当前文档 | 下一篇 |
| --- | --- | --- |
| [04 环境需求文档](04_environment_requirements.md) | 仓库结构规划 | [06 开源项目计划](06_external_project_plan.md) |

## 本文用途

本文件定义仓库目录职责，保证后续 Codex 和人工开发都按同一结构工作。

## 核心结论

| 项目 | 说明 |
| --- | --- |
| 当前重点目录 | `docs/00_preparation/`、`docs/00_project_management/` |
| A/B/C 入口 | `projects/A_self_baseline/`、`projects/B_legged_control_study/`、`projects/C_unitree_rl_mjlab_study/` |
| 代码目录状态 | A 标准 TODO 模块在 `projects/A_self_baseline/src/robot_baseline/` |
| 输出目录规则 | 根 `outputs/` 作为总输出索引；项目级输出放 `projects/*/outputs/` |

## 目录结构准备表

| 目录 | 是否创建 | 用途 | 注意事项 |
| --- | --- | --- | --- |
| `docs/00_preparation/` | 是 | 准备阶段全部文档 | 当前阶段重点 |
| `projects/A_self_baseline/docs/` | 是 | A 项目文档占位 | 暂不写算法细节 |
| `projects/B_legged_control_study/docs/` | 是 | B 项目文档占位 | 只放阅读和复现模板 |
| `projects/C_unitree_rl_mjlab_study/docs/` | 是 | C 项目文档占位 | 只放 RL 拆解模板 |
| `docs/04_compare/` | 是 | 模型控制 vs RL 对比 | 准备阶段放模板 |
| `docs/interview/` | 是 | 面试讲稿 | 准备阶段放模板 |
| `projects/A_self_baseline/src/robot_baseline/` | 可创建 | 后续自研代码 | 只放 `__init__.py` 和 README |
| `projects/A_self_baseline/scripts/` | 可创建 | 后续运行脚本 | 准备阶段不写核心算法 |
| `projects/A_self_baseline/configs/` | 可创建 | 参数配置 | 可放空模板说明 |
| `projects/B_legged_control_study/external/` | 是 | B 外部项目说明 | 不下载大仓库 |
| `projects/C_unitree_rl_mjlab_study/external/` | 是 | C 外部项目说明 | 不下载大仓库 |
| `outputs/` | 是 | 图、视频、日志、报告 | 配合 `.gitignore` |
| `projects/A_self_baseline/tests/` | 可创建 | 后续测试 | 准备阶段只放 README |

## 推荐最终结构

```text
Robot_Dynamics_Control/
├── projects/
│   ├── A_self_baseline/
│   ├── B_legged_control_study/
│   └── C_unitree_rl_mjlab_study/
├── shared/
├── tools/
├── docs/
├── exports/
├── outputs/
└── requirements.txt
```

## 文件职责表

| 文件或目录 | 职责 | 验收标准 |
| --- | --- | --- |
| `README.md` | 项目入口、三条主线、当前阶段说明 | 能跳转到准备阶段文档 |
| `.gitignore` | 大文件和缓存忽略规则 | 不默认提交视频、日志、权重 |
| `projects/*/external/` | 开源项目用途说明 | 明确准备阶段不下载源码 |
| `projects/A_self_baseline/src/robot_baseline/README.md` | 后续自研代码说明 | 明确当前不实现算法 |
| `projects/A_self_baseline/scripts/` | A 标准学习脚本 | 当前是 TODO 教学骨架 |
| `projects/A_self_baseline/configs/` | A 配置模板 | 不写死旧项目路径 |
| `projects/A_self_baseline/tests/` | A 测试入口 | 后续随算法实现补充 |

## 准备任务

| 任务编号 | 任务 | 输出 | 验收标准 |
| --- | --- | --- | --- |
| PREP-007 | 规划目录职责 | 本文件 | 每个目录都有用途说明 |
| PREP-014 | 创建占位目录 | 目录结构 | 后续阶段入口存在 |
| PREP-014 | 创建占位 README | 各目录 README | 能说明当前阶段边界 |
