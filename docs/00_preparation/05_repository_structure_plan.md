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
| 当前重点目录 | `docs/00_preparation/` |
| 后续 A/B/C 入口 | `docs/01_self_baseline/`、`docs/02_legged_control/`、`docs/03_unitree_rl_mjlab/` |
| 代码目录状态 | `src/robot_baseline/` 只放占位文件 |
| 输出目录规则 | `outputs/` 用于图、视频、日志、报告；大文件默认忽略 |

## 目录结构准备表

| 目录 | 是否创建 | 用途 | 注意事项 |
| --- | --- | --- | --- |
| `docs/00_preparation/` | 是 | 准备阶段全部文档 | 当前阶段重点 |
| `docs/01_self_baseline/` | 是 | A 项目文档占位 | 暂不写算法细节 |
| `docs/02_legged_control/` | 是 | B 项目文档占位 | 只放阅读和复现模板 |
| `docs/03_unitree_rl_mjlab/` | 是 | C 项目文档占位 | 只放 RL 拆解模板 |
| `docs/04_compare/` | 是 | 模型控制 vs RL 对比 | 准备阶段放模板 |
| `docs/interview/` | 是 | 面试讲稿 | 准备阶段放模板 |
| `src/robot_baseline/` | 可创建 | 后续自研代码 | 只放 `__init__.py` 和 README |
| `scripts/` | 可创建 | 后续运行脚本 | 准备阶段不写核心算法 |
| `configs/` | 可创建 | 参数配置 | 可放空模板说明 |
| `external/` | 是 | 开源项目说明 | 不下载大仓库 |
| `outputs/` | 是 | 图、视频、日志、报告 | 配合 `.gitignore` |
| `tests/` | 可创建 | 后续测试 | 准备阶段只放 README |

## 推荐最终结构

```text
.
├── README.md
├── docs/
│   ├── 00_preparation/
│   ├── 01_self_baseline/
│   ├── 02_legged_control/
│   ├── 03_unitree_rl_mjlab/
│   ├── 04_compare/
│   └── interview/
├── src/
│   └── robot_baseline/
├── scripts/
├── configs/
├── external/
├── outputs/
│   ├── figures/
│   ├── videos/
│   ├── logs/
│   └── reports/
└── tests/
```

## 文件职责表

| 文件或目录 | 职责 | 验收标准 |
| --- | --- | --- |
| `README.md` | 项目入口、三条主线、当前阶段说明 | 能跳转到准备阶段文档 |
| `.gitignore` | 大文件和缓存忽略规则 | 不默认提交视频、日志、权重 |
| `external/*_README.md` | 开源项目用途说明 | 明确准备阶段不下载源码 |
| `src/robot_baseline/README.md` | 后续自研代码说明 | 明确当前不实现算法 |
| `scripts/README.md` | 后续脚本命名规则 | 明确准备阶段无运行脚本 |
| `configs/README.md` | 后续配置管理规则 | 明确准备阶段无参数文件 |
| `tests/README.md` | 后续测试策略 | 明确准备阶段无算法测试 |

## 准备任务

| 任务编号 | 任务 | 输出 | 验收标准 |
| --- | --- | --- | --- |
| PREP-007 | 规划目录职责 | 本文件 | 每个目录都有用途说明 |
| PREP-014 | 创建占位目录 | 目录结构 | 后续阶段入口存在 |
| PREP-014 | 创建占位 README | 各目录 README | 能说明当前阶段边界 |

