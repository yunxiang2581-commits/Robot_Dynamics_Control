# 机器人模型与资源计划

## 阅读导航

| 上一篇 | 当前文档 | 下一篇 |
| --- | --- | --- |
| [06 开源项目计划](06_external_project_plan.md) | 机器人模型与资源计划 | [08 Codex 工作流规则](08_codex_workflow_rules.md) |

## 本文用途

本文件定义 URDF、MJCF、mesh、视频、日志、模型权重等资源的放置和 Git 管理规则，避免大文件污染仓库。

## 核心结论

| 资源 | 策略 |
| --- | --- |
| 小型文本资源 | 可提交，例如轻量 URDF、配置、Markdown 报告 |
| 大型二进制资源 | 默认不提交，例如 mesh、视频、训练权重 |
| 实验输出 | 图表可精选提交，视频和日志默认忽略 |
| 第三方源码 | 不复制进仓库，优先使用外部目录、submodule 或阅读记录 |

## 资源分类表

| 资源类型 | 用途 | 推荐位置 | 是否提交 Git | 规则 |
| --- | --- | --- | --- | --- |
| URDF | A 项 Pinocchio 模型加载 | `shared/robot_assets/models/` 或外部路径 | 小型示例可提交 | 大型 mesh 不随 URDF 一起提交 |
| MJCF | MuJoCo 仿真和 RL | `shared/robot_assets/models/` 或外部路径 | 小型示例可提交 | 复杂模型优先引用官方仓库 |
| mesh | 视觉和碰撞模型 | 外部数据目录 | 默认不提交 | 大文件通过下载说明或 submodule 管理 |
| 图片 | 结果图 | `projects/*/outputs/` 或根 `outputs/figures/` | 可提交精选图 | 只提交小体积展示图 |
| 视频 | 控制或 Play demo | `projects/*/outputs/videos/` 或根 `outputs/videos/` | 默认不提交 | 可在 release 或网盘外链管理 |
| 日志 | 安装、编译、训练记录 | `projects/*/outputs/logs/` 或根 `outputs/logs/` | 默认不提交完整日志 | 文档中保留摘要 |
| 权重 | RL policy、checkpoint | `projects/*/outputs/checkpoints/` | 不提交 | 只记录来源和指标 |
| 报告 | 实验总结 | `projects/*/outputs/reports/` 或根 `outputs/reports/` | 可提交 | 使用 Markdown 或小型 PDF |

## 模型选择策略

| 阶段 | 推荐模型 | 原因 | fallback |
| --- | --- | --- | --- |
| A1-A6 | 简单机械臂或轻量机器人 URDF | 易于验证 FK 和 Jacobian | 使用 Pinocchio 示例模型 |
| A7-A10 | 带关节限位的模型 | 适合 IK 和 QP 约束 | 先用低自由度模型 |
| B | `legged_control` 原项目模型 | 与 NMPC/WBC 工程一致 | 只读配置和文档，不强制运行 |
| C | Unitree MJCF 模型 | 与 RL 环境一致 | 先跑官方 Play 或短训练 |

## Git 忽略策略

| 路径 | 规则 | 目的 |
| --- | --- | --- |
| `outputs/videos/*` | 忽略视频，保留 README | 防止大视频进入仓库 |
| `outputs/logs/*` | 忽略日志，保留 README | 防止长日志进入仓库 |
| `outputs/checkpoints/` | 整目录忽略 | 防止模型权重进入仓库 |
| `projects/*/external/*/src/` | 忽略第三方源码 | 防止把大仓库复制进来 |
| `third_party/` | 忽略 | 第三方依赖不直接提交 |
| `build/`、`install/`、`log/` | 忽略 | ROS 构建产物不提交 |

## 准备任务

| 任务编号 | 任务 | 输出 | 验收标准 |
| --- | --- | --- | --- |
| PREP-009 | 定义资源放置规则 | 本文件 | URDF、MJCF、mesh、视频、日志、权重都有规则 |
| PREP-009 | 定义 Git 忽略策略 | `.gitignore` | 大文件不会默认提交 |
| PREP-009 | 定义 fallback | 模型选择策略表 | 模型缺失时项目仍能推进 |
