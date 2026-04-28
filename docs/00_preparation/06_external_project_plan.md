# 开源项目计划

> English version: [06_external_project_plan_en.md](06_external_project_plan_en.md)

## 阅读导航

| 上一篇 | 当前文档 | 下一篇 |
| --- | --- | --- |
| [05 仓库结构规划](05_repository_structure_plan.md) | 开源项目计划 | [07 机器人模型与资源计划](07_robot_model_and_asset_plan.md) |

## 本文用途

本文件定义两个主支撑开源项目在本 baseline 中的使用方式。准备阶段只记录用途、阅读目标、复现目标和产物位置，不下载大型源码。

## 核心结论

| 项目 | 用途 | 当前阶段动作 |
| --- | --- | --- |
| `qiayuanl/legged_control` | 支撑传统模型控制、NMPC、WBC、状态估计 | 写清阅读和复现计划，不下载源码 |
| `unitreerobotics/unitree_rl_mjlab` | 支撑 MuJoCo RL、Train/Play/Sim2Real | 写清 obs/action/reward 拆解计划，不训练模型 |

## 开源项目准备表

| 开源项目 | 类型 | 准备阶段目标 | 准备阶段产物 |
| --- | --- | --- | --- |
| `qiayuanl/legged_control` | 传统模型控制 | 明确它用于 NMPC/WBC/状态估计学习 | `projects/B_legged_control_study/docs/`、`projects/B_legged_control_study/external/` |
| `unitreerobotics/unitree_rl_mjlab` | 强化学习控制 | 明确它用于 MuJoCo RL、Train/Play/Sim2Real 学习 | `projects/C_unitree_rl_mjlab_study/docs/`、`projects/C_unitree_rl_mjlab_study/external/` |
| OCS2 | 辅助参考 | 作为 NMPC 求解器体系参考 | 在 B 文档中列为参考 |
| Unitree RL Lab | 后续扩展 | 作为 IsaacLab 方向扩展 | 在 C 文档中列为扩展 |

## `legged_control` 使用计划

| 项目 | 内容 |
| --- | --- |
| 官方仓库 | `https://github.com/qiayuanl/legged_control` |
| 本项目用途 | 学习传统模型控制工程链路，重点关注 NMPC、WBC、状态估计、关节力矩控制 |
| 阅读目标 | README、依赖、目录结构、核心控制器、WBC、状态估计 |
| 复现目标 | 记录环境、依赖、编译命令、运行日志、失败原因和 fallback |
| 对照目标 | 与 A 项 Mini-WBC 对比变量、目标函数、约束和输出 |
| 本阶段限制 | 不下载完整源码，不提交第三方仓库内容 |

## `unitree_rl_mjlab` 使用计划

| 项目 | 内容 |
| --- | --- |
| 官方仓库 | `https://github.com/unitreerobotics/unitree_rl_mjlab` |
| 本项目用途 | 学习 Unitree MuJoCo 强化学习控制流程，重点关注 Train、Play、Sim2Real |
| 阅读目标 | README、训练脚本、环境定义、模型文件、部署流程 |
| 复现目标 | 优先跑 Play 或短训练，不追求大规模训练收敛 |
| 拆解目标 | observation、action、reward、termination、policy 输出到控制命令 |
| 本阶段限制 | 不下载完整源码，不提交模型权重和训练产物 |

## 准备任务

| 任务编号 | 任务 | 输出 | 验收标准 |
| --- | --- | --- | --- |
| PREP-008 | 定义开源项目用途 | 本文件和 B/C `external/` 说明 | 不把开源项目当成黑盒 |
| PREP-008 | 定义阅读和复现目标 | B/C 项 README | 后续能按目标拆解源码和日志 |
| PREP-008 | 定义限制 | 本文件限制说明 | 准备阶段不下载大型仓库 |
