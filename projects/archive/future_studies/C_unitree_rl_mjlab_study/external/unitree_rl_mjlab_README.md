# unitree_rl_mjlab 外部项目说明

## 项目用途

`unitreerobotics/unitree_rl_mjlab` 是本 baseline 的强化学习运动控制支撑项目，用于学习 Unitree 机器人在 MuJoCo 中的 Train、Play、Sim2Real、observation、action、reward、termination 和 policy 部署流程。

## 官方仓库地址

| 项目 | 地址 |
| --- | --- |
| unitree_rl_mjlab | `https://github.com/unitreerobotics/unitree_rl_mjlab` |

## 本项目如何使用它

| 用途 | 说明 |
| --- | --- |
| 流程学习 | 拆解 Train -> Play -> Sim2Real 的工程流程 |
| 模型分析 | 阅读 MJCF、关节定义、action 维度和控制接口 |
| RL 拆解 | 分析 observation、action、reward、termination |
| 部署分析 | 理解 policy 输出如何变成机器人控制命令和安全限制 |

## 准备阶段限制

准备阶段不下载源码，不提交模型权重、训练日志、大视频或大型 mesh。后续优先跑 Play 或短训练，重点是形成可解释的 obs/action/reward/policy 部署文档。

## 后续记录位置

| 内容 | 路径 |
| --- | --- |
| 项目阅读入口 | `projects/C_unitree_rl_mjlab_study/docs/README.md` |
| Play 或短训练日志摘要 | `projects/C_unitree_rl_mjlab_study/reproduce_logs/` |
| 模型控制与 RL 对比 | `docs/04_compare/` |
