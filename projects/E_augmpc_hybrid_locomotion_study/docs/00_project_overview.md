# Project E Overview

## New Positioning

Project E 现在是 `AugMPC / LRHControl / IBRIDO` 复现项目，不再沿用旧的 `isaac-quad-loco` / Orbit 学习线。

核心目标是复现 RL-augmented MPC locomotion 结构：

- high-level RL 输出 contact schedules / twist commands
- low-level MPC 执行动力学控制
- 优先复现 public bundles、visualization 和 eval 路径

## Relation To Project B / C

- Project B：提供 MuJoCo MPC、solver ladder、iLQR / CEM / MPPI 等控制视角，可作为“低层 MPC 控制思想”的对照线
- Project C：提供 humanoid WBC / MPC-WBC / contact constraint 学习线，可作为“复杂控制栈分层方式”的对照线
- Project E：聚焦 RL-augmented MPC locomotion reproduction，重点不是自写教学骨架，而是围绕公开复现资源做静态审查、容器路线、bundle/eval smoke 和后续最小运行

## Current Non-Goals

- 不做真实机器人部署
- 不做硬件驱动
- 不做 sim2real 声称
- 当前不做完整论文级复现声明
