# Project D 与 A_self_baseline 的关系

## A 提供的基础

A 当前提供基础 MuJoCo / Pinocchio / QP-IK 能力，适合理解：

- 模型加载。
- FK / Jacobian。
- task-space tracking。
- 简化 QP。

## D 提供的参考

Project D 提供四足机器人 NMPC + WBC + 状态估计的复杂控制栈参考。它比 A 高几个层级，重点不是马上复现，而是学习：

- NMPC 问题如何组织。
- WBC 如何把高层优化输出转换为关节力矩。
- state estimation 如何服务于闭环控制。
- sim2real 控制栈如何划分模块。

## 适合借鉴给 A 的内容

- WBC-QP 决策变量设计。
- contact force QP 的约束思想。
- 状态估计输入输出边界。
- optimal control problem 中 dynamics/cost/constraints 的分层。

## 当前不适合借鉴的内容

- ROS controller 架构。
- OCS2 全套依赖。
- Unitree 硬件接口。
- Gazebo 仿真流程。

## 建议连接方式

先在 A 的后续新脚本中做极简接触力 QP 或 WBC-QP 教学 demo，不要直接接入 legged_control。
