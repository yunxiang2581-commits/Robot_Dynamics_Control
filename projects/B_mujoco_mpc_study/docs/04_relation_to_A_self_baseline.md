# Project B 与 A_self_baseline 的关系

## A 当前提供的基础

`A_self_baseline` 当前是 MuJoCo / Pinocchio / QP-IK / task-space tracking 的学习基线。它适合回答：

- 当前模型状态是什么？
- 某个 frame 的位置或姿态如何计算？
- 当前时刻如何用 Jacobian 做单步修正？
- 如何在 MuJoCo 中播放或跟踪一个目标？

## B 要推进的问题

Project B 负责把 A 从“单步优化 / IK”推进到“预测控制 / MPC”。

核心变化：

```text
A: 当前误差 -> 单步 QP-IK -> 当前控制/位姿修正
B: 当前状态 + 未来目标 -> horizon rollout -> 未来 cost -> 执行第一步控制
```

## 适合借鉴给 A 的内容

- task residual 的定义方式。
- horizon cost 的组织方式。
- rollout 作为控制评价器的思路。
- planner 与 task 解耦的工程结构。

## 不应直接并入 A 的内容

- MJPC 完整 C++ 框架。
- GUI / app / gRPC service。
- 复杂 humanoid tracking task。
- 未理解前直接替换 A05 / A06 控制逻辑。

## 后续最小连接点

可以先在 A 的旁路教学脚本中实现：

1. 读取 A05 或 A06 的目标轨迹。
2. 定义 horizon tracking cost。
3. 用 MuJoCo rollout 评价候选控制。
4. 输出文本和 CSV，不直接改已有控制代码。
