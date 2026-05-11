# Project B 与 A_self_baseline / Project C / Project D 的关系

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

当前路线采用：

```text
路线 2：B 先最小 demo，再升级到 OpenLoong 人形 MPC。
```

B01-B03 用小模型验证 MPC 概念，避免一开始就把模型维度、接触、floating-base、控制约束全部混在一起。B04-B06 再切到 OpenLoong，成为后续 Project C 的前置层。

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

## B 与 Project C 的关系

Project C 的目标是从 OpenLoong-Dyn-Control 学习人形机器人 MPC + WBC + PVT + MuJoCo 闭环架构。

B 与 C 的边界建议如下：

```text
B: OpenLoong-oriented humanoid MPC prototype
C: OpenLoong MPC-WBC-PVT full control-chain study
```

B 重点产出 MPC 侧的对象：

- 当前人形状态读取。
- pelvis / torso / foot residual。
- horizon rollout。
- horizon cost。
- receding horizon control。
- MPC target。
- 可选 contact schedule。

C 重点学习如何把这些目标接到完整控制链：

```text
MPC target -> WBC-QP -> joint torque / joint command -> PVT / PD -> MuJoCo closed loop
```

因此 B 的 OpenLoong 阶段不是替代 C，而是降低 C 的入口复杂度。

## B 与 Project D 的关系

Project D 学习四足 legged_control / OCS2 / NMPC / WBC / state estimation。

D 不走 OpenLoong 人形主线，但它补充腿式控制中的通用问题：

- contact schedule。
- friction cone。
- contact force QP。
- NMPC-WBC interface。
- state estimation。

这些概念会反过来帮助理解 C 中的接触力分配、WBC 约束和状态估计，但 D 不应抢 B/C 的 OpenLoong 人形主线。

## BCD 总路线

```text
A: kinematics / IK / tracking baseline
B: OpenLoong-oriented humanoid MPC prototype
C: OpenLoong MPC-WBC-PVT full control-chain study
D: legged NMPC-WBC-contact-estimation generalization
```
