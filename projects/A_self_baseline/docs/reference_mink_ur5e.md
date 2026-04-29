# Reference: mink UR5e

## 1. mink 做了什么

`mink` 是围绕 MuJoCo 机器人模型构建的 differential IK 工具库。它把模型状态、任务、限制条件和 IK 求解组织成清晰接口，使用户可以用较少样板代码表达任务空间控制问题。

A 项目不会直接调用 mink 替代自己的实现，而是把 mink 当作对照组，学习它如何组织 model、configuration、task、limit、QP 和 demo。

## 2. mink UR5e 示例做了什么

mink 的 UR5e 示例围绕 6-DOF 机械臂演示：

- 加载 MuJoCo UR5e 模型；
- 查询和更新 configuration；
- 使用 site/body 作为末端执行器目标；
- 通过 differential IK 让末端跟踪目标；
- 在 viewer 或 actuator 示例中展示 target tracking 和 actuator tracking。

## 3. 为什么适合作为 A 项目对照组

UR5e 是固定基 6-DOF 机械臂，比 H1 人形机器人更适合作为第一条完整可展示 baseline：

- 关节维度较小，便于手动推导和调试；
- site pose、Jacobian、IK 和 QP-IK 链路完整；
- MuJoCo 可视化与 actuator 控制路径清楚；
- 便于和开源库 mink 的抽象做逐项对照。

## 4. A 项目会复现的核心概念

- MuJoCo model loading；
- site pose；
- site Jacobian；
- differential IK；
- task + limit；
- QP-IK；
- actuator tracking。

## 5. A 项目暂不复现的高级功能

- 完整 mink API；
- 完整 collision avoidance；
- 多机器人 example；
- 完整库级测试体系。

这些内容后续可作为扩展目标，但不应阻塞 A01-A07 最小教学链路。

## 6. A01-A09 对照关系

| A 步骤 | A 项目学习目标 | 对标 mink 概念 |
|---|---|---|
| A00 | 记录参考范围和资产路径 | examples / model assets |
| A01 | 检查 MJCF 模型维度与对象名称 | MuJoCo model loading |
| A02 | 查询 configuration / site pose | `Configuration` |
| A03 | 检查 site Jacobian | differential IK velocity mapping |
| A04 | 实现 DLS differential IK TODO | minimal `solve_ik` 思路 |
| A05 | 引入 task、limit、QP-IK TODO | `FrameTask` / `PostureTask` / limits |
| A06 | target / mocap-style tracking TODO | viewer target / mocap target |
| A07 | actuator tracking TODO | `arm_ur5e_actuators.py` |
| A08 | collision avoidance TODO | collision avoidance constraint |
| A09 | 产出对照报告 | comparison with mink |

## 7. 模型资产建议路径

后续如果用户确认引入 UR5e/MJCF 资产，建议放到：

```text
shared/robot_assets/models/mink_universal_robots_ur5e/
```

第一版建议关注：

- `scene.xml`
- 相关 mesh 目录；
- actuator 定义；
- site / body / keyframe 名称。

本步骤不下载 mink 仓库，也不复制模型资产。

## 8. 当前边界

当前只更新文档、配置模板和 TODO 骨架说明。所有 FK、Jacobian、IK、QP、tracking 和 collision avoidance 逻辑都保留为后续学习任务。
