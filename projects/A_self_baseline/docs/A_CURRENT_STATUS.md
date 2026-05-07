# Project A 当前状态

Project A 的主目录是：

```text
projects/A_self_baseline/
```

## 1. 项目定位

A 是当前自研基础主线项目，服务于整个 simulation-only 机器人运动控制求职项目。

它的作用不是复刻某一个外部仓库，而是建立后续 B/C/D 项目所需的基础能力：

- MuJoCo 模型加载与仿真理解。
- Pinocchio / 运动学基础。
- Jacobian 与 IK。
- QP-IK。
- task-space tracking。
- simulation-only full motion control plan。
- Mink / UR5e 对齐计划。

## 2. 当前状态

A 已经具备基础 QP-IK / task-space tracking / simulation-only 方向的能力积累。

当前 A 的价值是：

- 为 Project B 的 MPC 简化 demo 提供 MuJoCo、目标跟踪和误差指标经验。
- 为 Project C/D 的简化 QP / WBC demo 提供 QP 建模经验。
- 为后续 video demo 和 metrics 输出提供 baseline 项目结构参考。

## 3. 本轮边界

本轮不修改 A 源码，不调整 A 的 Python/C++ 控制代码。

本轮只新增当前状态说明文档。

## 4. 后续角色

A 后续继续作为基础能力来源：

- 提供基础 MuJoCo / Pinocchio / QP 能力。
- 可承接 Project B 的 MPC 简化 demo。
- 可承接 Project C/D 的简化 QP / WBC demo。
- 继续保持 simulation-only，不做实物部署。

## 5. 当前不做

- 不做实物部署。
- 不做 sim2real。
- 不接电机。
- 不写硬件接口。
- 不做真实机器人安全测试。
