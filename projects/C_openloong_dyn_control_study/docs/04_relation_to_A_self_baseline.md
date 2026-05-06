# Project C 与 A_self_baseline 的关系

## A 当前是什么

A 是当前仓库的基础学习项目，主要覆盖：

- MuJoCo / Pinocchio 基础。
- FK / Jacobian。
- QP-IK。
- task-space tracking。
- 简单可复盘的文本输出。

## C 提供什么参考

Project C 是人形机器人完整 MPC + WBC 控制架构参考。它回答的问题更高级：

- 行走命令如何变成 gait phase？
- 落脚点如何规划？
- MPC 如何给出 CoM 和接触力目标？
- WBC 如何把高层目标变成关节级任务和力矩？
- PVT 如何执行底层关节控制？

## 关系边界

C 不应直接并入 A。原因：

- 控制栈复杂度远高于 A 当前阶段。
- A 仍处于 QP-IK / tracking baseline。
- 直接引入 C 会掩盖基础概念，降低学习可控性。

## 适合借鉴给 A 的内容

- DataBus 的数据流思想。
- StateEstimator 的输入输出边界。
- WBC-QP 的任务和约束组织方式。
- MuJoCo 闭环仿真的模块划分。

## 当前不应借鉴的内容

- 完整人形 MPC。
- jumping / obstacle stepping。
- 真实机器人部署接口。
- 复杂工程结构。
