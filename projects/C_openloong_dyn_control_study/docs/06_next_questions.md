# Project C 后续问题

## 架构问题

- DataBus 中哪些字段来自 MuJoCo，哪些字段来自控制器内部？
- StateEstimator 在仿真中是否直接使用 ground truth？
- MPC 输出给 WBC 的变量具体有哪些？
- WBC 输出给 PVT 的变量是期望位置、速度、力矩，还是三者组合？

## 数学问题

- MPC 使用的状态模型是什么？
- 接触力约束如何表达？
- WBC_QP 的任务优先级如何实现？
- 摆动腿轨迹是多项式、Bezier，还是其他形式？

## 与 A 连接的问题

- A 的 QP-IK 能否作为 WBC_QP 的前置学习？
- 当前 A 是否需要先补充 floating-base `nq/nv` 专题笔记？
- 如果做简化双足接触力 QP，是否应放在 A 的新教学脚本中？
