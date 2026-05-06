# Project D 后续问题

## NMPC 问题

- `legged_interface` 中 state 和 input 的定义是什么？
- dynamics 是 centroidal model 还是更完整的刚体动力学？
- cost 中 torso tracking、input regularization、足端约束如何加权？
- contact schedule 如何进入 switched system？

## WBC 问题

- WBC-QP 是否严格使用 `qddot`、contact force、tau？
- QP 中动力学方程如何线性化或离散化？
- friction cone 是线性约束还是二阶锥近似？
- slack variable 如何设置？

## 状态估计问题

- Kalman filter 的状态向量是什么？
- 观测量包括哪些 foot position measurement？
- 接触状态错误时估计会如何退化？

## 与 A 连接的问题

- A 是否需要先补充 contact Jacobian 学习脚本？
- contact force QP 应该先用二维还是三维？
- 是否需要先写一份 OCS2 概念笔记，再阅读源码？
