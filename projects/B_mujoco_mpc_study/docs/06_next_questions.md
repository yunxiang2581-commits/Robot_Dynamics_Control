# Project B 后续问题

## 概念问题

- task residual 和 cost 在 MJPC 中是否完全分离？
- horizon 长度如何影响实时性和稳定性？
- Predictive Sampling 与 iLQG 的适用任务边界是什么？
- multiple shooting 在 MJPC 中是如何保证轨迹连续性的？

## 源码问题

- `agent` 中状态更新、planner 调用、控制输出的主循环在哪里？
- `tasks` 目录里最小 task 需要实现哪些函数？
- Python API 与 C++ task 的兼容性由谁保证？
- humanoid mocap tracking 的 reference motion 如何进入 residual？

## 与 A 连接的问题

- A05 的 QP-IK 输出更适合作为状态参考，还是末端位姿参考？
- A06 的 target mocap tracking 能否先改成 horizon cost 的数据源？
- 极简 MPC demo 应该先选 single-joint 还是 two-link？
