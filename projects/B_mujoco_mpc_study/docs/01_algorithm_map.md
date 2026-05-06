# Project B 算法地图

## 主线问题

当前 A 项目更接近“当前时刻给定目标，解一个单步 QP-IK”。Project B 关心的是：给定当前状态和未来目标，如何在 MuJoCo 中滚动预测未来，并选择一段未来控制序列。

## 核心算法概念

### Predictive Sampling

已确认：MJPC README 明确列出 Predictive Sampling 是 derivative-free planner。

学习理解：

- 对候选控制序列做采样。
- 用 MuJoCo rollout 预测每条候选序列的未来状态。
- 用 cost / residual 对轨迹打分。
- 选择更优序列，并只执行第一个控制量。

### iLQG

已确认：MJPC README 明确列出 iLQG 是 derivative-based method。

学习理解：

- 在当前名义轨迹附近线性化动力学、二次近似代价。
- 反向传播计算局部反馈律。
- 前向 rollout 更新控制序列。

### Gradient Descent

已确认：MJPC README 明确列出 Gradient Descent planner，并提示搜索步长受 cost scale 影响，需要按 task 调整。

学习理解：

- 直接对控制序列或策略参数沿梯度方向更新。
- 优点是概念清楚。
- 风险是尺度敏感，调参成本高。

### Multiple Shooting

已确认：MJPC README 提到支持 multiple shooting-based planners。

学习理解：

- 将长 horizon 分成多个短段。
- 每段都有状态和控制变量。
- 通过连续性约束或 rollout 连接相邻段。

### Rollout

学习理解：

- 输入：当前状态、控制序列、MuJoCo 模型。
- 输出：未来状态序列、观测、代价。
- 作用：把“控制候选”转成“可评价的未来行为”。

### Cost / Residual

学习理解：

- residual 是 task 误差，例如末端位置误差、姿态误差、速度误差、接触误差。
- cost 通常由 residual 加权平方得到。
- A 项目的 task-space tracking 可以自然扩展成 horizon 上的 residual 累积。

### Horizon 与 Receding Horizon Control

学习理解：

- horizon：每次优化向未来看多远。
- receding horizon：每次只执行优化序列的第一个控制量，然后重新观测、重新优化。

## 待源码确认的问题

- MJPC 中 residual 是如何注册和组合的。
- planner 是否统一使用同一套 trajectory / policy 数据结构。
- task XML、C++ residual、Python model 三者如何保持一致。
- humanoid mocap tracking 中参考轨迹如何进入 cost。
