# Project D 算法地图

## 主线数据流

```text
cmd_vel / goal
-> torso state trajectory
-> NMPC
-> optimized state and input
-> WBC-QP
-> torque feed-forward
-> low-gain joint-space PD
-> robot / simulation
-> state estimation
```

## 核心算法

### OCS2 NMPC

已确认：OCS2 是 Optimal Control for Switched Systems 的 C++ toolbox，支持 SLQ、iLQR、SQP、IPM、SLP 等算法。

在 legged_control 中，NMPC 用于在滚动 horizon 内求解四足机器人的状态和输入。

### Switched Systems

四足运动的接触模式会切换，例如站立、对角小跑、摆动腿阶段。接触切换使系统动力学和约束随时间改变。

### Contact Schedule

contact schedule 描述每条腿什么时候支撑、什么时候摆动，是 NMPC 和 WBC 的共同输入。

### Friction Cone

接触力必须满足摩擦锥约束，否则足端会滑动。源码阅读时应关注该约束如何线性化。

### Swing Foot Constraint

摆动腿需要满足离地高度、落脚位置和轨迹平滑性约束。

### WBC-QP

README 明确给出 WBC 决策变量：

```text
x_wbc = [qddot, contact force, tau]
```

它只考虑当前时刻，用 QP 将 NMPC 输出转换为关节力矩。

### State Estimation

README 提到使用 IMU、电机反馈、足端位置，并通过 linear Kalman filter 估计 base position 和 velocity。

### Torque Feedforward + PD Tracking

WBC 输出的 torque 作为 feed-forward，低增益 joint-space PD 负责减小触地冲击和提升跟踪表现。

## 待后续源码阅读确认的信息

- NMPC 状态 `x` 和输入 `u` 在代码中的维度。
- dynamics/cost/constraints 的类名和配置文件。
- contact schedule 如何传给 NMPC 与 WBC。
- Kalman filter 的状态转移和观测矩阵。
