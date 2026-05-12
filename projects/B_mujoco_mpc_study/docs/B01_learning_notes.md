# B01 单关节 MPC 学习笔记

## 1. B01 任务目标

B01 的目标是用一个最小单关节 hinge joint 模型，学习 MPC 的基本闭环：

```text
当前状态 -> rollout 预测未来 -> horizon cost 评价 -> predictive sampling 选择 torque 序列 -> 只执行第一个 torque -> 下一轮重新规划
```

这个任务不是为了做复杂机器人控制，而是把后续 OpenLoong 人形 MPC 中会反复出现的概念拆到最小：

- state。
- control。
- rollout。
- residual。
- horizon cost。
- predictive sampling。
- receding horizon control。
- video / figures / metrics / log 可复盘输出。

## 2. 单关节物理模型

B01 使用一个单自由度转动关节：

```text
hinge joint + torque actuator
```

物理意义：

- 关节角度 `q` 表示当前转动位置。
- 关节角速度 `dq` 表示当前运动速度。
- 控制输入 `tau` 表示施加在关节上的力矩。
- MuJoCo step 会根据动力学把 `q, dq, tau` 推进到下一时刻。

这个系统足够简单，适合先理解“控制输入如何影响未来状态”，避免一开始就被多关节、接触、floating-base 干扰。

## 3. 状态 `x = [q, dq]`

B01 的状态定义为：

```text
x = [q, dq]
```

其中：

- `q`：关节角度。
- `dq`：关节角速度。

数学意义：

```text
x_k = [q_k, dq_k]
x_{k+1} = f(x_k, tau_k)
```

MuJoCo 承担 `f(...)` 的物理积分。

## 4. 控制 `u = tau`

B01 的控制输入是单个力矩：

```text
u = tau
```

物理意义：

- 正力矩让关节沿正方向加速。
- 负力矩让关节沿负方向加速。
- 力矩过大会导致不真实或不稳定，因此需要 `torque_limit`。

数学意义：

```text
tau_k in [-torque_limit, torque_limit]
```

## 5. 目标 `q_target(t)`

B01 的目标是让关节角度跟踪目标角：

```text
q -> q_target(t)
```

最小 residual 可以定义为：

```text
r_q = q - q_target(t)
r_dq = dq
r_tau = tau
```

含义：

- `r_q` 惩罚角度偏差。
- `r_dq` 惩罚速度过大，帮助稳定。
- `r_tau` 惩罚控制力矩过大，避免粗暴控制。

默认建议使用 smooth target：

```text
q_target(0.0s) = q_initial
q_target(0.5s) = target.angle
```

这样可以避免从 `0 rad` 瞬间跳到 `1.0 rad` 带来的早期大误差和力矩饱和，更符合机器人控制中的轨迹跟踪。

## 6. rollout 输入输出

rollout 的输入：

```text
initial_state = [q0, dq0]
torque_sequence = [tau_0, tau_1, ..., tau_{H-1}]
horizon = H
```

rollout 的输出：

```text
states = [x_0, x_1, ..., x_H]
controls = [tau_0, tau_1, ..., tau_{H-1}]
```

物理意义：

- rollout 是“假设未来执行这串 torque，会发生什么”的仿真预测。

数学意义：

```text
x_{k+1} = f(x_k, tau_k)
```

验证标准：

- rollout 长度应与 horizon 一致。
- 每一步状态都应包含 `q` 和 `dq`。
- 相同初始状态和相同控制序列应得到可复现的结果。

## 7. horizon cost 公式

B01 的基础 horizon cost 可以规划为：

```text
J = sum_{k=0}^{H} w_q * (q_k - q_target(t_k))^2
  + sum_{k=0}^{H} w_dq * dq_k^2
  + sum_{k=0}^{H-1} w_tau * tau_k^2
  + w_terminal * (q_H - q_target(t_H))^2
```

其中：

- `w_q`：角度误差权重。
- `w_dq`：速度误差权重。
- `w_tau`：控制力矩权重。
- `w_terminal`：horizon 末端目标误差权重。

物理意义：

- 既希望角度接近目标，也希望速度不要太大，力矩不要太粗暴。

数学意义：

- MPC 不只看当前误差，而是在未来 horizon 上累计误差。

## 8. predictive sampling 原理

Predictive Sampling 的思路是：

1. 随机或规则采样多条 torque sequence。
2. 对每条 torque sequence 做 rollout。
3. 对每条 rollout 计算 horizon cost。
4. 选择 cost 最小的 torque sequence。

伪代码：

```text
best_cost = +inf
for sequence in candidate_sequences:
    trajectory = rollout(initial_state, sequence)
    cost = compute_horizon_cost(trajectory, sequence)
    if cost < best_cost:
        best_sequence = sequence
        best_cost = cost
```

验证标准：

- planner 应记录 best cost。
- best sequence 的第一项是当前控制周期要执行的 torque。
- candidate 数量越大，搜索越充分，但 runtime 也会增加。

## 9. receding horizon control 原理

Receding horizon control 的关键是：

```text
每次规划一整段未来控制序列，但只执行第一步。
```

控制循环：

```text
for control_step:
    state = env.get_state()
    target_sequence = build_target_sequence(current_time)
    best_sequence = planner.plan(state, target_sequence)
    tau = best_sequence[0]
    env.step(tau)
```

为什么只执行第一步：

- 下一步状态已经变化，应重新观测、重新规划。
- 这样可以处理模型误差、扰动、数值误差。

## 10. 可视化输出要求

B01 必须规划并最终输出：

```text
outputs/runs/B01_single_joint_mpc_demo/<run_id>/videos/B01_single_joint_mpc_demo.mp4
outputs/runs/B01_single_joint_mpc_demo/<run_id>/figures/B01_angle_tracking.png
outputs/runs/B01_single_joint_mpc_demo/<run_id>/figures/B01_angle_error.png
outputs/runs/B01_single_joint_mpc_demo/<run_id>/figures/B01_torque.png
outputs/runs/B01_single_joint_mpc_demo/<run_id>/figures/B01_best_cost.png
outputs/runs/B01_single_joint_mpc_demo/<run_id>/metrics/B01_metrics.csv
outputs/runs/B01_single_joint_mpc_demo/<run_id>/logs/B01_run_log.txt
```

每张图的含义：

- `B01_angle_tracking.png`：显示 `q` 与 `q_target(t)` 随时间变化。
- `B01_angle_error.png`：显示 `q - q_target(t)` 随时间变化。
- `B01_torque.png`：显示每个控制步执行的 torque。
- `B01_best_cost.png`：显示每个控制步选中的 best horizon cost。

## 11. 验收标准

B01 任务完成必须满足：

- 有 simulation video。
- 有 angle tracking figure。
- 有 angle error figure。
- 有 torque figure。
- 有 best cost figure。
- 有 metrics CSV。
- 有 run log。
- README 或运行说明包含复现实验命令。
- 图像含义被解释清楚。

没有可视化输出的任务不算完成；没有 metrics 的视频不算完成；没有 README 复现命令的结果不算完成；没有解释图像含义的结果不算完成。
