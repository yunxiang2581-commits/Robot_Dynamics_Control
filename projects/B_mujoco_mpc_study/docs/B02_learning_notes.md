# B02 二连杆 MPC Tracking 学习笔记

## 1. B02 任务目标

B02 的目标是把 B01 的单关节角度跟踪扩展到二维平面二连杆末端轨迹跟踪。

核心闭环仍然是：

```text
当前状态 -> rollout 预测未来 -> horizon cost 评价 -> predictive sampling 选择 torque 序列 -> 只执行第一步 -> 下一轮重新规划
```

但 B02 的 residual 不再是单个角度误差，而是末端位置误差：

```text
r_ee(t) = p_ee(q(t)) - p_target(t)
```

## 2. 二连杆物理模型

B02 使用一个平面二连杆机械臂：

```text
joint1: shoulder hinge
joint2: elbow hinge
link1 length: l1
link2 length: l2
```

控制输入是两个关节的力矩：

```text
u = [tau1, tau2]
```

## 3. 状态定义

状态为：

```text
x = [q1, q2, dq1, dq2]
```

其中：

- `q1` 是第一关节角。
- `q2` 是第二关节角。
- `dq1` 是第一关节角速度。
- `dq2` 是第二关节角速度。

## 4. 末端位置

二连杆末端位置为：

```text
p_ee = [x_ee, y_ee]
```

后续可以先用 MuJoCo site 读取末端位置，再和手写 FK 公式做对照。

手写 FK 参考：

```text
x_ee = l1 cos(q1) + l2 cos(q1 + q2)
y_ee = l1 sin(q1) + l2 sin(q1 + q2)
```

## 5. 目标轨迹

B02 不建议先做随机目标点，建议从可解释轨迹开始：

```text
p_target(t) = [x_target(t), y_target(t)]
```

第一版建议使用圆形或小幅 smooth trajectory：

```text
x_target(t) = center_x + radius cos(omega t)
y_target(t) = center_y + radius sin(omega t)
```

## 6. rollout 输入输出

rollout 输入：

```text
initial_state = [q1, q2, dq1, dq2]
torque_sequence = [[tau1_0, tau2_0], ..., [tau1_{H-1}, tau2_{H-1}]]
target_sequence = [p_target_1, ..., p_target_H]
```

rollout 输出：

```text
states = [x_0, x_1, ..., x_H]
ee_positions = [p_ee_0, p_ee_1, ..., p_ee_H]
```

## 7. horizon cost 公式

第一版 B02 cost 建议为：

```text
J = sum_k w_ee * ||p_ee(q_k) - p_target(t_k)||^2
  + sum_k w_dq * ||dq_k||^2
  + sum_k w_tau * ||tau_k||^2
  + w_terminal * ||p_ee(q_H) - p_target(t_H)||^2
```

物理意义：

- `w_ee` 惩罚末端偏离目标轨迹。
- `w_dq` 抑制关节速度过大。
- `w_tau` 抑制控制力矩过大。
- `w_terminal` 强调 horizon 末端的目标达成。

## 8. B02 与 B01 的关系

B01：

```text
q -> q_target(t)
```

B02：

```text
p_ee(q) -> p_target(t)
```

B02 增加的关键概念是：

- task-space residual。
- 二维目标轨迹。
- 两个 actuator 同时优化。
- 末端轨迹图和末端误差图。

## 9. 可视化输出要求

B02 每次验收 run 必须输出：

```text
outputs/runs/B02_two_link_mpc_tracking_demo/<run_id>/videos/B02_two_link_mpc_tracking_demo.mp4
outputs/runs/B02_two_link_mpc_tracking_demo/<run_id>/figures/B02_ee_trajectory_xy.png
outputs/runs/B02_two_link_mpc_tracking_demo/<run_id>/figures/B02_ee_tracking_error.png
outputs/runs/B02_two_link_mpc_tracking_demo/<run_id>/figures/B02_joint_torque.png
outputs/runs/B02_two_link_mpc_tracking_demo/<run_id>/metrics/B02_metrics.csv
outputs/runs/B02_two_link_mpc_tracking_demo/<run_id>/logs/B02_control_log.txt
```

## 10. 验收标准

B02 第一版验收建议：

- 能加载二连杆 MuJoCo XML。
- 能读取状态 `[q1, q2, dq1, dq2]`。
- 能读取末端 site 位置。
- 能生成目标轨迹 `p_target(t)`。
- 能输出 video、figures、metrics CSV、run log。
- figures 必须解释末端轨迹、末端误差和双关节 torque。

注意：

```text
B02 初期先追求可解释闭环，不追求复杂最优控制性能。
```
