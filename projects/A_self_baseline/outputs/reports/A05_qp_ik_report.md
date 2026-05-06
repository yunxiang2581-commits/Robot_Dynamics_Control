# A05 Task + Limit + QP-IK Report

## 目标

- 从 A04 的无约束 DLS IK 过渡到带 task weight 和 limit 的 QP-IK。
- 第一版只生成受约束 IK 轨迹，不进入 actuator tracking、MuJoCo 控制、collision avoidance 或 video。

## 输入与配置

- target site: attachment_site
- task mode: position
- solver: scipy
- target offset: [0.03, 0.0, 0.0]
- dt: 0.02
- max_iters: 300
- tolerance: 0.001
- regularization: 0.0001
- velocity_limit: 0.5
- position_margin: 0.05
- posture_weight: 0.02

## QP 形式

```text
minimize 1/2 dq^T H dq + c^T dq
subject to lower <= dq <= upper

H = J_task.T @ J_task + regularization * I
c = -J_task.T @ v_task
```

## 结果

- converged: True
- stop_reason: tolerance_reached
- initial position error: 0.02999999999999997
- final position error: 0.0009963500438705801
- q trajectory shape: (176, 6)
- max constraint violation: 0.0

## 输出文件

- trajectory: D:\project\Robot_Dynamics_Control\projects\A_self_baseline\outputs\trajectories\A05_qp_ik_q_traj.npy
- error log: D:\project\Robot_Dynamics_Control\projects\A_self_baseline\outputs\logs\A05_qp_ik_error.csv
- constraint log: D:\project\Robot_Dynamics_Control\projects\A_self_baseline\outputs\logs\A05_qp_ik_constraints.csv
- error figure: D:\project\Robot_Dynamics_Control\projects\A_self_baseline\outputs\figures\A05_qp_ik_error.png

## 与 A04 / A06 / A07 的关系

- A04 提供无约束 DLS baseline。
- A05 输出受约束 QP-IK q trajectory。
- A06 后续可以把 A05 trajectory 作为 target tracking 输入。
- A07 后续才把 trajectory 送入 actuator tracking。
