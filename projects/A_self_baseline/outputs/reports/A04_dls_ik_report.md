# A04 DLS 差分逆运动学报告

## 目标

- 任务: 让 attachment_site 从初始位置移动到目标位置。
- 模式: position-only DLS IK。
- 边界: 不做 QP、不做 actuator tracking、不做 MuJoCo 控制、不做 collision avoidance。

## 输入

- 目标 site: attachment_site
- 任务模式: position
- 目标位置偏移: 0.03,0.00,0.00
- 目标位置: [0.5219992984124819, 0.1339978254660598, 0.48800036731899227]

## 求解器参数

- damping: 0.001
- gain: 1.0
- dt: 0.2
- max_iter: 100
- tolerance: 0.001

## 结果

- 是否收敛: True
- 停止原因: tolerance_reached
- 初始位置误差范数: 0.02999999999999997
- 最终位置误差范数: 0.0008449797498493482
- q 轨迹长度: 17

## 输出文件

- q 轨迹: D:\project\Robot_Dynamics_Control\projects\A_self_baseline\outputs\trajectories\A04_dls_ik_q_traj.npy
- 误差日志: D:\project\Robot_Dynamics_Control\projects\A_self_baseline\outputs\logs\A04_dls_ik_error.csv
- 误差图: D:\project\Robot_Dynamics_Control\projects\A_self_baseline\outputs\figures\A04_dls_ik_error.png

## DLS 公式

```text
e = x_target - x_current
dq = J.T @ solve(J @ J.T + damping^2 I, gain * e)
q_next = q + dq * dt
```

## 说明

- A04 使用 A03 已验证过的 site Jacobian。
- A04 第一版只做位置 IK，不控制姿态。
- DLS 中的 damping 用于降低奇异附近 dq 过大的风险。
- A05 才会进入 QP-IK、约束和关节限制。
