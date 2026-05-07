# A IK Interface

## 1. 定位

IK interface 回答一个问题：给定 target，关节轨迹怎么来。

A04 和 A05 都归入这个接口：

- A04: `solver_type=dls`，无约束 differential IK。
- A05: `solver_type=qp_scipy` 或 `qp_osqp`，带 box limit 的 QP-IK。

## 2. IkRequest / IkResult

`IkRequest` 包含：

- solver_type。
- task_mode。
- site_name。
- q_init。
- TargetDefinition。
- weights。
- limits。
- solver_config。

`IkResult` 包含：

- q_traj。
- error_rows。
- constraint_rows。
- converged。
- stop_reason。
- solver_status。
- trajectory_source。

## 3. DLS 数学关系

```text
e_pos = p_target - p_current
dq = J^T (J J^T + lambda^2 I)^-1 gain e_pos
q_next = integrate(q, dq, dt)
```

DLS 风险：

- 奇异点附近 damping 太小会不稳定。
- gain 太大会振荡。
- 无约束 DLS 不处理 joint limits。

## 4. QP-IK 数学关系

```text
minimize ||J_task dq - gain e_task||^2 + damping ||dq||^2
subject to lower <= dq <= upper
```

其中 bounds 后续来自 velocity limit 与 position limit。

## 5. 与 mink 的关系

mink 使用 FrameTask、PostureTask、limits 和 solve_ik。A 项目不调用 mink，而是在 `ik_interface.py` 中保留同类概念的学习骨架。

## 6. 验证标准

- `J_task.shape == (m, nv)`。
- `dq.shape == (nv,)`。
- `q_traj.shape == (N, nq)`。
- error norm 下降。
- QP constraint violation 可解释。
- IK interface 不启动 viewer，不写 `data.ctrl`。
