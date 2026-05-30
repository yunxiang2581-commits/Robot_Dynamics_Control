# Step B03-R4D: Task-Space End-Effector Tracking

B03-R4D 的目标是把 R4C 的 joint-space state tracking 推进到二连杆末端轨迹控制：

```text
x = [q1,q2,dq1,dq2]
u = [tau1,tau2]
p_ee(q) = [x_ee,y_ee]
cost = ||p_ee(q)-p_ref|| + ||dq|| + ||u||
```

本阶段仍然是 two-link 学习型 smoke，不修改 B02 controller / benchmark / regression，不进入 task-space Jacobian、QP、WBC 或 humanoid 控制。

## R4D 拆分

```text
B03-R4D-1：最小 task-space MPCProblem
B03-R4D-2：sampling solver 跑末端轨迹 tracking smoke
B03-R4D-3：iLQR-lite 跑末端轨迹 tracking smoke
B03-R4D-4：sampling warm-start iLQR 末端轨迹对比
B03-R4D-5：补末端轨迹图、误差曲线、控制曲线、report
```

当前实现用一个轻量 runner 覆盖这五个 smoke 目标：

```text
projects/B_mujoco_mpc_study/simulator/scripts/run_B03_task_space_warm_start_smoke.py
```

## 关键实现

### `compute_end_effector_xy(env, x)`

输入：

- `env`: `TwoLinkEnv` 或测试 fake env；
- `x`: shape `(4,)`，约定 `x=[q1,q2,dq1,dq2]`。

输出：

- `p_ee`: shape `(2,)`，即 `[x_ee,y_ee]`。

函数会保存并恢复 env 状态，因此用于 cost 查询时不会污染真实 rollout。

### `build_task_space_reference(...)`

构造末端参考轨迹 `p_ref[k]`，当前支持：

- `hold`: 保持初始末端位置；
- `line`: 末端做小直线偏移；
- `circle`: 围绕初始末端点走一小段圆轨迹。

### `build_task_space_tracking_problem(...)`

构造统一 `MPCProblem`，其中：

- `target_horizon` 存 `target_ee_positions`，shape `(H+1,2)`；
- `rollout_cost_fn=evaluate_task_space_rollouts` 供 CEM/MPPI 使用；
- `metadata["stage_cost_fn"]` 和 `metadata["terminal_cost_fn"]` 供 iLQR-lite 使用；
- `metadata["end_effector_fn"]` 用来把 predicted states 映射成 predicted EE trajectory；
- `metadata["initial_controls"]` 保存 zero-init 或 sampling warm-start 初值。

### iLQR-lite 通用 cost 路径

`ILQGLiteSolver` 现在保留原 state-quadratic 默认路径，同时增加可选通用 cost 路径：

```text
metadata["stage_cost_fn"]
metadata["terminal_cost_fn"]
-> finite-difference cost quadratization
-> backward pass / forward pass
```

这使 R4D 可以优化非线性的 task-space cost：

```text
||p_ee(q)-p_ref||
```

而不需要把它伪装成 joint-space `x_ref/Q/R/Q_terminal`。

## 输出文件

默认输出到：

```text
projects/B_mujoco_mpc_study/outputs/runs/B03_task_space_warm_start_smoke/<timestamp>/
```

本次 smoke 约定输出：

```text
outputs/metrics/B03_R4D_task_space_metrics.csv
outputs/cache/B03_R4D_task_space_cache.npz
outputs/figures/B03_R4D_ee_trajectory_xy.png
outputs/figures/B03_R4D_ee_tracking_error.png
outputs/figures/B03_R4D_control_sequence.png
outputs/figures/B03_R4D_cost_runtime_comparison.png
outputs/reports/B03_R4D_task_space_report.md
```

CSV 指标包括：

- `best_cost`
- `initial_cost`
- `final_cost`
- `runtime_ms`
- `num_rollouts`
- `num_iterations`
- `final_ee_error`
- `mean_ee_error`
- `max_ee_error`
- `control_energy`
- `warm_start_used`
- `warm_start_source`

NPZ cache 保存 target EE trajectory、三路 predicted states、predicted controls、predicted EE positions，以及 warm-start iLQR 的 initial controls。

## 运行命令

```powershell
D:\anaconda\envs\mujoco_py311\python.exe projects/B_mujoco_mpc_study/simulator/scripts/run_B03_task_space_warm_start_smoke.py `
  --output-dir outputs/pytest_tmp/B03_R4D_task_space_smoke `
  --horizon 4 `
  --num-candidates 5 `
  --sampling-iterations 1 `
  --sampling-std 0.1 `
  --torque-limit 1.0 `
  --seed 3 `
  --log-level INFO
```

## 当前边界

本步没有做：

- 长时间 benchmark；
- 多 seed sweep；
- MP4 / viewer；
- task-space Jacobian 解析控制；
- QP / WBC；
- B02 controller 或 benchmark/regression 修改。

## 建议下一步

下一步可以做 **B03-R4D-6: closed-loop task-space MPC rollout**：

1. 不只比较一次 open-loop solve，而是在短仿真中每步重新规划；
2. 执行每次 solver 的 `first_control`；
3. 记录真实 closed-loop `actual_ee_positions`；
4. 生成 target vs actual 末端轨迹图和 tracking error 曲线。
