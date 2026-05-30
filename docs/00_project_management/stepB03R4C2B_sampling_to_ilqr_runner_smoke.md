# Step B03-R4C-2B: Sampling-to-iLQR Warm-Start Runner Smoke

B03-R4C-2B 的目标是在 R4C-2 已经支持 `previous_solution` warm-start 的基础上，补一个真实 sampling-to-iLQR 对照 runner：

```text
CEM/MPPI state-tracking MPCProblem
-> sampling_solution.predicted_controls
-> shifted iLQR initial_controls
-> zero-init iLQR vs sampling-warm-start iLQR comparison
```

本步仍然是 joint-space state tracking smoke，不改变 iLQR-lite solver 核心数学，不修改 B02 controller / benchmark / regression，也不进入 task-space end-effector tracking。

## 新增脚本

新增：

```text
projects/B_mujoco_mpc_study/simulator/scripts/run_B03_sampling_to_ilqr_warm_start_smoke.py
```

脚本职责：

1. 使用同一条 two-link state reference `x_ref`；
2. 用同一个 R4C one-step dynamics adapter `dynamics_fn(x,u)->x_next`；
3. 先运行 sampling-family solver，默认 `CEMShootingSolver`，也支持 `MPPILiteSolver`；
4. 再运行 zero-init `ILQGLiteSolver`；
5. 最后把真实 `sampling_solution` 作为 `previous_solution` 传给 iLQR-lite warm-start 路径；
6. 写出 CSV / NPZ / PNG 对比图 / Markdown report。

## 关键函数

### `evaluate_state_tracking_rollouts(...)`

给 CEM/MPPI 使用的批量 rollout cost adapter。

输入：

- `candidate_controls`: shape `(N, H, 2)`；
- `problem.current_state`: 当前 two-link state，约定 `x=[q1,q2,dq1,dq2]`；
- `problem.dynamics_fn`: R4C-1C 的无副作用 MuJoCo one-step adapter；
- `x_refs / Q / R / Q_terminal`: state tracking cost 数据。

输出：

- `costs`: shape `(N,)`；
- `predicted_states`: shape `(N, H+1, 4)`。

### `build_sampling_state_tracking_problem(...)`

在 R4C `build_state_tracking_problem(...)` 基础上补 sampling 需要的：

- `solver_config`；
- `rollout_cost_fn=evaluate_state_tracking_rollouts`；
- `metadata["sampling_solver_family"]`；
- `warm_start_used=False` 和 `warm_start_source="zeros"`。

### `run_sampling_to_ilqr_comparison(...)`

三路对照入口：

1. `sampling_cem` 或 `sampling_mppi_lite`；
2. `zero_init_ilqr`；
3. `sampling_warm_start_ilqr`。

每一路都会在相同初始环境状态下求解。脚本会保存并恢复 env snapshot，避免第一路 sampling rollout 污染后续 iLQR 对照。

## 输出文件

默认输出到：

```text
projects/B_mujoco_mpc_study/outputs/runs/B03_sampling_to_ilqr_warm_start_smoke/<timestamp>/
```

本次 smoke 示例输出：

```text
outputs/pytest_tmp/B03_R4C2B_figures_smoke/outputs/metrics/B03_R4C2B_sampling_to_ilqr_metrics.csv
outputs/pytest_tmp/B03_R4C2B_figures_smoke/outputs/cache/B03_R4C2B_sampling_to_ilqr_cache.npz
outputs/pytest_tmp/B03_R4C2B_figures_smoke/outputs/figures/B03_R4C2B_cost_runtime_comparison.png
outputs/pytest_tmp/B03_R4C2B_figures_smoke/outputs/figures/B03_R4C2B_state_trajectory_comparison.png
outputs/pytest_tmp/B03_R4C2B_figures_smoke/outputs/figures/B03_R4C2B_control_sequence_comparison.png
outputs/pytest_tmp/B03_R4C2B_figures_smoke/outputs/reports/B03_R4C2B_sampling_to_ilqr_report.md
```

CSV 指标包括：

- `best_cost`
- `initial_cost`
- `final_cost`
- `runtime_ms`
- `num_rollouts`
- `num_iterations`
- `final_state_error_norm`
- `warm_start_used`
- `warm_start_source`

NPZ cache 保存：

- sampling predicted states / controls；
- zero-init iLQR initial controls / predicted states / predicted controls；
- warm-start iLQR initial controls / predicted states / predicted controls；
- `x_refs`。

PNG 图包括：

- `B03_R4C2B_cost_runtime_comparison.png`：对比 sampling、zero-init iLQR、sampling-warm-start iLQR 的 `best_cost`、`initial_cost`、`runtime_ms` 和最终状态误差；
- `B03_R4C2B_state_trajectory_comparison.png`：对比 `q1/q2` 的 reference、sampling rollout、zero-init iLQR 和 warm-start iLQR 预测轨迹；
- `B03_R4C2B_control_sequence_comparison.png`：对比 sampling 真实控制序列、shift 后传入 iLQR 的 warm-start 初值、zero-init iLQR 优化控制和 warm-start iLQR 优化控制。

其中控制序列图最适合检查 warm-start 链路：

```text
sampling predicted_controls
-> shifted warm-start initial_controls
-> warm-start iLQR optimized_controls
```

状态轨迹图用于确认三路 solver 仍在跟踪同一条 `x_ref`。由于当前默认 reference 基本保持初始状态，`q1/q2` 轨迹变化会比较小，这是本 smoke 的预期现象。

## 运行命令

```powershell
D:\anaconda\envs\mujoco_py311\python.exe projects/B_mujoco_mpc_study/simulator/scripts/run_B03_sampling_to_ilqr_warm_start_smoke.py `
  --output-dir outputs/pytest_tmp/B03_R4C2B_figures_smoke `
  --num-candidates 12 `
  --sampling-iterations 1 `
  --sampling-std 0.15 `
  --torque-limit 1.0 `
  --seed 7 `
  --log-level INFO
```

示例结果：

```text
sampling_cem:
  best_cost=0.08740462459974174
  runtime_ms=26.0877
  num_rollouts=12
  num_iterations=1
  final_state_error_norm=0.06665738293841802

zero_init_ilqr:
  best_cost=1.5449372487697226e-07
  initial_cost=2.516716749963712e-07
  final_cost=1.5449372487697226e-07
  runtime_ms=38.612
  warm_start_used=False
  warm_start_source=zeros

sampling_warm_start_ilqr:
  best_cost=1.6958950622978133e-07
  initial_cost=0.095300328258794
  final_cost=1.6958950622978133e-07
  runtime_ms=150.979
  warm_start_used=True
  warm_start_source=cem
```

说明：当前 R4C 默认 `x_ref` 基本是保持初始状态，因此 zero-init iLQR 已经非常接近最优。这个 smoke 的重点是验证真实 sampling solution 能进入 iLQR warm-start 链路，并输出可复盘对照；它不表示当前参数下 warm-start 一定优于 zero-init。

## 验证

已运行：

```powershell
D:\anaconda\envs\mujoco_py311\python.exe -m py_compile `
  projects/B_mujoco_mpc_study/simulator/scripts/run_B03_sampling_to_ilqr_warm_start_smoke.py `
  projects/B_mujoco_mpc_study/simulator/scripts/run_B03_ilqr_lite_two_link_smoke.py

D:\anaconda\envs\mujoco_py311\python.exe -m pytest -q `
  projects/B_mujoco_mpc_study/tests/test_B03_sampling_to_ilqr_warm_start_smoke.py `
  projects/B_mujoco_mpc_study/tests/test_B03_ilqr_two_link_smoke_skeleton.py `
  projects/B_mujoco_mpc_study/tests/test_B03_ilqr_two_link_state_adapter.py `
  projects/B_mujoco_mpc_study/tests/test_B03_ilqg_solver.py `
  projects/B_mujoco_mpc_study/tests/test_B03_sampling_mpc_solvers.py
```

结果：

```text
51 passed in 1.77s
```

## 当前边界

本步没有做：

- 长时间 benchmark；
- 自动调参；
- MP4 / viewer；
- task-space end-effector tracking；
- B02 controller 或 benchmark/regression 修改。

## 建议下一步

下一步可以做 **B03-R4C-2C: nontrivial state reference warm-start comparison**：

1. 给 R4C smoke 增加一个小幅非平凡 joint-space reference，例如 q1/q2 缓慢偏移；
2. 在相同参考下比较 zero-init iLQR 与 CEM/MPPI warm-start iLQR；
3. 观察 warm-start 是否能降低 initial cost、减少 iLQR 迭代或改善 line-search 行为。

也可以直接进入 **B03-R4D: task-space end-effector tracking**。
