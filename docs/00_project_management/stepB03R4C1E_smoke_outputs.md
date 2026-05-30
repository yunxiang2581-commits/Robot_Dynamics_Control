# Step B03-R4C-1E: State Tracking Smoke Outputs

## 本步目标

B03-R4C-1E 的目标是在 R4C-1C/1D 已经跑通 `dynamics_fn` 和 `MPCProblem` 的基础上，补齐最小可复盘输出链路。

本步仍然只做短 horizon state tracking smoke，不做 task-space tracking，不生成正式 MP4，不修改 B02 controller 或 B02 benchmark/regression 配置。

## 本步输出

脚本输出目录仍使用：

```text
<run_dir>/outputs/
  cache/
  figures/
  reports/
  metrics/
```

本步新增真实写入：

```text
outputs/metrics/B03_R4C_ilqr_cost_history.csv
outputs/metrics/B03_R4C_ilqr_two_link_smoke_metrics.csv
outputs/cache/B03_R4C_two_link_state_tracking_placeholder.npz
```

其中：

- `B03_R4C_ilqr_cost_history.csv` 记录 `iteration,cost`；
- `B03_R4C_ilqr_two_link_smoke_metrics.csv` 记录 solver、horizon、维度、best cost、runtime、rollout 数、控制幅值和最终状态误差；
- `.npz` cache 保存 `predicted_states`、`predicted_controls`、`x_refs`、`first_control` 和 `cost_history`。

## 本步实现内容

新增 `write_smoke_outputs(solution, problem, output_paths)`：

1. 从 `solution.metadata["cost_history"]` 读取 cost history；
2. 写入 cost history CSV；
3. 从 `solution` 和 `problem` 计算单行 metrics；
4. 写入 metrics CSV；
5. 保存 state/control/reference cache；
6. 在日志中打印输出路径。

同时更新：

- `build_output_paths(...)` 增加 `cost_history_csv`；
- `run_smoke_simulation(...)` 支持传入 `config` 和 `output_paths`；
- `main(...)` 读取 YAML config，并在 smoke solve 后写入输出。

## 验证命令

```bash
D:\anaconda\envs\mujoco_py311\python.exe -m py_compile \
  projects/B_mujoco_mpc_study/simulator/scripts/run_B03_ilqr_lite_two_link_smoke.py

D:\anaconda\envs\mujoco_py311\python.exe -m pytest -q \
  projects/B_mujoco_mpc_study/tests/test_B03_ilqr_two_link_smoke_skeleton.py \
  projects/B_mujoco_mpc_study/tests/test_B03_ilqr_two_link_state_adapter.py \
  projects/B_mujoco_mpc_study/tests/test_B03_ilqg_solver.py
```

真实 MuJoCo smoke：

```bash
D:\anaconda\envs\mujoco_py311\python.exe \
  projects/B_mujoco_mpc_study/simulator/scripts/run_B03_ilqr_lite_two_link_smoke.py \
  --output-dir outputs/pytest_tmp/B03_R4C1E_real_smoke \
  --log-level INFO
```

示例输出摘要：

```text
cost history rows: 2
horizon: 32
predicted_states: (33, 4)
predicted_controls: (32, 2)
best_cost: 1.5449372487697226e-07
success: True
```

## 当前仍保留的 TODO

- `plot_trajectory(...)` 仍是占位；
- `plot_cost_history(...)` 仍是占位；
- 还没有 task-space end-effector tracking；
- 还没有 CEM/MPPI warm-start；
- 还没有长 benchmark；
- 还没有正式 MP4。

## 下一步 B03-R4C-1F

建议下一步补最小可视化：

1. 读取 `solution.predicted_states` 和 `x_refs`；
2. 画 `q1/q2` state tracking 曲线；
3. 画 cost history 曲线；
4. 继续保持短 smoke，不进入正式视频。
