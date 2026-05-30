# Step B03-R4D-6: Closed-Loop Task-Space MPC Rollout

B03-R4D-6 的目标是把 R4D-1~5 的一次 open-loop task-space solve，推进到真正的 MPC 闭环结构：

```text
for each control step:
  read current real state x_real
  build existing target trajectory horizon p_ref[t:t+H]
  solve task-space MPCProblem
  execute only solution.first_control
  record actual p_ee after MuJoCo step
```

本步复用已有复杂末端轨迹，不再使用 R4D-1~5 的局部小圆弧 reference。

## 新增脚本

```text
projects/B_mujoco_mpc_study/simulator/scripts/run_B03_task_space_closed_loop_smoke.py
```

默认目标轨迹来自 B03 solver ladder：

```text
target_source = b03_lissajous
```

也支持 B02 风格轨迹：

```text
b02_figure8
b02_circle
b02_sinusoidal
b02_custom
```

## 关键逻辑

### `build_closed_loop_target_horizon(...)`

输入：

- `target_source`；
- `current_time`；
- `horizon`；
- `dt`；
- config 中已有 `target` 或 `target_trajectory` 配置。

输出：

```text
target_ee_positions.shape == (H+1, 2)
```

其中第 0 项是当前控制步目标，后面 H 项是 MPC 预测窗口内的未来目标。

### `build_task_space_tracking_problem(..., target_ee_positions=...)`

R4D problem builder 现在支持外部传入完整 target horizon。这样 R4D-6 可以复用已有复杂轨迹，而不是从当前末端位置临时生成小 reference。

### `run_closed_loop_task_space_rollout(...)`

闭环执行流程：

```text
target_horizon = build_closed_loop_target_horizon(...)
problem = build_task_space_tracking_problem(..., target_ee_positions=target_horizon)
solution = solver.solve(problem)
env.step(solution.first_control)
actual_ee = env.get_end_effector_position()
record step metrics
```

当前支持 solver：

```text
cem
mppi_lite
ilqg_lite
sampling_warm_start_ilqg
```

## 输出文件

默认输出到：

```text
projects/B_mujoco_mpc_study/outputs/runs/B03_task_space_closed_loop_smoke/<timestamp>/
```

本次 smoke 输出结构：

```text
outputs/metrics/B03_R4D6_closed_loop_steps.csv
outputs/metrics/B03_R4D6_closed_loop_summary.csv
outputs/cache/B03_R4D6_closed_loop_cache.npz
outputs/figures/B03_R4D6_closed_loop_ee_xy.png
outputs/figures/B03_R4D6_closed_loop_ee_error.png
outputs/figures/B03_R4D6_closed_loop_controls.png
outputs/figures/B03_R4D6_closed_loop_runtime.png
outputs/reports/B03_R4D6_closed_loop_report.md
```

关键指标：

- `final_ee_error`
- `mean_ee_error`
- `max_ee_error`
- `mean_runtime_ms`
- `max_runtime_ms`
- `mean_abs_torque`
- `max_abs_torque`
- `control_energy`

## 运行命令

```powershell
D:\anaconda\envs\mujoco_py311\python.exe projects/B_mujoco_mpc_study/simulator/scripts/run_B03_task_space_closed_loop_smoke.py `
  --output-dir outputs/pytest_tmp/B03_R4D6_closed_loop_smoke `
  --target-source b03_lissajous `
  --solver-family cem `
  --num-steps 6 `
  --horizon 4 `
  --num-candidates 5 `
  --sampling-iterations 1 `
  --sampling-std 0.1 `
  --torque-limit 1.0 `
  --seed 4 `
  --log-level INFO
```

## 当前边界

本步仍然是 smoke：

- 不做长时间 benchmark；
- 不输出 MP4；
- 不修改 B02 controller / benchmark / regression；
- 不进入 task-space Jacobian / QP / WBC；
- 默认短 horizon 和少量候选，优先验证 closed-loop 数据链路。

## 建议下一步

下一步可以做 **B03-R4D-7: closed-loop solver comparison**：

1. 对 `cem / mppi_lite / ilqg_lite / sampling_warm_start_ilqg` 跑同一条 lissajous；
2. 输出 solver 间的 closed-loop `mean_ee_error / runtime / torque` 对比；
3. 增加 `b02_figure8` 作为第二条复杂轨迹。
