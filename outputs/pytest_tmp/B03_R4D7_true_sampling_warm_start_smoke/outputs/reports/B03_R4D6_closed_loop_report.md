# B03-R4D-6 Closed-Loop Task-Space MPC Smoke Report

## Summary

- target_source: `b03_lissajous`
- solver_family: `sampling_warm_start_ilqg`
- steps: 3
- final_ee_error: 0.318248
- mean_ee_error: 0.320311
- max_ee_error: 0.322382
- mean_runtime_ms: 176.978

## Output Files

- step_metrics_csv: `outputs/metrics/B03_R4D6_closed_loop_steps.csv`
- summary_metrics_csv: `outputs/metrics/B03_R4D6_closed_loop_summary.csv`
- cache_npz: `outputs/cache/B03_R4D6_closed_loop_cache.npz`
- ee_trajectory_figure: `outputs/figures/B03_R4D6_closed_loop_ee_xy.png`
- ee_error_figure: `outputs/figures/B03_R4D6_closed_loop_ee_error.png`
- control_figure: `outputs/figures/B03_R4D6_closed_loop_controls.png`
- runtime_figure: `outputs/figures/B03_R4D6_closed_loop_runtime.png`

## Current Scope

- Reuses existing B03/B02 task-space reference trajectories.
- Replans at each control step and executes only `solution.first_control`.
- Records actual closed-loop end-effector trajectory after MuJoCo step.
- Does not modify B02 controller / benchmark / regression.