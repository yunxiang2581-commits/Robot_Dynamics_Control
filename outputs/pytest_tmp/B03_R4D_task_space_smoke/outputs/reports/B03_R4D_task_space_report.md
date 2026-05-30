# B03-R4D Task-Space Warm-Start Smoke Report

## Summary

This smoke run tracks a two-link end-effector XY reference with sampling, zero-init iLQR-lite, and sampling-warm-start iLQR-lite.

## Metrics

| label | solver | warm_start | best_cost | runtime_ms | final_ee_error | mean_ee_error | max_ee_error | iterations | rollouts |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| sampling_cem | cem | False | 0.318333 | 4.7191 | 0.0561286 | 0.0292183 | 0.0561286 | 1 | 5 |
| zero_init_ilqr | ilqg_lite | False | 0.315707 | 109.376 | 0.0560796 | 0.0292308 | 0.0560796 | 2 | 6 |
| sampling_warm_start_ilqr | ilqg_lite | True | 0.315318 | 160.328 | 0.0560581 | 0.0292224 | 0.0560581 | 3 | 7 |

## Output Files

- metrics_csv: `outputs/metrics/B03_R4D_task_space_metrics.csv`
- cache_npz: `outputs/cache/B03_R4D_task_space_cache.npz`
- ee_trajectory_figure: `outputs/figures/B03_R4D_ee_trajectory_xy.png`
- ee_error_figure: `outputs/figures/B03_R4D_ee_tracking_error.png`
- control_figure: `outputs/figures/B03_R4D_control_sequence.png`
- cost_runtime_figure: `outputs/figures/B03_R4D_cost_runtime_comparison.png`

## Current Scope

- Tracks task-space XY position `p_ee(q)=[x_ee,y_ee]`.
- Uses finite-difference cost quadratization for iLQR-lite task-space cost.
- Does not modify B02 controller / benchmark / regression.
- Does not implement task-space Jacobian, QP, WBC, or humanoid control.