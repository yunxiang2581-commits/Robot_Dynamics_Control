# B03-R4C-2B Sampling to iLQR Warm-Start Smoke Report

## Summary

This smoke run first generates a real sampling-family MPCSolution, then compares zero-init iLQR-lite against sampling-warm-start iLQR-lite on the same joint-space state-tracking problem.

## Metrics

| label | solver | warm_start | best_cost | initial_cost | final_cost | runtime_ms | rollouts | iterations | final_state_error_norm |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| sampling_cem | cem | False | 0.0874046 | 0.0874046 | 0.0874046 | 26.3412 | 12 | 1 | 0.0666574 |
| zero_init_ilqr | ilqg_lite | False | 1.54494e-07 | 2.51672e-07 | 1.54494e-07 | 39.7145 | 1 | 1 | 4.22653e-05 |
| sampling_warm_start_ilqr | ilqg_lite | True | 1.6959e-07 | 0.0953003 | 1.6959e-07 | 155.741 | 4 | 4 | 5.2147e-05 |

## Output Files

- metrics_csv: `outputs/metrics/B03_R4C2B_sampling_to_ilqr_metrics.csv`
- cache_npz: `outputs/cache/B03_R4C2B_sampling_to_ilqr_cache.npz`
- cost_runtime_figure: `outputs/figures/B03_R4C2B_cost_runtime_comparison.png`
- state_trajectory_figure: `outputs/figures/B03_R4C2B_state_trajectory_comparison.png`
- control_sequence_figure: `outputs/figures/B03_R4C2B_control_sequence_comparison.png`

## Current Scope

- Uses joint-space state tracking: `x=[q1,q2,dq1,dq2]`.
- Uses the same R4C one-step dynamics adapter for sampling rollouts and iLQR-lite rollouts.
- Does not modify the iLQR-lite core solver math.
- Does not modify the B02 controller, B02 benchmark, or B02 regression settings.
- Does not implement task-space end-effector tracking yet.
