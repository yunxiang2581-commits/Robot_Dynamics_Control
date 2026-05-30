# B03-R4C iLQR Two-Link Smoke Report

## Summary

This smoke run connects the B03 mini iLQR-lite solver to the B02 two-link MuJoCo environment for joint-space state tracking.
It is still a short learning smoke, not a task-space benchmark or video demo.

## Key Metrics

- solver_name: `ilqg_lite`
- success: `True`
- message: `cost improvement below tolerance`
- termination_reason: `tolerance`
- warm_start_used: `True`
- warm_start_source: `fake_sampling_warm_start`
- horizon: `32`
- state_dim: `4`
- control_dim: `2`
- best_cost: `1.15647520965e-07`
- initial_cost: `0.000206021128805`
- final_cost: `1.15647520965e-07`
- cost_entries: `4`
- runtime_ms: `121.021`
- num_rollouts: `4`
- num_iterations: `3`
- max_abs_control: `0.000334929606743`
- final_state_error_norm: `6.06184731914e-05`

## Output Files

- metrics_csv: `outputs/metrics/B03_R4C_ilqr_two_link_smoke_metrics.csv`
- cost_history_csv: `outputs/metrics/B03_R4C_ilqr_cost_history.csv`
- state_cache: `outputs/cache/B03_R4C_two_link_state_tracking_placeholder.npz`
- trajectory_figure: `outputs/figures/B03_R4C_two_link_state_trajectory_todo.png`
- cost_figure: `outputs/figures/B03_R4C_ilqr_cost_history_todo.png`

## Current Scope

- Uses joint-space state tracking: `x=[q1,q2,dq1,dq2]`.
- Uses torque input: `u=[tau1,tau2]`.
- Does not implement task-space end-effector tracking yet.
- Does not generate a formal MP4 or long benchmark yet.
- Does not modify the B02 controller core logic.
