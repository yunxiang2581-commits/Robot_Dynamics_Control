# Project B 可视化输出要求

## Project B 可视化总原则

Project B 的每个任务都必须做到：

```text
算法可解释、仿真可观看、误差可量化、结果可复现。
```

因此，B01-B07 不再接受“只有代码”或“只有日志”的完成方式。每个任务至少必须包含：

1. simulation video。
2. tracking / error / torque / cost / runtime figures。
3. metrics CSV。
4. run log。
5. README 复现实验命令。

## 统一输出目录

所有任务必须按任务名和运行时间单独保存。统一格式：

```text
outputs/runs/<task_name>/<run_id>/
├── videos/
├── figures/
├── logs/
└── metrics/
```

其中：

- `<task_name>` 例如 `B01_single_joint_mpc_demo`。
- `<run_id>` 默认使用当前时间 `YYYYMMDD_HHMMSS`。
- 每个任务的输出文件名仍必须带任务编号，例如 `B01_`、`B04_`、`B07_`，便于复盘和对比。

## 每个任务的最低可视化要求

### B01_single_joint_mpc_demo

```text
outputs/runs/B01_single_joint_mpc_demo/<run_id>/videos/B01_single_joint_mpc_demo.mp4
outputs/runs/B01_single_joint_mpc_demo/<run_id>/figures/B01_angle_tracking.png
outputs/runs/B01_single_joint_mpc_demo/<run_id>/figures/B01_angle_error.png
outputs/runs/B01_single_joint_mpc_demo/<run_id>/figures/B01_torque.png
outputs/runs/B01_single_joint_mpc_demo/<run_id>/figures/B01_best_cost.png
outputs/runs/B01_single_joint_mpc_demo/<run_id>/metrics/B01_metrics.csv
outputs/runs/B01_single_joint_mpc_demo/<run_id>/logs/B01_run_log.txt
```

### B02_two_link_mpc_tracking_demo

```text
outputs/runs/B02_two_link_mpc_tracking_demo/<run_id>/videos/B02_two_link_mpc_tracking_demo.mp4
outputs/runs/B02_two_link_mpc_tracking_demo/<run_id>/figures/B02_ee_trajectory_xy.png
outputs/runs/B02_two_link_mpc_tracking_demo/<run_id>/figures/B02_ee_tracking_error.png
outputs/runs/B02_two_link_mpc_tracking_demo/<run_id>/figures/B02_joint_torque.png
outputs/runs/B02_two_link_mpc_tracking_demo/<run_id>/metrics/B02_metrics.csv
outputs/runs/B02_two_link_mpc_tracking_demo/<run_id>/logs/B02_control_log.txt
```

### B03_rollout_predictive_sampling_demo

```text
outputs/runs/B03_rollout_predictive_sampling_demo/<run_id>/videos/B03_rollout_predictive_sampling_demo.mp4
outputs/runs/B03_rollout_predictive_sampling_demo/<run_id>/figures/B03_rollout_candidates.png
outputs/runs/B03_rollout_predictive_sampling_demo/<run_id>/figures/B03_rollout_costs.png
outputs/runs/B03_rollout_predictive_sampling_demo/<run_id>/figures/B03_selected_rollout.png
outputs/runs/B03_rollout_predictive_sampling_demo/<run_id>/metrics/B03_metrics.csv
outputs/runs/B03_rollout_predictive_sampling_demo/<run_id>/logs/B03_selected_rollout_log.txt
```

### B04_cart_double_inverted_pendulum_mpc

```text
outputs/runs/B04_cart_double_inverted_pendulum_mpc/<run_id>/videos/B04_cart_double_inverted_pendulum_mpc.mp4
outputs/runs/B04_cart_double_inverted_pendulum_mpc/<run_id>/figures/B04_cart_position.png
outputs/runs/B04_cart_double_inverted_pendulum_mpc/<run_id>/figures/B04_pendulum_angles.png
outputs/runs/B04_cart_double_inverted_pendulum_mpc/<run_id>/figures/B04_angle_errors.png
outputs/runs/B04_cart_double_inverted_pendulum_mpc/<run_id>/figures/B04_control_force.png
outputs/runs/B04_cart_double_inverted_pendulum_mpc/<run_id>/metrics/B04_cart_double_pendulum_metrics.csv
outputs/runs/B04_cart_double_inverted_pendulum_mpc/<run_id>/logs/B04_cart_double_pendulum_run_log.txt
```

### B05_openloong_model_mpc_setup

```text
outputs/runs/B05_openloong_model_mpc_setup/<run_id>/videos/B05_openloong_passive_or_static_preview.mp4
outputs/runs/B05_openloong_model_mpc_setup/<run_id>/figures/B05_body_site_map.png
outputs/runs/B05_openloong_model_mpc_setup/<run_id>/figures/B05_actuator_index_map.png
outputs/runs/B05_openloong_model_mpc_setup/<run_id>/metrics/B05_model_dimensions.csv
outputs/runs/B05_openloong_model_mpc_setup/<run_id>/logs/B05_openloong_model_summary.txt
```

### B06_openloong_standing_balance_mpc

```text
outputs/runs/B06_openloong_standing_balance_mpc/<run_id>/videos/B06_openloong_standing_balance_mpc.mp4
outputs/runs/B06_openloong_standing_balance_mpc/<run_id>/figures/B06_pelvis_height_error.png
outputs/runs/B06_openloong_standing_balance_mpc/<run_id>/figures/B06_torso_orientation_error.png
outputs/runs/B06_openloong_standing_balance_mpc/<run_id>/figures/B06_joint_torque.png
outputs/runs/B06_openloong_standing_balance_mpc/<run_id>/figures/B06_foot_contact_summary.png
outputs/runs/B06_openloong_standing_balance_mpc/<run_id>/metrics/B06_balance_metrics.csv
outputs/runs/B06_openloong_standing_balance_mpc/<run_id>/logs/B06_balance_run_log.txt
```

### B07_openloong_weight_shift_or_stepping_mpc

```text
outputs/runs/B07_openloong_weight_shift_or_stepping_mpc/<run_id>/videos/B07_openloong_weight_shift_or_stepping_mpc.mp4
outputs/runs/B07_openloong_weight_shift_or_stepping_mpc/<run_id>/figures/B07_com_or_pelvis_tracking.png
outputs/runs/B07_openloong_weight_shift_or_stepping_mpc/<run_id>/figures/B07_foot_target_error.png
outputs/runs/B07_openloong_weight_shift_or_stepping_mpc/<run_id>/figures/B07_contact_schedule.png
outputs/runs/B07_openloong_weight_shift_or_stepping_mpc/<run_id>/figures/B07_torque.png
outputs/runs/B07_openloong_weight_shift_or_stepping_mpc/<run_id>/metrics/B07_shift_or_step_metrics.csv
outputs/runs/B07_openloong_weight_shift_or_stepping_mpc/<run_id>/logs/B07_shift_or_step_run_log.txt
```

## 每个任务的验收标准

- 没有可视化输出的任务不算完成。
- 没有 metrics 的视频不算完成。
- 没有 README 复现命令的结果不算完成。
- 没有解释图像含义的结果不算完成。

每个任务的 README 或运行说明必须解释：

- video 展示了什么仿真现象。
- 每张 figure 的横轴、纵轴、单位和含义。
- metrics CSV 中每一列的含义。
- run log 中记录了哪些关键配置、模型路径和最终结果。

## simulator 工具规划

后续规划以下工具文件，但当前不实现：

```text
simulator/utils/plotting.py
simulator/utils/visualization.py
simulator/record_video.py
```

### simulator/utils/plotting.py

用于保存 error、torque、cost、runtime 曲线。

典型职责：

- 保存 tracking 曲线。
- 保存 error 曲线。
- 保存 torque / control force 曲线。
- 保存 best cost / rollout cost 曲线。
- 保存 runtime per control step 曲线。

### simulator/utils/visualization.py

用于绘制轨迹、目标点、rollout candidates、contact schedule。

典型职责：

- 绘制末端轨迹与目标轨迹。
- 绘制 rollout candidates。
- 绘制 selected rollout。
- 绘制小车倒立摆角度和位置示意图。
- 绘制 OpenLoong body / site / actuator 索引图。
- 绘制 foot contact schedule。

### simulator/record_video.py

用于 MuJoCo 离屏渲染和 mp4 导出。

典型职责：

- 根据 demo 名称选择渲染配置。
- 设置 camera、resolution、fps。
- 保存 `outputs/runs/<task_name>/<run_id>/videos/<task_name>.mp4`。
- 在 run log 中记录视频导出参数。
