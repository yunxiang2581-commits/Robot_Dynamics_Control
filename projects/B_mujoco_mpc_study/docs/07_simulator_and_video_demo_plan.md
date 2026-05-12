# Project B Simulator 与视频 Demo 计划

## Simulator 名称

MuJoCo MPC Simulator。

## 目标

后续实现一个 simulation-only 的 MuJoCo MPC 教学仿真器，用最小模型复现 MJPC 的核心思想：task residual、rollout、horizon cost、predictive sampling、receding horizon control。

每个 demo 都必须满足：

```text
simulation video + figures + metrics CSV + run log + README 复现实验命令
```

可视化不是可选项，而是 B01-B07 的验收条件。没有可视化输出的任务不算完成；没有 metrics 的视频不算完成；没有 README 复现命令的结果不算完成；没有解释图像含义的结果不算完成。

## Demo 列表

| Demo | 目标 | 视频输出 | 关键指标 |
|---|---|---|---|
| Demo | 目标 | 必须视频输出 | 必须 figures | 必须 metrics / log |
|---|---|---|---|---|
| B01_single_joint_mpc_demo | 单关节目标角度跟踪 | `outputs/runs/B01_single_joint_mpc_demo/<run_id>/videos/B01_single_joint_mpc_demo.mp4` | `B01_angle_tracking.png`、`B01_angle_error.png`、`B01_torque.png`、`B01_best_cost.png` | `outputs/runs/B01_single_joint_mpc_demo/<run_id>/metrics/B01_metrics.csv`、`outputs/runs/B01_single_joint_mpc_demo/<run_id>/logs/B01_run_log.txt` |
| B02_two_link_mpc_tracking_demo | 二连杆末端轨迹跟踪 | `outputs/runs/B02_two_link_mpc_tracking_demo/<run_id>/videos/B02_two_link_mpc_tracking_demo.mp4` | `B02_ee_trajectory_xy.png`、`B02_ee_tracking_error.png`、`B02_joint_torque.png` | `outputs/runs/B02_two_link_mpc_tracking_demo/<run_id>/metrics/B02_metrics.csv`、`outputs/runs/B02_two_link_mpc_tracking_demo/<run_id>/logs/B02_control_log.txt` |
| B03_rollout_predictive_sampling_demo | 多 rollout 选择最低 cost 序列 | `outputs/runs/B03_rollout_predictive_sampling_demo/<run_id>/videos/B03_rollout_predictive_sampling_demo.mp4` | `B03_rollout_candidates.png`、`B03_rollout_costs.png`、`B03_selected_rollout.png` | `outputs/runs/B03_rollout_predictive_sampling_demo/<run_id>/metrics/B03_metrics.csv`、`outputs/runs/B03_rollout_predictive_sampling_demo/<run_id>/logs/B03_selected_rollout_log.txt` |
| B04_cart_double_inverted_pendulum_mpc | 小车倒立二阶摆欠驱动平衡 | `outputs/runs/B04_cart_double_inverted_pendulum_mpc/<run_id>/videos/B04_cart_double_inverted_pendulum_mpc.mp4` | `B04_cart_position.png`、`B04_pendulum_angles.png`、`B04_angle_errors.png`、`B04_control_force.png` | `outputs/runs/B04_cart_double_inverted_pendulum_mpc/<run_id>/metrics/B04_cart_double_pendulum_metrics.csv`、`outputs/runs/B04_cart_double_inverted_pendulum_mpc/<run_id>/logs/B04_cart_double_pendulum_run_log.txt` |
| B05_openloong_model_mpc_setup | OpenLoong 人形模型接入与状态摘要 | `outputs/runs/B05_openloong_model_mpc_setup/<run_id>/videos/B05_openloong_passive_or_static_preview.mp4` | `B05_body_site_map.png`、`B05_actuator_index_map.png` | `outputs/runs/B05_openloong_model_mpc_setup/<run_id>/metrics/B05_model_dimensions.csv`、`outputs/runs/B05_openloong_model_mpc_setup/<run_id>/logs/B05_openloong_model_summary.txt` |
| B06_openloong_standing_balance_mpc | OpenLoong 站立平衡 / 姿态保持 | `outputs/runs/B06_openloong_standing_balance_mpc/<run_id>/videos/B06_openloong_standing_balance_mpc.mp4` | `B06_pelvis_height_error.png`、`B06_torso_orientation_error.png`、`B06_joint_torque.png`、`B06_foot_contact_summary.png` | `outputs/runs/B06_openloong_standing_balance_mpc/<run_id>/metrics/B06_balance_metrics.csv`、`outputs/runs/B06_openloong_standing_balance_mpc/<run_id>/logs/B06_balance_run_log.txt` |
| B07_openloong_weight_shift_or_stepping_mpc | OpenLoong 重心转移或小步踏步 | `outputs/runs/B07_openloong_weight_shift_or_stepping_mpc/<run_id>/videos/B07_openloong_weight_shift_or_stepping_mpc.mp4` | `B07_com_or_pelvis_tracking.png`、`B07_foot_target_error.png`、`B07_contact_schedule.png`、`B07_torque.png` | `outputs/runs/B07_openloong_weight_shift_or_stepping_mpc/<run_id>/metrics/B07_shift_or_step_metrics.csv`、`outputs/runs/B07_openloong_weight_shift_or_stepping_mpc/<run_id>/logs/B07_shift_or_step_run_log.txt` |

## 计划接口

```text
simulator/envs/
simulator/controllers/
simulator/planners/
simulator/utils/plotting.py
simulator/utils/visualization.py
simulator/run_demo.py
simulator/record_video.py
simulator/metrics.py
```

本轮只规划工具，不实现：

- `simulator/utils/plotting.py`：用于保存 error、torque、cost、runtime 曲线。
- `simulator/utils/visualization.py`：用于绘制轨迹、目标点、rollout candidates、contact schedule。
- `simulator/record_video.py`：用于 MuJoCo 离屏渲染和 mp4 导出。

## 可复现命令规划

```bash
python simulator/run_demo.py --demo B01_single_joint_mpc_demo --export-video
python simulator/run_demo.py --demo B02_two_link_mpc_tracking_demo --export-video
python simulator/run_demo.py --demo B03_rollout_predictive_sampling_demo --export-video
python simulator/run_demo.py --demo B04_cart_double_inverted_pendulum_mpc --export-video
python simulator/run_demo.py --demo B05_openloong_model_mpc_setup
python simulator/run_demo.py --demo B06_openloong_standing_balance_mpc --export-video
python simulator/run_demo.py --demo B07_openloong_weight_shift_or_stepping_mpc --export-video
```

当前命令只是后续接口设计，本次不执行。
