# Project D Simulator 与视频 Demo 计划

## Simulator 名称

legged_control-inspired Quadruped Simulator。

## 目标

后续实现一个 simulation-only 的四足 NMPC / WBC / 状态估计简化仿真器或离线算法复现。它不运行 `legged_hw`，不接 Unitree，不做 ros-control hardware interface。

## Demo 列表

| Demo | 目标 | 视频输出 | 关键指标 |
|---|---|---|---|
| D01_quadruped_contact_qp_demo | 四足接触力 QP 分配 | `outputs/videos/D01_quadruped_contact_qp_demo.mp4` | contact force residual、friction cone violation、QP solve status |
| D02_trot_gait_schedule_visual_demo | trot 步态相位和足端轨迹动画 | `outputs/videos/D02_trot_gait_schedule_visual_demo.mp4` | gait phase correctness、swing foot trajectory continuity |
| D03_state_estimation_tracking_demo | 仿真真值加噪后的 base velocity 估计 | `outputs/videos/D03_state_estimation_tracking_demo.mp4` | state estimation RMSE、velocity tracking lag、noise attenuation ratio |

## 当前不做

- 不运行 `legged_hw`。
- 不做 Unitree 实机接口。
- 不做 ros-control hardware interface。
- 不编译 OCS2。
- 不运行 Gazebo demo。

## 可复现命令规划

```bash
python simulator/run_demo.py --demo D01_quadruped_contact_qp_demo --export-video
python simulator/run_demo.py --demo D02_trot_gait_schedule_visual_demo --export-video
python simulator/run_demo.py --demo D03_state_estimation_tracking_demo --export-video
```

当前命令只是后续接口设计，本次不执行。
