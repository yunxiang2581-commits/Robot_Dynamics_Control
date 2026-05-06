# Project D Simulator 规划

## 目标

后续在本目录实现 legged_control-inspired Quadruped Simulator，用 simulation-only 或离线算法复现方式学习四足 contact QP、trot gait schedule 和 state estimation。

## 当前不做

- 不运行 `legged_hw`。
- 不做 Unitree 实机接口。
- 不做 ros-control hardware interface。
- 不做实物部署。
- 不做 sim2real 实机测试。
- 不接电机 SDK。
- 不接 CAN / EtherCAT / 串口通信。
- 不涉及固件。
- 不做真实传感器标定。

## 只做什么

- 仿真或离线算法复现。
- 接触力 QP。
- trot gait schedule 可视化。
- 简化 Kalman filter 或低通滤波状态估计。
- 视频或图表导出。

## 最小可运行命令规划

```bash
python simulator/run_demo.py --demo D01_quadruped_contact_qp_demo --export-video
python simulator/run_demo.py --demo D02_trot_gait_schedule_visual_demo --export-video
python simulator/run_demo.py --demo D03_state_estimation_tracking_demo --export-video
```

当前命令只是后续 TODO skeleton 设计，本次不实现、不执行。

## 输出位置

- 视频：`outputs/videos/`
- 曲线：`outputs/figures/`
- 日志：`outputs/logs/`
- 指标：`outputs/metrics/`

## 评价指标

- contact force residual。
- friction cone violation。
- gait phase correctness。
- state estimation RMSE。
