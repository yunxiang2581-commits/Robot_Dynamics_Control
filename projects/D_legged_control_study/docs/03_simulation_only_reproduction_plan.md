# Project D 仿真-only 复现计划：legged_control-inspired Quadruped Simulator

## 最终目标

Project D 最终要实现一个受 legged_control 启发的四足 NMPC / WBC / 状态估计简化仿真模拟器，并导出视频 demo。

最终展示形式必须说明：

1. 读懂了四足 NMPC、WBC、contact schedule、state estimation 的核心思想。
2. 把复杂 ROS / OCS2 控制栈抽象成可解释的数学模块。
3. 在 simulation-only 条件下完成最小可运行仿真器或离线算法复现。
4. 导出 mp4 视频或图表 demo。
5. 记录 contact force residual、friction cone violation、gait phase correctness、state estimation RMSE 等指标。

## 严格边界

- 不运行 `legged_hw`。
- 不做 Unitree 实机接口。
- 不做 ros-control hardware interface。
- 不做实物部署。
- 不做 sim2real 实机测试。
- 不接电机 SDK。
- 不接 CAN / EtherCAT / 串口通信。
- 不涉及固件。
- 不做真实传感器标定。
- 只做仿真或离线算法复现。

## 后续 TODO skeleton 接口规划

```text
simulator/
├── envs/                 # TODO: 简化四足支撑/摆动仿真环境
├── controllers/          # TODO: contact QP、WBC-like torque command
├── planners/             # TODO: trot gait schedule、swing foot trajectory
├── run_demo.py           # TODO: 统一 demo 入口
├── record_video.py       # TODO: 视频或动画导出
└── metrics.py            # TODO: 接触力、摩擦锥、步态、估计误差指标
```

本次不实现上述 Python/C++ 逻辑，只保留规划。

## D01_quadruped_contact_qp_demo

目标：

- 简化四足支撑模型。
- 分配四个足端接触力。
- 满足摩擦锥近似约束。
- 可视化接触力。
- 导出 mp4 视频。

输出：

- `outputs/videos/D01_quadruped_contact_qp_demo.mp4`
- `outputs/figures/D01_contact_force_arrows.png`
- `outputs/logs/D01_contact_qp_log.txt`
- `outputs/metrics/D01_metrics.csv`

评价指标：

- contact force residual。
- friction cone violation。
- QP solve status。

## D02_trot_gait_schedule_visual_demo

目标：

- 四足 trot 步态相位切换。
- 支撑足 / 摆动足状态可视化。
- 足端轨迹动画。
- 导出 mp4 视频。

输出：

- `outputs/videos/D02_trot_gait_schedule_visual_demo.mp4`
- `outputs/figures/D02_gait_phase_timeline.png`
- `outputs/logs/D02_gait_schedule_log.txt`
- `outputs/metrics/D02_metrics.csv`

评价指标：

- gait phase correctness。
- swing foot trajectory continuity。
- contact schedule consistency。

## D03_state_estimation_tracking_demo

目标：

- 使用仿真真值添加噪声。
- 用简化 Kalman filter 或低通滤波估计 base velocity。
- 显示真实值 vs 估计值曲线。
- 导出视频或图表。

输出：

- `outputs/videos/D03_state_estimation_tracking_demo.mp4`
- `outputs/figures/D03_velocity_estimation.png`
- `outputs/logs/D03_estimator_log.txt`
- `outputs/metrics/D03_metrics.csv`

评价指标：

- state estimation RMSE。
- velocity tracking lag。
- noise attenuation ratio。

## 最小可复现命令规划

```bash
python simulator/run_demo.py --demo D01_quadruped_contact_qp_demo --export-video
python simulator/run_demo.py --demo D02_trot_gait_schedule_visual_demo --export-video
python simulator/run_demo.py --demo D03_state_estimation_tracking_demo --export-video
```

这些命令当前只是规划，不在本次执行。
