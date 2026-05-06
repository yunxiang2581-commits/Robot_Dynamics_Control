# Project B/C/D 视频 Demo 交付要求

## 通用交付要求

Project B/C/D 都不是纯阅读项目。源码阅读只是第一步，最终必须落到 simulation-only 的可运行仿真、视频 demo 和指标报告。

每个 Project 的可复现阶段都必须规划：

- runnable simulator。
- simulation video demo。
- video export script。
- demo result directory。
- reproducible command。
- README 中的运行说明。
- 每个 demo 的评价指标。

每个项目最终都必须有：

- 一个 runnable simulator。
- 至少一组 video demo。
- 每个 demo 的可复现实验命令。
- 每个 demo 的 metrics 输出。

当前严格不做：

- 实物部署。
- sim2real 实机测试。
- 电机 SDK。
- CAN。
- EtherCAT。
- 串口通信。
- 固件。
- 真实机器人安全测试。

## 统一目录约定

```text
simulator/
├── envs/
├── controllers/
├── planners/
├── run_demo.py
├── record_video.py
└── metrics.py

outputs/
├── videos/
├── figures/
├── logs/
└── metrics/
```

## Project B 视频 demo

| Demo | 视频输出 | 指标 |
|---|---|---|
| B01_single_joint_mpc_demo | `outputs/videos/B01_single_joint_mpc_demo.mp4` | final error、mean tracking error、max torque、runtime per control step |
| B02_two_link_mpc_tracking_demo | `outputs/videos/B02_two_link_mpc_tracking_demo.mp4` | final error、mean tracking error、max torque、runtime per control step |
| B03_rollout_predictive_sampling_demo | `outputs/videos/B03_rollout_predictive_sampling_demo.mp4` | best rollout cost、mean tracking error、max torque、runtime per control step |

## Project C 视频 demo

| Demo | 视频输出 | 指标 |
|---|---|---|
| C01_contact_force_allocation_demo | `outputs/videos/C01_contact_force_allocation_demo.mp4` | CoM tracking error、contact force constraint violation、QP solve status |
| C02_simplified_wbc_qp_balance_demo | `outputs/videos/C02_simplified_wbc_qp_balance_demo.mp4` | CoM tracking error、balance duration、QP solve status |
| C03_mpc_wbc_pipeline_visual_demo | `outputs/videos/C03_mpc_wbc_pipeline_visual_demo.mp4` | contact schedule consistency、runtime per control step、QP solve status |

## Project D 视频 demo

| Demo | 视频输出 | 指标 |
|---|---|---|
| D01_quadruped_contact_qp_demo | `outputs/videos/D01_quadruped_contact_qp_demo.mp4` | contact force residual、friction cone violation、QP solve status |
| D02_trot_gait_schedule_visual_demo | `outputs/videos/D02_trot_gait_schedule_visual_demo.mp4` | gait phase correctness、swing foot trajectory continuity |
| D03_state_estimation_tracking_demo | `outputs/videos/D03_state_estimation_tracking_demo.mp4` | state estimation RMSE、velocity tracking lag、noise attenuation ratio |

## 复现命令要求

每个 demo 后续都应支持统一命令格式：

```bash
python simulator/run_demo.py --demo <demo_name> --export-video
```

本次只写规划，不实现该命令。
