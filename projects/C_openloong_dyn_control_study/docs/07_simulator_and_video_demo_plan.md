# Project C Simulator 与视频 Demo 计划

## Simulator 名称

OpenLoong-inspired Humanoid WBC Simulator。

## 目标

后续实现一个 simulation-only 的简化人形 / 双足 WBC 仿真器。它不复现完整 OpenLoong-Dyn-Control，而是最小复现接触力分配、WBC-QP 平衡和 MPC-WBC-PVT 数据流。

## Demo 列表

| Demo | 目标 | 视频输出 | 关键指标 |
|---|---|---|---|
| C01_contact_force_allocation_demo | 双足接触力 QP 分配 | `outputs/videos/C01_contact_force_allocation_demo.mp4` | CoM tracking error、contact force constraint violation、QP solve status |
| C02_simplified_wbc_qp_balance_demo | 简化 WBC-QP 保持平衡 | `outputs/videos/C02_simplified_wbc_qp_balance_demo.mp4` | CoM tracking error、balance duration、QP solve status |
| C03_mpc_wbc_pipeline_visual_demo | command 到 MuJoCo control 数据流动画 | `outputs/videos/C03_mpc_wbc_pipeline_visual_demo.mp4` | contact schedule consistency、runtime per control step、QP solve status |

## 参考但不直接复现的 OpenLoong 模块

- DataBus。
- StateEstimator。
- Pin_KinDyn。
- MPC。
- WBC_QP。
- PVT_Ctr。
- GaitScheduler。
- FootPlacement。
- MJ_Interface。

## 最小复现模块

- 简化 contact schedule。
- 简化 CoM target。
- 简化 WBC-QP。
- 简化 MuJoCo 或离线动画可视化。
- 指标统计与视频导出。

## 可复现命令规划

```bash
python simulator/run_demo.py --demo C01_contact_force_allocation_demo --export-video
python simulator/run_demo.py --demo C02_simplified_wbc_qp_balance_demo --export-video
python simulator/run_demo.py --demo C03_mpc_wbc_pipeline_visual_demo --export-video
```

当前命令只是后续接口设计，本次不执行。
