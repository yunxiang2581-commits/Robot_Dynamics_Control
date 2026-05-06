# Project C Simulator 规划

## 目标

后续在本目录实现 OpenLoong-inspired Humanoid WBC Simulator，用 simulation-only 方式复现简化双足接触力分配、WBC-QP 平衡和 MPC-WBC-PVT 数据流可视化。

## 为什么不是直接实机部署

用户没有实物机器人。本项目禁止实物部署、sim2real、电机 SDK、CAN / EtherCAT / 串口通信、固件、真实机器人安全测试、真实传感器标定和硬件接口。

## 为什么先做简化 WBC-QP

OpenLoong-Dyn-Control 的完整链路包含步态调度、MPC、WBC、PVT、MuJoCo 接口和状态估计。先做简化 WBC-QP，可以把学习重点放在决策变量、接触约束、任务误差和求解状态上。

## 参考但不直接复现的模块

- DataBus。
- StateEstimator。
- Pin_KinDyn。
- MPC。
- WBC_QP。
- PVT_Ctr。
- GaitScheduler。
- FootPlacement。
- JoyStickInterpreter。
- MJ_Interface。

## 仿真器中最小复现的模块

- 简化 contact schedule。
- 简化 CoM target。
- contact force allocation QP。
- simplified WBC-QP。
- video export。
- metrics logging。

## 最小可运行命令规划

```bash
python simulator/run_demo.py --demo C01_contact_force_allocation_demo --export-video
python simulator/run_demo.py --demo C02_simplified_wbc_qp_balance_demo --export-video
python simulator/run_demo.py --demo C03_mpc_wbc_pipeline_visual_demo --export-video
```

当前命令只是后续 TODO skeleton 设计，本次不实现、不执行。

## 输出位置

- 视频：`outputs/videos/`
- 曲线：`outputs/figures/`
- 日志：`outputs/logs/`
- 指标：`outputs/metrics/`

## 评价指标

- CoM tracking error。
- contact force constraint violation。
- balance duration。
- QP solve status。
