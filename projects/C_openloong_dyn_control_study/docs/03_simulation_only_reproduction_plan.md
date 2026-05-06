# Project C 仿真-only 复现计划：OpenLoong-inspired Humanoid WBC Simulator

## 最终目标

Project C 最终要实现一个受 OpenLoong-Dyn-Control 启发的简化人形 / 双足 WBC 仿真模拟器，并导出视频 demo。

最终展示形式必须说明：

1. 读懂了人形 MPC + WBC + PVT + 步态调度的核心思想。
2. 把复杂控制栈抽象成可解释的数学模块。
3. 在 simulation-only 条件下完成最小可运行仿真器。
4. 导出 mp4 视频或动画 demo。
5. 记录 CoM tracking error、contact force constraint violation、balance duration、QP solve status 等指标。

## 严格边界

- 不做实物部署。
- 不做 sim2real 实机测试。
- 不接电机 SDK。
- 不接 CAN / EtherCAT / 串口通信。
- 不涉及固件。
- 不做真实机器人安全测试。
- 不做真实传感器标定。
- 不实现硬件接口。

## 后续 TODO skeleton 接口规划

```text
simulator/
├── envs/                 # TODO: 简化双足/浮动基仿真环境
├── controllers/          # TODO: WBC-QP、PVT-like 控制器
├── planners/             # TODO: contact schedule、CoM target、foot placement
├── run_demo.py           # TODO: 统一 demo 入口
├── record_video.py       # TODO: 离屏渲染与动画导出
└── metrics.py            # TODO: CoM、接触力、QP 状态指标
```

本次不实现上述 Python/C++ 逻辑，只保留规划。

## C01_contact_force_allocation_demo

目标：

- 简化双足支撑模型。
- 给定质心目标。
- QP 分配左右脚接触力。
- 可视化接触力箭头。
- 导出 mp4 视频。

输出：

- `outputs/videos/C01_contact_force_allocation_demo.mp4`
- `outputs/figures/C01_contact_force_arrows.png`
- `outputs/logs/C01_qp_log.txt`
- `outputs/metrics/C01_metrics.csv`

评价指标：

- CoM tracking error。
- contact force constraint violation。
- QP solve status。

## C02_simplified_wbc_qp_balance_demo

目标：

- 简化浮动基模型。
- WBC-QP 计算 `qddot / contact force / tau`。
- 让机器人或简化模型保持平衡。
- 导出 mp4 视频。

输出：

- `outputs/videos/C02_simplified_wbc_qp_balance_demo.mp4`
- `outputs/figures/C02_balance_error.png`
- `outputs/logs/C02_wbc_qp_log.txt`
- `outputs/metrics/C02_metrics.csv`

评价指标：

- CoM tracking error。
- contact force constraint violation。
- balance duration。
- QP solve status。

## C03_mpc_wbc_pipeline_visual_demo

目标：

- 不要求完整人形行走。
- 展示 `command -> gait/contact schedule -> MPC target -> WBC-QP -> MuJoCo control` 的数据流。
- 导出可视化视频或动画。

输出：

- `outputs/videos/C03_mpc_wbc_pipeline_visual_demo.mp4`
- `outputs/figures/C03_pipeline_timeline.png`
- `outputs/logs/C03_pipeline_log.txt`
- `outputs/metrics/C03_metrics.csv`

评价指标：

- contact schedule consistency。
- CoM target tracking error。
- QP solve status。
- runtime per control step。

## 为什么先做简化 WBC-QP

OpenLoong-Dyn-Control 是完整人形控制框架，直接运行或复现会同时引入模型、步态、MPC、WBC、PVT、MuJoCo 接口等复杂因素。先做简化 WBC-QP，可以把核心问题收敛到“给定目标和接触约束，如何求解加速度、接触力和力矩”。
