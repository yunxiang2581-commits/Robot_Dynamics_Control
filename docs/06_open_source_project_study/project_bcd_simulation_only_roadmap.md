# Project B/C/D Simulation-only 路线图

## 总目标

Project B/C/D 不是纯资料阅读项目。最终展示形式不是“我读过源码”，而是：

1. 我读懂了复杂开源运动控制项目的核心思想。
2. 我把核心算法抽象成可解释的数学模块。
3. 我在 simulation-only 条件下做了最小可运行仿真器。
4. 我导出了可展示的视频 demo。
5. 我记录了误差、约束违反、运行时间等指标。

## 禁止范围

- 实物部署。
- sim2real 实机测试。
- 电机 SDK。
- CAN / EtherCAT / 串口通信。
- 固件。
- 真实机器人安全测试。
- 真实传感器标定。
- 硬件接口。

## Phase 1：资料阅读，不运行

目标：

- 阅读公开资料、README、论文入口和官方文档。
- 明确已确认信息和待源码确认信息。
- 不 clone、不下载、不编译、不运行。

输出：

- 项目 overview。
- algorithm map。
- source reading map。

## Phase 2：数学模型抽象

目标：

- 从开源项目中抽象最小数学问题。
- 将复杂工程拆成可解释模块。

对应抽象：

- B：rollout、horizon cost、receding horizon、predictive sampling。
- C：contact force allocation、WBC-QP、MPC-WBC-PVT 数据流。
- D：contact QP、trot gait schedule、state estimation。

## Phase 3：simulation-only 最小仿真器

目标：

- 每个项目都有 `simulator/` 目录。
- 每个项目都规划 `run_demo.py`、`record_video.py`、`metrics.py`。
- 每个 demo 都有可复现命令、输出目录和评价指标。

输出目录：

```text
outputs/
├── videos/
├── figures/
├── logs/
└── metrics/
```

## Phase 4：视频 demo 与指标报告

目标：

- 导出 mp4 视频。
- 导出误差曲线。
- 导出日志和指标 CSV。
- README 中写清楚如何复现。

最小交付：

- B：MPC tracking 视频与 tracking error。
- C：双足接触力 / WBC 平衡视频与约束违反指标。
- D：四足接触力 / trot gait / 状态估计视频或图表与 RMSE。

## Phase 5：选择是否阅读或运行原项目 demo

只有当前四项完成后，才考虑运行外部原项目 demo：

- 已有自己的 simulation-only 最小复现。
- 已有视频 demo。
- 已有指标。
- 已有依赖风险评估。

当前优先顺序：

1. B：最贴近 MuJoCo baseline。
2. C：最贴近人形机器人求职方向。
3. D：依赖最重且原项目不再维护，优先阅读和离线复现。
