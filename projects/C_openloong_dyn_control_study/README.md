# Project C：OpenLoong-Dyn-Control 学习区

本目录用于整理 OpenLoong-Dyn-Control 的公开资料，并规划一个 simulation-only 的 OpenLoong-inspired Humanoid WBC Simulator。

## 最终交付目标

Project C 不是纯资料阅读项目。最终需要实现一个受 OpenLoong-Dyn-Control 启发的简化人形 / 双足 WBC 仿真模拟器，并导出视频 demo。

最终展示形式：

1. 读懂 MPC + WBC + PVT + 步态调度的核心思想。
2. 把人形控制栈抽象成可解释的数学模块。
3. 在 simulation-only 条件下做最小可运行仿真器。
4. 导出 mp4 视频或动画 demo。
5. 记录 CoM tracking error、contact force constraint violation、balance duration、QP solve status。

## 当前阶段边界

本次只做目录和 Markdown 计划：

- 不 clone 外部仓库。
- 不下载大文件。
- 不编译。
- 不运行仿真。
- 不实现 Python/C++ 控制逻辑。
- 不修改 `A_self_baseline`。

## 禁止范围

- 实物部署。
- sim2real 实机测试。
- 电机 SDK。
- CAN / EtherCAT / 串口通信。
- 固件。
- 真实机器人安全测试。
- 真实传感器标定。
- 硬件接口。

## 阅读入口

- [00_project_overview.md](docs/00_project_overview.md)
- [01_algorithm_map.md](docs/01_algorithm_map.md)
- [02_source_reading_map.md](docs/02_source_reading_map.md)
- [03_simulation_only_reproduction_plan.md](docs/03_simulation_only_reproduction_plan.md)
- [04_relation_to_A_self_baseline.md](docs/04_relation_to_A_self_baseline.md)
- [05_dependency_and_risk.md](docs/05_dependency_and_risk.md)
- [06_next_questions.md](docs/06_next_questions.md)
- [07_simulator_and_video_demo_plan.md](docs/07_simulator_and_video_demo_plan.md)
- [08_source_repo_detailed_introduction.md](docs/08_source_repo_detailed_introduction.md)
- [09_engineering_reproduction_audit.md](docs/09_engineering_reproduction_audit.md)
- [10_official_reproduction_runbook.md](docs/10_official_reproduction_runbook.md)
- [11_source_reading_workflow.md](docs/11_source_reading_workflow.md)
- [simulator/README.md](simulator/README.md)
- [literature_and_links.md](notes/literature_and_links.md)

## 规划 demo

- `C01_contact_force_allocation_demo`
- `C02_simplified_wbc_qp_balance_demo`
- `C03_mpc_wbc_pipeline_visual_demo`

结果目录：

- `outputs/videos/`
- `outputs/figures/`
- `outputs/logs/`
- `outputs/metrics/`

## 规范化项目骨架补充

### 项目定位

Project C 用于学习 OpenLoong-Dyn-Control 的 humanoid MPC + WBC + PVT + MuJoCo 闭环架构，并将其中最适合求职展示的核心思想抽象成 simulation-only 的简化人形 / 双足 / WBC 仿真 demo。

### 对应开源项目

- 开源项目：OpenLoong-Dyn-Control
- 上游仓库：<https://github.com/loongOpen/OpenLoong-Dyn-Control>

### 对应外部源码路径

```text
external/open_source_repos/OpenLoong-Dyn-Control/
```

该路径只作为源码阅读参考，不合并进 Project C，也不在本仓库中修改其内部源码。

### simulation-only 范围

Project C 只做仿真和离线算法复现：

- 简化双足支撑模型。
- 简化浮动基模型。
- contact force allocation QP。
- simplified WBC-QP balance。
- MPC-WBC-PVT 数据流可视化。

### 不做实物部署声明

本项目不做实物部署、不做 sim2real 实机测试、不接电机 SDK、不接 CAN / EtherCAT / 串口通信、不做固件、不做真实机器人安全测试、不做真实传感器标定、不实现硬件接口。

### 最终目标

最终目标是形成 OpenLoong-inspired 可运行仿真模拟器 + 视频 demo：

- runnable simulator。
- mp4 video demo 或动画。
- 可复现实验命令。
- metrics 输出。
- README 运行说明。

### 计划 demo 列表

1. `C01_contact_force_allocation_demo`
   - 简化双足支撑。
   - 给定质心目标。
   - QP 分配左右脚接触力。
   - 可视化接触力箭头。
   - 导出 mp4。

2. `C02_simplified_wbc_qp_balance_demo`
   - 简化浮动基模型。
   - WBC-QP 计算 qddot / contact force / tau。
   - 平衡控制。
   - 导出 mp4。

3. `C03_mpc_wbc_pipeline_visual_demo`
   - 展示 command -> gait/contact schedule -> MPC target -> WBC-QP -> MuJoCo control。
   - 不要求完整人形行走。
   - 重点展示控制链路。
   - 导出视频或动画。

### simulator/ 目录说明

```text
simulator/
├── envs/          # 简化双足 / 浮动基仿真环境
├── controllers/   # contact force allocation、WBC-QP、PVT-like 控制骨架
├── planners/      # contact schedule、CoM target、foot placement 骨架
├── utils/         # 通用工具、路径、指标和可视化辅助
├── scripts/       # 后续命令行脚本入口
└── README.md
```

### outputs/ 目录说明

```text
outputs/
├── videos/        # mp4 demo 或动画
├── figures/       # CoM error、contact force、QP 状态图
├── logs/          # QP 求解日志和运行日志
└── metrics/       # CoM tracking error、constraint violation、balance duration
```

### 与 A_self_baseline 的关系

`A_self_baseline` 提供 MuJoCo / Pinocchio / QP-IK 基础。Project C 不修改 A 的源码，而是在独立目录中把 A 已学到的 Jacobian、QP 和 tracking 概念扩展到人形 MPC/WBC 架构阅读与简化仿真。

### 后续最小实现顺序

1. 先实现 `C01_contact_force_allocation_demo`，把接触力约束讲清楚。
2. 再实现 `C02_simplified_wbc_qp_balance_demo`，加入 qddot / contact force / tau。
3. 最后实现 `C03_mpc_wbc_pipeline_visual_demo`，展示控制链路而不是追求完整人形行走。
