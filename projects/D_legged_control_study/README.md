# Project D：legged_control 学习区

本目录用于整理 `legged_control` 与 OCS2 公开资料，并规划一个 simulation-only 的 legged_control-inspired Quadruped Simulator。

## 最终交付目标

Project D 不是纯资料阅读项目。最终需要实现一个受 legged_control 启发的四足 NMPC / WBC / 状态估计简化仿真模拟器，并导出视频 demo。

最终展示形式：

1. 读懂四足 NMPC、WBC、contact schedule、state estimation 的核心思想。
2. 把 ROS / OCS2 复杂控制栈抽象成可解释的数学模块。
3. 在 simulation-only 条件下做最小可运行仿真器或离线算法复现。
4. 导出 mp4 视频或图表 demo。
5. 记录 contact force residual、friction cone violation、gait phase correctness、state estimation RMSE。

## 当前阶段边界

本次只做目录和 Markdown 计划：

- 不 clone 外部仓库。
- 不下载大文件。
- 不编译。
- 不运行仿真。
- 不实现 Python/C++ 控制逻辑。
- 不修改 `A_self_baseline`。

## 禁止范围

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
- [simulator/README.md](simulator/README.md)
- [literature_and_links.md](notes/literature_and_links.md)

## 规划 demo

- `D01_quadruped_contact_qp_demo`
- `D02_trot_gait_schedule_visual_demo`
- `D03_state_estimation_tracking_demo`

结果目录：

- `outputs/videos/`
- `outputs/figures/`
- `outputs/logs/`
- `outputs/metrics/`

## 规范化项目骨架补充

### 项目定位

Project D 用于学习 legged_control 的 quadruped NMPC + WBC + state estimation 架构，并将其抽象成 simulation-only 的四足简化仿真 demo。

### 对应开源项目

- 开源项目：legged_control
- 上游仓库：<https://github.com/qiayuanl/legged_control>

### 对应外部源码路径

```text
external/open_source_repos/legged_control/
```

该路径只作为源码阅读参考，不合并进 Project D，也不在本仓库中修改其内部源码。

### simulation-only 范围

Project D 只做仿真和离线算法复现：

- 简化四足支撑模型。
- contact force QP。
- trot gait schedule 可视化。
- 简化 Kalman filter 或低通滤波 state estimation。
- 视频、图表和 metrics 导出。

### 不做实物部署声明

本项目当前不运行 `legged_hw`，不做 Unitree 实机接口，不做 ros-control hardware interface，不做实物部署，不做 sim2real 实机测试，不接电机 SDK，不接 CAN / EtherCAT / 串口通信，不做固件，不做真实机器人安全测试，不做真实传感器标定，不实现硬件接口。

### 最终目标

最终目标是形成 legged_control-inspired 可运行仿真模拟器 + 视频 demo：

- runnable simulator。
- mp4 video demo 或图表动画。
- 可复现实验命令。
- metrics 输出。
- README 运行说明。

### 计划 demo 列表

1. `D01_quadruped_contact_qp_demo`
   - 简化四足支撑模型。
   - 分配四个足端接触力。
   - 满足摩擦锥近似约束。
   - 可视化接触力。
   - 导出 mp4。

2. `D02_trot_gait_schedule_visual_demo`
   - 四足 trot 步态相位切换。
   - 支撑足 / 摆动足状态可视化。
   - 足端轨迹动画。
   - 导出 mp4。

3. `D03_state_estimation_tracking_demo`
   - 使用仿真真值添加噪声。
   - 简化 Kalman filter 或低通滤波估计 base velocity。
   - 显示真实值 vs 估计值曲线。
   - 导出视频或图表。

### simulator/ 目录说明

```text
simulator/
├── envs/          # 简化四足支撑 / 摆动仿真环境
├── controllers/   # contact QP、WBC-like torque command 骨架
├── planners/      # trot gait schedule、swing foot trajectory 骨架
├── utils/         # 通用工具、路径、指标和可视化辅助
├── scripts/       # 后续命令行脚本入口
└── README.md
```

### outputs/ 目录说明

```text
outputs/
├── videos/        # mp4 demo 或动画
├── figures/       # contact force、gait phase、estimation 曲线
├── logs/          # QP、步态、估计器运行日志
└── metrics/       # contact force residual、friction cone violation、RMSE
```

### 与 A_self_baseline 的关系

`A_self_baseline` 提供 MuJoCo / Pinocchio / QP-IK 基础。Project D 不修改 A 的源码，而是在独立目录中把 QP、接触约束、状态估计等复杂控制栈概念拆成四足 simulation-only demo。

### 后续最小实现顺序

1. 先实现 `D01_quadruped_contact_qp_demo`，讲清楚接触力和摩擦锥。
2. 再实现 `D02_trot_gait_schedule_visual_demo`，讲清楚 contact schedule。
3. 最后实现 `D03_state_estimation_tracking_demo`，讲清楚带噪声状态估计。
