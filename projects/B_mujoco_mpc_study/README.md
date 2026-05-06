# Project B：MuJoCo MPC / MJPC 学习区

本目录用于整理 Google DeepMind MuJoCo MPC（MJPC）的公开资料，并规划一个 simulation-only 的 MuJoCo MPC Simulator。

## 最终交付目标

Project B 不是纯资料阅读项目。最终需要实现一个基于 MuJoCo 的 MPC 仿真模拟器，并导出视频 demo。

最终展示形式：

1. 读懂 MJPC 的 task / residual / rollout / planner 核心思想。
2. 把 MPC 抽象成可解释的数学模块。
3. 在 simulation-only 条件下做最小可运行仿真器。
4. 导出 mp4 视频 demo。
5. 记录 final error、mean tracking error、max torque、runtime per control step。

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
- [simulator/README.md](simulator/README.md)
- [literature_and_links.md](notes/literature_and_links.md)

## 规划 demo

- `B01_single_joint_mpc_demo`
- `B02_two_link_mpc_tracking_demo`
- `B03_rollout_predictive_sampling_demo`

结果目录：

- `outputs/videos/`
- `outputs/figures/`
- `outputs/logs/`
- `outputs/metrics/`

## 规范化项目骨架补充

### 项目定位

Project B 用于学习 MuJoCo MPC / MJPC 的 task、cost、residual、rollout、planner、horizon 和 receding horizon control，并把这些概念抽象成当前仓库内可解释、可复现的 simulation-only MuJoCo MPC 仿真项目。

### 对应开源项目

- 开源项目：MuJoCo MPC / MJPC
- 上游仓库：<https://github.com/google-deepmind/mujoco_mpc>

### 对应外部源码路径

```text
external/open_source_repos/mujoco_mpc/
```

该路径只作为源码阅读参考，不合并进 Project B，也不在本仓库中修改其内部源码。

### simulation-only 范围

Project B 只做仿真和离线算法复现：

- MuJoCo 单关节模型。
- MuJoCo 二连杆机械臂模型。
- rollout / predictive sampling 教学 demo。
- 视频导出和 metrics 记录。

### 不做实物部署声明

本项目不做实物部署、不做 sim2real 实机测试、不接电机 SDK、不接 CAN / EtherCAT / 串口通信、不做固件、不做真实机器人安全测试、不做真实传感器标定、不实现硬件接口。

### 最终目标

最终目标是形成可运行仿真模拟器 + 视频 demo：

- runnable simulator。
- mp4 video demo。
- 可复现实验命令。
- metrics 输出。
- README 运行说明。

### 计划 demo 列表

1. `B01_single_joint_mpc_demo`
   - 单关节 MuJoCo 模型。
   - 目标角度跟踪。
   - MPC horizon 预测。
   - 导出 mp4。
   - 输出 final error、mean tracking error、max torque、runtime per control step。

2. `B02_two_link_mpc_tracking_demo`
   - 二连杆机械臂。
   - 末端轨迹跟踪。
   - MuJoCo 仿真。
   - 导出 mp4。
   - 输出 tracking error 和 torque 曲线。

3. `B03_rollout_predictive_sampling_demo`
   - 多条未来控制序列 rollout。
   - 选择 cost 最低的控制序列。
   - receding horizon 执行。
   - 导出轨迹可视化视频。

### simulator/ 目录说明

```text
simulator/
├── envs/          # MuJoCo 环境与模型封装
├── controllers/   # MPC 控制器和 baseline 控制器骨架
├── planners/      # rollout / predictive sampling / horizon planner 骨架
├── utils/         # 通用工具、路径、配置、可视化辅助
├── scripts/       # 后续命令行脚本入口
└── README.md
```

### outputs/ 目录说明

```text
outputs/
├── videos/        # mp4 demo
├── figures/       # tracking error、torque、cost 曲线
├── logs/          # 控制日志和运行日志
└── metrics/       # final error、mean error、runtime 等指标
```

### 与 A_self_baseline 的关系

`A_self_baseline` 提供 MuJoCo / Pinocchio / FK / Jacobian / QP-IK / tracking 基础。Project B 不修改 A 的源码，而是在独立目录中学习如何把 A 的单步 tracking 思路推进到 horizon cost 和 MPC。

### 后续最小实现顺序

1. 先实现 `B01_single_joint_mpc_demo` 的模型、rollout、cost、metrics 和视频导出。
2. 再实现 `B02_two_link_mpc_tracking_demo`，连接末端轨迹 tracking。
3. 最后实现 `B03_rollout_predictive_sampling_demo`，展示多 rollout 选择和 receding horizon。
