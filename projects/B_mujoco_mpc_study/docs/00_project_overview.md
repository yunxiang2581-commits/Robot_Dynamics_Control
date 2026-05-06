# Project B 概览：MuJoCo MPC / MJPC

## 项目定位

MuJoCo MPC（MJPC）是 Google DeepMind 开源的实时 Model Predictive Control 框架，基于 MuJoCo 物理引擎，适合学习 simulation-in-the-loop MPC。

对当前学习路径来说，它位于：

```text
URDF / MuJoCo model -> FK / Jacobian / IK -> tracking baseline -> MPC / predictive control
```

Project B 的最终交付不是“读过 MJPC”，而是自建一个 simulation-only 的 MuJoCo MPC Simulator，导出视频 demo，并记录 tracking error、torque、runtime 等指标。

## 已从公开资料确认的信息

- GitHub README 将 MJPC 描述为基于 MuJoCo 的实时 predictive control 交互式应用和软件框架。
- 项目支持用户编写和求解复杂机器人任务。
- 支持 multiple shooting-based planners。
- planner 包括 derivative-based 的 iLQG、Gradient Descent，以及 derivative-free 的 Predictive Sampling。
- 示例任务覆盖 locomotion、manipulation、humanoid motion-capture tracking 等方向。
- 项目提供 Python API，但 README 提醒它仍偏实验性，需要模型与 C++ task residual / transition 函数保持兼容。

## 项目具体功能

- 交互式 MPC 应用：用于可视化和调试 predictive control 行为。
- 自定义机器人 task：通过 task、residual、transition 等接口描述目标。
- 多 planner 支持：便于比较采样、梯度、iLQG 等方法。
- MuJoCo rollout：直接利用物理仿真预测未来状态。
- 多任务场景：包括四足、双臂操作、魔方操作、人形 mocap tracking。

## 待后续源码阅读确认的信息

- `mjpc/tasks` 中每个 task 的 residual 具体如何组织。
- planner 与 agent 的接口边界。
- horizon、cost、policy update 的配置入口。
- Python API 如何调用 C++ service / agent。
- humanoid tracking 示例中的 mocap 数据流与 residual 定义。

## 适合复现的模块

- task residual 的数学思想。
- rollout + horizon cost 的教学版实现。
- single-joint / two-link 的极简 MPC demo。
- receding horizon control 的控制循环。

## 只适合阅读的模块

- 完整 MJPC GUI。
- gRPC service。
- 多任务 C++ 工程组织。
- 高复杂度 humanoid tracking 示例。

## 暂不建议执行的模块

- 完整 clone / build / GUI 运行。
- gRPC 相关构建。
- Python API 安装测试。
- 复杂 humanoid 或 manipulation demo。

## 仿真模拟器交付要求

- runnable simulator：后续在 `simulator/` 中实现。
- simulation video demo：保存到 `outputs/videos/`。
- video export script：规划为 `simulator/record_video.py`。
- reproducible command：规划为 `python simulator/run_demo.py --demo <demo_name> --export-video`。
- demo metrics：保存到 `outputs/metrics/`。
