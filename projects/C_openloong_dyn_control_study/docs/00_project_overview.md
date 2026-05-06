# Project C 概览：OpenLoong-Dyn-Control

## 项目定位

OpenLoong-Dyn-Control 是人形机器人运动控制框架，基于 MPC + WBC，可部署在 MuJoCo 仿真平台，面向 walking、jumping、blind obstacle stepping 等动作。

Project C 的最终交付不是“读过 OpenLoong-Dyn-Control”，而是自建一个 simulation-only 的简化人形 / 双足 WBC Simulator，导出视频 demo，并记录 CoM tracking error、接触力约束违反、balance duration、QP solve status 等指标。

## 已从公开资料确认的信息

- README 标题明确为 humanoid robots 的 MPC and WBC motion control framework。
- README 描述该项目可部署到 MuJoCo simulation platform。
- README 提到三类 motion examples：walking、jumping、blind obstacle stepping。
- README 提到控制框架采用 layered modular design。
- README 提到模块间通过 bus 进行数据交互。
- README 环境说明提到项目包含或使用 MuJoCo、Pinocchio、Eigen、Quill、GLFW、JsonCpp 等组件。

## 项目具体功能

- Joystick / command 输入。
- GaitScheduler 步态调度。
- FootPlacement 落脚点规划。
- MPC 计算 CoM 轨迹和接触力。
- WBC 转换为关节级任务或力矩。
- PVT 执行底层关节控制。
- MuJoCo 闭环仿真。
- StateEstimator 状态估计。

## 待后续源码阅读确认的信息

- DataBus 中具体字段和数据生命周期。
- MPC 状态量、输入量和约束的具体定义。
- WBC_QP 的决策变量、任务优先级和约束矩阵。
- PVT_Ctr 的控制律和参数。
- walk / jump / obstacle stepping demo 之间共享和差异的模块。

## 适合复现的模块

- MPC-WBC-PVT 数据流图。
- 简化 WBC-QP 数学形式。
- 简化浮动基 / 双足接触力分配例子。
- GaitScheduler 和 FootPlacement 的概念级伪代码。

## 只适合阅读的模块

- 完整 MuJoCo 闭环 demo。
- jumping 和 blind obstacle stepping 复杂动作。
- 真实机器人部署相关内容。

## 暂不建议执行的模块

- 编译完整工程。
- 运行原项目 demo。
- 直接修改 A 项目来接入 C 的控制架构。

## 仿真模拟器交付要求

- runnable simulator：后续在 `simulator/` 中实现。
- simulation video demo：保存到 `outputs/videos/`。
- video export script：规划为 `simulator/record_video.py`。
- reproducible command：规划为 `python simulator/run_demo.py --demo <demo_name> --export-video`。
- demo metrics：保存到 `outputs/metrics/`。
