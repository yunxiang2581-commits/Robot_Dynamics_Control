# legged_control 详细项目介绍

## 资料来源与当前边界

本文参考本地源码仓库：

```text
external/open_source_repos/legged_control/README.md
external/open_source_repos/legged_control/docs/
external/open_source_repos/legged_control/legged_interface/
external/open_source_repos/legged_control/legged_wbc/
external/open_source_repos/legged_control/legged_estimation/
```

当前只做中文项目介绍和源码阅读整理：

- 不编译 legged_control。
- 不运行 Gazebo。
- 不运行 ROS controller。
- 不运行硬件接口。
- 不修改外部仓库源码。
- 不修改 `A_self_baseline`。

## 一句话定位

legged_control 是一个基于 OCS2 与 ros-control 的腿式机器人 NMPC-WBC 控制栈，覆盖 NMPC、WBC、状态估计和 sim2real。README 明确提示该项目已经不再维护，因此当前只适合作为复杂四足控制架构的阅读参考，不作为长期工程依赖。

对当前求职项目来说，它的最大价值是提供一个完整四足控制栈的结构样板：

```text
target velocity / goal
-> torso state trajectory
-> NMPC
-> optimized state and input
-> WBC
-> joint torque feedforward + low-gain PD
-> robot / simulation
-> state estimation
```

## 项目状态提醒

README 开头明确说明：

- 该软件已经不再维护。
- 作者正在开发新的框架。
- 若关注带感知的 pipeline，可参考另一个项目 legged_perceptive。

这意味着：

- 不建议把 legged_control 作为长期工程依赖。
- 不建议现在搭完整 ROS / OCS2 环境。
- 不建议直接追求运行原项目。
- 更适合作为 NMPC / WBC / estimation 架构阅读对象。

## 项目背景

README 将 legged_control 描述为：

```text
NMPC, WBC, state estimation, and sim2real framework for legged robots based on OCS2 and ros-controls
```

项目作者认为它可以为腿式机器人社区提供高性能、易使用的 model-based baseline。README 还提到它可部署到 Unitree A1，且 ros-control interface 让用户可以适配自定义机器人。

但对于当前仓库，用户没有实物机器人，因此所有硬件相关内容只作为背景知识，不进入执行计划。

## 依赖与运行方式

README 中的完整使用流程涉及：

- OCS2。
- Pinocchio。
- hpp-fcl。
- ocs2_robotic_assets。
- catkin tools。
- Gazebo。
- RViz。
- ros-control。
- Unitree hardware interface。

README 还特别提醒：OCS2 是巨大 monorepo，不要编译整个 OCS2，只需要编译 `ocs2_legged_robot_ros` 及其依赖。

当前仓库不执行这些步骤，因为它们属于重依赖运行环境，不适合当前资料整理和 simulation-only 规划阶段。

## 框架主线

README 的 Framework 部分给出系统流程：

### 1. 目标输入

机器人 torso 的期望速度或目标位置会被转换成 state trajectory，并发送给 NMPC。

这说明上层命令不是直接变成关节力矩，而是先被整理成适合 optimal control 的参考轨迹。

### 2. NMPC

NMPC 根据当前状态、参考轨迹、动力学、约束和代价，计算优化后的系统状态和输入。

在 legged_control 中，NMPC 是高层预测控制器，负责未来 horizon 内的最优行为。

### 3. WBC

WBC 根据 NMPC 输出的 optimized states and inputs 计算关节力矩。

这一步的作用是把高层质心 / 接触力 / 状态目标转换成当前时刻的全身控制命令。

### 4. Torque feedforward + low-gain PD

WBC 输出的 torque 作为 feed-forward 项发送给电机控制器，同时叠加低增益 joint-space position / velocity PD。

README 解释低增益 PD 的作用是：

- 减小足端接触冲击。
- 提高跟踪性能。

### 5. State estimation

NMPC 和 WBC 都需要当前机器人状态，包括：

- base orientation。
- joint state。
- base position。
- base velocity。

README 提到这些信息来自 IMU、电机反馈、base acceleration、joint foot position measurements，并在与 WBC 同一个 loop 中使用 linear Kalman filter 估计 base position 和 velocity。

## NMPC 数学问题

README 给出 NMPC 的一般 optimal control problem：

```text
minimize terminal cost + integral running cost
subject to:
  initial state
  system flow map
  state-input equality constraints
  state-only equality constraints
  inequality constraints
```

在 legged_control 中，状态和输入定义为：

```text
x = [h_com, q_b, q_j]
u = [f_c, v_j]
```

含义：

- `h_com`：normalized centroidal momentum，维度为 6。
- `q_b`：base generalized coordinate。
- `q_j`：joint positions。
- `f_c`：四个接触点的 contact forces，总维度为 12。
- `v_j`：joint velocities。

README 说明 cost function 主要是对所有状态误差和输入的 quadratic tracking cost。

系统动力学使用 centroidal dynamics，并包含以下约束：

- friction cone。
- standing foot 不运动。
- swinging foot 的 z 轴位置满足 gait-generated curve。

求解方式上，README 提到：

- 使用 multiple shooting 将 optimal control problem 转写为 NLP。
- 使用 SQP 求解 NLP。
- QP 子问题使用 HPIPM。

这对当前学习非常关键，因为它展示了四足 NMPC 的核心组织方式：状态、输入、动力学、接触约束、摆动足约束和代价函数。

## WBC 数学问题

README 明确说明 WBC 只考虑当前时刻。WBC 中每个 task 都是对决策变量的等式或不等式约束。

WBC 决策变量为：

```text
x_wbc = [qddot, f_c, tau]
```

含义：

- `qddot`：广义坐标加速度。
- `f_c`：接触力。
- `tau`：关节力矩。

README 说明 WBC 在高优先级任务线性约束的 null space 中求解 QP，并尝试最小化不等式约束的 slack variables。这种方式可以考虑完整非线性刚体动力学，并保证严格层级任务结果。

对当前学习来说，WBC 部分最适合抽象成：

```text
动力学等式约束
接触力约束
摩擦锥约束
足端任务
base / torso tracking
tau limit
slack variable
```

## 顶层模块结构

本地仓库顶层包含：

```text
legged_common
legged_control
legged_controllers
legged_estimation
legged_examples
legged_gazebo
legged_hw
legged_interface
legged_wbc
qpoases_catkin
docs
```

### legged_common

通用类型、参数、工具函数和共享数据结构的候选位置。后续阅读时应关注机器人状态、接触状态、命名和配置如何统一。

### legged_interface

很可能是 NMPC 与 OCS2 问题定义的核心入口。后续应重点阅读：

- state 和 input 的定义。
- dynamics。
- cost。
- constraints。
- reference manager。
- gait schedule 如何进入 OCS2。

### legged_wbc

WBC-QP 的核心模块。后续应重点阅读：

- 决策变量 `qddot / f_c / tau` 如何排列。
- 动力学约束如何构造。
- task 如何组织。
- slack variable 如何使用。
- QP 求解器如何调用。

### legged_estimation

状态估计模块。后续应重点阅读：

- Kalman filter 的状态向量。
- IMU 和 joint feedback 如何进入估计器。
- foot position measurement 如何用于 base velocity / position 估计。
- cheater mode 与真实估计之间的差异。

### legged_controllers

ros-control controller 的运行入口。它连接 NMPC、WBC、state estimation 和 ros-control update loop。

当前只读，不运行。

### legged_gazebo

Gazebo 仿真相关模块。当前不运行。

### legged_hw / legged_examples

硬件接口和 Unitree 示例相关模块。当前只作为架构背景，不做任何执行和修改。

## Quick Start 的安全边界

README 的 Quick Start 包括：

- 设置 `ROBOT_TYPE=a1`。
- 启动 Gazebo simulation。
- 或启动 robot hardware。
- load controller。
- switch controller。
- 使用 RViz、`cmd_vel`、`move_base_simple/goal` 控制。

当前仓库禁止执行这些步骤。尤其是：

- 不运行 `legged_hw`。
- 不运行 robot hardware launch。
- 不运行 controller manager。
- 不接 Unitree。
- 不运行 Gazebo demo。

## 对当前仓库的学习价值

Project D 的学习价值主要有：

1. 理解四足 NMPC 的状态、输入、约束和代价组织。
2. 理解 switched systems 和 contact schedule 为什么是四足运动核心。
3. 理解 friction cone 和 standing foot no-motion constraints。
4. 理解 WBC-QP 的决策变量 `qddot / f_c / tau`。
5. 理解 torque feedforward + low-gain PD 的控制分层。
6. 理解状态估计如何服务 NMPC 和 WBC。
7. 理解复杂 ROS 控制栈的模块划分，但不被环境依赖拖住。

## 风险与边界

主要风险：

- 项目不再维护。
- ROS / OCS2 / catkin / Gazebo / Pinocchio 依赖重。
- 真机相关接口多，不适合当前用户环境。
- 直接运行原项目可能偏离当前 simulation-only 学习目标。

当前正确做法：

- 阅读 README 和关键模块。
- 抽象 NMPC / WBC / estimation 数学结构。
- 自建最小四足 contact QP、trot gait、state estimation demo。
- 导出视频和指标，不追求原项目完整运行。

## 后续源码阅读建议

推荐顺序：

1. `README.md`：理解项目状态、框架、NMPC 和 WBC 公式。
2. `docs/system_diagram.png`：看系统数据流。
3. `legged_interface/`：读 NMPC 问题定义。
4. `legged_wbc/`：读 WBC-QP。
5. `legged_estimation/`：读 Kalman filter 状态估计。
6. `legged_controllers/`：只读控制循环入口。
7. `legged_examples/legged_unitree/`：只读硬件适配思路，不执行。
8. `legged_gazebo/`：只读仿真接口，不运行。

## 与 Project D 仿真器的连接

后续自建 legged_control-inspired Quadruped Simulator 时，不复现 ROS / OCS2 全工程，而抽象三个最小 demo：

```text
D01_quadruped_contact_qp_demo:
  学习四足接触力分配和摩擦锥约束。

D02_trot_gait_schedule_visual_demo:
  学习 contact schedule、支撑足 / 摆动足相位切换。

D03_state_estimation_tracking_demo:
  学习带噪声仿真真值下的 base velocity 估计。
```

最终交付应包括：

- simulation-only runnable demo。
- mp4 视频或图表。
- metrics CSV。
- contact force residual。
- friction cone violation。
- gait phase correctness。
- state estimation RMSE。
