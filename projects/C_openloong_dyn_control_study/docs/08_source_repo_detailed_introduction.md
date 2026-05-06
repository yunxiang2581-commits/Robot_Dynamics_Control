# OpenLoong-Dyn-Control 详细项目介绍

## 资料来源与当前边界

本文参考本地源码仓库：

```text
external/open_source_repos/OpenLoong-Dyn-Control/README-zh.md
external/open_source_repos/OpenLoong-Dyn-Control/Tutorial.md
external/open_source_repos/OpenLoong-Dyn-Control/algorithm/
external/open_source_repos/OpenLoong-Dyn-Control/demo/
external/open_source_repos/OpenLoong-Dyn-Control/sim_interface/
```

当前只做中文项目介绍和源码阅读整理：

- 不编译 OpenLoong-Dyn-Control。
- 不运行 MuJoCo demo。
- 不修改外部仓库源码。
- 不修改 `A_self_baseline`。

## 一句话定位

OpenLoong-Dyn-Control 是一套面向仿人机器人的运动控制框架，核心是 MPC + WBC，可部署在 MuJoCo 仿真平台上。它基于“青龙”人形机器人模型，提供行走、跳跃、盲踩障碍物等运动示例。

它对当前求职项目的价值，不是直接拿来做实机部署，而是学习完整人形控制栈如何把命令、步态、MPC、WBC、PVT 和 MuJoCo 闭环串起来。

## 项目背景

README-zh 说明，OpenLoong 开源项目由人形机器人（上海）有限公司、国家地方共建人形机器人创新中心与开放原子开源基金会共同运营。本仓库提供的是基于 MPC 与 WBC 的仿人机器人控制框架。

README 明确提到：

- 框架可部署在 MuJoCo 仿真平台。
- 使用“青龙”机器人模型。
- 提供行走、跳跃、盲踩障碍物三个运动示例。
- 在实物样机上实现过行走和盲踩障碍。

对于当前仓库，因为用户没有实物机器人，Project C 只把它作为 simulation-only 学习参考，不做实机部署。

## 项目特点

### 1. 易部署

README 表示仓库包含主要依赖，减少第三方库安装负担。它仍然需要系统层 OpenGL 支持，并建议使用 Ubuntu 22.04.4 LTS 与 g++ 11.4.0。

当前学习中，这一点主要用于了解依赖边界，不执行安装和编译。

### 2. 可扩展

README 强调控制框架采用分层模块化设计，各功能模块在逻辑和功能上有清晰边界。对学习者来说，这说明它适合作为“人形控制架构阅读项目”。

重点不是抄代码，而是理解模块职责：

- 命令输入。
- 步态调度。
- 落脚点规划。
- 状态估计。
- MPC。
- WBC。
- PVT 低层控制。
- MuJoCo 接口。

### 3. 易理解

README 提到项目使用总线进行模块间数据交互，算法实现采用“读取-计算-写入”的简单逻辑。这对学习很重要，因为复杂人形控制项目最容易混乱的地方就是数据流。

Project C 后续应重点画出：

```text
DataBus -> StateEstimator -> GaitScheduler / FootPlacement -> MPC -> WBC_QP -> PVT_Ctr -> MJ_Interface
```

## 主要 demo

本地 `demo/` 目录包含：

```text
float_control.cpp
jump_mpc.cpp
walk_mpc_wbc.cpp
walk_mpc_wbc_joystick.cpp
walk_wbc.cpp
walk_wbc_joystick.cpp
walk_wbc_speed_test.cpp
walk_wbc_staircase.cpp
```

### walk_wbc.cpp

这是理解 WBC 行走控制的入口。它适合用来观察：

- 不使用完整 MPC 时，WBC 如何组织行走任务。
- 支撑腿和摆动腿任务如何切换。
- PVT 控制如何被调用。

### walk_mpc_wbc.cpp

这是理解 MPC + WBC 串联的重点入口。它适合用来观察：

- MPC 输出什么。
- WBC 如何使用 MPC 的结果。
- 步态调度和落脚点规划如何进入控制链。

### jump_mpc.cpp

跳跃任务更复杂，涉及更强的动力学瞬态和接触切换。当前只适合阅读，不适合第一阶段复现。

### joystick demo

README 更新日志提到增加了 `walk_wbc_joystick` 与 `walk_mpc_wbc_joystick`，可以利用键盘控制机器人运动并实现转弯。它们适合后续理解命令输入如何进入 gait / MPC / WBC 流程。

## 核心源码模块

本地 `algorithm/` 目录包含：

```text
Eul_W_filter.cpp / .h
foot_placement.cpp / .h
gait_scheduler.cpp / .h
joystick_interpreter.cpp / .h
mpc.cpp / .h
pino_kin_dyn.cpp / .h
priority_tasks.cpp / .h
StateEst.cpp / .h
wbc_priority.cpp / .h
```

### joystick_interpreter

负责把摇杆或键盘命令解释成机器人运动目标。它是用户输入到控制器的第一层转换。

学习重点：

- 目标速度或目标方向如何表示。
- 转弯命令如何进入步态或 MPC。
- 命令是否经过滤波或限幅。

### gait_scheduler

负责步态相位和腿状态切换。README 中提到 `tSwing` 表示单步时长，`FzThrehold` 表示触地足底力阈值。

学习重点：

- 哪条腿是支撑腿。
- 哪条腿是摆动腿。
- 何时切换。
- 足底力如何影响接触状态判断。

### foot_placement

负责落脚点规划和腾空腿轨迹。README 提到关键参数包括：

- `kp_vx`：腾空腿 x 方向落脚点调节参数。
- `kp_vy`：腾空腿 y 方向落脚点调节参数。
- `kp_wz`：腾空腿 z 方向姿态落脚点调节参数。
- `stepHeight`：抬腿高度。

学习重点：

- 期望速度如何影响落脚点。
- 摆动腿轨迹如何生成。
- foot placement 如何连接 gait scheduler 与 WBC。

### mpc

MPC 是高层动力学优化模块。README 提到 `set_weight` 接口：

```text
set_weight(u_weight, L_diag, K_diag)
```

其中：

- `u_weight`：系统输入最小权重。
- `L_diag`：状态与期望误差权重，顺序为 eul、pos、omega、vel。
- `K_diag`：系统输入权重，顺序为 fl、tl、fr、tr。

学习重点：

- 状态误差如何加权。
- 接触力或输入如何加权。
- MPC 输出如何被 WBC 使用。

### wbc_priority / priority_tasks

WBC 是把高层目标转成全身任务和力矩的关键模块。README 提到任务优先级示例：

```text
RedundantJoints
static_Contact
Roll_Pitch_Yaw_Pz
PxPy
SwingLeg
HandTrack
```

学习重点：

- 接触约束为什么应该是高优先级。
- 躯干姿态、高度、水平位置、摆动腿任务如何排序。
- 任务优先级变化会怎样影响行为。

### pino_kin_dyn

这是 Pinocchio 运动学 / 动力学封装。它对当前仓库特别重要，因为 A 项目也在学习 Pinocchio。

学习重点：

- floating-base humanoid 的 `nq/nv` 如何处理。
- frame / joint 的搜索和映射。
- Jacobian、质心、动力学项如何提供给 WBC 或 MPC。

### StateEst

状态估计模块为 MPC 和 WBC 提供当前状态。

学习重点：

- MuJoCo 仿真中哪些状态来自 ground truth。
- 哪些状态被估计或滤波。
- 状态估计与 DataBus 如何交互。

## 命名约定

README-zh 提供了非常有用的变量命名说明：

- `_L`、`_W`：本体坐标系、世界坐标系。
- `fe_`：足末端。
- `_L/_l/_R/_r`：左侧、右侧。
- `swing/sw`：摆动腿。
- `stance/st`：支撑腿。
- `eul/rpy`：姿态角。
- `omega`：角速度。
- `pos`：位置。
- `vel`：线速度。
- `tor/tau`：力矩。
- `base`：BaseLink。
- `_des`：期望值。
- `_cur`：当前实际值。
- `_rot`：坐标变换矩阵。

这套命名约定值得在阅读源码时单独记录，因为它能降低理解成本。

## 安装与运行信息

README 给出的运行流程包括：

```text
git clone
mkdir build
cmake ..
make
./walk_mpc_wbc 或 ./walk_wbc 或 ./jump_mpc
```

但在当前仓库中，这些命令只作为资料记录，不执行。

## 对当前仓库的学习价值

Project C 的学习价值主要有：

1. 学习人形机器人分层控制架构。
2. 学习 DataBus 式模块数据交互。
3. 学习 gait scheduler 和 foot placement 如何服务行走。
4. 学习 MPC 如何输出质心、姿态或接触相关目标。
5. 学习 WBC-QP 如何组织任务优先级。
6. 学习 PVT 低层控制如何接收高层命令。
7. 学习 MuJoCo 闭环仿真如何承载人形控制。

## 风险与边界

OpenLoong-Dyn-Control 涉及完整人形控制栈，复杂度高于当前 A 项目。当前不建议：

- 直接把它并入 A。
- 直接运行原 demo。
- 直接修改其源码。
- 直接进入跳跃或障碍任务。
- 讨论或执行实机部署。

## 后续源码阅读建议

推荐顺序：

1. `README-zh.md`：理解项目定位和术语。
2. `demo/walk_wbc.cpp`：理解 WBC 行走入口。
3. `demo/walk_mpc_wbc.cpp`：理解 MPC + WBC 主流程。
4. `algorithm/gait_scheduler.*`：理解步态相位。
5. `algorithm/foot_placement.*`：理解落脚点。
6. `algorithm/mpc.*`：理解 MPC 权重和输入输出。
7. `algorithm/wbc_priority.*` 与 `priority_tasks.*`：理解 WBC 任务。
8. `algorithm/pino_kin_dyn.*`：理解 Pinocchio 封装。
9. `algorithm/StateEst.*`：理解状态估计。

## 与 Project C 仿真器的连接

后续自建 OpenLoong-inspired Humanoid WBC Simulator 时，不复现完整工程，而抽象最小可解释模块：

```text
envs: 简化双足或浮动基模型
planners: contact schedule / CoM target / foot placement
controllers: contact force allocation / simplified WBC-QP
metrics: CoM error、contact force violation、QP solve status
record_video.py: 接触力箭头、平衡动画、数据流动画导出
```

第一阶段建议从 `C01_contact_force_allocation_demo` 开始，因为它能在不完整实现人形行走的情况下，先理解接触力和约束。
