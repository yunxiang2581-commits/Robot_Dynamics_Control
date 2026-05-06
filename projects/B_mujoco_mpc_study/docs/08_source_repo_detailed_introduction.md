# MuJoCo MPC / MJPC 详细项目介绍

## 资料来源与当前边界

本文参考本地源码仓库：

```text
external/open_source_repos/mujoco_mpc/README.md
external/open_source_repos/mujoco_mpc/docs/
external/open_source_repos/mujoco_mpc/mjpc/
external/open_source_repos/mujoco_mpc/python/
```

当前只做源码阅读参考与中文整理：

- 不编译 MJPC。
- 不运行 GUI。
- 不运行 Python API demo。
- 不修改外部仓库源码。
- 不修改 `A_self_baseline`。

## 一句话定位

MuJoCo MPC（MJPC）是 Google DeepMind 开源的、基于 MuJoCo 物理引擎的实时 predictive control 研究框架。它提供交互式 GUI、任务定义接口和多种 planner，用于在仿真中实时求解复杂机器人控制任务。

从学习路线看，MJPC 是从“当前时刻的 IK / QP 修正”走向“带未来预测 horizon 的模型预测控制”的重要参考项目。

```text
A_self_baseline: FK / Jacobian / QP-IK / task-space tracking
MJPC: task residual -> rollout -> horizon cost -> planner -> receding horizon control
```

## 项目背景

README 明确说明，MJPC 是一个 interactive application and software framework，用于 real-time predictive control with MuJoCo。它配套论文为：

```text
Predictive Sampling: Real-time Behaviour Synthesis with MuJoCo
```

这说明 MJPC 的核心不是单一机器人模型，也不是只为某个 demo 服务，而是一个围绕 MuJoCo 仿真器组织起来的通用预测控制框架。它关心的问题是：给定当前状态、任务 residual、控制变量和未来 horizon，如何通过仿真 rollout 评估未来行为，并实时选择控制动作。

## 核心能力

### 1. 实时 predictive control

MJPC 的控制循环可以理解为：

```text
读取当前状态
-> 构造未来控制候选或优化变量
-> 使用 MuJoCo rollout 预测未来状态
-> 根据 task residual / cost 评价轨迹
-> planner 更新控制序列
-> 只执行第一步控制
-> 下一控制周期重新优化
```

这正是 receding horizon control 的基本思想。

### 2. 复杂机器人 task 定义

README 提到 MJPC 允许用户 author and solve complex robotics tasks。这里的 task 不只是一个目标点，而是包含模型、目标、残差、权重、约束或 transition 等组织方式。

对当前学习最有价值的是：

- residual 如何表示任务误差。
- cost 如何由 residual 组合而来。
- task 如何与 planner 解耦。
- 同一个 planner 如何服务不同机器人任务。

### 3. 多种 planner

README 明确列出 MJPC 支持 multiple shooting-based planners，包括：

- iLQG。
- Gradient Descent。
- Predictive Sampling。

其中 iLQG 和 Gradient Descent 是 derivative-based 方法，Predictive Sampling 是 derivative-free 方法。

对学习者来说，这三个 planner 代表三类思路：

- iLQG：在名义轨迹附近进行局部二次近似和反馈控制更新。
- Gradient Descent：直接沿 cost 梯度调整控制序列。
- Predictive Sampling：采样多条未来控制序列，通过 rollout 和 cost 选择更好的序列。

Project B 后续最适合先复现 Predictive Sampling，因为它对数学推导和代码依赖最少，更容易形成可解释的 simulation-only demo。

## 示例任务

README 展示了多个任务方向：

- Quadruped task。
- Bimanual manipulation。
- Rubik's cube 10-move unscramble。
- Humanoid motion-capture tracking。

这些示例说明 MJPC 不是单一任务工程，而是围绕“MuJoCo 模型 + task residual + planner”的通用控制框架。

对当前求职项目最相关的方向是：

- humanoid motion-capture tracking：与 `A_self_baseline/scripts/06_target_mocap_tracking.py` 的目标跟踪主题接近。
- quadruped task：可为后续 Project D 的四足控制学习提供概念桥梁。
- manipulation：可作为二连杆或机械臂 tracking demo 的远期参考。

## 顶层源码结构初读

本地 `mjpc/` 目录包含：

```text
agent.cc / agent.h
app.cc / app.h
simulate.cc / simulate.h
task.cc / task.h
trajectory.cc / trajectory.h
norm.cc / norm.h
planners/
tasks/
states/
estimators/
direct/
grpc/
```

### agent

`agent` 很可能是控制器主协调层。后续源码阅读应重点确认：

- 当前 MuJoCo 状态如何进入 agent。
- agent 如何调用 task。
- agent 如何调用 planner。
- planner 输出如何转成下一步控制。

### task

`task` 是理解 MJPC 的关键入口。后续应重点看：

- task 如何声明 residual。
- residual 的维度和权重在哪里定义。
- task 是否负责 transition。
- task 与具体 MuJoCo model 如何绑定。

### planners

`planners/` 是理解不同 MPC 求解方法的入口。建议阅读顺序：

1. Predictive Sampling。
2. Gradient Descent。
3. iLQG。

先读 Predictive Sampling 的原因是它更接近“采样候选控制 -> rollout -> cost 排序”的直观教学 demo。

### simulate

`simulate` 负责把控制候选放入 MuJoCo 中向前推演。对 Project B 后续仿真器来说，这部分对应：

```text
simulator/envs/
simulator/planners/
simulator/run_demo.py
```

### trajectory

`trajectory` 可能保存状态序列、控制序列、时间序列或 rollout 结果。它对应 Project B 后续的：

- tracking error 曲线。
- control input log。
- rollout cost 记录。

### Python API

README 提到 MJPC 提供实验性 Python API，示例在：

```text
python/mujoco_mpc/demos
```

但 README 也提醒：Python 中定义的 model 必须和 C++ task residual / transition 函数兼容，当前 API 对这种兼容性缺乏自动错误检查，调试可能较难。

因此当前阶段不建议直接运行 Python API，而是先自建更小的 Python/MuJoCo 教学仿真器。

## 安装与运行信息

README 中给出的完整运行流程包括：

- 安装 CMake、Ninja、OpenGL 相关库、clang-12 等依赖。
- CMake configure。
- build。
- 运行 `mjpc` GUI。
- 可选构建 gRPC service。
- 可选安装 Python API。

README 特别提醒 gRPC 是大依赖，初次下载可能耗时 10-20 分钟。

本仓库当前只把这些信息作为依赖风险记录，不执行安装、编译或运行。

## 已知问题与风险

README 明确说明 MJPC 是 research prototype，不是 production-quality software。已知风险包括：

- Windows 未经测试。
- XML 中 task specification、norm 参数设置仍有些笨重。
- Gradient Descent 的搜索步长与 cost scale 相关，需要按 task 调参。
- Python API 对 model 与 C++ task 的兼容性缺少自动检查。

对当前项目的含义：

- 不应直接把 MJPC 当作稳定工程依赖。
- 不应一开始复现完整 GUI 或 gRPC。
- 应先复现最小 MPC 概念闭环。

## 对当前仓库的学习价值

Project B 最重要的学习价值有五点：

1. 学会把 task-space tracking 写成 residual。
2. 学会在 horizon 上累计 cost，而不是只看当前时刻误差。
3. 学会用 MuJoCo rollout 评价未来控制序列。
4. 学会区分 task、planner、simulator、trajectory log。
5. 学会把可运行 demo 变成视频和指标，而不是只停留在源码阅读。

## 后续源码阅读建议

推荐顺序：

1. `README.md`：确认项目定位和 planner 类型。
2. `docs/OVERVIEW.md`：理解 predictive control 文档。
3. `mjpc/task.h` / `mjpc/task.cc`：理解 task 抽象。
4. `mjpc/planners/`：先读 Predictive Sampling。
5. `mjpc/agent.h` / `mjpc/agent.cc`：理解控制循环。
6. `mjpc/tasks/`：挑一个简单 task 看 residual 实现。
7. `python/mujoco_mpc/demos/`：只读调用方式，不先运行。

## 与 Project B 仿真器的连接

后续自建 MuJoCo MPC Simulator 时，不复刻 MJPC 工程，而抽象它的核心思想：

```text
envs: MuJoCo 模型与 step / reset / render
controllers: baseline PD 或 MPC 控制器
planners: predictive sampling / rollout planner
metrics: tracking error、torque、runtime
record_video.py: mp4 导出
run_demo.py: demo 统一入口
```

第一阶段建议从 `B01_single_joint_mpc_demo` 开始，因为它能最小化模型复杂度，把注意力集中在 horizon、rollout 和 cost 上。
