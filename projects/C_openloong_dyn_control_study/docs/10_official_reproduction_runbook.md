# Project C 官方 OpenLoong-Dyn-Control 复现 Runbook

本文档用于把 Project C 的下一步从“阅读与规划”推进到“官方工程复现”。

复现对象不是本仓库自建的简化 demo，而是本地已经 clone 的官方源码：

```text
external/open_source_repos/OpenLoong-Dyn-Control/
```

## 1. 本轮学习目标

这一轮要验证的是完整工程链路是否能在本机跑起来：

```text
MuJoCo sensors
-> MJ_Interface
-> DataBus
-> Pin_KinDyn
-> GaitScheduler / FootPlacement
-> MPC 或 WBC_priority
-> PVT_Ctr
-> MuJoCo torque control
```

为什么先从官方 demo 开始：

- `walk_wbc` 能先验证 MuJoCo、Pinocchio、WBC、PVT 和模型资源。
- `walk_mpc_wbc` 在 `walk_wbc` 成功后再验证 MPC + WBC 完整链路。
- 第一阶段不改控制算法，不调控制参数，只保存构建日志、运行日志、截图 / 视频和架构理解材料。

输入：官方源码、`models/scene_board.xml`、`models/scene.xml`、`models/AzureLoong.urdf`、`common/joint_ctrl_config.json`。

输出：C++ 可执行文件、MuJoCo 窗口、`record/datalog.log`、构建日志、运行日志、截图 / 视频、复现报告。

数学逻辑：第一阶段不改变官方 MPC、WBC、PVT、步态调度、Pinocchio 动力学或 MuJoCo 模型逻辑。

参数变化：第一阶段不修改控制参数，只记录观察到的 `dt`、`dt_200Hz`、`mpc_N`、QP 状态和 demo 时间节点。

主要风险：系统依赖、OpenGL / GLFW 显示、官方源码工作区状态、运行目录、日志输出路径。

## 2. 2026-05-30 预检结果

当前预检结果如下：

```text
外部源码 commit: 4dd7a7e4
外部源码路径: external/open_source_repos/OpenLoong-Dyn-Control/
操作系统: Ubuntu 25.10
g++: 15.2.0
make: /usr/bin/make
cmake: 未安装
gcc-11 / g++-11: 未安装
sudo apt: 需要用户输入密码，Codex 无法代输
```

外部源码当前 `git status` 显示大量 `M`。抽样检查 `CMakeLists.txt`、`demo/walk_wbc.cpp`、`algorithm/wbc_priority.cpp` 后，主要表现是 LF/CRLF 行尾变化，不是算法内容差异。第一阶段不要在外部源码中手工修改控制代码。

外部源码还存在：

```text
external/open_source_repos/OpenLoong-Dyn-Control/.git/index.lock
```

如果后续 git 命令提示 lock 错误，应先确认没有其他 git 进程正在操作外部仓库，再决定是否清理该 lock 文件。第一阶段不主动删除它。

## 3. 用户需要先执行的系统依赖命令

官方推荐环境是 Ubuntu 22.04 + g++ 11。当前环境是 Ubuntu 25.10 + g++ 15，所以推荐优先在 Ubuntu 22.04 / WSL2 Ubuntu 22.04 中复现。

如果继续使用当前环境，先安装最小构建依赖：

```bash
sudo apt-get update
sudo apt-get install -y cmake gcc-11 g++-11 libglu1-mesa-dev freeglut3-dev
```

安装后验证：

```bash
cmake --version
gcc-11 --version
g++-11 --version
make --version
```

如果 Ubuntu 25.10 源里没有 `gcc-11/g++-11`，先不要急着调代码。优先选择：

1. 切换到 Ubuntu 22.04 / WSL2 Ubuntu 22.04。
2. 或先只安装 `cmake`，用当前 `g++ 15.2.0` 做一次“风险构建”，把失败日志保存下来再判断。

## 4. 官方源码只读原则

第一阶段遵守以下规则：

- 不修改 `external/open_source_repos/OpenLoong-Dyn-Control/` 内部源码。
- 不改官方控制参数。
- 不删除官方文件。
- 不在外部源码中提交 commit。
- 所有复现证据写入 Project C 输出目录。

建议证据目录：

```text
projects/C_openloong_dyn_control_study/outputs/external_reproduction/openloong/20260530_walk_wbc_baseline/
```

建议子目录：

```text
logs/
screenshots/
videos/
record/
reports/
```

## 5. 依赖满足后的构建命令

以下命令应在依赖安装完成后执行。

```bash
export REPO_ROOT=/home/ubuntu/Robot_Dynamics_Control
export OPENLOONG_ROOT="$REPO_ROOT/external/open_source_repos/OpenLoong-Dyn-Control"
export RUN_ROOT="$REPO_ROOT/projects/C_openloong_dyn_control_study/outputs/external_reproduction/openloong/20260530_walk_wbc_baseline"

mkdir -p "$RUN_ROOT/logs" "$RUN_ROOT/screenshots" "$RUN_ROOT/videos" "$RUN_ROOT/record" "$RUN_ROOT/reports"

cd "$OPENLOONG_ROOT"
git --no-optional-locks rev-parse --short HEAD > "$RUN_ROOT/00_source_commit.txt"
git --no-optional-locks status --short --branch > "$RUN_ROOT/01_source_status.txt"

mkdir -p build
cd build
CC=gcc-11 CXX=g++-11 cmake .. 2>&1 | tee "$RUN_ROOT/logs/02_cmake_configure.log"
make -j"$(nproc)" 2>&1 | tee "$RUN_ROOT/logs/03_make_build.log"
```

构建成功后应看到这些可执行文件中的至少前两个：

```text
build/walk_wbc
build/walk_mpc_wbc
build/jump_mpc
build/float_control
build/wbc_speed_test
build/walk_wbc_joystick
build/walk_mpc_wbc_joystick
build/walk_wbc_staircase
```

## 6. 第一轮运行：walk_wbc

`walk_wbc` 是第一目标，因为它先验证 WBC + PVT + MuJoCo 闭环，不把 MPC 风险混进来。

```bash
export REPO_ROOT=/home/ubuntu/Robot_Dynamics_Control
export OPENLOONG_ROOT="$REPO_ROOT/external/open_source_repos/OpenLoong-Dyn-Control"
export RUN_ROOT="$REPO_ROOT/projects/C_openloong_dyn_control_study/outputs/external_reproduction/openloong/20260530_walk_wbc_baseline"

cd "$OPENLOONG_ROOT/build"
./walk_wbc 2>&1 | tee "$RUN_ROOT/logs/04_walk_wbc.log"
```

运行时需要注意：

- demo 要从 `build/` 目录启动，因为模型路径是相对路径 `../models/...`。
- 如果 GLFW / OpenGL 窗口打不开，优先排查 WSLg、X server 或图形驱动，不要改控制算法。
- demo 可能写入 `external/open_source_repos/OpenLoong-Dyn-Control/record/datalog.log`。

运行结束后复制日志证据：

```bash
cp "$OPENLOONG_ROOT/record/datalog.log" "$RUN_ROOT/record/datalog_walk_wbc.log"
```

通过标准：

```text
walk_wbc 能启动 MuJoCo 窗口
仿真能运行超过 startSteppingTime=3s
接近或进入 startWalkingTime=5s 后的行走阶段
record/datalog.log 有 base、foot force、motor state 等数据
至少保存一张截图或一段短视频
```

## 7. 第二轮运行：walk_mpc_wbc

`walk_mpc_wbc` 是第二目标，因为它在 WBC 基础上加入 MPC QP 链路。

```bash
export REPO_ROOT=/home/ubuntu/Robot_Dynamics_Control
export OPENLOONG_ROOT="$REPO_ROOT/external/open_source_repos/OpenLoong-Dyn-Control"
export RUN_ROOT="$REPO_ROOT/projects/C_openloong_dyn_control_study/outputs/external_reproduction/openloong/20260530_walk_wbc_baseline"

cd "$OPENLOONG_ROOT/build"
./walk_mpc_wbc 2>&1 | tee "$RUN_ROOT/logs/05_walk_mpc_wbc.log"
```

运行结束后复制日志证据：

```bash
cp "$OPENLOONG_ROOT/record/datalog.log" "$RUN_ROOT/record/datalog_walk_mpc_wbc.log"
```

重点观察：

```text
MPC 是否按 dt_200Hz = 0.005 的节奏参与控制
MPC QP status 是否正常
WBC QP status 是否正常
Fr_ff / X_cal / dX_cal 是否写回 DataBus
MPC+WBC 相比 walk_wbc 增加了哪些输入输出
```

## 8. 复现报告最低内容

成功或失败都要记录。第一版报告建议写到：

```text
projects/C_openloong_dyn_control_study/outputs/external_reproduction/openloong/20260530_walk_wbc_baseline/reports/REPRODUCTION_REPORT.md
```

报告至少包含：

- 环境：OS、compiler、cmake、make、显示环境。
- 源码：commit、source status、是否有行尾变化。
- 构建：configure 命令、build 命令、成功 / 失败日志路径。
- 运行：`walk_wbc` 和 `walk_mpc_wbc` 命令、运行时长、日志路径。
- 证据：截图、视频、`datalog.log`。
- 问题：失败现象、根因假设、下一步验证。
- 架构理解：`MJ_Interface -> DataBus -> Pin_KinDyn -> MPC/WBC -> PVT -> MuJoCo`。

## 9. 当前下一步

当前硬阻塞是：

```text
cmake 未安装，sudo apt 需要用户输入密码。
```

用户安装依赖后，继续从本文档第 5 节开始执行构建。若构建失败，先保存 `02_cmake_configure.log` 或 `03_make_build.log`，再根据第一条真实错误定位，不直接修改算法代码。
