# OpenLoong-Dyn-Control 工程复现审计

## 0. 审计结论

本文件用于把 Project C 从“学习型 simulation-only 规划”切换为“求职导向的工程型外部项目复现”。主复现对象是 `OpenLoong-Dyn-Control` 官方仓库，而不是本仓库自建的简化 demo。

第一轮复现目标建议锁定为：

1. 在 Ubuntu 22.04 / WSL2 Ubuntu 22.04 环境完成官方 CMake 构建。
2. 跑通 `walk_wbc`，获取 MuJoCo 人形行走仿真截图 / 视频 / 控制日志。
3. 再跑 `walk_mpc_wbc`，验证 MPC + WBC + PVT 完整控制链。
4. 整理模块数据流：`MJ_Interface -> DataBus -> Pin_KinDyn -> GaitScheduler / FootPlacement / MPC -> WBC_priority -> PVT_Ctr -> MuJoCo torque`。
5. 输出求职可展示材料：复现报告、架构图、运行证据、问题记录和简历项目描述。

这类复现的价值不是“自己重新实现 OpenLoong”，而是证明能够把一个完整 C++ 人形机器人控制工程拉起来、定位依赖、理解控制链路、保存证据并讲清楚技术细节。

## 1. 外部源码身份

本地源码位置：

```text
external/open_source_repos/OpenLoong-Dyn-Control
```

上游仓库：

```text
https://github.com/loongOpen/OpenLoong-Dyn-Control.git
```

当前本地提交：

```text
4dd7a7e4 fix the copyright
```

当前外部仓库状态：

```text
## main...origin/main
```

第一轮复现应保持官方源码只读。除非后续明确进入“补丁复现”阶段，否则不要在 `external/open_source_repos/OpenLoong-Dyn-Control` 内修改源码。运行日志、截图、视频和报告统一写入 Project C 输出目录：

```text
projects/C_openloong_dyn_control_study/outputs/external_reproduction/openloong/20260530_walk_wbc_baseline/
```

## 2. 项目定位与求职价值

OpenLoong-Dyn-Control 是一个基于 MuJoCo 的人形机器人运动控制框架，官方 README 明确其核心是：

```text
Humanoid robot motion control framework based on MPC and WBC
```

它比 MJPC 更适合作为求职型复现主项目，原因是它具备完整机器人控制工程形态：

- 有真实人形机器人模型：`AzureLoong.urdf`、`scene.xml`、mesh assets。
- 有完整 C++ 控制链：MuJoCo interface、DataBus、Pinocchio kinematics/dynamics、GaitScheduler、FootPlacement、MPC、WBC、PVT joint control。
- 有官方可执行 demo：`walk_wbc`、`walk_mpc_wbc`、`jump_mpc`、`walk_wbc_joystick`、`walk_mpc_wbc_joystick`。
- 有日志系统：`DataLogger` 写入 `record/datalog.log`。
- 有求职关键词：humanoid、MPC、WBC、Pinocchio、MuJoCo、qpOASES、floating-base dynamics、contact force、joint torque control。

求职展示时建议描述为：

```text
Reproduced OpenLoong-Dyn-Control, a C++ humanoid MuJoCo control stack based on MPC and WBC. Built the official project, ran walking demos, traced the DataBus-centered control pipeline, and documented module inputs/outputs, runtime logs, and reproduction risks.
```

## 3. 官方 demo 分层

官方 CMake 生成以下主要可执行文件：

```text
walk_mpc_wbc
walk_wbc
jump_mpc
float_control
wbc_speed_test
walk_wbc_joystick
walk_mpc_wbc_joystick
walk_wbc_staircase
```

第一轮不建议一口气跑所有 demo。推荐顺序如下。

### 3.1 第一目标：walk_wbc

源码入口：

```text
demo/walk_wbc.cpp
```

模型入口：

```text
models/scene_board.xml
models/AzureLoong.urdf
```

核心链路：

```text
MuJoCo sensors
-> MJ_Interface::dataBusWrite
-> DataBus
-> Pin_KinDyn::computeJ_dJ / computeDyn
-> StateEst
-> GaitScheduler
-> FootPlacement
-> WBC_priority::computeDdq / computeTau
-> PVT_Ctr::calMotorsPVT
-> MJ_Interface::setMotorsTorque
-> MuJoCo step
```

选择它作为第一目标的原因：

- 控制链完整，但不强依赖 MPC 调参。
- 更适合先验证 MuJoCo、Pinocchio、WBC、PVT 和模型资源是否正常。
- 能输出 `rpy`、base position、base velocity、foot force、motor state 等日志。
- 成功跑通后，后续进入 `walk_mpc_wbc` 的风险会小很多。

### 3.2 第二目标：walk_mpc_wbc

源码入口：

```text
demo/walk_mpc_wbc.cpp
```

模型入口：

```text
models/scene.xml
models/AzureLoong.urdf
```

新增核心链路：

```text
JoyStickInterpreter
-> GaitScheduler / FootPlacement
-> MPC::dataBusRead
-> MPC::cal
-> MPC::dataBusWrite
-> WBC_priority
-> PVT_Ctr
-> MuJoCo torque command
```

关键参数来自源码：

```text
dt = 0.001
dt_200Hz = 0.005
mpc_N = 10
nx = 12
nu = 13
MPC QP solver = qpOASES
WBC QP decision size = 18
WBC QP constraints = 22
friction coefficient passed to WBC = 0.7
```

选择它作为第二目标的原因：

- 它最贴合“人形 MPC + WBC 控制栈”求职叙事。
- 它能展示 MPC 生成接触力 / centroidal state 相关结果，再交给 WBC 求解加速度和关节力矩。
- 它比 `jump_mpc` 更适合作为稳定基线。

### 3.3 后续目标

后续再考虑：

```text
walk_wbc_joystick
walk_mpc_wbc_joystick
jump_mpc
walk_wbc_staircase
```

这些 demo 更有展示效果，但交互、稳定性、录屏和参数风险更高。第一阶段先保证 `walk_wbc` 和 `walk_mpc_wbc` 能被可靠复现。

## 4. 官方构建路径

官方 README 建议环境：

```text
Ubuntu 22.04.4 LTS
g++ 11.4.0
```

官方依赖安装命令：

```bash
sudo apt-get update
sudo apt install git cmake gcc-11 g++-11
sudo apt install libglu1-mesa-dev freeglut3-dev
```

官方构建命令：

```bash
git clone https://github.com/loongOpen/Openloong-dyn-control.git
cd openloong-dyn-control
mkdir build
cd build
cmake ..
make
./walk_mpc_wbc # or ./walk_wbc or ./jump_mpc
```

本仓库复现时建议使用本地已 clone 的源码：

```bash
export REPO_ROOT=/mnt/d/project/Robot_Dynamics_Control
export OPENLOONG_ROOT="$REPO_ROOT/external/open_source_repos/OpenLoong-Dyn-Control"
export RUN_ROOT="$REPO_ROOT/projects/C_openloong_dyn_control_study/outputs/external_reproduction/openloong/20260530_walk_wbc_baseline"

mkdir -p "$RUN_ROOT/logs" "$RUN_ROOT/screenshots" "$RUN_ROOT/videos" "$RUN_ROOT/reports"

cd "$OPENLOONG_ROOT"
git rev-parse --short HEAD | tee "$RUN_ROOT/00_source_commit.txt"
git status --short --branch | tee "$RUN_ROOT/01_source_status.txt"

rm -rf build
mkdir build
cd build
cmake .. 2>&1 | tee "$RUN_ROOT/logs/02_cmake_configure.log"
make -j"$(nproc)" 2>&1 | tee "$RUN_ROOT/logs/03_make_build.log"
```

构建成功后的预期产物：

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

第一轮运行命令：

```bash
cd "$OPENLOONG_ROOT/build"
./walk_wbc 2>&1 | tee "$RUN_ROOT/logs/04_walk_wbc.log"
```

第二轮运行命令：

```bash
cd "$OPENLOONG_ROOT/build"
./walk_mpc_wbc 2>&1 | tee "$RUN_ROOT/logs/05_walk_mpc_wbc.log"
```

注意：demo 会创建 MuJoCo / GLFW 窗口。如果在 WSL2 中运行，需要确认 WSLg 或 X server 可用。

## 5. third_party 与平台风险

该仓库自带主要 third_party：

```text
third_party/boost
third_party/eigen3
third_party/glfw
third_party/jsoncpp
third_party/mujoco
third_party/pinocchio
third_party/qpOASES
third_party/quill
third_party/urdfdom
```

CMake 会按架构链接预编译库：

```text
third_party/mujoco/lin_x64 or lin_arm64
third_party/qpOASES/lin_x64 or lin_arm64
pinocchio_lin_x64 / urdfdom_model_lin_x64 / jsoncpp_lin_x64 / quill_lin_x64 等
```

这说明第一轮复现最好使用 Linux x64，而不是 Windows 原生。当前 Windows 机器虽然有 CMake、Ninja、MSVC 和 MSYS2 gcc，但官方 CMake 直接面向 Linux 预编译库目录，Windows 原生构建风险高，不建议作为正式复现路径。

主要风险：

- OpenGL / GLFW 显示环境不可用，导致窗口创建失败。
- WSL2 若没有 WSLg 或 X server，MuJoCo 可执行文件可能无法显示。
- 运行目录必须是 `build/`，因为 demo 中模型路径写成 `../models/...`。
- demo 会写 `../record/datalog.log`，需要确认 `record/` 可写。
- 第一次复现不要改控制参数，否则无法判断失败来自环境还是算法。
- `walk_mpc_wbc` 比 `walk_wbc` 多 MPC QP 链路，失败定位更复杂。

## 6. 工程模块数据流

第一轮报告必须能讲清楚下面这条链路。

### 6.1 MuJoCo interface

相关文件：

```text
sim_interface/MJ_interface.h
sim_interface/MJ_interface.cpp
sim_interface/GLFW_callbacks.h
sim_interface/GLFW_callbacks.cpp
```

职责：

- 从 MuJoCo 读取 sensor、joint、base 等状态。
- 写入 `DataBus`。
- 把最终 joint torque 写回 MuJoCo actuator。
- 管理 GLFW 窗口、键盘、鼠标和相机视角。

### 6.2 DataBus

相关文件：

```text
common/data_bus.h
```

职责：

- 作为模块间共享状态总线。
- 保存 base pose / velocity / acceleration、foot force、motor current state、desired motor command。
- 保存 Pinocchio dynamics、Jacobian、CoM、foot pose。
- 保存 MPC 输出、WBC 输出、QP 状态、步态状态和足端规划状态。

关键字段示例：

```text
q, dq, ddq
dyn_M, dyn_G, dyn_Non
pCoM_W, fe_l_pos_W, fe_r_pos_W
Xd, X_cur, X_cal, dX_cal, Fr_ff
wbc_delta_q_final, wbc_dq_final, wbc_tauJointRes
motors_pos_des, motors_vel_des, motors_tor_des, motors_tor_out
motionState, legState, leg_contact
qpStatus_MPC, qp_status
```

### 6.3 Pinocchio kinematics / dynamics

相关文件：

```text
algorithm/pino_kin_dyn.h
algorithm/pino_kin_dyn.cpp
```

职责：

- 加载 `models/AzureLoong.urdf`。
- 计算 Jacobian、dJ、动力学矩阵、CoM、足端位姿。
- 为 WBC 提供动力学和约束相关量。

### 6.4 Gait and foot placement

相关文件：

```text
algorithm/gait_scheduler.h
algorithm/gait_scheduler.cpp
algorithm/foot_placement.h
algorithm/foot_placement.cpp
algorithm/joystick_interpreter.h
algorithm/joystick_interpreter.cpp
```

职责：

- 根据期望速度生成 `Walk` / `Stand` / `Walk2Stand` 状态。
- 生成左右脚支撑 / 摆动切换。
- 规划摆动脚落点和 swing trajectory。

### 6.5 MPC

相关文件：

```text
algorithm/mpc.h
algorithm/mpc.cpp
```

职责：

- 使用单刚体模型近似人形质心动力学。
- 以 `nx=12` 的状态和 `nu=13` 的输入构建 horizon QP。
- 使用 qpOASES 求解接触力 / 输入序列。
- 将 `X_cal`、`dX_cal`、`Fr_ff`、QP 状态等写回 `DataBus`。

### 6.6 WBC

相关文件：

```text
algorithm/wbc_priority.h
algorithm/wbc_priority.cpp
algorithm/priority_tasks.h
algorithm/priority_tasks.cpp
```

职责：

- 读取动力学、Jacobian、足端约束、MPC feedforward force 和期望状态。
- 求解 WBC QP，决策变量规模在 demo 中配置为 18，约束规模为 22。
- 计算 `delta_q`、`dq`、`ddq` 和关节力矩 `tauJointRes`。

### 6.7 PVT joint control

相关文件：

```text
common/PVT_ctrl.h
common/PVT_ctrl.cpp
common/joint_ctrl_config.json
```

职责：

- 根据 WBC 生成的期望位置、速度、力矩计算最终 motor torque。
- 读取关节 kp、kd、limit、gear、filter 等参数。
- 输出 `motors_tor_out`，再由 `MJ_Interface` 写回 MuJoCo。

## 7. 最小复现证据清单

第一轮正式复现的证据目录建议为：

```text
projects/C_openloong_dyn_control_study/outputs/external_reproduction/openloong/20260530_walk_wbc_baseline/
```

建议保存：

```text
00_source_commit.txt
01_source_status.txt
logs/02_cmake_configure.log
logs/03_make_build.log
logs/04_walk_wbc.log
logs/05_walk_mpc_wbc.log
record/datalog_walk_wbc.log
record/datalog_walk_mpc_wbc.log
screenshots/walk_wbc_start.png
screenshots/walk_wbc_stable_walk.png
screenshots/walk_mpc_wbc_stable_walk.png
videos/walk_wbc.mp4
videos/walk_mpc_wbc.mp4
reports/REPRODUCTION_REPORT.md
reports/ARCHITECTURE_TRACE.md
reports/RESUME_PROJECT_DESCRIPTION.md
```

第一轮通过标准：

```text
External source unchanged
CMake configure succeeds
make succeeds
walk_wbc launches and runs beyond startSteppingTime=3s
walk_wbc reaches walking phase around startWalkingTime=5s
record/datalog.log contains base pose / velocity / foot force / motor state
at least one screenshot or short video is captured
architecture trace explains DataBus-centered control pipeline
```

第二轮通过标准：

```text
walk_mpc_wbc launches
MPC loop runs at dt_200Hz=0.005 scheduling interval
MPC QP status and WBC QP status are recorded or observed
stable walking screenshot / video is captured
comparison notes explain what MPC adds over walk_wbc
```

## 8. 输入、输出、参数与数学逻辑变化

第一阶段不改变官方数学逻辑，不调整 controller 参数，不修改 demo 源码。

输入：

```text
Official source at 4dd7a7e4
models/scene_board.xml for walk_wbc
models/scene.xml for walk_mpc_wbc
models/AzureLoong.urdf
common/joint_ctrl_config.json
official CMake build path
```

输出：

```text
C++ binaries
MuJoCo window / video / screenshot
record/datalog.log
console logs
build logs
reproduction report
architecture trace
```

数学逻辑：

```text
No change in official MPC, WBC, gait scheduler, foot placement, PVT, Pinocchio dynamics, or MuJoCo model logic.
```

参数：

```text
No control parameter changes in first reproduction pass.
Record observed parameters only: dt, dt_200Hz, mpc_N, nx, nu, WBC QP size, startSteppingTime, startWalkingTime, desired velocity.
```

风险：

```text
If build fails, inspect Linux library path and compiler first.
If window fails, inspect OpenGL / GLFW / WSLg first.
If simulation launches but falls, do not tune parameters immediately; first preserve logs and compare with official environment.
If walk_mpc_wbc fails but walk_wbc works, isolate MPC QP status, DataBus fields Xd/X_cur/Fr_ff, and gait state.
```

## 9. 求职交付物规划

这次复现最终应形成一套可以放进简历和面试讲解的材料，而不是只停留在“能运行”。

建议交付物：

1. `REPRODUCTION_REPORT.md`
   - 环境、commit、构建命令、运行命令、成功截图、失败与修复记录。

2. `ARCHITECTURE_TRACE.md`
   - 按主循环解释每个模块的输入输出。
   - 画出 `MJ_Interface -> DataBus -> Pin_KinDyn -> MPC/WBC -> PVT -> MuJoCo` 数据流。

3. `CONTROL_CHAIN_NOTES.md`
   - 解释 WBC 和 MPC 的职责边界。
   - 解释 `walk_wbc` 与 `walk_mpc_wbc` 的区别。

4. `videos/`
   - 30 秒以内稳定行走视频。
   - 可选：WBC-only 和 MPC+WBC 对比视频。

5. `RESUME_PROJECT_DESCRIPTION.md`
   - 一段简历 bullet。
   - 一段面试讲解稿。
   - 三个常见追问的回答：DataBus 为什么存在、MPC 输出是什么、WBC QP 决策变量是什么。

建议简历 bullet 初稿：

```text
Reproduced OpenLoong-Dyn-Control, a C++ humanoid locomotion stack on MuJoCo, built and ran official WBC / MPC-WBC walking demos, traced the DataBus-centered pipeline from MuJoCo sensors through Pinocchio dynamics, gait scheduling, MPC contact-force optimization, WBC-QP, and PVT torque control, and documented build logs, runtime evidence, and module-level inputs/outputs.
```

## 10. 下一步执行建议

下一步不要继续写学习骨架，直接进入工程复现准备：

1. 准备 Ubuntu 22.04 / WSL2 Ubuntu 22.04。
2. 安装官方 README 中列出的系统依赖。
3. 用本审计文档的命令构建官方源码。
4. 先运行 `walk_wbc`，保存截图、视频和 `record/datalog.log`。
5. 再运行 `walk_mpc_wbc`，保存同样证据。
6. 基于实际运行结果写 `REPRODUCTION_REPORT.md`。
7. 再决定是否需要为录屏、日志复制或报告生成增加本仓库侧辅助脚本。
