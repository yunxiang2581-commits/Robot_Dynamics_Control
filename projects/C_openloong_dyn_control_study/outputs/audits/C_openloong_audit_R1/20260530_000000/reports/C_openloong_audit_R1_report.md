# C-OPENLOONG-AUDIT-R1 Report

## 1. 审查目标与边界

本次任务只做 OpenLoong-Dyn-Control 开源项目审查、代码阅读、架构梳理和学习地图生成。

本次不复现、不开发新功能、不修改源码、不重构目录、不自动修 bug、不运行长时间 MuJoCo、不生成 MP4、不执行 `git add` / `git commit` / `git push`。

允许写入范围：

```text
projects/C_openloong_dyn_control_study/outputs/audits/C_openloong_audit_R1/20260530_000000/
```

## 2. 仓库状态

本次开始时运行了：

```bash
git status --short
git diff --stat
```

结果摘要：

- tracked dirty tree: FOUND。仓库已有大量 tracked 修改，主要集中在 `docs/00_project_management/`、`docs/archive/`、`projects/A_self_baseline/`、`projects/B_mujoco_mpc_study/`，以及 Project C 的 `README.md` / `docs/C_CURRENT_STATUS.md`。
- untracked files: FOUND。包含大量新文档、Project A/B 新文件、`tests/`、`tools/`，以及 Project C 的 `docs/09_engineering_reproduction_audit.md`、`docs/10_official_reproduction_runbook.md`。
- Project C 相关 dirty 文件：
  - `M projects/C_openloong_dyn_control_study/README.md`
  - `M projects/C_openloong_dyn_control_study/docs/C_CURRENT_STATUS.md`
  - `?? projects/C_openloong_dyn_control_study/docs/09_engineering_reproduction_audit.md`
  - `?? projects/C_openloong_dyn_control_study/docs/10_official_reproduction_runbook.md`
- 本次是否修改源码：没有。
- 是否执行 `git add` / `git commit` / `git push`：没有。

`git diff --stat` 显示全仓库已有 67 个 tracked 文件变化，合计 `7378 insertions(+), 3383 deletions(-)`。这些不是本次审查产生的源码修改。

## 3. Project C 目录结构

Project C 当前是一个学习与审查容器，不是已实现的仿真工程。目录结构摘要如下：

| 目录 | 作用 | 当前状态 | 备注 |
|---|---|---|---|
| `README.md` | Project C 总入口 | FOUND | 描述 OpenLoong 学习区、simulation-only 目标和外部源码路径。 |
| `docs/` | 项目说明、路线、审计文档 | FOUND | 已有 00-10 系列文档和当前状态文档。 |
| `notes/` | 文献与链接 | FOUND | 目前只有 `literature_and_links.md`。 |
| `simulator/` | 未来自建简化仿真器骨架 | FOUND | 子目录只有 `.gitkeep` 和 README，无可运行代码。 |
| `simulator/controllers/` | 未来控制器模块 | FOUND | 仅 `.gitkeep`。 |
| `simulator/envs/` | 未来仿真环境模块 | FOUND | 仅 `.gitkeep`。 |
| `simulator/planners/` | 未来 gait/contact/CoM planner | FOUND | 仅 `.gitkeep`。 |
| `simulator/scripts/` | 未来脚本入口 | FOUND | 仅 `.gitkeep`。 |
| `simulator/utils/` | 未来通用工具 | FOUND | 仅 `.gitkeep`。 |
| `outputs/figures` | 图像输出 | FOUND | 仅 `.gitkeep`。 |
| `outputs/logs` | 日志输出 | FOUND | 仅 `.gitkeep`。 |
| `outputs/metrics` | 指标输出 | FOUND | 仅 `.gitkeep`。 |
| `outputs/videos` | 视频输出 | FOUND | 仅 `.gitkeep`。 |
| `configs/` | 配置文件 | NOT FOUND | Project C 本体还没有配置文件。 |
| `scripts/` | 独立脚本目录 | NOT FOUND | 只有 `simulator/scripts/` 骨架。 |
| `src/` | 源码包 | NOT FOUND | Project C 本体无源码实现。 |
| `tests/` | 测试 | NOT FOUND | 本体无测试。 |
| `assets/` | Project C 内部资源 | NOT FOUND | 机器人资源在 external 官方仓库内。 |
| `external/` | Project C 内部外部引用 | NOT FOUND | 当前采用根目录 `external/open_source_repos/...`。 |
| `examples/` | 示例 | NOT FOUND | 暂无。 |

结论：Project C documentation is present but implementation is incomplete. Project C 本体当前没有 Python/C++ 控制实现、没有模型资源、没有测试、没有真实可运行入口。

## 4. 文档与 README 审查

Project C 文档清单：

- `README.md`: 项目定位、边界、目标 demo、外部源码路径。
- `docs/00_project_overview.md`: OpenLoong 概览和 Project C 最终交付定位。
- `docs/01_algorithm_map.md`: command/gait/MPC/WBC/PVT/MuJoCo 控制链概念地图。
- `docs/02_source_reading_map.md`: 建议阅读模块列表。
- `docs/03_simulation_only_reproduction_plan.md`: C01-C03 简化仿真 demo 规划。
- `docs/04_relation_to_A_self_baseline.md`: 与 A 项目基础运动学/IK/QP 的关系。
- `docs/05_dependency_and_risk.md`: 环境依赖和风险。
- `docs/06_next_questions.md`: 后续源码阅读问题。
- `docs/07_simulator_and_video_demo_plan.md`: 自建仿真器和视频计划。
- `docs/08_source_repo_detailed_introduction.md`: 官方源码详细介绍。
- `docs/09_engineering_reproduction_audit.md`: 从学习规划切换到官方工程复现的审计。
- `docs/10_official_reproduction_runbook.md`: 官方构建/运行 runbook。
- `docs/C_CURRENT_STATUS.md`: 当前状态。
- `simulator/README.md`: simulator 目录用途和未来命令规划。
- `notes/literature_and_links.md`: 上游链接与关键词。

文档提取结果：

- 项目目标：学习 OpenLoong-Dyn-Control 的 humanoid MPC + WBC + PVT + MuJoCo 闭环，并后续抽象为 simulation-only 简化 demo。
- 当前状态：Project C 骨架已完成；官方外部源码已 clone；尚未实现 C01；尚未运行仿真；尚未导出视频或 metrics。
- 运行命令：Project C 文档里规划了 `python simulator/run_demo.py --demo ... --export-video`，但该脚本尚不存在；官方复现命令在 09/10 文档中，指向外部 C++ 仓库。
- 模块解释：已有 DataBus、Pin_KinDyn、MPC、WBC、PVT、GaitScheduler、FootPlacement、MJ_Interface 等模块级说明。
- 已知限制：当前环境缺 `cmake`；当前系统 Ubuntu 25.10 / g++ 15.2.0，不是官方推荐 Ubuntu 22.04 / g++ 11；外部源码存在大量行尾 dirty 状态。
- 下一步路线：不要立即跑完整 demo；先做架构地图和模型/入口审查，再进入最小 smoke。
- 用户之前已经写过 C 项目 roadmap/status：是，包含 03/07/09/10 和 `C_CURRENT_STATUS.md`。

## 5. 开源项目来源与本地封装关系

原始开源项目已经 clone 到本地：

```text
external/open_source_repos/OpenLoong-Dyn-Control/
```

上游仓库：

```text
https://github.com/loongOpen/OpenLoong-Dyn-Control
```

当前本地 commit：

```text
4dd7a7e4
```

官方项目许可证文件存在：

```text
external/open_source_repos/OpenLoong-Dyn-Control/license
```

官方项目还包含第三方依赖许可证，例如：

- `third_party/boost/LICENSE_1_0.txt`
- `third_party/glfw/LICENSE.md`
- `third_party/jsoncpp/LICENSE.txt`
- `third_party/mujoco/LICENSE.txt`
- `third_party/pinocchio/LICENSE.txt`
- `third_party/qpOASES/LICENSE.txt`
- `third_party/quill/LICENSE.txt`
- `third_party/urdfdom/LICENSE.txt`

Project C 当前不是直接复制官方源码的实现目录，而是学习封装、审查文档和未来自建 simplified simulator 的容器。真实官方代码、模型、CMake 和 third_party 都在根目录 `external/open_source_repos/OpenLoong-Dyn-Control/`。

不应该修改的路径：

- `external/open_source_repos/OpenLoong-Dyn-Control/`
- `shared/robot_assets/vendor/`
- `projects/A_self_baseline/`
- `projects/B_mujoco_mpc_study/`
- `projects/D_*` 或其他项目目录

如果要读原始项目，建议从以下文件开始：

1. `external/open_source_repos/OpenLoong-Dyn-Control/README.md`
2. `external/open_source_repos/OpenLoong-Dyn-Control/README-zh.md`
3. `external/open_source_repos/OpenLoong-Dyn-Control/Tutorial.md`
4. `external/open_source_repos/OpenLoong-Dyn-Control/CMakeLists.txt`
5. `external/open_source_repos/OpenLoong-Dyn-Control/demo/walk_wbc.cpp`
6. `external/open_source_repos/OpenLoong-Dyn-Control/demo/walk_mpc_wbc.cpp`
7. `external/open_source_repos/OpenLoong-Dyn-Control/common/data_bus.h`

## 6. 可运行入口审查

Project C 本体没有可运行 Python/C++ 文件。文档中的 `python simulator/run_demo.py ...` 是后续接口规划，不是现有入口。

官方 OpenLoong CMake 入口如下：

| 入口文件 | 类型 | 参数 | 是否像 smoke run | 是否可能打开 viewer | 风险 |
|---|---|---|---|---|---|
| `demo/walk_wbc.cpp` | C++ MuJoCo WBC demo | 无 CLI 参数 | 否，完整 30s demo | 是，创建 GLFW 窗口 | 需要 CMake 构建、OpenGL/GLFW、从 `build/` 运行、会写 `record/datalog.log`。 |
| `demo/walk_mpc_wbc.cpp` | C++ MuJoCo MPC+WBC demo | 无 CLI 参数 | 否，完整 30s demo | 是，创建 GLFW 窗口 | 比 `walk_wbc` 多 MPC QP 链路，失败定位更复杂。 |
| `demo/jump_mpc.cpp` | C++ MuJoCo jump demo | 无 CLI 参数 | 否 | 是 | 动作更激进，第一轮不建议。 |
| `demo/float_control.cpp` | C++ MuJoCo float/control demo | 无 CLI 参数 | 部分像模型/control 检查 | 是 | 仍需 viewer 和构建。 |
| `demo/walk_wbc_speed_test.cpp` | C++ WBC speed test | 无 CLI 参数 | 是，偏算法速度测试 | 否，未包含 MuJoCo/GLFW viewer | 仍需 CMake 构建和相对路径 `../models`、`../common`；循环 10000 次，会写 `record/datalog.log`。 |
| `demo/walk_wbc_joystick.cpp` | C++ interactive demo | 键盘/窗口交互 | 否 | 是 | 交互与窗口风险更高。 |
| `demo/walk_mpc_wbc_joystick.cpp` | C++ interactive MPC+WBC demo | 键盘/窗口交互 | 否 | 是 | 交互、MPC 和 viewer 风险叠加。 |
| `demo/walk_wbc_staircase.cpp` | C++ staircase demo | 无 CLI 参数 | 否 | 是 | 地形/障碍物场景更复杂。 |

最小安全入口判断：

- Project C 本体：没有真实入口。
- 官方源码构建后：`wbc_speed_test` 最像非 viewer smoke，但仍不是 Project C 本体入口。
- 第一轮工程复现入口：`walk_wbc`，因为它验证完整 WBC/PVT/MuJoCo 闭环，但不引入 MPC 调参复杂度。
- 第一轮学习阅读入口：`walk_wbc.cpp` + `data_bus.h`。

## 7. 配置与资源审查

Project C 本体没有 `.yaml/.json/.xml/.urdf/.mjcf/.csv` 配置或资源文件。

官方 OpenLoong 配置与资源：

| 文件 | 类型 | 作用 | 关键参数 | 是否存在 | 风险 |
|---|---|---|---|---|---|
| `models/AzureLoong.urdf` | URDF | Pinocchio 动力学/运动学模型 | 31 个关节，floating-base 由 Pinocchio `JointModelFreeFlyer` 加入 | FOUND | URDF 与 MuJoCo XML joint 名称必须一致。 |
| `models/AzureLoong_simplified.urdf` | URDF | 简化模型 | 简化 link/joint 结构 | FOUND | 不一定是官方 walking demo 使用模型。 |
| `models/AzureLoong.xml` | MuJoCo XML | 主 MuJoCo 人形模型 | `timestep=0.001`、`freejoint`、motor、sensor | FOUND | actuator gear/ctrlrange 与 PVT 输出单位要对应。 |
| `models/AzureLoong_float.xml` | MuJoCo XML | floating model 变体 | `timestep=0.001`、sensor、motor | FOUND | 与 demo 选择的 scene 关系需看 include。 |
| `models/scene.xml` | MuJoCo scene | `walk_mpc_wbc` 场景 | include `AzureLoong.xml` | FOUND | 必须从 `build/` 运行才能解析相对路径。 |
| `models/scene_board.xml` | MuJoCo scene | `walk_wbc` 场景 | include `AzureLoong.xml` | FOUND | 第一轮 WBC demo 入口。 |
| `models/scene_float.xml` | MuJoCo scene | floating scene | include `AzureLoong_float.xml` | FOUND | 不是第一轮入口。 |
| `models/scene_staircase.xml` | MuJoCo scene | staircase demo | include `AzureLoong.xml` | FOUND | 地形更复杂。 |
| `models/meshes/*.STL` | Mesh | 视觉/碰撞网格 | base、arm、waist、leg、ankle 等 | FOUND | meshdir 必须正确。 |
| `common/joint_ctrl_config.json` | JSON | PVT/PD joint config | `kp`、`kd`、`maxTorque`、`maxSpeed`、`minPos/maxPos` | FOUND | joint name 必须与 XML/URDF/PVT 列表一致。 |

重点判断：

- robot model 不在 Project C 内，而在官方 external 仓库内。
- 当前不依赖 `shared/robot_assets`。
- 当前依赖 `external/open_source_repos/OpenLoong-Dyn-Control`。
- MuJoCo XML 包含 actuator/sensor，scene 通过 include 引入主模型。
- `MJ_Interface.h`、`Pin_KinDyn.h`、`PVT_ctrl.h` 都写有同一套 31 个关节名列表，名称一致性是关键风险。
- MuJoCo timestep 在 `models/AzureLoong.xml` / `AzureLoong_float.xml` 的 `<option timestep="0.001">`。
- `walk_mpc_wbc.cpp` 额外定义 `dt=0.001`、`dt_200Hz=0.005`。

## 8. 核心模块架构

### MuJoCo

- 文件：`sim_interface/MJ_interface.*`、`sim_interface/GLFW_callbacks.*`、`models/*.xml`、`demo/*.cpp`
- 核心类/函数：`MJ_Interface::updateSensorValues`、`MJ_Interface::dataBusWrite`、`MJ_Interface::setMotorsTorque`、`UIctr::createWindow`
- 输入：`mjModel`、`mjData`、sensor/joint/body names、最终 torque。
- 输出：base pose/velocity、rpy、foot force、motor state、MuJoCo actuator torque。
- 状态：官方完整实现；Project C 本体没有封装实现。

### Robot model / assets

- 文件：`models/AzureLoong.urdf`、`models/AzureLoong.xml`、`models/scene*.xml`、`models/meshes/*.STL`
- 输入：URDF/XML/mesh 文件。
- 输出：Pinocchio model、MuJoCo model。
- 状态：官方资源存在；Project C 本体没有资源副本。

### State representation / DataBus

- 文件：`common/data_bus.h`
- 核心字段：`q/dq/ddq`、`dyn_M/dyn_G/dyn_Non`、`pCoM_W`、`fe_l_pos_W`、`Xd/X_cur/X_cal/dX_cal/Fr_ff`、`wbc_delta_q_final/wbc_dq_final/wbc_tauJointRes`、`motors_pos_des/motors_vel_des/motors_tor_out`、`motionState/legState/leg_contact`。
- 输入：MuJoCo sensor、Pinocchio dynamics、planner/MPC/WBC/PVT 输出。
- 输出：模块间共享状态。
- 状态：官方完整实现；是最关键阅读文件。

### Kinematics / dynamics

- 文件：`algorithm/pino_kin_dyn.*`
- 核心类/函数：`Pin_KinDyn`、`computeJ_dJ`、`computeDyn`、`computeInK_Leg`、`computeInK_Hand`、`integrateDIY`
- 输入：URDF、`DataBus.q/dq`、目标足/手位姿。
- 输出：Jacobian、dJ、CoM、mass matrix、gravity/nonlinear terms、IK result。
- 状态：官方完整实现。使用 Pinocchio 的 `JointModelFreeFlyer`、`forwardKinematics`、`computeJointJacobiansTimeVariation`、`crba`、`computeGeneralizedGravity`、`integrate` 等。

### Planner / trajectory / command

- 文件：`algorithm/joystick_interpreter.*`、`algorithm/gait_scheduler.*`、`algorithm/foot_placement.*`
- 核心类/函数：`JoyStickInterpreter`、`GaitScheduler::step`、`FootPlacement::getSwingPos`、`FootPlacement::Trajectory`
- 输入：desired velocity、base pose、leg contact、foot pose。
- 输出：`js_pos_des/js_vel_des`、`motionState`、`legState`、swing foot target。
- 状态：官方实现存在；Project C 暂无简化版本。

### MPC

- 文件：`algorithm/mpc.*`
- 核心类/函数：`MPC::dataBusRead`、`MPC::cal`、`MPC::dataBusWrite`、`MPC::set_weight`
- 输入：`X_cur`、desired state horizon `Xd`、contact schedule/leg state、CoM/foot state。
- 输出：`X_cal`、`dX_cal`、`Fr_ff`、QP status。
- 关键维度：`mpc_N=10`、`nx=12`、`nu=13`、`dt_200Hz=0.005` 在 demo 中调度。
- 状态：官方实现存在，使用 qpOASES。

### WBC / QP

- 文件：`algorithm/wbc_priority.*`、`algorithm/priority_tasks.*`
- 核心类/函数：`WBC_priority::computeDdq`、`computeTau`、`PriorityTasks`
- 输入：Pinocchio dynamics/Jacobian、WBC desired `des_ddq/des_dq/des_delta_q`、MPC feedforward contact force `Fr_ff`、foot contact state。
- 输出：`wbc_delta_q_final`、`wbc_dq_final`、`wbc_ddq_final`、`wbc_tauJointRes`、`wbc_FrRes`、QP status。
- 关键维度：demo 初始化 `WBC_priority(model_nv, 18, 22, 0.7, timestep)`；QP decision size 18，constraints 22。
- 状态：官方实现存在，使用 qpOASES。

### PVT / low-level tracking

- 文件：`common/PVT_ctrl.*`、`common/joint_ctrl_config.json`
- 核心类/函数：`PVT_Ctr::calMotorsPVT`、`setJointPD`、`dataBusRead`、`dataBusWrite`
- 输入：desired joint position、velocity、torque；current joint state；joint config。
- 输出：`motors_tor_out`。
- 状态：官方实现存在。注意 README 提到 damping 可能偏大。

### Estimator

- 文件：`algorithm/StateEst.*`、`algorithm/Eul_W_filter.*`
- 核心类/函数：`StateEst::init`、`set/update/get`、`setF/updateF/getF`
- 输入：IMU/base/joint/foot/contact 数据。
- 输出：estimated base/CoM/foot/contact force fields。
- 状态：官方实现存在；`walk_wbc.cpp` 使用，`walk_mpc_wbc.cpp` 当前未显式使用 `StateEst`。

### Logging / visualization

- 文件：`common/data_logger.*`、`record/*.txt`、`sim_interface/GLFW_callbacks.*`
- 核心类/函数：`DataLogger::addIterm`、`recItermData`、`finishLine`、`UIctr`
- 输入：DataBus 中被记录变量。
- 输出：`record/datalog.log` 和 matlab 读取辅助信息。
- 状态：官方实现存在；Project C 输出目录尚未接管官方 record 文件。

### Tests

- Project C 本体：NOT FOUND。
- 官方 OpenLoong：未发现标准 tests 目录；有 `wbc_speed_test` 可作为构建后算法性能 smoke 候选。

## 9. 控制链路图

Project C 文档期望链路：

```text
command / target
    ↓
planner / trajectory generator
    ↓
MPC
    ↓
WBC / QP
    ↓
PVT / joint-level tracking
    ↓
MuJoCo actuator / robot
    ↓
state feedback / logger
```

官方 `walk_wbc` 实际链路：

```text
MuJoCo mj_step
    ↓
MJ_Interface::updateSensorValues / dataBusWrite
    ↓
StateEst::set/update/get
    ↓
Pin_KinDyn::computeJ_dJ / computeDyn / dataBusWrite
    ↓
JoyStickInterpreter
    ↓
GaitScheduler
    ↓
FootPlacement
    ↓
manual WBC desired state + fixed Fr_ff
    ↓
WBC_priority::computeDdq / computeTau
    ↓
PVT_Ctr::calMotorsPVT
    ↓
MJ_Interface::setMotorsTorque
    ↓
DataLogger -> record/datalog.log
```

官方 `walk_mpc_wbc` 实际链路：

```text
MuJoCo mj_step
    ↓
MJ_Interface::updateSensorValues / dataBusWrite
    ↓
Pin_KinDyn::computeJ_dJ / computeDyn / dataBusWrite
    ↓
JoyStickInterpreter
    ↓
GaitScheduler
    ↓
FootPlacement
    ↓
MPC::dataBusRead / cal / dataBusWrite   (200 Hz, dt_200Hz=0.005)
    ↓
WBC_priority::computeDdq / computeTau
    ↓
PVT_Ctr::calMotorsPVT
    ↓
MJ_Interface::setMotorsTorque
    ↓
DataLogger -> record/datalog.log
```

| 节点 | 文件 | 输入 | 输出 | 当前状态 | 备注 |
|---|---|---|---|---|---|
| command / target | `joystick_interpreter.*` | desired vx/wz/time | desired base pos/vel/yaw | 官方实现 | Project C 未实现。 |
| planner / trajectory | `gait_scheduler.*`, `foot_placement.*` | DataBus state/contact | leg state, swing foot target | 官方实现 | 支撑/摆动腿切换和落脚点。 |
| MPC | `mpc.*` | current state, desired horizon, contact schedule | `Fr_ff`, `X_cal`, `dX_cal`, QP status | 官方实现 | 仅 `walk_mpc_wbc` 主链使用。 |
| WBC / QP | `wbc_priority.*`, `priority_tasks.*` | dynamics, Jacobian, desired state, `Fr_ff` | ddq/dq/delta_q/tau/contact force | 官方实现 | QP size 18, constraints 22。 |
| PVT | `PVT_ctrl.*` | desired P/V/T + current joint state | final joint torque | 官方实现 | 读取 JSON 参数。 |
| MuJoCo actuator | `MJ_interface.*`, `models/*.xml` | `motors_tor_out` | MuJoCo control | 官方实现 | 需要 viewer/OpenGL。 |
| feedback/logger | `MJ_interface.*`, `data_logger.*` | sensors, DataBus fields | log rows | 官方实现 | 写 `record/datalog.log`。 |

关键判断：

- MPC 是否真的实现：是，在官方 `algorithm/mpc.*`，使用 qpOASES。
- WBC 是否真的实现：是，在官方 `algorithm/wbc_priority.*` 和 `priority_tasks.*`。
- PVT 是否真的实现：是，在官方 `common/PVT_ctrl.*`。
- MuJoCo 闭环是否真的实现：是，在官方 demo + `MJ_interface.*`。
- Project C 本体是否实现这些模块：否，当前只是文档与骨架。

## 10. 依赖与环境风险

| 依赖 | 用途 | 出现位置 | 是否当前环境可能已有 | 风险 |
|---|---|---|---|---|
| CMake | 官方 C++ 构建 | `CMakeLists.txt` | 未安装，`cmake: command not found` | 当前无法构建。 |
| g++ 11 | 官方推荐编译器 | README | 当前是 g++ 15.2.0 | 版本偏离官方环境。 |
| make | 构建 | README/CMake | 已有 GNU Make 4.4.1 | 不是阻塞。 |
| MuJoCo C library | 仿真和 model loading | `third_party/mujoco`, demo include | vendored | OpenGL/GLFW 仍需系统支持。 |
| GLFW | viewer/window | `third_party/glfw`, `GLFW_callbacks.*` | vendored static lib | 无显示环境会失败。 |
| OpenGL system libs | GLFW/MuJoCo 窗口 | README apt deps | 未验证安装 | WSL/X11/WSLg 风险。 |
| Pinocchio | kinematics/dynamics | `third_party/pinocchio`, `pino_kin_dyn.*` | vendored static lib | 与 compiler/libstdc++ ABI 可能相关。 |
| Eigen | linear algebra | `third_party/eigen3`, many includes | vendored header | 低风险。 |
| qpOASES | MPC/WBC QP solver | `third_party/qpOASES`, `mpc.*`, `wbc_priority.*` | vendored | QP status/数值稳定性风险。 |
| JsonCpp | joint config parsing | `third_party/jsoncpp`, `PVT_ctrl.*` | vendored | JSON key 与 joint name 必须匹配。 |
| Quill | data logging | `third_party/quill`, `data_logger.*` | vendored | 输出路径和权限风险。 |
| urdfdom/tinyxml/console_bridge | URDF parsing | `third_party/urdfdom`, CMake | vendored | URDF 路径/mesh package URI 风险。 |
| Python/pytest | Project C 审查环境 | 本次安全检查 | Python 3.13.12 / pytest 9.0.3 | Project C 当前无 Python 文件。 |

ROS / ROS2 / `ament`：Project C 和官方 OpenLoong 主链未发现 ROS/ROS2 build 依赖。不要按 ROS 工程方式编译。

环境事实：

```text
OS: Ubuntu 25.10
Python: 3.13.12
pytest: 9.0.3
cmake: command not found
g++: 15.2.0
make: 4.4.1
```

官方推荐：Ubuntu 22.04.4 LTS + g++ 11.4.0。

## 11. 测试与可编译性结果

Python 文件编译：

```bash
find projects/C_openloong_dyn_control_study -name "*.py" | sort
```

结果：Project C 本体没有 Python 文件，因此 `python -m py_compile` 无可执行对象，标记 `NOT FOUND / NOT APPLICABLE`。

测试目录：

```bash
find projects/C_openloong_dyn_control_study -maxdepth 5 -type f | grep -Ei "test_|_test|tests/" | sort
```

结果：Project C 本体没有 tests，标记 `NOT FOUND`。因此没有运行 `pytest -q projects/C_openloong_dyn_control_study/tests`。

C++ 可编译性：本次按边界要求没有运行 CMake build。只做只读审查。当前环境缺 `cmake`，即使要构建也会先被系统依赖阻塞。

## 12. 当前可复现状态

当前不能直接说 Project C 已可复现。

能复现到哪：

- 文档层：Project C 已有清晰的 OpenLoong 学习目标、工程复现审计和 runbook。
- 源码层：官方 OpenLoong 源码和模型已存在于 external。
- 执行层：尚未构建，尚未运行 `walk_wbc`、`walk_mpc_wbc` 或 `wbc_speed_test`。

最大阻塞：

1. 当前环境缺 `cmake`。
2. 当前编译器是 g++ 15.2.0，不是官方推荐 g++ 11.4.0。
3. 官方 demo 多数会打开 GLFW/MuJoCo viewer，需要 OpenGL/显示环境。
4. 外部源码工作树显示大量 `M`，抽样看主要是 CRLF/LF 行尾变化，但作为复现证据需记录。

## 13. 学习阅读顺序

### 第 1 层：项目总览

- 目标：知道 OpenLoong 是什么、Project C 为什么学它。
- 阅读文件：`README.md`、`README-zh.md`、Project C `docs/00_project_overview.md`。
- 关键问题：官方项目支持哪些 demo？Project C 是直接复现还是学习封装？
- 预期产出笔记：项目一句话定义、模块列表、边界声明。

### 第 2 层：运行入口

- 目标：知道 main loop 如何串起模块。
- 阅读文件：`demo/walk_wbc.cpp`、`demo/walk_mpc_wbc.cpp`、`demo/walk_wbc_speed_test.cpp`。
- 关键问题：每个类何时初始化？DataBus 何时读写？MPC 在哪个频率运行？
- 预期产出笔记：每个 demo 的流程图和风险表。

### 第 3 层：模型与配置

- 目标：确认 URDF/XML/joint/motor/sensor 对应关系。
- 阅读文件：`models/AzureLoong.urdf`、`models/AzureLoong.xml`、`models/scene*.xml`、`common/joint_ctrl_config.json`、`MJ_interface.h`。
- 关键问题：31 个 joint 名是否一致？freejoint 如何进入 `nq/nv`？sensor 名称如何查找？
- 预期产出笔记：joint/motor/sensor mapping 表。

### 第 4 层：状态总线

- 目标：理解模块间数据生命周期。
- 阅读文件：`common/data_bus.h`。
- 关键问题：哪些字段由 MuJoCo 写？哪些由 Pinocchio 写？哪些由 MPC/WBC/PVT 写？
- 预期产出笔记：DataBus 字段分组表。

### 第 5 层：运动学/动力学

- 目标：理解 Pinocchio 如何为 WBC/MPC 提供模型量。
- 阅读文件：`algorithm/pino_kin_dyn.h`、`algorithm/pino_kin_dyn.cpp`。
- 关键问题：floating-base `nq/nv` 怎么处理？Jacobian 和 dynamics 矩阵写回哪些字段？
- 预期产出笔记：Pinocchio API 对照表。

### 第 6 层：控制链路

- 目标：理解 planner -> MPC -> WBC -> PVT。
- 阅读文件：`algorithm/gait_scheduler.*`、`algorithm/foot_placement.*`、`algorithm/mpc.*`、`algorithm/wbc_priority.*`、`algorithm/priority_tasks.*`、`common/PVT_ctrl.*`。
- 关键问题：MPC 输出什么？WBC QP 决策变量是什么？PVT 如何叠加 P/V/T？
- 预期产出笔记：MPC/WBC/PVT 输入输出表。

### 第 7 层：仿真闭环与日志

- 目标：知道 torque 如何回到 MuJoCo、日志如何保存。
- 阅读文件：`sim_interface/MJ_interface.*`、`sim_interface/GLFW_callbacks.*`、`common/data_logger.*`。
- 关键问题：viewer 风险在哪里？`record/datalog.log` 记录哪些字段？
- 预期产出笔记：运行证据清单。

## 14. 后续复现路线

| 步骤 | 目标 | 输入 | 输出 | 验证 | 风险 |
|---|---|---|---|---|---|
| C00 | OpenLoong 源码与本地封装审查报告 | Project C docs + external official source | audit report + learning map | 报告能回答入口/模块/资源/依赖 | 误把 Project C 骨架当实现。 |
| C01 | robot model / MuJoCo XML loading smoke | `models/scene_board.xml`, `models/AzureLoong.urdf` | model loading log | 不开长仿真，只验证路径和 model dimensions | 当前无 CMake；viewer/OpenGL 风险。 |
| C02 | state read / joint mapping / actuator mapping audit | XML/URDF/joint JSON/MJ_Interface | mapping table | 31 joint names 对齐 | 手工索引和 floating-base offset 易错。 |
| C03 | PVT / low-level joint tracking smoke | `PVT_ctrl.*`, `joint_ctrl_config.json` | torque command log | 输入 P/V/T 后输出 torque 在 limit 内 | 单位、gear、limit 不一致。 |
| C04 | WBC / QP controller skeleton or replay | `wbc_speed_test`, recorded fake state | WBC QP status + tau | 非 viewer smoke 可运行 | QP 数值问题、外部构建依赖。 |
| C05 | MPC command / trajectory generator skeleton | `mpc.*`, `gait_scheduler.*` | MPC `Fr_ff/X_cal` log | QP status 正常 | contact schedule 和状态维度难。 |
| C06 | closed-loop short demo | `walk_wbc` | 5-10s log/screenshot | 能运行到 startSteppingTime/startWalkingTime | OpenGL/稳定性风险。 |
| C07 | metrics + figures | `record/datalog.log` | CSV/figures | base/rpy/foot force 曲线可读 | 日志字段解析。 |
| C08 | video demo | viewer/recording tool | short MP4 | 30s 内稳定片段 | 文件大、显示环境、录屏风险。 |
| C09 | final reproduction report | logs/screenshots/video/metrics | final report + resume notes | 可复现实验命令完整 | 证据不完整会影响可信度。 |

## 15. 下一步建议

推荐唯一下一步：C00 architecture map completion。

理由：当前 Project C 本体结构清楚但实现为空，官方源码模块清楚但尚未形成一张“真实文件级输入/输出架构图”。在安装依赖和构建前，先把 `DataBus` 字段、demo 主循环、模型资源、MPC/WBC/PVT 输入输出做成结构化 architecture map，最稳。

不建议下一步直接运行 `walk_mpc_wbc`。它会同时引入 CMake、OpenGL、MuJoCo viewer、MPC QP、WBC QP、PVT 和模型路径问题，不利于定位。

## 16. 边界确认

- 未修改源码：是。
- 未执行 `git add`：是。
- 未执行 `git commit`：是。
- 未执行 `git push`：是。
- 未修改 Project A / B / D：是。
- 未修改 `external/open_source_repos`：是。
- 未修改 `shared/robot_assets/vendor`：是。
- 未运行长时间 MuJoCo：是。
- 未生成 MP4：是。
