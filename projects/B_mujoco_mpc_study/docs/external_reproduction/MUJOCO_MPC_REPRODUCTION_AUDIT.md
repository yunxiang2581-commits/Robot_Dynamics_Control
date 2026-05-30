# MuJoCo MPC 外部项目复现审计

## 0. 审计结论

本文件用于锁定 `mujoco_mpc` 外部开源项目的第一轮复现范围。它不是学习型任务骨架，也不是已经跑通后的复现报告；它的作用是把论文目标、官方 demo、构建命令、当前环境缺口和最小证据链先固定下来，避免后续复现时目标漂移。

建议第一轮复现目标：

1. 在 Linux / WSL Ubuntu 环境中完成 MJPC C++ Release 构建。
2. 运行官方 C++ test suite，保存 `ctest` 结果。
3. 构建并安装 MJPC Python API。
4. 运行 `agent_test.py`。
5. 运行 Cartpole Python demo 或 MJPC GUI Cartpole demo，形成最小可视化证据。

不建议第一轮直接复现 quadruped、humanoid tracking、bimanual 或 Rubik's cube 高级任务。这些任务适合作为第二轮展示目标。

## 1. 外部源码身份

本地源码位置：

```text
external/open_source_repos/mujoco_mpc
```

上游仓库：

```text
https://github.com/google-deepmind/mujoco_mpc.git
```

当前本地提交：

```text
ff572a2 Added support for motion strategies
```

当前外部仓库状态：

```text
## main...origin/main
```

第一轮复现应保持外部源码只读，不在 `external/open_source_repos/mujoco_mpc` 内修改官方代码。若需要保存额外日志、视频或报告，优先写入本仓库的输出目录：

```text
projects/B_mujoco_mpc_study/outputs/external_reproduction/mujoco_mpc/20260529_mjpc_cartpole_audit/
```

## 2. 论文与核心结果锁定

MJPC README 指向的论文是：

```text
Predictive Sampling: Real-time Behaviour Synthesis with MuJoCo
arXiv:2212.00541
Howell et al., 2022
```

本项目要复现的不是单个固定数值表格，而是官方系统的核心能力链：

```text
MuJoCo model + task residual + horizon cost + planner + receding horizon execution
```

官方文档中锁定的核心概念：

- MJPC 是基于 MuJoCo 的实时 predictive control 框架。
- task 由 MJCF / XML 资源和 C++ residual function 共同定义。
- cost 由 residual、norm 和 weight 组成。
- planner 包含 derivative-free 的 Predictive Sampling，也包含 derivative-based 的 iLQG 和 Gradient Descent。
- 官方示例覆盖 Cartpole、Acrobot、Swimmer、Walker、Quadruped、Humanoid tracking、Panda、in-hand manipulation 等任务。

第一轮可复现结果应定义为工程型证据，而不是论文指标复刻：

```text
C++ build passes
C++ tests pass
MJPC binary exists and can launch
Python API installs
agent_test.py passes
Cartpole demo can run and produce trajectory / cost / visual evidence
```

## 3. 官方入口与 demo 分层

### 3.1 C++ GUI 入口

官方 README 给出的 GUI 构建和运行入口：

```bash
cd mujoco_mpc
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE:STRING=Release -G Ninja -DMJPC_BUILD_GRPC_SERVICE:BOOL=ON
cmake --build . --config=Release
cd bin
./mjpc
```

在 Ubuntu README 版本中，官方建议使用 `clang-12`；当前 GitHub Actions workflow 使用 Ubuntu 22.04 + `clang-13`。因此第一轮 Linux / WSL 复现建议优先对齐 GitHub Actions，而不是当前 Windows 原生环境。

### 3.2 C++ 测试入口

GitHub Actions 中的测试命令：

```bash
cd build/mjpc/test
ctest -C Release --output-on-failure .
```

这应作为第一轮最小复现的核心判据。GUI 能打开很重要，但 `ctest` 更适合作为可记录、可比较、可重复的证据。

### 3.3 Python API 入口

官方 README 的 Python API 流程：

```bash
conda create -n mjpc python=3.10
conda activate mjpc
pip install mujoco
cd python
python setup.py install
python mujoco_mpc/agent_test.py
python mujoco_mpc/demos/agent/cartpole_gui.py
```

`python/setup.py` 会触发 CMake 构建 `agent_server` 和 `ui_agent_server`，并要求 `build/mjpc/tasks` 中已经存在 task assets。因此 Python API 不能脱离 C++ 构建独立验证。

Python 安装依赖包括：

```text
brax
grpcio
grpcio-tools
matplotlib
mediapy
mujoco >= 3.1.1
mujoco-mjx
protobuf
absl-py (test extra)
```

### 3.4 第一轮推荐 demo

推荐第一轮只选择 Cartpole：

```text
python/mujoco_mpc/demos/agent/cartpole.py
python/mujoco_mpc/demos/agent/cartpole_gui.py
```

原因：

- Cartpole 是官方 Python API demo 中最小的闭环 MPC 示例。
- 任务文件路径固定指向 `build/mjpc/tasks/cartpole/task.xml`。
- 运行过程会调用 `Agent(task_id="Cartpole", model=model)`、`planner_step()`、`get_action()`，能验证 gRPC agent service 和 MuJoCo step 的完整链路。
- demo 里已经记录 `qpos`、`qvel`、`ctrl`、`cost_total`、`cost_terms` 和 rendered frames，适合作为后续保存 metrics / video 的基础。

第二轮再考虑：

```text
quadruped/task_flat.xml
humanoid/tracking/task.xml
shadow_reorient/task.xml
```

这些任务展示价值更强，但模型、资源和运行时间风险更高。

## 4. 推荐构建命令

### 4.1 推荐平台

第一轮建议使用：

```text
Ubuntu 22.04 / WSL2 Ubuntu 22.04
clang-13
cmake
ninja-build
zlib1g-dev
OpenGL / X11 related dev packages
Python 3.10 for Python API
```

README 说 MJPC 测试过 Ubuntu 20.04 和 macOS-12；当前 workflow 已经更新到 Ubuntu 22.04 和 macOS-15。为了贴近 CI，建议优先使用 Ubuntu 22.04 + `clang-13`。

### 4.2 Linux / WSL 依赖安装

```bash
sudo apt-get update
sudo apt-get install -y \
  cmake \
  clang-13 \
  libgl1-mesa-dev \
  libxinerama-dev \
  libxcursor-dev \
  libxrandr-dev \
  libxi-dev \
  ninja-build \
  zlib1g-dev
```

### 4.3 C++ Release 构建

```bash
export REPO_ROOT=/mnt/d/project/Robot_Dynamics_Control
export MJPC_ROOT="$REPO_ROOT/external/open_source_repos/mujoco_mpc"

cd "$MJPC_ROOT"
git rev-parse --short HEAD

rm -rf build
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE:STRING=Release \
  -G Ninja \
  -DCMAKE_C_COMPILER:STRING=clang-13 \
  -DCMAKE_CXX_COMPILER:STRING=clang++-13 \
  -DMJPC_BUILD_GRPC_SERVICE:BOOL=ON

cmake --build build --config=Release
```

预期产物：

```text
build/bin/mjpc
build/bin/agent_server
build/bin/ui_agent_server
build/mjpc/tasks/cartpole/task.xml
```

### 4.4 C++ 测试

```bash
export REPO_ROOT=/mnt/d/project/Robot_Dynamics_Control
export MJPC_ROOT="$REPO_ROOT/external/open_source_repos/mujoco_mpc"

cd "$MJPC_ROOT/build/mjpc/test"
ctest -C Release --output-on-failure .
```

预期证据：

```text
100% tests passed, 0 tests failed out of <N>
```

实际 `<N>` 以当前源码和 CMake 配置为准。

### 4.5 Python API 安装与测试

```bash
conda create -n mjpc python=3.10 -y
conda activate mjpc
python -m pip install --upgrade pip
python -m pip install mujoco

export REPO_ROOT=/mnt/d/project/Robot_Dynamics_Control
export MJPC_ROOT="$REPO_ROOT/external/open_source_repos/mujoco_mpc"

cd "$MJPC_ROOT/python"
python setup.py install
python mujoco_mpc/agent_test.py
```

若 `agent_test.py` 通过，再运行最小 demo：

```bash
python mujoco_mpc/demos/agent/cartpole.py
```

若需要 GUI 交互：

```bash
python mujoco_mpc/demos/agent/cartpole_gui.py
```

## 5. 当前机器环境缺口

当前默认环境：

```text
OS: Windows
Default Python: D:\anaconda\python.exe
Python version: 3.12.4
```

默认 Python 缺口：

```text
mujoco: missing
pinocchio: missing, but MJPC first round does not require it
osqp: missing, but MJPC official first round does not require it
qpsolvers: missing, but MJPC official first round does not require it
```

当前已有较接近的 conda 环境：

```text
D:\anaconda\envs\mujoco_py311\python.exe
Python 3.11.15
mujoco 3.6.0
osqp 1.1.1
qpsolvers 4.11.0
```

但官方 Python API 建议 Python 3.10，因此 `mujoco_py311` 只能作为备用 smoke 环境，不建议作为第一轮正式复现环境。

当前 Windows 构建工具：

```text
cmake 4.3.1
ninja 1.13.2
MSVC cl.exe available
MSYS2 gcc/g++ available
uv 0.11.8
```

主要缺口和风险：

- MJPC README 明确 Windows 未测试。
- 当前 GitHub Actions matrix 不包含 Windows 构建任务。
- 官方 Linux 构建依赖 OpenGL / X11 dev packages，Windows 原生不等价。
- `MJPC_BUILD_GRPC_SERVICE=ON` 会引入较大的 gRPC 构建 / 下载成本。
- CMake 会通过外部依赖获取 MuJoCo、menagerie、dm_control 等资源，需要稳定网络。
- GUI / renderer 需要可用显示环境；WSL 下可能需要 WSLg 或 X server。

因此，第一轮正式复现建议走 Linux / WSL，不建议直接用当前 Windows 默认 Python 环境硬跑。

## 6. 最小复现证据清单

建议每次正式复现都保存到独立 run 目录：

```text
projects/B_mujoco_mpc_study/outputs/external_reproduction/mujoco_mpc/20260529_mjpc_cartpole_audit/
```

最小证据包括：

```text
00_source_identity.txt
01_environment.txt
02_cmake_configure.log
03_cmake_build.log
04_ctest.log
05_python_install.log
06_agent_test.log
07_cartpole_demo.log
videos/cartpole_demo.mp4 or screenshots/cartpole_gui.png
figures/cartpole_cost.png or metrics/cartpole_cost.csv
REPRODUCTION_REPORT.md
```

每个文件的含义：

- `00_source_identity.txt`：记录 repo URL、commit hash、`git status --short --branch`。
- `01_environment.txt`：记录 OS、compiler、CMake、Ninja、Python、MuJoCo 版本。
- `02_cmake_configure.log`：证明 CMake 配置成功，包含关键 flags。
- `03_cmake_build.log`：证明 C++ 构建成功。
- `04_ctest.log`：证明官方 C++ tests 通过。
- `05_python_install.log`：证明 Python API 安装成功。
- `06_agent_test.log`：证明 Python API 基础测试通过。
- `07_cartpole_demo.log`：证明 Cartpole demo 执行过。
- `videos/` 或 `screenshots/`：证明可视化链路正常。
- `figures/` 或 `metrics/`：证明轨迹、控制量或 cost 有可复盘输出。
- `REPRODUCTION_REPORT.md`：总结实际命令、结果、失败点和偏离官方说明的地方。

第一轮通过标准：

```text
External source unchanged
C++ configure success
C++ build success
C++ ctest success
Python API install success
agent_test.py success
Cartpole demo produces at least one visual or numeric artifact
```

## 7. 数学逻辑、输入输出与参数变化

本审计阶段不改变 MJPC 数学逻辑，也不改变官方 residual、cost、planner 或 task XML。后续第一轮复现的输入输出应固定为：

输入：

```text
Official MJPC source at ff572a2
Official Cartpole task XML
Official C++ / Python API commands
Release build configuration
```

输出：

```text
C++ binaries
ctest result
Python API test result
Cartpole rollout states / controls / costs
Cartpole visual artifact
Reproduction report
```

构建参数：

```text
CMAKE_BUILD_TYPE=Release
Generator=Ninja
C compiler=clang-13
C++ compiler=clang++-13
MJPC_BUILD_GRPC_SERVICE=ON
```

可能风险：

- 若网络下载失败，CMake configure 可能中断。
- 若 gRPC 构建失败，Python API 和 agent demo 无法复现。
- 若显示环境不可用，GUI demo 或 renderer 可能失败，但 `ctest` 仍可作为第一层证据。
- 若使用 Windows 原生构建，失败不一定说明项目不可复现，只能说明偏离官方测试平台。

## 8. 下一步建议

推荐下一步执行顺序：

1. 准备 WSL / Ubuntu 22.04 复现环境。
2. 按本审计文档运行 C++ configure / build / ctest。
3. 保存完整日志到 `projects/B_mujoco_mpc_study/outputs/external_reproduction/mujoco_mpc/20260529_mjpc_cartpole_audit/`，后续复现实验可按日期另建 run 目录。
4. 再安装 Python API 并运行 `agent_test.py`。
5. 最后运行 Cartpole demo，补充视频、截图、cost 曲线或 CSV。
6. 写正式 `REPRODUCTION_REPORT.md`，把实际结果和本审计中的预期证据逐项对齐。

如果第一轮完成，再决定是否进入第二轮高级任务：Quadruped flat、Humanoid tracking 或 Shadow hand reorientation。
