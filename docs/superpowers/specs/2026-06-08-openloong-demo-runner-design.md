# OpenLoong Demo Runner Design

**Goal**

为 OpenLoong 的多个 MuJoCo demo 提供统一的 Docker + X11 启动入口，同时保留每个 demo 的独立薄包装脚本，方便学习 demo 名称、可执行文件名和运行方式之间的对应关系。

**Scope**

- 提供一个宿主机总入口脚本
- 提供一个容器内通用执行脚本
- 提供一个包装脚本生成器
- 生成每个 demo 的薄包装脚本
- 为每次运行创建独立输出目录并保存日志

**Architecture**

宿主机总入口脚本负责解析 demo 名、检查环境、创建运行输出目录，并统一发起 `docker run`。容器内通用脚本负责记录 GPU/OpenGL/依赖证据并执行目标可执行文件。薄包装脚本只固定 demo 名，然后回调总入口脚本，避免重复维护多份 `docker run` 命令。

**Demo Mapping**

本实现不再假设 `demo/*.cpp` 文件名与可执行文件名总是完全一致，而是显式维护 demo 到 target 的映射表。这样可以正确覆盖 `walk_wbc_speed_test.cpp -> wbc_speed_test` 这类特例。

**Output Layout**

每次运行创建：

`projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_demo_runs/<timestamp>_<demo_name>/`

目录下包含：

- `commands/`
- `logs/`
- `runtime_record/`

**Non-Goals**

- 本轮不自动编译缺失 target
- 本轮不自动解析全部 CMake target
- 本轮不增加 joystick/headless/参数透传等扩展模式

