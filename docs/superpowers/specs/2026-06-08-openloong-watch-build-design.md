# OpenLoong Watch Sync Build Design

**Goal**

为 OpenLoong 任意 demo 提供“源码自动同步到 build-root worktree + Docker 内自动增量编译”的宿主机工具，减少手工同步和手工编译的重复操作。

**Scope**

- 监听 `external/open_source_repos/OpenLoong-Dyn-Control`
- 复用现有 demo 名到 executable/target 的映射
- 将源码同步到 runner 当前使用的 build-root worktree
- 在 `openloong-ubuntu22-build:local` 容器中执行指定 target 的增量编译
- 提供 `--dry-run`，用于在不监听、不编译的情况下核对路径和命令

**Architecture**

将现有 runner 中的 demo 映射与路径解析抽到共享 shell 脚本中，由 runner 和 watcher 共用。新增 watcher 脚本只负责三件事：解析 demo 与构建目标、监听源码变化、执行一次“rsync 同步 + Docker build”。为了兼容当前环境缺少 `inotifywait` 的情况，watcher 优先使用 `inotifywait`，否则退化为轮询模式。

**Data Flow**

`external/OpenLoong-Dyn-Control` 源码变更
-> watcher 检测到事件
-> `rsync` 到 `outputs/.../worktree/OpenLoong-Dyn-Control`
-> Docker 容器内 `cmake --build build --target <demo target>`
-> build-root 中的可执行文件更新时间戳刷新

**Non-Goals**

- 不做正在运行 demo 的热更新
- 不自动重启 viewer
- 不自动推断多个受影响 demo 并批量编译
- 不改动控制器数学逻辑，只处理工具链自动化
