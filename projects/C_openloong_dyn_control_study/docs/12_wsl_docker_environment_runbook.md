# Project C WSL + Docker 环境记录

## 当前目标

在当前 Windows 电脑上，为 Project C 建立可复用的 WSL2 + Docker Engine 环境，并尽量把 WSL 发行版数据放在 D 盘。

这一步只处理环境：

- 输入：已有 Project C/OpenLoong 源码、Dockerfile、Ubuntu 22.04 WSL rootfs。
- 输出：D 盘上的 WSL Ubuntu 22.04 发行版、后续 Docker Engine 环境、可复用检查脚本。
- 数学逻辑：不改变机器人运动学、动力学、WBC、IK、控制器逻辑。
- 风险：WSL 组件启用后通常需要重启 Windows；Docker 安装可能需要网络；不要修改 Docker socket 权限绕过问题。

## 当前状态（2026-06-22）

- `Ubuntu-22.04-ProjectC` 已导入并运行在 WSL2 上。
- WSL 内 `docker version` 可用，客户端和服务端版本均为 `29.6.0`。
- `docker info` 显示的 `DockerRootDir` 为 `/var/lib/docker`，由 D 盘上的 WSL 发行版虚拟磁盘承载。
- `openloong-ubuntu22-build:local` 镜像已成功构建。
- `test_openloong_watch_sync_build.sh` 已通过。
- `/mnt/d/project/Robot_Dynamics_Control` 在 WSL 内可访问。

## 已完成

- 已启用 Windows 功能：
  - `Microsoft-Windows-Subsystem-Linux`
  - `VirtualMachinePlatform`
- 已创建 D 盘目录：
  - `D:\wsl\downloads`
  - `D:\wsl\distros`
  - `D:\wsl\docker-data`（历史尝试目录，安装完成后已清理）
- 已下载并使用 Ubuntu 22.04 WSL rootfs 导入发行版：
  - `D:\wsl\downloads\ubuntu-jammy-wsl-amd64-ubuntu22.04lts.rootfs.tar.gz`
  - 下载时大小约 `325.3 MB`
  - 本机校验到的 SHA256：
    `1483cc5c1dce13064f774834cbffdff226559fd522a67a381a8ea77d63fb4109`
- 安装完成后已清理下载残留：
  - `D:\wsl\downloads\ubuntu-jammy-wsl-amd64-ubuntu22.04lts.rootfs.tar.gz`
  - `D:\wsl\downloads\wsl.2.7.8.0.x64.msi`
  - `D:\wsl\downloads`
  - `D:\wsl\docker-data`
- Docker build cache 已通过 `docker builder prune -af` 清理；保留镜像：
  - `openloong-ubuntu22-build:local`
  - `ubuntu:22.04`
- WSL 内 apt 缓存已通过 `apt-get clean` 和清理 `/var/lib/apt/lists/*` 释放。

## 历史阻塞点（已解决）

当时 `wsl.exe` 仍只显示安装引导功能，`--list` / `--import` 还不可用；`LxssManager` 服务也尚未出现。

这说明 Windows WSL 功能虽然已经启用，但当前 Windows 会话还没有完整加载 WSL 服务。下一步优先重启 Windows，再继续导入 Ubuntu。

## 重启后继续（保留入口，方便复现）

在 PowerShell 中运行：

```powershell
cd D:\project\Robot_Dynamics_Control
powershell -ExecutionPolicy Bypass -File .\projects\C_openloong_dyn_control_study\tools\wsl_projectc_env\continue_after_reboot.ps1
```

脚本会做这些事：

1. 检查 `D:\wsl` 目录和 Ubuntu rootfs。
2. 检查 `wsl.exe` 是否已经支持 `--import` / `--list`。
3. 将 Ubuntu 22.04 导入为：
   - 发行版名：`Ubuntu-22.04-ProjectC`
   - 数据目录：`D:\wsl\distros\Ubuntu-22.04-ProjectC`
4. 设置 WSL2 为默认版本。
5. 在 WSL 内检查 `/mnt/d/project/Robot_Dynamics_Control` 是否可访问。

## 后续 Docker 原则

进入 WSL 后再安装 Docker Engine：

```powershell
wsl -d Ubuntu-22.04-ProjectC
```

在 WSL 内安装 Docker 时遵守：

- 不修改 `/var/run/docker.sock` 为 `666`。
- 优先使用官方 Docker Engine 安装流程。
- Docker 数据优先保留在 WSL 默认的 `/var/lib/docker`，由 D 盘上的 WSL 发行版虚拟磁盘承载。
- Project C 仓库路径固定使用 `/mnt/d/project/Robot_Dynamics_Control`。

## 下一步验证目标

WSL 和 Docker 可用后，验证顺序是：

1. `wsl -l -v` 能看到 `Ubuntu-22.04-ProjectC`，版本为 WSL2。
2. WSL 内能访问 `/mnt/d/project/Robot_Dynamics_Control`。
3. WSL 内 `docker version` 可用。
4. 构建 Project C 镜像：`openloong-ubuntu22-build:local`。
5. 使用已有 R2 构建产物做 dry-run / smoke check。

## 本机验证结果

- `wsl -l -v`：已看到 `Ubuntu-22.04-ProjectC`，状态为 Running，版本为 2。
- `docker version`：已返回 Client/Server `29.6.0`。
- `docker build -t openloong-ubuntu22-build:local ...`：已成功。
- `test_openloong_watch_sync_build.sh`：已成功。

## 2026-06-27 复核

- WSL 发行版：`Ubuntu-22.04-ProjectC`，WSL2。
- WSL 发行版虚拟磁盘：`D:\wsl\distros\Ubuntu-22.04-ProjectC\ext4.vhdx`，当前约 `5.36 GB`。
- Docker daemon：active。
- Docker version：Client/Server `29.6.0`。
- Docker info：`DockerRootDir=/var/lib/docker`，`Driver=overlayfs`，`Cgroup=systemd`。
- Docker 镜像：`openloong-ubuntu22-build:local` 已存在，约 `2.34 GB`。
- Project C 脚本测试：`test_openloong_watch_sync_build.sh` 通过。
- 容器 smoke check：镜像内 `gcc-11`、`g++-11`、`cmake`、`python3` 可用；已有 R2 build 产物中的 `walk_wbc` 和 `wbc_speed_test` 可在容器挂载路径下访问。
