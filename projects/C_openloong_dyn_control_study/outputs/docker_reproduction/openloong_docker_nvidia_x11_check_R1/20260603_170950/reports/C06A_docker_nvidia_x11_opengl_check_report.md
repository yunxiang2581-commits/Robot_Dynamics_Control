# C06A Docker NVIDIA X11 OpenGL Check Report

## 1. 本轮目标

本轮目标是验证普通 Ubuntu 桌面环境下，`openloong-ubuntu22-build:local` 是否具备后续 MuJoCo / GLFW viewer 所需的 Docker NVIDIA、X11 和 OpenGL 前置条件。

本轮明确不做以下事情：

- 不运行 `walk_mpc_wbc`
- 不自动长时间运行 MuJoCo viewer
- 不生成 MP4
- 不修改 `external/open_source_repos/OpenLoong-Dyn-Control/` 官方源码

## 2. 宿主机 GPU 与显示环境

宿主机检查日志：

- `logs/01_host_gpu_display_check.log`

结果摘要：

- `DISPLAY=:0`
- `XDG_SESSION_TYPE=wayland`
- `WAYLAND_DISPLAY=wayland-0`
- `/tmp/.X11-unix` 存在，看到 `X0` 和 `X1`
- 宿主机 `nvidia-smi`：未找到
- 宿主机 `glxinfo`：未找到
- `docker --version`：`29.4.1`
- `docker compose version`：`v5.1.3`

判断：

- 宿主机显示环境变量存在，X11 socket 也存在。
- 但宿主机缺少 `nvidia-smi`，因此不能证明当前系统已经具备可用的 NVIDIA 驱动用户态工具链。
- 宿主机缺少 `glxinfo`，因此不能在宿主机侧先验证 OpenGL / GLX 是否可用。

状态：

- `HOST_NVIDIA_NOT_READY`

## 3. Docker GPU runtime 检查

相关日志：

- `logs/02_docker_info.log`
- `logs/03b_docker_nvidia_runtime_static_check.log`
- `logs/03_docker_gpu_cuda_image_test.log`
- `logs/04_docker_gpu_openloong_image_test.log`

静态证据：

- Docker `runtimes` 只有 `io.containerd.runc.v2` 和 `runc`
- `Default Runtime` 是 `runc`
- 宿主机 `nvidia-ctk`：未找到

执行策略：

- 按任务边界，宿主机 `nvidia-smi` 缺失后，不再继续 `docker run --gpus all ...` 的主动 GPU 运行测试。
- 因此 `logs/03_docker_gpu_cuda_image_test.log` 和 `logs/04_docker_gpu_openloong_image_test.log` 记录的是“策略性跳过”，不是 PASS。

判断：

- 没有 fresh 证据表明 `docker run --gpus all` 可用。
- 从静态信息看，当前 Docker 没注册 `nvidia` runtime，且宿主机没看到 `nvidia-ctk`，这强烈指向 `NVIDIA Container Toolkit` 尚未配置。

建议的用户手动命令：

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi
```

状态：

- `NVIDIA_CONTAINER_TOOLKIT_NOT_CONFIGURED`（基于静态证据推断）
- `GPU_RUNTIME_PASS`：未验证

## 4. OpenLoong Docker image GUI 工具

相关日志：

- `logs/05_openloong_image_gui_tools_check.log`
- `logs/06_rebuild_openloong_image_with_gui_tools.log`
- `logs/07_openloong_image_gui_tools_recheck.log`

修改文件：

- `projects/C_openloong_dyn_control_study/docker/openloong_ubuntu22_build/Dockerfile`

修改内容：

- 新增 `x11-apps`
- 新增 `mesa-utils`
- 新增 `libgl1-mesa-dri`
- 新增 `libglx-mesa0`
- 新增 `libglfw3`

修改原因：

- 初次镜像检查时，`xeyes` 和 `glxinfo` 都不存在，不能支撑后续 X11 / OpenGL viewer 调试。

重建后结果：

- `xeyes`：`/usr/bin/xeyes`
- `glxinfo`：`/usr/bin/glxinfo`
- `libglfw.so.3`、`libGL*`、`libEGL*`、`libX11*` 均可在镜像内查询到

判断：

- 镜像层 GUI 调试工具已补齐。

状态：

- `IMAGE_GUI_TOOLS_READY`

## 5. X11 测试

相关日志：

- `logs/08_xhost_enable.log`
- `logs/09_docker_xeyes_test.log`

本轮状态：

- 未执行

原因：

- 宿主机 `nvidia-smi` 缺失，GPU / GLX 基础未就绪。
- 按本轮任务边界，停止后续依赖 GPU 通过性判断的 Docker viewer 测试。

状态：

- `SKIPPED_DUE_TO_HOST_NVIDIA_NOT_READY`

## 6. OpenGL 测试

相关日志：

- `logs/10_docker_glxinfo_test.log`

本轮状态：

- 未执行

原因：

- 宿主机 `glxinfo` 与 `nvidia-smi` 都缺失，且 Docker `nvidia` runtime 未见注册。
- 在这个前提下运行容器内 `glxinfo -B`，只能得到噪声，不足以证明 viewer 条件成立。

状态：

- `OpenGL_NOT_READY`

## 7. walk_wbc runtime precheck

相关日志：

- `logs/11_walk_wbc_ldd_rpath_check.log`

检查结果：

- R2 挂载到 `/run_root`：PASS
- `walk_wbc` 存在：PASS
- `walk_mpc_wbc` 存在：PASS
- `wbc_speed_test` 存在：PASS
- `ldd ./walk_wbc` 能找到关键库：
  - `libmujoco.so.3.1.1`
  - `libqpOASES.so.3.2`
- `RUNPATH` 仍指向 `/run_root/worktree/OpenLoong-Dyn-Control/...`

判断：

- 从静态运行时依赖角度看，`walk_wbc` 的 build 产物与 `/run_root` 挂载策略匹配。
- 本轮没有自动实际启动 `walk_wbc` viewer。
- `record/` 可写挂载模板已写入宿主机运行脚本，避免回写 R2 只读 worktree。

状态：

- `WALK_WBC_PRECHECK_PASS`

## 8. 是否可以进入 walk_wbc viewer

当前状态：

- `NOT_READY`

原因：

- 宿主机 `nvidia-smi` 缺失，首先卡在 host NVIDIA readiness。
- 宿主机 `glxinfo` 缺失，无法先做 host OpenGL 诊断。
- Docker 未见 `nvidia` runtime，`nvidia-ctk` 也未找到，说明 `--gpus all` 尚无通过证据。
- 虽然镜像调试工具和 `walk_wbc` 静态依赖已经就绪，但 GPU/X11/OpenGL 通过链路没有打通。

只有在以下条件都补齐后，才可进入 viewer 手动运行：

- 宿主机 `nvidia-smi` 正常
- 宿主机 `glxinfo -B` 正常
- Docker `--gpus all` 测试通过
- X11 `xeyes` 测试通过
- 容器内 `glxinfo -B` 可返回有效 renderer/vendor

## 9. walk_wbc viewer 运行模板

已生成：

- `commands/container_run_walk_wbc_viewer.sh`
- `commands/run_walk_wbc_viewer_host_template.sh`

说明：

- `container_run_walk_wbc_viewer.sh`：容器内执行，先写预检查日志，再运行 `./walk_wbc`
- `run_walk_wbc_viewer_host_template.sh`：宿主机执行，负责 `xhost`、挂载 `/run_root`、挂载 `runtime_record/`、传递显示环境

本轮不自动运行该模板脚本。

当且仅当上面的宿主机和 Docker GPU 条件补齐后，建议手动执行：

```bash
bash commands/run_walk_wbc_viewer_host_template.sh
```

GUI 测试结束后，可手动收回 X11 权限：

```bash
xhost -SI:localuser:root
```

## 10. 本轮结论

本轮已经完成两件事情：

- 把 OpenLoong Docker 镜像补到具备 `xeyes` / `glxinfo` / OpenGL 调试库的状态
- 验证 `walk_wbc` build 产物、关键动态库和 `/run_root` RUNPATH 策略仍然成立

本轮没有打通 viewer 的根因不在 OpenLoong 源码，也不在 `walk_wbc` 本身，而是在更前面的宿主机 / Docker GPU runtime 层：

- `HOST_NVIDIA_NOT_READY`
- `NVIDIA_CONTAINER_TOOLKIT_NOT_CONFIGURED`（静态证据推断）

因此本轮最终状态为：

- `NOT_READY_FOR_WALK_WBC_VIEWER`
