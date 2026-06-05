# C06A R2 Docker NVIDIA X11 OpenGL Resume Report

## 1. 本轮目标

本轮继续上一轮 C06A。

已知前提：

- 宿主机 NVIDIA / OpenGL / Docker GPU runtime 已修复
- 本轮重点验证 OpenLoong Docker 镜像内 GPU / X11 / OpenGL
- 不运行 `walk_mpc_wbc`
- 不自动长时间运行 `walk_wbc`

## 2. 宿主机复查结果

日志：

- `logs/01_host_gpu_display_runtime_recheck.log`

结果摘要：

- `nvidia-smi`：PASS
- 两张 RTX 3090：PASS
- `glxinfo -B`：PASS
- `OpenGL vendor`：`NVIDIA Corporation`
- `OpenGL renderer`：`NVIDIA GeForce RTX 3090/PCIe/SSE2`
- `DISPLAY=:0`
- `XDG_SESSION_TYPE=wayland`
- `WAYLAND_DISPLAY=wayland-0`
- `/tmp/.X11-unix` 存在，含 `X0` 和 `X1`
- `docker info` 中 runtime 含 `nvidia`
- `nvidia-ctk --version`：`1.19.1`

判断：

- 宿主机图形、驱动与 Docker NVIDIA runtime 已就绪。

## 3. Docker GPU runtime 结果

日志：

- `logs/02_docker_cuda_gpu_recheck.log`
- `logs/03_openloong_image_gpu_tools_check.log`

结果摘要：

- `docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi`：PASS
- 容器内看到两张 RTX 3090：PASS
- `openloong-ubuntu22-build:local` 中 `nvidia-smi`：PASS

判断：

- Docker GPU runtime 已通过。

## 4. OpenLoong 镜像工具检查

日志：

- `logs/03_openloong_image_gpu_tools_check.log`

结果摘要：

- `xeyes`：`/usr/bin/xeyes`
- `glxinfo`：`/usr/bin/glxinfo`
- `readelf`：`/usr/bin/readelf`
- `ldd`：`/usr/bin/ldd`
- 镜像内可见 `libGL*`、`libGLX_nvidia*`、`libEGL_nvidia*`、`libX11*`
- 镜像内可见 `libglfw.so.3`

Dockerfile 结论：

- 本轮不需要 rebuild
- Project C Dockerfile 已带 viewer debug tools 补丁

## 5. X11 测试结果

日志：

- `logs/05_xhost_enable.log`
- `logs/06_docker_xeyes_test.log`

结果摘要：

- `xhost +SI:localuser:root`：PASS
- `xeyes` 测试：用户人工确认窗口已经弹出
- `EXIT_CODE=124`，但这次不视为失败
- 没有出现 `cannot open display`

状态：

- `X11_PASS_MANUAL_CONFIRM`

说明：

- 容器内 `xeyes` 已实际启动，后续由人工关闭/容器清理收尾。

## 6. OpenGL 测试结果

日志：

- `logs/07_docker_glxinfo_test.log`

结果摘要：

- 容器内 `glxinfo -B`：PASS
- `OpenGL vendor`：`NVIDIA Corporation`
- `OpenGL renderer`：`NVIDIA GeForce RTX 3090/PCIe/SSE2`
- `direct rendering: Yes`
- 未见 `GLXBadContext`、`libGL error`、`cannot open display`

状态：

- `PASS`

## 7. walk_wbc precheck

日志：

- `logs/08_walk_wbc_ldd_rpath_check.log`
- `logs/08b_glfw_library_presence_check.log`

结果摘要：

- R2 挂载到 `/run_root`：PASS
- `walk_wbc` 存在：PASS
- `walk_mpc_wbc` 存在：PASS，但本轮未运行
- `wbc_speed_test` 存在：PASS
- `libmujoco.so.3.1.1`：found
- `libqpOASES.so.3.2`：found
- `RUNPATH` 仍指向 `/run_root/worktree/OpenLoong-Dyn-Control/...`
- `third_party/glfw/lin_x64/libglfw.so.3`、`libglfw.so.3.2`、`libglfw3.so` 均存在

判断：

- `walk_wbc` 运行前置依赖与 `/run_root` 挂载策略成立。

## 8. 当前 readiness

状态：

- `READY_FOR_MANUAL_WALK_WBC_VIEWER_RUN`

理由：

- Docker GPU PASS
- `xeyes` 已人工确认弹窗
- 容器 `glxinfo -B` PASS
- `walk_wbc` 依赖与 RUNPATH PASS

## 9. 生成的手动运行模板

已生成：

- `commands/container_run_walk_wbc_viewer.sh`
- `commands/run_walk_wbc_viewer_host_template.sh`

手动运行命令：

```bash
bash /home/ubuntu/Robot_Dynamics_Control/projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_docker_nvidia_x11_check_R2/20260603_181426/commands/run_walk_wbc_viewer_host_template.sh
```

## 10. 下一步建议

推荐进入 `C06B-WALK-WBC-VIEWER-R1`：

- 手动运行 `walk_wbc`
- 观察 viewer 是否正常显示
- 使用 `Ctrl+C` 结束
- 保存 `runtime log` / `datalog` / screenshot
- 不运行 `walk_mpc_wbc`

GUI 测试结束后，可手动执行：

```bash
xhost -SI:localuser:root
```

## 11. 边界确认

本轮确认：

- 未修改 `external/open_source_repos/OpenLoong-Dyn-Control/` 官方源码
- 未修改 R2 worktree 源码
- 未运行 `walk_mpc_wbc`
- 未自动长时间运行 `walk_wbc`
- 未生成 MP4
- 未执行 `git add`
- 未执行 `git commit`
- 未执行 `git push`
