#!/usr/bin/env bash
set -euo pipefail

# 这个脚本在宿主机执行。
# 目的：为后续手动 viewer 测试提供一个固定模板，不在本轮自动长时间运行。
# 输入：
# - DISPLAY 和 /tmp/.X11-unix
# - R2_ROOT: C03 R2 build 产物
# - RUN_ROOT: 本轮输出目录
# 输出：
# - 容器内 record/ 写入 RUN_ROOT/runtime_record
# - 预检查与运行日志写入 RUN_ROOT/logs

REPO_ROOT=/home/ubuntu/Robot_Dynamics_Control
R2_ROOT="$REPO_ROOT/projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827"
RUN_ROOT="/home/ubuntu/Robot_Dynamics_Control/projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_docker_nvidia_x11_check_R1/20260603_170950"

xhost +SI:localuser:root

docker run --rm -it \
--gpus all \
-e DISPLAY="$DISPLAY" \
-e NVIDIA_DRIVER_CAPABILITIES=all \
-e __GLX_VENDOR_LIBRARY_NAME=nvidia \
-e QT_X11_NO_MITSHM=1 \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
-v "$R2_ROOT:/run_root:ro" \
-v "$RUN_ROOT/runtime_record:/run_root/worktree/OpenLoong-Dyn-Control/record" \
-v "$RUN_ROOT:/c06_root" \
openloong-ubuntu22-build:local \
bash /c06_root/commands/container_run_walk_wbc_viewer.sh

echo "After testing, you may run:"
echo "xhost -SI:localuser:root"
