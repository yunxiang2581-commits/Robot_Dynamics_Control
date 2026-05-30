#!/usr/bin/env bash
set -euo pipefail

# C04-FIX-RUNTIME-R1：只通过 /run_root 只读挂载匹配 build-time rpath，重跑 wbc_speed_test。

cd /home/ubuntu/Robot_Dynamics_Control

REPO_ROOT="/home/ubuntu/Robot_Dynamics_Control"
R2_ROOT="$REPO_ROOT/projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827"
RUN_ROOT="$REPO_ROOT/projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_wbc_speed_test_runtime_fix_R1/20260530_184108"
IMAGE_TAG="openloong-ubuntu22-build:local"

git status --short
git diff --stat

ls -lah "$R2_ROOT/worktree/OpenLoong-Dyn-Control/build" || true
ls -lah "$R2_ROOT/worktree/OpenLoong-Dyn-Control/third_party/qpOASES/lin_x64" || true
ls -lah "$R2_ROOT/worktree/OpenLoong-Dyn-Control/third_party/mujoco/lin_x64" || true

docker run --rm \
  -v "$R2_ROOT:/run_root:ro" \
  -v "$RUN_ROOT:/c04_fix_root" \
  "$IMAGE_TAG" \
  bash /c04_fix_root/commands/container_run_wbc_speed_test_rpath_fix.sh

# 只有 /run_root 路径匹配仍然找不到 shared library 时，才执行 LD_LIBRARY_PATH 对照。
