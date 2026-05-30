#!/usr/bin/env bash
set -euo pipefail

# C04-FIX-RUNTIME-R2：R2 worktree 仍只读挂载，record/ 使用可写 overlay。

cd /home/ubuntu/Robot_Dynamics_Control

REPO_ROOT="/home/ubuntu/Robot_Dynamics_Control"
R2_ROOT="$REPO_ROOT/projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827"
RUN_ROOT="$REPO_ROOT/projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_wbc_speed_test_runtime_fix_R2/20260530_184509"
IMAGE_TAG="openloong-ubuntu22-build:local"

git status --short
git diff --stat

mkdir -p "$RUN_ROOT/runtime_record"

docker run --rm \
  -v "$R2_ROOT:/run_root:ro" \
  -v "$RUN_ROOT:/c04_fix_root" \
  -v "$RUN_ROOT/runtime_record:/run_root/worktree/OpenLoong-Dyn-Control/record" \
  "$IMAGE_TAG" \
  bash /c04_fix_root/commands/container_run_wbc_speed_test_record_overlay.sh
