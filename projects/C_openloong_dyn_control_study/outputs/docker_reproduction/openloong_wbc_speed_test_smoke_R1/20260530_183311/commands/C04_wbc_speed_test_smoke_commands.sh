#!/usr/bin/env bash
set -euo pipefail

# C04-DOCKER-SMOKE-R1：审查并短时运行 wbc_speed_test。
# 边界：只运行 wbc_speed_test；不运行 walk_wbc / walk_mpc_wbc；不修改 R2 worktree。

cd /home/ubuntu/Robot_Dynamics_Control

REPO_ROOT="/home/ubuntu/Robot_Dynamics_Control"
R2_ROOT="$REPO_ROOT/projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827"
C04_RUN_ROOT="$REPO_ROOT/projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_wbc_speed_test_smoke_R1/20260530_183311"
IMAGE_TAG="openloong-ubuntu22-build:local"

git status --short
git diff --stat

ls -lah "$R2_ROOT/worktree/OpenLoong-Dyn-Control/build"
ls -lah "$R2_ROOT/build_artifacts"

find "$R2_ROOT/worktree/OpenLoong-Dyn-Control" -maxdepth 4 -type f \
  | grep -Ei "wbc.*speed|speed.*test|wbc_speed_test|test" \
  | sort

rg -n "wbc_speed_test|main\(|mujoco|mj_|GLFW|glfw|viewer|window|simulate|while|for \(|DataBus|WBC|WBC_priority|Pin_KinDyn" \
  "$R2_ROOT/worktree/OpenLoong-Dyn-Control"

docker run --rm \
  -v "$R2_ROOT:/r2_root:ro" \
  -v "$C04_RUN_ROOT:/c04_root" \
  "$IMAGE_TAG" \
  bash /c04_root/commands/container_run_wbc_speed_test.sh
