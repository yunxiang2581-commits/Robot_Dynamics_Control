#!/usr/bin/env bash
set -euo pipefail

cd /home/ubuntu/Robot_Dynamics_Control

# Docker availability checks.
docker --version || true
docker compose version || true
docker ps || true

# Repository state checks.
git status --short
git diff --stat

# Build the Ubuntu 22.04 image.
bash projects/C_openloong_dyn_control_study/docker/openloong_ubuntu22_build/build_image.sh

# Copy official source to the run worktree and build inside the container.
bash projects/C_openloong_dyn_control_study/docker/openloong_ubuntu22_build/run_build_in_container.sh \
  /home/ubuntu/Robot_Dynamics_Control/projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R1/20260530_000000
