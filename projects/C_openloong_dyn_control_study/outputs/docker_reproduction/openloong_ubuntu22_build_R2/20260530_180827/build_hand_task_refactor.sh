#!/usr/bin/env bash
set -euo pipefail

RUN_ROOT=/mnt/d/project/Robot_Dynamics_Control/projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827
mkdir -p "$RUN_ROOT/logs"

docker run --rm \
  -v "$RUN_ROOT:/run_root" \
  openloong-ubuntu22-build:local \
  bash -lc '
    set -euo pipefail
    cd /run_root/worktree/OpenLoong-Dyn-Control
    cmake -S . -B build \
      -DCMAKE_C_COMPILER=gcc-11 \
      -DCMAKE_CXX_COMPILER=g++-11 \
      2>&1 | tee /run_root/logs/20260703_hand_task_refactor_cmake.log
    cmake --build build --target walk_wbc -j"$(nproc)" \
      2>&1 | tee /run_root/logs/20260703_hand_task_refactor_build_walk_wbc.log
  '
