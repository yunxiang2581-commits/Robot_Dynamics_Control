#!/usr/bin/env bash
set -euo pipefail

cd /run_root/worktree/OpenLoong-Dyn-Control

{
  echo "gcc-11:"
  gcc-11 --version
  echo "g++-11:"
  g++-11 --version
  echo "cmake:"
  cmake --version
  echo "make:"
  make --version
  echo "mujoco libs:"
  ls -lah third_party/mujoco/lin_x64/libmujoco.so*
  if command -v file >/dev/null 2>&1; then
    file third_party/mujoco/lin_x64/libmujoco.so third_party/mujoco/lin_x64/libmujoco.so.3.1.1
  else
    echo "file command not available in this Docker image; skip file-type check inside container"
  fi
  readlink third_party/mujoco/lin_x64/libmujoco.so || true
} 2>&1 | tee /run_root/logs/03_container_env_and_mujoco.log

cmake -S . -B build \
  -DCMAKE_C_COMPILER=gcc-11 \
  -DCMAKE_CXX_COMPILER=g++-11 \
  2>&1 | tee /run_root/logs/04_cmake_configure.log

cmake --build build -j"$(nproc)" \
  2>&1 | tee /run_root/logs/05_cmake_build.log

find build -maxdepth 2 -type f -executable | sort \
  | tee /run_root/build_artifacts/executable_files.txt

for target in walk_wbc walk_mpc_wbc wbc_speed_test jump_mpc float_control; do
  if [ -x "build/$target" ]; then
    echo "$target FOUND build/$target"
  else
    echo "$target NOT_FOUND build/$target"
  fi
done | tee /run_root/build_artifacts/build_targets_check.txt
