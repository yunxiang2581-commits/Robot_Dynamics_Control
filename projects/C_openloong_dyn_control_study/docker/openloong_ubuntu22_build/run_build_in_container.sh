#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="/home/ubuntu/Robot_Dynamics_Control"
SRC_ROOT="$REPO_ROOT/external/open_source_repos/OpenLoong-Dyn-Control"
RUN_ROOT="${1:-$REPO_ROOT/projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R1/20260530_000000}"
IMAGE_TAG="openloong-ubuntu22-build:local"

mkdir -p \
  "$RUN_ROOT/logs" \
  "$RUN_ROOT/reports" \
  "$RUN_ROOT/commands" \
  "$RUN_ROOT/build_artifacts" \
  "$RUN_ROOT/worktree"

if [[ ! -d "$SRC_ROOT" ]]; then
  echo "Official source directory not found: $SRC_ROOT" | tee "$RUN_ROOT/logs/04_source_copy.log"
  exit 1
fi

{
  echo "# container_build_openloong.sh"
  echo
  echo 'set -euo pipefail'
  echo
  echo 'cd /run_root/worktree/OpenLoong-Dyn-Control'
  echo
  echo '{'
  echo '  echo "## Container compiler environment"'
  echo '  echo'
  echo '  echo "### gcc-11"'
  echo '  gcc-11 --version'
  echo '  echo'
  echo '  echo "### g++-11"'
  echo '  g++-11 --version'
  echo '  echo'
  echo '  echo "### cmake"'
  echo '  cmake --version'
  echo '  echo'
  echo '  echo "### make"'
  echo '  make --version'
  echo '} 2>&1 | tee /run_root/logs/01_container_env.log'
  echo
  echo 'cmake -S . -B build \'
  echo '  -DCMAKE_C_COMPILER=gcc-11 \'
  echo '  -DCMAKE_CXX_COMPILER=g++-11 \'
  echo '  2>&1 | tee /run_root/logs/02_cmake_configure.log'
  echo
  echo 'cmake --build build -j"$(nproc)" \'
  echo '  2>&1 | tee /run_root/logs/03_cmake_build.log'
  echo
  echo 'find build -maxdepth 2 -type f -executable | sort \'
  echo '  | tee /run_root/build_artifacts/executable_files.txt'
  echo
  echo 'for target in walk_wbc walk_mpc_wbc wbc_speed_test jump_mpc float_control; do'
  echo '  if [[ -x "build/$target" ]]; then'
  echo '    echo "$target build/$target FOUND"'
  echo '  else'
  echo '    echo "$target build/$target NOT FOUND"'
  echo '  fi'
  echo 'done | tee /run_root/build_artifacts/build_targets_check.txt'
} > "$RUN_ROOT/commands/container_build_openloong.sh"

rsync -a --delete \
  --exclude build \
  --exclude .git/index.lock \
  "$SRC_ROOT/" \
  "$RUN_ROOT/worktree/OpenLoong-Dyn-Control/" \
  2>&1 | tee "$RUN_ROOT/logs/04_source_copy.log"

docker run --rm \
  -v "$RUN_ROOT:/run_root" \
  "$IMAGE_TAG" \
  bash /run_root/commands/container_build_openloong.sh
