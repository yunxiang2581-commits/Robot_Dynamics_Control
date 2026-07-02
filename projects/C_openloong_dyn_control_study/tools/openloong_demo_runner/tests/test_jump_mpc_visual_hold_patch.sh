#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/../../../../../" && pwd)

container_runner="$REPO_ROOT/projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/commands/container_run_openloong_demo.sh"
build_jump="$REPO_ROOT/projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827/worktree/OpenLoong-Dyn-Control/demo/jump_mpc.cpp"

grep -q "OPENLOONG_HOLD_WINDOW_AFTER_END" "$container_runner"
grep -q "OPENLOONG_HOLD_WINDOW_SECONDS" "$container_runner"
grep -q "OPENLOONG_HOLD_WINDOW_AFTER_END" "$build_jump"
grep -q "OPENLOONG_HOLD_WINDOW_SECONDS" "$build_jump"
grep -q "holdWindowAfterEnd" "$build_jump"
grep -q "holdWindowSeconds" "$build_jump"
