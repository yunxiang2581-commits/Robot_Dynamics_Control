#!/usr/bin/env bash
set -euo pipefail

BUILD_ROOT=/mnt/d/project/Robot_Dynamics_Control/projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827
SMOKE_SECONDS="${1:-35}"

echo "=== RUN T1 force smoke ==="
docker run --rm \
  -v "$BUILD_ROOT:/run_root" \
  openloong-ubuntu22-build:xvfb \
  bash -lc '
    set -euo pipefail
    export XDG_RUNTIME_DIR=/tmp/xdg
    mkdir -p "$XDG_RUNTIME_DIR"
    cd /run_root/worktree/OpenLoong-Dyn-Control/build
    export OPENLOONG_T1_STAND_ONLY=1
    export OPENLOONG_T1_CART_HAND=1
    export OPENLOONG_T1_VIS_FORCE=1
    timeout '"$SMOKE_SECONDS"' xvfb-run -a -s "-screen 0 1280x720x24" ./walk_wbc > /tmp/t1_force.log 2>&1 || true
    cat /tmp/t1_force.log
  ' > /tmp/t1_force.log 2>&1 || true

force_count=$(grep -ac "T1-FORCE" /tmp/t1_force.log || true)
hold_count=$(grep -ac "T1-HOLD" /tmp/t1_force.log || true)

echo "force_count=$force_count hold_count=$hold_count"
grep -aE "T1-LOCK|T1-FORCE|T1-HOLD" /tmp/t1_force.log | tail -20 || true

if [[ "$force_count" -lt 1 ]]; then
    echo "FAIL: expected at least one [T1-FORCE] log line" >&2
    exit 1
fi

echo "PASS: force log detected"
