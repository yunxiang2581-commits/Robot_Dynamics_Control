#!/usr/bin/env bash
set -euo pipefail

RUN_ROOT=/mnt/d/project/Robot_Dynamics_Control/projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827
SMOKE_SECONDS="${1:-12}"
mkdir -p "$RUN_ROOT/logs"

docker run --rm \
  -v "$RUN_ROOT:/run_root" \
  openloong-ubuntu22-build:xvfb \
  bash -lc '
    set -euo pipefail
    export XDG_RUNTIME_DIR=/tmp/xdg
    mkdir -p "$XDG_RUNTIME_DIR"
    cd /run_root/worktree/OpenLoong-Dyn-Control/build
    OPENLOONG_T1_STAND_ONLY=1 \
    OPENLOONG_T1_CART_HAND=1 \
    timeout '"$SMOKE_SECONDS"' xvfb-run -a -s "-screen 0 1280x720x24" ./walk_wbc \
      > /run_root/logs/20260703_hand_task_refactor_t1_smoke.log 2>&1 || true
  '

LOG="$RUN_ROOT/logs/20260703_hand_task_refactor_t1_smoke.log"
echo "log=$LOG"
echo "lines=$(wc -l < "$LOG")"
echo "hold_count=$(grep -ac "T1-HOLD" "$LOG" || true)"
echo "lock_count=$(grep -ac "T1-LOCK" "$LOG" || true)"
echo "diag_count=$(grep -ac "T1-D1" "$LOG" || true)"
echo "naninf_count=$(grep -aciE "nan|inf" "$LOG" || true)"
grep -aE "T1-HOLD|T1-LOCK|T1-D1" "$LOG" | head -40 || true
grep -aE "T1-HOLD|T1-LOCK|T1-D1" "$LOG" | tail -40 || true
