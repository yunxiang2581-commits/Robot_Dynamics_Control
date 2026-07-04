#!/usr/bin/env bash
set -euo pipefail

BUILD_ROOT=/mnt/d/project/Robot_Dynamics_Control/projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827
MODE="${1:-LINE}"
SMOKE_SECONDS="${2:-60}"
MAX_ERR_LIMIT_MM="${3:-25}"
RMS_ERR_LIMIT_MM="${4:-12}"
LOG="/tmp/t1_traj_${MODE}.log"

echo "=== RUN T1 trajectory smoke: mode=${MODE} ==="
docker run --rm \
  -v "$BUILD_ROOT:/run_root" \
  openloong-ubuntu22-build:xvfb \
  bash -lc '
    set -euo pipefail
    export XDG_RUNTIME_DIR=/tmp/xdg
    mkdir -p "$XDG_RUNTIME_DIR"
    cd /run_root/worktree/OpenLoong-Dyn-Control/build
    OPENLOONG_T1_STAND_ONLY=1 \
    OPENLOONG_T1_CART_HAND=1 \
    OPENLOONG_T1_HAND_TRAJ='"$MODE"' \
    timeout '"$SMOKE_SECONDS"' xvfb-run -a -s "-screen 0 1280x720x24" ./walk_wbc > /tmp/t1_traj.log 2>&1 || true
    cat /tmp/t1_traj.log
  ' > "$LOG" 2>&1 || true

traj_count=$(grep -ac "T1-TRAJ" "$LOG" || true)
lock_count=$(grep -ac "T1-LOCK" "$LOG" || true)
nan_count=$(grep -aciE '(^|[^A-Za-z])(-?nan|\+?nan|-?inf|\+?inf)([^A-Za-z]|$)' "$LOG" || true)

metrics=$(awk '
  /T1-TRAJ/ {
    if (match($0, /err_mm=([0-9.]+)/, m)) {
      val = m[1] + 0.0
      n += 1
      sumsq += val * val
      if (val > maxv) {
        maxv = val
      }
    }
  }
  END {
    if (n > 0) {
      printf "max_err_mm=%.6f rms_err_mm=%.6f", maxv, sqrt(sumsq / n)
    } else {
      printf "max_err_mm=nan rms_err_mm=nan"
    }
  }
' "$LOG")

max_err_mm=$(echo "$metrics" | awk '{print $1}' | cut -d= -f2)
rms_err_mm=$(echo "$metrics" | awk '{print $2}' | cut -d= -f2)

echo "traj_count=$traj_count lock_count=$lock_count nan_count=$nan_count $metrics"
echo "--- tail ---"
grep -aE "T1-LOCK|T1-TRAJ|T1-HOLD" "$LOG" | tail -30 || true

if [[ "$traj_count" -lt 3 ]]; then
  echo "FAIL: expected at least 3 [T1-TRAJ] log lines" >&2
  exit 1
fi

if [[ "$nan_count" -gt 0 ]]; then
  echo "FAIL: found numeric nan/inf candidates" >&2
  exit 1
fi

if ! awk "BEGIN { exit !($max_err_mm <= $MAX_ERR_LIMIT_MM) }"; then
  echo "FAIL: expected max_err_mm <= $MAX_ERR_LIMIT_MM, got $max_err_mm" >&2
  exit 1
fi

if ! awk "BEGIN { exit !($rms_err_mm <= $RMS_ERR_LIMIT_MM) }"; then
  echo "FAIL: expected rms_err_mm <= $RMS_ERR_LIMIT_MM, got $rms_err_mm" >&2
  exit 1
fi

cp "$LOG" "$BUILD_ROOT/logs/20260703_t1_traj_${MODE}.log"
echo "PASS: mode=${MODE} trajectory tracked within limits"
