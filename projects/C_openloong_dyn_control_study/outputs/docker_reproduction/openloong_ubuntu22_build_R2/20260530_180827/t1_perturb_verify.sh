#!/usr/bin/env bash
set -euo pipefail

BUILD_ROOT=/mnt/d/project/Robot_Dynamics_Control/projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827
SMOKE_SECONDS="${1:-30}"

echo "=== RUN T1 perturb smoke ==="
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
    OPENLOONG_T1_PERTURB_X=0.05 \
    timeout '"$SMOKE_SECONDS"' xvfb-run -a -s "-screen 0 1280x720x24" ./walk_wbc > /tmp/t1_perturb.log 2>&1 || true
    cat /tmp/t1_perturb.log
  ' > /tmp/t1_perturb.log 2>&1 || true

hold_count=$(grep -ac "T1-HOLD" /tmp/t1_perturb.log || true)
pert_count=$(grep -ac "T1-PERT" /tmp/t1_perturb.log || true)
lock_count=$(grep -ac "T1-LOCK" /tmp/t1_perturb.log || true)
max_post_pert_drift_mm=$(awk '
    /T1-PERT/ { seen_pert=1; next }
    seen_pert && /T1-HOLD/ {
        if (match($0, /drift_mm=([0-9.]+)/, m)) {
            val = m[1] + 0.0
            if (val > max_val) {
                max_val = val
            }
        }
    }
    END {
        if (seen_pert) {
            printf "%.6f", max_val
        } else {
            printf "nan"
        }
    }
' /tmp/t1_perturb.log)

echo "hold_count=$hold_count pert_count=$pert_count lock_count=$lock_count max_post_pert_drift_mm=$max_post_pert_drift_mm"
echo "--- tail ---"
grep -aE "T1-LOCK|T1-PERT|T1-HOLD" /tmp/t1_perturb.log | tail -20 || true

if [[ "$pert_count" -lt 1 ]]; then
    echo "FAIL: expected at least one [T1-PERT] log line" >&2
    exit 1
fi

if ! awk "BEGIN { exit !($max_post_pert_drift_mm <= 10.0) }"; then
    echo "FAIL: expected max_post_pert_drift_mm <= 10.0, got $max_post_pert_drift_mm" >&2
    exit 1
fi

echo "PASS: perturb log detected and drift stayed within 10 mm"
