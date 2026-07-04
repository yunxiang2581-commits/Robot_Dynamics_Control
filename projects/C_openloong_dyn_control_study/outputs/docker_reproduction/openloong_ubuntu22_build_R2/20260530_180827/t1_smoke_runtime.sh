#!/usr/bin/env bash
set -euo pipefail

export XDG_RUNTIME_DIR=/tmp/xdg
mkdir -p "$XDG_RUNTIME_DIR" /run_root/logs

cd /run_root/worktree/OpenLoong-Dyn-Control/build
rm -f /tmp/t1_base.log /tmp/t1_enabled.log

summarize_log() {
    local label="$1"
    local path="$2"
    local lines
    local hold
    local diag
    local lock
    local naninf

    lines=$(wc -l "$path" | awk '{print $1}')
    hold=$(grep -ac "T1-HOLD" "$path" || true)
    diag=$(grep -ac "T1-D1" "$path" || true)
    lock=$(grep -ac "T1-LOCK" "$path" || true)
    naninf=$(grep -aciE "nan|inf" "$path" || true)

    echo "${label}_lines=$lines ${label}_T1HOLD=$hold ${label}_T1D1=$diag ${label}_T1LOCK=$lock ${label}_naninf=$naninf"
}

echo "=== RUN1 baseline no env ==="
timeout 12 xvfb-run -a -s "-screen 0 1280x720x24" ./walk_wbc > /tmp/t1_base.log 2>&1 || true
cp /tmp/t1_base.log /run_root/logs/20260702_t1_base_runtime.log
summarize_log "base" /tmp/t1_base.log

echo "=== RUN2 T1 enabled, stand only ==="
OPENLOONG_T1_STAND_ONLY=1 OPENLOONG_T1_CART_HAND=1 timeout 12 xvfb-run -a -s "-screen 0 1280x720x24" ./walk_wbc > /tmp/t1_enabled.log 2>&1 || true
cp /tmp/t1_enabled.log /run_root/logs/20260702_t1_enabled_runtime.log
summarize_log "enabled" /tmp/t1_enabled.log

echo "--- enabled T1 first lines ---"
grep -aE "T1-HOLD|T1-D1|T1-LOCK" /tmp/t1_enabled.log | head -30 || true
echo "--- enabled T1 last lines ---"
grep -aE "T1-HOLD|T1-D1|T1-LOCK" /tmp/t1_enabled.log | tail -30 || true
