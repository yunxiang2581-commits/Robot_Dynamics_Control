#!/usr/bin/env bash
export XDG_RUNTIME_DIR=/tmp/xdg && mkdir -p "$XDG_RUNTIME_DIR"
cd /run_root/worktree/OpenLoong-Dyn-Control/build
echo "=== RUN1 baseline (no env) ==="
xvfb-run -a -s "-screen 0 1280x720x24" bash -c "timeout 12 ./walk_wbc > /tmp/r1.log 2>&1; true"
echo "run1_lines=$(wc -l < /tmp/r1.log) run1_T1HOLD=$(grep -ac T1-HOLD /tmp/r1.log)"
echo "=== RUN2 T1 enabled ==="
OPENLOONG_T1_CART_HAND=1 xvfb-run -a -s "-screen 0 1280x720x24" bash -c "timeout 12 ./walk_wbc > /tmp/r2.log 2>&1; true"
echo "run2_lines=$(wc -l < /tmp/r2.log) run2_T1HOLD=$(grep -ac T1-HOLD /tmp/r2.log)"
echo "--- drift first3 ---"; grep -a T1-HOLD /tmp/r2.log | head -3
echo "--- drift last3 ---";  grep -a T1-HOLD /tmp/r2.log | tail -3
echo "run2_nan=$(grep -ac -iE "nan|inf" /tmp/r2.log)"
