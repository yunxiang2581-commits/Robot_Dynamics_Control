#!/usr/bin/env bash
set -euo pipefail

# Project C jump_mpc 阶梯扫描：容器内单次运行脚本。
# 输入：环境变量 OPENLOONG_JUMP_Z / OPENLOONG_JUMP_ACC_T 等。
# 输出：/run_root/worktree/OpenLoong-Dyn-Control/record 下的 datalog.log。
# 数学/控制逻辑：本脚本只准备 Xvfb 显示环境并运行 jump_mpc，不修改控制器。

BUILD_DIR="/run_root/worktree/OpenLoong-Dyn-Control/build"
RECORD_DIR="/run_root/worktree/OpenLoong-Dyn-Control/record"
LOG_DIR="/demo_run/logs"
RUN_TIMEOUT_SECONDS="${OPENLOONG_RUN_TIMEOUT_SECONDS:-95}"

mkdir -p "$LOG_DIR" "$RECORD_DIR" /tmp/runtime-root
chmod 700 /tmp/runtime-root

export DISPLAY="${DISPLAY:-:99}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export OPENLOONG_HOLD_WINDOW_AFTER_END="${OPENLOONG_HOLD_WINDOW_AFTER_END:-0}"

rm -f "$RECORD_DIR/datalog.log" "$RECORD_DIR/matlabReadDataScript.txt"

Xvfb "$DISPLAY" -screen 0 "${OPENLOONG_NOVNC_GEOMETRY:-1280x900x24}" +extension GLX +render -noreset \
    >"$LOG_DIR/00_xvfb.log" 2>&1 &
XVFB_PID=$!

for _ in $(seq 1 100); do
    if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
        break
    fi
    sleep 0.1
done

{
    echo "time: $(date)"
    echo "build_dir: $BUILD_DIR"
    echo "record_dir: $RECORD_DIR"
    echo "jump_z: ${OPENLOONG_JUMP_Z:-default}"
    echo "jump_acc_t: ${OPENLOONG_JUMP_ACC_T:-default}"
    echo "run_timeout_seconds: $RUN_TIMEOUT_SECONDS"
} >"$LOG_DIR/01_ladder_once_precheck.log"

cd "$BUILD_DIR"
if [[ ! -x ./jump_mpc ]]; then
    echo "jump_mpc executable not found: $BUILD_DIR/jump_mpc" >&2
    exit 1
fi

set +e
timeout "${RUN_TIMEOUT_SECONDS}s" ./jump_mpc >"$LOG_DIR/02_jump_mpc_runtime.log" 2>&1
STATUS=$?
set -e

{
    echo "jump_mpc_status: $STATUS"
    echo "time: $(date)"
} >"$LOG_DIR/03_jump_mpc_exit.log"

kill "$XVFB_PID" >/dev/null 2>&1 || true
exit "$STATUS"
