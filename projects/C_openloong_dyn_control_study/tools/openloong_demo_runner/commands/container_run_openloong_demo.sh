#!/usr/bin/env bash
set -euo pipefail

# 容器内通用运行脚本。
# 输入：
#   $1: build 目录下的可执行文件名
#   $2: demo 名
# 输出：
#   /demo_run/logs/* 预检查与运行日志

if [[ $# -ne 2 ]]; then
    echo "Usage: bash container_run_openloong_demo.sh <executable_name> <demo_name>" >&2
    exit 1
fi

EXECUTABLE_NAME="$1"
DEMO_NAME="$2"
BUILD_DIR="/run_root/worktree/OpenLoong-Dyn-Control/build"
RECORD_DIR="/run_root/worktree/OpenLoong-Dyn-Control/record"
PRECHECK_LOG="/demo_run/logs/01_${DEMO_NAME}_precheck.log"
RUNTIME_LOG="/demo_run/logs/02_${DEMO_NAME}_runtime.log"

cd "$BUILD_DIR"

if [[ ! -x "./$EXECUTABLE_NAME" ]]; then
    echo "Executable not found or not executable: $BUILD_DIR/$EXECUTABLE_NAME" >&2
    exit 1
fi

if [[ "$DEMO_NAME" == "jump_mpc" && -z "${OPENLOONG_HOLD_WINDOW_AFTER_END:-}" ]]; then
    export OPENLOONG_HOLD_WINDOW_AFTER_END=1
fi
if [[ "$DEMO_NAME" == "jump_mpc" && -z "${OPENLOONG_HOLD_WINDOW_SECONDS:-}" ]]; then
    export OPENLOONG_HOLD_WINDOW_SECONDS=600
fi

{
echo "pwd: $(pwd)"
echo "demo: $DEMO_NAME"
echo "executable: $EXECUTABLE_NAME"
echo "start time: $(date)"
echo "target:"
ls -lah "./$EXECUTABLE_NAME"
echo
echo "nvidia-smi:"
nvidia-smi || true
echo
echo "glxinfo:"
glxinfo -B || true
echo
echo "ldd:"
ldd "./$EXECUTABLE_NAME" || true
echo
echo "rpath/runpath:"
readelf -d "./$EXECUTABLE_NAME" | grep -E "RPATH|RUNPATH" || true
echo
echo "record dir:"
ls -lah "$RECORD_DIR" || true
echo
echo "visual hold:"
echo "OPENLOONG_HOLD_WINDOW_AFTER_END=${OPENLOONG_HOLD_WINDOW_AFTER_END:-}"
echo "OPENLOONG_HOLD_WINDOW_SECONDS=${OPENLOONG_HOLD_WINDOW_SECONDS:-}"
} 2>&1 | tee "$PRECHECK_LOG"

"./$EXECUTABLE_NAME" 2>&1 | tee "$RUNTIME_LOG"
