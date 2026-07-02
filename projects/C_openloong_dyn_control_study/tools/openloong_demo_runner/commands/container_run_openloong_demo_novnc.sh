#!/usr/bin/env bash
set -euo pipefail

# noVNC 容器内启动脚本。
# 输入：
#   $1: build 目录下的可执行文件名
#   $2: demo 名
# 输出：
#   :99 虚拟 X11 桌面 + 5900 VNC + 6080 noVNC 浏览器端口。
#
# 数学/控制逻辑：
#   本脚本只改变显示通道。OpenLoong demo 的控制器、MuJoCo 模型和参数不变。

if [[ $# -ne 2 ]]; then
    echo "Usage: bash container_run_openloong_demo_novnc.sh <executable_name> <demo_name>" >&2
    exit 1
fi

EXECUTABLE_NAME="$1"
DEMO_NAME="$2"
BUILD_DIR="/run_root/worktree/OpenLoong-Dyn-Control/build"
RECORD_DIR="/run_root/worktree/OpenLoong-Dyn-Control/record"
LOG_DIR="/demo_run/logs"
mkdir -p "$LOG_DIR" "$RECORD_DIR"

export DISPLAY=:99
export LIBGL_ALWAYS_SOFTWARE=1
NOVNC_HOLD_SECONDS="${OPENLOONG_NOVNC_HOLD_SECONDS:-600}"
EXIT_TRAP_LOG="$LOG_DIR/99_container_exit.log"

ignore_external_signal() {
    echo "Ignored external signal at $(date); use docker rm -f to stop this container." \
        >>"$EXIT_TRAP_LOG"
}

if [[ "${OPENLOONG_NOVNC_TRACE:-0}" == "1" ]]; then
    TRACE_LOG="$LOG_DIR/99_container_trace.log"
    exec 9>"$TRACE_LOG"
    BASH_XTRACEFD=9
    PS4='+ ${BASH_SOURCE##*/}:${LINENO}: '
    set -x
fi

if [[ "$DEMO_NAME" == "jump_mpc" && -z "${OPENLOONG_HOLD_WINDOW_AFTER_END:-}" ]]; then
    export OPENLOONG_HOLD_WINDOW_AFTER_END=1
fi
if [[ "$DEMO_NAME" == "jump_mpc" && -z "${OPENLOONG_HOLD_WINDOW_SECONDS:-}" ]]; then
    export OPENLOONG_HOLD_WINDOW_SECONDS=600
fi

Xvfb "$DISPLAY" -screen 0 "${OPENLOONG_NOVNC_GEOMETRY:-1280x900x24}" +extension GLX +render -noreset \
    >"$LOG_DIR/00_xvfb.log" 2>&1 &
XVFB_PID=$!
trap ignore_external_signal HUP INT QUIT TERM USR1 USR2

for _ in $(seq 1 50); do
    if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
        break
    fi
    sleep 0.1
done

mkdir -p "$HOME/.fluxbox"
cat >"$HOME/.fluxbox/init" <<'EOF'
session.screen0.rootCommand: /bin/true
session.screen0.toolbar.visible: false
EOF
fluxbox >"$LOG_DIR/00_fluxbox.log" 2>&1 &
x11vnc -display "$DISPLAY" -forever -shared -nopw -listen 0.0.0.0 -rfbport 5900 \
    >"$LOG_DIR/00_x11vnc.log" 2>&1 &
websockify --web=/usr/share/novnc/ 0.0.0.0:6080 localhost:5900 \
    >"$LOG_DIR/00_websockify.log" 2>&1 &

cd "$BUILD_DIR"
if [[ ! -x "./$EXECUTABLE_NAME" ]]; then
    echo "Executable not found or not executable: $BUILD_DIR/$EXECUTABLE_NAME" >&2
    exit 1
fi

{
    echo "pwd: $(pwd)"
    echo "demo: $DEMO_NAME"
    echo "executable: $EXECUTABLE_NAME"
    echo "record dir: $RECORD_DIR"
    echo "display: $DISPLAY"
    echo "novnc: http://localhost:6080/vnc.html?autoconnect=true&resize=remote"
    echo "glxinfo:"
    glxinfo -B || true
    echo "target:"
    ls -lah "./$EXECUTABLE_NAME"
    echo "visual hold:"
    echo "OPENLOONG_HOLD_WINDOW_AFTER_END=${OPENLOONG_HOLD_WINDOW_AFTER_END:-}"
    echo "OPENLOONG_HOLD_WINDOW_SECONDS=${OPENLOONG_HOLD_WINDOW_SECONDS:-}"
    echo "OPENLOONG_NOVNC_HOLD_SECONDS=$NOVNC_HOLD_SECONDS"
} 2>&1 | tee "$LOG_DIR/01_${DEMO_NAME}_novnc_precheck.log"

RUNTIME_LOG="$LOG_DIR/02_${DEMO_NAME}_runtime.log"
DEMO_PID_FILE="$LOG_DIR/02_${DEMO_NAME}.pid"
: >"$RUNTIME_LOG"

set +e
(
    set +e
    setsid stdbuf -oL -eL "./$EXECUTABLE_NAME" >"$RUNTIME_LOG" 2>&1 &
    demo_pid=$!
    echo "$demo_pid" >"$DEMO_PID_FILE"
    wait "$demo_pid"
    demo_status=$?
    {
        echo "Demo exited with status: $demo_status"
        echo "time: $(date)"
    } >"$LOG_DIR/03_${DEMO_NAME}_exit.log"
    exit 0
) &
DEMO_WATCH_PID=$!

set -e

if [[ "${OPENLOONG_NOVNC_STREAM_LOGS:-0}" == "1" ]]; then
    tail -n +1 -f "$RUNTIME_LOG" &
fi

echo "noVNC desktop is ready; use docker rm -f to stop this container." \
    | tee "$LOG_DIR/04_${DEMO_NAME}_novnc_ready.log"

trap '' HUP INT QUIT TERM USR1 USR2
exec sleep infinity
