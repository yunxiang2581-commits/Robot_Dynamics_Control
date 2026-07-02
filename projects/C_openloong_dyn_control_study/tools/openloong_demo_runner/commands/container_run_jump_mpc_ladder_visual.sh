#!/usr/bin/env bash
set -euo pipefail

# Container-side launcher for the Project C jump_mpc ladder experiment.
# It only prepares the visual desktop and runs the already-built jump_mpc binary.

BUILD_DIR="/run_root/worktree/OpenLoong-Dyn-Control/build"
RECORD_DIR="/run_root/worktree/OpenLoong-Dyn-Control/record"
LOG_DIR="/demo_run/logs"

mkdir -p "$LOG_DIR" "$RECORD_DIR" /tmp/runtime-root
chmod 700 /tmp/runtime-root

export DISPLAY="${DISPLAY:-:99}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

if [[ -z "${OPENLOONG_HOLD_WINDOW_AFTER_END:-}" ]]; then
    export OPENLOONG_HOLD_WINDOW_AFTER_END=1
fi
if [[ -z "${OPENLOONG_HOLD_WINDOW_SECONDS:-}" ]]; then
    export OPENLOONG_HOLD_WINDOW_SECONDS=3600
fi

Xvfb "$DISPLAY" -screen 0 "${OPENLOONG_NOVNC_GEOMETRY:-1280x900x24}" +extension GLX +render -noreset \
    >"$LOG_DIR/00_xvfb.log" 2>&1 &
XVFB_PID=$!

for _ in $(seq 1 100); do
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

{
    echo "time: $(date)"
    echo "build_dir: $BUILD_DIR"
    echo "record_dir: $RECORD_DIR"
    echo "jump_z: ${OPENLOONG_JUMP_Z:-default}"
    echo "jump_acc_t: ${OPENLOONG_JUMP_ACC_T:-default}"
    echo "ankle_pitch_comp_gain: ${OPENLOONG_ANKLE_PITCH_COMP_GAIN:-default}"
    echo "landing_x_offset: ${OPENLOONG_LANDING_X_OFFSET:-default}"
    echo "novnc: http://localhost:6080/vnc.html?autoconnect=true&resize=remote"
} >"$LOG_DIR/01_ladder_visual_precheck.log"

cd "$BUILD_DIR"
if [[ ! -x ./jump_mpc ]]; then
    echo "jump_mpc executable not found: $BUILD_DIR/jump_mpc" >&2
    exit 1
fi

set +e
./jump_mpc >"$LOG_DIR/02_jump_mpc_runtime.log" 2>&1
STATUS=$?
set -e

{
    echo "jump_mpc_status: $STATUS"
    echo "time: $(date)"
} >"$LOG_DIR/03_jump_mpc_exit.log"

kill "$XVFB_PID" >/dev/null 2>&1 || true
sleep infinity
