#!/usr/bin/env bash
set -euo pipefail

ROOT=/mnt/d/project/Robot_Dynamics_Control
BUILD_ROOT="$ROOT/projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827"
RUNS_ROOT="$ROOT/projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_demo_runs"
TOOL_ROOT="$ROOT/projects/C_openloong_dyn_control_study/tools/openloong_demo_runner"
IMAGE_NAME="${OPENLOONG_IMAGE_NAME:-openloong-ubuntu22-build:local}"
SMOKE_SECONDS="${1:-30}"
TS=$(date +%Y%m%d_%H%M%S)
RUN_ROOT="$RUNS_ROOT/${TS}_walk_wbc_t5_d2_forward_palm_up_visual_wslg"
CONTAINER_NAME="projectc_walk_wbc_t5_d2_visual_$TS"

mkdir -p "$RUN_ROOT/commands" "$RUN_ROOT/logs" "$RUN_ROOT/runtime_record"

cat > "$RUN_ROOT/run_info.txt" <<EOF
mode: foreground visual smoke
demo: walk_wbc T5-D2 in-place stepping + forward palm-up 6-DoF right-hand posture hold
image: $IMAGE_NAME
build_root: $BUILD_ROOT
run_root: $RUN_ROOT
container_name: $CONTAINER_NAME
env: OPENLOONG_T4_IN_PLACE=1 OPENLOONG_T5_POSTURE=1 OPENLOONG_T5_FORWARD_PALM_UP=1
smoke_seconds: $SMOKE_SECONDS
EOF

docker_args=(timeout "${SMOKE_SECONDS}s" docker run --rm)
docker_args+=(--name "$CONTAINER_NAME")

if [[ -e /dev/dxg && -d /usr/lib/wsl/lib ]]; then
    docker_args+=(--device /dev/dxg)
    docker_args+=(-v /usr/lib/wsl:/usr/lib/wsl:ro)
    docker_args+=(-e "LD_LIBRARY_PATH=/usr/lib/wsl/lib:${LD_LIBRARY_PATH:-}")
    docker_args+=(-e LIBGL_ALWAYS_SOFTWARE=0)
fi

docker_args+=(
    -e "DISPLAY=${DISPLAY:-}"
    -e "WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-}"
    -e "XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-}"
    -e "PULSE_SERVER=${PULSE_SERVER:-}"
    -e QT_X11_NO_MITSHM=1
    -e OPENLOONG_T4_IN_PLACE=1
    -e OPENLOONG_T5_POSTURE=1
    -e OPENLOONG_T5_FORWARD_PALM_UP=1
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
)

if [[ -d /mnt/wslg ]]; then
    docker_args+=(-v /mnt/wslg:/mnt/wslg:rw)
fi

docker_args+=(
    --mount "type=bind,source=$BUILD_ROOT,target=/run_root,readonly"
    --mount "type=bind,source=$RUN_ROOT/runtime_record,target=/run_root/worktree/OpenLoong-Dyn-Control/record"
    --mount "type=bind,source=$RUN_ROOT,target=/demo_run"
    --mount "type=bind,source=$TOOL_ROOT,target=/tool_root,readonly"
    "$IMAGE_NAME"
    bash /tool_root/commands/container_run_openloong_demo.sh walk_wbc walk_wbc
)

printf '%q ' "${docker_args[@]}" > "$RUN_ROOT/commands/run_command.sh"
printf '\n' >> "$RUN_ROOT/commands/run_command.sh"
chmod +x "$RUN_ROOT/commands/run_command.sh"

echo "Run root: $RUN_ROOT"
echo "Container: $CONTAINER_NAME"
echo "Smoke seconds: $SMOKE_SECONDS"

set +e
"${docker_args[@]}"
status=$?
set -e

if [[ "$status" -eq 124 ]]; then
    echo "Visual smoke reached timeout: ${SMOKE_SECONDS}s"
    status=0
fi

echo "status=$status"
echo "Logs dir: $RUN_ROOT/logs"
echo "Record dir: $RUN_ROOT/runtime_record"
exit "$status"
