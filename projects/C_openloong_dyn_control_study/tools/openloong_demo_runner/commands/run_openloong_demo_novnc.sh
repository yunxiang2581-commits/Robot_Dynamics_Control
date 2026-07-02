#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "$SCRIPT_DIR/openloong_demo_common.sh"

usage() {
    cat <<'EOF'
Usage:
  bash run_openloong_demo_novnc.sh --list
  bash run_openloong_demo_novnc.sh --build-image <demo_name>
  bash run_openloong_demo_novnc.sh [--hosted] [--name NAME] [--port PORT] [--hold-seconds SECONDS] <demo_name>

Options:
  --build-image         Build openloong-ubuntu22-novnc:local before running.
  --dry-run             Print the Docker command without running it.
  --hosted              Run the Docker container in the foreground; keep this shell alive.
  --list                List supported demo names.
  --name NAME           Use a specific Docker container name.
  --port PORT           Bind noVNC to localhost:PORT. Default: 6080.
  --hold-seconds N      Keep the browser desktop alive after demo exit. Default: 600.
  -h, --help            Show this help.
EOF
}

BUILD_IMAGE=0
DRY_RUN=0
HOSTED=0
DEMO_NAME=""
CONTAINER_NAME_OVERRIDE=""
NOVNC_IMAGE_NAME="${OPENLOONG_NOVNC_IMAGE_NAME:-openloong-ubuntu22-novnc:local}"
NOVNC_PORT="${OPENLOONG_NOVNC_PORT:-6080}"
NOVNC_HOLD_SECONDS="${OPENLOONG_NOVNC_HOLD_SECONDS:-600}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --build-image)
            BUILD_IMAGE=1
            shift
            ;;
        --dry-run)
            DRY_RUN=1
            shift
            ;;
        --hosted)
            HOSTED=1
            shift
            ;;
        --list)
            openloong_list_demos
            exit 0
            ;;
        --name)
            shift
            [[ $# -gt 0 ]] || { echo "Missing value for --name" >&2; exit 1; }
            CONTAINER_NAME_OVERRIDE="$1"
            shift
            ;;
        --port)
            shift
            [[ $# -gt 0 ]] || { echo "Missing value for --port" >&2; exit 1; }
            NOVNC_PORT="$1"
            shift
            ;;
        --hold-seconds)
            shift
            [[ $# -gt 0 ]] || { echo "Missing value for --hold-seconds" >&2; exit 1; }
            NOVNC_HOLD_SECONDS="$1"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            if [[ -n "$DEMO_NAME" ]]; then
                echo "Only one demo name may be provided." >&2
                usage >&2
                exit 1
            fi
            DEMO_NAME="$1"
            shift
            ;;
    esac
done

if [[ -z "$DEMO_NAME" ]]; then
    echo "Missing demo name." >&2
    usage >&2
    exit 1
fi

if [[ ! "$NOVNC_PORT" =~ ^[0-9]+$ ]] || [[ "$NOVNC_PORT" -lt 1 ]] || [[ "$NOVNC_PORT" -gt 65535 ]]; then
    echo "Invalid --port value: $NOVNC_PORT" >&2
    exit 1
fi

if [[ ! "$NOVNC_HOLD_SECONDS" =~ ^[0-9]+$ ]]; then
    echo "Invalid --hold-seconds value: $NOVNC_HOLD_SECONDS" >&2
    exit 1
fi

if ! openloong_resolve_demo "$DEMO_NAME"; then
    echo "Unknown demo: $DEMO_NAME" >&2
    openloong_list_demos >&2
    exit 1
fi

openloong_init_build_context
BUILD_ROOT="$OPENLOONG_BUILD_ROOT"
RUNS_ROOT="$OPENLOONG_RUNS_ROOT"
TOOL_ROOT="$OPENLOONG_TOOL_ROOT"
NOVNC_DOCKERFILE="$TOOL_ROOT/novnc/Dockerfile"
CONTAINER_SCRIPT="$TOOL_ROOT/commands/container_run_openloong_demo_novnc.sh"

openloong_require_path "$BUILD_ROOT" "Build root"
openloong_require_path "$OPENLOONG_BUILD_WORKTREE" "Build worktree"
openloong_require_path "$OPENLOONG_BUILD_DIR" "Build directory"
openloong_require_path "$NOVNC_DOCKERFILE" "noVNC Dockerfile"
openloong_require_path "$CONTAINER_SCRIPT" "noVNC container script"

if [[ "$BUILD_IMAGE" -eq 1 ]] || ! docker image inspect "$NOVNC_IMAGE_NAME" >/dev/null 2>&1; then
    docker build -t "$NOVNC_IMAGE_NAME" -f "$NOVNC_DOCKERFILE" "$TOOL_ROOT/novnc"
fi

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
RUN_ROOT="$RUNS_ROOT/${TIMESTAMP}_${DEMO_NAME}_novnc"
CONTAINER_NAME="${CONTAINER_NAME_OVERRIDE:-projectc_${DEMO_NAME}_novnc_${TIMESTAMP}}"
mkdir -p "$RUN_ROOT/logs" "$RUN_ROOT/runtime_record" "$RUN_ROOT/commands"

docker_args=(
    docker run
    --rm
    --name "$CONTAINER_NAME"
    --label projectc.openloong.runner=novnc
    --label "projectc.openloong.demo=$DEMO_NAME"
    -p "127.0.0.1:$NOVNC_PORT:6080"
    -e "OPENLOONG_NOVNC_HOLD_SECONDS=$NOVNC_HOLD_SECONDS"
    --mount "type=bind,source=$BUILD_ROOT,target=/run_root,readonly"
    --mount "type=bind,source=$RUN_ROOT/runtime_record,target=/run_root/worktree/OpenLoong-Dyn-Control/record"
    --mount "type=bind,source=$RUN_ROOT,target=/demo_run"
    --mount "type=bind,source=$TOOL_ROOT,target=/tool_root,readonly"
    "$NOVNC_IMAGE_NAME"
    bash /tool_root/commands/container_run_openloong_demo_novnc.sh "$OPENLOONG_DEMO_EXECUTABLE" "$DEMO_NAME"
)

if [[ "$HOSTED" -eq 0 ]]; then
    docker_args=(docker run -d "${docker_args[@]:2}")
fi

printf '%q ' "${docker_args[@]}" > "$RUN_ROOT/commands/run_command.sh"
printf '\n' >> "$RUN_ROOT/commands/run_command.sh"
chmod +x "$RUN_ROOT/commands/run_command.sh"

if [[ "$DRY_RUN" -eq 1 ]]; then
    echo "Demo: $DEMO_NAME"
    echo "Executable: $OPENLOONG_DEMO_EXECUTABLE"
    echo "Image: $NOVNC_IMAGE_NAME"
    echo "Run root: $RUN_ROOT"
    echo "Container name: $CONTAINER_NAME"
    echo "noVNC URL: http://localhost:$NOVNC_PORT/vnc.html?autoconnect=true&resize=remote"
    echo "Hold seconds: $NOVNC_HOLD_SECONDS"
    echo "Docker command: $(printf '%q ' "${docker_args[@]}")"
    exit 0
fi

mapfile -t existing_novnc_containers < <(docker ps -aq --filter label=projectc.openloong.runner=novnc)
if [[ "${#existing_novnc_containers[@]}" -gt 0 ]]; then
    docker rm -f "${existing_novnc_containers[@]}" >/dev/null
fi

docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true

cat > "$RUN_ROOT/run_info.txt" <<EOF
demo: $DEMO_NAME
executable: $OPENLOONG_DEMO_EXECUTABLE
image: $NOVNC_IMAGE_NAME
run_root: $RUN_ROOT
container_name: $CONTAINER_NAME
novnc_url: http://localhost:$NOVNC_PORT/vnc.html?autoconnect=true&resize=remote
hold_seconds: $NOVNC_HOLD_SECONDS
stop_command: docker rm -f $CONTAINER_NAME
EOF

echo "Demo: $DEMO_NAME"
echo "Container name: $CONTAINER_NAME"
echo "Run root: $RUN_ROOT"
echo "noVNC URL:"
echo "  http://localhost:$NOVNC_PORT/vnc.html?autoconnect=true&resize=remote"
echo "Stop command:"
echo "  docker rm -f $CONTAINER_NAME"

if [[ "$HOSTED" -eq 1 ]]; then
    echo "Hosted mode: this shell will stay attached to keep WSL/Docker alive."
fi

"${docker_args[@]}"
