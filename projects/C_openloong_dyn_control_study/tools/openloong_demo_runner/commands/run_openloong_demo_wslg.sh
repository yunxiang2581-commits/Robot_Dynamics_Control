#!/usr/bin/env bash
set -euo pipefail

# WSLg 可视化 demo 启动器。
# 作用：
# 1. 覆盖 openloong_demo_common.sh 中登记的所有 demo。
# 2. 默认使用 WSLg/Mesa 可视化路径，不强制依赖 NVIDIA GPU。
# 3. 每次运行都创建独立输出目录，保存命令、日志和 datalog。
#
# 输入：
#   demo 名称，例如 walk_wbc、walk_wbc_staircase、walk_mpc_wbc。
# 输出：
#   outputs/docker_reproduction/openloong_demo_runs/<timestamp>_<demo>/
#
# 数学/控制逻辑：
#   本脚本只改变启动方式，不修改 MuJoCo、WBC、MPC 或控制器参数。

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "$SCRIPT_DIR/openloong_demo_common.sh"

usage() {
    cat <<'EOF'
Usage:
  bash run_openloong_demo_wslg.sh --list
  bash run_openloong_demo_wslg.sh --dry-run [options] <demo_name>
  bash run_openloong_demo_wslg.sh [options] <demo_name>

Options:
  --detach              Run container in background and print a stop command.
  --gpu                 Add NVIDIA Docker GPU flags. Default is WSLg/Mesa only.
  --no-wslg-gpu         Do not pass WSLg /dev/dxg and D3D12 libraries.
  --smoke-seconds SEC   Stop the run after SEC seconds.
  --name NAME           Use a specific Docker container name.
  -h, --help            Show this help.

Environment overrides:
  OPENLOONG_IMAGE_NAME
  OPENLOONG_BUILD_ROOT
  OPENLOONG_RUNS_ROOT
EOF
}

quote_arg() {
    printf '%q' "$1"
}

join_command() {
    local out=""
    local arg
    for arg in "$@"; do
        if [[ -n "$out" ]]; then
            out+=" "
        fi
        out+="$(quote_arg "$arg")"
    done
    printf '%s\n' "$out"
}

require_path() {
    openloong_require_path "$1" "$2" || exit 1
}

require_positive_integer() {
    local value="$1"
    local label="$2"
    if [[ ! "$value" =~ ^[1-9][0-9]*$ ]]; then
        echo "$label must be a positive integer: $value" >&2
        exit 1
    fi
}

DRY_RUN=0
DETACH=0
ENABLE_GPU=0
ENABLE_WSLG_GPU=1
SMOKE_SECONDS=""
DEMO_NAME=""
CONTAINER_NAME_OVERRIDE=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --list)
            openloong_list_demos
            exit 0
            ;;
        --dry-run)
            DRY_RUN=1
            shift
            ;;
        --detach)
            DETACH=1
            shift
            ;;
        --gpu)
            ENABLE_GPU=1
            shift
            ;;
        --no-wslg-gpu)
            ENABLE_WSLG_GPU=0
            shift
            ;;
        --smoke-seconds)
            shift
            [[ $# -gt 0 ]] || { echo "Missing value for --smoke-seconds" >&2; exit 1; }
            require_positive_integer "$1" "Smoke seconds"
            SMOKE_SECONDS="$1"
            shift
            ;;
        --name)
            shift
            [[ $# -gt 0 ]] || { echo "Missing value for --name" >&2; exit 1; }
            CONTAINER_NAME_OVERRIDE="$1"
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

if ! openloong_resolve_demo "$DEMO_NAME"; then
    echo "Unknown demo: $DEMO_NAME" >&2
    echo "Available demos:" >&2
    openloong_list_demos >&2
    exit 1
fi

DEMO_EXECUTABLE="$OPENLOONG_DEMO_EXECUTABLE"
BUILD_ROOT="$OPENLOONG_BUILD_ROOT"
RUNS_ROOT="$OPENLOONG_RUNS_ROOT"
IMAGE_NAME="$OPENLOONG_IMAGE_NAME"
openloong_init_build_context
BUILD_WORKTREE="$OPENLOONG_BUILD_WORKTREE"
BUILD_DIR="$OPENLOONG_BUILD_DIR"
TOOL_ROOT="$OPENLOONG_TOOL_ROOT"
CONTAINER_SCRIPT="$TOOL_ROOT/commands/container_run_openloong_demo.sh"

require_path "$BUILD_ROOT" "Build root"
require_path "$BUILD_WORKTREE" "Build worktree"
require_path "$BUILD_DIR" "Build directory"
require_path "$CONTAINER_SCRIPT" "Container runner script"
require_path "/tmp/.X11-unix" "WSLg X11 socket directory"
if [[ -d "/mnt/wslg" ]]; then
    WSLG_MOUNT_EXISTS=1
else
    WSLG_MOUNT_EXISTS=0
fi
if [[ "$ENABLE_WSLG_GPU" -eq 1 && -e "/dev/dxg" && -d "/usr/lib/wsl/lib" ]]; then
    WSLG_GPU_AVAILABLE=1
else
    WSLG_GPU_AVAILABLE=0
fi

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
RUN_ROOT="$RUNS_ROOT/${TIMESTAMP}_${DEMO_NAME}_wslg"
CONTAINER_NAME="${CONTAINER_NAME_OVERRIDE:-projectc_${DEMO_NAME}_wslg_${TIMESTAMP}}"
MODE="foreground"
if [[ "$DETACH" -eq 1 ]]; then
    MODE="detached"
fi

docker_args=(docker run --rm)
if [[ -n "$SMOKE_SECONDS" ]]; then
    docker_args=(timeout "${SMOKE_SECONDS}s" "${docker_args[@]}")
fi

if [[ "$DETACH" -eq 1 ]]; then
    docker_args+=(-d)
else
    docker_args+=(-it)
fi

docker_args+=(--name "$CONTAINER_NAME")

if [[ "$ENABLE_GPU" -eq 1 ]]; then
    docker_args+=(--gpus all)
    docker_args+=(-e NVIDIA_DRIVER_CAPABILITIES=all)
    docker_args+=(-e __GLX_VENDOR_LIBRARY_NAME=nvidia)
fi

if [[ "$WSLG_GPU_AVAILABLE" -eq 1 ]]; then
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
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
)

if [[ "$WSLG_MOUNT_EXISTS" -eq 1 ]]; then
    docker_args+=(-v /mnt/wslg:/mnt/wslg:rw)
fi

docker_args+=(
    --mount "type=bind,source=$BUILD_ROOT,target=/run_root,readonly"
    --mount "type=bind,source=$RUN_ROOT/runtime_record,target=/run_root/worktree/OpenLoong-Dyn-Control/record"
    --mount "type=bind,source=$RUN_ROOT,target=/demo_run"
    --mount "type=bind,source=$TOOL_ROOT,target=/tool_root,readonly"
    "$IMAGE_NAME"
    bash /tool_root/commands/container_run_openloong_demo.sh "$DEMO_EXECUTABLE" "$DEMO_NAME"
)

docker_command=$(join_command "${docker_args[@]}")
stop_command="docker stop $CONTAINER_NAME"

if [[ "$DRY_RUN" -eq 1 ]]; then
    smoke_label="none"
    if [[ -n "$SMOKE_SECONDS" ]]; then
        smoke_label="$SMOKE_SECONDS"
    fi
    cat <<EOF
Mode: $MODE
Demo: $DEMO_NAME
Executable: $DEMO_EXECUTABLE
Image: $IMAGE_NAME
Build root: $BUILD_ROOT
Run root: $RUN_ROOT
Container name: $CONTAINER_NAME
GPU enabled: $ENABLE_GPU
WSLg GPU enabled: $WSLG_GPU_AVAILABLE
Smoke seconds: $smoke_label
Docker command: $docker_command
Stop command: $stop_command
EOF
    exit 0
fi

mkdir -p "$RUN_ROOT/commands" "$RUN_ROOT/logs" "$RUN_ROOT/runtime_record"

cat > "$RUN_ROOT/commands/run_command.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
$docker_command
EOF
chmod +x "$RUN_ROOT/commands/run_command.sh"

cat > "$RUN_ROOT/run_info.txt" <<EOF
mode: $MODE
demo: $DEMO_NAME
executable: $DEMO_EXECUTABLE
image: $IMAGE_NAME
build_root: $BUILD_ROOT
run_root: $RUN_ROOT
container_name: $CONTAINER_NAME
gpu_enabled: $ENABLE_GPU
wslg_gpu_enabled: $WSLG_GPU_AVAILABLE
smoke_seconds: ${SMOKE_SECONDS:-none}
docker_command: $docker_command
stop_command: $stop_command
EOF

echo "Mode: $MODE"
echo "Demo: $DEMO_NAME"
echo "Executable: $DEMO_EXECUTABLE"
echo "Image: $IMAGE_NAME"
echo "Build root: $BUILD_ROOT"
echo "Run root: $RUN_ROOT"
echo "Container name: $CONTAINER_NAME"
echo "GPU enabled: $ENABLE_GPU"
echo "WSLg GPU enabled: $WSLG_GPU_AVAILABLE"
echo "Smoke seconds: ${SMOKE_SECONDS:-none}"
echo "Docker command saved to: $RUN_ROOT/commands/run_command.sh"

if [[ "$DETACH" -eq 1 ]]; then
    "${docker_args[@]}"
    echo "Stop command:"
    echo "  $stop_command"
    echo "Logs dir: $RUN_ROOT/logs"
    echo "Record dir: $RUN_ROOT/runtime_record"
else
    set +e
    "${docker_args[@]}"
    status=$?
    set -e
    if [[ -n "$SMOKE_SECONDS" && "$status" -eq 124 ]]; then
        echo "Smoke run reached timeout: ${SMOKE_SECONDS}s"
        status=0
    fi
    echo "Logs dir: $RUN_ROOT/logs"
    echo "Record dir: $RUN_ROOT/runtime_record"
    exit "$status"
fi
