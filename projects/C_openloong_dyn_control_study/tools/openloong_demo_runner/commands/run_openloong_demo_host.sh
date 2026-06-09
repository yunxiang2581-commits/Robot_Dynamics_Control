#!/usr/bin/env bash
set -euo pipefail

# 宿主机总入口。
# 作用：
# 1. 列出可运行 demo
# 2. 为一次 demo 运行创建独立输出目录
# 3. 统一发起 Docker + X11 运行
#
# 用法：
#   bash run_openloong_demo_host.sh --list
#   bash run_openloong_demo_host.sh --dry-run walk_wbc
#   bash run_openloong_demo_host.sh walk_wbc_staircase

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "$SCRIPT_DIR/openloong_demo_common.sh"

usage() {
    cat <<'EOF'
Usage:
  bash run_openloong_demo_host.sh --list
  bash run_openloong_demo_host.sh --dry-run <demo_name>
  bash run_openloong_demo_host.sh <demo_name>

Environment overrides:
  OPENLOONG_IMAGE_NAME
  OPENLOONG_BUILD_ROOT
  OPENLOONG_RUNS_ROOT
EOF
}

list_demos() {
    openloong_list_demos
}

require_path() {
    openloong_require_path "$1" "$2" || exit 1
}

DRY_RUN=0
DEMO_NAME=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --list)
            list_demos
            exit 0
            ;;
        --dry-run)
            DRY_RUN=1
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
    list_demos >&2
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

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
RUN_ROOT="$RUNS_ROOT/${TIMESTAMP}_${DEMO_NAME}"

if [[ "$DRY_RUN" -eq 1 ]]; then
    cat <<EOF
Demo: $DEMO_NAME
Executable: $DEMO_EXECUTABLE
Image: $IMAGE_NAME
Build root: $BUILD_ROOT
Run root: $RUN_ROOT
Container script: $CONTAINER_SCRIPT
EOF
    exit 0
fi

mkdir -p "$RUN_ROOT/commands" "$RUN_ROOT/logs" "$RUN_ROOT/runtime_record"

echo "Demo: $DEMO_NAME"
echo "Executable: $DEMO_EXECUTABLE"
echo "Image: $IMAGE_NAME"
echo "Build root: $BUILD_ROOT"
echo "Run root: $RUN_ROOT"

xhost +SI:localuser:root >/dev/null
trap 'xhost -SI:localuser:root >/dev/null 2>&1 || true' EXIT

docker run --rm -it \
--gpus all \
-e DISPLAY="${DISPLAY:-}" \
-e NVIDIA_DRIVER_CAPABILITIES=all \
-e __GLX_VENDOR_LIBRARY_NAME=nvidia \
-e QT_X11_NO_MITSHM=1 \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
-v "$BUILD_ROOT:/run_root:ro" \
-v "$RUN_ROOT/runtime_record:/run_root/worktree/OpenLoong-Dyn-Control/record" \
-v "$RUN_ROOT:/demo_run" \
-v "$TOOL_ROOT:/tool_root:ro" \
"$IMAGE_NAME" \
bash /tool_root/commands/container_run_openloong_demo.sh "$DEMO_EXECUTABLE" "$DEMO_NAME"

echo "Logs saved to: $RUN_ROOT/logs"
echo "Record dir: $RUN_ROOT/runtime_record"
