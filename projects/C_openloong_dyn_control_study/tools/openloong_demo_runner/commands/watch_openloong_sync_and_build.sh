#!/usr/bin/env bash
set -euo pipefail

# 自动同步 + 自动增量编译 watcher。
# 输入：
#   demo_name
# 可选参数：
#   --dry-run        只打印解析结果，不监听、不编译
#   --debounce SEC   变化后等待多少秒再触发一次 sync/build
#   --poll-interval  轮询模式下每隔多少秒检查一次
#
# 说明：
# 1. 优先使用 inotifywait 监听源码变化
# 2. 当前环境没有 inotifywait 时，自动退化为轮询模式
# 3. 不自动运行 demo，只更新 build-root worktree 和目标 executable

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "$SCRIPT_DIR/openloong_demo_common.sh"

DRY_RUN=0
DEMO_NAME=""
DEBOUNCE_SEC="1.0"
POLL_INTERVAL_SEC="1.0"

usage() {
    cat <<'EOF'
Usage:
  bash watch_openloong_sync_and_build.sh --dry-run <demo_name>
  bash watch_openloong_sync_and_build.sh [--debounce 1.0] [--poll-interval 1.0] <demo_name>

Environment overrides:
  OPENLOONG_IMAGE_NAME
  OPENLOONG_BUILD_ROOT
  OPENLOONG_SOURCE_ROOT
EOF
}

require_numeric() {
    local value="$1"
    local label="$2"
    if [[ ! "$value" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
        echo "$label must be a non-negative number: $value" >&2
        exit 1
    fi
}

log_msg() {
    local timestamp
    timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[$timestamp] $*"
}

is_relevant_path() {
    local path="$1"
    case "$path" in
        *.cpp|*.cc|*.cxx|*.h|*.hpp|*/CMakeLists.txt|*.cmake|*.json|*.xml|*.urdf)
            return 0
            ;;
        *)
            return 1
            ;;
    esac
}

compute_snapshot() {
    find "$SOURCE_ROOT" \
        \( -path "$SOURCE_ROOT/.git" -o -path "$SOURCE_ROOT/build" -o -path "$SOURCE_ROOT/record" \) -prune \
        -o \
        \( -name '*.cpp' -o -name '*.cc' -o -name '*.cxx' -o -name '*.h' -o -name '*.hpp' -o -name 'CMakeLists.txt' -o -name '*.cmake' -o -name '*.json' -o -name '*.xml' -o -name '*.urdf' \) \
        -type f -printf '%T@ %p\n' | sort | sha256sum | awk '{print $1}'
}

sync_source_to_build_worktree() {
    rsync -a \
        --exclude '.git/' \
        --exclude 'build/' \
        --exclude 'record/' \
        --exclude 'third_party/' \
        --exclude 'doc/doxygen/' \
        "$SOURCE_ROOT/" "$BUILD_WORKTREE/"
}

build_target_in_docker() {
    docker run --rm \
        -v "$BUILD_ROOT:/run_root" \
        "$IMAGE_NAME" \
        bash -lc "set -euo pipefail; BUILD_DIR=/run_root/worktree/OpenLoong-Dyn-Control/build; cmake --build \"\$BUILD_DIR\" --target \"$TARGET_NAME\" -j\$(nproc)"
}

sync_and_build_once() {
    log_msg "sync start: $SOURCE_ROOT -> $BUILD_WORKTREE"
    sync_source_to_build_worktree
    log_msg "build start: target=$TARGET_NAME image=$IMAGE_NAME"
    build_target_in_docker
    log_msg "build done: target=$TARGET_NAME"
}

print_dry_run() {
    cat <<EOF
Demo: $DEMO_NAME
Executable: $EXECUTABLE_NAME
Target: $TARGET_NAME
Source root: $SOURCE_ROOT
Build root: $BUILD_ROOT
Build worktree: $BUILD_WORKTREE
Image: $IMAGE_NAME
Debounce: $DEBOUNCE_SEC
Poll interval: $POLL_INTERVAL_SEC
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run)
            DRY_RUN=1
            shift
            ;;
        --debounce)
            shift
            [[ $# -gt 0 ]] || { echo "Missing value for --debounce" >&2; exit 1; }
            DEBOUNCE_SEC="$1"
            shift
            ;;
        --poll-interval)
            shift
            [[ $# -gt 0 ]] || { echo "Missing value for --poll-interval" >&2; exit 1; }
            POLL_INTERVAL_SEC="$1"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            if [[ -n "$DEMO_NAME" ]]; then
                echo "Only one demo name may be provided." >&2
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

require_numeric "$DEBOUNCE_SEC" "Debounce"
require_numeric "$POLL_INTERVAL_SEC" "Poll interval"

if ! openloong_resolve_demo "$DEMO_NAME"; then
    echo "Unknown demo: $DEMO_NAME" >&2
    echo "Available demos:" >&2
    openloong_list_demos >&2
    exit 1
fi

openloong_init_build_context
SOURCE_ROOT="$OPENLOONG_SOURCE_ROOT"
BUILD_ROOT="$OPENLOONG_BUILD_ROOT"
BUILD_WORKTREE="$OPENLOONG_BUILD_WORKTREE"
IMAGE_NAME="$OPENLOONG_IMAGE_NAME"
EXECUTABLE_NAME="$OPENLOONG_DEMO_EXECUTABLE"
TARGET_NAME="$OPENLOONG_DEMO_TARGET"

openloong_require_path "$SOURCE_ROOT" "Source root" || exit 1
openloong_require_path "$BUILD_ROOT" "Build root" || exit 1
openloong_require_path "$BUILD_WORKTREE" "Build worktree" || exit 1

if [[ "$DRY_RUN" -eq 1 ]]; then
    print_dry_run
    exit 0
fi

log_msg "watch start: demo=$DEMO_NAME target=$TARGET_NAME"
log_msg "source root: $SOURCE_ROOT"
log_msg "build worktree: $BUILD_WORKTREE"

if command -v inotifywait >/dev/null 2>&1; then
    log_msg "mode: inotifywait"
    while IFS= read -r changed_path; do
        if ! is_relevant_path "$changed_path"; then
            continue
        fi
        log_msg "change detected: $changed_path"
        sleep "$DEBOUNCE_SEC"
        sync_and_build_once
    done < <(
        inotifywait -m -r \
            --format '%w%f' \
            -e close_write -e create -e delete -e move \
            "$SOURCE_ROOT"
    )
else
    log_msg "mode: polling"
    previous_snapshot=$(compute_snapshot)
    while true; do
        sleep "$POLL_INTERVAL_SEC"
        current_snapshot=$(compute_snapshot)
        if [[ "$current_snapshot" != "$previous_snapshot" ]]; then
            log_msg "change detected by polling"
            sleep "$DEBOUNCE_SEC"
            sync_and_build_once
            previous_snapshot=$(compute_snapshot)
        fi
    done
fi
