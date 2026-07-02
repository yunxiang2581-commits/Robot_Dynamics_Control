#!/usr/bin/env bash

# OpenLoong demo runner / watcher 共享配置。
# 作用：
# 1. 统一维护 demo 名 -> executable/target 的映射
# 2. 统一维护 build-root、source-root、image 等默认路径
# 3. 提供基础路径校验与 demo 解析函数

OPENLOONG_COMMON_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
OPENLOONG_TOOL_ROOT=$(cd "$OPENLOONG_COMMON_DIR/.." && pwd)
OPENLOONG_REPO_ROOT=$(cd "$OPENLOONG_COMMON_DIR/../../../../.." && pwd)

OPENLOONG_DEFAULT_BUILD_ROOT="$OPENLOONG_REPO_ROOT/projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827"
OPENLOONG_DEFAULT_RUNS_ROOT="$OPENLOONG_REPO_ROOT/projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_demo_runs"
OPENLOONG_DEFAULT_SOURCE_ROOT="$OPENLOONG_REPO_ROOT/external/open_source_repos/OpenLoong-Dyn-Control"

OPENLOONG_IMAGE_NAME="${OPENLOONG_IMAGE_NAME:-openloong-ubuntu22-build:local}"
OPENLOONG_BUILD_ROOT="${OPENLOONG_BUILD_ROOT:-$OPENLOONG_DEFAULT_BUILD_ROOT}"
OPENLOONG_RUNS_ROOT="${OPENLOONG_RUNS_ROOT:-$OPENLOONG_DEFAULT_RUNS_ROOT}"
OPENLOONG_SOURCE_ROOT="${OPENLOONG_SOURCE_ROOT:-$OPENLOONG_DEFAULT_SOURCE_ROOT}"

declare -gA OPENLOONG_DEMO_TO_EXEC=(
    [walk_wbc]=walk_wbc
    [walk_wbc_scene_staircase]=walk_wbc_scene_staircase
    [walk_wbc_staircase]=walk_wbc_staircase
    [walk_mpc_wbc]=walk_mpc_wbc
    [walk_mpc_wbc_joystick]=walk_mpc_wbc_joystick
    [walk_wbc_joystick]=walk_wbc_joystick
    [jump_mpc]=jump_mpc
    [jump_mpc_modular]=jump_mpc_modular
    [float_control]=float_control
    [walk_wbc_speed_test]=wbc_speed_test
    [wbc_speed_test]=wbc_speed_test
)

declare -ga OPENLOONG_CANONICAL_DEMOS=(
    walk_wbc
    walk_wbc_scene_staircase
    walk_wbc_staircase
    walk_mpc_wbc
    walk_mpc_wbc_joystick
    walk_wbc_joystick
    jump_mpc
    jump_mpc_modular
    float_control
    walk_wbc_speed_test
)

openloong_list_demos() {
    printf '%s\n' "${OPENLOONG_CANONICAL_DEMOS[@]}"
}

openloong_require_path() {
    local path="$1"
    local label="$2"
    if [[ ! -e "$path" ]]; then
        echo "$label not found: $path" >&2
        return 1
    fi
}

openloong_resolve_demo() {
    local demo_name="$1"

    if [[ -z "${OPENLOONG_DEMO_TO_EXEC[$demo_name]:-}" ]]; then
        return 1
    fi

    OPENLOONG_DEMO_NAME="$demo_name"
    OPENLOONG_DEMO_EXECUTABLE="${OPENLOONG_DEMO_TO_EXEC[$demo_name]}"
    OPENLOONG_DEMO_TARGET="$OPENLOONG_DEMO_EXECUTABLE"
}

openloong_init_build_context() {
    OPENLOONG_BUILD_WORKTREE="$OPENLOONG_BUILD_ROOT/worktree/OpenLoong-Dyn-Control"
    OPENLOONG_BUILD_DIR="$OPENLOONG_BUILD_WORKTREE/build"
}
