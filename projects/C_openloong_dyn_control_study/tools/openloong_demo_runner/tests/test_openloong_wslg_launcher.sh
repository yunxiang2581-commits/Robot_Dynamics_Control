#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/../../../../../" && pwd)
LAUNCHER="$SCRIPT_DIR/../commands/run_openloong_demo_wslg.sh"

output=$(bash "$LAUNCHER" --list)
echo "$output" | grep -q '^walk_wbc$'
echo "$output" | grep -q '^walk_mpc_wbc_joystick$'
echo "$output" | grep -q '^walk_wbc_speed_test$'

output=$(bash "$LAUNCHER" --dry-run walk_wbc)
echo "$output" | grep -q '^Mode: foreground$'
echo "$output" | grep -q '^Demo: walk_wbc$'
echo "$output" | grep -q '^Executable: walk_wbc$'
echo "$output" | grep -q '^Image: openloong-ubuntu22-build:local$'
echo "$output" | grep -q "^Build root: $REPO_ROOT/projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827$"
echo "$output" | grep -q '^GPU enabled: 0$'
echo "$output" | grep -q '^WSLg GPU enabled: [01]$'
echo "$output" | grep -q '^Smoke seconds: none$'
echo "$output" | grep -q '^Docker command: docker run --rm -it '
echo "$output" | grep -q -- '-e DISPLAY='
echo "$output" | grep -q -- '-v /mnt/wslg:/mnt/wslg:rw'

if echo "$output" | grep -q '^WSLg GPU enabled: 1$'; then
    echo "$output" | grep -q -- '--device /dev/dxg'
    echo "$output" | grep -q -- '-v /usr/lib/wsl:/usr/lib/wsl:ro'
    echo "$output" | grep -q -- 'LD_LIBRARY_PATH=/usr/lib/wsl/lib:'
fi

output=$(bash "$LAUNCHER" --dry-run --detach --gpu --smoke-seconds 12 walk_wbc_staircase)
echo "$output" | grep -q '^Mode: detached$'
echo "$output" | grep -q '^Demo: walk_wbc_staircase$'
echo "$output" | grep -q '^Executable: walk_wbc_staircase$'
echo "$output" | grep -q '^GPU enabled: 1$'
echo "$output" | grep -q '^WSLg GPU enabled: [01]$'
echo "$output" | grep -q '^Smoke seconds: 12$'
echo "$output" | grep -q '^Docker command: timeout 12s docker run '
echo "$output" | grep -q -- ' --rm '
echo "$output" | grep -q -- ' -d '
echo "$output" | grep -q -- '--gpus all'
echo "$output" | grep -q 'NVIDIA_DRIVER_CAPABILITIES=all'

output=$(bash "$LAUNCHER" --dry-run --no-wslg-gpu walk_wbc)
echo "$output" | grep -q '^WSLg GPU enabled: 0$'
if echo "$output" | grep -q -- '--device /dev/dxg'; then
    echo "--no-wslg-gpu unexpectedly passed /dev/dxg" >&2
    exit 1
fi

if bash "$LAUNCHER" --dry-run does_not_exist >/dev/null 2>&1; then
    echo "invalid demo unexpectedly succeeded" >&2
    exit 1
fi
