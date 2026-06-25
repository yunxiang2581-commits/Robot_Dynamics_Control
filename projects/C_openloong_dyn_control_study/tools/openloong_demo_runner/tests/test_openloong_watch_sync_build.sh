#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/../../../../../" && pwd)
WATCHER="$SCRIPT_DIR/../commands/watch_openloong_sync_and_build.sh"

output=$(bash "$WATCHER" --dry-run walk_wbc_staircase)
echo "$output" | grep -q '^Demo: walk_wbc_staircase$'
echo "$output" | grep -q '^Executable: walk_wbc_staircase$'
echo "$output" | grep -q "^Source root: $REPO_ROOT/external/open_source_repos/OpenLoong-Dyn-Control$"
echo "$output" | grep -q "^Build root: $REPO_ROOT/projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827$"
echo "$output" | grep -q "^Build worktree: $REPO_ROOT/projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827/worktree/OpenLoong-Dyn-Control$"
echo "$output" | grep -q '^Image: openloong-ubuntu22-build:local$'

output=$(bash "$WATCHER" --dry-run --poll-interval 2.5 walk_wbc)
echo "$output" | grep -q '^Demo: walk_wbc$'
echo "$output" | grep -q '^Poll interval: 2.5$'

if bash "$WATCHER" --dry-run does_not_exist >/dev/null 2>&1; then
    echo "invalid demo unexpectedly succeeded" >&2
    exit 1
fi
