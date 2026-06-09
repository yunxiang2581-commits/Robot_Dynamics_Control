#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
RUNNER="$SCRIPT_DIR/../commands/run_openloong_demo_host.sh"

bash "$RUNNER" --list | grep -q '^walk_wbc$'
bash "$RUNNER" --list | grep -q '^walk_wbc_staircase$'
bash "$RUNNER" --list | grep -q '^walk_wbc_speed_test$'

bash "$RUNNER" --dry-run walk_wbc | grep -q 'Executable: walk_wbc'
bash "$RUNNER" --dry-run walk_wbc_speed_test | grep -q 'Executable: wbc_speed_test'

if bash "$RUNNER" does_not_exist >/dev/null 2>&1; then
    echo "invalid demo unexpectedly succeeded" >&2
    exit 1
fi
