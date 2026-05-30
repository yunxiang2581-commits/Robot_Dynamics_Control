#!/usr/bin/env bash
set -euo pipefail

cd /r2_root/worktree/OpenLoong-Dyn-Control/build

{
  echo "pwd: $(pwd)"
  echo "listing target:"
  ls -lah ./wbc_speed_test
  echo "ldd:"
  ldd ./wbc_speed_test || true
  echo "start time:"
  date
} 2>&1 | tee /c04_root/logs/02_wbc_speed_test_precheck.log

set +e
timeout 20s ./wbc_speed_test > /c04_root/logs/03_wbc_speed_test_stdout.log 2> /c04_root/logs/04_wbc_speed_test_stderr.log
EXIT_CODE=$?
set -e

echo "$EXIT_CODE" | tee /c04_root/artifacts/wbc_speed_test_exit_code.txt

{
  echo "exit_code=$EXIT_CODE"
  echo "end time:"
  date
  echo "stdout tail:"
  tail -n 80 /c04_root/logs/03_wbc_speed_test_stdout.log || true
  echo "stderr tail:"
  tail -n 80 /c04_root/logs/04_wbc_speed_test_stderr.log || true
} 2>&1 | tee /c04_root/logs/05_wbc_speed_test_summary.log
