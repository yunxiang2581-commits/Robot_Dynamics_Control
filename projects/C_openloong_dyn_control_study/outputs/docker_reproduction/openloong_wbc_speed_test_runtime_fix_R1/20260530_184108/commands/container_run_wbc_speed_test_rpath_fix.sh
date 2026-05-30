#!/usr/bin/env bash
set -euo pipefail

cd /run_root/worktree/OpenLoong-Dyn-Control/build

{
  echo "pwd: $(pwd)"
  echo "target:"
  ls -lah ./wbc_speed_test

  echo "third_party qpOASES:"
  ls -lah ../third_party/qpOASES/lin_x64 || true

  echo "third_party mujoco:"
  ls -lah ../third_party/mujoco/lin_x64 || true

  echo "ldd:"
  ldd ./wbc_speed_test || true

  echo "readelf rpath/runpath:"
  readelf -d ./wbc_speed_test | grep -E "RPATH|RUNPATH" || true

  echo "start time:"
  date
} 2>&1 | tee /c04_fix_root/logs/01_wbc_speed_test_precheck.log

set +e
timeout 20s ./wbc_speed_test \
  > /c04_fix_root/logs/02_wbc_speed_test_stdout.log \
  2> /c04_fix_root/logs/03_wbc_speed_test_stderr.log
EXIT_CODE=$?
set -e

echo "$EXIT_CODE" | tee /c04_fix_root/artifacts/wbc_speed_test_exit_code.txt

{
  echo "exit_code=$EXIT_CODE"
  echo "end time:"
  date

  echo "stdout tail:"
  tail -n 160 /c04_fix_root/logs/02_wbc_speed_test_stdout.log || true

  echo "stderr tail:"
  tail -n 160 /c04_fix_root/logs/03_wbc_speed_test_stderr.log || true
} 2>&1 | tee /c04_fix_root/logs/04_wbc_speed_test_summary.log
