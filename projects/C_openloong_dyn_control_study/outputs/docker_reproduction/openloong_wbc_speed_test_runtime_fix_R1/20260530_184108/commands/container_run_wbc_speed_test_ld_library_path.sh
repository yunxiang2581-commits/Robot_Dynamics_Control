#!/usr/bin/env bash
set -euo pipefail

export OPENLOONG_ROOT=/run_root/worktree/OpenLoong-Dyn-Control
export LD_LIBRARY_PATH="$OPENLOONG_ROOT/third_party/qpOASES/lin_x64:$OPENLOONG_ROOT/third_party/mujoco/lin_x64:$OPENLOONG_ROOT/third_party/glfw/lin_x64:${LD_LIBRARY_PATH:-}"

cd "$OPENLOONG_ROOT/build"

{
  echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
  echo "ldd:"
  ldd ./wbc_speed_test || true
  echo "readelf rpath/runpath:"
  readelf -d ./wbc_speed_test | grep -E "RPATH|RUNPATH" || true
} 2>&1 | tee /c04_fix_root/logs/05_ld_library_path_precheck.log

set +e
timeout 20s ./wbc_speed_test \
  > /c04_fix_root/logs/06_wbc_speed_test_ld_stdout.log \
  2> /c04_fix_root/logs/07_wbc_speed_test_ld_stderr.log
EXIT_CODE=$?
set -e

echo "$EXIT_CODE" | tee /c04_fix_root/artifacts/wbc_speed_test_ld_exit_code.txt

{
  echo "exit_code=$EXIT_CODE"
  echo "stdout tail:"
  tail -n 160 /c04_fix_root/logs/06_wbc_speed_test_ld_stdout.log || true
  echo "stderr tail:"
  tail -n 160 /c04_fix_root/logs/07_wbc_speed_test_ld_stderr.log || true
} 2>&1 | tee /c04_fix_root/logs/08_wbc_speed_test_ld_summary.log
