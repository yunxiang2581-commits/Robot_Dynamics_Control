#!/usr/bin/env bash
set -euo pipefail

cd /run_root/worktree/OpenLoong-Dyn-Control/build

{
  echo "pwd: $(pwd)"
  echo "target:"
  ls -lah ./wbc_speed_test

  echo "record overlay:"
  ls -lah ../record || true
  touch ../record/write_probe.txt
  echo "record overlay write probe: OK"
  rm -f ../record/write_probe.txt

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

  echo "record directory after run:"
  ls -lah ../record || true

  echo "datalog line count:"
  wc -l ../record/datalog.log || true

  echo "datalog tail:"
  tail -n 5 ../record/datalog.log || true
} 2>&1 | tee /c04_fix_root/logs/04_wbc_speed_test_summary.log

cp -f ../record/datalog.log /c04_fix_root/artifacts/datalog.log 2>/dev/null || true
