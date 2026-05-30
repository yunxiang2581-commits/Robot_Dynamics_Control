#!/usr/bin/env bash
set -euo pipefail

# C05A command record. This file records the read-only inspection commands and
# report-generation commands used for this source trace.

cd /home/ubuntu/Robot_Dynamics_Control

OUT_DIR="projects/C_openloong_dyn_control_study/outputs/source_trace/C05A_wbc_speed_test_trace_R1/20260530_213038"
OPENLOONG_ROOT="projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827/worktree/OpenLoong-Dyn-Control"
C04_ROOT="projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_wbc_speed_test_runtime_fix_R2/20260530_184509"

mkdir -p "$OUT_DIR"/{reports,tables,diagrams,logs,commands}

git status --short
git diff --stat

ls -lah "$OPENLOONG_ROOT/build/wbc_speed_test" || true
ls -lah "$C04_ROOT/logs" || true
ls -lah "$C04_ROOT/artifacts" || true

find "$OPENLOONG_ROOT" -maxdepth 5 -type f \
  | grep -Ei "wbc.*speed|speed.*test|wbc_speed_test|walk_wbc_speed_test" \
  | sort \
  | tee "$OUT_DIR/logs/00_wbc_speed_test_source_candidates.log"

rg -n "int main|LoopNum|DataBus|Pin_KinDyn|WBC_priority|GaitScheduler|FootPlacement|PVT_Ctr|DataLogger|dataBusRead|dataBusWrite|computeJ_dJ|computeDyn|computeDdq|computeTau|calMotorsPVT|motors_|tau|q\b|dq\b|ddq|qp|log|datalog|cout|std::cout|chrono|time|duration" \
  "$OPENLOONG_ROOT/demo/walk_wbc_speed_test.cpp" \
  "$OPENLOONG_ROOT/common" \
  "$OPENLOONG_ROOT/algorithm" \
  2>&1 | tee "$OUT_DIR/logs/01_wbc_speed_test_rg_trace.log"

nl -ba "$OPENLOONG_ROOT/demo/walk_wbc_speed_test.cpp" | sed -n '1,260p'
nl -ba "$OPENLOONG_ROOT/common/data_logger.h" | sed -n '1,220p'
nl -ba "$OPENLOONG_ROOT/common/data_logger.cpp" | sed -n '1,260p'
nl -ba "$OPENLOONG_ROOT/common/data_bus.h" | sed -n '1,260p'
nl -ba "$OPENLOONG_ROOT/algorithm/pino_kin_dyn.cpp" | sed -n '1,330p'
nl -ba "$OPENLOONG_ROOT/algorithm/wbc_priority.cpp" | sed -n '1,780p'
nl -ba "$OPENLOONG_ROOT/algorithm/gait_scheduler.cpp" | sed -n '1,260p'
nl -ba "$OPENLOONG_ROOT/algorithm/foot_placement.cpp" | sed -n '1,300p'
nl -ba "$OPENLOONG_ROOT/algorithm/joystick_interpreter.cpp" | sed -n '1,260p'
nl -ba "$OPENLOONG_ROOT/common/PVT_ctrl.cpp" | sed -n '1,320p'
nl -ba "$OPENLOONG_ROOT/common/PVT_ctrl.h" | sed -n '1,240p'
nl -ba "$OPENLOONG_ROOT/algorithm/priority_tasks.cpp" | sed -n '1,260p'
nl -ba "$OPENLOONG_ROOT/algorithm/priority_tasks.h" | sed -n '1,240p'

wc -l "$C04_ROOT/logs/02_wbc_speed_test_stdout.log"
head -n 20 "$C04_ROOT/logs/02_wbc_speed_test_stdout.log"
tail -n 20 "$C04_ROOT/logs/02_wbc_speed_test_stdout.log"
sed -n '1,80p' "$C04_ROOT/logs/02_wbc_speed_test_stdout.log" > "$OUT_DIR/logs/02_stdout_head_sample.log"
tail -n 80 "$C04_ROOT/logs/02_wbc_speed_test_stdout.log" > "$OUT_DIR/logs/03_stdout_tail_sample.log"

wc -l "$C04_ROOT/artifacts/datalog.log"
head -n 5 "$C04_ROOT/artifacts/datalog.log"
tail -n 5 "$C04_ROOT/artifacts/datalog.log"

python3 - <<'PY'
from pathlib import Path
p = Path('projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_wbc_speed_test_runtime_fix_R2/20260530_184509/artifacts/datalog.log')
lines = p.read_text(errors='replace').splitlines()
print('num_lines', len(lines))
if lines:
    first = lines[0]
    for sep in [',', ' ', '\t']:
        parts = [x for x in first.split(sep) if x != '']
        print('sep', repr(sep), 'num_cols', len(parts), 'sample', parts[:10])
PY

sed -n '1,80p' "$C04_ROOT/runtime_record/matlabReadDataScript.txt"

python3 - <<'PY'
from pathlib import Path
import statistics
p = Path('projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_wbc_speed_test_runtime_fix_R2/20260530_184509/artifacts/datalog.log')
vals = [float(line.split(',')[-1]) for line in p.read_text(errors='replace').splitlines() if line.strip()]
vals_sorted = sorted(vals)
def pct(q):
    i = round((len(vals_sorted) - 1) * q)
    return vals_sorted[i]
print('count', len(vals))
print('min_sec', min(vals))
print('mean_sec', statistics.fmean(vals))
print('median_sec', statistics.median(vals))
print('p95_sec', pct(0.95))
print('max_sec', max(vals))
print('first_sec', vals[0])
print('last_sec', vals[-1])
PY

# The Markdown/CSV/Mermaid files in this output directory were created with
# apply_patch in this session; no source file under external/ or the R2 worktree
# was edited.

git diff --stat
