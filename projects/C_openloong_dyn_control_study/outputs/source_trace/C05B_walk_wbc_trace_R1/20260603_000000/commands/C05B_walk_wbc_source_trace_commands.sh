#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../../../.." && pwd)"
cd "$repo_root"

git status

nl -ba external/open_source_repos/OpenLoong-Dyn-Control/demo/walk_wbc.cpp | sed -n '1,380p'
nl -ba external/open_source_repos/OpenLoong-Dyn-Control/demo/walk_wbc_speed_test.cpp | sed -n '1,260p'
nl -ba external/open_source_repos/OpenLoong-Dyn-Control/sim_interface/MJ_interface.cpp | sed -n '1,280p'
nl -ba external/open_source_repos/OpenLoong-Dyn-Control/sim_interface/MJ_interface.h | sed -n '1,240p'
nl -ba external/open_source_repos/OpenLoong-Dyn-Control/common/data_bus.h | sed -n '1,260p'

rg -n "void (dataBusRead|dataBusWrite)|computeDdq|computeTau|qp_status|wbc_tauJointRes|wbc_FrRes" \
  external/open_source_repos/OpenLoong-Dyn-Control/algorithm/wbc_priority.cpp \
  external/open_source_repos/OpenLoong-Dyn-Control/algorithm/wbc_priority.h \
  external/open_source_repos/OpenLoong-Dyn-Control/algorithm/pino_kin_dyn.cpp \
  external/open_source_repos/OpenLoong-Dyn-Control/common/PVT_ctrl.cpp

git diff --stat
