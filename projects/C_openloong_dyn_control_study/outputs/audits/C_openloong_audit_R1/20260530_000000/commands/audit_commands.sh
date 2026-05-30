#!/usr/bin/env bash
set -euo pipefail

cd /home/ubuntu/Robot_Dynamics_Control

# Repository state
git status --short
git diff --stat

# Project C structure
find projects/C_openloong_dyn_control_study -maxdepth 5 -type d | sort
find projects/C_openloong_dyn_control_study -maxdepth 6 -type f | sort | sed -n '1,400p'

# Project C docs and related repo docs
find projects/C_openloong_dyn_control_study -maxdepth 6 -type f | grep -Ei "README|readme|\.md$|\.txt$" | sort
find docs -maxdepth 4 -type f | grep -Ei "OpenLoong|openloong|C_openloong|dyn_control|WBC|MPC|PVT|humanoid|人形|复现|审查|roadmap|status" | sort || true

# Entrypoints and modules
rg -n "if __name__ == .__main__.|def main|argparse|click|typer|run_|demo|smoke|simulate|mujoco|MuJoCo|viewer|headless|no-viewer|no_show|export" \
  projects/C_openloong_dyn_control_study docs || true
find projects/C_openloong_dyn_control_study -name "*.py" -o -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.cc" | sort
rg -n "class |def |struct |namespace |main\(|Controller|MPC|WBC|PVT|PD|QP|IK|dynamics|kinematics|mujoco|pinocchio|state|torque|joint|contact|trajectory|planner|estimator" \
  projects/C_openloong_dyn_control_study || true

# Config and resources
find projects/C_openloong_dyn_control_study -maxdepth 8 -type f | grep -Ei "\.ya?ml$|\.json$|\.toml$|\.ini$|\.xml$|\.urdf$|\.mjcf$|\.xacro$|\.csv$" | sort
rg -n "model_path|xml|urdf|mjcf|asset|robot|OpenLoong|openloong|joint|actuator|torque|ctrlrange|timestep|dt|horizon|contact|foot|base|pelvis" \
  projects/C_openloong_dyn_control_study || true

# External upstream source audit
find external -maxdepth 4 -type d | sort | grep -Ei "openloong|loong|dyn|control|humanoid" || true
find external -maxdepth 5 -type f | sort | grep -Ei "openloong|loong|dyn|control|humanoid|README|LICENSE" || true
git --no-optional-locks -C external/open_source_repos/OpenLoong-Dyn-Control rev-parse --short HEAD
git --no-optional-locks -C external/open_source_repos/OpenLoong-Dyn-Control status --short --branch | sed -n '1,80p'

# Dependency audit
find projects/C_openloong_dyn_control_study -maxdepth 6 -type f | grep -Ei "requirements|environment|conda|pyproject|setup.py|CMakeLists|package.xml|Dockerfile|compose|install" | sort
rg -n "import |from |#include|find_package|target_link_libraries|mujoco|pinocchio|casadi|osqp|eigen|yaml|numpy|scipy|matplotlib|torch|ros|rclcpp|ament|cmake" \
  projects/C_openloong_dyn_control_study \
  external/open_source_repos/OpenLoong-Dyn-Control/CMakeLists.txt \
  external/open_source_repos/OpenLoong-Dyn-Control/demo \
  external/open_source_repos/OpenLoong-Dyn-Control/algorithm \
  external/open_source_repos/OpenLoong-Dyn-Control/common \
  external/open_source_repos/OpenLoong-Dyn-Control/sim_interface || true

# Safe compile/test discovery only
find projects/C_openloong_dyn_control_study -name "*.py" | sort
find projects/C_openloong_dyn_control_study -maxdepth 5 -type f | grep -Ei "test_|_test|tests/" | sort

# Environment evidence
python --version
pytest --version
cmake --version || true
g++ --version
make --version
lsb_release -a
