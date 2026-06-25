# container_build_openloong.sh

set -euo pipefail

cd /run_root/worktree/OpenLoong-Dyn-Control

{
  echo "## Container compiler environment"
  echo
  echo "### gcc-11"
  gcc-11 --version
  echo
  echo "### g++-11"
  g++-11 --version
  echo
  echo "### cmake"
  cmake --version
  echo
  echo "### make"
  make --version
} 2>&1 | tee /run_root/logs/01_container_env.log

cmake -S . -B build \
  -DCMAKE_C_COMPILER=gcc-11 \
  -DCMAKE_CXX_COMPILER=g++-11 \
  2>&1 | tee /run_root/logs/02_cmake_configure.log

cmake --build build -j"$(nproc)" \
  2>&1 | tee /run_root/logs/03_cmake_build.log

find build -maxdepth 2 -type f -executable | sort \
  | tee /run_root/build_artifacts/executable_files.txt

for target in walk_wbc walk_mpc_wbc wbc_speed_test jump_mpc float_control; do
  if [[ -x "build/$target" ]]; then
    echo "$target build/$target FOUND"
  else
    echo "$target build/$target NOT FOUND"
  fi
done | tee /run_root/build_artifacts/build_targets_check.txt
