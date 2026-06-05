#!/usr/bin/env bash
set -euo pipefail

# 容器内脚本。
# 作用：在真正启动 walk_wbc 前，先记录 GPU / OpenGL / 依赖证据。
# 输入：
# - /run_root: R2 build 产物只读挂载
# - /c06_root: 本轮输出目录可写挂载
# 输出：
# - /c06_root/logs/09_walk_wbc_precheck.log
# - /c06_root/logs/10_walk_wbc_runtime.log

cd /run_root/worktree/OpenLoong-Dyn-Control/build

{
echo "pwd: $(pwd)"
echo "target:"
ls -lah ./walk_wbc

echo "nvidia-smi:"
nvidia-smi || true

echo "glxinfo:"
glxinfo -B || true

echo "ldd:"
ldd ./walk_wbc || true

echo "rpath/runpath:"
readelf -d ./walk_wbc | grep -E "RPATH|RUNPATH" || true

echo "record dir:"
ls -lah /run_root/worktree/OpenLoong-Dyn-Control/record || true

echo "start time:"
date
} 2>&1 | tee /c06_root/logs/09_walk_wbc_precheck.log

./walk_wbc 2>&1 | tee /c06_root/logs/10_walk_wbc_runtime.log
