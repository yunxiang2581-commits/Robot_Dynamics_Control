#!/usr/bin/env bash
set -euo pipefail

# 这个脚本在容器内执行。
# 目的：先记录 viewer 运行前的环境证据，再手动启动 walk_wbc。
# 输入：
# - /run_root: R2 build 产物只读挂载
# - /c06_root: 本轮输出目录可写挂载
# 输出：
# - /c06_root/logs/12_walk_wbc_precheck.log
# - /c06_root/logs/13_walk_wbc_runtime.log

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

echo "start time:"
date
} 2>&1 | tee /c06_root/logs/12_walk_wbc_precheck.log

./walk_wbc 2>&1 | tee /c06_root/logs/13_walk_wbc_runtime.log
