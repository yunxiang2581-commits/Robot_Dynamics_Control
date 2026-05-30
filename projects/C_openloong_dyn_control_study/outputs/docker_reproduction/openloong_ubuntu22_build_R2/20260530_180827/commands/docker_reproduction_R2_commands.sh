#!/usr/bin/env bash
set -euo pipefail

# C03-DOCKER-R2：只在 R2 输出 worktree 中修复 MuJoCo 符号链接并重新构建。
# 说明：本文件记录本轮关键命令，便于复盘；实际执行过程的 stdout/stderr 另存 logs/。

cd /home/ubuntu/Robot_Dynamics_Control

REPO_ROOT="/home/ubuntu/Robot_Dynamics_Control"
SRC_ROOT="$REPO_ROOT/external/open_source_repos/OpenLoong-Dyn-Control"
RUN_ROOT="$REPO_ROOT/projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827"
IMAGE_TAG="openloong-ubuntu22-build:local"

git status --short
git diff --stat

rsync -a --delete \
  --exclude build \
  --exclude .git/index.lock \
  "$SRC_ROOT/" \
  "$RUN_ROOT/worktree/OpenLoong-Dyn-Control/"

cd "$RUN_ROOT/worktree/OpenLoong-Dyn-Control/third_party/mujoco/lin_x64"
ls -lah libmujoco.so*
file libmujoco.so libmujoco.so.3.1.1
cat libmujoco.so 2>/dev/null || true
test -f libmujoco.so.3.1.1
rm -f libmujoco.so
ln -s libmujoco.so.3.1.1 libmujoco.so
ls -lah libmujoco.so*
file libmujoco.so libmujoco.so.3.1.1
readlink libmujoco.so

docker image inspect "$IMAGE_TAG" >/dev/null

docker run --rm \
  -v "$RUN_ROOT:/run_root" \
  "$IMAGE_TAG" \
  bash /run_root/commands/container_build_openloong_R2.sh

# R2 继续修复：MuJoCo symlink 修复后，build 前进到 qpOASES symlink 问题。
# 以下逻辑只在 R2 worktree 内恢复 Git 索引中记录的第三方库 symlink。
git -C "$SRC_ROOT" ls-files -s | awk '$1 == "120000" {print $4}' | while IFS= read -r path; do
  target=$(git -C "$SRC_ROOT" show HEAD:"$path" 2>/dev/null || true)
  wt_path="$RUN_ROOT/worktree/OpenLoong-Dyn-Control/$path"
  wt_dir=$(dirname "$wt_path")
  target_path="$wt_dir/$target"
  if [ -n "$target" ] && [ -e "$target_path" ]; then
    rm -f "$wt_path"
    ln -s "$target" "$wt_path"
  fi
done

docker run --rm \
  -v "$RUN_ROOT:/run_root" \
  "$IMAGE_TAG" \
  bash /run_root/commands/container_build_openloong_R2.sh
