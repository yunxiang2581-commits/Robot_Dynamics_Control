#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="/home/ubuntu/Robot_Dynamics_Control"
IMAGE_TAG="openloong-ubuntu22-build:local"
DOCKERFILE="projects/C_openloong_dyn_control_study/docker/openloong_ubuntu22_build/Dockerfile"

cd "$REPO_ROOT"

docker build \
  -t "$IMAGE_TAG" \
  -f "$DOCKERFILE" \
  .
