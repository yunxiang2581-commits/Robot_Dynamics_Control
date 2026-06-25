#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/../../../.." && pwd)
IMAGE_TAG="openloong-ubuntu22-build:local"
DOCKERFILE="projects/C_openloong_dyn_control_study/docker/openloong_ubuntu22_build/Dockerfile"

cd "$REPO_ROOT"

docker build \
  -t "$IMAGE_TAG" \
  -f "$DOCKERFILE" \
  .
