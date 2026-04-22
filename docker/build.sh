#!/bin/bash

IMAGE_NAME=ros2_humble_dev
USERNAME=ros
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "🔨 Building Docker image: $IMAGE_NAME"

docker build \
  --build-arg USERNAME=$USERNAME \
  --build-arg USER_UID=$(id -u) \
  --build-arg USER_GID=$(id -g) \
  -t $IMAGE_NAME "$SCRIPT_DIR"

echo "✅ Build complete!"