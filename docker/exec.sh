#!/bin/bash

CONTAINER_NAME=ros2_container
USERNAME=ros
CONTAINER_WS="/home/$USERNAME"

# Check if container exists
if ! docker ps -a --format '{{.Names}}' | grep -Eq "^${CONTAINER_NAME}\$"; then
    echo "❌ Container '$CONTAINER_NAME' does not exist."
    echo "👉 Run ./run.sh first"
    exit 1
fi

# Start container if it's stopped
if [ "$(docker inspect -f '{{.State.Running}}' $CONTAINER_NAME)" != "true" ]; then
    echo "▶️ Starting container..."
    docker start $CONTAINER_NAME
fi

# Allow GUI access
xhost +local:docker > /dev/null 2>&1

# Exec into container
echo "🔗 Attaching to container..."
docker exec -it -u $USERNAME -w $CONTAINER_WS $CONTAINER_NAME bash