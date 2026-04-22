#!/bin/bash

IMAGE_NAME=ros2_humble_dev
CONTAINER_NAME=ros2_container
USERNAME=ros

# Allow Docker to use X11
xhost +local:docker

docker run -it \
    --name $CONTAINER_NAME \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --network=host \
    --ipc=host \
    --privileged \
    $IMAGE_NAME

# Revoke permission after exit (optional but safer)
# xhost -local:docker