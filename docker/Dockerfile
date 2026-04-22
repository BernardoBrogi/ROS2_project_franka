# Base image: ROS 2 Humble (Ubuntu 22.04)
FROM osrf/ros:humble-desktop

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Arguments for user setup
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

# Install sudo and useful tools
RUN apt-get update && apt-get install -y \
    sudo \
    git \
    python3-pip \
    nano \
    gedit \
    wget \
    libgtest-dev \
    libgmock-dev \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -m -s /bin/bash --uid $USER_UID --gid $USER_GID $USERNAME \
    && usermod -aG sudo $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Set up ROS environment automatically
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

# Set working directory
WORKDIR /home/$USERNAME

# Switch to non-root user
USER $USERNAME

# Default shell
CMD ["bash"]