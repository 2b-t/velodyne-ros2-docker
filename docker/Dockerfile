FROM ros:humble-perception

ENV WS_DIR="/ros2_ws"
WORKDIR ${WS_DIR}

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -y \
    build-essential \
    cmake \
    git-all \
    software-properties-common \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
 && apt-get -y install \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-velodyne \
    ros-${ROS_DISTRO}-velodyne-driver \
 && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=dialog

