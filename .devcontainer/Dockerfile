FROM osrf/ros:humble-desktop-full

SHELL [ "/bin/bash" , "-c" ]

# Upgrade all packages
RUN sudo apt update && sudo apt upgrade -y

# Install essential packages
RUN sudo apt install -y wget \
    software-properties-common \
    python3-pip \
    xauth \
    python3-venv \
    git \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Create directory for workspace
WORKDIR /sb-robot

# Copy packages.repos if needed
# COPY packages.repos .

# Install dependencies and build workspace if needed
# RUN vcs import < packages.repos; \
#     rosdep install --from-paths src --ignore-src -r -y; \
#     source /opt/ros/${ROS_DISTRO}/setup.bash; \
#     colcon build;

# Add sourcing ROS setup.bash to .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Set default user to root for now
USER root
