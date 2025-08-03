# Use Ubuntu 20.04 as base for ROS Noetic compatibility
FROM ubuntu:20.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC
ENV ROS_DISTRO=noetic
ENV ROS_ROOT=/opt/ros/noetic
ENV ROS_PACKAGE_PATH=/opt/ros/noetic/share
ENV ROS_PYTHON_VERSION=3
ENV PYTHONPATH=/opt/simulation:$PYTHONPATH
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    wget \
    curl \
    gnupg2 \
    lsb-release \
    ca-certificates \
    software-properties-common \
    apt-transport-https \
    && rm -rf /var/lib/apt/lists/*

# Add ROS repository
RUN wget -O - https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

# Install ROS Noetic Desktop Full + Gazebo + additional tools
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-effort-controllers \
    ros-noetic-joint-state-controller \
    ros-noetic-position-controllers \
    ros-noetic-velocity-controllers \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# Install video recording dependencies
RUN apt-get update && apt-get install -y \
    xvfb \
    ffmpeg \
    x11-utils \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Create simulation workspace
WORKDIR /opt/simulation
RUN mkdir -p /opt/simulation/robots /opt/simulation/scripts /opt/simulation/launch /output

# Copy robot descriptions and simulation files
COPY robots/ /opt/simulation/robots/
COPY scripts/ /opt/simulation/scripts/

# Create launch files directory and copy launch files
COPY launch/ /opt/simulation/launch/ 

# Create main simulation runner script
COPY scripts/record_simulation.sh /opt/simulation/record_simulation.sh
COPY scripts/run_simulation.py /opt/simulation/run_simulation.py

# Make scripts executable
RUN chmod +x /opt/simulation/record_simulation.sh

# Source ROS setup in bashrc for interactive sessions
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
    && echo "export ROS_PACKAGE_PATH=/opt/simulation:$ROS_PACKAGE_PATH" >> /root/.bashrc

# Create entrypoint script that sources ROS
RUN echo '#!/bin/bash\n\
source /opt/ros/noetic/setup.bash\n\
export ROS_PACKAGE_PATH=/opt/simulation:$ROS_PACKAGE_PATH\n\
export PYTHONPATH=/opt/simulation:$PYTHONPATH\n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["bash", "/opt/simulation/record_simulation.sh", "--help"]
