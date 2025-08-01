# Use ROS Noetic base image with Ubuntu 20.04
FROM osrf/ros:noetic-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    ffmpeg \
    x11vnc \
    xvfb \
    mesa-utils \
    libgl1-mesa-dri \
    gazebo11 \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-xacro \
    ros-noetic-turtlebot3 \
    ros-noetic-turtlebot3-msgs \
    ros-noetic-turtlebot3-simulations \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install \
    rospy \
    rospkg \
    geometry_msgs \
    sensor_msgs \
    std_msgs

# Create workspace
WORKDIR /opt/simulation
RUN mkdir -p /opt/simulation/catkin_ws/src

# Copy robot descriptions and simulation files
COPY robots/ /opt/simulation/robots/
COPY scripts/ /opt/simulation/scripts/

# Create main simulation runner script
COPY scripts/run_simulation.py /opt/simulation/run_simulation.py

# Set up ROS workspace
WORKDIR /opt/simulation/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Create entrypoint script
RUN echo '#!/bin/bash\n\
source /opt/ros/noetic/setup.bash\n\
source /opt/simulation/catkin_ws/devel/setup.bash\n\
export GAZEBO_MODEL_PATH=/opt/simulation/robots:$GAZEBO_MODEL_PATH\n\
export ROS_PACKAGE_PATH=/opt/simulation/robots:$ROS_PACKAGE_PATH\n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["bash"]