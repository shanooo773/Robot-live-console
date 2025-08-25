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
    xauth \
    && rm -rf /var/lib/apt/lists/*

# Install VNC and NoVNC dependencies for browser-based GUI access
RUN apt-get update && apt-get install -y \
    tigervnc-standalone-server \
    tigervnc-xorg-extension \
    websockify \
    python3-numpy \
    openbox \
    xterm \
    git \
    supervisor \
    && rm -rf /var/lib/apt/lists/*

# Install NoVNC
RUN git clone https://github.com/novnc/noVNC.git /opt/novnc && \
    git clone https://github.com/novnc/websockify /opt/novnc/utils/websockify && \
    chmod +x /opt/novnc/utils/novnc_proxy

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
RUN chmod +x /opt/simulation/record_simulation.sh && \
    chmod +x /opt/simulation/run_simulation.py

# Source ROS setup in bashrc for interactive sessions
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
    && echo "export ROS_PACKAGE_PATH=/opt/simulation:$ROS_PACKAGE_PATH" >> /root/.bashrc

# Set up VNC configuration
RUN mkdir -p /root/.vnc && \
    echo "gazebo" | vncpasswd -f > /root/.vnc/passwd && \
    chmod 600 /root/.vnc/passwd

# Create VNC xstartup script
RUN echo '#!/bin/bash\n\
export DISPLAY=:1\n\
source /opt/ros/noetic/setup.bash\n\
export ROS_PACKAGE_PATH=/opt/simulation:$ROS_PACKAGE_PATH\n\
export PYTHONPATH=/opt/simulation:$PYTHONPATH\n\
\n\
# Start window manager\n\
openbox-session &\n\
WM_PID=$!\n\
\n\
# Wait a moment for window manager\n\
sleep 2\n\
\n\
# Start roscore in background\n\
echo "Starting ROS core..."\n\
roscore &\n\
ROS_PID=$!\n\
sleep 5\n\
\n\
# Launch Gazebo with GUI\n\
echo "Starting Gazebo..."\n\
gazebo --verbose /opt/simulation/robots/worlds/empty.world &\n\
GAZEBO_PID=$!\n\
\n\
# Function to cleanup processes\n\
cleanup() {\n\
    echo "Cleaning up processes..."\n\
    kill $GAZEBO_PID $ROS_PID $WM_PID 2>/dev/null || true\n\
    exit 0\n\
}\n\
\n\
# Set up signal handlers\n\
trap cleanup SIGINT SIGTERM\n\
\n\
# Keep the session alive\n\
wait $GAZEBO_PID' > /root/.vnc/xstartup && \
    chmod +x /root/.vnc/xstartup

# Create startup script for VNC/NoVNC
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Source ROS environment\n\
source /opt/ros/noetic/setup.bash\n\
export ROS_PACKAGE_PATH=/opt/simulation:$ROS_PACKAGE_PATH\n\
export PYTHONPATH=/opt/simulation:$PYTHONPATH\n\
\n\
# Create necessary directories with proper permissions\n\
mkdir -p /tmp /output /workspace /root/.vnc\n\
chmod 777 /tmp /output /workspace\n\
\n\
# Start VNC server\n\
echo "Starting VNC server..."\n\
vncserver :1 -geometry 1024x768 -depth 24 -SecurityTypes None\n\
\n\
# Start NoVNC websocket proxy\n\
echo "Starting NoVNC web interface..."\n\
cd /opt/novnc\n\
./utils/websockify --web . 8080 localhost:5901 &\n\
\n\
echo "VNC server started on :5901"\n\
echo "NoVNC web interface available on port 8080"\n\
echo "Access via browser: http://localhost:8080"\n\
\n\
# Keep container running\n\
tail -f /dev/null' > /start-vnc.sh && chmod +x /start-vnc.sh

# Create entrypoint script that sources ROS
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/noetic/setup.bash\n\
export ROS_PACKAGE_PATH=/opt/simulation:$ROS_PACKAGE_PATH\n\
export PYTHONPATH=/opt/simulation:$PYTHONPATH\n\
# Create necessary directories with proper permissions\n\
mkdir -p /tmp /output /workspace\n\
chmod 777 /tmp /output /workspace\n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

# Expose VNC and NoVNC ports
EXPOSE 5901 8080

ENTRYPOINT ["/entrypoint.sh"]

# Default command - start VNC/NoVNC interface
CMD ["/start-vnc.sh"]
