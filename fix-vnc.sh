#!/bin/bash
set -e

echo "🔧 VNC/NoVNC Configuration Fix Script"
echo "======================================"

# This script addresses the main VNC/NoVNC issues without rebuilding

# Function to create an improved VNC startup script
create_vnc_startup() {
    cat > /tmp/xstartup << 'EOF'
#!/bin/bash
set -e

# Set display and source environment
export DISPLAY=:1
source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH=/opt/simulation:$ROS_PACKAGE_PATH
export PYTHONPATH=/opt/simulation:$PYTHONPATH

# Configure Mesa for software rendering
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe

# Start D-Bus
eval `dbus-launch --sh-syntax`

# Start window manager
echo "Starting window manager..."
openbox-session &
WM_PID=$!

# Wait for window manager to start
sleep 3

# Start roscore in background
echo "Starting ROS core..."
roscore &
ROS_PID=$!

# Wait for roscore to start
sleep 5

# Test ROS connection
echo "Testing ROS connection..."
rostopic list > /dev/null 2>&1 || echo "Warning: ROS not fully ready"

# Launch Gazebo with GUI - use gzclient specifically for GUI
echo "Starting Gazebo GUI..."
GAZEBO_MASTER_URI=http://localhost:11345 gazebo --verbose /opt/simulation/robots/worlds/empty.world &
GAZEBO_PID=$!

# Function to cleanup processes
cleanup() {
    echo "Cleaning up processes..."
    kill $GAZEBO_PID $ROS_PID $WM_PID 2>/dev/null || true
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Keep the session alive and wait for Gazebo
echo "VNC session started successfully. Gazebo GUI should be visible."
wait $GAZEBO_PID
EOF
    chmod +x /tmp/xstartup
}

# Function to create improved VNC/NoVNC startup script
create_vnc_service() {
    cat > /tmp/start-vnc-fixed.sh << 'EOF'
#!/bin/bash
set -e

echo "Starting VNC/NoVNC Gazebo GUI setup..."

# Source ROS environment
source /opt/ros/noetic/setup.bash || echo "Warning: ROS not found"
export ROS_PACKAGE_PATH=/opt/simulation:$ROS_PACKAGE_PATH
export PYTHONPATH=/opt/simulation:$PYTHONPATH

# Create necessary directories with proper permissions
mkdir -p /tmp /output /workspace /root/.vnc /var/run/dbus
chmod 777 /tmp /output /workspace

# Configure Mesa for software rendering
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe

# Start D-Bus service
echo "Starting D-Bus..."
dbus-daemon --system --fork || true

# Kill any existing VNC servers
vncserver -kill :1 2>/dev/null || true
sleep 1

# Start VNC server with better configuration
echo "Starting VNC server on display :1..."
vncserver :1 -geometry 1280x720 -depth 24 -SecurityTypes None -localhost no

# Wait for VNC to start
sleep 3

# Verify VNC is running
if ! pgrep -f "Xvnc.*:1" > /dev/null; then
    echo "ERROR: VNC server failed to start"
    exit 1
fi

echo "VNC server started successfully"

# Start NoVNC websocket proxy
echo "Starting NoVNC web interface..."
cd /opt/novnc || { echo "NoVNC not found at /opt/novnc"; exit 1; }
./utils/websockify --web . 8080 localhost:5901 &
WEBSOCKIFY_PID=$!

# Wait for websockify to start
sleep 2

if ! pgrep -f websockify > /dev/null; then
    echo "ERROR: NoVNC websockify failed to start"
    exit 1
fi

echo "=== VNC/NoVNC Setup Complete ==="
echo "VNC server started on display :1 (port 5901)"
echo "NoVNC web interface available on port 8080"
echo "Access via browser: http://localhost:8080/vnc.html"
echo "VNC Password: gazebo"
echo "==============================="

# Function to cleanup
cleanup() {
    echo "Shutting down services..."
    kill $WEBSOCKIFY_PID 2>/dev/null || true
    vncserver -kill :1 2>/dev/null || true
    exit 0
}

trap cleanup SIGINT SIGTERM

# Keep container running and show logs
echo "Container is ready. Press Ctrl+C to stop."
tail -f /dev/null
EOF
    chmod +x /tmp/start-vnc-fixed.sh
}

# Function to create test scripts
create_test_scripts() {
    # Test VNC functionality
    cat > /tmp/test-vnc.sh << 'EOF'
#!/bin/bash
echo "Testing VNC setup..."

# Check if VNC server is running
if pgrep -f "Xvnc.*:1" > /dev/null; then
    echo "✅ VNC server is running"
else
    echo "❌ VNC server is not running"
fi

# Check if NoVNC websocket is running
if pgrep -f websockify > /dev/null; then
    echo "✅ NoVNC websockify is running"
else
    echo "❌ NoVNC websockify is not running"
fi

# Test display
export DISPLAY=:1
if xset q > /dev/null 2>&1; then
    echo "✅ Display :1 is accessible"
else
    echo "❌ Display :1 is not accessible"
fi

# Test OpenGL
if command -v glxinfo >/dev/null 2>&1; then
    if glxinfo | grep -q "software rendering\|llvmpipe"; then
        echo "✅ Software rendering is working"
    else
        echo "⚠️  Hardware rendering (may not work in container)"
    fi
else
    echo "⚠️  glxinfo not available"
fi

# Test Gazebo
if pgrep -f gazebo > /dev/null; then
    echo "✅ Gazebo is running"
else
    echo "❌ Gazebo is not running"
fi
EOF
    chmod +x /tmp/test-vnc.sh
}

# Main execution
main() {
    echo "Creating improved VNC/NoVNC configuration files..."
    
    create_vnc_startup
    create_vnc_service
    create_test_scripts
    
    echo ""
    echo "✅ Configuration files created in /tmp:"
    echo "  - xstartup: Improved VNC startup script"
    echo "  - start-vnc-fixed.sh: Fixed VNC/NoVNC service startup"
    echo "  - test-vnc.sh: Test script for debugging"
    echo ""
    echo "To apply these fixes to a running container:"
    echo "1. Copy scripts: docker compose cp /tmp/xstartup gazebo:/root/.vnc/xstartup"
    echo "2. Copy startup: docker compose cp /tmp/start-vnc-fixed.sh gazebo:/start-vnc-fixed.sh"
    echo "3. Restart container: docker compose restart gazebo"
    echo ""
    echo "Or build a new container with these improvements embedded."
}

main "$@"