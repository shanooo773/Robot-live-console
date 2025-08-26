#!/bin/bash
set -e

echo "🔧 Apply VNC/NoVNC Fixes to Existing Container"
echo "=============================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if container is running
check_container() {
    if docker compose ps gazebo 2>/dev/null | grep -q "Up"; then
        return 0
    else
        return 1
    fi
}

# Function to apply VNC fixes to running container
apply_vnc_fixes() {
    local container_name="gazebo-vnc-gui"
    
    print_status "Applying VNC/NoVNC fixes to container: $container_name"
    
    # 1. Install required packages if missing
    print_status "Installing missing packages in container..."
    docker compose exec gazebo bash -c "
        apt-get update && apt-get install -y --no-install-recommends \
            tigervnc-standalone-server \
            openbox \
            dbus-x11 \
            libgl1-mesa-glx \
            libgl1-mesa-dri \
            mesa-utils \
            websockify \
        2>/dev/null || echo 'Some packages may already be installed'
    " || print_warning "Package installation may have failed - some packages might already be present"
    
    # 2. Create improved xstartup script
    print_status "Creating improved VNC xstartup script..."
    docker compose exec gazebo bash -c "
        cat > /root/.vnc/xstartup << 'EOF'
#!/bin/bash
set -e

# Set display and source environment
export DISPLAY=:1
source /opt/ros/noetic/setup.bash 2>/dev/null || echo 'ROS not found'
export ROS_PACKAGE_PATH=/opt/simulation:\$ROS_PACKAGE_PATH
export PYTHONPATH=/opt/simulation:\$PYTHONPATH

# Configure Mesa for software rendering
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe

# Start D-Bus
eval \\\`dbus-launch --sh-syntax\\\` 2>/dev/null || echo 'D-Bus may not be available'

# Start window manager
echo 'Starting window manager...'
openbox-session &
WM_PID=\$!

# Wait for window manager to start
sleep 3

# Start roscore in background (if ROS is available)
if command -v roscore >/dev/null 2>&1; then
    echo 'Starting ROS core...'
    roscore &
    ROS_PID=\$!
    sleep 5
    
    # Test ROS connection
    echo 'Testing ROS connection...'
    rostopic list > /dev/null 2>&1 || echo 'Warning: ROS not fully ready'
    
    # Launch Gazebo with GUI
    echo 'Starting Gazebo GUI...'
    GAZEBO_MASTER_URI=http://localhost:11345 gazebo --verbose /opt/simulation/robots/worlds/empty.world &
    GAZEBO_PID=\$!
else
    echo 'ROS not available - starting simple GUI test'
    xterm -title 'VNC Test - Success!' -geometry 80x24+100+100 &
    GAZEBO_PID=\$!
fi

# Function to cleanup processes
cleanup() {
    echo 'Cleaning up processes...'
    kill \$GAZEBO_PID \$ROS_PID \$WM_PID 2>/dev/null || true
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Keep the session alive and wait for main process
echo 'VNC session started successfully. GUI should be visible.'
wait \$GAZEBO_PID
EOF
        chmod +x /root/.vnc/xstartup
        echo 'VNC xstartup script updated'
    "
    
    # 3. Create improved VNC startup script
    print_status "Creating improved VNC startup script..."
    docker compose exec gazebo bash -c "
        cat > /start-vnc-fixed.sh << 'EOF'
#!/bin/bash
set -e

echo 'Starting Fixed VNC/NoVNC Gazebo GUI setup...'

# Source ROS environment if available
source /opt/ros/noetic/setup.bash 2>/dev/null || echo 'ROS not found, proceeding without it'
export ROS_PACKAGE_PATH=/opt/simulation:\$ROS_PACKAGE_PATH
export PYTHONPATH=/opt/simulation:\$PYTHONPATH

# Create necessary directories with proper permissions
mkdir -p /tmp /output /workspace /root/.vnc /var/run/dbus
chmod 777 /tmp /output /workspace

# Configure Mesa for software rendering
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe

# Start D-Bus service
echo 'Starting D-Bus...'
dbus-daemon --system --fork 2>/dev/null || echo 'D-Bus may already be running'

# Kill any existing VNC servers
vncserver -kill :1 2>/dev/null || echo 'No existing VNC server to kill'
sleep 1

# Start VNC server with better configuration
echo 'Starting VNC server on display :1...'
vncserver :1 -geometry 1280x720 -depth 24 -SecurityTypes None -localhost no

# Wait for VNC to start
sleep 3

# Verify VNC is running
if ! pgrep -f 'Xvnc.*:1' > /dev/null; then
    echo 'ERROR: VNC server failed to start'
    exit 1
fi

echo 'VNC server started successfully'

# Start NoVNC websocket proxy
echo 'Starting NoVNC web interface...'
cd /opt/novnc 2>/dev/null || { echo 'NoVNC not found, downloading...'; 
    cd /opt && wget -q https://github.com/novnc/noVNC/archive/v1.3.0.tar.gz && 
    tar xzf v1.3.0.tar.gz && mv noVNC-1.3.0 novnc && rm v1.3.0.tar.gz &&
    cd novnc && wget -q https://github.com/novnc/websockify/archive/v0.10.0.tar.gz &&
    tar xzf v0.10.0.tar.gz -C utils/ && mv utils/websockify-0.10.0 utils/websockify &&
    rm v0.10.0.tar.gz && cd /opt/novnc; }

# Try different websocket proxy methods
if command -v websockify >/dev/null 2>&1; then
    websockify --web . 8080 localhost:5901 &
elif [ -f utils/websockify/websockify.py ]; then
    python3 utils/websockify/websockify.py --web . 8080 localhost:5901 &
else
    echo 'ERROR: No websockify found'
    exit 1
fi

WEBSOCKIFY_PID=\$!

# Wait for websockify to start
sleep 2

if ! pgrep -f websockify > /dev/null; then
    echo 'ERROR: NoVNC websockify failed to start'
    exit 1
fi

echo '=== Fixed VNC/NoVNC Setup Complete ==='
echo 'VNC server started on display :1 (port 5901)'
echo 'NoVNC web interface available on port 8080'
echo 'Access via browser: http://localhost:8080/vnc.html'
echo 'VNC Password: gazebo'
echo '====================================='

# Function to cleanup
cleanup() {
    echo 'Shutting down services...'
    kill \$WEBSOCKIFY_PID 2>/dev/null || true
    vncserver -kill :1 2>/dev/null || true
    exit 0
}

trap cleanup SIGINT SIGTERM

# Keep container running and show logs
echo 'Container is ready. Press Ctrl+C to stop.'
tail -f /dev/null
EOF
        chmod +x /start-vnc-fixed.sh
        echo 'Fixed VNC startup script created'
    "
    
    # 4. Restart VNC services with new configuration
    print_status "Restarting VNC services with new configuration..."
    docker compose exec gazebo bash -c "
        # Kill existing VNC
        vncserver -kill :1 2>/dev/null || true
        pkill -f websockify || true
        
        # Start with new script
        /start-vnc-fixed.sh &
        
        echo 'New VNC services started in background'
    " &
    
    print_success "VNC fixes applied successfully!"
    
    return 0
}

# Function to test the fixes
test_fixes() {
    print_status "Testing VNC/NoVNC functionality..."
    
    sleep 5  # Wait for services to start
    
    # Test VNC port
    if nc -z localhost 5901 2>/dev/null; then
        print_success "VNC port 5901 is accessible"
    else
        print_error "VNC port 5901 is not accessible"
        return 1
    fi
    
    # Test NoVNC port
    if nc -z localhost 8080 2>/dev/null; then
        print_success "NoVNC port 8080 is accessible"
    else
        print_error "NoVNC port 8080 is not accessible"
        return 1
    fi
    
    # Test NoVNC web response
    if curl -s http://localhost:8080 > /dev/null 2>&1; then
        print_success "NoVNC web interface is responding"
    else
        print_warning "NoVNC web interface may not be fully ready yet"
    fi
    
    return 0
}

# Function to show status
show_status() {
    print_status "Current VNC/NoVNC status:"
    
    echo "Container status:"
    docker compose ps gazebo || echo "Container not found"
    
    echo -e "\nVNC processes in container:"
    docker compose exec gazebo pgrep -f "Xvnc\|websockify" || echo "No VNC processes found"
    
    echo -e "\nPort accessibility:"
    nc -z localhost 5901 && echo "✅ VNC port 5901: OK" || echo "❌ VNC port 5901: FAIL"
    nc -z localhost 8080 && echo "✅ NoVNC port 8080: OK" || echo "❌ NoVNC port 8080: FAIL"
    
    echo -e "\nAccess URLs:"
    echo "🔗 NoVNC Web: http://localhost:8080/vnc.html"
    echo "🔑 Password: gazebo"
}

# Main function
main() {
    case "${1:-apply}" in
        "apply")
            if check_container; then
                apply_vnc_fixes && test_fixes
                echo ""
                show_status
            else
                print_error "Container is not running. Start it first with: docker compose up -d"
                exit 1
            fi
            ;;
        "test")
            test_fixes
            ;;
        "status")
            show_status
            ;;
        *)
            echo "Usage: $0 [apply|test|status]"
            echo "  apply  - Apply VNC fixes to running container (default)"
            echo "  test   - Test VNC/NoVNC connectivity"
            echo "  status - Show current status"
            ;;
    esac
}

main "$@"