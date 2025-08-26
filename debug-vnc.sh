#!/bin/bash
set -e

echo "🐛 VNC/NoVNC Debugging Script"
echo "=============================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

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
    print_status "Checking container status..."
    if docker compose ps gazebo | grep -q "Up"; then
        print_success "Container is running"
        return 0
    else
        print_error "Container is not running"
        return 1
    fi
}

# Function to check VNC server
check_vnc() {
    print_status "Checking VNC server..."
    if docker compose exec gazebo pgrep -f "Xvnc.*:1" > /dev/null 2>&1; then
        print_success "VNC server is running on display :1"
    else
        print_error "VNC server is not running"
        print_status "VNC server logs:"
        docker compose exec gazebo ls -la /root/.vnc/ || true
        docker compose exec gazebo cat /root/.vnc/*.log 2>/dev/null || true
    fi
}

# Function to check NoVNC websocket
check_novnc() {
    print_status "Checking NoVNC websocket..."
    if docker compose exec gazebo pgrep -f websockify > /dev/null 2>&1; then
        print_success "NoVNC websockify is running"
    else
        print_error "NoVNC websockify is not running"
    fi
}

# Function to check network connectivity
check_network() {
    print_status "Checking network connectivity..."
    
    # Check VNC port
    if nc -zv localhost 5901 2>/dev/null; then
        print_success "VNC port 5901 is accessible"
    else
        print_error "VNC port 5901 is not accessible"
    fi
    
    # Check NoVNC port  
    if nc -zv localhost 8080 2>/dev/null; then
        print_success "NoVNC port 8080 is accessible"
    else
        print_error "NoVNC port 8080 is not accessible"
    fi
    
    # Test NoVNC web interface
    if curl -s http://localhost:8080 > /dev/null; then
        print_success "NoVNC web interface is responding"
    else
        print_error "NoVNC web interface is not responding"
    fi
}

# Function to check Gazebo
check_gazebo() {
    print_status "Checking Gazebo status..."
    if docker compose exec gazebo pgrep -f gazebo > /dev/null 2>&1; then
        print_success "Gazebo is running"
        docker compose exec gazebo rostopic list | grep gazebo || print_warning "No Gazebo ROS topics found"
    else
        print_error "Gazebo is not running"
    fi
}

# Function to show container logs
show_logs() {
    print_status "Container logs (last 20 lines):"
    docker compose logs --tail=20 gazebo
}

# Function to test VNC connection
test_vnc_connection() {
    print_status "Testing VNC connection with vncviewer (if available)..."
    if command -v vncviewer >/dev/null 2>&1; then
        print_status "You can connect to VNC using: vncviewer localhost:5901"
        print_status "Password: gazebo"
    else
        print_warning "vncviewer not installed. Install with: sudo apt-get install tigervnc-viewer"
    fi
}

# Function to provide troubleshooting steps
provide_troubleshooting() {
    echo ""
    print_status "Troubleshooting Steps:"
    echo "1. Restart container: docker compose restart gazebo"
    echo "2. Check container logs: docker compose logs gazebo"
    echo "3. Access container shell: docker compose exec gazebo bash"
    echo "4. Test VNC manually: vncserver -list"
    echo "5. Test NoVNC: curl http://localhost:8080"
    echo "6. Check display: docker compose exec gazebo echo \$DISPLAY"
    echo "7. Test OpenGL: docker compose exec gazebo glxinfo | grep rendering"
    echo ""
    print_status "Manual VNC restart inside container:"
    echo "docker compose exec gazebo vncserver -kill :1"
    echo "docker compose exec gazebo vncserver :1 -geometry 1280x720 -depth 24 -SecurityTypes None"
    echo ""
    print_status "Access URLs:"
    echo "- NoVNC Web Interface: http://localhost:8080/vnc.html"
    echo "- Direct VNC: vnc://localhost:5901 (password: gazebo)"
}

# Main execution
main() {
    case "${1:-check}" in
        "check")
            check_container && check_vnc && check_novnc && check_network && check_gazebo
            ;;
        "logs")
            show_logs
            ;;
        "vnc")
            test_vnc_connection
            ;;
        "troubleshoot")
            provide_troubleshooting
            ;;
        "restart")
            print_status "Restarting container..."
            docker compose restart gazebo
            sleep 5
            $0 check
            ;;
        *)
            echo "Usage: $0 [check|logs|vnc|troubleshoot|restart]"
            echo "  check       - Run all diagnostic checks (default)"
            echo "  logs        - Show container logs"
            echo "  vnc         - Test VNC connection"
            echo "  troubleshoot - Show troubleshooting steps"
            echo "  restart     - Restart container and check"
            ;;
    esac
}

main "$@"