#!/bin/bash
set -e

echo "🧪 VNC/NoVNC Test Environment Setup"
echo "==================================="

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

# Function to build test container
build_test_container() {
    print_status "Building minimal VNC test container..."
    
    if docker build -f docker/Dockerfile.test-vnc -t vnc-test docker/ --progress=plain; then
        print_success "Test container built successfully"
        return 0
    else
        print_error "Failed to build test container"
        return 1
    fi
}

# Function to run test container
run_test_container() {
    print_status "Starting VNC test container..."
    
    # Stop any existing test container
    docker stop vnc-test-container 2>/dev/null || true
    docker rm vnc-test-container 2>/dev/null || true
    
    # Run the test container
    docker run -d \
        --name vnc-test-container \
        -p 8080:8080 \
        -p 5901:5901 \
        vnc-test
    
    if [ $? -eq 0 ]; then
        print_success "Test container started"
        return 0
    else
        print_error "Failed to start test container"
        return 1
    fi
}

# Function to wait for services
wait_for_services() {
    print_status "Waiting for VNC/NoVNC services to start..."
    
    # Wait up to 30 seconds for services
    for i in {1..30}; do
        if nc -z localhost 8080 2>/dev/null && nc -z localhost 5901 2>/dev/null; then
            print_success "Services are ready"
            return 0
        fi
        sleep 1
        echo -n "."
    done
    
    print_error "Services failed to start within 30 seconds"
    return 1
}

# Function to test VNC connectivity
test_vnc_connectivity() {
    print_status "Testing VNC connectivity..."
    
    # Test VNC port
    if nc -z localhost 5901; then
        print_success "VNC port 5901 is accessible"
    else
        print_error "VNC port 5901 is not accessible"
        return 1
    fi
    
    # Test NoVNC web interface
    if curl -s http://localhost:8080 > /dev/null; then
        print_success "NoVNC web interface is responding"
    else
        print_error "NoVNC web interface is not responding"
        return 1
    fi
    
    # Test NoVNC specific endpoint
    if curl -s http://localhost:8080/vnc.html | grep -q "noVNC"; then
        print_success "NoVNC client page is accessible"
    else
        print_warning "NoVNC client page may not be fully loaded"
    fi
    
    return 0
}

# Function to show container logs
show_logs() {
    print_status "Container logs:"
    docker logs vnc-test-container --tail=20
}

# Function to cleanup
cleanup() {
    print_status "Cleaning up test environment..."
    docker stop vnc-test-container 2>/dev/null || true
    docker rm vnc-test-container 2>/dev/null || true
    print_success "Cleanup complete"
}

# Function to show access information
show_access_info() {
    echo ""
    print_status "🌐 Access Information:"
    echo "======================================"
    echo "🔗 NoVNC Web Interface: http://localhost:8080/vnc.html"
    echo "🔑 VNC Password: gazebo"
    echo "📺 Direct VNC: vnc://localhost:5901"
    echo "======================================"
    echo ""
    print_status "💡 What you should see:"
    echo "- A desktop environment with openbox window manager"
    echo "- An xterm terminal window titled 'VNC Test - Success!'"
    echo "- Ability to interact with mouse and keyboard"
    echo ""
    print_status "🔍 Debugging commands:"
    echo "- Check container status: docker ps | grep vnc-test"
    echo "- View logs: docker logs vnc-test-container"
    echo "- Access container: docker exec -it vnc-test-container bash"
    echo "- Test inside container: export DISPLAY=:1 && xset q"
}

# Main function
main() {
    case "${1:-all}" in
        "build")
            build_test_container
            ;;
        "run")
            run_test_container && wait_for_services
            ;;
        "test")
            test_vnc_connectivity
            ;;
        "logs")
            show_logs
            ;;
        "info")
            show_access_info
            ;;
        "cleanup")
            cleanup
            ;;
        "all")
            print_status "Running complete VNC test sequence..."
            if build_test_container && run_test_container && wait_for_services && test_vnc_connectivity; then
                print_success "🎉 VNC test setup completed successfully!"
                show_access_info
                echo ""
                print_status "Test container is running. Use './test-vnc.sh cleanup' to stop."
            else
                print_error "❌ VNC test setup failed"
                show_logs
                cleanup
                exit 1
            fi
            ;;
        *)
            echo "Usage: $0 [build|run|test|logs|info|cleanup|all]"
            echo "  build   - Build the test container"
            echo "  run     - Run the test container"
            echo "  test    - Test VNC connectivity"
            echo "  logs    - Show container logs"
            echo "  info    - Show access information"
            echo "  cleanup - Stop and remove test container"
            echo "  all     - Run complete test sequence (default)"
            ;;
    esac
}

main "$@"