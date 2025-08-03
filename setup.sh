#!/bin/bash

set -e  # Exit on any error

echo "🤖 Robot Live Console Setup Script"
echo "====================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
BACKEND_PORT=8000
FRONTEND_PORT=3000  # Vite default port
DOCKER_IMAGE_NAME="robot-simulation"
DOCKER_TAG="latest"

# Function to print colored output
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

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to check if port is open (cross-platform)
is_port_open() {
    local port=$1
    local host=${2:-localhost}
    
    # Try multiple methods for cross-platform compatibility
    
    # Method 1: Use curl (most reliable for HTTP services)
    if command_exists curl; then
        if curl -s --connect-timeout 2 "http://${host}:${port}/" > /dev/null 2>&1; then
            return 0
        fi
    fi
    
    # Method 2: Use netcat if available
    if command_exists nc; then
        if nc -z "${host}" "${port}" 2>/dev/null; then
            return 0
        fi
    fi
    
    # Method 3: Use telnet as fallback (Windows compatible)
    if command_exists telnet; then
        if timeout 2 telnet "${host}" "${port}" 2>/dev/null | grep -q "Connected\|Escape character"; then
            return 0
        fi
    fi
    
    # Method 4: Check if something is listening on the port using netstat/ss
    if command_exists netstat; then
        if netstat -an 2>/dev/null | grep -q ":${port}.*LISTEN"; then
            return 0
        fi
    elif command_exists ss; then
        if ss -ln 2>/dev/null | grep -q ":${port}.*LISTEN"; then
            return 0
        fi
    fi
    
    return 1
}

# Function to wait for service with better logging
wait_for_service() {
    local service_name=$1
    local port=$2
    local timeout=${3:-30}
    local host=${4:-localhost}
    
    print_status "Waiting for ${service_name} to start on port ${port}..."
    
    for i in $(seq 1 $timeout); do
        if is_port_open "$port" "$host"; then
            print_success "${service_name} started successfully on port ${port}"
            return 0
        fi
        
        # More verbose logging every 5 seconds
        if [ $((i % 5)) -eq 0 ]; then
            print_status "Still waiting for ${service_name}... (${i}/${timeout}s)"
        fi
        
        sleep 1
    done
    
    print_error "${service_name} failed to start within ${timeout} seconds"
    print_status "Debugging info:"
    print_status "- Checking if port ${port} is open..."
    if command_exists netstat; then
        print_status "- Active connections on port ${port}:"
        netstat -an 2>/dev/null | grep ":${port}" || echo "  No connections found"
    fi
    if command_exists lsof; then
        print_status "- Processes using port ${port}:"
        lsof -i ":${port}" 2>/dev/null || echo "  No processes found"
    fi
    
    return 1
}

# Function to check system requirements
check_requirements() {
    print_status "Checking system requirements..."
    
    # Check Docker
    if ! command_exists docker; then
        print_error "Docker is not installed. Please install Docker first."
        echo "Visit: https://docs.docker.com/get-docker/"
        exit 1
    fi
    
    # Check Node.js
    if ! command_exists node; then
        print_error "Node.js is not installed. Please install Node.js first."
        echo "Visit: https://nodejs.org/"
        exit 1
    fi
    
    # Check npm
    if ! command_exists npm; then
        print_error "npm is not installed. Please install npm first."
        exit 1
    fi
    
    # Check Python3
    if ! command_exists python; then
        print_error "Python 3 is not installed. Please install Python 3 first."
        exit 1
    fi
    
    # Check pip3
    if ! command_exists pip3; then
        print_error "pip3 is not installed. Please install pip3 first."
        exit 1
    fi
    
    print_success "All requirements satisfied!"
}

# Function to load environment variables
load_env() {
    if [ -f .env ]; then
        print_status "Loading environment variables from .env"
        export $(cat .env | grep -v '^#' | xargs)
    else
        print_status "Creating .env from template"
        cp .env.template .env
        print_warning "Please review and modify .env file if needed"
    fi
}

# Function to build Docker image
build_docker_image() {
    # Skip Docker build if SKIP_DOCKER environment variable is set
    if [ "$SKIP_DOCKER" = "1" ] || [ "$SKIP_DOCKER" = "true" ]; then
        print_warning "Skipping Docker build (SKIP_DOCKER is set)"
        return 0
    fi
    
    print_status "Building Docker image for ROS simulation..."
    
    if docker images | grep -q "${DOCKER_IMAGE_NAME}.*${DOCKER_TAG}"; then
        print_warning "Docker image ${DOCKER_IMAGE_NAME}:${DOCKER_TAG} already exists"
        read -p "Do you want to rebuild it? (y/N): " rebuild
        if [[ ! $rebuild =~ ^[Yy]$ ]]; then
            print_status "Skipping Docker build"
            return 0
        fi
    fi
    
    print_status "This may take 10-15 minutes for the first build..."
    cd docker
    
    if docker build -t "${DOCKER_IMAGE_NAME}:${DOCKER_TAG}" .; then
        print_success "Docker image built successfully!"
    else
        print_error "Failed to build Docker image"
        exit 1
    fi
    
    cd ..
}

# Function to setup backend
setup_backend() {
    print_status "Setting up FastAPI backend..."
    
    cd backend
    
    # Create virtual environment if it doesn't exist
    if [ ! -d "venv" ]; then
        print_status "Creating Python virtual environment..."
        python -m venv venv
    fi
    
    # Activate virtual environment and install dependencies
    print_status "Installing Python dependencies..."
    # Cross-platform virtual environment activation
    if [ -f "venv/Scripts/activate" ]; then
        # Windows style
        source venv/Scripts/activate
    elif [ -f "venv/bin/activate" ]; then
        # Unix style
        source venv/bin/activate
    else
        print_error "Could not find virtual environment activation script"
        exit 1
    fi
    pip install -r requirements.txt
    
    # Create necessary directories
    mkdir -p videos temp
    
    cd ..
    print_success "Backend setup complete!"
}

# Function to setup frontend
setup_frontend() {
    print_status "Setting up React frontend..."
    
    cd frontend
    
    # Install npm dependencies
    print_status "Installing npm dependencies..."
    npm install
    
    cd ..
    print_success "Frontend setup complete!"
}

# Function to start backend server
start_backend() {
    print_status "Starting FastAPI backend server..."
    
    cd backend
    # Cross-platform virtual environment activation
    if [ -f "venv/Scripts/activate" ]; then
        # Windows style
        source venv/Scripts/activate
    elif [ -f "venv/bin/activate" ]; then
        # Unix style
        source venv/bin/activate
    else
        print_error "Could not find virtual environment activation script"
        exit 1
    fi

    
    # Start backend in background
    nohup python main.py > backend.log 2>&1 &
    BACKEND_PID=$!
    echo $BACKEND_PID > backend.pid
    
    cd ..
    
    # Wait for backend to start with improved detection
    if ! wait_for_service "Backend" "$BACKEND_PORT" 30; then
        print_error "Backend startup failed. Check backend/backend.log for details."
        return 1
    fi
    
    return 0
}

# Function to start frontend server
start_frontend() {
    print_status "Starting React frontend server..."
    
    cd frontend
    
    # Start frontend in background
    nohup npm run dev > frontend.log 2>&1 &
    FRONTEND_PID=$!
    echo $FRONTEND_PID > frontend.pid
    
    cd ..
    
    # Wait a moment for Vite to start and create log
    sleep 3
    
    # Try to extract the actual port from the Vite log
    actual_port="$FRONTEND_PORT"
    if [ -f frontend/frontend.log ]; then
        # Look for "Local: http://localhost:XXXX/" in the log
        detected_port=$(grep -o "Local:.*http://localhost:[0-9]*/" frontend/frontend.log | grep -o "[0-9]*" | head -1)
        if [ -n "$detected_port" ]; then
            actual_port="$detected_port"
            if [ "$actual_port" != "$FRONTEND_PORT" ]; then
                print_warning "Vite started on port ${actual_port} instead of ${FRONTEND_PORT}"
            fi
        fi
    fi
    
    # Wait for frontend to start with improved detection and longer timeout for Vite
    # Vite can take longer to start, especially on first run
    if ! wait_for_service "Frontend (Vite)" "$actual_port" 60; then
        print_error "Frontend startup failed. Check frontend/frontend.log for details."
        print_status "Vite output from log:"
        if [ -f frontend/frontend.log ]; then
            tail -20 frontend/frontend.log
        fi
        return 1
    fi
    
    # Update the FRONTEND_PORT variable for subsequent use
    FRONTEND_PORT="$actual_port"
    
    return 0
}

# Function to stop servers
stop_servers() {
    print_status "Stopping servers..."
    
    # Stop backend
    if [ -f backend/backend.pid ]; then
        BACKEND_PID=$(cat backend/backend.pid)
        if kill -0 $BACKEND_PID 2>/dev/null; then
            kill $BACKEND_PID
            rm backend/backend.pid
            print_success "Backend stopped"
        fi
    fi
    
    # Stop frontend
    if [ -f frontend/frontend.pid ]; then
        FRONTEND_PID=$(cat frontend/frontend.pid)
        if kill -0 $FRONTEND_PID 2>/dev/null; then
            kill $FRONTEND_PID
            rm frontend/frontend.pid
            print_success "Frontend stopped"
        fi
    fi
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [COMMAND]"
    echo ""
    echo "Commands:"
    echo "  start    - Start the application (default)"
    echo "  stop     - Stop running servers"
    echo "  build    - Build Docker image only"
    echo "  setup    - Setup dependencies only"
    echo "  status   - Show status of running servers"
    echo "  help     - Show this help message"
}

# Function to show status
show_status() {
    print_status "Checking server status..."
    
    # Check backend
    if is_port_open "$BACKEND_PORT"; then
        print_success "Backend is running on http://localhost:${BACKEND_PORT}"
    else
        print_warning "Backend is not running"
    fi
    
    # Check frontend
    if is_port_open "$FRONTEND_PORT"; then
        print_success "Frontend is running on http://localhost:${FRONTEND_PORT}"
    else
        print_warning "Frontend is not running"
    fi
    
    # Check Docker image
    if docker images | grep -q "${DOCKER_IMAGE_NAME}.*${DOCKER_TAG}"; then
        print_success "Docker image ${DOCKER_IMAGE_NAME}:${DOCKER_TAG} is available"
    else
        print_warning "Docker image ${DOCKER_IMAGE_NAME}:${DOCKER_TAG} is not built"
    fi
}

# Main execution
main() {
    local command=${1:-start}
    
    case $command in
        "start")
            check_requirements
            load_env
            build_docker_image
            setup_backend
            setup_frontend
            start_backend
            start_frontend
            echo ""
            print_success "🎉 Robot Live Console is ready!"
            echo ""
            echo "  Frontend: http://localhost:${FRONTEND_PORT}"
            echo "  Backend:  http://localhost:${BACKEND_PORT}"
            echo ""
            print_status "Use '$0 stop' to stop the servers"
            print_status "Check logs: backend/backend.log and frontend/frontend.log"
            ;;
        "stop")
            stop_servers
            ;;
        "build")
            check_requirements
            load_env
            build_docker_image
            ;;
        "setup")
            check_requirements
            load_env
            setup_backend
            setup_frontend
            ;;
        "status")
            load_env
            show_status
            ;;
        "help"|"--help"|"-h")
            show_usage
            ;;
        *)
            print_error "Unknown command: $command"
            show_usage
            exit 1
            ;;
    esac
}

# Trap to cleanup on exit
trap 'echo ""; print_status "Cleaning up..."; stop_servers; exit' INT TERM

# Run main function
main "$@"