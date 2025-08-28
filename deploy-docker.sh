#!/bin/bash

# Docker Deployment Script for Robot Console
# This script deploys the Robot Console using Docker Compose

set -e

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

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to check Docker and Docker Compose
check_dependencies() {
    print_status "Checking dependencies..."
    
    if ! command_exists docker; then
        print_error "Docker is not installed. Please install Docker first."
        print_status "Visit: https://docs.docker.com/get-docker/"
        exit 1
    fi
    
    if ! command_exists docker-compose && ! docker compose version >/dev/null 2>&1; then
        print_error "Docker Compose is not available. Please install Docker Compose."
        print_status "Visit: https://docs.docker.com/compose/install/"
        exit 1
    fi
    
    print_success "Dependencies check passed"
}

# Function to create .env file if not exists
create_env_file() {
    if [ ! -f ".env" ]; then
        print_status "Creating .env file from template..."
        cp .env.template .env
        
        print_warning "Please edit .env file with your configuration:"
        print_warning "  - Set CORS_ORIGINS for your domain"
        print_warning "  - Set JWT_SECRET to a secure random value"
        print_warning "  - Configure other settings as needed"
        
        read -p "Press Enter to continue after editing .env file..."
    else
        print_success ".env file already exists"
    fi
}

# Function to build and start services
deploy_services() {
    local action=${1:-up}
    
    print_status "Building Docker images..."
    if command_exists docker-compose; then
        docker-compose build
    else
        docker compose build
    fi
    
    print_status "Starting services..."
    if command_exists docker-compose; then
        docker-compose $action -d
    else
        docker compose $action -d
    fi
    
    print_status "Waiting for services to start..."
    sleep 10
}

# Function to check service health
check_service_health() {
    print_status "Checking service health..."
    
    # Check backend health
    for i in {1..30}; do
        if curl -sf http://localhost:${BACKEND_PORT:-8000}/health >/dev/null 2>&1; then
            print_success "Backend is healthy"
            break
        fi
        if [ $i -eq 30 ]; then
            print_error "Backend health check failed"
            return 1
        fi
        sleep 2
    done
    
    # Check frontend health
    for i in {1..30}; do
        if curl -sf http://localhost:${FRONTEND_PORT:-3000} >/dev/null 2>&1; then
            print_success "Frontend is healthy"
            break
        fi
        if [ $i -eq 30 ]; then
            print_error "Frontend health check failed"
            return 1
        fi
        sleep 2
    done
}

# Function to show service status
show_status() {
    print_status "Service Status:"
    if command_exists docker-compose; then
        docker-compose ps
    else
        docker compose ps
    fi
    
    echo ""
    print_status "Service Logs (last 10 lines):"
    echo "=== Backend Logs ==="
    if command_exists docker-compose; then
        docker-compose logs --tail=10 backend
    else
        docker compose logs --tail=10 backend
    fi
    
    echo ""
    echo "=== Frontend Logs ==="
    if command_exists docker-compose; then
        docker-compose logs --tail=10 frontend
    else
        docker compose logs --tail=10 frontend
    fi
}

# Function to stop services
stop_services() {
    print_status "Stopping services..."
    if command_exists docker-compose; then
        docker-compose down
    else
        docker compose down
    fi
    print_success "Services stopped"
}

# Function to show service info
show_deployment_info() {
    print_success "🎉 Robot Console deployed successfully with Docker!"
    echo ""
    print_status "📱 Frontend: http://localhost:${FRONTEND_PORT:-3000}"
    print_status "🔧 Backend API: http://localhost:${BACKEND_PORT:-8000}"
    print_status "📊 API Docs: http://localhost:${BACKEND_PORT:-8000}/docs"
    if [ -f "docker-compose.yml" ] && grep -q "nginx:" docker-compose.yml; then
        print_status "🌐 Nginx Proxy: http://localhost:80"
    fi
    echo ""
    print_status "🐳 Docker Commands:"
    print_status "  View logs: docker-compose logs -f [service]"
    print_status "  Restart: docker-compose restart [service]"
    print_status "  Stop: docker-compose down"
    print_status "  Update: docker-compose build && docker-compose up -d"
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [COMMAND]"
    echo ""
    echo "Commands:"
    echo "  deploy, up     - Deploy the application (default)"
    echo "  stop, down     - Stop the application"
    echo "  restart        - Restart the application"
    echo "  status         - Show service status and logs"
    echo "  build          - Build Docker images only"
    echo "  logs [service] - Show logs for all services or specific service"
    echo "  clean          - Remove containers and images"
    echo "  -h, --help     - Show this help"
    echo ""
    echo "Examples:"
    echo "  $0                    # Deploy the application"
    echo "  $0 stop               # Stop the application"
    echo "  $0 logs backend       # Show backend logs"
    echo "  $0 status             # Show service status"
}

# Load environment variables
if [ -f ".env" ]; then
    source .env
fi

# Main function
main() {
    local command=${1:-deploy}
    
    case $command in
        deploy|up)
            print_status "🚀 Deploying Robot Console with Docker"
            check_dependencies
            create_env_file
            deploy_services "up"
            check_service_health
            show_deployment_info
            ;;
        stop|down)
            stop_services
            ;;
        restart)
            print_status "Restarting services..."
            deploy_services "restart"
            check_service_health
            print_success "Services restarted"
            ;;
        status)
            show_status
            ;;
        build)
            check_dependencies
            print_status "Building Docker images..."
            if command_exists docker-compose; then
                docker-compose build
            else
                docker compose build
            fi
            print_success "Images built successfully"
            ;;
        logs)
            local service=${2:-""}
            if [ -n "$service" ]; then
                if command_exists docker-compose; then
                    docker-compose logs -f "$service"
                else
                    docker compose logs -f "$service"
                fi
            else
                if command_exists docker-compose; then
                    docker-compose logs -f
                else
                    docker compose logs -f
                fi
            fi
            ;;
        clean)
            print_warning "This will remove all containers, images, and volumes. Are you sure? (y/N)"
            read -r response
            if [ "$response" = "y" ] || [ "$response" = "Y" ]; then
                if command_exists docker-compose; then
                    docker-compose down -v --rmi all
                else
                    docker compose down -v --rmi all
                fi
                print_success "Cleanup completed"
            else
                print_status "Cleanup cancelled"
            fi
            ;;
        -h|--help)
            show_usage
            ;;
        *)
            print_error "Unknown command: $command"
            show_usage
            exit 1
            ;;
    esac
}

# Run main function
main "$@"