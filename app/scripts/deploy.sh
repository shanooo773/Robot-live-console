#!/bin/bash

set -e  # Exit on any error

echo "🚀 Robot Live Console App Deployment Script"
echo "==========================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values (will be overridden by .env if present)
BACKEND_PORT=8000
FRONTEND_PORT=5173
PRODUCTION_MODE=false

# Load environment variables if .env exists
if [ -f ".env" ]; then
    print_status "Loading environment variables from .env..."
    export $(grep -v '^#' .env | xargs)
    BACKEND_PORT=${BACKEND_PORT:-8000}
    FRONTEND_PORT=${FRONTEND_PORT:-5173}
    PRODUCTION_MODE=${PRODUCTION_MODE:-false}
fi

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

# Function to check if port is in use
is_port_in_use() {
    local port=$1
    if command_exists lsof; then
        lsof -Pi :$port -sTCP:LISTEN -t >/dev/null 2>&1
    elif command_exists netstat; then
        netstat -ln | grep ":$port " >/dev/null 2>&1
    elif command_exists ss; then
        ss -ln | grep ":$port " >/dev/null 2>&1
    else
        # Fallback: try to connect to the port
        (echo >/dev/tcp/localhost/$port) >/dev/null 2>&1
    fi
}

# Function to find an available port
find_available_port() {
    local start_port=$1
    local max_attempts=${2:-10}
    
    for i in $(seq 0 $((max_attempts - 1))); do
        local test_port=$((start_port + i))
        if ! is_port_in_use $test_port; then
            echo $test_port
            return 0
        fi
    done
    
    print_error "Could not find available port starting from $start_port"
    return 1
}

# Function to validate and update ports
validate_ports() {
    print_status "Validating port configuration..."
    
    if is_port_in_use $BACKEND_PORT; then
        print_warning "Backend port $BACKEND_PORT is in use"
        NEW_BACKEND_PORT=$(find_available_port $BACKEND_PORT)
        if [ $? -eq 0 ]; then
            print_status "Using alternative backend port: $NEW_BACKEND_PORT"
            BACKEND_PORT=$NEW_BACKEND_PORT
        else
            exit 1
        fi
    fi
    
    if is_port_in_use $FRONTEND_PORT; then
        print_warning "Frontend port $FRONTEND_PORT is in use"
        NEW_FRONTEND_PORT=$(find_available_port $FRONTEND_PORT)
        if [ $? -eq 0 ]; then
            print_status "Using alternative frontend port: $NEW_FRONTEND_PORT"
            FRONTEND_PORT=$NEW_FRONTEND_PORT
        else
            exit 1
        fi
    fi
    
    print_success "Port validation completed - Backend: $BACKEND_PORT, Frontend: $FRONTEND_PORT"
}

# Function to deploy backend
deploy_backend() {
    print_status "Deploying FastAPI backend..."
    
    cd backend
    
    # Create production virtual environment
    if [ ! -d "venv" ]; then
        print_status "Creating production virtual environment..."
        python3 -m venv venv
    fi
    
    # Activate virtual environment
    if [ -f "venv/Scripts/activate" ]; then
        source venv/Scripts/activate
    elif [ -f "venv/bin/activate" ]; then
        source venv/bin/activate
    else
        print_error "Could not find virtual environment activation script"
        exit 1
    fi
    
    # Install production dependencies
    print_status "Installing production dependencies..."
    pip install --upgrade pip
    pip install -r requirements.txt
    
    # Create necessary directories
    mkdir -p videos temp logs
    
    # Copy environment template if .env doesn't exist
    if [ ! -f ".env" ] && [ -f "../.env.template" ]; then
        print_status "Creating .env file from template..."
        cp ../.env.template .env
        print_warning "Please edit .env file with your production settings"
    fi
    
    # Start backend with production settings
    if [ "$PRODUCTION_MODE" = true ]; then
        print_status "Starting backend in production mode..."
        nohup gunicorn main:app -w 4 -k uvicorn.workers.UvicornWorker --bind 0.0.0.0:$BACKEND_PORT > logs/backend.log 2>&1 &
        echo $! > backend.pid
    else
        print_status "Starting backend in development mode..."
        nohup python main.py > logs/backend.log 2>&1 &
        echo $! > backend.pid
    fi
    
    cd ..
    print_success "Backend deployed successfully"
}

# Function to deploy frontend
deploy_frontend() {
    print_status "Deploying React frontend..."
    
    cd frontend
    
    # Install dependencies
    print_status "Installing frontend dependencies..."
    npm install
    
    if [ "$PRODUCTION_MODE" = true ]; then
        # Build for production
        print_status "Building frontend for production..."
        npm run build
        
        # Serve with a production server (e.g., serve)
        if ! command_exists serve; then
            print_status "Installing serve globally..."
            npm install -g serve
        fi
        
        print_status "Starting frontend production server..."
        nohup serve -s dist -l $FRONTEND_PORT > logs/frontend.log 2>&1 &
        echo $! > frontend.pid
    else
        # Start development server
        print_status "Starting frontend development server..."
        mkdir -p logs
        nohup npm run dev > logs/frontend.log 2>&1 &
        echo $! > frontend.pid
    fi
    
    cd ..
    print_success "Frontend deployed successfully"
}

# Function to setup nginx (if available)
setup_nginx() {
    if command_exists nginx; then
        print_status "Setting up nginx configuration..."
        
        # Get domain/server name from environment or use localhost
        SERVER_NAME=${SERVER_NAME:-localhost}
        
        # Create enhanced nginx config for the app
        cat > robot-console-app.nginx.conf << EOF
# Robot Console Nginx Configuration
upstream backend {
    server localhost:$BACKEND_PORT;
}

upstream frontend {
    server localhost:$FRONTEND_PORT;
}

# HTTP server block (redirects to HTTPS if SSL is enabled)
server {
    listen 80;
    server_name $SERVER_NAME;
    
    # Security headers
    add_header X-Frame-Options DENY;
    add_header X-Content-Type-Options nosniff;
    add_header X-XSS-Protection "1; mode=block";
    add_header Referrer-Policy "strict-origin-when-cross-origin";
    
    # Handle Let's Encrypt challenges
    location /.well-known/acme-challenge/ {
        root /var/www/certbot;
    }
    
    # If HTTPS is enabled, redirect HTTP to HTTPS
    location / {
        if (\$scheme = http) {
            return 301 https://\$server_name\$request_uri;
        }
    }
}

# HTTPS server block (uncomment and configure for production)
server {
    listen 443 ssl http2;
    server_name $SERVER_NAME;
    
    # SSL configuration (update paths as needed)
    # ssl_certificate /etc/letsencrypt/live/$SERVER_NAME/fullchain.pem;
    # ssl_certificate_key /etc/letsencrypt/live/$SERVER_NAME/privkey.pem;
    
    # SSL security settings
    ssl_protocols TLSv1.2 TLSv1.3;
    ssl_ciphers ECDHE-RSA-AES256-GCM-SHA512:DHE-RSA-AES256-GCM-SHA512:ECDHE-RSA-AES256-GCM-SHA384;
    ssl_prefer_server_ciphers off;
    ssl_session_cache shared:SSL:10m;
    ssl_session_timeout 10m;
    
    # Security headers
    add_header Strict-Transport-Security "max-age=31536000; includeSubDomains" always;
    add_header X-Frame-Options DENY;
    add_header X-Content-Type-Options nosniff;
    add_header X-XSS-Protection "1; mode=block";
    add_header Referrer-Policy "strict-origin-when-cross-origin";
    
    # Gzip compression
    gzip on;
    gzip_vary on;
    gzip_min_length 1024;
    gzip_proxied expired no-cache no-store private must-revalidate auth;
    gzip_types text/plain text/css text/xml text/javascript application/x-javascript application/xml+rss application/javascript application/json;

    # Frontend
    location / {
        proxy_pass http://frontend;
        proxy_http_version 1.1;
        proxy_set_header Upgrade \$http_upgrade;
        proxy_set_header Connection 'upgrade';
        proxy_set_header Host \$host;
        proxy_set_header X-Real-IP \$remote_addr;
        proxy_set_header X-Forwarded-For \$proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto \$scheme;
        proxy_cache_bypass \$http_upgrade;
        
        # Handle CORS preflight
        if (\$request_method = 'OPTIONS') {
            add_header 'Access-Control-Allow-Origin' '*';
            add_header 'Access-Control-Allow-Methods' 'GET, POST, OPTIONS, PUT, DELETE';
            add_header 'Access-Control-Allow-Headers' 'DNT,User-Agent,X-Requested-With,If-Modified-Since,Cache-Control,Content-Type,Range,Authorization';
            add_header 'Access-Control-Max-Age' 1728000;
            add_header 'Content-Type' 'text/plain; charset=utf-8';
            add_header 'Content-Length' 0;
            return 204;
        }
    }

    # Backend API (no rewrite, direct proxy)
    location /api/ {
        proxy_pass http://backend/;
        proxy_http_version 1.1;
        proxy_set_header Upgrade \$http_upgrade;
        proxy_set_header Connection 'upgrade';
        proxy_set_header Host \$host;
        proxy_set_header X-Real-IP \$remote_addr;
        proxy_set_header X-Forwarded-For \$proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto \$scheme;
        proxy_cache_bypass \$http_upgrade;
        
        # CORS headers for API calls
        add_header 'Access-Control-Allow-Origin' '*' always;
        add_header 'Access-Control-Allow-Methods' 'GET, POST, OPTIONS, PUT, DELETE' always;
        add_header 'Access-Control-Allow-Headers' 'DNT,User-Agent,X-Requested-With,If-Modified-Since,Cache-Control,Content-Type,Range,Authorization' always;
    }

    # Backend direct access (docs, health checks)
    location ~ ^/(docs|openapi.json|health|robots) {
        proxy_pass http://backend;
        proxy_set_header Host \$host;
        proxy_set_header X-Real-IP \$remote_addr;
        proxy_set_header X-Forwarded-For \$proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto \$scheme;
    }
    
    # WebSocket support for real-time features
    location /ws {
        proxy_pass http://backend;
        proxy_http_version 1.1;
        proxy_set_header Upgrade \$http_upgrade;
        proxy_set_header Connection "upgrade";
        proxy_set_header Host \$host;
        proxy_set_header X-Real-IP \$remote_addr;
        proxy_set_header X-Forwarded-For \$proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto \$scheme;
        proxy_read_timeout 86400;
    }
}

# HTTP-only configuration for development/testing
server {
    listen 8080;
    server_name $SERVER_NAME;
    
    # Same configuration as HTTPS but without SSL
    location / {
        proxy_pass http://frontend;
        proxy_http_version 1.1;
        proxy_set_header Upgrade \$http_upgrade;
        proxy_set_header Connection 'upgrade';
        proxy_set_header Host \$host;
        proxy_set_header X-Real-IP \$remote_addr;
        proxy_set_header X-Forwarded-For \$proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto \$scheme;
        proxy_cache_bypass \$http_upgrade;
    }

    location /api/ {
        proxy_pass http://backend/;
        proxy_http_version 1.1;
        proxy_set_header Host \$host;
        proxy_set_header X-Real-IP \$remote_addr;
        proxy_set_header X-Forwarded-For \$proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto \$scheme;
    }
}
EOF
        
        print_status "Enhanced Nginx configuration created: robot-console-app.nginx.conf"
        print_warning "To enable this configuration:"
        print_warning "  1. Copy to nginx sites-available: sudo cp robot-console-app.nginx.conf /etc/nginx/sites-available/"
        print_warning "  2. Create symbolic link: sudo ln -s /etc/nginx/sites-available/robot-console-app.nginx.conf /etc/nginx/sites-enabled/"
        print_warning "  3. Test configuration: sudo nginx -t"
        print_warning "  4. Reload nginx: sudo systemctl reload nginx"
        print_warning ""
        print_warning "For HTTPS (production):"
        print_warning "  1. Install certbot: sudo apt install certbot python3-certbot-nginx"
        print_warning "  2. Get SSL certificate: sudo certbot --nginx -d $SERVER_NAME"
        print_warning "  3. Uncomment SSL lines in the configuration"
    else
        print_warning "Nginx not found. Install nginx for reverse proxy support."
    fi
}

# Function to install production dependencies
install_production_deps() {
    print_status "Installing production dependencies..."
    
    # Install gunicorn for production backend
    cd backend
    if [ -f "venv/bin/activate" ]; then
        source venv/bin/activate
        pip install gunicorn
    elif [ -f "venv/Scripts/activate" ]; then
        source venv/Scripts/activate
        pip install gunicorn
    fi
    cd ..
    
    # Install serve for production frontend
    if command_exists npm; then
        if [ "$PRODUCTION_MODE" = true ]; then
            npm install -g serve
        fi
    fi
}

# Function to create systemd service files
create_systemd_services() {
    if command_exists systemctl; then
        print_status "Creating systemd service files..."
        
        # Backend service
        cat > robot-console-backend.service << EOF
[Unit]
Description=Robot Console Backend
After=network.target

[Service]
Type=simple
User=\${USER}
WorkingDirectory=$(pwd)/backend
Environment=PATH=$(pwd)/backend/venv/bin
ExecStart=$(pwd)/backend/venv/bin/gunicorn main:app -w 4 -k uvicorn.workers.UvicornWorker --bind 0.0.0.0:$BACKEND_PORT
Restart=always

[Install]
WantedBy=multi-user.target
EOF
        
        # Frontend service
        cat > robot-console-frontend.service << EOF
[Unit]
Description=Robot Console Frontend
After=network.target

[Service]
Type=simple
User=\${USER}
WorkingDirectory=$(pwd)/frontend
ExecStart=/usr/local/bin/serve -s dist -l $FRONTEND_PORT
Restart=always

[Install]
WantedBy=multi-user.target
EOF
        
        print_status "Systemd service files created"
        print_warning "To install services, run:"
        print_warning "  sudo cp robot-console-*.service /etc/systemd/system/"
        print_warning "  sudo systemctl enable robot-console-backend"
        print_warning "  sudo systemctl enable robot-console-frontend"
        print_warning "  sudo systemctl start robot-console-backend"
        print_warning "  sudo systemctl start robot-console-frontend"
    fi
}

# Function to show deployment info
show_deployment_info() {
    print_success "🎉 Robot Live Console App deployed successfully!"
    echo ""
    print_status "📱 Frontend: http://localhost:$FRONTEND_PORT"
    print_status "🔧 Backend API: http://localhost:$BACKEND_PORT"
    print_status "📊 API Docs: http://localhost:$BACKEND_PORT/docs"
    echo ""
    
    if [ "$PRODUCTION_MODE" = true ]; then
        print_status "🏭 Production mode enabled"
        print_status "📂 Backend logs: backend/logs/backend.log"
        print_status "📂 Frontend logs: frontend/logs/frontend.log"
    else
        print_status "🔧 Development mode enabled"
    fi
    
    echo ""
    print_status "🛑 To stop the application:"
    print_status "  pkill -f 'python main.py'"
    print_status "  pkill -f 'npm run dev'"
    print_status "  or use: ./scripts/setup.sh stop"
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -p, --production    Deploy in production mode"
    echo "  -d, --development   Deploy in development mode (default)"
    echo "  --port-backend PORT Set backend port (default: 8000)"
    echo "  --port-frontend PORT Set frontend port (default: 3000)"
    echo "  --nginx             Setup nginx configuration"
    echo "  --systemd           Create systemd service files"
    echo "  -h, --help          Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                  # Deploy in development mode"
    echo "  $0 --production     # Deploy in production mode"
    echo "  $0 --nginx --systemd # Deploy with nginx and systemd configs"
}

# Parse command line arguments
parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            -p|--production)
                PRODUCTION_MODE=true
                shift
                ;;
            -d|--development)
                PRODUCTION_MODE=false
                shift
                ;;
            --port-backend)
                BACKEND_PORT="$2"
                shift 2
                ;;
            --port-frontend)
                FRONTEND_PORT="$2"
                shift 2
                ;;
            --nginx)
                SETUP_NGINX=true
                shift
                ;;
            --systemd)
                SETUP_SYSTEMD=true
                shift
                ;;
            -h|--help)
                show_usage
                exit 0
                ;;
            *)
                print_error "Unknown option: $1"
                show_usage
                exit 1
                ;;
        esac
    done
}

# Main function
main() {
    parse_args "$@"
    
    print_status "Deployment mode: $([ "$PRODUCTION_MODE" = true ] && echo "Production" || echo "Development")"
    print_status "Backend port: $BACKEND_PORT"
    print_status "Frontend port: $FRONTEND_PORT"
    echo ""
    
    # Check requirements
    if ! command_exists python3; then
        print_error "Python 3 is required but not installed"
        exit 1
    fi
    
    if ! command_exists node; then
        print_error "Node.js is required but not installed"
        exit 1
    fi
    
    if ! command_exists npm; then
        print_error "npm is required but not installed"
        exit 1
    fi
    
    # Validate and update ports if necessary
    validate_ports
    
    # Install production dependencies if needed
    if [ "$PRODUCTION_MODE" = true ]; then
        install_production_deps
    fi
    
    # Deploy services
    deploy_backend
    deploy_frontend
    
    # Setup additional services if requested
    if [ "$SETUP_NGINX" = true ]; then
        setup_nginx
    fi
    
    if [ "$SETUP_SYSTEMD" = true ]; then
        create_systemd_services
    fi
    
    # Wait for services to start
    sleep 5
    
    show_deployment_info
}

# Run main function with all arguments
main "$@"