#!/bin/bash

# Robot Console Quick Start Guide
# This script provides guidance for fixing CORS and deployment issues

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m' # No Color

print_header() {
    echo -e "${BOLD}${BLUE}$1${NC}"
    echo "=============================================="
}

print_step() {
    echo -e "${GREEN}✓${NC} $1"
}

print_command() {
    echo -e "${YELLOW}  $ $1${NC}"
}

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_header "🚀 Robot Console - CORS & Deployment Fix Guide"

echo ""
echo "This project has been enhanced with comprehensive CORS fixes and production deployment tools."
echo ""

print_header "🔧 What's Been Fixed"

print_step "CORS 403 Errors - Enhanced FastAPI CORSMiddleware configuration"
print_step "Frontend API Calls - Added automatic credential handling with axios interceptors"
print_step "Environment Security - Added .env templates for GitHub tokens and secrets"
print_step "Deployment Scripts - Enhanced with port checking, Nginx, and systemd support"
print_step "Docker Support - Complete containerization with health checks"
print_step "Testing Tools - CORS validation and deployment testing scripts"

echo ""
print_header "🚀 Quick Start Options"

echo ""
echo -e "${BOLD}Option 1: Docker Deployment (Recommended)${NC}"
print_command "./deploy-docker.sh"
echo "   - Automatic setup with containers"
echo "   - Built-in health checks and auto-restart"
echo "   - Production-ready configuration"

echo ""
echo -e "${BOLD}Option 2: Traditional Deployment${NC}"
print_command "./app/scripts/deploy.sh --production --nginx --systemd"
echo "   - Native deployment with systemd services"
echo "   - Nginx reverse proxy configuration"
echo "   - Auto-restart and monitoring"

echo ""
echo -e "${BOLD}Option 3: Development Mode${NC}"
print_command "./app/scripts/deploy.sh"
echo "   - Quick development setup"
echo "   - No additional services required"

echo ""
print_header "⚙️ Configuration Steps"

echo ""
print_step "1. Configure Environment Variables"
print_command "cp .env.template .env"
print_command "nano .env  # Edit CORS_ORIGINS and other settings"

echo ""
print_step "2. Set CORS Origins (Important!)"
echo "   For development:"
print_command "CORS_ORIGINS=http://localhost:3000,http://localhost:5173"
echo "   For production:"
print_command "CORS_ORIGINS=https://yourdomain.com,https://api.yourdomain.com"

echo ""
print_step "3. Configure Security"
print_command "JWT_SECRET=your-super-secret-jwt-key-change-this"
print_command "GITHUB_TOKEN=your-github-token-if-needed"

echo ""
print_header "🧪 Testing & Validation"

echo ""
print_step "Test CORS Configuration"
print_command "./test-cors.sh"
print_command "./test-cors.sh --test-origin https://yourdomain.com"

echo ""
print_step "Validate Deployment"
print_command "./validate-deployment.sh"

echo ""
print_step "Check Service Status"
print_command "curl http://localhost:8000/health"
print_command "curl http://localhost:3000/"

echo ""
print_header "🔒 Production Security"

echo ""
print_step "Enable HTTPS with Let's Encrypt"
print_command "sudo apt install certbot python3-certbot-nginx"
print_command "sudo certbot --nginx -d yourdomain.com"

echo ""
print_step "Update Environment for HTTPS"
print_command "ENABLE_HTTPS=true"
print_command "CORS_ORIGINS=https://yourdomain.com"

echo ""
print_header "📊 Monitoring"

echo ""
if command -v systemctl >/dev/null 2>&1; then
    print_step "Systemd Service Management"
    print_command "sudo systemctl status robot-console-backend"
    print_command "sudo journalctl -u robot-console-backend -f"
fi

if command -v docker >/dev/null 2>&1; then
    print_step "Docker Container Management"
    print_command "docker-compose logs -f backend"
    print_command "docker-compose ps"
fi

echo ""
print_header "🐛 Troubleshooting"

echo ""
print_step "Common CORS Issues"
echo "   • Verify CORS_ORIGINS matches your frontend domain exactly"
echo "   • Check for trailing slashes in origins"
echo "   • Ensure backend is loading .env file properly"

echo ""
print_step "Port Conflicts"
echo "   • Scripts automatically check for port conflicts"
echo "   • Manual check: netstat -tlnp | grep :8000"

echo ""
print_step "Service Issues"
echo "   • Check logs for error messages"
echo "   • Verify all dependencies are installed"
echo "   • Test endpoints individually"

echo ""
print_header "📚 Documentation"

echo ""
print_step "Complete deployment guide: DEPLOYMENT.md"
print_step "CORS validation guide: VPS_CORS_GUIDE.md"
print_step "Architecture overview: SERVICE_ARCHITECTURE.md"

echo ""
print_header "🎯 Next Steps"

echo ""
echo "1. Choose your deployment method (Docker recommended)"
echo "2. Configure .env file with your settings"
echo "3. Run the deployment script"
echo "4. Test with validation scripts"
echo "5. Configure HTTPS for production"

echo ""
echo -e "${BOLD}${GREEN}Ready to deploy? Run one of these commands:${NC}"
echo -e "${YELLOW}  ./deploy-docker.sh${NC}                    # Docker deployment"
echo -e "${YELLOW}  ./app/scripts/deploy.sh --production${NC}  # Traditional deployment"

echo ""
echo -e "${BLUE}For help and support, check the troubleshooting section in DEPLOYMENT.md${NC}"
echo ""