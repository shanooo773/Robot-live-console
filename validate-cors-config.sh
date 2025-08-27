#!/bin/bash

# VPS CORS Configuration Validator
# Run this script to validate your CORS configuration for VPS deployment

echo "🔍 VPS CORS Configuration Validator"
echo "===================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

# Check if .env file exists
ENV_FILE="app/.env"
if [ ! -f "$ENV_FILE" ]; then
    print_error ".env file not found at $ENV_FILE"
    print_info "Create one by copying: cp app/.env.template app/.env"
    exit 1
fi

print_success ".env file found"

# Check CORS_ORIGINS configuration
echo ""
echo "🔍 Checking CORS Configuration..."

# Extract CORS_ORIGINS from .env file
CORS_ORIGINS=$(grep "^CORS_ORIGINS=" "$ENV_FILE" | cut -d'=' -f2- | tr -d ' ')

if [ -z "$CORS_ORIGINS" ]; then
    print_error "CORS_ORIGINS not found in .env file"
    print_info "Add: CORS_ORIGINS=http://localhost:3000,http://localhost:5173,http://YOUR_VPS_IP"
    exit 1
fi

print_info "CORS_ORIGINS found: $CORS_ORIGINS"

# Parse CORS origins
IFS=',' read -ra ORIGINS <<< "$CORS_ORIGINS"

# Check for common issues
has_localhost=false
has_vps_config=false
has_https=false

echo ""
echo "📋 Analyzing CORS Origins:"

for origin in "${ORIGINS[@]}"; do
    origin=$(echo "$origin" | tr -d ' ')  # Remove spaces
    echo "   - $origin"
    
    # Check for localhost (development compatibility)
    if [[ "$origin" == *"localhost"* ]]; then
        has_localhost=true
    fi
    
    # Check for non-localhost (VPS configuration)
    if [[ "$origin" != *"localhost"* && "$origin" != "" ]]; then
        has_vps_config=true
    fi
    
    # Check for HTTPS
    if [[ "$origin" == "https://"* ]]; then
        has_https=true
    fi
done

echo ""

# Validation checks
if [ "$has_localhost" = true ]; then
    print_success "Development compatibility maintained (localhost origins present)"
else
    print_warning "No localhost origins found - may affect local development"
fi

if [ "$has_vps_config" = true ]; then
    print_success "VPS/production origins configured"
else
    print_error "No VPS/production origins found - CORS errors will persist"
    print_info "Add your VPS IP or domain to CORS_ORIGINS"
    exit 1
fi

if [ "$has_https" = true ]; then
    print_success "HTTPS origins configured (recommended for production)"
else
    print_warning "No HTTPS origins found - consider adding for production security"
fi

# Check for common mistakes
echo ""
echo "🔍 Checking for Common Issues..."

# Check for wildcard
if [[ "$CORS_ORIGINS" == *"*"* ]]; then
    print_error "Wildcard (*) found in CORS_ORIGINS - security risk in production"
fi

# Check for trailing slashes
if [[ "$CORS_ORIGINS" == *"/" ]]; then
    print_warning "Trailing slashes found - may cause CORS issues"
fi

# Check for spaces
if [[ "$CORS_ORIGINS" =~ [[:space:]] ]]; then
    print_info "Spaces found in CORS_ORIGINS (will be stripped automatically)"
fi

# Check frontend configuration
echo ""
echo "🔍 Checking Frontend Configuration..."

FRONTEND_ENV="app/frontend/.env"
if [ -f "$FRONTEND_ENV" ]; then
    VITE_API_URL=$(grep "^VITE_API_URL=" "$FRONTEND_ENV" | cut -d'=' -f2-)
    if [ -n "$VITE_API_URL" ]; then
        print_success "Frontend API URL configured: $VITE_API_URL"
    else
        print_warning "VITE_API_URL not configured in frontend .env"
    fi
else
    print_info "Frontend .env file not found (will use defaults)"
fi

# Summary
echo ""
echo "📊 Configuration Summary:"
echo "========================="

if [ "$has_vps_config" = true ] && [ "$has_localhost" = true ]; then
    print_success "Configuration looks good for VPS deployment!"
    echo ""
    print_info "Next steps:"
    echo "   1. Deploy your backend services"
    echo "   2. Deploy your frontend"
    echo "   3. Test sign-in/sign-up functionality"
    echo "   4. Check browser console for CORS errors"
else
    print_error "Configuration issues found - please fix before deploying"
    echo ""
    print_info "Quick fix example:"
    echo "   CORS_ORIGINS=http://localhost:3000,http://localhost:5173,http://YOUR_VPS_IP"
fi

echo ""
echo "📖 For detailed instructions, see: VPS_CORS_GUIDE.md"