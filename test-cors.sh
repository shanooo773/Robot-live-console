#!/bin/bash

# CORS Configuration Validator and Tester
# This script validates CORS configuration and tests connectivity

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

# Function to test CORS configuration
test_cors() {
    local backend_url=$1
    local origin=$2
    
    print_status "Testing CORS for origin: $origin"
    
    # Test preflight OPTIONS request
    response=$(curl -s -o /dev/null -w "%{http_code}" \
        -H "Origin: $origin" \
        -H "Access-Control-Request-Method: POST" \
        -H "Access-Control-Request-Headers: Content-Type,Authorization" \
        -X OPTIONS \
        "$backend_url/auth/login" 2>/dev/null || echo "000")
    
    if [ "$response" = "200" ] || [ "$response" = "204" ]; then
        print_success "CORS preflight passed for $origin"
        return 0
    else
        print_error "CORS preflight failed for $origin (HTTP $response)"
        return 1
    fi
}

# Function to test API endpoint
test_api_endpoint() {
    local backend_url=$1
    local endpoint=$2
    
    print_status "Testing API endpoint: $endpoint"
    
    response=$(curl -s -o /dev/null -w "%{http_code}" "$backend_url$endpoint" 2>/dev/null || echo "000")
    
    if [ "$response" = "200" ] || [ "$response" = "422" ] || [ "$response" = "401" ]; then
        print_success "API endpoint $endpoint is accessible (HTTP $response)"
        return 0
    else
        print_error "API endpoint $endpoint failed (HTTP $response)"
        return 1
    fi
}

# Main validation function
validate_cors_setup() {
    print_status "🔍 CORS Configuration Validator"
    echo ""
    
    # Default values
    BACKEND_URL="http://localhost:8000"
    FRONTEND_URL="http://localhost:5173"
    
    # Load from .env if available
    if [ -f ".env" ]; then
        source .env
        BACKEND_URL="http://localhost:${BACKEND_PORT:-8000}"
        FRONTEND_URL="http://localhost:${FRONTEND_PORT:-5173}"
    fi
    
    print_status "Backend URL: $BACKEND_URL"
    print_status "Frontend URL: $FRONTEND_URL"
    echo ""
    
    # Test backend connectivity
    print_status "Testing backend connectivity..."
    if curl -s "$BACKEND_URL/health" > /dev/null 2>&1; then
        print_success "Backend is accessible"
    else
        print_error "Backend is not accessible at $BACKEND_URL"
        print_warning "Make sure the backend is running"
        return 1
    fi
    
    # Test API endpoints
    echo ""
    print_status "Testing API endpoints..."
    test_api_endpoint "$BACKEND_URL" "/docs"
    test_api_endpoint "$BACKEND_URL" "/auth/login"
    test_api_endpoint "$BACKEND_URL" "/bookings"
    
    # Test CORS configuration
    echo ""
    print_status "Testing CORS configuration..."
    
    # Get CORS origins from environment or defaults
    CORS_ORIGINS=${CORS_ORIGINS:-"http://localhost:3000,http://localhost:5173"}
    IFS=',' read -ra ORIGINS <<< "$CORS_ORIGINS"
    
    cors_tests_passed=0
    cors_tests_total=0
    
    for origin in "${ORIGINS[@]}"; do
        origin=$(echo "$origin" | xargs) # trim whitespace
        if [ -n "$origin" ]; then
            cors_tests_total=$((cors_tests_total + 1))
            if test_cors "$BACKEND_URL" "$origin"; then
                cors_tests_passed=$((cors_tests_passed + 1))
            fi
        fi
    done
    
    echo ""
    print_status "CORS Test Results: $cors_tests_passed/$cors_tests_total origins passed"
    
    if [ $cors_tests_passed -eq $cors_tests_total ] && [ $cors_tests_total -gt 0 ]; then
        print_success "✅ All CORS tests passed!"
        echo ""
        print_status "Your CORS configuration is working correctly."
        print_status "Frontend applications from allowed origins can communicate with the backend."
    else
        print_warning "⚠️  Some CORS tests failed."
        echo ""
        print_status "Troubleshooting tips:"
        print_status "1. Ensure the backend server is running"
        print_status "2. Check CORS_ORIGINS environment variable"
        print_status "3. Verify FastAPI CORS middleware configuration"
        print_status "4. Check firewall and network settings"
    fi
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --backend URL       Backend URL (default: http://localhost:8000)"
    echo "  --frontend URL      Frontend URL (default: http://localhost:5173)"
    echo "  --test-origin URL   Test specific origin"
    echo "  -h, --help          Show this help"
    echo ""
    echo "Examples:"
    echo "  $0                                      # Test with defaults"
    echo "  $0 --backend http://localhost:8080     # Test with custom backend"
    echo "  $0 --test-origin https://myapp.com     # Test specific origin"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --backend)
            BACKEND_URL="$2"
            shift 2
            ;;
        --frontend)
            FRONTEND_URL="$2"
            shift 2
            ;;
        --test-origin)
            TEST_ORIGIN="$2"
            shift 2
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

# If specific origin test requested
if [ -n "$TEST_ORIGIN" ]; then
    print_status "Testing specific origin: $TEST_ORIGIN"
    BACKEND_URL=${BACKEND_URL:-"http://localhost:8000"}
    test_cors "$BACKEND_URL" "$TEST_ORIGIN"
    exit $?
fi

# Run full validation
validate_cors_setup