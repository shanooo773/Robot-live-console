#!/bin/bash

# Deployment Validation Script
# Tests the deployed Robot Console application

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

# Default configuration
BACKEND_PORT=8000
FRONTEND_PORT=5173
BACKEND_URL="http://localhost:$BACKEND_PORT"
FRONTEND_URL="http://localhost:$FRONTEND_PORT"

# Load environment variables if available
if [ -f ".env" ]; then
    source .env
    BACKEND_PORT=${BACKEND_PORT:-8000}
    FRONTEND_PORT=${FRONTEND_PORT:-5173}
    BACKEND_URL="http://localhost:$BACKEND_PORT"
    FRONTEND_URL="http://localhost:$FRONTEND_PORT"
fi

# Test counter
TESTS_PASSED=0
TESTS_TOTAL=0

# Function to run a test
run_test() {
    local test_name="$1"
    local test_command="$2"
    
    TESTS_TOTAL=$((TESTS_TOTAL + 1))
    print_status "Testing: $test_name"
    
    if eval "$test_command" > /dev/null 2>&1; then
        print_success "✅ $test_name"
        TESTS_PASSED=$((TESTS_PASSED + 1))
        return 0
    else
        print_error "❌ $test_name"
        return 1
    fi
}

# Function to test HTTP endpoint
test_http_endpoint() {
    local url="$1"
    local expected_status="${2:-200}"
    
    local actual_status=$(curl -s -o /dev/null -w "%{http_code}" "$url" 2>/dev/null || echo "000")
    [ "$actual_status" = "$expected_status" ]
}

# Function to test JSON API endpoint
test_json_endpoint() {
    local url="$1"
    
    local response=$(curl -s -H "Accept: application/json" "$url" 2>/dev/null || echo "")
    echo "$response" | python3 -m json.tool > /dev/null 2>&1
}

# Function to validate CORS
test_cors_endpoint() {
    local url="$1"
    local origin="${2:-http://localhost:3000}"
    
    local status=$(curl -s -o /dev/null -w "%{http_code}" \
        -H "Origin: $origin" \
        -H "Access-Control-Request-Method: GET" \
        -H "Access-Control-Request-Headers: Content-Type" \
        -X OPTIONS "$url" 2>/dev/null || echo "000")
    
    [ "$status" = "200" ] || [ "$status" = "204" ]
}

# Main validation function
main() {
    echo "🚀 Robot Console Deployment Validation"
    echo "======================================="
    echo ""
    
    print_status "Configuration:"
    print_status "  Backend URL: $BACKEND_URL"
    print_status "  Frontend URL: $FRONTEND_URL"
    echo ""
    
    # Test 1: Backend connectivity
    run_test "Backend connectivity" "test_http_endpoint '$BACKEND_URL/health'"
    
    # Test 2: Backend health endpoint returns JSON
    run_test "Backend health JSON response" "test_json_endpoint '$BACKEND_URL/health'"
    
    # Test 3: Backend API documentation
    run_test "Backend API documentation" "test_http_endpoint '$BACKEND_URL/docs'"
    
    # Test 4: Backend service status
    run_test "Backend service status" "test_json_endpoint '$BACKEND_URL/health/services'"
    
    # Test 5: Backend features endpoint
    run_test "Backend features endpoint" "test_json_endpoint '$BACKEND_URL/health/features'"
    
    # Test 6: Frontend connectivity
    run_test "Frontend connectivity" "test_http_endpoint '$FRONTEND_URL/'"
    
    # Test 7: CORS configuration
    run_test "CORS configuration (localhost:3000)" "test_cors_endpoint '$BACKEND_URL/auth/login' 'http://localhost:3000'"
    run_test "CORS configuration (localhost:5173)" "test_cors_endpoint '$BACKEND_URL/auth/login' 'http://localhost:5173'"
    
    # Test 8: Authentication endpoint structure
    run_test "Auth login endpoint structure" "test_http_endpoint '$BACKEND_URL/auth/login' '422'"
    
    # Test 9: Booking endpoint structure
    run_test "Booking endpoint structure" "test_http_endpoint '$BACKEND_URL/bookings' '401'"
    
    # Test 10: Static file serving (if applicable)
    if test_http_endpoint "$FRONTEND_URL/assets" "404" 2>/dev/null; then
        run_test "Frontend static assets accessible" "true"
    else
        run_test "Frontend static assets accessible" "false"
    fi
    
    echo ""
    echo "📊 Test Results"
    echo "==============="
    
    if [ $TESTS_PASSED -eq $TESTS_TOTAL ]; then
        print_success "🎉 All tests passed! ($TESTS_PASSED/$TESTS_TOTAL)"
        echo ""
        print_status "✅ Your Robot Console application is properly deployed!"
        print_status "✅ Backend API is accessible and responding correctly"
        print_status "✅ Frontend is serving content"
        print_status "✅ CORS configuration is working"
        echo ""
        print_status "🌐 Access your application:"
        print_status "   Frontend: $FRONTEND_URL"
        print_status "   Backend API: $BACKEND_URL"
        print_status "   API Docs: $BACKEND_URL/docs"
        
        exit 0
    else
        print_warning "⚠️ Some tests failed ($TESTS_PASSED/$TESTS_TOTAL passed)"
        echo ""
        print_status "🔧 Troubleshooting suggestions:"
        
        # Check if services are running
        if ! test_http_endpoint "$BACKEND_URL/health" > /dev/null 2>&1; then
            print_status "   • Backend may not be running. Try: cd app/backend && python main.py"
        fi
        
        if ! test_http_endpoint "$FRONTEND_URL/" > /dev/null 2>&1; then
            print_status "   • Frontend may not be running. Try: cd app/frontend && npm run dev"
        fi
        
        print_status "   • Check firewall settings"
        print_status "   • Verify port availability with: netstat -tlnp | grep :$BACKEND_PORT"
        print_status "   • Check logs for error messages"
        print_status "   • Run the deployment script: ./app/scripts/deploy.sh"
        
        exit 1
    fi
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --backend-url URL    Backend URL (default: http://localhost:8000)"
    echo "  --frontend-url URL   Frontend URL (default: http://localhost:5173)"
    echo "  --backend-port PORT  Backend port (default: 8000)"
    echo "  --frontend-port PORT Frontend port (default: 5173)"
    echo "  -h, --help           Show this help"
    echo ""
    echo "Examples:"
    echo "  $0                                  # Test with defaults"
    echo "  $0 --backend-port 8080             # Test with custom backend port"
    echo "  $0 --backend-url https://api.com   # Test production deployment"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --backend-url)
            BACKEND_URL="$2"
            shift 2
            ;;
        --frontend-url)
            FRONTEND_URL="$2"
            shift 2
            ;;
        --backend-port)
            BACKEND_PORT="$2"
            BACKEND_URL="http://localhost:$2"
            shift 2
            ;;
        --frontend-port)
            FRONTEND_PORT="$2"
            FRONTEND_URL="http://localhost:$2"
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

# Run main function
main