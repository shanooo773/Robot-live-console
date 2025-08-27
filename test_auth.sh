#!/bin/bash

# Test script to verify authentication functionality
# This script tests the login authentication to ensure demo credentials work

echo "🧪 Testing Robot Live Console Authentication"
echo "============================================"

# Backend URL
BACKEND_URL="http://localhost:8000"

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test function
test_login() {
    local email=$1
    local password=$2
    local description=$3
    local expected_result=$4
    
    echo -n "Testing $description... "
    
    response=$(curl -s -X POST "$BACKEND_URL/auth/login" \
        -H "Content-Type: application/json" \
        -d "{\"email\": \"$email\", \"password\": \"$password\"}")
    
    if [[ $expected_result == "success" ]]; then
        if echo "$response" | grep -q "access_token"; then
            echo -e "${GREEN}✅ PASSED${NC}"
            user_name=$(echo "$response" | jq -r '.user.name' 2>/dev/null || echo "Unknown")
            echo "   Logged in as: $user_name"
        else
            echo -e "${RED}❌ FAILED${NC}"
            echo "   Expected success but got: $response"
            return 1
        fi
    else
        if echo "$response" | grep -q "Invalid email or password"; then
            echo -e "${GREEN}✅ PASSED${NC}"
            echo "   Correctly rejected invalid credentials"
        else
            echo -e "${RED}❌ FAILED${NC}"
            echo "   Expected failure but got: $response"
            return 1
        fi
    fi
    return 0
}

# Check if backend is running
echo "Checking if backend is running on $BACKEND_URL..."
if ! curl -s "$BACKEND_URL/" > /dev/null; then
    echo -e "${RED}❌ Backend is not running on $BACKEND_URL${NC}"
    echo "Please start the backend with:"
    echo "cd backend && python -m uvicorn main:app --host 0.0.0.0 --port 8000 --reload"
    exit 1
fi
echo -e "${GREEN}✅ Backend is running${NC}"
echo ""

# Run tests
total_tests=0
passed_tests=0

# Test demo user
total_tests=$((total_tests + 1))
if test_login "demo@user.com" "password" "Demo User Login" "success"; then
    passed_tests=$((passed_tests + 1))
fi
echo ""

# Test demo admin
total_tests=$((total_tests + 1))
if test_login "admin@demo.com" "password" "Demo Admin Login" "success"; then
    passed_tests=$((passed_tests + 1))
fi
echo ""

# Test real admin
total_tests=$((total_tests + 1))
if test_login "admin@robot-console.com" "admin123" "Real Admin Login" "success"; then
    passed_tests=$((passed_tests + 1))
fi
echo ""

# Test invalid credentials
total_tests=$((total_tests + 1))
if test_login "wrong@email.com" "wrongpass" "Invalid Credentials" "failure"; then
    passed_tests=$((passed_tests + 1))
fi
echo ""

# Test wrong password for demo user
total_tests=$((total_tests + 1))
if test_login "demo@user.com" "wrongpass" "Demo User Wrong Password" "failure"; then
    passed_tests=$((passed_tests + 1))
fi
echo ""

# Results
echo "============================================"
echo "🧪 Test Results: $passed_tests/$total_tests tests passed"

if [[ $passed_tests -eq $total_tests ]]; then
    echo -e "${GREEN}🎉 All tests passed! Authentication is working correctly.${NC}"
    exit 0
else
    echo -e "${RED}❌ Some tests failed. Please check the backend configuration.${NC}"
    exit 1
fi