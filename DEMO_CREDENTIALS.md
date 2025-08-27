# Demo Credentials for Robot Live Console

This document provides the demo credentials that can be used to test the Robot Live Console without creating new accounts.

## 🎯 Quick Demo Access

The easiest way to test the system is using the demo accounts that work out of the box:

### Demo User Account
- **Email:** `demo@user.com`  
- **Password:** `password`  
- **Role:** Regular User  
- **Features:** Can create bookings, access development console, view own bookings
- **Special Access:** Demo mode with unrestricted console access

### Demo Admin Account  
- **Email:** `admin@demo.com`  
- **Password:** `password`  
- **Role:** Administrator  
- **Features:** Full admin access, admin dashboard, manage all bookings/users
- **Special Access:** Demo mode + admin privileges

## 🔑 Database Accounts

### Default Admin Account
- **Email:** `admin@robot-console.com`  
- **Password:** `admin123`  
- **Role:** Administrator  
- **Features:** Real admin account stored in database, full admin privileges

## 🚀 Getting Started

1. **Start the backend server:**
   ```bash
   cd app/backend
   python main.py
   ```

2. **Start the frontend server:**
   ```bash
   cd app/frontend  
   npm run dev
   ```

3. **Access the application:** 
   - Open http://localhost:3000
   - Click "Book Development Session"
   - Use any of the demo credentials above

## 🔧 Customization

You can customize the demo credentials by setting these environment variables:

```bash
export DEMO_USER_EMAIL="demo@user.com"
export DEMO_USER_PASSWORD="password"
export DEMO_ADMIN_EMAIL="admin@demo.com"  
export DEMO_ADMIN_PASSWORD="password"
```

## 🧪 Testing Authentication

Test the authentication API directly using curl:

```bash
# Test demo user login
curl -X POST "http://localhost:8000/auth/login" \
     -H "Content-Type: application/json" \
     -d '{"email": "demo@user.com", "password": "password"}'

# Test demo admin login
curl -X POST "http://localhost:8000/auth/login" \
     -H "Content-Type: application/json" \
     -d '{"email": "admin@demo.com", "password": "password"}'

# Test user registration
curl -X POST "http://localhost:8000/auth/register" \
     -H "Content-Type: application/json" \
     -d '{"name": "Test User", "email": "test@example.com", "password": "testpass123"}'
```

## ✅ What's Working

- ✅ Demo user login with instant access
- ✅ Demo admin login with admin dashboard access  
- ✅ User registration with automatic login
- ✅ Database user authentication
- ✅ Password hashing and verification
- ✅ JWT token generation and validation
- ✅ CORS configuration for frontend/backend communication
- ✅ Detailed logging for debugging
- ✅ Error handling with user-friendly messages
- ✅ Demo mode badges and special access
- ✅ Booking system integration

## 📝 Implementation Details

- **Demo accounts** are not stored in the database and exist only for testing purposes
- **Demo accounts** are created dynamically during login and use negative IDs (-1, -2) to avoid conflicts
- **Demo accounts** include special flags (`isDemoUser`, `isDemoAdmin`) in the JWT token
- **Frontend** automatically recognizes demo accounts and shows "DEMO MODE" badges
- **Backend** has comprehensive logging for authentication debugging
- **Password hashing** uses SHA-256 with secure salt generation
- **JWT tokens** expire after 24 hours and include user role information