# Demo Credentials for Robot Live Console

This document provides the demo credentials that can be used to test the Robot Live Console without creating new accounts.

## Demo User Account

**Email:** `demo@user.com`  
**Password:** `password`  
**Role:** Regular User  
**Features:** Can create bookings, access development console, view own bookings

## Demo Admin Account

**Email:** `admin@demo.com`  
**Password:** `password`  
**Role:** Administrator  
**Features:** Full admin access, can manage all bookings, view all users, access admin dashboard

## Default Admin Account (Database)

**Email:** `admin@robot-console.com`  
**Password:** `admin123`  
**Role:** Administrator  
**Features:** Real admin account stored in database, full admin privileges

## Environment Variables

You can customize the demo credentials by setting these environment variables:

```bash
export DEMO_USER_EMAIL="demo@user.com"
export DEMO_USER_PASSWORD="password"
export DEMO_ADMIN_EMAIL="admin@demo.com"  
export DEMO_ADMIN_PASSWORD="password"
```

## Testing Authentication

You can test the authentication using curl:

```bash
# Test demo user login
curl -X POST "http://localhost:8000/auth/login" \
     -H "Content-Type: application/json" \
     -d '{"email": "demo@user.com", "password": "password"}'

# Test demo admin login
curl -X POST "http://localhost:8000/auth/login" \
     -H "Content-Type: application/json" \
     -d '{"email": "admin@demo.com", "password": "password"}'
```

## Notes

- Demo accounts are not stored in the database and exist only for testing purposes
- Demo accounts are created dynamically during login and use negative IDs (-1, -2) to avoid conflicts
- Demo accounts include special flags (`isDemoUser`, `isDemoAdmin`) in the JWT token
- The frontend automatically recognizes demo accounts and shows "DEMO MODE" badges