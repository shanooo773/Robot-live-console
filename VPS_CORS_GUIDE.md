# VPS CORS Configuration Guide

## Problem
When deploying the Robot Live Console on a VPS, you may encounter CORS (Cross-Origin Resource Sharing) errors during sign-in and sign-up operations. This happens because the frontend is served from your VPS domain, but the backend only allows localhost origins by default.

## Error Symptoms
- CORS error in browser console during registration/login
- 400 status code on preflight OPTIONS requests
- Authentication requests failing with "Access-Control-Allow-Origin" errors

## Solution

### 1. Backend Configuration
The backend services now support configurable CORS origins through the `CORS_ORIGINS` environment variable.

#### Create .env file
Copy the environment template and configure for your VPS:
```bash
cp .env.template .env
```

#### Configure CORS Origins
Edit your `.env` file to include your VPS domain/IP:
```bash
# Example for VPS with IP 172.104.207.139
CORS_ORIGINS=http://localhost:3000,http://localhost:5173,http://172.104.207.139,http://your-domain.com,http://your-domain.com:3000

# Or if using HTTPS:
CORS_ORIGINS=http://localhost:3000,http://localhost:5173,https://your-domain.com,https://your-domain.com:3000
```

### 2. Frontend Configuration
The frontend API configuration now supports environment variables for production deployment.

#### For React-based frontend (frontend/ directory):
Create a `.env` file:
```bash
REACT_APP_ADMIN_API=http://your-vps-ip:8000
REACT_APP_SIMULATION_API=http://your-vps-ip:8001
```

#### For Vite-based frontend (app/frontend/ directory):
Create a `.env` file:
```bash
VITE_API_URL=http://your-vps-ip:8000
VITE_ADMIN_API_URL=http://your-vps-ip:8000  
VITE_SIMULATION_API_URL=http://your-vps-ip:8001
```

### 3. Deployment Steps

1. **Configure environment variables:**
   ```bash
   cd Robot-live-console/app
   cp .env.template .env
   # Edit .env with your VPS configuration
   ```

2. **Set CORS origins for your VPS:**
   ```bash
   # In your .env file
   CORS_ORIGINS=http://localhost:3000,http://localhost:5173,http://YOUR_VPS_IP,http://YOUR_DOMAIN.com
   ```

3. **Build and deploy:**
   ```bash
   # Start backend with environment variables
   cd app/backend
   python main.py

   # Build and serve frontend
   cd ../frontend
   npm run build
   npm run preview
   ```

### 4. Testing
After configuration, test that CORS is working:

#### Automated Validation
Run the included validation script:
```bash
./validate-cors-config.sh
```

#### Manual Testing
1. Open browser developer tools
2. Try to register/login
3. Check that there are no CORS errors in console
4. Verify that preflight OPTIONS requests return 200 status

### 5. Common VPS Configurations

#### Basic VPS with IP only:
```bash
CORS_ORIGINS=http://localhost:3000,http://localhost:5173,http://172.104.207.139,http://172.104.207.139:3000
```

#### VPS with custom domain:
```bash
CORS_ORIGINS=http://localhost:3000,http://localhost:5173,http://robotconsole.example.com,https://robotconsole.example.com
```

#### Behind reverse proxy (nginx):
```bash
CORS_ORIGINS=http://localhost:3000,http://localhost:5173,https://your-domain.com
```

### 6. Security Notes
- Only add trusted domains to CORS_ORIGINS
- Use HTTPS in production when possible
- Never use "*" as CORS origin in production
- Consider limiting CORS origins to only necessary domains

## Troubleshooting
- Ensure all origins in CORS_ORIGINS are exact matches (including protocol and port)
- Check that environment variables are loaded correctly
- Restart backend services after changing CORS configuration
- Verify that frontend is configured to use correct API URLs