# Robot Console - Production Deployment Guide

This guide covers deploying the Robot Live Console application in production with proper CORS configuration, security, and auto-restart capabilities.

## 🚀 Quick Start

### Option 1: Docker Deployment (Recommended)

```bash
# Clone and navigate to project
git clone <repository-url>
cd Robot-live-console

# Deploy with Docker
./deploy-docker.sh

# Access the application
# Frontend: http://localhost:3000
# Backend API: http://localhost:8000
# API Docs: http://localhost:8000/docs
```

### Option 2: Traditional Deployment

```bash
# Deploy with traditional setup
./app/scripts/deploy.sh --production --nginx --systemd

# Test deployment
./validate-deployment.sh
```

## 📋 Prerequisites

### For Docker Deployment
- Docker 20.10+
- Docker Compose 2.0+

### For Traditional Deployment
- Python 3.11+
- Node.js 18+
- npm 8+
- (Optional) Nginx for reverse proxy
- (Optional) systemd for service management

## 🔧 Configuration

### Environment Variables

Copy and customize the environment file:

```bash
cp .env.template .env
```

Key variables to configure:

```bash
# Backend Configuration
BACKEND_PORT=8000
BACKEND_HOST=0.0.0.0

# Frontend Configuration
FRONTEND_PORT=5173

# CORS Configuration (IMPORTANT!)
CORS_ORIGINS=http://localhost:3000,http://localhost:5173,https://yourdomain.com

# Security
JWT_SECRET=your-super-secret-jwt-key-change-this-in-production
GITHUB_TOKEN=your-github-token-if-needed

# Production Settings
PRODUCTION_MODE=true
ENABLE_HTTPS=false
```

### CORS Configuration

The application includes enhanced CORS support. Configure allowed origins in `.env`:

```bash
# Development
CORS_ORIGINS=http://localhost:3000,http://localhost:5173

# Production
CORS_ORIGINS=https://yourdomain.com,https://api.yourdomain.com

# Mixed (development + production)
CORS_ORIGINS=http://localhost:3000,http://localhost:5173,https://yourdomain.com
```

## 🛠️ Deployment Options

### Docker Deployment

The Docker deployment includes:
- FastAPI backend with Gunicorn
- React frontend with Nginx
- Automatic health checks
- Volume persistence
- Network isolation

```bash
# Full deployment
./deploy-docker.sh

# View logs
./deploy-docker.sh logs

# Stop services
./deploy-docker.sh stop

# Check status
./deploy-docker.sh status
```

### Traditional Deployment

```bash
# Development mode
./app/scripts/deploy.sh

# Production mode with Nginx and systemd
./app/scripts/deploy.sh --production --nginx --systemd
```

### Manual Deployment

#### Backend Setup

```bash
cd app/backend

# Create virtual environment
python3 -m venv venv
source venv/bin/activate  # Linux/macOS
# or
venv\Scripts\activate     # Windows

# Install dependencies
pip install -r requirements.txt

# Run in development
python main.py

# Run in production
gunicorn main:app -w 4 -k uvicorn.workers.UvicornWorker --bind 0.0.0.0:8000
```

#### Frontend Setup

```bash
cd app/frontend

# Install dependencies
npm install

# Development server
npm run dev

# Production build
npm run build
npm install -g serve
serve -s dist -l 3000
```

## 🔒 Security Configuration

### HTTPS Setup

1. **With Nginx (Recommended)**:
   ```bash
   # Install Certbot
   sudo apt install certbot python3-certbot-nginx
   
   # Get SSL certificate
   sudo certbot --nginx -d yourdomain.com
   
   # Use the generated nginx configuration
   cp robot-console-app.nginx.conf /etc/nginx/sites-available/
   sudo ln -s /etc/nginx/sites-available/robot-console-app.nginx.conf /etc/nginx/sites-enabled/
   sudo nginx -t && sudo systemctl reload nginx
   ```

2. **Update environment variables**:
   ```bash
   ENABLE_HTTPS=true
   CORS_ORIGINS=https://yourdomain.com
   ```

### Security Headers

The deployment includes security headers:
- HSTS (HTTP Strict Transport Security)
- X-Frame-Options: DENY
- X-Content-Type-Options: nosniff
- X-XSS-Protection
- Referrer-Policy

## 🔄 Auto-Restart and Monitoring

### Systemd Services (Traditional Deployment)

The deployment script creates systemd services with:
- Automatic restart on failure
- Health monitoring
- Resource limits
- Security sandboxing

```bash
# Check service status
sudo systemctl status robot-console-backend
sudo systemctl status robot-console-frontend

# View logs
sudo journalctl -u robot-console-backend -f
sudo journalctl -u robot-console-frontend -f

# Restart services
sudo systemctl restart robot-console-backend
sudo systemctl restart robot-console-frontend
```

### Docker Health Checks

Docker containers include health checks:
- Backend: HTTP health endpoint
- Frontend: HTTP connectivity check
- Automatic restart on health failure

## 🧪 Testing and Validation

### Test CORS Configuration

```bash
# Test CORS setup
./test-cors.sh

# Test specific origin
./test-cors.sh --test-origin https://yourdomain.com
```

### Validate Deployment

```bash
# Full deployment validation
./validate-deployment.sh

# Test with custom URLs
./validate-deployment.sh --backend-url https://api.yourdomain.com
```

### Manual Testing

```bash
# Test backend health
curl http://localhost:8000/health

# Test CORS preflight
curl -H "Origin: https://yourdomain.com" \
     -H "Access-Control-Request-Method: POST" \
     -H "Access-Control-Request-Headers: Content-Type" \
     -X OPTIONS http://localhost:8000/auth/login
```

## 🐛 Troubleshooting

### Common CORS Issues

1. **403 Forbidden Errors**:
   - Check CORS_ORIGINS environment variable
   - Verify origins match exactly (no trailing slashes)
   - Ensure backend is loading .env file

2. **Preflight Request Failures**:
   - Verify OPTIONS method is allowed
   - Check browser developer tools Network tab
   - Confirm CORS headers are present in response

### Port Conflicts

The deployment script automatically checks for port conflicts:

```bash
# Manual port checking
netstat -tlnp | grep :8000  # Check backend port
netstat -tlnp | grep :3000  # Check frontend port
```

### Service Issues

```bash
# Check service logs
docker-compose logs backend     # Docker deployment
sudo journalctl -u robot-console-backend  # Systemd deployment

# Restart services
docker-compose restart         # Docker deployment
sudo systemctl restart robot-console-backend  # Systemd deployment
```

### Network Issues

```bash
# Test connectivity
curl -v http://localhost:8000/health
curl -v http://localhost:3000/

# Check firewall
sudo ufw status
sudo iptables -L
```

## 📊 Monitoring

### Health Endpoints

- Backend Health: `GET /health`
- Service Status: `GET /health/services`
- Feature Status: `GET /health/features`

### Log Files

#### Traditional Deployment
- Backend: `app/backend/logs/`
- Frontend: `app/frontend/logs/`
- Nginx: `/var/log/nginx/`

#### Docker Deployment
- View with: `docker-compose logs [service]`
- Persist with volumes (configured automatically)

## 🔄 Updates and Maintenance

### Docker Deployment Updates

```bash
# Pull latest changes
git pull

# Rebuild and redeploy
./deploy-docker.sh build
./deploy-docker.sh restart
```

### Traditional Deployment Updates

```bash
# Pull latest changes
git pull

# Update backend
cd app/backend
source venv/bin/activate
pip install -r requirements.txt
sudo systemctl restart robot-console-backend

# Update frontend
cd app/frontend
npm install
npm run build
sudo systemctl restart robot-console-frontend
```

## 📞 Support

For issues and support:
1. Check the troubleshooting section above
2. Run the validation scripts
3. Check service logs
4. Review CORS configuration
5. Verify environment variables

## 🎯 Production Checklist

- [ ] Configure proper CORS origins
- [ ] Set secure JWT secret
- [ ] Enable HTTPS with valid certificates
- [ ] Configure proper firewall rules
- [ ] Set up monitoring and health checks
- [ ] Configure automatic backups
- [ ] Test disaster recovery procedures
- [ ] Document deployment-specific configurations