🤖 Robot Development Console Booking System

An interactive web platform for booking development sessions and testing robotics code in professional simulation environments. Users can reserve time slots, write ROS Python code in a Monaco editor, and optionally execute simulations in Gazebo via Docker containers with VNC/NoVNC GUI access.

✨ Features

Time Slot Booking System: Reserve development sessions with guaranteed access to computing resources

Professional Development Console: Monaco Editor with Python syntax highlighting, autocomplete, and ROS library support

Multiple Robot Environments: TurtleBot3 navigation, Robot Arm manipulation, Dexterous Hand control

Optional Real-time Gazebo Simulation: Dockerized ROS Noetic simulations with physics and GUI

VNC/NoVNC GUI Access: Browser-based visualization of simulations

Secure Sandboxed Execution: Docker containers with CPU/memory limits

One-Command Setup: Start frontend, backend, and optional simulation system

Note: Direct MP4 video generation is no longer the default; users interact via VNC/NoVNC GUI.

🚀 Quick Start
Prerequisites

Docker (Linux containers) v20.10+

Docker Compose v2+

Node.js v18+

Python 3.8+

4GB+ RAM (recommended for simulation)

Ports 8080 and 5901 free

Installation

Clone the repository:

git clone https://github.com/shanooo773/Robot-live-console.git
cd Robot-live-console


Validate environment (optional):

./setup.sh validate


Start backend and frontend:

./setup.sh start


Optional: Start VNC/NoVNC Gazebo GUI:

docker-compose up -d


Access the application:

Frontend: http://localhost:5173

Backend API: http://localhost:8000

NoVNC Gazebo GUI: http://localhost:8080/vnc.html

VNC password: gazebo

🔑 Demo Credentials

For quick testing, use these demo credentials:

**Demo User:**
- Email: `demo@user.com`
- Password: `password`

**Demo Admin:**
- Email: `admin@demo.com`  
- Password: `password`

**Default Admin:**
- Email: `admin@robot-console.com`
- Password: `admin123`

> See [DEMO_CREDENTIALS.md](DEMO_CREDENTIALS.md) for more details and testing instructions.

🏗️ Architecture
Robot-live-console/
├── frontend/           # React + Vite interface (booking, console)
├── backend/            # FastAPI booking/admin backend
├── docker/             # ROS/Gazebo simulation stack
│   ├── Dockerfile
│   ├── robots/
│   └── scripts/
├── setup.sh            # Main setup script
├── docker-compose.yml  # VNC/NoVNC Gazebo configuration
└── README.md

🐳 VNC/NoVNC Gazebo GUI Issues & Fixes
Problem Summary

Users reported multiple failure scenarios:

Black screen in NoVNC

"Cannot connect" errors in browser

Gazebo GUI not rendering

Slow or unresponsive GUI

Container startup failures related to GUI dependencies

Root Causes

Missing GUI libraries and Mesa/OpenGL drivers

Misconfigured VNC xstartup scripts

NoVNC websocket proxy/port mapping issues

Gazebo GPU-less rendering misconfiguration

Solution Overview

Enhanced Docker Configuration

Added tigervnc-standalone-server, tigervnc-xorg-extension

Installed Mesa/OpenGL libraries (libgl1-mesa-glx, libglu1-mesa, libgl1-mesa-dri)

D-Bus support for GUI applications

Software rendering for GPU-less containers

Fixed VNC Startup Scripts

Proper DISPLAY and LIBGL_ALWAYS_SOFTWARE setup

D-Bus initialization

Correct Gazebo GUI launch sequence

Improved NoVNC Configuration

Correct port mapping

Websocket proxy fixes

Error detection and recovery

Docker Compose Enhancements

Increased shared memory (shm_size: 1g)

Software rendering environment variables

Security configuration for X11 forwarding

Automated Fix Tools

debug-vnc.sh: Check container status, ports, ROS topics

apply-vnc-fixes.sh: Apply missing packages, fix VNC configs

test-vnc.sh: Minimal test containers to validate VNC connectivity

Usage
# Diagnose VNC/NoVNC issues
./debug-vnc.sh check
./apply-vnc-fixes.sh apply

# Access Gazebo GUI
http://localhost:8080/vnc.html
Password: gazebo


Verification after fixes:

✅ VNC server running on :1 (port 5901)

✅ NoVNC GUI accessible on port 8080

✅ Gazebo GUI visible and interactive in browser

✅ ROS topics active

🔧 Backend API Reference

GET / – Health check

GET /robots – List of available robots

POST /run-code – Execute ROS Python code (optional Docker simulation)

⚙️ Configuration (.env)
BACKEND_PORT=8000
FRONTEND_PORT=3000
DOCKER_IMAGE_NAME=robot-simulation
DOCKER_MEMORY_LIMIT=2g
DOCKER_CPU_LIMIT=1.0
MAX_EXECUTION_TIME=60
DISABLE_NETWORKING=true

🛠️ Management Commands
./setup.sh start          # Frontend + backend
./setup.sh stop           # Stop servers
./setup.sh status         # Check service status
./setup.sh validate       # Validate environment
./setup.sh cleanup        # Cleanup temp files/videos
docker-compose up -d      # Start VNC/NoVNC Gazebo GUI
docker-compose down       # Stop Gazebo container

🚀 VPS/Production Deployment
For deploying on a VPS and resolving CORS errors, see: [VPS_CORS_GUIDE.md](VPS_CORS_GUIDE.md)

Common VPS deployment issues:
- CORS errors during sign-in/sign-up: Configure `CORS_ORIGINS` environment variable
- Authentication failing: Update frontend API URLs for production
- Preflight request failures: Ensure proper CORS configuration

Quick VPS setup:
```bash
# Copy and configure environment
cp app/.env.template app/.env
# Edit app/.env and set:
# CORS_ORIGINS=http://localhost:3000,http://localhost:5173,http://YOUR_VPS_IP
```

🔒 Security Features

Sandboxed execution in Docker

CPU/memory limits

Optional network restriction

Automatic cleanup of temp files

Execution time limits

📄 License

MIT License
