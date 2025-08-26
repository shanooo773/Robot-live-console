# 🤖 Robot Live Console

An interactive web-based platform for writing Python code to control robots in a simulated environment using ROS and Gazebo. Users can write ROS Python code in a Monaco editor and see their robot behavior as a video simulation.

## ✨ Features

- **Interactive Code Editor**: Monaco Editor with Python syntax highlighting and autocomplete
- **Robot Selection**: Choose between different robot types (Arm, Hand, TurtleBot3)
- **Real-time Simulation**: Execute code in a Dockerized ROS Noetic environment with Gazebo
- **Video Output**: See simulation results as MP4 videos
- **Secure Execution**: Sandboxed Docker containers with resource limits
- **One-Command Setup**: Get everything running with a single script

## 🚀 Quick Start

### Prerequisites

- **Docker** (with ability to run Linux containers) - v20.10+
- **Node.js** (v18 or higher) 
- **Python** 3.8+
- **4GB+ RAM** (for ROS/Gazebo simulation)
- **2GB+ free disk space** (for Docker images)

### Installation

1. **Clone the repository**:
   ```bash
   git clone https://github.com/shanooo773/Robot-live-console.git
   cd Robot-live-console
   ```

2. **Validate your environment** (optional but recommended):
   ```bash
   ./setup.sh validate
   ```

3. **Run the setup script**:
   ```bash
   ./setup.sh start
   ```

   This command will:
   - ✅ Check system requirements
   - ✅ Validate environment and available resources
   - ✅ Build the Docker image (10-15 minutes first time)
   - ✅ Set up Python virtual environment for backend
   - ✅ Install and secure frontend dependencies
   - ✅ Start both servers

4. **Access the application**:
   - **Frontend**: http://localhost:5173 (Vite dev server)
   - **Backend API**: http://localhost:8000
   - **API Documentation**: http://localhost:8000/docs

## 🏗️ Architecture

```
Robot-live-console/
├── frontend/           # React + Vite + Monaco Editor
│   ├── src/
│   │   ├── components/
│   │   │   ├── CodeEditor.jsx
│   │   │   ├── RobotSelector.jsx
│   │   │   └── VideoPlayer.jsx
│   │   ├── api.js
│   │   └── constants.js
│   └── package.json
├── backend/            # FastAPI backend
│   ├── main.py
│   ├── requirements.txt
│   ├── videos/         # Generated simulation videos
│   └── temp/           # Temporary execution files
├── docker/             # ROS simulation environment
│   ├── Dockerfile      # ROS Noetic + Gazebo + Python
│   ├── robots/         # URDF files and world configs
│   │   ├── arm/
│   │   ├── hand/
│   │   ├── turtlebot/
│   │   └── worlds/
│   └── scripts/
│       └── run_simulation.py
├── setup.sh            # Main setup script
├── .env.template       # Environment configuration
└── README.md
```

## 🤖 Available Robots

### 1. Robot Arm
- **Description**: 2-DOF robotic arm with base and two joints
- **Use Cases**: Pick and place, reaching tasks
- **Sample Code**: Joint position control using ROS topics

### 2. Robot Hand
- **Description**: Simple gripper with fingers and thumb
- **Use Cases**: Grasping, manipulation tasks
- **Sample Code**: Finger control for opening/closing

### 3. TurtleBot3
- **Description**: Differential drive mobile robot
- **Use Cases**: Navigation, path following, obstacle avoidance
- **Sample Code**: Velocity commands for movement

## 📝 Code Examples

### TurtleBot Movement
```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def move_turtlebot():
    rospy.init_node('turtlebot_controller', anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    move_cmd = Twist()
    rospy.sleep(1)
    
    # Move forward
    move_cmd.linear.x = 0.2
    vel_pub.publish(move_cmd)
    rospy.sleep(2)
    
    # Stop
    move_cmd.linear.x = 0.0
    vel_pub.publish(move_cmd)

if __name__ == '__main__':
    try:
        move_turtlebot()
    except rospy.ROSInterruptException:
        pass
```

### Robot Arm Control
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

def move_arm():
    rospy.init_node('arm_controller', anonymous=True)
    
    joint1_pub = rospy.Publisher('/robot_arm/joint1_position_controller/command', Float64, queue_size=10)
    joint2_pub = rospy.Publisher('/robot_arm/joint2_position_controller/command', Float64, queue_size=10)
    
    rospy.sleep(1)
    
    # Move to 45 degrees
    joint1_pub.publish(0.785)
    joint2_pub.publish(0.524)

if __name__ == '__main__':
    try:
        move_arm()
    except rospy.ROSInterruptException:
        pass
```

## 🔧 API Reference

### Backend Endpoints

#### `GET /`
Health check endpoint.

#### `GET /robots`
Get list of available robot types.
```json
{
  "robots": ["arm", "hand", "turtlebot"]
}
```

#### `POST /run-code`
Execute Python code in robot simulation.

**Request:**
```json
{
  "code": "# Python code here",
  "robot_type": "turtlebot"
}
```

**Response:**
```json
{
  "success": true,
  "video_url": "/videos/execution-id.mp4",
  "execution_id": "uuid-string"
}
```

#### `GET /videos/{execution_id}.mp4`
Serve simulation video files.

## ⚙️ Configuration

### Environment Variables (.env)

```bash
# Backend Configuration
BACKEND_PORT=8000
BACKEND_HOST=0.0.0.0

# Frontend Configuration  
FRONTEND_PORT=3000

# Docker Configuration
DOCKER_IMAGE_NAME=robot-simulation
DOCKER_MEMORY_LIMIT=2g
DOCKER_CPU_LIMIT=1.0

# Video Configuration
VIDEO_DURATION=10
VIDEO_FRAMERATE=30

# Security Configuration
MAX_EXECUTION_TIME=60
DISABLE_NETWORKING=true
```

## 🐳 Docker Details

The simulation runs in a secure Docker container with:
- **Base Image**: `osrf/ros:noetic-desktop-full`
- **Components**: ROS Noetic + Gazebo 11 + Python 3.8
- **Security**: No network access, limited CPU/memory
- **Video Recording**: ffmpeg with virtual X display
- **Resource Limits**: 2GB RAM, 1 CPU core max

## 🛠️ Management Commands

### Basic Commands
```bash
# Start everything (recommended for first-time setup)
./setup.sh start

# Stop running servers
./setup.sh stop

# Check status of all services
./setup.sh status

# Show system information
./setup.sh info
```

### Setup Commands
```bash
# Setup dependencies only (without starting)
./setup.sh setup

# Build Docker image only
./setup.sh build

# Validate environment requirements
./setup.sh validate
```

### Maintenance Commands
```bash
# Clean up temporary files and old videos
./setup.sh cleanup

# Show all available commands
./setup.sh help
```

### Environment Variables
```bash
# Skip Docker operations (for development)
SKIP_DOCKER=1 ./setup.sh start

# Override default ports
FRONTEND_PORT=3000 BACKEND_PORT=8080 ./setup.sh start
```

## 🔒 Security Features

- **Sandboxed Execution**: Code runs in isolated Docker containers
- **Resource Limits**: CPU and memory constraints prevent abuse
- **No Network Access**: Containers cannot access external networks
- **Temporary Files**: All execution files are cleaned up automatically
- **Time Limits**: Maximum execution time prevents infinite loops

## 🐛 Troubleshooting

### Common Issues

1. **Docker build fails**
   ```bash
   # Clean Docker cache and retry
   docker system prune -a
   ./setup.sh build
   ```

2. **Backend connection error**
   ```bash
   # Check if backend is running
   curl http://localhost:8000/
   
   # View backend logs
   tail -f backend/backend.log
   ```

3. **Frontend not loading**
   ```bash
   # Check if frontend is running
   curl http://localhost:3000/
   
   # View frontend logs
   tail -f frontend/frontend.log
   ```

4. **Simulation videos not generating**
   - Ensure Docker has enough memory allocated (minimum 2GB)
   - Check Docker container logs for Gazebo errors
   - Verify X11 forwarding is working in the container

### Log Files

- Backend: `backend/backend.log`
- Frontend: `frontend/frontend.log`
- Docker builds: Check terminal output during build

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature-name`
3. Make your changes and test thoroughly
4. Submit a pull request with a clear description

---

# 🖥️ VNC/NoVNC Gazebo GUI Access

This repository now supports **browser-based Gazebo GUI access** via VNC and NoVNC, perfect for running on remote VPS servers.

## 🚀 Quick Start with VNC/NoVNC

### Prerequisites for VNC Setup
- Docker and Docker Compose
- 4GB+ RAM (for Gazebo GUI)
- Port 8080 and 5901 available

### 1. Local Development

```bash
# Clone repository
git clone https://github.com/shanooo773/Robot-live-console.git
cd Robot-live-console

# Build and start with docker-compose
docker-compose up -d

# Access Gazebo GUI in browser
open http://localhost:8080
```

### 2. VPS Deployment 

```bash
# On your VPS
git clone https://github.com/shanooo773/Robot-live-console.git
cd Robot-live-console

# Build and start the container
docker-compose up -d

# Access from anywhere via browser
# Replace YOUR-VPS-IP with your server's IP
open http://YOUR-VPS-IP:8080
```

## 🔑 VNC Access Details

- **NoVNC Web Interface**: `http://<VPS-IP>:8080`
- **Direct VNC**: `<VPS-IP>:5901` 
- **Default VNC Password**: `gazebo`
- **Display Resolution**: 1024x768 (configurable)

## 🗺️ Custom Gazebo Worlds

### Adding Your Own World Files

1. **Place world files** in the `custom_worlds/` directory:
   ```bash
   cp my_custom_world.world custom_worlds/
   ```

2. **Restart the container** to mount new files:
   ```bash
   docker-compose restart gazebo
   ```

3. **Access in Gazebo**:
   - Open File → Open World
   - Navigate to `/opt/simulation/custom_worlds/`
   - Select your world file

### Sample World File
Create `custom_worlds/sample.world`:
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="sample_world">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <!-- Add your custom models here -->
  </world>
</sdf>
```

## 🐳 Docker Compose Configuration

The `docker-compose.yml` includes:

- **Service**: `gazebo` - Main Gazebo GUI container
- **Ports**: 
  - `8080:8080` - NoVNC web interface
  - `5901:5901` - VNC server
- **Volumes**:
  - `./docker/robots/worlds` - Built-in world files
  - `./custom_worlds` - Your custom world files
- **Auto-restart**: Container restarts unless manually stopped

## 🔧 Configuration Options

### Change VNC Password
```bash
# Modify docker-compose.yml
environment:
  - VNC_PASSWORD=your_new_password

# Rebuild container
docker-compose up -d --build
```

### Change Display Resolution
Edit the VNC geometry in `docker/Dockerfile`:
```bash
vncserver :1 -geometry 1920x1080 -depth 24 -SecurityTypes None
```

### Custom Gazebo Startup
Modify `/root/.vnc/xstartup` in the Dockerfile to launch specific worlds:
```bash
gazebo --verbose /opt/simulation/custom_worlds/my_world.world &
```

## 🛠️ Management Commands

```bash
# Start Gazebo GUI
docker-compose up -d

# Stop services
docker-compose down

# View logs
docker-compose logs gazebo

# Rebuild after changes
docker-compose up -d --build

# Access container shell
docker-compose exec gazebo bash
```

## 🌐 Network and Security

### Firewall Configuration (VPS)
```bash
# Allow VNC and NoVNC ports
sudo ufw allow 8080/tcp  # NoVNC
sudo ufw allow 5901/tcp  # VNC (optional)
```

### SSH Tunneling (Alternative)
For enhanced security, access via SSH tunnel:
```bash
# On your local machine
ssh -L 8080:localhost:8080 user@your-vps-ip

# Then access via localhost:8080
```

## 🐛 Troubleshooting VNC/NoVNC

### Quick Fix Tools

For VNC/NoVNC issues, use these automated tools:

```bash
# Comprehensive diagnosis
./debug-vnc.sh check

# Apply fixes to existing container  
./apply-vnc-fixes.sh apply

# Test VNC functionality
./test-vnc.sh all
```

### Common Issues

1. **NoVNC shows black screen**
   ```bash
   # Check if VNC server is running
   docker-compose exec gazebo ps aux | grep vnc
   
   # Apply automatic fixes
   ./apply-vnc-fixes.sh apply
   
   # Restart the container
   docker-compose restart gazebo
   ```

2. **Cannot connect to VNC**
   ```bash
   # Check port accessibility
   curl http://your-vps-ip:8080
   
   # Check container logs
   docker-compose logs gazebo
   
   # Run comprehensive diagnostics
   ./debug-vnc.sh check
   ```

3. **Gazebo fails to start**
   ```bash
   # Check available memory
   free -h
   
   # Check GPU/3D acceleration
   docker-compose exec gazebo glxinfo | grep rendering
   
   # Apply VNC fixes (includes Gazebo startup fixes)
   ./apply-vnc-fixes.sh apply
   ```

4. **Custom world not loading**
   ```bash
   # Verify file permissions
   ls -la custom_worlds/
   
   # Check mount inside container
   docker-compose exec gazebo ls -la /opt/simulation/custom_worlds/
   ```

### Detailed Troubleshooting

For comprehensive troubleshooting guides, see:
- [`VNC_TROUBLESHOOTING.md`](VNC_TROUBLESHOOTING.md) - Detailed issue analysis and solutions
- [`SETUP_GUIDE.md`](SETUP_GUIDE.md) - Complete setup instructions with multiple approaches

### Performance Optimization

- **Increase container memory**: Add `mem_limit: 4g` to docker-compose.yml
- **Enable GPU acceleration**: Add GPU support for better performance
- **Reduce display quality**: Lower VNC color depth for faster connection

## 🐛 Troubleshooting

### Common Setup Issues

#### ❌ Docker Build Fails
```bash
# Check Docker daemon status
docker info

# Clean up Docker resources
./setup.sh cleanup
docker system prune -a

# Try building with more memory
export DOCKER_BUILDKIT=1
./setup.sh build
```

#### ❌ Port Already in Use
```bash
# Check what's using the ports
./setup.sh status

# Kill processes using the ports
sudo lsof -ti:8000,5173 | xargs kill -9

# Or use different ports
FRONTEND_PORT=3000 BACKEND_PORT=8080 ./setup.sh start
```

#### ❌ NPM Security Vulnerabilities
```bash
# The setup script now automatically fixes these
./setup.sh setup

# Or manually fix
cd frontend && npm audit fix
```

#### ❌ Python Virtual Environment Issues
```bash
# Remove and recreate the environment
rm -rf backend/venv
./setup.sh setup
```

### Performance Issues

#### 🐌 Slow Simulation
- **Check available resources**: `./setup.sh info`
- **Increase Docker memory**: Edit `.env` file, set `DOCKER_MEMORY_LIMIT=4g`
- **Close other applications** to free up RAM
- **Use SSD storage** for better I/O performance

#### 🐌 Slow Frontend Loading
```bash
# Clear npm cache
cd frontend && npm cache clean --force

# Update dependencies
npm update

# Use production build for faster loading
npm run build && npm run preview
```

### Development Issues

#### 🔧 Hot Reload Not Working
```bash
# Ensure Vite is configured properly
cd frontend && cat vite.config.js

# Check if files are being watched
./setup.sh status
```

#### 🔧 Backend API Not Responding
```bash
# Check backend logs
cat backend/backend.log

# Restart backend only
./setup.sh stop
cd backend && source venv/bin/activate && python main.py
```

### Docker Issues

#### 🐳 Docker Image Issues
```bash
# Force rebuild without cache
docker build --no-cache -t robot-simulation:latest docker/

# Check image exists
docker images | grep robot-simulation

# Test container manually
docker run -it robot-simulation:latest /bin/bash
```

#### 🐳 Container Permission Issues
```bash
# On Linux, add user to docker group
sudo usermod -aG docker $USER
newgrp docker

# Check Docker permissions
docker run hello-world
```

### Environment Validation

```bash
# Run comprehensive environment check
./setup.sh validate

# Check system resources
./setup.sh info

# Verify all dependencies
./setup.sh validate && echo "✅ Environment OK"
```

### Getting Help

If you're still experiencing issues:

1. **Check the logs**: `backend/backend.log` and `frontend/frontend.log`
2. **Run validation**: `./setup.sh validate`
3. **Clean up and retry**: `./setup.sh cleanup && ./setup.sh start`
4. **Check system requirements**: Ensure you have enough RAM and disk space
5. **Create an issue**: Include output from `./setup.sh info` and relevant logs

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🙏 Acknowledgments

- [ROS](https://www.ros.org/) - Robot Operating System
- [Gazebo](http://gazebosim.org/) - Robot simulation
- [Monaco Editor](https://microsoft.github.io/monaco-editor/) - Code editor
- [FastAPI](https://fastapi.tiangolo.com/) - Backend framework
- [React](https://reactjs.org/) - Frontend framework
