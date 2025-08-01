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

- Docker (with ability to run Linux containers)
- Node.js (v16 or higher)
- Python 3.8+
- 4GB+ RAM (for ROS/Gazebo simulation)

### Installation

1. **Clone the repository**:
   ```bash
   git clone https://github.com/shanooo773/Robot-live-console.git
   cd Robot-live-console
   ```

2. **Run the setup script**:
   ```bash
   ./setup.sh
   ```

   This single command will:
   - Check system requirements
   - Build the Docker image (10-15 minutes first time)
   - Set up the FastAPI backend
   - Set up the React frontend
   - Start both servers

3. **Access the application**:
   - Frontend: http://localhost:3000
   - Backend API: http://localhost:8000

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

```bash
# Start everything
./setup.sh start

# Stop servers
./setup.sh stop

# Check status
./setup.sh status

# Setup dependencies only
./setup.sh setup

# Build Docker image only
./setup.sh build

# Show help
./setup.sh help
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

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🙏 Acknowledgments

- [ROS](https://www.ros.org/) - Robot Operating System
- [Gazebo](http://gazebosim.org/) - Robot simulation
- [Monaco Editor](https://microsoft.github.io/monaco-editor/) - Code editor
- [FastAPI](https://fastapi.tiangolo.com/) - Backend framework
- [React](https://reactjs.org/) - Frontend framework
