# ROS Noetic + Gazebo Simulation System

This document explains how to build and use the ROS Noetic + Gazebo simulation system with video recording.

## Overview

This system provides:
- **Real ROS Noetic + Gazebo simulation** in Docker containers
- **Video recording** with ffmpeg and virtual X display
- **REST API** for simulation control
- **File upload** for custom URDF and world files
- **Fallback mock system** when Docker is unavailable

## Quick Start

### 1. Build the Docker Image

The simulation requires a Docker image with ROS Noetic, Gazebo, and video recording tools:

```bash
# Build the simulation Docker image
cd docker
docker build -t robot-simulation:latest .
```

**Note:** This build takes 15-20 minutes the first time as it downloads ROS Noetic, Gazebo, and all dependencies.

### 2. Start the Backend

```bash
cd backend
python3 -m pip install -r requirements.txt
python3 main.py
```

The backend will start on http://localhost:8000

### 3. Start the Frontend

```bash
cd frontend
npm install
npm run dev
```

The frontend will start on http://localhost:3000

### 4. Use the System

#### Option A: Simulation Uploader (New Feature)
1. Go to the "Simulation Uploader" tab
2. Upload your URDF and World files
3. Set simulation duration
4. Click "Run Gazebo Simulation"
5. Download or preview the generated video

#### Option B: Code Editor (Existing Feature)
1. Select a robot type (arm, hand, turtlebot)
2. Write ROS Python code
3. Click "Run Code"
4. View the simulation video

## API Endpoints

### New Simulation Endpoints

#### `POST /upload-files`
Upload URDF and world files for simulation.

**Request:** `multipart/form-data`
- `urdf_file`: URDF file (.urdf or .xacro)
- `world_file`: Gazebo world file (.world)

**Response:**
```json
{
  "success": true,
  "upload_id": "uuid",
  "urdf_path": "/path/to/urdf",
  "world_path": "/path/to/world"
}
```

#### `POST /simulate`
Run a simulation with uploaded files.

**Request:**
```json
{
  "urdf_path": "/path/to/robot.urdf",
  "world_path": "/path/to/world.world", 
  "duration": 10
}
```

**Response:**
```json
{
  "success": true,
  "video_url": "/videos/execution-id.mp4",
  "execution_id": "uuid"
}
```

### Existing Endpoints

#### `POST /run-code`
Execute Python code with a predefined robot.

#### `GET /robots`
Get available robot types.

#### `GET /videos/{execution_id}.mp4`
Download simulation videos.

## Docker Image Details

The `robot-simulation:latest` image includes:

- **Base:** Ubuntu 20.04
- **ROS:** Noetic Desktop Full
- **Gazebo:** 11 (included with ROS Noetic)
- **Video Tools:** xvfb, ffmpeg, x11-utils
- **Controllers:** Position controllers, joint state controllers
- **Scripts:** Bash recording script, Python simulation runner

### Image Size
- Compressed: ~2-3 GB
- Uncompressed: ~6-8 GB

### Build Time
- First build: 15-20 minutes
- Subsequent builds: 2-5 minutes (with cache)

## File Structure

```
docker/
├── Dockerfile                 # ROS Noetic + Gazebo + ffmpeg
├── launch/
│   └── spawn_robot.launch    # Robot spawning launch file
├── robots/
│   ├── arm/
│   │   ├── arm.urdf          # Robot arm description
│   │   └── controllers.yaml  # Arm controllers
│   ├── hand/
│   │   ├── hand.urdf         # Robot hand description  
│   │   └── controllers.yaml  # Hand controllers
│   ├── turtlebot/
│   │   ├── turtlebot.urdf    # TurtleBot description
│   │   └── controllers.yaml  # TurtleBot controllers
│   └── worlds/
│       ├── arm_world.world   # World for arm simulation
│       ├── hand_world.world  # World for hand simulation
│       └── turtlebot_world.world # World for TurtleBot
└── scripts/
    ├── record_simulation.sh  # Main recording script
    └── run_simulation.py     # Python simulation coordinator
```

## How It Works

### Real Simulation Process

1. **File Upload**: User uploads URDF and world files via web interface
2. **Container Launch**: Docker container starts with:
   - ROS Noetic environment
   - Virtual X display (Xvfb)
   - Volume mounts for files and videos
3. **Gazebo Launch**: Robot spawned in world using roslaunch
4. **Video Recording**: ffmpeg captures X display for specified duration
5. **User Code Execution**: Custom Python code runs during simulation
6. **Video Output**: MP4 file saved to host-accessible directory

### Fallback System

If Docker image is not available or simulation fails:
- System automatically falls back to mock video generation
- Ensures the API always returns a video response
- Logs indicate whether real or mock simulation was used

## Troubleshooting

### Docker Image Build Issues

```bash
# Clean Docker cache and rebuild
docker system prune -a
cd docker
docker build --no-cache -t robot-simulation:latest .
```

### Memory Issues

The simulation requires significant resources:
- **Minimum RAM:** 4GB
- **Recommended RAM:** 8GB+
- **CPU:** 2+ cores recommended

Adjust Docker resource limits if needed:

```bash
# Check current limits
docker system info | grep -i memory

# Increase memory limit in Docker Desktop settings
```

### Video Recording Issues

Common issues and solutions:

1. **Empty video files**: Check if Xvfb started correctly
2. **Garbled video**: Increase container memory
3. **No video output**: Check volume mount permissions

### Network Issues

The containers run with `--network none` for security. If you need network access:

1. Remove `network_mode="none"` from container run parameters
2. Add necessary firewall rules
3. Consider security implications

## Sample Files

### Sample URDF (minimal robot)

```xml
<?xml version="1.0"?>
<robot name="sample_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/sample_robot</robotNamespace>
    </plugin>
  </gazebo>
</robot>
```

### Sample World (minimal environment)

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
  </world>
</sdf>
```

## Performance Tips

1. **Reduce simulation duration** for faster testing
2. **Use simpler worlds** to reduce computational load
3. **Build Docker image once** and reuse across deployments
4. **Monitor resource usage** with `docker stats`
5. **Clean up old containers** with `docker container prune`

## Security Considerations

- Containers run with no network access
- File uploads are validated by extension
- Resource limits prevent DoS attacks
- Temporary files are cleaned up automatically
- User code execution is sandboxed in containers

## Development

To modify the simulation system:

1. **Update robot descriptions** in `docker/robots/`
2. **Modify launch files** in `docker/launch/`
3. **Adjust recording script** in `docker/scripts/record_simulation.sh`
4. **Rebuild Docker image** after changes
5. **Test with sample files** before deployment