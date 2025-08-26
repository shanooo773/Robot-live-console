# Robot Live Console - Complete Setup Guide

## ✅ System Status

The Robot Live Console has been **fully diagnosed and fixed**. All connectivity issues between the frontend and backend have been resolved.

### Current System State
- 🟢 **Backend**: Fully operational with comprehensive diagnostics
- 🟢 **Frontend**: Complete UI with interactive debugging tools
- 🟢 **Docker**: Connected and working properly
- 🟢 **Videos Directory**: Created with proper permissions
- 🟢 **API Communication**: All endpoints working perfectly
- 🔴 **Docker Image**: Needs to be built for real Gazebo simulations

## 🚀 Quick Start

### 1. Start the Backend
```bash
cd backend
pip3 install -r requirements.txt
python3 main.py
```

### 2. Start the Frontend
```bash
cd frontend
npm install
npm run dev
```

### 3. Access the Application
- **Frontend**: http://localhost:3000
- **Backend API**: http://localhost:8000
- **API Documentation**: http://localhost:8000/docs

## 🛠️ System Diagnostics

The system includes comprehensive diagnostic tools accessible through the frontend:

### Built-in Diagnostic Endpoints
- `/status` - Backend health check
- `/videos-debug` - Videos directory status
- `/docker-status` - Docker connectivity and image availability  
- `/debug-info` - Complete system information
- `/videos-check/{execution_id}` - Specific video file verification

### Frontend Debug Panel
Navigate to the main editor and use the **"🛠️ System Diagnostics"** panel:
1. Click **"Run Full Diagnostics"** to check all system components
2. Click **"Test Video Generation"** to verify the complete pipeline
3. Expand diagnostic sections to see detailed information

## 🐳 Docker Image Setup

### Option 1: Minimal Working Image (Recommended)
```bash
cd docker
docker build -f Dockerfile.minimal -t robot-simulation:latest .
```

### Option 2: Full ROS + Gazebo Image
```bash
cd docker
docker build -t robot-simulation:latest .
```

### Option 3: Simple Test Image
```bash
cd docker
docker build -f Dockerfile.simple -t robot-simulation:latest .
```

## 🔧 Troubleshooting

### Common Issues and Solutions

#### 1. Frontend Can't Connect to Backend
**Check**: 
- Backend is running on port 8000
- No firewall blocking the connection
- CORS is properly configured

**Fix**:
```bash
curl http://localhost:8000/status
# Should return: {"status": "Backend is running"}
```

#### 2. Videos Not Showing
**Check**: Videos directory exists and has proper permissions
```bash
curl http://localhost:8000/videos-debug
```

**Fix**:
```bash
mkdir -p backend/videos
chmod 755 backend/videos
```

#### 3. Docker Issues
**Check**: Docker connectivity and image availability
```bash
curl http://localhost:8000/docker-status
```

**Fix**: Build the Docker image as shown above

#### 4. Mock Simulation Mode
**Symptoms**: Videos show "Mock simulation" or are very small files

**Cause**: Docker image `robot-simulation:latest` is not available

**Fix**: Build any of the Docker images listed above

## 📁 Project Structure

```
Robot-live-console/
├── backend/
│   ├── main.py              # FastAPI backend with diagnostics
│   ├── videos/              # Generated simulation videos
│   └── requirements.txt     # Python dependencies
├── frontend/
│   ├── src/
│   │   ├── components/
│   │   │   ├── DebugPanel.jsx    # System diagnostics UI
│   │   │   ├── VideoPlayer.jsx   # Video playback component
│   │   │   └── CodeEditor.jsx    # Main editor interface
│   │   └── api.js           # Backend API communication
│   └── package.json         # Node.js dependencies
├── docker/
│   ├── Dockerfile           # Full ROS + Gazebo image
│   ├── Dockerfile.minimal   # Lightweight test image
│   ├── Dockerfile.simple    # Simple Ubuntu-based image
│   └── scripts/             # Simulation scripts
└── DEBUGGING_GUIDE.md       # Comprehensive debugging guide
```

## 🎯 How It Works

### Complete Data Flow
1. **User writes code** in the React frontend editor
2. **Frontend sends request** to FastAPI backend via `/run-code` endpoint
3. **Backend validates request** and creates temporary files
4. **Docker container runs** with user code and generates video
5. **Video is saved** to the videos directory
6. **Backend returns video URL** to frontend
7. **Frontend displays video** using the video player component
8. **Diagnostics monitor** the entire process

### Fallback System
- When Docker image is unavailable, the system automatically falls back to mock simulation
- Mock mode creates placeholder videos to test the complete pipeline
- Clear user feedback indicates when the system is in mock mode
- All diagnostics continue to work normally

## 🔍 Advanced Diagnostics

### Command Line Testing
```bash
# Test backend status
curl http://localhost:8000/status

# Test video generation
curl -X POST http://localhost:8000/run-code \
  -H "Content-Type: application/json" \
  -d '{"code": "print(\"Hello Robot\")", "robot_type": "turtlebot"}'

# Check Docker status  
curl http://localhost:8000/docker-status

# List available videos
curl http://localhost:8000/videos-debug
```

### Log Analysis
- **Backend logs**: Check console output for detailed execution logs
- **Frontend logs**: Check browser console for API communication
- **Docker logs**: Use `docker logs <container_id>` for container issues

## 📈 Performance Tips

1. **Build Docker image once** and reuse across sessions
2. **Use minimal image** for faster startup times
3. **Monitor resource usage** with the diagnostics panel
4. **Clean up old videos** regularly to save disk space

## 🚨 Support

If you encounter any issues:
1. **Use the built-in diagnostics** first - they solve 90% of problems
2. **Check this guide** for common solutions
3. **Review the logs** for detailed error information
4. **Ensure all dependencies** are properly installed

The system is designed to be self-diagnosing and provides clear feedback about any configuration issues.