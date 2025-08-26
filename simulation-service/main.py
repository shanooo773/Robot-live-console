"""
Simulation Service - Standalone Docker/ROS/Gazebo simulation service
Handles resource-intensive robot simulation tasks independently
"""

import os
import uuid
import asyncio
import json
from contextlib import asynccontextmanager
import aiofiles
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from typing import Optional
import logging
from pathlib import Path
import time

# Import simulation service
from docker_service import DockerService

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize Docker service
docker_service = DockerService()

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Handle application lifespan events"""
    logger.info("🚀 Simulation Service starting up...")
    
    # Log Docker service status
    if docker_service.available:
        logger.info("✅ Docker service initialized and available")
    else:
        logger.warning("⚠️ Docker service unavailable - will use fallback mode")
    
    yield
    logger.info("🛑 Simulation Service shutting down...")

# Create FastAPI app with lifespan
app = FastAPI(title="Robot Simulation Service", version="1.0.0", lifespan=lifespan)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://localhost:5173", "http://localhost:8000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Create directories for video storage
VIDEOS_DIR = Path("videos")
VIDEOS_DIR.mkdir(exist_ok=True)

# Mount static files for video serving
app.mount("/videos", StaticFiles(directory="videos"), name="videos")

# API Models
class CodeExecutionRequest(BaseModel):
    code: str
    robot_type: str

class CodeExecutionResponse(BaseModel):
    success: bool
    video_url: Optional[str] = None
    error: Optional[str] = None
    execution_id: Optional[str] = None

# Health Check Endpoints
@app.get("/health")
async def health_check():
    """Get simulation service health"""
    return {
        "status": "operational" if docker_service.available else "limited",
        "timestamp": time.time(),
        "docker_available": docker_service.available,
        "service": "simulation"
    }

@app.get("/health/docker")
async def docker_status():
    """Get Docker service status"""
    return docker_service.get_status()

# Simulation Endpoints
@app.post("/run-code", response_model=CodeExecutionResponse)
async def execute_robot_code(request: CodeExecutionRequest):
    """Execute Python code in robot simulation environment"""
    
    try:
        # Validate inputs
        if not request.code or not request.code.strip():
            raise HTTPException(status_code=400, detail="Code cannot be empty")
        
        if request.robot_type not in ["turtlebot", "arm", "hand"]:
            raise HTTPException(status_code=400, detail="Invalid robot type")
        
        # Use Docker service for simulation
        if docker_service.available:
            result = await docker_service.run_simulation(request.code, request.robot_type)
            return CodeExecutionResponse(
                success=result["success"],
                video_url=result.get("video_url"),
                error=result.get("error"),
                execution_id=result["execution_id"]
            )
        else:
            # Docker service not available - return error with guidance
            return CodeExecutionResponse(
                success=False,
                error="Docker simulation service is currently unavailable. Please ensure Docker is running and the robot-simulation image is built.",
                execution_id=str(uuid.uuid4())
            )
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Simulation execution failed: {e}")
        return CodeExecutionResponse(
            success=False,
            error=str(e),
            execution_id=str(uuid.uuid4())
        )

@app.get("/robots")
def get_available_robots():
    """Get list of available robot types for simulation"""
    return {
        "robots": ["turtlebot", "arm", "hand"],
        "docker_available": docker_service.available,
        "details": {
            "turtlebot": {"description": "TurtleBot3 navigation robot", "available": docker_service.available},
            "arm": {"description": "Robot arm manipulation", "available": docker_service.available},
            "hand": {"description": "Dexterous hand control", "available": docker_service.available}
        }
    }

# Debug and utility endpoints
@app.get("/videos-debug")
async def videos_debug():
    """Debug endpoint to check video directory"""
    videos_path = Path("videos")
    if videos_path.exists():
        video_files = list(videos_path.glob("*.mp4"))
        return {
            "videos_directory_exists": True,
            "videos_path": str(videos_path.absolute()),
            "total_files": len(video_files),
            "files": [f.name for f in video_files[:10]]  # Show first 10 files
        }
    else:
        return {
            "videos_directory_exists": False,
            "videos_path": str(videos_path.absolute()),
            "total_files": 0,
            "files": []
        }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8001)