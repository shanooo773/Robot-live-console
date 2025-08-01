import os
import uuid
import asyncio
import docker
import aiofiles
from fastapi import FastAPI, HTTPException, UploadFile, File
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from typing import Optional
import logging
from pathlib import Path

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(title="Robot Simulation API", version="1.0.0")

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://localhost:5173"],  # Vite dev server
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Create directories for video storage
VIDEOS_DIR = Path("videos")
VIDEOS_DIR.mkdir(exist_ok=True)

# Mount static files for video serving
app.mount("/videos", StaticFiles(directory="videos"), name="videos")

# Docker client
docker_client = docker.from_env()

class CodeExecutionRequest(BaseModel):
    code: str
    robot_type: str

class CodeExecutionResponse(BaseModel):
    success: bool
    video_url: Optional[str] = None
    error: Optional[str] = None
    execution_id: str

# Robot configurations
ROBOT_CONFIGS = {
    "arm": {
        "urdf_package": "robot_arm_description",
        "urdf_file": "arm.urdf.xacro",
        "world_file": "arm_world.world"
    },
    "hand": {
        "urdf_package": "robot_hand_description", 
        "urdf_file": "hand.urdf.xacro",
        "world_file": "hand_world.world"
    },
    "turtlebot": {
        "urdf_package": "turtlebot3_description",
        "urdf_file": "turtlebot3_burger.urdf.xacro", 
        "world_file": "turtlebot_world.world"
    }
}

@app.get("/")
async def root():
    return {"message": "Robot Simulation API is running"}

@app.get("/robots")
async def get_available_robots():
    """Get list of available robot types"""
    return {"robots": list(ROBOT_CONFIGS.keys())}

@app.post("/run-code", response_model=CodeExecutionResponse)
async def run_code(request: CodeExecutionRequest):
    """Execute Python code in a robot simulation environment"""
    
    execution_id = str(uuid.uuid4())
    logger.info(f"Starting execution {execution_id} for robot type: {request.robot_type}")
    
    # Validate robot type
    if request.robot_type not in ROBOT_CONFIGS:
        raise HTTPException(
            status_code=400, 
            detail=f"Invalid robot type. Available: {list(ROBOT_CONFIGS.keys())}"
        )
    
    try:
        # Create execution directory
        exec_dir = Path(f"temp/{execution_id}")
        exec_dir.mkdir(parents=True, exist_ok=True)
        
        # Write user code to file
        code_file = exec_dir / "user_code.py"
        async with aiofiles.open(code_file, 'w') as f:
            await f.write(request.code)
        
        # Get robot config
        robot_config = request.robot_type
        
        # Run simulation in Docker container
        video_path = await run_simulation_in_docker(
            execution_id=execution_id,
            robot_type=robot_config,
            code_file=str(code_file)
        )
        
        if video_path and Path(video_path).exists():
            # Move video to videos directory
            final_video_path = VIDEOS_DIR / f"{execution_id}.mp4"
            Path(video_path).rename(final_video_path)
            
            video_url = f"/videos/{execution_id}.mp4"
            logger.info(f"Execution {execution_id} completed successfully")
            
            return CodeExecutionResponse(
                success=True,
                video_url=video_url,
                execution_id=execution_id
            )
        else:
            raise Exception("Video generation failed")
            
    except Exception as e:
        logger.error(f"Execution {execution_id} failed: {str(e)}")
        return CodeExecutionResponse(
            success=False,
            error=str(e),
            execution_id=execution_id
        )

async def run_simulation_in_docker(execution_id: str, robot_type: str, code_file: str) -> str:
    """Run the simulation inside a Docker container"""
    
    container_name = f"robot-sim-{execution_id}"
    video_output_path = f"/tmp/videos/{execution_id}.mp4"
    
    # Docker run command
    volumes = {
        str(Path(code_file).parent.absolute()): {'bind': '/workspace', 'mode': 'ro'},
        str(VIDEOS_DIR.absolute()): {'bind': '/tmp/videos', 'mode': 'rw'}
    }
    
    environment = {
        'ROBOT_TYPE': robot_type,
        'EXECUTION_ID': execution_id,
        'DISPLAY': ':99'  # Virtual display
    }
    
    try:
        # Run container
        container = docker_client.containers.run(
            image="robot-simulation:latest",
            name=container_name,
            volumes=volumes,
            environment=environment,
            working_dir="/workspace",
            command=[
                "bash", "-c", 
                f"python3 /opt/simulation/run_simulation.py --robot-type {robot_type} --code-file user_code.py --output-video {video_output_path}"
            ],
            detach=True,
            mem_limit="2g",
            cpu_quota=100000,  # Limit to 1 CPU
            network_mode="none",  # Disable networking for security
            remove=True
        )
        
        # Wait for container to complete (max 60 seconds)
        result = container.wait(timeout=60)
        logs = container.logs().decode('utf-8')
        
        if result['StatusCode'] == 0:
            logger.info(f"Container {container_name} completed successfully")
            return video_output_path
        else:
            logger.error(f"Container {container_name} failed with status {result['StatusCode']}")
            logger.error(f"Container logs: {logs}")
            raise Exception(f"Simulation failed: {logs}")
            
    except docker.errors.ContainerError as e:
        logger.error(f"Container error: {e}")
        raise Exception(f"Container execution failed: {e}")
    except Exception as e:
        logger.error(f"Docker execution error: {e}")
        raise Exception(f"Docker execution failed: {e}")

@app.on_event("startup")
async def startup_event():
    """Initialize the application"""
    logger.info("Robot Simulation API starting up...")
    
    # Create necessary directories
    Path("temp").mkdir(exist_ok=True)
    Path("videos").mkdir(exist_ok=True)
    
    # Check if Docker image exists
    try:
        docker_client.images.get("robot-simulation:latest")
        logger.info("Docker image 'robot-simulation:latest' found")
    except docker.errors.ImageNotFound:
        logger.warning("Docker image 'robot-simulation:latest' not found. Please build it using the setup script.")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)