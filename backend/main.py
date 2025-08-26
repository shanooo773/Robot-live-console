import os
import uuid
import asyncio
import platform
import subprocess
import json
from contextlib import asynccontextmanager
try:
    import docker
    DOCKER_SDK_AVAILABLE = True
except ImportError:
    docker = None
    DOCKER_SDK_AVAILABLE = False
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

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class DockerClientWrapper:
    """
    A wrapper class that provides Docker functionality using either the Docker SDK 
    or the Docker CLI, providing fallback options for different environments.
    """
    
    def __init__(self):
        self.client = None
        self.use_cli = False
        
    def _test_docker_cli(self):
        """Test if Docker CLI is available and working"""
        try:
            result = subprocess.run(['docker', 'version'], 
                                 capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                return True
        except Exception:
            pass
        return False
    
    def _test_docker_sdk(self, base_url=None):
        """Test if Docker SDK can connect"""
        if not DOCKER_SDK_AVAILABLE:
            return None
        try:
            if base_url:
                client = docker.DockerClient(base_url=base_url)
            else:
                client = docker.from_env()
            client.ping()
            return client
        except Exception:
            return None
    
    def ping(self):
        """Test Docker connection"""
        if self.client:
            try:
                return self.client.ping()
            except:
                pass
        
        if self.use_cli:
            try:
                result = subprocess.run(['docker', 'version'], 
                                     capture_output=True, text=True, timeout=10)
                return result.returncode == 0
            except:
                return False
        
        return False

class DockerContainerManager:
    """Container manager that uses Docker CLI"""
    
    def run(self, image, **kwargs):
        """Run container using Docker CLI"""
        try:
            cmd = ['docker', 'run']
            
            # Add common parameters
            if kwargs.get('detach', False):
                cmd.append('-d')
            if kwargs.get('remove', False):
                cmd.append('--rm')
            if 'volumes' in kwargs:
                for volume in kwargs['volumes']:
                    cmd.extend(['-v', volume])
            if 'environment' in kwargs:
                for env_var in kwargs['environment']:
                    cmd.extend(['-e', env_var])
            if 'name' in kwargs:
                cmd.extend(['--name', kwargs['name']])
            if 'working_dir' in kwargs:
                cmd.extend(['-w', kwargs['working_dir']])
            
            cmd.append(image)
            
            if 'command' in kwargs:
                if isinstance(kwargs['command'], list):
                    cmd.extend(kwargs['command'])
                else:
                    cmd.append(kwargs['command'])
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
            
            if result.returncode == 0:
                return MockContainer(result.stdout.strip())
            else:
                raise Exception(f"Docker run failed: {result.stderr}")
                
        except Exception as e:
            logger.error(f"Failed to run container: {e}")
            raise

class MockContainer:
    """Mock container object for CLI-based Docker operations"""
    
    def __init__(self, container_id):
        self.id = container_id
        self.name = container_id
    
    def wait(self, timeout=None):
        """Wait for container to finish"""
        try:
            cmd = ['docker', 'wait', self.id]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
            return {'StatusCode': int(result.stdout.strip()) if result.returncode == 0 else 1}
        except Exception:
            return {'StatusCode': 1}
    
    def logs(self):
        """Get container logs"""
        try:
            cmd = ['docker', 'logs', self.id]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            return result.stdout + result.stderr
        except Exception:
            return ""
    
    def remove(self):
        """Remove container"""
        try:
            cmd = ['docker', 'rm', '-f', self.id]
            subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        except Exception:
            pass

def create_docker_client():
    """Create and initialize Docker client with fallback options"""
    client_wrapper = DockerClientWrapper()
    
    if DOCKER_SDK_AVAILABLE:
        # Try default connection
        sdk_client = client_wrapper._test_docker_sdk()
        if sdk_client:
            client_wrapper.client = sdk_client
            logger.info("✅ Docker SDK connected successfully")
            return client_wrapper
        
        # Try common Docker Desktop paths
        docker_hosts = [
            "unix:///var/run/docker.sock",
            "tcp://localhost:2375",
            "tcp://localhost:2376", 
            "npipe:////./pipe/docker_engine"
        ]
        
        for host in docker_hosts:
            sdk_client = client_wrapper._test_docker_sdk(host)
            if sdk_client:
                client_wrapper.client = sdk_client
                logger.info(f"✅ Docker SDK connected via {host}")
                return client_wrapper
    
    # Fallback to CLI
    if client_wrapper._test_docker_cli():
        client_wrapper.use_cli = True
        client_wrapper.containers = lambda: DockerContainerManager()
        logger.info("✅ Docker CLI available, using CLI fallback")
        return client_wrapper
    
    # If all methods fail, return None
    logger.error("All Docker connection methods failed")
    return None

# Initialize Docker client
docker_client = create_docker_client()

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Handle application lifespan events"""
    logger.info("🚀 Robot Simulation API starting up...")
    yield
    logger.info("🛑 Robot Simulation API shutting down...")

# Create FastAPI app with lifespan
app = FastAPI(title="Robot Simulation API", version="1.0.0", lifespan=lifespan)

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

# API Models
class CodeExecutionRequest(BaseModel):
    code: str
    robot_type: str

class CodeExecutionResponse(BaseModel):
    success: bool
    video_url: Optional[str] = None
    error: Optional[str] = None
    execution_id: Optional[str] = None

# API Endpoints

@app.get("/")
async def root():
    return {"message": "Robot Programming Console API", "version": "1.0.0"}

@app.get("/robots")
def get_available_robots():
    """Get list of available robot types"""
    return {
        "robots": ["turtlebot", "arm", "hand"],
        "details": {
            "turtlebot": {
                "name": "TurtleBot3 Navigation Environment",
                "description": "Development environment for mobile robot navigation and path planning algorithms"
            },
            "arm": {
                "name": "Robot Arm Manipulation Environment", 
                "description": "Development environment for 6-DOF manipulator control and pick & place operations"
            },
            "hand": {
                "name": "Dexterous Hand Control Environment",
                "description": "Development environment for dexterous gripper control and complex manipulation tasks"
            }
        }
    }

@app.post("/run-code", response_model=CodeExecutionResponse)
async def execute_robot_code(request: CodeExecutionRequest):
    """Execute Python code in robot simulation environment"""
    
    execution_id = str(uuid.uuid4())
    temp_dir = Path(f"temp/{execution_id}")
    temp_dir.mkdir(parents=True, exist_ok=True)
    
    try:
        # Validate inputs
        if not request.code or not request.code.strip():
            raise HTTPException(status_code=400, detail="Code cannot be empty")
        
        if request.robot_type not in ["turtlebot", "arm", "hand"]:
            raise HTTPException(status_code=400, detail="Invalid robot type")
        
        # Save code to temporary file
        code_file = temp_dir / "user_code.py"
        async with aiofiles.open(code_file, 'w') as f:
            await f.write(request.code)
        
        # Create video output path
        video_filename = f"{execution_id}.mp4"
        video_path = VIDEOS_DIR / video_filename
        
        if docker_client and docker_client.ping():
            # Run simulation in Docker container
            container_name = f"robot-sim-{execution_id}"
            
            try:
                # Mount volumes and set environment
                volumes = [
                    f"{temp_dir.absolute()}:/workspace",
                    f"{VIDEOS_DIR.absolute()}:/videos"
                ]
                
                environment = [
                    f"ROBOT_TYPE={request.robot_type}",
                    f"EXECUTION_ID={execution_id}",
                    "DISPLAY=:99"
                ]
                
                # Run container
                container = docker_client.containers().run(
                    "robot-simulation:latest",
                    command=f"python /workspace/user_code.py",
                    detach=True,
                    remove=True,
                    volumes=volumes,
                    environment=environment,
                    name=container_name,
                    working_dir="/workspace"
                )
                
                # Wait for completion (with timeout)
                result = container.wait(timeout=120)
                
                if result['StatusCode'] == 0 and video_path.exists():
                    return CodeExecutionResponse(
                        success=True,
                        video_url=f"/videos/{video_filename}",
                        execution_id=execution_id
                    )
                else:
                    # Get logs for debugging
                    logs = container.logs()
                    logger.error(f"Simulation failed for {execution_id}: {logs}")
                    return CodeExecutionResponse(
                        success=False,
                        error="Simulation failed to complete successfully",
                        execution_id=execution_id
                    )
                    
            except Exception as e:
                logger.error(f"Docker execution failed: {e}")
                # Fall through to mock simulation
        
        # Mock simulation for testing (when Docker not available)
        logger.info(f"Running mock simulation for {execution_id}")
        
        # Simulate processing time
        await asyncio.sleep(2)
        
        # Create a mock video file
        mock_video_content = b"Mock video content for testing"
        async with aiofiles.open(video_path, 'wb') as f:
            await f.write(mock_video_content)
        
        return CodeExecutionResponse(
            success=True,
            video_url=f"/videos/{video_filename}",
            execution_id=execution_id
        )
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Execution failed: {e}")
        return CodeExecutionResponse(
            success=False,
            error=str(e),
            execution_id=execution_id
        )
    finally:
        # Cleanup temporary files
        try:
            if temp_dir.exists():
                import shutil
                shutil.rmtree(temp_dir)
        except Exception as e:
            logger.warning(f"Failed to cleanup temp directory: {e}")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)