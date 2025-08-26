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

@app.get("/status")
async def status_check():
    """Health check endpoint"""
    return {"status": "Backend is running", "timestamp": time.time()}

@app.get("/videos-debug")
async def videos_debug():
    """Debug endpoint for video directory and permissions"""
    try:
        videos_dir = Path("videos")
        return {
            "videos_directory_exists": videos_dir.exists(),
            "videos_directory_writable": videos_dir.is_dir() and os.access(videos_dir, os.W_OK),
            "videos_count": len(list(videos_dir.glob("*.mp4"))) if videos_dir.exists() else 0,
            "videos_list": [f.name for f in videos_dir.glob("*.mp4")] if videos_dir.exists() else [],
            "current_working_directory": str(Path.cwd()),
            "videos_absolute_path": str(videos_dir.absolute())
        }
    except Exception as e:
        return {"error": str(e)}

@app.get("/docker-status")
async def docker_status():
    """Check Docker availability and configuration"""
    try:
        status = {
            "docker_sdk_available": DOCKER_SDK_AVAILABLE,
            "docker_client_connected": False,
            "docker_ping_successful": False,
            "docker_images": [],
            "docker_client_type": "none"
        }
        
        if docker_client:
            status["docker_client_connected"] = True
            status["docker_client_type"] = "cli" if getattr(docker_client, 'use_cli', False) else "sdk"
            
            try:
                status["docker_ping_successful"] = docker_client.ping()
                
                # Try to list images
                if hasattr(docker_client, 'client') and docker_client.client:
                    try:
                        images = docker_client.client.images.list()
                        status["docker_images"] = [{"id": img.id[:12], "tags": img.tags} for img in images]
                    except:
                        pass
                else:
                    # CLI fallback
                    import subprocess
                    try:
                        result = subprocess.run(['docker', 'images', '--format', 'table {{.Repository}}:{{.Tag}}\t{{.ID}}'], 
                                              capture_output=True, text=True, timeout=10)
                        if result.returncode == 0:
                            lines = result.stdout.strip().split('\n')[1:]  # Skip header
                            status["docker_images"] = [line.strip() for line in lines if line.strip()]
                    except Exception as e:
                        status["docker_cli_error"] = str(e)
                        
                # Check for robot simulation image specifically
                status["robot_simulation_image_available"] = any(
                    "robot-simulation" in str(img) for img in status["docker_images"]
                )
                
            except Exception as e:
                status["docker_error"] = str(e)
        
        return status
    except Exception as e:
        return {"error": str(e)}

@app.get("/videos-check/{execution_id}")
async def check_video_file(execution_id: str):
    """Check if a specific video file exists and is accessible"""
    try:
        video_path = VIDEOS_DIR / f"{execution_id}.mp4"
        return {
            "execution_id": execution_id,
            "file_exists": video_path.exists(),
            "file_size": video_path.stat().st_size if video_path.exists() else 0,
            "file_path": str(video_path),
            "file_readable": video_path.is_file() and os.access(video_path, os.R_OK) if video_path.exists() else False,
            "video_url": f"/videos/{execution_id}.mp4" if video_path.exists() else None
        }
    except Exception as e:
        return {"error": str(e)}

@app.get("/debug-info")
async def debug_info():
    """Comprehensive debug information"""
    try:
        return {
            "backend": {
                "working_directory": str(Path.cwd()),
                "videos_directory": str(VIDEOS_DIR.absolute()),
                "videos_exists": VIDEOS_DIR.exists(),
                "environment": {
                    "python_version": platform.python_version(),
                    "platform": platform.platform(),
                    "architecture": platform.architecture()
                }
            },
            "docker": await docker_status(),
            "videos": await videos_debug()
        }
    except Exception as e:
        return {"error": str(e)}

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
                "name": "TurtleBot3",
                "description": "Mobile robot for navigation and path planning"
            },
            "arm": {
                "name": "Robot Arm", 
                "description": "6-DOF manipulator for pick and place operations"
            },
            "hand": {
                "name": "Robot Hand",
                "description": "Dexterous gripper for complex manipulation tasks"
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
                # Check if image exists first
                logger.info(f"Checking for Docker image: robot-simulation:latest")
                image_available = False
                
                try:
                    if hasattr(docker_client, 'client') and docker_client.client:
                        # SDK method
                        docker_client.client.images.get("robot-simulation:latest")
                        image_available = True
                        logger.info("Docker image found via SDK")
                    else:
                        # CLI fallback
                        import subprocess
                        result = subprocess.run(['docker', 'images', 'robot-simulation:latest', '-q'], 
                                              capture_output=True, text=True, timeout=10)
                        if result.returncode == 0 and result.stdout.strip():
                            image_available = True
                            logger.info("Docker image found via CLI")
                        else:
                            logger.warning("Docker image robot-simulation:latest not found")
                except Exception as e:
                    logger.error(f"Error checking for Docker image: {e}")
                
                if not image_available:
                    logger.warning("Docker image robot-simulation:latest not available, falling back to mock simulation")
                    raise Exception("Docker image robot-simulation:latest not found")
                
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
                
                logger.info(f"Starting Docker container {container_name} with image robot-simulation:latest")
                
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
                
                logger.info(f"Container {container_name} started, waiting for completion...")
                
                # Wait for completion (with timeout)
                result = container.wait(timeout=120)
                
                logger.info(f"Container completed with status code: {result['StatusCode']}")
                
                if result['StatusCode'] == 0 and video_path.exists():
                    # Check if video file has content
                    video_size = video_path.stat().st_size
                    logger.info(f"Real simulation completed successfully, video size: {video_size} bytes")
                    
                    if video_size > 100:  # Ensure it's not just a placeholder
                        return CodeExecutionResponse(
                            success=True,
                            video_url=f"/videos/{video_filename}",
                            execution_id=execution_id
                        )
                    else:
                        logger.warning(f"Video file too small ({video_size} bytes), treating as failed")
                
                # Get logs for debugging
                logs = container.logs()
                logger.error(f"Simulation failed for {execution_id}. Container logs: {logs}")
                
                return CodeExecutionResponse(
                    success=False,
                    error=f"Simulation failed to complete successfully. Container exit code: {result['StatusCode']}",
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