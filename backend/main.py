import os
import uuid
import asyncio
import platform
import urllib.parse
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
from fastapi import FastAPI, HTTPException, UploadFile, File
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
    
    def containers(self):
        """Get container manager"""
        if self.client:
            return self.client.containers
        else:
            return DockerContainerManager()

class DockerContainerManager:
    """Container manager that uses Docker CLI"""
    
    def list(self):
        """List containers using Docker CLI"""
        try:
            result = subprocess.run(['docker', 'ps', '--format', 'json'], 
                                 capture_output=True, text=True, timeout=30)
            if result.returncode == 0:
                containers = []
                for line in result.stdout.strip().split('\n'):
                    if line:
                        container_info = json.loads(line)
                        containers.append(DockerContainer(container_info))
                return containers
        except Exception as e:
            logger.error(f"Failed to list containers via CLI: {e}")
        return []
    
    def run(self, image, **kwargs):
        """Run container using Docker CLI"""
        cmd = ['docker', 'run']
        
        # Handle common kwargs
        if kwargs.get('name'):
            cmd.extend(['--name', kwargs['name']])
        if kwargs.get('detach'):
            cmd.append('-d')
        if kwargs.get('remove'):
            cmd.append('--rm')
        if kwargs.get('volumes'):
            for host_path, container_config in kwargs['volumes'].items():
                bind_path = container_config['bind']
                mode = container_config.get('mode', 'rw')
                cmd.extend(['-v', f"{host_path}:{bind_path}:{mode}"])
        if kwargs.get('environment'):
            for key, value in kwargs['environment'].items():
                cmd.extend(['-e', f"{key}={value}"])
        if kwargs.get('working_dir'):
            cmd.extend(['-w', kwargs['working_dir']])
        if kwargs.get('mem_limit'):
            cmd.extend(['--memory', kwargs['mem_limit']])
        if kwargs.get('cpu_quota'):
            cpu_limit = kwargs['cpu_quota'] / 100000  # Convert to decimal
            cmd.extend(['--cpus', str(cpu_limit)])
        if kwargs.get('network_mode') == 'none':
            cmd.extend(['--network', 'none'])
        
        cmd.append(image)
        
        if kwargs.get('command'):
            if isinstance(kwargs['command'], list):
                cmd.extend(kwargs['command'])
            else:
                cmd.append(kwargs['command'])
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
            if result.returncode == 0:
                container_id = result.stdout.strip()
                return DockerContainer({'ID': container_id})
            else:
                raise Exception(f"Docker run failed: {result.stderr}")
        except Exception as e:
            logger.error(f"Failed to run container via CLI: {e}")
            raise

class DockerContainer:
    """Container representation for CLI-based Docker operations"""
    
    def __init__(self, container_info):
        self.info = container_info
        self.name = container_info.get('Names', container_info.get('ID', ''))
        self.id = container_info.get('ID', '')
    
    def wait(self, timeout=None):
        """Wait for container to complete"""
        try:
            cmd = ['docker', 'wait', self.id]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
            if result.returncode == 0:
                exit_code = int(result.stdout.strip())
                return {'StatusCode': exit_code}
            else:
                return {'StatusCode': -1}
        except subprocess.TimeoutExpired:
            return {'StatusCode': -1}
        except Exception:
            return {'StatusCode': -1}
    
    def logs(self):
        """Get container logs"""
        try:
            cmd = ['docker', 'logs', self.id]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            return result.stdout.encode('utf-8')
        except Exception:
            return b''

def create_docker_client():
    """
    Create a Docker client that works across different platforms (Linux, Windows, WSL2).
    This function handles the various ways Docker can be accessed depending on the environment.
    """
    wrapper = DockerClientWrapper()
    
    # Get the current operating system
    system = platform.system().lower()
    
    # First, try to detect if we're in WSL by checking for WSL environment indicators
    is_wsl = False
    if system == "linux":
        try:
            with open("/proc/version", "r") as f:
                if "microsoft" in f.read().lower():
                    is_wsl = True
        except:
            pass
    
    # Try different connection methods in order of preference
    connection_methods = []
    
    if system == "windows":
        # Windows - use named pipe
        connection_methods.append(("Windows named pipe", 'npipe:////./pipe/docker_engine'))
    elif is_wsl:
        # WSL environment - try TCP connection, then Unix socket
        connection_methods.extend([
            ("WSL TCP connection", 'tcp://localhost:2375'),
            ("WSL Unix socket", 'unix:///var/run/docker.sock')
        ])
    else:
        # Standard Linux - use Unix socket
        connection_methods.append(("Linux Unix socket", 'unix:///var/run/docker.sock'))
    
    # Try each SDK connection method
    for method_name, base_url in connection_methods:
        client = wrapper._test_docker_sdk(base_url)
        if client:
            logger.info(f"Successfully connected to Docker using {method_name}")
            wrapper.client = client
            return wrapper
    
    # Try docker.from_env() as fallback
    client = wrapper._test_docker_sdk()
    if client:
        logger.info("Successfully connected to Docker using docker.from_env()")
        wrapper.client = client
        return wrapper
    
    # If SDK methods fail, try Docker CLI
    if wrapper._test_docker_cli():
        logger.info("Docker SDK failed, but Docker CLI is available - using CLI fallback")
        wrapper.use_cli = True
        return wrapper
    
    # If all methods fail, return None
    logger.error("All Docker connection methods failed")
    return None

# Docker client
try:
    docker_client = create_docker_client()
    if docker_client is None:
        raise Exception("Could not establish Docker connection")
except Exception as e:
    logger.error(f"Docker initialization failed: {e}")
    docker_client = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Handle application lifespan events"""
    # Startup
    logger.info("Robot Simulation API starting up...")
    
    # Create necessary directories
    Path("temp").mkdir(exist_ok=True)
    Path("videos").mkdir(exist_ok=True)
    
    # Check if Docker image exists
    try:
        if docker_client and hasattr(docker_client, 'client') and docker_client.client:
            docker_client.client.images.get("robot-simulation:latest")
            logger.info("Docker image 'robot-simulation:latest' found")
        elif docker_client and docker_client.use_cli:
            # Use CLI to check for image
            result = subprocess.run(['docker', 'images', '-q', 'robot-simulation:latest'], 
                                  capture_output=True, text=True, timeout=10)
            if result.returncode == 0 and result.stdout.strip():
                logger.info("Docker image 'robot-simulation:latest' found via CLI")
            else:
                logger.warning("Docker image 'robot-simulation:latest' not found via CLI")
        else:
            logger.warning("Docker not available - cannot check for image")
    except Exception as e:
        logger.warning(f"Could not check for Docker image: {e}")
        logger.warning("Docker image 'robot-simulation:latest' not found. Please build it using the setup script.")
    
    yield
    
    # Shutdown (if needed)
    logger.info("Robot Simulation API shutting down...")

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

@app.get("/status")
def get_status():
    return {"status": "Backend is running"}

@app.get("/docker-status")
def get_docker_status():
    if docker_client:
        try:
            containers = docker_client.containers().list()
            container_names = []
            for container in containers:
                if hasattr(container, 'name'):
                    container_names.append(container.name)
                elif hasattr(container, 'info'):
                    # CLI-based container
                    container_names.append(container.info.get('Names', container.info.get('ID', 'unknown')))
                else:
                    # SDK-based container
                    container_names.append(getattr(container, 'name', 'unknown'))
            
            return {"status": "Docker is running", "containers": container_names}
        except Exception as e:
            return {"status": "Docker error", "error": str(e)}
    else:
        return {"status": "Docker not available"}

@app.get("/videos-debug")
def get_videos_debug_info():
    """Debug endpoint to check videos directory status and contents"""
    videos_path = Path("videos")
    
    debug_info = {
        "videos_directory_exists": videos_path.exists(),
        "videos_directory_path": str(videos_path.absolute()),
        "videos_directory_permissions": None,
        "video_files": [],
        "total_files": 0
    }
    
    try:
        if videos_path.exists():
            # Check permissions
            debug_info["videos_directory_permissions"] = {
                "readable": os.access(videos_path, os.R_OK),
                "writable": os.access(videos_path, os.W_OK),
                "executable": os.access(videos_path, os.X_OK)
            }
            
            # List video files
            video_files = []
            for file_path in videos_path.glob("*.mp4"):
                file_stat = file_path.stat()
                video_files.append({
                    "name": file_path.name,
                    "size_bytes": file_stat.st_size,
                    "created": file_stat.st_ctime,
                    "modified": file_stat.st_mtime,
                    "accessible": os.access(file_path, os.R_OK)
                })
            
            debug_info["video_files"] = video_files
            debug_info["total_files"] = len(video_files)
        
    except Exception as e:
        debug_info["error"] = str(e)
    
    return debug_info

@app.get("/videos-check/{execution_id}")
def check_video_exists(execution_id: str):
    """Check if a specific video file exists and is accessible"""
    video_path = VIDEOS_DIR / f"{execution_id}.mp4"
    
    result = {
        "execution_id": execution_id,
        "expected_path": str(video_path.absolute()),
        "exists": video_path.exists(),
        "accessible": False,
        "size_bytes": 0,
        "url": f"/videos/{execution_id}.mp4"
    }
    
    if video_path.exists():
        try:
            result["accessible"] = os.access(video_path, os.R_OK)
            result["size_bytes"] = video_path.stat().st_size
        except Exception as e:
            result["error"] = str(e)
    
    return result

@app.get("/execution-logs/{execution_id}")
def get_execution_logs(execution_id: str):
    """Get logs for a specific execution (for debugging)"""
    try:
        # Check if there's a log file for this execution
        log_file = Path(f"temp/{execution_id}/simulation.log")
        if log_file.exists():
            with open(log_file, 'r') as f:
                logs = f.read()
            return {
                "execution_id": execution_id,
                "logs": logs,
                "log_file": str(log_file),
                "available": True
            }
        else:
            return {
                "execution_id": execution_id,
                "logs": "",
                "log_file": str(log_file),
                "available": False,
                "message": "No logs available for this execution"
            }
    except Exception as e:
        return {
            "execution_id": execution_id,
            "logs": "",
            "available": False,
            "error": str(e)
        }
class CodeExecutionRequest(BaseModel):
    code: str
    robot_type: str

class SimulationRequest(BaseModel):
    urdf_path: str
    world_path: str
    duration: Optional[int] = 10

class CodeExecutionResponse(BaseModel):
    success: bool
    video_url: Optional[str] = None
    error: Optional[str] = None
    execution_id: str
    logs_url: Optional[str] = None
    error_details: Optional[dict] = None

class SimulationResponse(BaseModel):
    success: bool
    video_url: Optional[str] = None
    error: Optional[str] = None
    execution_id: str
    logs_url: Optional[str] = None
    error_details: Optional[dict] = None

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

@app.get("/health")
async def health_check():
    """Health check endpoint for system monitoring"""
    return {
        "status": "healthy",
        "timestamp": time.time(),
        "services": {
            "backend": "running",
            "docker": "available" if docker_client else "unavailable",
            "videos_directory": VIDEOS_DIR.exists()
        }
    }

@app.get("/")
async def root():
    return {"message": "Robot Simulation API is running", "version": "1.0.0"}

@app.post("/simulate", response_model=SimulationResponse)
async def run_simulation(request: SimulationRequest):
    """Run a simulation with provided URDF and world files"""
    
    execution_id = str(uuid.uuid4())
    logger.info(f"Starting simulation {execution_id} with URDF: {request.urdf_path}, World: {request.world_path}")
    
    try:
        # Validate file paths
        urdf_path = Path(request.urdf_path)
        world_path = Path(request.world_path)
        
        if not urdf_path.exists():
            raise HTTPException(
                status_code=400,
                detail=f"URDF file not found: {request.urdf_path}"
            )
        
        if not world_path.exists():
            raise HTTPException(
                status_code=400,
                detail=f"World file not found: {request.world_path}"
            )
        
        # Determine robot type from URDF filename (basic heuristic)
        robot_type = "arm"  # default
        if "hand" in urdf_path.name.lower():
            robot_type = "hand"
        elif "turtle" in urdf_path.name.lower():
            robot_type = "turtlebot"
        elif "arm" in urdf_path.name.lower():
            robot_type = "arm"
        
        # Create execution directory
        exec_dir = Path(f"temp/{execution_id}")
        exec_dir.mkdir(parents=True, exist_ok=True)
        
        # Create a basic Python script for simulation (user can customize this)
        code_content = f"""#!/usr/bin/env python3
# Auto-generated simulation script for {robot_type}
import rospy
import time

def run_simulation():
    rospy.init_node('simulation_runner', anonymous=True)
    print("Simulation started for {robot_type}")
    
    # Basic simulation logic - can be customized
    time.sleep({request.duration - 2})  # Run for most of the duration
    
    print("Simulation completed")

if __name__ == '__main__':
    try:
        run_simulation()
    except rospy.ROSInterruptException:
        pass
"""
        
        code_file = exec_dir / "user_code.py"
        async with aiofiles.open(code_file, 'w') as f:
            await f.write(code_content)
        
        # Run simulation in Docker container
        video_path = await run_real_simulation_in_docker(
            execution_id=execution_id,
            urdf_path=str(urdf_path),
            world_path=str(world_path),
            code_file=str(code_file),
            duration=request.duration
        )
        
        # Check if video exists in backend/videos
        final_video_path = VIDEOS_DIR / f"{execution_id}.mp4"
        if final_video_path.exists() and final_video_path.stat().st_size > 0:
            video_url = f"/videos/{execution_id}.mp4"
            logger.info(f"Simulation {execution_id} completed successfully. Video size: {final_video_path.stat().st_size} bytes")
            return SimulationResponse(
                success=True,
                video_url=video_url,
                execution_id=execution_id,
                logs_url=f"/execution-logs/{execution_id}"
            )
        else:
            raise Exception(f"Video generation failed - no video file found at {final_video_path}")
            
    except Exception as e:
        logger.error(f"Simulation {execution_id} failed: {str(e)}")
        return SimulationResponse(
            success=False,
            error=str(e),
            execution_id=execution_id,
            logs_url=f"/execution-logs/{execution_id}"
        )

@app.post("/upload-files")
async def upload_simulation_files(
    urdf_file: UploadFile = File(...),
    world_file: UploadFile = File(...)
):
    """Upload URDF and world files for simulation with enhanced validation"""
    
    upload_id = str(uuid.uuid4())
    upload_dir = Path(f"temp/uploads/{upload_id}")
    upload_dir.mkdir(parents=True, exist_ok=True)
    
    try:
        # Enhanced file validation
        def validate_file_size(file: UploadFile, max_size_mb: int = 10):
            """Validate file size"""
            if hasattr(file, 'size') and file.size:
                if file.size > max_size_mb * 1024 * 1024:
                    raise HTTPException(
                        status_code=400,
                        detail=f"File {file.filename} is too large (max {max_size_mb}MB)"
                    )
        
        def validate_file_extension(filename: str, allowed_extensions: list):
            """Validate file extension"""
            if not any(filename.lower().endswith(ext) for ext in allowed_extensions):
                raise HTTPException(
                    status_code=400,
                    detail=f"File {filename} must have one of these extensions: {', '.join(allowed_extensions)}"
                )
        
        def validate_file_content(file_path: Path, required_content: str, file_type: str):
            """Validate file contains required content"""
            try:
                content = file_path.read_text(encoding='utf-8', errors='ignore')
                if required_content not in content:
                    raise HTTPException(
                        status_code=400,
                        detail=f"{file_type} file does not contain valid {required_content} definition"
                    )
            except Exception as e:
                raise HTTPException(
                    status_code=400,
                    detail=f"Failed to validate {file_type} file content: {str(e)}"
                )
        
        # Validate URDF file
        validate_file_extension(urdf_file.filename, ['.urdf', '.xacro'])
        validate_file_size(urdf_file, 5)  # 5MB max for URDF
        
        # Validate world file
        validate_file_extension(world_file.filename, ['.world'])
        validate_file_size(world_file, 10)  # 10MB max for world
        
        # Save URDF file
        urdf_path = upload_dir / urdf_file.filename
        async with aiofiles.open(urdf_path, 'wb') as f:
            content = await urdf_file.read()
            if len(content) == 0:
                raise HTTPException(
                    status_code=400,
                    detail="URDF file is empty"
                )
            await f.write(content)
        
        # Save world file
        world_path = upload_dir / world_file.filename
        async with aiofiles.open(world_path, 'wb') as f:
            content = await world_file.read()
            if len(content) == 0:
                raise HTTPException(
                    status_code=400,
                    detail="World file is empty"
                )
            await f.write(content)
        
        # Validate file contents
        validate_file_content(urdf_path, "<robot", "URDF")
        validate_file_content(world_path, "<world", "World")
        
        logger.info(f"Files uploaded and validated for {upload_id}: URDF={urdf_path}, World={world_path}")
        
        return {
            "success": True,
            "upload_id": upload_id,
            "urdf_path": str(urdf_path),
            "world_path": str(world_path),
            "urdf_filename": urdf_file.filename,
            "world_filename": world_file.filename,
            "validation": {
                "urdf_size_bytes": urdf_path.stat().st_size,
                "world_size_bytes": world_path.stat().st_size,
                "urdf_valid": True,
                "world_valid": True
            }
        }
        
    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logger.error(f"File upload failed: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"File upload failed: {str(e)}"
        )

@app.get("/robots")
async def get_available_robots():
    """Get list of available robot types"""
    return {"robots": list(ROBOT_CONFIGS.keys())}
# ...existing code...
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
        
        # Instead of moving, check if video exists in backend/videos
        final_video_path = VIDEOS_DIR / f"{execution_id}.mp4"
        if final_video_path.exists() and final_video_path.stat().st_size > 0:
            video_url = f"/videos/{execution_id}.mp4"
            logger.info(f"Execution {execution_id} completed successfully. Video size: {final_video_path.stat().st_size} bytes")
            return CodeExecutionResponse(
                success=True,
                video_url=video_url,
                execution_id=execution_id,
                logs_url=f"/execution-logs/{execution_id}"
            )
        else:
            raise Exception(f"Video generation failed - no video file found at {final_video_path}")
            
    except Exception as e:
        logger.error(f"Execution {execution_id} failed: {str(e)}")
        return CodeExecutionResponse(
            success=False,
            error=str(e),
            execution_id=execution_id,
            logs_url=f"/execution-logs/{execution_id}"
        )
app.mount("/videos", StaticFiles(directory="videos"), name="videos")
# ...existing code...
VIDEOS_DIR = Path(__file__).parent / "videos"
VIDEOS_DIR.mkdir(exist_ok=True)

async def run_real_simulation_in_docker(execution_id: str, urdf_path: str, world_path: str, code_file: str, duration: int = 10) -> str:
    """Run the real ROS/Gazebo simulation inside a Docker container with enhanced error handling"""
    
    container_name = f"robot-real-sim-{execution_id}"
    
    # Create videos directory if it doesn't exist
    VIDEOS_DIR.mkdir(exist_ok=True)
    
    # Define paths - Windows-safe volume mounting
    host_videos_dir = VIDEOS_DIR.absolute()
    container_videos_dir = "/output"
    video_output_path = f"{container_videos_dir}/video.mp4"
    host_video_path = host_videos_dir / f"{execution_id}.mp4"
    
    # Create simulation data directory and copy files
    simulation_data_dir = Path(f"temp/{execution_id}/simulation_data")
    simulation_data_dir.mkdir(parents=True, exist_ok=True)
    
    # Copy URDF and world files to simulation data directory
    urdf_basename = Path(urdf_path).name
    world_basename = Path(world_path).name
    
    import shutil
    container_urdf_path = simulation_data_dir / urdf_basename
    container_world_path = simulation_data_dir / world_basename
    
    shutil.copy2(urdf_path, container_urdf_path)
    shutil.copy2(world_path, container_world_path)
    
    logger.info(f"Starting Docker container {container_name}")
    logger.info(f"Host videos directory: {host_videos_dir}")
    logger.info(f"Container videos directory: {container_videos_dir}")
    logger.info(f"URDF: {urdf_path} -> /simulation_data/{urdf_basename}")
    logger.info(f"World: {world_path} -> /simulation_data/{world_basename}")
    
    # Check if Docker client is available and if robot simulation image exists
    use_mock = False
    error_details = ""
    
    try:
        if docker_client and hasattr(docker_client, 'client') and docker_client.client:
            docker_client.client.images.get("robot-simulation:latest")
            logger.info("Using Docker simulation")
        elif docker_client and docker_client.use_cli:
            # Use CLI to check for image
            result = subprocess.run(['docker', 'images', '-q', 'robot-simulation:latest'], 
                                  capture_output=True, text=True, timeout=10)
            if result.returncode != 0 or not result.stdout.strip():
                raise Exception("Docker image not found")
            logger.info("Using Docker CLI simulation")
        else:
            raise Exception("Docker not available")
    except Exception as e:
        error_details = str(e)
        logger.warning(f"Docker simulation not available ({e}), using mock simulation for testing")
        use_mock = True
    
    if use_mock:
        # Fallback to mock simulation with detailed error message
        mock_video_content = create_mock_video_with_reason(
            execution_id, 
            f"Real simulation unavailable: {error_details}. Docker image 'robot-simulation:latest' not found or Docker not accessible."
        )
        with open(host_video_path, 'wb') as f:
            f.write(mock_video_content)
        return str(host_video_path)
    
    # Docker simulation setup with Windows-safe paths
    volumes = {}
    
    # Handle Windows path conversion if needed
    if platform.system().lower() == "windows":
        # Convert Windows paths to proper Docker format
        def windows_to_docker_path(path):
            """Convert Windows path to Docker-compatible format"""
            path_str = str(path).replace('\\', '/')
            if path_str[1:3] == ':/':  # Drive letter format C:/
                path_str = '/' + path_str[0].lower() + path_str[2:]
            return path_str
        
        workspace_path = windows_to_docker_path(Path(code_file).parent.absolute())
        sim_data_path = windows_to_docker_path(simulation_data_dir.absolute())
        videos_path = windows_to_docker_path(host_videos_dir)
    else:
        workspace_path = str(Path(code_file).parent.absolute())
        sim_data_path = str(simulation_data_dir.absolute())
        videos_path = str(host_videos_dir)
    
    volumes = {
        workspace_path: {'bind': '/workspace', 'mode': 'ro'},
        sim_data_path: {'bind': '/simulation_data', 'mode': 'ro'},
        videos_path: {'bind': container_videos_dir, 'mode': 'rw'}
    }
    
    environment = {
        'EXECUTION_ID': execution_id,
        'DISPLAY': ':99'  # Virtual display
    }
    
    container_logs = ""
    container_exit_code = -1
    
    try:
        # Run container
        container = docker_client.containers().run(
            image="robot-simulation:latest",
            name=container_name,
            volumes=volumes,
            environment=environment,
            working_dir="/workspace",
            command=[
                "bash", "/opt/simulation/record_simulation.sh",
                "--robot-type", robot_type,
                "--urdf-file", f"/simulation_data/{urdf_basename}",
                "--world-file", f"/simulation_data/{world_basename}",
                "--output-video", video_output_path,
                "--duration", str(duration)
            ],
            detach=True,
            mem_limit="4g",  # Increased memory for Gazebo
            cpu_quota=200000,  # Allow 2 CPUs for Gazebo
            network_mode="none",  # Disable networking for security
            remove=True
        )
        
        logger.info(f"Container {container_name} started, waiting for completion...")
        
        # Wait for container to complete (max duration + 60 seconds for setup/cleanup)
        result = container.wait(timeout=duration + 60)
        container_logs = container.logs().decode('utf-8', errors='replace')
        container_exit_code = result['StatusCode']
        
        logger.info(f"Container {container_name} completed with status {container_exit_code}")
        
        # Save container logs to file for debugging
        log_file = Path(f"temp/{execution_id}/simulation.log")
        log_file.parent.mkdir(parents=True, exist_ok=True)
        try:
            with open(log_file, 'w') as f:
                f.write(f"Container: {container_name}\n")
                f.write(f"Exit Code: {container_exit_code}\n")
                f.write(f"Robot Type: {robot_type}\n")
                f.write(f"URDF: {urdf_basename}\n")
                f.write(f"World: {world_basename}\n")
                f.write(f"Output Video: {video_output_path}\n")
                f.write(f"Duration: {duration}\n")
                f.write(f"=== CONTAINER LOGS ===\n")
                f.write(container_logs)
        except Exception as e:
            logger.warning(f"Failed to save logs to file: {e}")
        
        # Parse container logs for specific error information
        error_info = parse_simulation_logs(container_logs)
        
        if container_exit_code == 0:
            # Check if video was created on the host side
            if host_video_path.exists() and host_video_path.stat().st_size > 1000:
                logger.info(f"Video successfully created at {host_video_path} (size: {host_video_path.stat().st_size} bytes)")
                return str(host_video_path)
            else:
                error_msg = "Container completed successfully but video not found or too small"
                if error_info.get('detected_issues'):
                    error_msg += f". Detected issues: {', '.join(error_info['detected_issues'])}"
                logger.error(f"{error_msg}. Container logs: {container_logs}")
                raise Exception(error_msg)
        else:
            # Container failed - provide detailed error information
            error_msg = f"Simulation failed with exit code {container_exit_code}"
            if error_info.get('primary_error'):
                error_msg = error_info['primary_error']
            elif error_info.get('detected_issues'):
                error_msg += f". Issues: {', '.join(error_info['detected_issues'])}"
            
            logger.error(f"Container {container_name} failed: {error_msg}")
            logger.error(f"Full container logs: {container_logs}")
            raise Exception(error_msg)
            
    except Exception as e:
        # Create a mock video with error details for user feedback
        error_message = str(e)
        if container_logs:
            # Include relevant log snippets
            log_snippets = extract_relevant_log_snippets(container_logs)
            if log_snippets:
                error_message += f"\n\nLog details:\n{log_snippets}"
        
        logger.error(f"Docker execution error: {error_message}")
        
        # Create mock video with error details
        mock_video_content = create_mock_video_with_reason(execution_id, error_message)
        with open(host_video_path, 'wb') as f:
            f.write(mock_video_content)
        
        raise Exception(error_message)


def parse_simulation_logs(logs: str) -> dict:
    """Parse simulation logs to extract error information and detect root causes"""
    info = {
        'primary_error': None,
        'detected_issues': [],
        'error_code': None
    }
    
    lines = logs.split('\n')
    
    for line in lines:
        line = line.strip()
        
        # Check for simulation error codes
        if 'SIMULATION_ERROR_CODE:' in line:
            try:
                info['error_code'] = int(line.split('SIMULATION_ERROR_CODE:')[1].strip())
            except:
                pass
        
        if 'SIMULATION_ERROR_MESSAGE:' in line:
            info['primary_error'] = line.split('SIMULATION_ERROR_MESSAGE:')[1].strip()
        
        # Detect specific issues
        if 'not found' in line.lower() and ('urdf' in line.lower() or 'world' in line.lower()):
            info['detected_issues'].append('Missing robot/world files')
        
        if 'failed to start gazebo' in line.lower():
            info['detected_issues'].append('Gazebo startup failure')
        
        if 'failed to start xvfb' in line.lower():
            info['detected_issues'].append('Virtual display failure')
        
        if 'ffmpeg' in line.lower() and 'failed' in line.lower():
            info['detected_issues'].append('Video recording failure')
        
        if 'ros master' in line.lower() and 'failed' in line.lower():
            info['detected_issues'].append('ROS master failure')
        
        if 'permission denied' in line.lower():
            info['detected_issues'].append('File permission issues')
        
        if 'out of memory' in line.lower() or 'oom' in line.lower():
            info['detected_issues'].append('Insufficient memory')
        
        # Detect argument parsing issues
        if '--robot-type and --output-video are required' in line:
            info['detected_issues'].append('Missing required arguments: --robot-type or --output-video')
            if not info['primary_error']:
                info['primary_error'] = 'Missing required arguments for simulation script'
        
        if 'unknown option:' in line.lower():
            info['detected_issues'].append('Invalid command line arguments')
            if not info['primary_error']:
                info['primary_error'] = 'Invalid arguments passed to simulation script'
        
        if 'usage:' in line.lower() and ('options' in line.lower() or 'help' in line.lower()):
            info['detected_issues'].append('Script showed help instead of running simulation')
            if not info['primary_error']:
                info['primary_error'] = 'Simulation script arguments not parsed correctly'
    
    return info


def extract_relevant_log_snippets(logs: str, max_lines: int = 10) -> str:
    """Extract relevant error information from logs"""
    lines = logs.split('\n')
    error_lines = []
    
    keywords = ['error', 'failed', 'exception', 'traceback', 'simulation_error']
    
    for line in lines:
        if any(keyword in line.lower() for keyword in keywords):
            error_lines.append(line.strip())
    
    # Return the last few error lines
    if error_lines:
        return '\n'.join(error_lines[-max_lines:])
    
    # If no specific errors found, return last few lines
    return '\n'.join([line.strip() for line in lines[-max_lines:] if line.strip()])


def create_mock_video_with_reason(execution_id: str, reason: str) -> bytes:
    """Create a mock video file with error reason embedded"""
    # Create a basic MP4 file structure with error information
    mp4_header = b'\x00\x00\x00\x20ftypmp4\x20\x00\x00\x00\x00mp41isom\x00\x00\x00\x08free'
    
    # Embed the error reason as metadata (as text)
    error_metadata = f"SIMULATION_FAILED:{execution_id}:{reason}".encode('utf-8')
    
    # Create a simple MP4-like structure
    mock_content = mp4_header + error_metadata + b'\x00' * (2048 - len(mp4_header) - len(error_metadata))
    
    return mock_content

async def run_simulation_in_docker(execution_id: str, robot_type: str, code_file: str) -> str:
    """Run the simulation inside a Docker container with enhanced error handling"""
    
    container_name = f"robot-sim-{execution_id}"
    
    # Create videos directory if it doesn't exist
    VIDEOS_DIR.mkdir(exist_ok=True)
    
    # Define paths
    host_videos_dir = VIDEOS_DIR.absolute()
    container_videos_dir = "/output"
    video_output_path = f"{container_videos_dir}/{execution_id}.mp4"
    host_video_path = host_videos_dir / f"{execution_id}.mp4"
    
    logger.info(f"Starting Docker container {container_name}")
    logger.info(f"Host videos directory: {host_videos_dir}")
    logger.info(f"Container videos directory: {container_videos_dir}")
    logger.info(f"Expected video output: {video_output_path}")
    
    # Check if Docker client is available and if robot simulation image exists
    use_mock = False
    error_details = ""
    
    try:
        if docker_client and hasattr(docker_client, 'client') and docker_client.client:
            docker_client.client.images.get("robot-simulation:latest")
            logger.info("Using Docker simulation")
        elif docker_client and docker_client.use_cli:
            # Use CLI to check for image
            result = subprocess.run(['docker', 'images', '-q', 'robot-simulation:latest'], 
                                  capture_output=True, text=True, timeout=10)
            if result.returncode != 0 or not result.stdout.strip():
                raise Exception("Docker image not found")
            logger.info("Using Docker CLI simulation")
        else:
            raise Exception("Docker not available")
    except Exception as e:
        error_details = str(e)
        logger.warning(f"Docker simulation not available ({e}), using mock simulation for testing")
        use_mock = True
    
    if use_mock:
        # Use mock simulation for testing with detailed error information
        try:
            mock_script = "/tmp/mock_simulation.py"
            
            # Create a simple mock script if it doesn't exist
            if not Path(mock_script).exists():
                mock_content = f'''#!/usr/bin/env python3
import sys
import time
import json

def create_mock_video(output_path, robot_type, reason="Mock simulation"):
    """Create a mock video file for testing"""
    # Create a minimal MP4-like file for browser compatibility
    mp4_header = b'\\x00\\x00\\x00\\x20ftypmp4\\x20\\x00\\x00\\x00\\x00mp41isom\\x00\\x00\\x00\\x08free'
    
    # Add some metadata
    metadata = f"MOCK_VIDEO:{{robot_type}}:{{reason}}".encode('utf-8')
    content = mp4_header + metadata + b'\\x00' * (2048 - len(mp4_header) - len(metadata))
    
    with open(output_path, 'wb') as f:
        f.write(content)
    
    print(f"Mock video created: {{output_path}} ({{len(content)}} bytes)")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-type", required=True)
    parser.add_argument("--code-file", required=True)
    parser.add_argument("--output-video", required=True)
    parser.add_argument("--duration", default="3")
    
    args = parser.parse_args()
    
    print(f"Mock simulation started for {{args.robot_type}}")
    time.sleep(2)  # Simulate some processing time
    
    create_mock_video(args.output_video, args.robot_type, f"Docker unavailable: {error_details}")
    print("Mock simulation completed successfully")
'''
                
                Path(mock_script).write_text(mock_content)
                Path(mock_script).chmod(0o755)
            
            # Run mock simulation
            cmd = [
                "python3", mock_script,
                "--robot-type", robot_type,
                "--code-file", code_file,
                "--output-video", str(host_video_path),
                "--duration", "3"
            ]
            
            logger.info(f"Running mock simulation: {' '.join(cmd)}")
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            
            if result.returncode == 0:
                logger.info("Mock simulation completed successfully")
                if host_video_path.exists():
                    logger.info(f"Mock video file created at {host_video_path} (size: {host_video_path.stat().st_size} bytes)")
                    return str(host_video_path)
                else:
                    raise Exception("Mock simulation did not create video file")
            else:
                logger.error(f"Mock simulation failed: {result.stderr}")
                raise Exception(f"Mock simulation failed: {result.stderr}")
                
        except Exception as e:
            logger.error(f"Mock simulation error: {e}")
            # Create a fallback mock video directly
            mock_content = create_mock_video_with_reason(execution_id, f"Mock simulation failed: {str(e)}")
            with open(host_video_path, 'wb') as f:
                f.write(mock_content)
            return str(host_video_path)
    
    # Original Docker simulation code with enhanced error handling
    volumes = {}
    
    # Handle Windows path conversion if needed
    if platform.system().lower() == "windows":
        def windows_to_docker_path(path):
            path_str = str(path).replace('\\', '/')
            if path_str[1:3] == ':/':  # Drive letter format C:/
                path_str = '/' + path_str[0].lower() + path_str[2:]
            return path_str
        
        workspace_path = windows_to_docker_path(Path(code_file).parent.absolute())
        videos_path = windows_to_docker_path(host_videos_dir)
    else:
        workspace_path = str(Path(code_file).parent.absolute())
        videos_path = str(host_videos_dir)
    
    volumes = {
        workspace_path: {'bind': '/workspace', 'mode': 'ro'},
        videos_path: {'bind': container_videos_dir, 'mode': 'rw'}
    }
    
    environment = {
        'ROBOT_TYPE': robot_type,
        'EXECUTION_ID': execution_id,
        'DISPLAY': ':99'  # Virtual display
    }
    
    container_logs = ""
    container_exit_code = -1
    
    try:
        # Run container
        container = docker_client.containers().run(
            image="robot-simulation:latest",
            name=container_name,
            volumes=volumes,
            environment=environment,
            working_dir="/workspace",
            command=[
                "python3", "/opt/simulation/run_simulation.py",
                "--robot-type", robot_type,
                "--code-file", "user_code.py",
                "--output-video", video_output_path
            ],
            detach=True,
            mem_limit="2g",
            cpu_quota=100000,  # Limit to 1 CPU
            network_mode="none",  # Disable networking for security
            remove=True
        )
        
        logger.info(f"Container {container_name} started, waiting for completion...")
        
        # Wait for container to complete (max 60 seconds)
        result = container.wait(timeout=60)
        container_logs = container.logs().decode('utf-8', errors='replace')
        container_exit_code = result['StatusCode']
        
        logger.info(f"Container {container_name} completed with status {container_exit_code}")
        
        # Save container logs to file for debugging
        log_file = Path(f"temp/{execution_id}/simulation.log")
        log_file.parent.mkdir(parents=True, exist_ok=True)
        try:
            with open(log_file, 'w') as f:
                f.write(f"Container: {container_name}\n")
                f.write(f"Exit Code: {container_exit_code}\n")
                f.write(f"Robot Type: {robot_type}\n")
                f.write(f"Code File: {code_file}\n")
                f.write(f"Output Video: {video_output_path}\n")
                f.write(f"=== CONTAINER LOGS ===\n")
                f.write(container_logs)
        except Exception as e:
            logger.warning(f"Failed to save logs to file: {e}")
        
        # Parse logs for specific error information
        error_info = parse_simulation_logs(container_logs)
        
        if container_exit_code == 0:
            # Check if video was created on the host side
            if host_video_path.exists() and host_video_path.stat().st_size > 1000:
                logger.info(f"Video successfully created at {host_video_path} (size: {host_video_path.stat().st_size} bytes)")
                return str(host_video_path)
            else:
                error_msg = "Container completed successfully but video not found or too small"
                if error_info.get('detected_issues'):
                    error_msg += f". Detected issues: {', '.join(error_info['detected_issues'])}"
                logger.error(f"{error_msg}. Container logs: {container_logs}")
                raise Exception(error_msg)
        else:
            # Container failed - provide detailed error information
            error_msg = f"Simulation failed with exit code {container_exit_code}"
            if error_info.get('primary_error'):
                error_msg = error_info['primary_error']
            elif error_info.get('detected_issues'):
                error_msg += f". Issues: {', '.join(error_info['detected_issues'])}"
            
            logger.error(f"Container {container_name} failed: {error_msg}")
            logger.error(f"Full container logs: {container_logs}")
            raise Exception(error_msg)
            
    except Exception as e:
        # Create mock video with error details
        error_message = str(e)
        if container_logs:
            log_snippets = extract_relevant_log_snippets(container_logs)
            if log_snippets:
                error_message += f"\n\nLog details:\n{log_snippets}"
        
        logger.error(f"Docker execution error: {error_message}")
        
        # Create mock video with error details
        mock_video_content = create_mock_video_with_reason(execution_id, error_message)
        with open(host_video_path, 'wb') as f:
            f.write(mock_video_content)
        
        raise Exception(error_message)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)