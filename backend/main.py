import os
import uuid
import asyncio
import platform
import urllib.parse
import subprocess
import json
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
                f"python /opt/simulation/run_simulation.py --robot-type {robot_type} --code-file user_code.py --output-video {video_output_path}"
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

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)