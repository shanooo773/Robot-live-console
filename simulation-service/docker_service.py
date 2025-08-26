"""
Docker Service - Handles all Docker-related functionality
This service is completely isolated and can fail without affecting other services.
"""

import os
import uuid
import asyncio
import subprocess
import json
import logging
from pathlib import Path
from typing import Optional, Dict, Any
import time

try:
    import docker
    DOCKER_SDK_AVAILABLE = True
except ImportError:
    docker = None
    DOCKER_SDK_AVAILABLE = False

logger = logging.getLogger(__name__)

class DockerServiceException(Exception):
    """Exception raised when Docker service is unavailable"""
    pass

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

class DockerContainerManager:
    """Container manager that uses Docker CLI"""
    
    def run(self, image, command=None, **kwargs):
        """Run a container using Docker CLI"""
        container_id = f"container-{uuid.uuid4().hex[:8]}"
        
        cmd = ['docker', 'run', '--name', container_id]
        
        # Handle common parameters
        if kwargs.get('detach'):
            cmd.append('-d')
        if kwargs.get('remove'):
            cmd.append('--rm')
        
        # Handle volumes
        if 'volumes' in kwargs:
            for volume in kwargs['volumes']:
                cmd.extend(['-v', volume])
        
        # Handle environment
        if 'environment' in kwargs:
            for env in kwargs['environment']:
                cmd.extend(['-e', env])
        
        # Handle working directory
        if 'working_dir' in kwargs:
            cmd.extend(['-w', kwargs['working_dir']])
        
        cmd.append(image)
        if command:
            cmd.append(command)
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                actual_id = result.stdout.strip()
                return MockContainer(actual_id)
            else:
                raise Exception(f"Docker run failed: {result.stderr}")
        except Exception as e:
            logger.error(f"Failed to run container: {e}")
            raise

class DockerClientWrapper:
    """Wrapper that provides Docker functionality with multiple fallback options"""
    
    def __init__(self):
        self.client = None
        self.use_cli = False
        self.available = False
        
    def ping(self):
        """Check if Docker is available"""
        return self.available
        
    def containers(self):
        """Get container manager"""
        if self.use_cli:
            return DockerContainerManager()
        elif self.client:
            return self.client.containers
        else:
            raise DockerServiceException("Docker not available")

class DockerService:
    """
    Service for handling Docker-based robot simulations.
    This service is designed to fail gracefully without affecting other services.
    """
    
    def __init__(self):
        self.client = None
        self.available = False
        self.status = "initializing"
        self.error_message = None
        self._initialize()
    
    def _initialize(self):
        """Initialize Docker service with fallback options"""
        try:
            self.client = self._create_docker_client()
            if self.client and self.client.ping():
                self.available = True
                self.status = "available"
                logger.info("✅ Docker service initialized successfully")
            else:
                self._set_unavailable("Docker client created but ping failed")
        except Exception as e:
            self._set_unavailable(f"Docker initialization failed: {str(e)}")
    
    def _set_unavailable(self, error_msg):
        """Set service as unavailable with error message"""
        self.available = False
        self.status = "unavailable"
        self.error_message = error_msg
        logger.warning(f"🔴 Docker service unavailable: {error_msg}")
    
    def _create_docker_client(self):
        """Create and initialize Docker client with fallback options"""
        client_wrapper = DockerClientWrapper()
        
        if DOCKER_SDK_AVAILABLE:
            # Try default connection
            sdk_client = self._test_docker_sdk()
            if sdk_client:
                client_wrapper.client = sdk_client
                client_wrapper.available = True
                return client_wrapper
            
            # Try common Docker Desktop paths
            docker_hosts = [
                "unix:///var/run/docker.sock",
                "tcp://localhost:2375",
                "tcp://localhost:2376", 
                "npipe:////./pipe/docker_engine"
            ]
            
            for host in docker_hosts:
                sdk_client = self._test_docker_sdk(host)
                if sdk_client:
                    client_wrapper.client = sdk_client
                    client_wrapper.available = True
                    return client_wrapper
        
        # Fallback to CLI
        if self._test_docker_cli():
            client_wrapper.use_cli = True
            client_wrapper.available = True
            return client_wrapper
        
        # If all methods fail, return None
        return None
    
    def _test_docker_cli(self):
        """Test if Docker CLI is available and working"""
        try:
            result = subprocess.run(['docker', 'version'], 
                                 capture_output=True, text=True, timeout=10)
            return result.returncode == 0
        except Exception:
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
    
    def get_status(self) -> Dict[str, Any]:
        """Get current service status"""
        return {
            "service": "docker",
            "available": self.available,
            "status": self.status,
            "error": self.error_message
        }
    
    async def run_simulation(self, code: str, robot_type: str) -> Dict[str, Any]:
        """
        Run a robot simulation with the provided code.
        Returns result dict with success status and video URL or error.
        """
        if not self.available:
            # Return mock simulation result
            return await self._run_mock_simulation(code, robot_type)
        
        execution_id = str(uuid.uuid4())
        
        try:
            # Run real Docker simulation
            result = await self._run_docker_simulation(execution_id, code, robot_type)
            return result
        except Exception as e:
            logger.error(f"Docker simulation failed: {e}")
            # Fallback to mock simulation
            return await self._run_mock_simulation(code, robot_type)
    
    async def _run_docker_simulation(self, execution_id: str, code: str, robot_type: str) -> Dict[str, Any]:
        """Run real Docker simulation"""
        # Create temporary directory for this execution
        temp_dir = Path(f"temp/{execution_id}")
        temp_dir.mkdir(parents=True, exist_ok=True)
        
        try:
            # Write user code to file
            code_file = temp_dir / "user_code.py"
            with open(code_file, 'w') as f:
                f.write(code)
            
            # Create video output path
            videos_dir = Path("videos")
            videos_dir.mkdir(exist_ok=True)
            video_filename = f"{execution_id}.mp4"
            video_path = videos_dir / video_filename
            
            # Run simulation in Docker container
            container_name = f"robot-sim-{execution_id}"
            
            # Mount volumes and set environment
            volumes = [
                f"{temp_dir.absolute()}:/workspace",
                f"{videos_dir.absolute()}:/videos"
            ]
            
            environment = [
                f"ROBOT_TYPE={robot_type}",
                f"EXECUTION_ID={execution_id}",
                "DISPLAY=:99"
            ]
            
            # Run container
            container = self.client.containers().run(
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
                return {
                    "success": True,
                    "video_url": f"/videos/{video_filename}",
                    "execution_id": execution_id,
                    "simulation_type": "docker"
                }
            else:
                # Get logs for debugging
                logs = container.logs()
                logger.error(f"Simulation failed for {execution_id}: {logs}")
                return {
                    "success": False,
                    "error": "Simulation failed to complete successfully",
                    "execution_id": execution_id,
                    "simulation_type": "docker"
                }
        
        finally:
            # Cleanup temporary files
            try:
                if temp_dir.exists():
                    import shutil
                    shutil.rmtree(temp_dir)
            except Exception as e:
                logger.warning(f"Failed to cleanup temp directory: {e}")
    
    async def _run_mock_simulation(self, code: str, robot_type: str) -> Dict[str, Any]:
        """Run mock simulation when Docker is not available"""
        execution_id = str(uuid.uuid4())
        
        logger.info(f"Running mock simulation for {execution_id}")
        
        # Simulate processing time
        await asyncio.sleep(2)
        
        # Create videos directory
        videos_dir = Path("videos")
        videos_dir.mkdir(exist_ok=True)
        
        # Create a mock video file
        video_filename = f"{execution_id}.mp4"
        video_path = videos_dir / video_filename
        
        mock_video_content = f"Mock video for {robot_type} simulation - {len(code)} characters of code".encode()
        with open(video_path, 'wb') as f:
            f.write(mock_video_content)
        
        return {
            "success": True,
            "video_url": f"/videos/{video_filename}",
            "execution_id": execution_id,
            "simulation_type": "mock",
            "warning": "Docker unavailable - using mock simulation"
        }