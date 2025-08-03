#!/usr/bin/env python3
"""
Demonstration script for the ROS Gazebo Simulation API
This script shows how to upload files and run simulations programmatically.
"""

import requests
import json
import time
import os
from pathlib import Path

def test_simulation_api():
    """Test the simulation API with sample files"""
    
    base_url = "http://localhost:8000"
    
    # Check if server is running
    try:
        response = requests.get(f"{base_url}/health")
        print(f"✓ Server is running: {response.json()}")
    except requests.exceptions.ConnectionError:
        print("✗ Error: Backend server is not running!")
        print("Please start the backend with: cd backend && python3 main.py")
        return False
    
    # Sample file paths
    script_dir = Path(__file__).parent
    urdf_file = script_dir / "sample_robot.urdf"
    world_file = script_dir / "sample_world.world"
    
    if not urdf_file.exists() or not world_file.exists():
        print("✗ Error: Sample files not found!")
        print(f"Expected files:")
        print(f"  - {urdf_file}")
        print(f"  - {world_file}")
        return False
    
    print(f"✓ Found sample files:")
    print(f"  - URDF: {urdf_file}")
    print(f"  - World: {world_file}")
    
    # Upload files
    print("\n📤 Uploading files...")
    try:
        with open(urdf_file, 'rb') as uf, open(world_file, 'rb') as wf:
            files = {
                'urdf_file': (urdf_file.name, uf, 'application/xml'),
                'world_file': (world_file.name, wf, 'application/xml')
            }
            
            response = requests.post(f"{base_url}/upload-files", files=files)
            
        if response.status_code != 200:
            print(f"✗ Upload failed: {response.status_code} - {response.text}")
            return False
            
        upload_result = response.json()
        print(f"✓ Files uploaded successfully!")
        print(f"  Upload ID: {upload_result['upload_id']}")
        print(f"  URDF path: {upload_result['urdf_path']}")
        print(f"  World path: {upload_result['world_path']}")
        
    except Exception as e:
        print(f"✗ Upload error: {e}")
        return False
    
    # Run simulation
    print("\n🎬 Running simulation...")
    try:
        simulation_request = {
            "urdf_path": upload_result['urdf_path'],
            "world_path": upload_result['world_path'],
            "duration": 10
        }
        
        print(f"Simulation duration: {simulation_request['duration']} seconds")
        print("Please wait... this may take up to 60 seconds")
        
        response = requests.post(
            f"{base_url}/simulate",
            json=simulation_request,
            timeout=120  # Allow time for container startup and simulation
        )
        
        if response.status_code != 200:
            print(f"✗ Simulation failed: {response.status_code} - {response.text}")
            return False
            
        sim_result = response.json()
        
        if sim_result['success']:
            print(f"✓ Simulation completed successfully!")
            print(f"  Execution ID: {sim_result['execution_id']}")
            print(f"  Video URL: {sim_result['video_url']}")
            print(f"  Full video URL: {base_url}{sim_result['video_url']}")
            
            # Check video file
            video_response = requests.head(f"{base_url}{sim_result['video_url']}")
            if video_response.status_code == 200:
                video_size = video_response.headers.get('content-length', 'unknown')
                print(f"✓ Video file is accessible (size: {video_size} bytes)")
            else:
                print(f"⚠ Video file not accessible: {video_response.status_code}")
                
        else:
            print(f"✗ Simulation failed: {sim_result.get('error', 'Unknown error')}")
            print("This might be expected if Docker image is not built yet.")
            print("The system should have fallen back to mock video generation.")
            return False
            
    except requests.exceptions.Timeout:
        print("✗ Simulation timed out after 120 seconds")
        return False
    except Exception as e:
        print(f"✗ Simulation error: {e}")
        return False
    
    return True

def main():
    print("🤖 ROS Gazebo Simulation API Test")
    print("=" * 40)
    
    success = test_simulation_api()
    
    print("\n" + "=" * 40)
    if success:
        print("✅ All tests passed!")
        print("\nNext steps:")
        print("1. Build Docker image: cd docker && docker build -t robot-simulation:latest .")
        print("2. Try the web interface at http://localhost:3000")
        print("3. Upload your own URDF and world files")
    else:
        print("❌ Some tests failed")
        print("\nTroubleshooting:")
        print("1. Make sure backend is running: cd backend && python3 main.py")
        print("2. Check if Docker is available")
        print("3. Review the logs for error details")

if __name__ == "__main__":
    main()