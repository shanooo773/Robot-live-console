#!/usr/bin/env python3
"""
Test script for backend simulation endpoints
"""
import asyncio
import requests
import json
import time
import os
from pathlib import Path

BASE_URL = "http://localhost:8000"

def test_backend_health():
    """Test if backend is running and healthy"""
    try:
        response = requests.get(f"{BASE_URL}/status", timeout=5)
        if response.status_code == 200:
            print("✅ Backend health check passed")
            return True
        else:
            print(f"❌ Backend health check failed: {response.status_code}")
            return False
    except requests.exceptions.RequestException as e:
        print(f"❌ Backend connection failed: {e}")
        return False

def test_docker_status():
    """Test Docker availability"""
    try:
        response = requests.get(f"{BASE_URL}/docker-status", timeout=5)
        data = response.json()
        print(f"Docker status: {data['status']}")
        if 'containers' in data:
            print(f"Running containers: {data['containers']}")
        return response.status_code == 200
    except Exception as e:
        print(f"❌ Docker status check failed: {e}")
        return False

def test_code_execution():
    """Test basic code execution endpoint"""
    test_code = """#!/usr/bin/env python3
import rospy
import time

def test_robot():
    print("Test robot code executed successfully")
    time.sleep(1)
    print("Robot simulation completed")

if __name__ == '__main__':
    test_robot()
"""
    
    payload = {
        "code": test_code,
        "robot_type": "arm"
    }
    
    try:
        print("Testing code execution endpoint...")
        response = requests.post(f"{BASE_URL}/run-code", json=payload, timeout=120)
        data = response.json()
        
        if response.status_code == 200:
            if data.get('success'):
                print("✅ Code execution successful")
                print(f"Video URL: {data.get('video_url')}")
                print(f"Execution ID: {data.get('execution_id')}")
                
                # Test video accessibility
                if data.get('video_url'):
                    video_response = requests.head(f"{BASE_URL}{data['video_url']}", timeout=10)
                    if video_response.status_code == 200:
                        print("✅ Video file accessible")
                        print(f"Video size: {video_response.headers.get('content-length', 'unknown')} bytes")
                    else:
                        print(f"❌ Video file not accessible: {video_response.status_code}")
                
                return True
            else:
                print(f"❌ Code execution failed: {data.get('error')}")
                return False
        else:
            print(f"❌ Code execution request failed: {response.status_code}")
            print(f"Response: {data}")
            return False
            
    except Exception as e:
        print(f"❌ Code execution test failed: {e}")
        return False

def test_file_upload():
    """Test file upload functionality"""
    try:
        # Create test URDF file
        test_urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
  </link>
</robot>"""

        # Create test world file
        test_world = """<?xml version="1.0"?>
<sdf version="1.6">
  <world name="test_world">
    <physics type="ode">
      <gravity>0 0 -9.81</gravity>
    </physics>
  </world>
</sdf>"""

        # Create temporary files
        urdf_path = Path("/tmp/test_robot.urdf")
        world_path = Path("/tmp/test_world.world")
        
        urdf_path.write_text(test_urdf)
        world_path.write_text(test_world)
        
        # Test upload
        with open(urdf_path, 'rb') as urdf_file, open(world_path, 'rb') as world_file:
            files = {
                'urdf_file': ('test_robot.urdf', urdf_file, 'application/xml'),
                'world_file': ('test_world.world', world_file, 'application/xml')
            }
            
            response = requests.post(f"{BASE_URL}/upload-files", files=files, timeout=30)
            
        if response.status_code == 200:
            data = response.json()
            if data.get('success'):
                print("✅ File upload successful")
                print(f"Upload ID: {data.get('upload_id')}")
                print(f"Validation: {data.get('validation', {})}")
                
                # Test simulation with uploaded files
                if data.get('urdf_path') and data.get('world_path'):
                    sim_payload = {
                        "urdf_path": data['urdf_path'],
                        "world_path": data['world_path'],
                        "duration": 5
                    }
                    
                    sim_response = requests.post(f"{BASE_URL}/simulate", json=sim_payload, timeout=120)
                    if sim_response.status_code == 200:
                        sim_data = sim_response.json()
                        if sim_data.get('success'):
                            print("✅ Simulation with uploaded files successful")
                        else:
                            print(f"❌ Simulation failed: {sim_data.get('error')}")
                    else:
                        print(f"❌ Simulation request failed: {sim_response.status_code}")
                
                return True
            else:
                print(f"❌ File upload failed: {data}")
                return False
        else:
            print(f"❌ File upload request failed: {response.status_code}")
            return False
            
        # Cleanup
        urdf_path.unlink(missing_ok=True)
        world_path.unlink(missing_ok=True)
        
    except Exception as e:
        print(f"❌ File upload test failed: {e}")
        return False

def test_videos_debug():
    """Test videos debug endpoint"""
    try:
        response = requests.get(f"{BASE_URL}/videos-debug", timeout=10)
        if response.status_code == 200:
            data = response.json()
            print("✅ Videos debug info retrieved")
            print(f"Videos directory exists: {data.get('videos_directory_exists')}")
            print(f"Total video files: {data.get('total_files')}")
            return True
        else:
            print(f"❌ Videos debug failed: {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ Videos debug test failed: {e}")
        return False

def main():
    """Run all tests"""
    print("🧪 Starting backend simulation tests...\n")
    
    tests = [
        ("Backend Health", test_backend_health),
        ("Docker Status", test_docker_status),
        ("Videos Debug", test_videos_debug),
        ("Code Execution", test_code_execution),
        ("File Upload", test_file_upload),
    ]
    
    results = []
    
    for test_name, test_func in tests:
        print(f"\n🔍 Running {test_name} test...")
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"❌ {test_name} test crashed: {e}")
            results.append((test_name, False))
    
    print("\n" + "="*50)
    print("📊 Test Results Summary:")
    print("="*50)
    
    passed = 0
    total = len(results)
    
    for test_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{test_name}: {status}")
        if result:
            passed += 1
    
    print(f"\nTotal: {passed}/{total} tests passed")
    
    if passed == total:
        print("🎉 All tests passed!")
        return True
    else:
        print(f"⚠️  {total - passed} tests failed")
        return False

if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)