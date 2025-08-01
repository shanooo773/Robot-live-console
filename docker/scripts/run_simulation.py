#!/usr/bin/env python3
"""
Main simulation runner that executes user code in a ROS/Gazebo environment
and records the simulation as a video.
"""

import os
import sys
import time
import argparse
import subprocess
import threading
import signal
from pathlib import Path

class SimulationRunner:
    def __init__(self, robot_type, code_file, output_video, duration=10):
        self.robot_type = robot_type
        self.code_file = code_file
        self.output_video = output_video
        self.duration = duration
        self.processes = []
        
    def start_virtual_display(self):
        """Start virtual X server for headless operation"""
        print("Starting virtual display...")
        xvfb_proc = subprocess.Popen([
            'Xvfb', ':99', '-screen', '0', '1024x768x24'
        ])
        self.processes.append(xvfb_proc)
        time.sleep(2)
        
    def start_roscore(self):
        """Start ROS master"""
        print("Starting ROS master...")
        roscore_proc = subprocess.Popen(['roscore'])
        self.processes.append(roscore_proc)
        time.sleep(3)
        
    def start_gazebo(self):
        """Start Gazebo simulation with the appropriate world"""
        print(f"Starting Gazebo simulation for {self.robot_type}...")
        
        # Select world file based on robot type
        world_files = {
            'arm': '/opt/simulation/robots/worlds/arm_world.world',
            'hand': '/opt/simulation/robots/worlds/hand_world.world', 
            'turtlebot': '/opt/simulation/robots/worlds/turtlebot_world.world'
        }
        
        world_file = world_files.get(self.robot_type, world_files['turtlebot'])
        
        # Start Gazebo
        gazebo_proc = subprocess.Popen([
            'gazebo', '--verbose', 
            '-s', 'libgazebo_ros_api_plugin.so',
            world_file
        ], env=dict(os.environ, DISPLAY=':99'))
        self.processes.append(gazebo_proc)
        time.sleep(5)
        
    def spawn_robot(self):
        """Spawn the robot in Gazebo"""
        print(f"Spawning {self.robot_type} robot...")
        
        robot_configs = {
            'arm': {
                'model': 'robot_arm',
                'urdf': '/opt/simulation/robots/arm/arm.urdf',
                'x': 0, 'y': 0, 'z': 1
            },
            'hand': {
                'model': 'robot_hand',
                'urdf': '/opt/simulation/robots/hand/hand.urdf',
                'x': 0, 'y': 0, 'z': 0.5
            },
            'turtlebot': {
                'model': 'turtlebot3_burger',
                'urdf': '/opt/simulation/robots/turtlebot/turtlebot3_burger.urdf',
                'x': 0, 'y': 0, 'z': 0.1
            }
        }
        
        config = robot_configs.get(self.robot_type, robot_configs['turtlebot'])
        
        # Spawn model using rosrun
        spawn_proc = subprocess.Popen([
            'rosrun', 'gazebo_ros', 'spawn_model',
            '-file', config['urdf'],
            '-urdf',
            '-model', config['model'],
            '-x', str(config['x']),
            '-y', str(config['y']),
            '-z', str(config['z'])
        ])
        
        spawn_proc.wait()
        time.sleep(2)
        
    def start_video_recording(self):
        """Start recording the simulation"""
        print("Starting video recording...")
        
        # Ensure output directory exists
        Path(self.output_video).parent.mkdir(parents=True, exist_ok=True)
        
        # Start ffmpeg recording
        ffmpeg_proc = subprocess.Popen([
            'ffmpeg', '-y',
            '-f', 'x11grab',
            '-video_size', '1024x768',
            '-i', ':99.0',
            '-t', str(self.duration),
            '-r', '30',
            '-c:v', 'libx264',
            '-preset', 'fast',
            '-crf', '23',
            self.output_video
        ])
        self.processes.append(ffmpeg_proc)
        return ffmpeg_proc
        
    def execute_user_code(self):
        """Execute the user's Python code"""
        print("Executing user code...")
        
        # Prepare ROS environment for user code
        env = os.environ.copy()
        env['ROS_MASTER_URI'] = 'http://localhost:11311'
        env['ROS_IP'] = '127.0.0.1'
        
        try:
            # Execute user code
            result = subprocess.run([
                'python3', self.code_file
            ], env=env, capture_output=True, text=True, timeout=self.duration-2)
            
            if result.returncode != 0:
                print(f"User code error: {result.stderr}")
            else:
                print("User code executed successfully")
                print(f"Output: {result.stdout}")
                
        except subprocess.TimeoutExpired:
            print("User code execution timed out")
        except Exception as e:
            print(f"Error executing user code: {e}")
            
    def cleanup(self):
        """Clean up all processes"""
        print("Cleaning up processes...")
        for proc in self.processes:
            try:
                proc.terminate()
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
            except:
                pass
                
    def run(self):
        """Run the complete simulation"""
        try:
            # Start all components
            self.start_virtual_display()
            self.start_roscore()
            self.start_gazebo()
            self.spawn_robot()
            
            # Start recording and wait a moment for everything to stabilize
            ffmpeg_proc = self.start_video_recording()
            time.sleep(1)
            
            # Execute user code
            self.execute_user_code()
            
            # Wait for recording to complete
            ffmpeg_proc.wait()
            
            print(f"Simulation complete. Video saved to: {self.output_video}")
            
        except Exception as e:
            print(f"Simulation error: {e}")
            return False
        finally:
            self.cleanup()
            
        return True

def main():
    parser = argparse.ArgumentParser(description='Run robot simulation')
    parser.add_argument('--robot-type', required=True, 
                       choices=['arm', 'hand', 'turtlebot'],
                       help='Type of robot to simulate')
    parser.add_argument('--code-file', required=True,
                       help='Path to Python code file to execute')
    parser.add_argument('--output-video', required=True,
                       help='Path for output video file')
    parser.add_argument('--duration', type=int, default=10,
                       help='Simulation duration in seconds')
    
    args = parser.parse_args()
    
    # Validate inputs
    if not Path(args.code_file).exists():
        print(f"Error: Code file {args.code_file} not found")
        return 1
        
    # Run simulation
    runner = SimulationRunner(
        robot_type=args.robot_type,
        code_file=args.code_file,
        output_video=args.output_video,
        duration=args.duration
    )
    
    success = runner.run()
    return 0 if success else 1

if __name__ == '__main__':
    sys.exit(main())