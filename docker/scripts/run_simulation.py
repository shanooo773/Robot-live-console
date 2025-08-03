#!/usr/bin/env python3
"""
Simplified simulation runner that creates mock videos.
This version works without ROS/Gazebo for testing and development.
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
        
    def execute_user_code(self):
        """Execute the user's Python code"""
        print("Executing user code...")
        
        try:
            # Execute user code
            result = subprocess.run([
                'python3', self.code_file
            ], capture_output=True, text=True, timeout=self.duration-2)
            
            if result.returncode != 0:
                print(f"User code error: {result.stderr}")
            else:
                print("User code executed successfully")
                print(f"Output: {result.stdout}")
                
        except subprocess.TimeoutExpired:
            print("User code execution timed out")
        except Exception as e:
            print(f"Error executing user code: {e}")
    
    def create_mock_video(self):
        """Create a mock MP4 video file"""
        print(f"Creating simulation video for {self.robot_type}...")
        
        # Ensure output directory exists
        Path(self.output_video).parent.mkdir(parents=True, exist_ok=True)
        
        # Create a minimal valid MP4 file structure
        mp4_header = b'\x00\x00\x00\x20ftypmp4\x20\x00\x00\x00\x00mp41isom\x00\x00\x00\x08free'
        
        # Add robot-specific content
        robot_content = {
            'arm': b'ARM_SIMULATION_DATA',
            'hand': b'HAND_SIMULATION_DATA', 
            'turtlebot': b'TURTLEBOT_SIMULATION_DATA'
        }
        
        specific_content = robot_content.get(self.robot_type, b'GENERIC_ROBOT_DATA')
        
        # Scale content with duration
        mock_data_size = max(1000, self.duration * 200)
        mock_video_data = specific_content + (b'\x00' * mock_data_size)
        
        # Write the mock MP4 file
        with open(self.output_video, 'wb') as f:
            f.write(mp4_header)
            f.write(mock_video_data)
        
        print(f"Video created: {self.output_video} ({len(mp4_header) + len(mock_video_data)} bytes)")
            
    def run(self):
        """Run the complete simulation"""
        try:
            print(f"Starting simulation for {self.robot_type}")
            print(f"Code file: {self.code_file}")
            print(f"Output video: {self.output_video}")
            print(f"Duration: {self.duration}s")
            
            # Execute user code
            self.execute_user_code()
            
            # Create mock video
            self.create_mock_video()
            
            print(f"Simulation complete. Video saved to: {self.output_video}")
            
        except Exception as e:
            print(f"Simulation error: {e}")
            return False
            
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