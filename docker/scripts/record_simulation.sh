#!/bin/bash

# record_simulation.sh - Script to record Gazebo simulation as video
# Usage: record_simulation.sh --robot-type <type> --urdf-file <path> --world-file <path> --output-video <path> [--duration <seconds>]

set -e

# Default values
ROBOT_TYPE=""
URDF_FILE=""
WORLD_FILE=""
OUTPUT_VIDEO=""
DURATION=10
DISPLAY_NUM=99
RESOLUTION="1024x768"
FRAMERATE=30

# Help function
show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Record a ROS Gazebo simulation as a video"
    echo ""
    echo "OPTIONS:"
    echo "  --robot-type TYPE     Robot type (arm|hand|turtlebot)"
    echo "  --urdf-file PATH      Path to URDF file"
    echo "  --world-file PATH     Path to Gazebo world file"
    echo "  --output-video PATH   Output video file path"
    echo "  --duration SECONDS    Recording duration (default: 10)"
    echo "  --resolution WxH      Video resolution (default: 1024x768)"
    echo "  --framerate FPS       Video framerate (default: 30)"
    echo "  --help               Show this help"
    echo ""
    echo "Example:"
    echo "  $0 --robot-type arm --urdf-file /simulation_data/robot.urdf --world-file /simulation_data/world.world --output-video /output/video.mp4"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --robot-type)
            ROBOT_TYPE="$2"
            shift 2
            ;;
        --urdf-file)
            URDF_FILE="$2"
            shift 2
            ;;
        --world-file)
            WORLD_FILE="$2"
            shift 2
            ;;
        --output-video)
            OUTPUT_VIDEO="$2"
            shift 2
            ;;
        --duration)
            DURATION="$2"
            shift 2
            ;;
        --resolution)
            RESOLUTION="$2"
            shift 2
            ;;
        --framerate)
            FRAMERATE="$2"
            shift 2
            ;;
        --help)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Validate required arguments
if [[ -z "$ROBOT_TYPE" || -z "$OUTPUT_VIDEO" ]]; then
    echo "Error: --robot-type and --output-video are required"
    show_help
    exit 1
fi

# Validate robot type
if [[ ! "$ROBOT_TYPE" =~ ^(arm|hand|turtlebot)$ ]]; then
    echo "Error: Invalid robot type. Must be arm, hand, or turtlebot"
    exit 1
fi

# Use default URDF/world files if not provided
if [[ -z "$URDF_FILE" ]]; then
    URDF_FILE="/opt/simulation/robots/${ROBOT_TYPE}/${ROBOT_TYPE}.urdf"
fi

if [[ -z "$WORLD_FILE" ]]; then
    WORLD_FILE="/opt/simulation/robots/worlds/${ROBOT_TYPE}_world.world"
fi

# Check if files exist
if [[ ! -f "$URDF_FILE" ]]; then
    echo "Error: URDF file not found: $URDF_FILE"
    exit 1
fi

if [[ ! -f "$WORLD_FILE" ]]; then
    echo "Error: World file not found: $WORLD_FILE"
    exit 1
fi

# Create output directory
mkdir -p "$(dirname "$OUTPUT_VIDEO")"

echo "Starting Gazebo simulation recording..."
echo "Robot type: $ROBOT_TYPE"
echo "URDF file: $URDF_FILE"
echo "World file: $WORLD_FILE"
echo "Output video: $OUTPUT_VIDEO"
echo "Duration: ${DURATION}s"
echo "Resolution: $RESOLUTION"

# Source ROS environment
source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH=/opt/simulation:$ROS_PACKAGE_PATH

# Start virtual display
echo "Starting virtual display :$DISPLAY_NUM..."
Xvfb :$DISPLAY_NUM -screen 0 ${RESOLUTION}x24 &
XVFB_PID=$!
export DISPLAY=:$DISPLAY_NUM

# Wait for display to start
sleep 2

# Function to cleanup on exit
cleanup() {
    echo "Cleaning up..."
    # Kill all ROS processes
    pkill -f roslaunch || true
    pkill -f rosmaster || true
    pkill -f gzserver || true
    pkill -f gzclient || true
    # Kill virtual display
    kill $XVFB_PID 2>/dev/null || true
    # Kill ffmpeg if still running
    pkill -f ffmpeg || true
}

# Set up cleanup on script exit
trap cleanup EXIT

# Start ROS master
echo "Starting ROS master..."
roscore &
ROS_PID=$!

# Wait for ROS master to start
sleep 3

# Launch Gazebo with robot
echo "Launching Gazebo simulation..."
roslaunch /opt/simulation/launch/spawn_robot.launch \
    robot_type:=$ROBOT_TYPE \
    urdf_file:=$URDF_FILE \
    world_file:=$WORLD_FILE \
    gui:=false \
    headless:=false &
GAZEBO_PID=$!

# Wait for Gazebo to stabilize
echo "Waiting for Gazebo to stabilize..."
sleep 5

# Start video recording
echo "Starting video recording for ${DURATION} seconds..."
ffmpeg -y -f x11grab -r $FRAMERATE -s $RESOLUTION -i :$DISPLAY_NUM \
    -vcodec libx264 -preset ultrafast -crf 25 \
    -t $DURATION "$OUTPUT_VIDEO" &
FFMPEG_PID=$!

# Execute user code if it exists
USER_CODE_FILE="/workspace/user_code.py"
if [[ -f "$USER_CODE_FILE" ]]; then
    echo "Executing user code..."
    timeout $((DURATION - 2)) python3 "$USER_CODE_FILE" || true
fi

# Wait for recording to complete
wait $FFMPEG_PID

echo "Recording completed. Video saved to: $OUTPUT_VIDEO"

# Verify video file
if [[ -f "$OUTPUT_VIDEO" && -s "$OUTPUT_VIDEO" ]]; then
    VIDEO_SIZE=$(du -h "$OUTPUT_VIDEO" | cut -f1)
    echo "Video file created successfully (size: $VIDEO_SIZE)"
else
    echo "Error: Video file was not created or is empty"
    exit 1
fi

echo "Simulation recording completed successfully!"