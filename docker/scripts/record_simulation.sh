#!/bin/bash

# record_simulation.sh - Script to record Gazebo simulation as video
# Usage: record_simulation.sh --robot-type <type> --urdf-file <path> --world-file <path> --output-video <path> [--duration <seconds>]

# Enhanced error handling and logging
set -e
set -o pipefail

# Logging configuration
LOG_FILE="/tmp/simulation.log"
exec > >(tee -a "$LOG_FILE")
exec 2> >(tee -a "$LOG_FILE" >&2)

# Function to log with timestamp
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

# Function to log error and exit
error_exit() {
    local error_code=$1
    local message=$2
    log "ERROR: $message"
    echo "SIMULATION_ERROR_CODE: $error_code"
    echo "SIMULATION_ERROR_MESSAGE: $message"
    exit $error_code
}

log "Starting simulation script with arguments: $*"

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
    error_exit 10 "--robot-type and --output-video are required"
fi

# Validate robot type
if [[ ! "$ROBOT_TYPE" =~ ^(arm|hand|turtlebot)$ ]]; then
    error_exit 11 "Invalid robot type '$ROBOT_TYPE'. Must be arm, hand, or turtlebot"
fi

log "Robot type validated: $ROBOT_TYPE"

# Use default URDF/world files if not provided
if [[ -z "$URDF_FILE" ]]; then
    URDF_FILE="/opt/simulation/robots/${ROBOT_TYPE}/${ROBOT_TYPE}.urdf"
    log "Using default URDF file: $URDF_FILE"
fi

if [[ -z "$WORLD_FILE" ]]; then
    WORLD_FILE="/opt/simulation/robots/worlds/${ROBOT_TYPE}_world.world"
    log "Using default world file: $WORLD_FILE"
fi

# Enhanced file validation with detailed error messages
log "Validating input files..."

if [[ ! -f "$URDF_FILE" ]]; then
    error_exit 20 "URDF file not found: $URDF_FILE"
fi

# Check URDF file is readable and not empty
if [[ ! -r "$URDF_FILE" ]]; then
    error_exit 21 "URDF file not readable: $URDF_FILE"
fi

if [[ ! -s "$URDF_FILE" ]]; then
    error_exit 22 "URDF file is empty: $URDF_FILE"
fi

# Basic URDF validation
if ! grep -q "<robot" "$URDF_FILE"; then
    error_exit 23 "URDF file does not contain valid robot definition: $URDF_FILE"
fi

log "URDF file validated: $URDF_FILE ($(stat -c%s "$URDF_FILE") bytes)"

if [[ ! -f "$WORLD_FILE" ]]; then
    error_exit 30 "World file not found: $WORLD_FILE"
fi

if [[ ! -r "$WORLD_FILE" ]]; then
    error_exit 31 "World file not readable: $WORLD_FILE"
fi

if [[ ! -s "$WORLD_FILE" ]]; then
    error_exit 32 "World file is empty: $WORLD_FILE"
fi

# Basic world file validation
if ! grep -q "<world" "$WORLD_FILE"; then
    error_exit 33 "World file does not contain valid world definition: $WORLD_FILE"
fi

log "World file validated: $WORLD_FILE ($(stat -c%s "$WORLD_FILE") bytes)"

# Create output directory
OUTPUT_DIR="$(dirname "$OUTPUT_VIDEO")"
if ! mkdir -p "$OUTPUT_DIR"; then
    error_exit 40 "Failed to create output directory: $OUTPUT_DIR"
fi

log "Output directory created: $OUTPUT_DIR"

log "Starting Gazebo simulation recording..."
log "Robot type: $ROBOT_TYPE"
log "URDF file: $URDF_FILE"
log "World file: $WORLD_FILE"
log "Output video: $OUTPUT_VIDEO"
log "Duration: ${DURATION}s"
log "Resolution: $RESOLUTION"
log "Display: :$DISPLAY_NUM"

# Check system requirements
log "Checking system requirements..."

# Check if required commands are available
if ! command -v Xvfb >/dev/null; then
    error_exit 50 "Xvfb not found - virtual display server required"
fi

if ! command -v ffmpeg >/dev/null; then
    error_exit 51 "ffmpeg not found - video encoding required"
fi

if ! command -v roslaunch >/dev/null; then
    error_exit 52 "roslaunch not found - ROS environment not properly set up"
fi

if ! command -v roscore >/dev/null; then
    error_exit 53 "roscore not found - ROS environment not properly set up"
fi

log "System requirements validated"

# Source ROS environment
log "Setting up ROS environment..."
if [[ ! -f "/opt/ros/noetic/setup.bash" ]]; then
    error_exit 60 "ROS Noetic setup file not found: /opt/ros/noetic/setup.bash"
fi

source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH=/opt/simulation:$ROS_PACKAGE_PATH

# Validate ROS environment
if [[ -z "$ROS_DISTRO" ]]; then
    error_exit 61 "ROS_DISTRO not set after sourcing ROS setup"
fi

if [[ "$ROS_DISTRO" != "noetic" ]]; then
    error_exit 62 "Expected ROS Noetic, got: $ROS_DISTRO"
fi

log "ROS environment configured (distro: $ROS_DISTRO)"

# Start virtual display with enhanced error checking
log "Starting virtual display :$DISPLAY_NUM..."

# Kill any existing Xvfb processes on the display
pkill -f "Xvfb :$DISPLAY_NUM" || true

# Start Xvfb with more detailed logging
if ! Xvfb :$DISPLAY_NUM -screen 0 ${RESOLUTION}x24 -ac +extension GLX +render -noreset > /tmp/xvfb.log 2>&1 &
then
    error_exit 70 "Failed to start Xvfb virtual display"
fi

XVFB_PID=$!
export DISPLAY=:$DISPLAY_NUM

log "Virtual display started (PID: $XVFB_PID)"

# Wait for display to start and verify it's working
sleep 3

if ! xset q > /dev/null 2>&1; then
    error_exit 71 "Virtual display not responding after startup"
fi

log "Virtual display verified and responding"

# Enhanced cleanup function with better logging
cleanup() {
    local exit_code=$?
    log "Cleanup initiated (exit code: $exit_code)"
    
    # Kill all processes with proper error handling
    log "Stopping ROS processes..."
    pkill -f roslaunch || true
    pkill -f rosmaster || true
    pkill -f gzserver || true
    pkill -f gzclient || true
    
    # Kill virtual display
    if [[ -n "$XVFB_PID" ]]; then
        log "Stopping virtual display (PID: $XVFB_PID)..."
        kill $XVFB_PID 2>/dev/null || true
    fi
    
    # Kill ffmpeg if still running
    log "Stopping video recording..."
    pkill -f ffmpeg || true
    
    # Copy log file to output directory for debugging
    if [[ -f "$LOG_FILE" && -d "$(dirname "$OUTPUT_VIDEO")" ]]; then
        cp "$LOG_FILE" "$(dirname "$OUTPUT_VIDEO")/simulation.log" || true
        log "Log file copied to output directory"
    fi
    
    log "Cleanup completed"
}

# Set up cleanup on script exit
trap cleanup EXIT INT TERM

# Start ROS master with enhanced error checking
log "Starting ROS master..."

# Kill any existing ROS processes
pkill -f rosmaster || true
sleep 1

if ! roscore > /tmp/roscore.log 2>&1 &
then
    error_exit 80 "Failed to start ROS master"
fi

ROS_PID=$!
log "ROS master started (PID: $ROS_PID)"

# Wait for ROS master to start and verify connectivity
log "Waiting for ROS master to become available..."
for i in {1..30}; do
    if rostopic list > /dev/null 2>&1; then
        log "ROS master is responding (attempt $i)"
        break
    fi
    if [[ $i -eq 30 ]]; then
        error_exit 81 "ROS master failed to become available after 30 seconds"
    fi
    sleep 1
done

# Check if launch file exists
LAUNCH_FILE="/opt/simulation/launch/spawn_robot.launch"
if [[ ! -f "$LAUNCH_FILE" ]]; then
    error_exit 82 "Launch file not found: $LAUNCH_FILE"
fi

log "Launch file validated: $LAUNCH_FILE"

# Launch Gazebo with robot and enhanced error detection
log "Launching Gazebo simulation..."
log "Command: roslaunch $LAUNCH_FILE robot_type:=$ROBOT_TYPE urdf_file:=$URDF_FILE world_file:=$WORLD_FILE gui:=false headless:=false"

if ! roslaunch "$LAUNCH_FILE" \
    robot_type:="$ROBOT_TYPE" \
    urdf_file:="$URDF_FILE" \
    world_file:="$WORLD_FILE" \
    gui:=false \
    headless:=false > /tmp/gazebo.log 2>&1 &
then
    error_exit 90 "Failed to start Gazebo simulation"
fi

GAZEBO_PID=$!
log "Gazebo simulation launched (PID: $GAZEBO_PID)"

# Wait for Gazebo to stabilize with better status checking
log "Waiting for Gazebo to stabilize..."
STABILIZE_TIME=10
for i in $(seq 1 $STABILIZE_TIME); do
    log "Gazebo stabilization check $i/$STABILIZE_TIME"
    
    # Check if Gazebo process is still running
    if ! kill -0 $GAZEBO_PID 2>/dev/null; then
        error_exit 91 "Gazebo process died during startup"
    fi
    
    # Check for Gazebo topics (indicates Gazebo is running)
    if rostopic list 2>/dev/null | grep -q "/gazebo"; then
        log "Gazebo topics detected, simulation is running"
        break
    fi
    
    if [[ $i -eq $STABILIZE_TIME ]]; then
        log "Gazebo log output:"
        tail -20 /tmp/gazebo.log || true
        error_exit 92 "Gazebo failed to start properly - no topics detected after ${STABILIZE_TIME}s"
    fi
    
    sleep 1
done

log "Gazebo simulation stabilized successfully"

# Start video recording with enhanced error detection
log "Starting video recording for ${DURATION} seconds..."
log "Command: ffmpeg -y -f x11grab -r $FRAMERATE -s $RESOLUTION -i :$DISPLAY_NUM -vcodec libx264 -preset ultrafast -crf 25 -t $DURATION $OUTPUT_VIDEO"

# Test if we can capture from the display first
if ! timeout 2 ffmpeg -y -f x11grab -r 1 -s $RESOLUTION -i :$DISPLAY_NUM -t 1 -f null - >/dev/null 2>&1; then
    error_exit 100 "Cannot capture from display :$DISPLAY_NUM - X11 grab test failed"
fi

log "X11 display capture test successful"

# Start actual recording
if ! ffmpeg -y -f x11grab -r $FRAMERATE -s $RESOLUTION -i :$DISPLAY_NUM \
    -vcodec libx264 -preset ultrafast -crf 25 \
    -t $DURATION "$OUTPUT_VIDEO" > /tmp/ffmpeg.log 2>&1 &
then
    error_exit 101 "Failed to start ffmpeg video recording"
fi

FFMPEG_PID=$!
log "Video recording started (PID: $FFMPEG_PID)"

# Execute user code if it exists
USER_CODE_FILE="/workspace/user_code.py"
if [[ -f "$USER_CODE_FILE" ]]; then
    log "Executing user code: $USER_CODE_FILE"
    if timeout $((DURATION - 2)) python3 "$USER_CODE_FILE" > /tmp/user_code.log 2>&1; then
        log "User code executed successfully"
    else
        local user_exit_code=$?
        log "WARNING: User code execution failed with exit code: $user_exit_code"
        log "User code output:"
        tail -20 /tmp/user_code.log || true
        # Don't fail the entire simulation for user code errors
    fi
else
    log "No user code file found at $USER_CODE_FILE"
fi

# Wait for recording to complete with progress monitoring
log "Waiting for video recording to complete..."
WAIT_START=$(date +%s)

while kill -0 $FFMPEG_PID 2>/dev/null; do
    CURRENT_TIME=$(date +%s)
    ELAPSED=$((CURRENT_TIME - WAIT_START))
    
    if [[ $ELAPSED -gt $((DURATION + 10)) ]]; then
        log "WARNING: ffmpeg is taking longer than expected, killing it"
        kill $FFMPEG_PID 2>/dev/null || true
        break
    fi
    
    if [[ $((ELAPSED % 5)) -eq 0 ]]; then
        log "Recording in progress... ${ELAPSED}s elapsed"
    fi
    
    sleep 1
done

# Get ffmpeg exit code
wait $FFMPEG_PID 2>/dev/null
FFMPEG_EXIT_CODE=$?

if [[ $FFMPEG_EXIT_CODE -ne 0 ]]; then
    log "ffmpeg log output:"
    tail -20 /tmp/ffmpeg.log || true
    error_exit 102 "ffmpeg failed with exit code: $FFMPEG_EXIT_CODE"
fi

log "Video recording completed successfully"

# Enhanced video file verification
log "Verifying output video file..."

if [[ ! -f "$OUTPUT_VIDEO" ]]; then
    error_exit 110 "Video file was not created: $OUTPUT_VIDEO"
fi

if [[ ! -s "$OUTPUT_VIDEO" ]]; then
    error_exit 111 "Video file is empty: $OUTPUT_VIDEO"
fi

VIDEO_SIZE=$(stat -c%s "$OUTPUT_VIDEO")
if [[ $VIDEO_SIZE -lt 1000 ]]; then
    error_exit 112 "Video file too small (${VIDEO_SIZE} bytes), likely corrupted: $OUTPUT_VIDEO"
fi

# Test video file integrity with ffprobe if available
if command -v ffprobe >/dev/null; then
    if ! ffprobe -v quiet -print_format json -show_format "$OUTPUT_VIDEO" > /tmp/video_info.json 2>&1; then
        log "ffprobe output:"
        cat /tmp/video_info.json || true
        error_exit 113 "Video file appears to be corrupted (ffprobe failed): $OUTPUT_VIDEO"
    fi
    
    # Extract duration from ffprobe output
    ACTUAL_DURATION=$(python3 -c "
import json, sys
try:
    with open('/tmp/video_info.json') as f:
        data = json.load(f)
    duration = float(data['format']['duration'])
    print(f'{duration:.1f}')
except:
    print('unknown')
" 2>/dev/null)
    
    log "Video file integrity verified (duration: ${ACTUAL_DURATION}s)"
else
    log "ffprobe not available, skipping video integrity check"
fi

VIDEO_SIZE_MB=$(python3 -c "print(f'{$VIDEO_SIZE/1024/1024:.1f}')")
log "Video file created successfully: $OUTPUT_VIDEO (${VIDEO_SIZE_MB}MB)"

log "Simulation recording completed successfully!"
log "Final status: SUCCESS"