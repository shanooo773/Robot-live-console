# Sample Files for Testing

This directory contains sample URDF and world files for testing the ROS Gazebo simulation system.

## Files

### sample_robot.urdf
A simple robot with:
- Blue rectangular base
- Red cylindrical upper link
- Single revolute joint (rotates around Z-axis)
- Gazebo control plugin

### sample_world.world
A basic Gazebo world with:
- Ground plane
- Sun (lighting)
- Wooden table
- Orange box for interaction
- Physics settings

## How to Use

1. **Via Web Interface:**
   - Go to the "Simulation Uploader" tab
   - Upload `sample_robot.urdf` as the URDF file
   - Upload `sample_world.world` as the World file
   - Set duration (e.g., 10 seconds)
   - Click "Run Gazebo Simulation"

2. **Via API:**
   ```bash
   # Upload files
   curl -X POST http://localhost:8000/upload-files \
     -F "urdf_file=@sample_robot.urdf" \
     -F "world_file=@sample_world.world"
   
   # Run simulation (use paths from upload response)
   curl -X POST http://localhost:8000/simulate \
     -H "Content-Type: application/json" \
     -d '{"urdf_path": "/path/to/uploaded/sample_robot.urdf", "world_path": "/path/to/uploaded/sample_world.world", "duration": 10}'
   ```

## Expected Behavior

The simulation will:
1. Spawn the blue robot base on the ground
2. Show the red upper link attached via joint
3. Display the table and orange box
4. Record a 10-second video showing the static scene
5. Allow any user Python code to control the joint

## Customization

You can modify these files to:
- Add more joints and links
- Change colors and materials
- Add more objects to the world
- Include sensors (cameras, lidars)
- Add physics properties

## Troubleshooting

If the simulation fails:
1. Check URDF syntax with `check_urdf sample_robot.urdf`
2. Validate world file in Gazebo directly
3. Look at backend logs for error messages
4. The system will fall back to mock video if real simulation fails