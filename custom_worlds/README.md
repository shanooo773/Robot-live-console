# Custom Gazebo Worlds

Place your custom `.world` files in this directory.

They will be mounted to `/opt/simulation/custom_worlds` inside the container.

## Example Usage

1. Copy your world file here: `custom_worlds/my_world.world`
2. Access it in Gazebo via the File menu or modify the startup script
3. The path inside container will be: `/opt/simulation/custom_worlds/my_world.world`