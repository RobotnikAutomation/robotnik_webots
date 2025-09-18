# robotnik_webots

This package provides integration and launch files for simulating Robotnik robots in the Webots simulator.

## Structure
- `launch/`: ROS 2 launch files for spawning robots and worlds in Webots.
- `protos/`: Custom Webots PROTO files for robot models.
- `resource/`: Robot resources, including URDF, configuration files, and meshes.
- `robotnik_webots/`: Python package directory (can be used for ROS 2 nodes or utilities).
- `worlds/`: Webots world files.

## Usage
To build this package with ROS 2 and colcon:

```bash
colcon build --packages-select robotnik_webots
```

To launch a robot in Webots:

```bash
ros2 launch robotnik_webots spawn_world.launch.py
ros2 launch robotnik_webots spawn_robot.launch.py
```

## Requirements
- ROS 2 (Humble or later)
- Webots simulator

## License
See the LICENSE file for details.
