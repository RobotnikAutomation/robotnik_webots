# robotnik_webots

This package provides integration and launch files for simulating Robotnik robots in the Webots simulator (lastly tested webotsR2025a version for ROS2 Jazzy).

## Structure
- `launch/`: ROS 2 launch files for spawning robots and worlds in Webots.
- `protos/`: Custom Webots PROTO files for robot models.
- `resource/`: Robot resources, including URDF, configuration files, and meshes.
- `robotnik_webots/`: Python package directory (can be used for ROS 2 nodes or utilities).
- `worlds/`: Webots world files.

## Requirements
- ROS 2 Jazzy or Humble
- Webots simulator version: R2023b

## Usage
Install required packages to run this package:

```bash
python3 -m pip install --user catkin_pkg

sudo apt install -y ros-$ROS_DISTRO-geometry-msgs ros-$ROS_DISTRO-webots-ros2 ros-$ROS_DISTRO-webots-ros2-driver ros-$ROS_DISTRO-webots-ros2* ros-$ROS_DISTRO-controller-manager ros-$ROS_DISTRO-control-msgs ros-$ROS_DISTRO-control-toolbox ros-$ROS_DISTRO-controller-interface ros-$ROS_DISTRO-joint-state-broadcaster ros-$ROS_DISTRO-joint-trajectory-controller ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-rqt-joint-trajectory-controller
```

To build this package with ROS 2 and colcon:

```bash
colcon build --packages-select robotnik_webots
```


## Launch files and arguments

### 1. Launch the Webots world

```bash
ros2 launch robotnik_webots spawn_world.launch.py
```
<!--
or with the new launch file:
```bash
ros2 launch robotnik_webots new_spawn_world.launch.py
```
-->

#### Arguments for `spawn_world.launch.py` 
<!--
and `new_spawn_world.launch.py`
-->

- `world`: Name of the world file (without extension). Default: `default_world`
- `world_path`: Full path to the world file. Default: `<package_share>/worlds/<world>.wbt`
- `gui`: Launch Webots with GUI (`true`/`false`). Default: `true`

**Examples:**

Launch a specific world:
```bash
ros2 launch robotnik_webots spawn_world.launch.py world:=my_custom_world
```

Launch headless (no GUI):
```bash
ros2 launch robotnik_webots spawn_world.launch.py gui:=false
```

### 2. Spawn a robot in Webots

```bash
ros2 launch robotnik_webots spawn_robot.launch.py
```

#### Arguments for `spawn_robot.launch.py`

- `robot_id`: Robot personal name. Default: `robot`
- `robot`: Robot model (e.g., `rbwatcher`, `rbvogui`, `rbkairos`, `rbtheron`, `rbsummit`). Default: `rbwatcher`
- `robot_model`: Robot type variation. Default: value of `robot`
- `x`, `y`, `z`: Initial position in the world. Default: `0.0`
- `run_rviz`: Launch RViz2 (`true`/`false`). Default: `false`

**Examples:**

Spawn a robot at a specific position:
```bash
ros2 launch robotnik_webots spawn_robot.launch.py x:=2.0 y:=1.0 z:=0.0
```

Spawn a robot with a custom name and model:
```bash
ros2 launch robotnik_webots spawn_robot.launch.py robot:=rbkairos robot_id:=rbkairos1
```

Spawn a robot and launch RViz2:
```bash
ros2 launch robotnik_webots spawn_robot.launch.py run_rviz:=true
```

Example of cmd_vel command to interact with robot:
```bash
ros2 topic pub /rbkairos1/diffdrive_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist:{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 2.0}}}" -r 500
```

## Known issues
 - Webots simulator in versions R2025 seem to have issues with compatibility with current versions of proto files. Therefore we recommend using older version R2023b.

## License
See the LICENSE file for details.
