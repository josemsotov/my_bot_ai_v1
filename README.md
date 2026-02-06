# SMART-TROLLEY V4 AI v1 - ROS 2 Workspace

This workspace contains a ROS 2 Jazzy + Gazebo Sim (Harmonic) package for the SMART-TROLLEY V4 differential drive robot.

## Overview

**Workspace**: `ws_smart_trolley_ai_v1`
**Package**: `smart_t_ai_v1`
**Robot Version**: SMART-TROLLEY V4 (Differential drive with two wheels + rear caster ball)
**ROS2 Distribution**: Jazzy
**Gazebo**: Sim Harmonic

## Features

This workspace combines:
- **Robot Structure**: Modern ROS 2 layout from `dev_ws_js` (with Python launch files, modular URDF/Xacro)
- **Robot Dimensions & Physics**: SMART-TROLLEY V4 parameters (wheel positions, inertias, camera position, meshes)
- **Sensors**:
  - RGB Camera (640x480, 10 Hz)
  - GPU LiDAR (360 samples, 10 Hz)
- **Control**: Differential drive with `/cmd_vel` topic (ROS 2 to Gazebo Sim via `ros_gz_bridge`)

## Directory Structure

```
ws_smart_trolley_ai_v1/
└── src/
    └── smart_t_ai_v1/
        ├── config/              # RViz configuration files
        │   ├── smart_trolley_drive.rviz
        │   └── view_bot.rviz
        ├── description/         # URDF/Xacro robot definitions
        │   ├── robot.urdf.xacro (wrapper)
        │   ├── SMART-TROLLEY_V4_ai.xacro (main robot with V4 parameters)
        │   ├── gazebo_control_trolley.xacro (DiffDrive plugin)
        │   ├── camera.xacro
        │   ├── lidar.xacro
        │   ├── materials.xacro
        │   └── ...
        ├── launch/              # Python launch scripts
        │   ├── launch_sim.launch.py (Gazebo Sim + bridges)
        │   ├── rsp.launch.py (Robot State Publisher)
        │   └── rviz.launch.py (RViz visualization)
        ├── meshes/              # Robot STL mesh files (from V4)
        │   ├── base_link.stl
        │   ├── rueda_link_1.stl
        │   ├── rueda_link_2.stl
        │   ├── camera_link_1.stl
        │   └── RR-RUEDA_link_1.stl (rear caster)
        ├── worlds/              # Gazebo simulation worlds
        │   └── empty.sdf
        ├── CMakeLists.txt
        ├── package.xml
        └── README.md (this file)
```

## Prerequisites

- ROS 2 Jazzy
- Gazebo Sim Harmonic
- Dependencies: `robot_state_publisher`, `xacro`, `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image`

## Build Instructions

```bash
# Navigate to workspace root
cd ~/ws_smart_trolley_ai_v1

# Install dependencies (if needed)
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build

# Source the environment
source install/setup.bash
```

## Usage

### 1. Launch Robot State Publisher Only (for visualization without simulation)

```bash
source install/setup.bash
ros2 launch smart_t_ai_v1 rsp.launch.py
```

Then in another terminal, open RViz:
```bash
rviz2
# Load configuration from config/view_bot.rviz
```

### 2. Launch Full Simulation with Gazebo Sim

```bash
source install/setup.bash
ros2 launch smart_t_ai_v1 launch_sim.launch.py
```

This will:
- Start Gazebo Sim with an empty world
- Spawn the SMART-TROLLEY robot
- Launch Robot State Publisher
- Create ROS ↔ Gazebo bridges for:
  - `/cmd_vel` (velocity commands)
  - `/odom` (odometry feedback)
  - `/scan` (LiDAR data)
  - `/camera` (camera images)
  - `/joint_states` (robot joint states)

### 3. Launch with RViz Visualization

In parallel with the simulation:
```bash
source install/setup.bash
ros2 launch smart_t_ai_v1 rviz.launch.py
```

## Control the Robot

Once the simulation is running, send velocity commands:

```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.2}, angular: {z: 0.0}"

# Rotate in place
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0}, angular: {z: 0.5}"

# Move forward and turn
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.2}, angular: {z: 0.3}"
```

## Available Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Input | Robot velocity command |
| `/odom` | nav_msgs/Odometry | Output | Odometry feedback from DiffDrive plugin |
| `/scan` | sensor_msgs/LaserScan | Output | LiDAR point cloud data (360 samples, 12m range) |
| `/camera` | sensor_msgs/Image | Output | RGB camera images (640x480, 10 Hz) |
| `/joint_states` | sensor_msgs/JointState | Output | Robot joint positions/velocities |
| `/tf` | tf2_msgs/TFMessage | Output | Transform tree (odom → base_link) |

## Robot Parameters

From SMART-TROLLEY V4 specifications:
- **Wheel Separation**: 0.63 m (distance between left and right wheels)
- **Wheel Radius**: ~0.10 m
- **Total Mass**: ~18 kg (chassis + wheels + sensors)
- **Wheel Positions**: ±0.32 m (Y-axis) from center, 0.1 m behind chassis center
- **Camera Position**: Camera link mesh at center-top of chassis
- **Rear Caster**: Passive sphere wheel at back for stability

## Differences from Original Versions

- **vs. dev_ws_js**: Same structure but with SMART-TROLLEY V4 dimensions, inertias, and mesh files
- **vs. ws_smart_trolley**: Modern ROS 2 Jazzy + Gazebo Sim instead of ROS 1 + Gazebo Classic

## Gazebo Plugins Used

- `gz-sim-physics-system` - Physics simulation
- `gz-sim-diff-drive-system` - Differential drive controller
- `gz-sim-sensors-system` - Sensor simulation
- `gz-sim-joint-state-publisher-system` - Joint state publishing
- `gz-sim-scene-broadcaster-system` - Scene state publishing

## Common Issues

1. **Package Not Found**: Make sure to build the workspace and source the setup file:
   ```bash
   colcon build
   source install/setup.bash
   ```

2. **Gazebo Not Starting**: Ensure Gazebo Sim Harmonic is installed:
   ```bash
   sudo apt install gz-sim
   ```

3. **Camera/LiDAR Not Publishing**: Check that bridges are running in the simulation launch terminal

## Further Development

This workspace is v1 and provides a solid foundation for:
- Path planning algorithms
- Vision-based navigation
- SLAM implementation
- Deep learning inference on robot camera
- ROS 2 control stack integration

## Author

Jose Soto Villasmil (josemsotov@gmail.com)

## License

Apache-2.0
