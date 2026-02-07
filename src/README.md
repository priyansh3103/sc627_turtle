# Gauntlet Gazebo Package

A ROS2 Gazebo simulation package for TurtleBot3 in the Gauntlet world environment.

## Description

This package provides a custom Gazebo world (Gauntlet) with TurtleBot3 simulation support. It includes all necessary launch files and configurations to spawn a TurtleBot3 robot with full sensor support including:

- `/scan` - LiDAR laser scan data
- `/imu` - IMU sensor data
- `/odom` - Odometry data
- `/cmd_vel` - Velocity commands (input)
- `/tf` - Transform data
- `/joint_states` - Joint state data
- `/clock` - Simulation clock

## Prerequisites

- ROS2 (Humble/Iron/Jazzy)
- Gazebo Sim (Ignition/Gazebo)
- turtlebot3_gazebo package
- ros_gz_sim, ros_gz_bridge packages

## Environment Setup

Before launching, set the TurtleBot3 model:

```bash
export TURTLEBOT3_MODEL=burger  # or waffle, waffle_pi
```

## Building

```bash
cd ~/your_ws
colcon build --packages-select gauntlet_gazebo
source install/setup.bash
```

## Usage

### Launch the Gauntlet World with TurtleBot3

```bash
ros2 launch gauntlet_gazebo gauntlet_world.launch.py
```

### With Custom Robot Position

```bash
ros2 launch gauntlet_gazebo gauntlet_world.launch.py x_pose:=-1.0 y_pose:=0.5
```

## World Description

The Gauntlet world is a 3m x 3m enclosed arena with:

- **Outer walls** (gray) - Boundary walls
- **L-shaped structure** (green) - Complex obstacle in the center
- **Side block** (blue) - Obstacle on the left side
- **Top blocks** (red) - Two obstacles near the top

## Available Topics

After launching, these topics will be available:

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | sensor_msgs/msg/LaserScan | LiDAR scan data |
| `/imu` | sensor_msgs/msg/Imu | IMU sensor data |
| `/odom` | nav_msgs/msg/Odometry | Robot odometry |
| `/cmd_vel` | geometry_msgs/msg/TwistStamped | Velocity commands |
| `/tf` | tf2_msgs/msg/TFMessage | Transform tree |
| `/joint_states` | sensor_msgs/msg/JointState | Joint states |
| `/clock` | rosgraph_msgs/msg/Clock | Simulation clock |

## Teleoperation

To control the robot:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

## License

Apache 2.0
