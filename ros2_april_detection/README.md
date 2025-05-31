# ROS2 AprilTag Detection Package

This package provides AprilTag detection and autonomous obstacle avoidance capabilities for the RB5 robotic car. It uses computer vision to detect AprilTag markers and implements reactive navigation to avoid obstacles.

## Package Overview

The `ros2_april_detection` package contains:
- **AprilTag Detection**: Real-time fiducial marker detection using camera input
- **Obstacle Avoidance**: Autonomous navigation logic based on detected tags
- **Transform Publishing**: TF broadcasts for detected marker poses
- **Robot Control**: Direct cmd_vel publishing for movement commands

## Prerequisites

```bash
# Install AprilTag library
sudo apt install ros-foxy-apriltag-ros libapriltag-dev

# Install required ROS2 packages
sudo apt install ros-foxy-cv-bridge ros-foxy-tf2-geometry-msgs

# Ensure camera package is available
source /opt/ros/foxy/setup.bash
```

## Basic Commands

### 1. Build the Package

```bash
# Navigate to your workspace root
cd ~/ros2_ws

# Build the package
colcon build --packages-select ros2_april_detection
source install/setup.bash
```

### 2. Launch Complete AprilTag System

```bash
# Terminal 1: Launch camera system
ros2 launch rb5_ros2_vision rb_camera_main_ocv_launch.py

# Terminal 2: Publish /april_poses and /tf
ros2 run ros2_april_detection april_detection_node

# Terminal 3: Subscribe to /april_poses and /tf, publish /cmd_vel
ros2 run ros2_april_detection april_obstacle_avoidance
```

### 4. Monitor System Output

```bash
# Check AprilTag pose detections
ros2 topic echo /april_poses

# Monitor transform broadcasts
ros2 topic echo /tf

# View robot movement commands
ros2 topic echo /cmd_vel
```

### 5. Test Robot Movement

```bash
# Run subscriber to execute movement commands
ros2 run rb5_ros2_control subscriber_node.py
```

## Troubleshooting

### Camera Issues

If camera has error, run this to free up the video device:

```bash
# Check which process is using the camera
fuser /dev/video0

# Kill the process (replace <process_number> with actual PID)
kill <process_number>
```