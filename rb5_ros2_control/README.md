# RB5 ROS2 Control Package

This package provides comprehensive control capabilities for the RB5 robotic car, including web-based control interface, keyboard teleoperation, and motor control integration.

## Package Overview

The `rb5_ros2_control` package contains:
- **Web Interface**: Flask-based web server with responsive control interface
- **Motor Control**: Integration with mBot Mega platform via serial communication
- **Keyboard Control**: Direct keyboard teleoperation support
- **ROS2 Integration**: Bridges web/keyboard inputs to ROS2 topics

## Prerequisites

```bash
# Install required Python packages
pip3 install flask megapi rclpy

# Ensure ROS2 Foxy is installed and sourced
source /opt/ros/foxy/setup.bash
```

## Basic Commands

### 1. Build the Package

```bash
# Navigate to your workspace root
cd ~/ros2_ws

# Build the package
colcon build --packages-select rb5_ros2_control
source install/setup.bash
```

### 2. Launch Web Interface Control

```bash
# Terminal 1: Start the Flask web server with ROS2 bridge
ros2 run rb5_ros2_control combined_flask_server.py

# Terminal 2: Start the robot control node
ros2 run rb5_ros2_control rb5_keyjoy_control_node.py
```

**Access the web interface at:** `http://<robot-ip>:8004`

### 3. Keyboard Control

```bash
# Terminal 1: Enable keyboard input control
ros2 run rb5_ros2_control keyboard_to_joy.py

# Terminal 2: Subscribe to /cmd_vel and execute robot movement
ros2 run rb5_ros2_control subscriber_node.py
```

### 4. Open Loop Testing

```bash
# Terminal 1: Generate test movement commands
ros2 run rb5_ros2_control open_loop_node.py

# Terminal 2: Execute movement commands via /cmd_vel subscription
ros2 run rb5_ros2_control subscriber_node.py
```

### 5. Motor Control Testing

```bash
# Direct motor control testing (adjust port as needed)
cd rb5_ros2_control/
python3 mpi_control.py
```