# RTAB-Map Odometry Package

This package provides SLAM (Simultaneous Localization and Mapping) capabilities for the RB5 robotic car using RTAB-Map and ORB-SLAM3 integration. It includes camera info publishing, transform handling, and complete mapping pipeline setup.

## Package Overview

The `my_rtabmap_odom` package contains:
- **Camera Info Publisher**: Publishes camera calibration information for SLAM
- **Transform Handling**: TF to odometry conversion for proper frame relationships
- **RTAB-Map Integration**: Complete SLAM pipeline with visual odometry
- **ORB-SLAM3 Support**: Integration with ORB-SLAM3 for enhanced visual odometry
- **Launch Files**: Comprehensive launch configurations for different mapping modes

## Prerequisites

### Install RTAB-Map (Binary Version)
```bash
sudo apt update
sudo apt install ros-foxy-rtabmap ros-foxy-rtabmap-ros
```

### Install ORB-SLAM3 for RB5

Follow the tutorial: [Setup ORB-SLAM3 on RB5](https://autonomousvehiclelaboratory.github.io/RB5_Robotics_Tutorials/2022/05/18/3%20Robotics%20Applications/setup-orb-slam3-on-rb5/)

#### Step-by-Step ORB-SLAM3 Installation

1. **Clone ORB-SLAM3 for RB5**
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/AutonomousVehicleLaboratory/ORB_SLAM3_RB5
   ```

2. **Modify build.sh (Critical Step)**
   ```bash
   cd ORB_SLAM3_RB5
   nano build.sh
   # Comment out the Sophus compilation section in build.sh
   ```

3. **Install Sophus via apt**
   ```bash
   sudo apt install ros-foxy-sophus
   ```

4. **Build ORB-SLAM3**
   ```bash
   ./build.sh
   ```

5. **Clone RB5 ROS2 Wrapper**
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/AutonomousVehicleLaboratory/rb5_ros2
   ```

6. **Setup Environment Variables**
   ```bash
   echo 'export LD_LIBRARY_PATH=/root/ros2_ws/src/ORB_SLAM3_RB5/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
   source ~/.bashrc
   ```

7. **Build ROS2 Workspace**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select orb_slam3_rb5_ros2
   source install/setup.bash
   ```

## Basic Commands

### 1. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select my_rtabmap_odom
source install/setup.bash
```

### 2. Complete SLAM Pipeline

Launch the complete system in the following order:

#### Terminal 1: Camera System
```bash
# Launch camera feed (/camera_0)
ros2 launch rb5_ros2_vision rb_camera_main_ocv_launch.py
```

#### Terminal 2: IMU Data (Optional)
```bash
# Launch IMU data (/imu)
ros2 run imu-ros2node imu-ros2node
```

#### Terminal 3: ORB-SLAM3 Visual Odometry
```bash
# Launch ORB-SLAM3 for camera pose and transforms (/camera_pose & /tf)
ros2 run orb_slam3_rb5_ros2 Mono \
  /root/ros2_ws/src/ORB_SLAM3_RB5/Vocabulary/ORBvoc.txt \
  /root/ros2_ws/src/ORB_SLAM3_RB5/Examples/Monocular/EuRoC.yaml
```

#### Terminal 4: Camera Info and Odometry
```bash
# Launch camera info publisher and odometry conversion (/camera_0/camera_info & /odom)
ros2 launch my_rtabmap_odom full_mapping.launch.py
```

#### Terminal 5: RTAB-Map SLAM
```bash
# Launch RTAB-Map with ORB-SLAM3 integration (/rtabmap)
ros2 launch rtabmap_launch rtabmap.launch.py \
  use_odom:=true \
  odom_topic:=/odom \
  visual_odometry:=false \
  subscribe_rgb:=true \
  rgb_topic:=/camera_0 \
  camera_info_topic:=/camera_0/camera_info \
  depth:=false \
  approx_sync:=true
```

### 3. Alternative Launch Configurations

#### Basic Camera Setup Only
```bash
# Launch just camera info and image processing
ros2 launch my_rtabmap_odom camera_mapping.launch.py
```

#### RTAB-Map Odometry Only
```bash
# Launch RTAB-Map odometry without full SLAM
ros2 launch my_rtabmap_odom rtabmap_odom.launch.py
```

#### Transform Chain Setup
```bash
# Launch TF to odometry conversion
ros2 launch my_rtabmap_odom tf_to_odom_launch.py
```

## System Architecture

### Data Flow
```
Camera → ORB-SLAM3 → Transform Publisher → RTAB-Map → Map Database
   ↓         ↓             ↓                ↓
/camera_0  /camera_pose   /tf            /rtabmap
           /trajectory    /odom          /map
```

### Topics Generated
- `/camera_0` - Raw camera feed
- `/camera_0/camera_info` - Camera calibration parameters
- `/camera_pose` - ORB-SLAM3 camera poses
- `/tf` - Transform tree (camera poses, odometry)
- `/odom` - Odometry messages converted from TF
- `/rtabmap/map` - Generated map data
- `/rtabmap/trajectory` - Robot trajectory
```

### Transform Chain Configuration
The system establishes the following transform chain:
- `odom` → `base_link` (robot position in odometry frame)
- `base_link` → `camera_frame` (camera position on robot)
- `camera_frame` → `camera_optical_frame` (standard camera orientation)

Edit static transforms in [`full_mapping.launch.py`](launch/full_mapping.launch.py) as needed.