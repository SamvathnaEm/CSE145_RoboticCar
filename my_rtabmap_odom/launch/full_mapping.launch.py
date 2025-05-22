#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Camera info publisher
        Node(
            package='my_rtabmap_odom',
            executable='camera_info_publisher.py',
            name='camera_info_publisher',
            output='screen'
        ),
        
        # Image processor node
        Node(
            package='image_proc',
            executable='image_proc',
            name='image_proc',
            parameters=[{'use_sim_time': False}],
            remappings=[
                ('image', '/camera_0'),
                ('camera_info', '/camera_0/camera_info')
            ],
            output='screen'
        ),
        
        # TF to Odom node 
        Node(
            package='my_rtabmap_odom',
            executable='tf_to_odom_node.py',
            name='tf_to_odom_node',
            output='screen'
        ),
        
        # Add static TF transforms for the frames RTAB-Map is looking for
        
        # 1. odom → base_link transform (robot's position in odometry frame)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen'
        ),
        
        # 2. base_link → camera_frame transform (camera position on robot)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera_frame',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'camera_frame'],
            output='screen'
        ),
        
        # 3. camera_frame → camera_optical_frame transform (standard camera orientation adjustment)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_frame_to_optical',
            # 90 degree rotation around X to get to optical frame orientation
            arguments=['0', '0', '0', '-0.5', '0.5', '-0.5', '0.5', 'camera_frame', 'camera_optical_frame'],
            output='screen'
        ),
        
        # RTAB-Map with correct parameters
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'qos': '1',
                'localization': 'false',
                'use_odom': 'true',
                'odom_topic': '/odom',
                'visual_odometry': 'false',
                'subscribe_rgb': 'true',
                'rgb_topic': '/camera_0',
                'camera_info_topic': '/camera_0/camera_info',
                'depth': 'false',
                'approx_sync': 'true',
                'approx_sync_max_interval': '0.1',
                'queue_size': '20',
                'frame_id': 'base_link',
                'wait_for_transform': 'true',
                'wait_for_transform_duration': '0.2',
                'publish_tf': 'true'
            }.items()
        )
    ])