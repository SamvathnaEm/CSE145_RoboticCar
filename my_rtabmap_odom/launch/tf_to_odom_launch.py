#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_rtabmap_odom',
            executable='tf_to_odom_node.py',
            name='tf_to_odom_node',
            output='screen'
        )
    ])