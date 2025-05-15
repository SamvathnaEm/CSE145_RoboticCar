# example.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_april_detection',
            executable='april_detection_node',
            name='april_detection_node',
            output='screen',
            parameters=[
                {'use_sim_time': False}  # Set to True if using simulation
            ]
        )
    ])