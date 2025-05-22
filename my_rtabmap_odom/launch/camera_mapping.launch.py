from launch import LaunchDescription
from launch_ros.actions import Node

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
        )
    ])