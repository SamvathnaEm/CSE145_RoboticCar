from setuptools import setup

package_name = 'rb5_ros2_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS2 control for RB5',
    license='MIT',
    entry_points={
        'console_scripts': [
            'keyboard_to_joy = rb5_ros2_control.keyboard_to_joy:main',
        ],
    },
)
