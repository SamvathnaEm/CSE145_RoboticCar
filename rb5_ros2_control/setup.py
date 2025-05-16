# from setuptools import setup, find_packages

# setup(
#     name='rb5_ros2_control',
#     version='0.1.0',
#     packages=find_packages(),
# )

from setuptools import setup

package_name = 'rb5_ros2_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='RB5 ROS2 control package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'package_practice2 = rb5_ros2_control.package_practice2:main',
        ],
    },
)
