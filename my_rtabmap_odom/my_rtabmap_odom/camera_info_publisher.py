#!/usr/bin/env python3
# filepath: /root/ros2_ws/src/CSE145_RoboticCar/my_rtabmap_odom/my_rtabmap_odom/camera_info_publisher.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        
        # Subscribe to the image to get image dimensions and timing
        self.image_sub = self.create_subscription(
            Image,
            '/camera_0',
            self.image_callback,
            10
        )
        
        # Create a publisher for camera info
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '/camera_0/camera_info',
            10
        )
        
        # Initialize camera info message with default values
        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = 'camera_optical_frame'
        
        # Default values (will be updated when first image arrives)
        self.camera_info.height = 480
        self.camera_info.width = 640
        
        # Default calibration values for a generic camera
        # These should be replaced with actual calibration data if available
        fx = 500.0  # Focal length x
        fy = 500.0  # Focal length y
        cx = 320.0  # Principal point x (width/2)
        cy = 240.0  # Principal point y (height/2)
        
        # Set K matrix (camera intrinsics)
        self.camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        
        # Set distortion parameters (no distortion by default)
        self.camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Set projection matrix P
        self.camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        # Set rectification matrix R (identity for a non-stereo camera)
        self.camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        self.get_logger().info('Camera info publisher initialized')
        
    def image_callback(self, msg):
        # Update camera info dimensions based on image
        if self.camera_info.width != msg.width or self.camera_info.height != msg.height:
            self.camera_info.width = msg.width
            self.camera_info.height = msg.height
            
            # Update camera info based on new dimensions
            cx = msg.width / 2
            cy = msg.height / 2
            
            # Estimate focal length based on typical FOV for webcams (60-70 degrees)
            # This is a rough estimation; real calibration would be better
            fx = msg.width / (2.0 * np.tan(np.radians(65) / 2.0))
            fy = fx  # Assume square pixels
            
            # Update K matrix
            self.camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            
            # Update P matrix
            self.camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            
            self.get_logger().info(f'Updated camera info with dimensions: {msg.width}x{msg.height}')
        
        # Update timestamp and publish camera info synchronized with image
        self.camera_info.header.stamp = msg.header.stamp
        self.camera_info_pub.publish(self.camera_info)

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()