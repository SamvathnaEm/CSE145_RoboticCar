#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import tf2_ros
from rclpy.time import Time
import numpy as np
from sensor_msgs.msg import Imu

class TfToOdomNode(Node):
    def __init__(self):
        super().__init__('tf_to_odom_node')

        # Subscriptions
        self.tf_sub = self.create_subscription(
            TFMessage,  # Correct message type for /tf
            '/tf',
            self.tf_callback,
            10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/camera_pose',
            self.pose_callback,
            10
        )
        self.imu_sub = self.create_subscription(
            Imu,  # Correct message type for IMU data
            '/imu',
            self.imu_callback,
            10
        )

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Internal state
        self.latest_tf = None
        self.latest_pose = None
        self.latest_imu = None
        self.prev_time = None
        self.prev_position = None
        
        # Timer for publishing odometry regularly
        self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('TF to Odom Node initialized')

    def tf_callback(self, msg):
        # Extract the transform we're interested in
        for transform in msg.transforms:
            if transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_link':
                self.latest_tf = transform
                self.get_logger().debug('Received tf transform')
                break

    def pose_callback(self, msg):
        self.latest_pose = msg
        self.get_logger().debug('Received pose message')

    def imu_callback(self, msg):
        self.latest_imu = msg
        self.get_logger().debug('Received IMU message')

    def timer_callback(self):
        self.publish_odom()

    def publish_odom(self):
        current_time = self.get_clock().now()
        
        # Create a new odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        if self.latest_pose:
            # Use the camera pose for position
            odom_msg.pose.pose = self.latest_pose.pose
            
            # Calculate velocity if we have previous position
            if self.prev_position and self.prev_time:
                dt = (current_time - self.prev_time).nanoseconds / 1e9
                if dt > 0:
                    dx = odom_msg.pose.pose.position.x - self.prev_position.x
                    dy = odom_msg.pose.pose.position.y - self.prev_position.y
                    dz = odom_msg.pose.pose.position.z - self.prev_position.z
                    
                    # Simple velocity estimation
                    odom_msg.twist.twist.linear.x = dx / dt
                    odom_msg.twist.twist.linear.y = dy / dt
                    odom_msg.twist.twist.linear.z = dz / dt
        
        # Use IMU data for orientation and angular velocity if available
        if self.latest_imu:
            odom_msg.pose.pose.orientation = self.latest_imu.orientation
            odom_msg.twist.twist.angular = self.latest_imu.angular_velocity
        
        # Publish the odometry message
        self.odom_pub.publish(odom_msg)
        self.get_logger().info('Published /odom message')
        
        # Store current position and time for next velocity calculation
        if self.latest_pose:
            self.prev_position = self.latest_pose.pose.position
            self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = TfToOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()