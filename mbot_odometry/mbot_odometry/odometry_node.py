#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
import math
from megapi import MegaPi
from math import pi, sin, cos
import time

# === Global variables for encoder readings ===
left_ticks_global = 0
right_ticks_global = 0

# === Global callback functions (not class methods) ===
def left_encoder_callback(pos):
    global left_ticks_global
    left_ticks_global = pos

def right_encoder_callback(pos):
    global right_ticks_global
    right_ticks_global = pos

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # Initialize MegaPi controller
        self.bot = MegaPi()
        self.bot.start('/dev/ttyUSB0')

        # Motor ports
        self.MFR = 2
        self.MBL = 3
        self.MBR = 10
        self.MFL = 11

        # Publisher
        self.odom_pub = self.create_publisher(PoseWithCovarianceStamped, '/odom', 10)
        self.get_logger().info("Odometry Publisher Started")

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

        # Start with motors off
        self.set_four_motors(0, 0, 0, 0)

        # Publish every 0.1 seconds
        self.timer = self.create_timer(0.1, self.publish_odometry)

    def set_four_motors(self, vfl=0, vfr=0, vbl=0, vbr=0):
        self.bot.motorRun(self.MFL, -vfl)
        self.bot.motorRun(self.MFR, vfr)
        self.bot.motorRun(self.MBL, -vbl)
        self.bot.motorRun(self.MBR, vbr)

    def get_encoder_ticks(self):
        global left_ticks_global, right_ticks_global
        self.bot.encoderMotorPosition(self.MFL, left_encoder_callback)
        self.bot.encoderMotorPosition(self.MFR, right_encoder_callback)
        time.sleep(0.1)  # Wait for encoder data
        print(f"Left ticks: {left_ticks_global}, Right ticks: {right_ticks_global}")
        return left_ticks_global, right_ticks_global

    def update_odometry(self, left_ticks, right_ticks):
        WHEEL_RADIUS = 0.03  # meters
        TICKS_PER_REV = 360  # adjust based on motor
        WHEEL_BASE = 0.1     # distance between wheels

        delta_left_ticks = left_ticks - self.prev_left_ticks
        delta_right_ticks = right_ticks - self.prev_right_ticks
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

        left_distance = (delta_left_ticks / TICKS_PER_REV) * (2 * pi * WHEEL_RADIUS)
        right_distance = (delta_right_ticks / TICKS_PER_REV) * (2 * pi * WHEEL_RADIUS)

        distance = (left_distance + right_distance) / 2.0
        dtheta = (right_distance - left_distance) / WHEEL_BASE

        self.theta += dtheta
        self.x += distance * cos(self.theta)
        self.y += distance * sin(self.theta)

    def publish_odometry(self):
        try:
            left_ticks, right_ticks = self.get_encoder_ticks()
            self.update_odometry(left_ticks, right_ticks)

            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            msg.pose.pose.position.x = self.x
            msg.pose.pose.position.y = self.y
            
            # Compute quaternion for orientation
            q = Quaternion()
            q.x = 0.0
            q.y = 0.0
            q.z = math.sin(self.theta / 2.0)
            q.w = math.cos(self.theta / 2.0)
            msg.pose.pose.orientation = q

            self.odom_pub.publish(msg)
            self.get_logger().info(f"Published odometry: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}")
        except Exception as e:
            self.get_logger().error(f"Failed to read encoder: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()