#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mpi_control import MegaPiController  # Import MegaPiController

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        # Create a subscriber to the topic '/cmd_vel'
        self.subscription = self.create_subscription(
            Twist,  # Message type
            '/cmd_vel',  # Topic to subscribe to
            self.listener_callback,  # Callback function when data is received
            10  # QoS (Quality of Service) profile - most common is 10
        )
        self.motor_controller = MegaPiController()  # Initialize the motor controller

    def listener_callback(self, msg):
        # Print the received message's linear and angular velocities
        self.get_logger().info('Received: Linear X = %f, Angular Z = %f' % (msg.linear.x, msg.angular.z))

        # Now send these values to the motor control system
        self.send_motor_commands(msg.linear.x, msg.angular.z)

    def send_motor_commands(self, linear_x, angular_z):
        # Convert the linear and angular velocities into motor commands.
        # For simplicity, assuming linear_x controls the speed, and angular_z controls the rotation.

        # Speed scaling factors (you can tune them based on your needs)
        max_speed = 100  # maximum motor speed (adjustable)
        rotation_speed = angular_z * max_speed  # Conversion for angular velocity

        # Forward movement (linear_x) and rotation (angular_z)
        speed_left = linear_x * max_speed - rotation_speed
        speed_right = linear_x * max_speed + rotation_speed
        # Ensure the speed values are integers
        speed_left = int(round(speed_left))
        speed_right = int(round(speed_right))

        # Send the motor commands using MegaPiController's method
        self.motor_controller.setFourMotors(speed_left, speed_right, speed_left, speed_right)

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    # Spin the node so it can listen for messages
    rclpy.spin(node)
    # Shutdown the node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

