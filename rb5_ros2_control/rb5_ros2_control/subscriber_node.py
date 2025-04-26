#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

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

    def listener_callback(self, msg):
        # Print the received message's linear and angular velocities
        self.get_logger().info('Received: Linear X = %f, Angular Z = %f' % (msg.linear.x, msg.angular.z))


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

