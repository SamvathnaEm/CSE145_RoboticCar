#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class OpenLoopControl(Node):
    def __init__(self):
        super().__init__('open_loop_control')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.2  # Move forward slowly
        msg.angular.z = 0.0 # No turning
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing open-loop forward command.')

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

