#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class SequenceController(Node):
    def __init__(self):
        super().__init__('sequence_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # 100 ms
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.step = 0
        self.counter = 0

    def timer_callback(self):
        msg = Twist()

        if self.step == 0:
            # Move forward for 4  seconds
            msg.linear.x = 0.4
            self.counter += 1
            if self.counter >= 40:  # 0.1s * 50 = 5s
                self.step += 1
                self.counter = 0

        elif self.step == 1:
            # Turn right 45° (approx 0.5 rad/s for 1.57s)
            msg.angular.z = -0.5
            msg.linear.x = 0.2
            self.counter += 1
            if self.counter >= 16:
                self.step += 1
                self.counter = 0

        elif self.step == 2:
            # Turn left 45°
            msg.angular.z = 0.5
            self.counter += 1
            if self.counter >= 16:
                self.step += 1
                self.counter = 0

        elif self.step == 3:
            # Move backward for 4 seconds
            msg.linear.x = -0.4
            self.counter += 1
            if self.counter >= 40:
                self.step += 1
                self.counter = 0

        elif self.step == 4:
            # Pause (stop) for 2 seconds
            self.counter += 1
            if self.counter >= 20:
                self.get_logger().info('Sequence complete. Stopping.')
                rclpy.shutdown()
                return

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SequenceController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

