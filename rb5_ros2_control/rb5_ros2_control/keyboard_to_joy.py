#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import sys
import termios
import tty

class KeyboardToJoy(Node):
    def __init__(self):
        super().__init__('keyboard_to_joy')
        self.publisher_ = self.create_publisher(Joy, 'joy', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info('Keyboard Teleop Started: Use W/A/S/D to move, Q/E to rotate, SPACE to stop.')

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def loop(self):
        try:
            while rclpy.ok():
                key = self.get_key()

                joy_msg = Joy()
                joy_msg.axes = [0.0, 0.0, 0.0]  # [left/right, forward/backward, rotation]
                joy_msg.buttons = []

                if key == 'w':
                    joy_msg.axes[1] = 1.0  # forward
                elif key == 's':
                    joy_msg.axes[1] = -1.0  # backward
                elif key == 'a':
                    joy_msg.axes[0] = 1.0  # left
                elif key == 'd':
                    joy_msg.axes[0] = -1.0  # right
                elif key == 'q':
                    joy_msg.axes[2] = 1.0  # counterclockwise
                elif key == 'e':
                    joy_msg.axes[2] = -1.0  # clockwise
                elif key == ' ':
                    # Stop
                    joy_msg.axes = [0.0, 0.0, 0.0]
                else:
                    continue  # Ignore unknown keys

                self.publisher_.publish(joy_msg)
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardToJoy()
    node.loop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

