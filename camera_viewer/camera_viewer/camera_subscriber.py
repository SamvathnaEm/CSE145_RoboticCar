import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera_0',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        try:
            # Assuming encoding is 'rgb8' or 'bgr8'
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))

            # Show the image using OpenCV
            cv2.imshow('Camera Feed', img)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Image decode failed: {e}")

def main():
    rclpy.init()
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
