import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class TestSubscriber(Node):
    def __init__(self):
        super().__init__('test_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera_0',
            self.listener_callback,
            10)
        print("✅ Subscriber initialized and listening to /camera_0")

    def listener_callback(self, msg):
        print(f"✅ Got image {msg.width}x{msg.height}, encoding={msg.encoding}")

def main():
    rclpy.init()
    node = TestSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
