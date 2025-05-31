import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
from flask import Flask, Response
import threading
import time

app = Flask(__name__)
frame_lock = threading.Lock()
shared_frame = None

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera_0',
            self.listener_callback,
            10
        )
        self.get_logger().info("✅ Subscribed to /camera_0")

    def listener_callback(self, msg):
        global shared_frame
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))

            # ⚠️ Ensure correct color format
            if msg.encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            # ⚡ Resize to avoid overload
            img = cv2.resize(img, (640, 360))

            ret, jpeg = cv2.imencode('.jpg', img)
            if ret:
                with frame_lock:
                    shared_frame = jpeg.tobytes()
                print(f"✅ Got image {msg.width}x{msg.height}, encoding={msg.encoding}")
        except Exception as e:
            self.get_logger().error(f"❌ Image decode failed: {e}")

def ros_thread():
    rclpy.init()
    node = CameraSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

def generate_stream():
    while True:
        with frame_lock:
            if shared_frame is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + shared_frame + b'\r\n')
        time.sleep(0.05)  # ✅ prevent high CPU usage

@app.route('/')
def index():
    return '''
    <h1>RB5 Live Stream</h1>
    <img src="/video_feed" width="640" height="360"/>
    '''

@app.route('/video_feed')
def video_feed():
    print("📥 /video_feed requested")
    return Response(generate_stream(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def main():
    threading.Thread(target=ros_thread, daemon=True).start()
    app.run(host='0.0.0.0', port=8007, threaded=True)

if __name__ == '__main__':
    main()
