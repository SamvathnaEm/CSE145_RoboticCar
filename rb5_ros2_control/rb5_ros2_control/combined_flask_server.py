from flask import Flask, Response, request, render_template
import threading
import time
import cv2
import numpy as np
import rclpy
import subprocess
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy
import os
import signal

app = Flask(__name__)
frame_lock = threading.Lock()
shared_frame = None

auto_processes = []

# ROS2 Node combining camera subscriber and joy publisher
class ROSNode(Node):
    def __init__(self):
        super().__init__('combined_flask_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera_0',
            self.camera_callback,
            10
        )
        self.publisher = self.create_publisher(Joy, 'joy', 10)
        self.get_logger().info("✅ Subscribed to /camera_0 and ready to publish /joy")

    def camera_callback(self, msg):
        global shared_frame
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            if msg.encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            img = cv2.resize(img, (640, 360))
            ret, jpeg = cv2.imencode('.jpg', img)
            if ret:
                with frame_lock:
                    shared_frame = jpeg.tobytes()
        except Exception as e:
            self.get_logger().error(f"❌ Image decode failed: {e}")

    def publish_joy(self, direction):
        joy_msg = Joy()
        joy_msg.axes = [0.0, 0.0, 0.0]
        speed = 2.0
        if direction == 'forward':
            joy_msg.axes[1] = speed
        elif direction == 'backward':
            joy_msg.axes[1] = -speed
        elif direction == 'left':
            joy_msg.axes[0] = speed
        elif direction == 'right':
            joy_msg.axes[0] = -speed
        elif direction == 'cwise':
            joy_msg.axes[2] = -speed
        elif direction == 'ccwise':
            joy_msg.axes[2] = speed
        elif direction == 'stop':
            joy_msg.axes = [0.0, 0.0, 0.0]
        self.publisher.publish(joy_msg)
        self.get_logger().info(f"📤 Published move: {direction}")

# Global ROS2 node
ros_node = None

# Flask endpoints
@app.route('/')
def home():
    return render_template('index.html')

@app.route('/move', methods=['POST'])
def move():
    direction = request.form.get('direction')
    if ros_node:
        ros_node.publish_joy(direction)
    return f'Moved {direction}'

@app.route('/video_feed')
def video_feed():
    return Response(generate_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/auto_april', methods=['POST'])
def auto_april():
    global auto_processes
    try:
        # Kill existing processes first (optional safety)
        auto_stop()

        p1 = subprocess.Popen(['ros2', 'run', 'ros2_april_detection', 'april_detection_node'],
                            preexec_fn=os.setsid)
        p2 = subprocess.Popen(['ros2', 'run', 'ros2_april_detection', 'aptil_tf_subscriber'],
                            preexec_fn=os.setsid)
        p3 = subprocess.Popen(['ros2', 'run', 'rb5_ros2_control', 'subscriber_node.py'],
                            preexec_fn=os.setsid)

        auto_processes = [p1, p2, p3]
        print("🚀 AprilTag detection launched")
        return 'AUTO mode activated'
    except Exception as e:
        print(f"❌ Failed to launch AUTO mode: {e}")
        return 'Error launching AUTO mode', 500
    
@app.route('/auto_stop', methods=['POST'])
def auto_stop():
    global auto_processes
    try:
        for p in auto_processes:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)  # Send SIGTERM to the entire group
            p.wait()
        auto_processes.clear()
        if ros_node:
            ros_node.publish_joy('stop')
        print("🛑 AUTO processes stopped")
        return 'AUTO mode stopped'
    except Exception as e:
        print(f"❌ Failed to stop AUTO mode: {e}")
        return 'Error stopping AUTO mode', 500




def generate_stream():
    while True:
        with frame_lock:
            if shared_frame:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + shared_frame + b'\r\n')
        time.sleep(0.05)

def ros_spin():
    global ros_node
    rclpy.init()
    ros_node = ROSNode()
    rclpy.spin(ros_node)
    rclpy.shutdown()

def main():
    threading.Thread(target=ros_spin, daemon=True).start()
    app.run(host='0.0.0.0', port=8013, threaded=True)

if __name__ == '__main__':
    main()
