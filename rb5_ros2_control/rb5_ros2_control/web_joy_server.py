#!/usr/bin/env python3
from flask import Flask, request, render_template
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import time

app = Flask(__name__)

# Initialize ROS2
rclpy.init()
node = rclpy.create_node('web_joy_node')
publisher = node.create_publisher(Joy, 'joy', 10)

@app.route('/')
def home():
    return render_template('control.html')

@app.route('/move', methods=['POST'])
def move():
    direction = request.form.get('direction')
    speed = 1.0
    joy_msg = Joy()
    joy_msg.axes = [0.0, 0.0, 0.0]  # x, y, z

    # Set direction
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

    # Publish movement
    publisher.publish(joy_msg)
    print(f"Sent move: {direction}")

    return f'Moved {direction}'

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8004)
