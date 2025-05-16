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
            # Move forward for 5 seconds
            msg.linear.x = 0.5
            self.counter += 1
            if self.counter >= 50:  # 0.1s * 50 = 5s
                self.step += 1
                self.counter = 0

        elif self.step == 1:
            # Turn right 45° (approx 0.5 rad/s for 1.57s)
            msg.angular.z = -0.4
            #msg.linear.x = 0.2
            self.counter += 1
            if self.counter >= 15:
                self.step += 1
                self.counter = 0

        elif self.step == 2:
            # Move forward for 2.5 seconds
            msg.linear.x = 0.4
            self.counter += 1
            if self.counter >= 28:  # 0.1s * 20 = 2s
                self.step += 1
                self.counter = 0

        elif self.step == 3:
            # Turn right 45° (approx 0.5 rad/s for 1.57s)
            msg.angular.z = -0.4
            #msg.linear.x = 0.2
            self.counter += 1
            if self.counter >= 15:
                self.step += 1
                self.counter = 0
        
        # elif self.step == 3:
        #     # Turn left 45°
        #     msg.angular.z = 0.4
        #     self.counter += 1
        #     if self.counter >= 16:
        #         self.step += 1
        #         self.counter = 0

        elif self.step == 4:
            # Move forward for 2 seconds
            msg.linear.x = 0.5
            self.counter += 1
            if self.counter >= 25:  # 0.1s * 50 = 5s
                self.step += 1
                self.counter = 0

        elif self.step == 5:
            # Turn right 45° (approx 0.5 rad/s for 1.57s)
            msg.angular.z = -0.4
            #msg.linear.x = 0.2
            self.counter += 1
            if self.counter >= 15:
                self.step += 1
                self.counter = 0   
        
        elif self.step == 6:
            # Move forward for 2 seconds
            msg.linear.x = 0.4
            self.counter += 1
            if self.counter >= 30:  # 0.1s * 20 = 2s
                self.step += 1
                self.counter = 0

        elif self.step == 7:
            # Turn left 45° (approx 0.5 rad/s for 1.57s)
            msg.angular.z = 0.4
            #msg.linear.x = 0.2
            self.counter += 1
            if self.counter >= 15:
                self.step += 1
                self.counter = 0          

        elif self.step == 8:
            # Move forward for 5 seconds
            msg.linear.x = 0.5
            self.counter += 1
            if self.counter >= 50:  # 0.1s * 50 = 5s
                self.step += 1
                self.counter = 0

        # elif self.step == 2:
        #     # Turn left 45°
        #     msg.angular.z = 0.5
        #     self.counter += 1
        #     if self.counter >= 16:
        #         self.step += 1
        #         self.counter = 0
        


        # elif self.step == 3:
        #     # Move backward for 4 seconds
        #     msg.linear.x = -0.4
        #     self.counter += 1
        #     if self.counter >= 40:
        #         self.step += 1
        #         self.counter = 0

        elif self.step == 9:
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



# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from math import sqrt


# class ClosedLoopController(Node):
#     def __init__(self):
#         super().__init__('closed_loop_controller')
#         self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
#         self.odom_subscriber = self.create_subscription(
#             Odometry,
#             'odom',
#             self.odom_callback,
#             10
#         )
#         self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

#         # Initial state
#         self.start_position = None
#         self.current_position = None
#         self.reached_target = False
#         self.target_distance = 2.0  # meters

#     def odom_callback(self, msg):
#         # Update current position
#         self.current_position = msg.pose.pose.position

#         if self.start_position is None:
#             self.start_position = msg.pose.pose.position
#             self.get_logger().info('Start position recorded.')

#     def timer_callback(self):
#         msg = Twist()

#         if self.reached_target or self.start_position is None or self.current_position is None:
#             self.publisher_.publish(msg)  # Stop
#             return

#         # Calculate distance from start
#         dx = self.current_position.x - self.start_position.x
#         dy = self.current_position.y - self.start_position.y
#         distance = sqrt(dx**2 + dy**2)

#         if distance < self.target_distance:
#             msg.linear.x = 0.3  # Forward speed
#         else:
#             msg.linear.x = 0.0
#             self.reached_target = True
#             self.get_logger().info(f'Target distance {self.target_distance}m reached. Stopping.')

#         self.publisher_.publish(msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = ClosedLoopController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

##***TRYING 2- ODOMETRY ******

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist, Quaternion
# import math

# def euler_to_quaternion(roll, pitch, yaw):
#     qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
#     qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
#     qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
#     qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
#     return [qx, qy, qz, qw]

# class OpenLoopOdom(Node):
#     def __init__(self):
#         super().__init__('open_loop_odom')
#         self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
#         self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)

#         self.x = 0.0
#         self.y = 0.0
#         self.theta = 0.0

#         self.vx = 0.0
#         self.vz = 0.0

#         now = self.get_clock().now().to_msg()
#         self.last_time = now.sec + now.nanosec * 1e-9

#         self.timer = self.create_timer(0.1, self.update_odom)

#     def cmd_callback(self, msg):
#         self.vx = msg.linear.x
#         self.vz = msg.angular.z

#     def update_odom(self):
#         now = self.get_clock().now().to_msg()
#         current_time = now.sec + now.nanosec * 1e-9
#         dt = current_time - self.last_time
#         self.last_time = current_time

#         delta_x = self.vx * math.cos(self.theta) * dt
#         delta_y = self.vx * math.sin(self.theta) * dt
#         delta_theta = self.vz * dt

#         self.x += delta_x
#         self.y += delta_y
#         self.theta += delta_theta

#         q = euler_to_quaternion(0, 0, self.theta)

#         odom = Odometry()
#         odom.header.stamp = self.get_clock().now().to_msg()
#         odom.header.frame_id = 'odom'
#         odom.child_frame_id = 'base_link'

#         odom.pose.pose.position.x = self.x
#         odom.pose.pose.position.y = self.y
#         odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

#         odom.twist.twist.linear.x = self.vx
#         odom.twist.twist.angular.z = self.vz

#         self.odom_pub.publish(odom)

# def main(args=None):
#     rclpy.init(args=args)
#     node = OpenLoopOdom()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

