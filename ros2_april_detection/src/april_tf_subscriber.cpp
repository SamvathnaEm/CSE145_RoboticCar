#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <chrono>

class AptilTfSubscriber : public rclcpp::Node {
public:
    AptilTfSubscriber() : Node("aptil_tf_subscriber") {
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/april_poses", 10, std::bind(&AptilTfSubscriber::poseCallback, this, std::placeholders::_1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&AptilTfSubscriber::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "AptilTfSubscriber node has been started.");
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        try {
            geometry_msgs::msg::PoseStamped transformed_pose;
            tf_buffer_->transform(*msg, transformed_pose, "camera");

            tag_detected_ = true; // Tag detected
            if (transformed_pose.pose.position.z < obstacle_threshold) {
                // Tag within threshold: turn the car
                RCLCPP_INFO(this->get_logger(), "Tag within threshold at x: %f, y: %f, z: %f",
                            transformed_pose.pose.position.x, transformed_pose.pose.position.y, transformed_pose.pose.position.z);

                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.5; // Rotate
                cmd_vel_publisher_->publish(cmd_vel);
            } else {
                // Tag outside threshold: move forward
                RCLCPP_INFO(this->get_logger(), "Tag outside threshold at x: %f, y: %f, z: %f",
                            transformed_pose.pose.position.x, transformed_pose.pose.position.y);

                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.linear.x = 0.5; // Move forward
                cmd_vel.angular.z = 0.0;
                cmd_vel_publisher_->publish(cmd_vel);
            }

            // Update last detected pose
            last_detected_pose_ = *msg;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform pose: %s", ex.what());
        }
    }

    void timerCallback() {
        if (!tag_detected_) {
            // No tag detected: move forward for 3 seconds, turn for 2 seconds, then stop
            auto now = this->get_clock()->now();

            // Use IMU or camera logic to infer obstacles (placeholder for actual implementation)
            // if (cameraDetectsObstacle()) {
            //     // Stop immediately if an obstacle is inferred
            //     geometry_msgs::msg::Twist cmd_vel;
            //     cmd_vel.linear.x = 0.0;
            //     cmd_vel.angular.z = 0.0;
            //     cmd_vel_publisher_->publish(cmd_vel);
            //     RCLCPP_WARN(this->get_logger(), "Obstacle inferred from camera! Stopping the car.");
            //     return;
            // }

            if (now - last_action_time_ < rclcpp::Duration::from_seconds(3.0)) {
                // Move forward
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.linear.x = 0.5;
                cmd_vel.angular.z = 0.0;
                cmd_vel_publisher_->publish(cmd_vel);
            } else if (now - last_action_time_ < rclcpp::Duration::from_seconds(5.0)) {
                // Turn
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.5;
                cmd_vel_publisher_->publish(cmd_vel);
            } else {
                // Stop
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                cmd_vel_publisher_->publish(cmd_vel);
                last_action_time_ = now; // Reset timer
            }
        } else {
            tag_detected_ = false; // Reset tag detection flag
            last_action_time_ = this->get_clock()->now(); // Reset timer
        }
    }

    bool cameraDetectsObstacle() {
        // Check if a tag was detected recently
        if (tag_detected_) {
            // If a tag is detected and within the obstacle threshold, infer an obstacle
            if (last_detected_pose_.pose.position.z < obstacle_threshold) {
                RCLCPP_WARN(this->get_logger(), "Obstacle detected based on tag position at z: %f",
                            last_detected_pose_.pose.position.z);
                return true;
            }
        }

        // If no tag is detected or no obstacle is inferred, return false
        return false;
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    const double obstacle_threshold = 0.5; // Example threshold for obstacle detection
    bool tag_detected_ = false;
    rclcpp::Time last_action_time_ = this->get_clock()->now();
    geometry_msgs::msg::PoseStamped last_detected_pose_; // Store the last detected pose
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AptilTfSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}