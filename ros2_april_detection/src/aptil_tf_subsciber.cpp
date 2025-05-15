#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class AptilTfSubscriber : public rclcpp::Node {
public:
    AptilTfSubscriber() : Node("aptil_tf_subscriber") {
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/april_poses", 10, std::bind(&AptilTfSubscriber::poseCallback, this, std::placeholders::_1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "AptilTfSubscriber node has been started.");
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        try {
            geometry_msgs::msg::PoseStamped transformed_pose;
            tf_buffer_->transform(*msg, transformed_pose, "camera");

            if (transformed_pose.pose.position.x < obstacle_threshold) {
                RCLCPP_INFO(this->get_logger(), "Obstacle detected at x: %f, y: %f",
                            transformed_pose.pose.position.x, transformed_pose.pose.position.y);

                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.5; // Rotate to avoid obstacle
                cmd_vel_publisher_->publish(cmd_vel);
            } else {
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.linear.x = 0.2; // Move forward
                cmd_vel.angular.z = 0.0;
                cmd_vel_publisher_->publish(cmd_vel);
            }
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform pose: %s", ex.what());
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    const double obstacle_threshold = 1.0; // Example threshold for obstacle detection
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AptilTfSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}