#include <chrono>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  // Setting default params
  robot::ControlConfig config;
  config.lookahead_distance = this->declare_parameter<double>("lookahead_distance", 1.5);
  config.goal_tolerance = this->declare_parameter<double>("goal_tolerance", 0.5);
  config.linear_speed = this->declare_parameter<double>("linear_speed", 1.0);
  config.min_linear_speed = this->declare_parameter<double>("min_linear_speed", 0.2);
  config.max_angular_speed = this->declare_parameter<double>("max_angular_speed", 1.5);
  config.rotate_in_place_angle = this->declare_parameter<double>("rotate_in_place_angle", 1.2);
  config.pursuit_point_offset = this->declare_parameter<double>("pursuit_point_offset", 1.3);
  const int control_period_ms = this->declare_parameter<int>("control_period_ms", 100);
  control_.configure(config);

  // Subscribers
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Timer
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(control_period_ms), std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = msg;
  if (!msg->poses.empty()) {
    active_ = true;
  }
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_odom_ = msg;
}

void ControlNode::controlLoop() {
  // Waiting for a path and pose.
  if (!current_path_ || !robot_odom_) {
    return;
  }

  // Checking if idle
  if (!active_) {
    return;
  }

  // Seeing if planner told robot to stop (goal reached, timed out, or no path found).
  if (current_path_->poses.empty()) {
    publishStop();
    return;
  }

  const auto result = control_.computeCommand(*current_path_, robot_odom_->pose.pose);

  if (result.goal_reached) {
    RCLCPP_INFO(this->get_logger(), "Within goal tolerance, stopping");
    publishStop();
    return;
  }

  cmd_vel_pub_->publish(result.cmd);
}

void ControlNode::publishStop() {
  // A default-constructed Twist is all zeros: no forward speed, no turning.
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
  active_ = false;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
