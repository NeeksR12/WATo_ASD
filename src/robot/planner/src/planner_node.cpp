#include <chrono>
#include <cmath>
#include <memory>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  // Setting param defaults
  map_frame_ = this->declare_parameter<std::string>("map_frame", "sim_world");
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.5);
  goal_timeout_s_ = this->declare_parameter<double>("goal_timeout_s", 120.0);
  replan_period_s_ = this->declare_parameter<double>("replan_period_s", 2.0);

  robot::PlannerConfig config;
  config.lethal_cost = this->declare_parameter<int>("lethal_cost", 50);
  config.lethal_multiplier = this->declare_parameter<double>("lethal_multiplier", 100.0);
  config.cost_weight = this->declare_parameter<double>("cost_weight", 5.0);
  planner_.configure(config);

  // Subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Timer 
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));

  goal_start_time_ = this->now();
  last_plan_time_ = this->now();
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  have_map_ = true;

  // New map = maybe new obstacles on current path
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
    RCLCPP_WARN(this->get_logger(),
                "Goal is in frame '%s' but the map is in '%s'; treating it as '%s' anyway",
                msg->header.frame_id.c_str(), map_frame_.c_str(), map_frame_.c_str());
  }

  goal_ = *msg;
  // A new goal always wins, even if we were in the middle of driving to an old one.
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  goal_start_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f)", goal_.point.x, goal_.point.y);
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
  have_odom_ = true;
}

void PlannerNode::timerCallback() {
  // Nothing to supervise while idle.
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;
  }

  // Checking if goal was reached
  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    state_ = State::WAITING_FOR_GOAL;
    publishEmptyPath();
    return;
  }

  // Giving up since goal was not reached in normal time, timeout
  const double elapsed = (this->now() - goal_start_time_).seconds();
  if (elapsed > goal_timeout_s_) {
    RCLCPP_WARN(this->get_logger(), "Goal timed out after %.0f s, giving up", elapsed);
    state_ = State::WAITING_FOR_GOAL;
    publishEmptyPath();
    return;
  }

  // Periodic replan of the path
  if ((this->now() - last_plan_time_).seconds() >= replan_period_s_) {
    planPath();
  }
}

bool PlannerNode::goalReached() const {
  if (!have_odom_) {
    return false;
  }
  const double dx = goal_.point.x - robot_pose_.position.x;
  const double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < goal_tolerance_;
}

void PlannerNode::planPath() {
  last_plan_time_ = this->now();

  if (!have_map_ || !have_odom_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Cannot plan path: missing %s", !have_map_ ? "map" : "odometry");
    return;
  }

  nav_msgs::msg::Path path;
  const bool ok = planner_.planPath(current_map_, robot_pose_.position, goal_.point, path);

  if (!ok) {
    // Publishing an empty path stops the robot to stand still so the robot 
    // doesn't follow an old path
    publishEmptyPath();
    return;
  }

  path.header.stamp = this->now();
  path_pub_->publish(path);
  RCLCPP_DEBUG(this->get_logger(), "Published path with %zu poses", path.poses.size());
}

void PlannerNode::publishEmptyPath() {
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = map_frame_;
  path_pub_->publish(path);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
