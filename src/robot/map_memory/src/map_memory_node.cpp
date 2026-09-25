#include <chrono>
#include <cmath>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
: Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  // Declaring default params
  const std::string frame_id = this->declare_parameter<std::string>("map_frame", "sim_world");
  const double resolution = this->declare_parameter<double>("resolution", 0.1);
  const int width = this->declare_parameter<int>("width", 400);
  const int height = this->declare_parameter<int>("height", 400);
  const double origin_x = this->declare_parameter<double>("origin_x", -20.0);
  const double origin_y = this->declare_parameter<double>("origin_y", -20.0);
  const double update_period_s = this->declare_parameter<double>("update_period_s", 1.0);
  distance_threshold_ = this->declare_parameter<double>("distance_threshold", 1.5);

  map_memory_.configure(frame_id, resolution, width, height, origin_x, origin_y);

  // Subscribers
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10,
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(update_period_s * 1000.0)),
    std::bind(&MapMemoryNode::updateMap, this));

  // Publishing empty map to start so planner has something to work with
  map_pub_->publish(map_memory_.map());
}

// Recieves the costmap from the subscription
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {

  latest_costmap_ = *msg;
  latest_costmap_pose_ = current_pose_;

  have_costmap_ = have_odom_;
}

// Recieves the odom from the subscription
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_pose_ = poseFromOdometry(*msg);
  have_odom_ = true;
}

// Checks if mapupdate is needed
void MapMemoryNode::updateMap()
{
  if (have_costmap_) {
    // Checking how far the robot has moved since the last fusion
    const double dx = latest_costmap_pose_.x - last_update_pose_.x;
    const double dy = latest_costmap_pose_.y - last_update_pose_.y;
    const double distance = std::sqrt(dx * dx + dy * dy);

    // Deciding whether a fuse is needed
    if (!has_integrated_once_ || distance >= distance_threshold_) {
      map_memory_.integrateCostmap(latest_costmap_, latest_costmap_pose_);
      last_update_pose_ = latest_costmap_pose_;
      has_integrated_once_ = true;
      RCLCPP_DEBUG(this->get_logger(), "Fused costmap at (%.2f, %.2f)",
                   last_update_pose_.x, last_update_pose_.y);
    }
  }

  // Publishes the map, whether updated or not, for any subscribers
  auto map_msg = map_memory_.map();
  map_msg.header.stamp = this->now();
  map_pub_->publish(map_msg);
}

// Returns the robots 2D pose from its quaternion pose
robot::Pose2D MapMemoryNode::poseFromOdometry(const nav_msgs::msg::Odometry& odom) {
  robot::Pose2D pose;
  pose.x = odom.pose.pose.position.x;
  pose.y = odom.pose.pose.position.y;

  const auto& q = odom.pose.pose.orientation;
  pose.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  return pose;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
