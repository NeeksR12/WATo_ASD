#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"


class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    // Callbacks
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateMap();

    // Pulls x, y and heading (yaw) out of a full 3D Odometry message.
    static robot::Pose2D poseFromOdometry(const nav_msgs::msg::Odometry& odom);

    robot::MapMemoryCore map_memory_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Info about the latest costmap
    nav_msgs::msg::OccupancyGrid latest_costmap_; // Map itself
    robot::Pose2D latest_costmap_pose_; // Robot pose upon read
    bool have_costmap_ = false;

    // Most recent pose from odometry.
    robot::Pose2D current_pose_;
    bool have_odom_ = false;

    // Pose at which we last fused a costmap, used for the "moved far enough?" check.
    robot::Pose2D last_update_pose_;
    bool has_integrated_once_ = false;

    // Only fuse a new costmap after the robot has moved this far (metres).
    double distance_threshold_ = 1.5;
};

#endif  // MAP_MEMORY_NODE_HPP_
