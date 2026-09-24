#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(this->get_logger()) {
  robot::CostmapParams params;
  params.resolution = this->declare_parameter<double>("resolution", 0.1);
  params.width = this->declare_parameter<int>("width", 300);
  params.height = this->declare_parameter<int>("height", 300);
  params.inflation_radius = this->declare_parameter<double>("inflation_radius", 1.0);
  params.max_cost = static_cast<int8_t>(this->declare_parameter<int>("max_cost", 100));
  costmap_.setParams(params);

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar",
      10,
      std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  costmap_pub_->publish(costmap_.processScan(*scan));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
