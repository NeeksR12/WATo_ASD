#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Declaring default params
  const double resolution = this->declare_parameter<double>("resolution", 0.1);
  const int width = this->declare_parameter<int>("width", 300);    // 300 * 0.1 = 30 m
  const int height = this->declare_parameter<int>("height", 300);
  const double inflation_radius = this->declare_parameter<double>("inflation_radius", 1.2);
  const int max_cost = this->declare_parameter<int>("max_cost", 100);
  const std::string lidar_topic = this->declare_parameter<std::string>("lidar_topic", "/lidar");
  const std::string costmap_topic =
    this->declare_parameter<std::string>("costmap_topic", "/costmap");

  costmap_.configure(resolution, width, height, inflation_radius, max_cost);

  // lidar subscriber
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    lidar_topic, 10,
    std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  // costmap publisher
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic, 10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  costmap_pub_->publish(costmap_.buildCostmap(*scan));
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
