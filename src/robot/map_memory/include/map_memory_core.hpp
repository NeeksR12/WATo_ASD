#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot {

// Only doing 2D math, since robot moves in x and y, no z or w needed
struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;  // 0 = facing world +x, positive = counter-clockwise
};


class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // Configures the empty global map
    void configure(const std::string& frame_id, double resolution, int width, int height,
                   double origin_x, double origin_y);

    // Takes a costmap and implements it into the global map using the robots pose
    void integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap, const Pose2D& robot_pose);

    // Read-only access to the global map, so the node can publish it.
    const nav_msgs::msg::OccupancyGrid& map() const { return global_map_; }

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid global_map_;
};

}  // namespace robot

#endif  // MAP_MEMORY_CORE_HPP_
