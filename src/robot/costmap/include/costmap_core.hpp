#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <cstdint>
#include <utility>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

struct CostmapParams {
  double resolution{0.1};
  int width{300};
  int height{300};
  double inflation_radius{1.0};
  int8_t max_cost{100};
};

class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger);

    void setParams(const CostmapParams& params);
    nav_msgs::msg::OccupancyGrid processScan(const sensor_msgs::msg::LaserScan& scan);

  private:
    void initializeGrid();
    bool worldToGrid(double x, double y, int& gx, int& gy) const;
    void markObstacle(int gx, int gy);
    void inflateObstacles();
    int toIndex(int gx, int gy) const;
    bool inBounds(int gx, int gy) const;

    rclcpp::Logger logger_;
    CostmapParams params_;
    std::vector<int8_t> grid_;
    std::vector<std::pair<int, int>> obstacles_;
};

}

#endif
