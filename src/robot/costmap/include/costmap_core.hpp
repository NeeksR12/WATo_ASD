#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <cstdint>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constants to declare cells
    static constexpr int8_t kUnknown = -1;
    static constexpr int8_t kFree = 0;
    static constexpr int8_t kOccupied = 100;

    // Constructor
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Configures the grid
    void configure(double resolution, int width_cells, int height_cells,
                   double inflation_radius, int max_cost);

    // Method to take the laser scan from the lidar and returns the message to be posted to costmap with the ocupancy grid
    nav_msgs::msg::OccupancyGrid buildCostmap(const sensor_msgs::msg::LaserScan& scan);

  private:
    // Re-initializes the costmap
    void initializeCostmap();

    // Marks each space the laser passes through as free between the robot's position and the end of the laser position
    void raytraceFree(int x0, int y0, int x1, int y1);

    // Marks an obstacle at the end of the laser
    void markObstacle(int x, int y);

    // Adds cost around an obsticle
    void inflateObstacles();

    // ---- Helpers ------------------------------------------------------------------

    // Converts a point in metres to integer grid coordinates. Returns false
    // if the point lies outside the grid, in which case gx/gy should not be used.
    bool worldToGrid(double x, double y, int& gx, int& gy) const;

    // True if (x, y) is a valid cell inside the grid.
    bool inBounds(int x, int y) const;

    // Converts a 2D coordinate into a 1D array index (rows are y and columns are x)
    int toIndex(int x, int y) const { return y * width_ + x; }

    // Builds the inflation cost "stamp", since inflation for a tile dx and dy from 
    // (x, y) has the same cost
    void buildInflationKernel();

    // One entry of the inflation stamp: "the cell dx, dy away from an obstacle gets
    // this much cost".
    struct KernelCell {
      int dx;
      int dy;
      int8_t cost;
    };

    rclcpp::Logger logger_;

    // Grid geometry
    double resolution_ = 0.1;
    int width_ = 300;
    int height_ = 300;
    // World coordinates (metres, lidar frame) of the *corner* of cell (0, 0). Because we
    // want the lidar in the middle of the grid, this is (-width/2, -height/2) in metres.
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;

    // Inflation settings
    double inflation_radius_ = 1.2;
    int max_cost_ = 100;
    std::vector<KernelCell> inflation_kernel_;

    // The grid itself (width_ * height_ cells), plus a list of the cells we marked as
    // OCCUPIED this scan so inflation doesn't have to scan the whole grid to find them.
    std::vector<int8_t> grid_;
    std::vector<int> obstacle_indices_;
};

}  // namespace robot

#endif  // COSTMAP_CORE_HPP_
