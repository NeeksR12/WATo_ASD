#include "costmap_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {
  configure(resolution_, width_, height_, inflation_radius_, max_cost_);
}

void CostmapCore::configure(double resolution, int width_cells, int height_cells,
                            double inflation_radius, int max_cost) {

  resolution_ = resolution;
  width_ = width_cells;
  height_ = height_cells;
  inflation_radius_ = inflation_radius;
  // OccupancyGrid values are int8 and "real" costs live in 0..100, so clamp to that.
  max_cost_ = std::clamp(max_cost, 0, 100);

  // Putting the lidar centered in the grid
  origin_x_ = -0.5 * width_ * resolution_;
  origin_y_ = -0.5 * height_ * resolution_;

  // Allocating the grid once here as opposed to every scan
  grid_.assign(static_cast<size_t>(width_) * height_, kUnknown);

  buildInflationKernel();

  RCLCPP_INFO(logger_,
              "Costmap configured: %dx%d cells @ %.2f m (%.1f x %.1f m), inflation %.2f m",
              width_, height_, resolution_, width_ * resolution_, height_ * resolution_,
              inflation_radius_);
}


// Builds the inflation cost for a cell around an object based on the inflation radius
// and resulution once based on the dx and dy to avoid taking repeated sqrt. Uses a
// lookup table for cell costs after this.
void CostmapCore::buildInflationKernel() {
  inflation_kernel_.clear();

  // Determining radius in cells
  const int radius_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));

  // Checks every cell in the square around the circle
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      if (dx == 0 && dy == 0) { // This is the obsticle
        continue;
      }

      // Calculating distance from obsticle and confirming it is in the circle
      const double distance = std::hypot(dx, dy) * resolution_;
      if (distance > inflation_radius_) {
        continue;
      }

      // Calculating the cost of the cell
      const double cost = max_cost_ * (1.0 - distance / inflation_radius_);

      // Ensuring the cell acutally has an inflated value
      const int8_t cost_int = static_cast<int8_t>(cost);
      if (cost_int <= 0) {
        continue;
      }

      inflation_kernel_.push_back({dx, dy, cost_int});
    }
  }
}

// Takes lidar scan and returns ocupancy grid message to publish
nav_msgs::msg::OccupancyGrid CostmapCore::buildCostmap(const sensor_msgs::msg::LaserScan& scan) {

  initializeCostmap();

  // The lidar sits at (0, 0) metres in its own frame, which is the centre of our grid.
  // Every beam starts here.
  int origin_gx = 0;
  int origin_gy = 0;
  worldToGrid(0.0, 0.0, origin_gx, origin_gy);

  // Proccessing beams in two passes:
  //   pass A: raytrace FREE space for every beam
  //   pass B: mark every hit as OCCUPIED
  struct Hit {
    int gx;
    int gy;
  };
  std::vector<Hit> hits;
  hits.reserve(scan.ranges.size());

  // Converting scan to a grid and tracing free space
  for (size_t i = 0; i < scan.ranges.size(); ++i) {.
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    double range = scan.ranges[i];

    // Ensuring only valid readings are used
    if (std::isnan(range) || range < scan.range_min) {
      continue;
    }

    // Seeing if the beam got to its max range without hitting anything (.'. free space 
    // to max range)
    bool is_hit = true;
    if (!std::isfinite(range) || range >= scan.range_max) {
      range = scan.range_max;
      is_hit = false;
    }

    // Polar -> Cartesian
    const double x = range * std::cos(angle);
    const double y = range * std::sin(angle);

    // Converting the beam's end point to a cell
    const int end_gx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    const int end_gy = static_cast<int>(std::floor((y - origin_y_) / resolution_));

    // Marking the free space
    raytraceFree(origin_gx, origin_gy, end_gx, end_gy);

    // Adding the hit 
    if (is_hit && inBounds(end_gx, end_gy)) {
      hits.push_back({end_gx, end_gy});
    }
  }

  // Marking and inflating obsticles
  for (const auto& hit : hits) {
    markObstacle(hit.gx, hit.gy);
  }

  inflateObstacles();

  // Packaging into an OccupancyGrid message 
  nav_msgs::msg::OccupancyGrid msg;

  // Reusing the scan's timestamp and frame
  msg.header = scan.header;

  msg.info.map_load_time = scan.header.stamp;
  msg.info.resolution = static_cast<float>(resolution_);
  msg.info.width = static_cast<uint32_t>(width_);
  msg.info.height = static_cast<uint32_t>(height_);

  // Position of the corner cell
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  // Sending the 1D array
  msg.data = grid_;

  return msg;
}

void CostmapCore::initializeCostmap() {
  // std::fill reuses the memory allocated in configure(); no new allocation per scan.
  std::fill(grid_.begin(), grid_.end(), kUnknown);
  obstacle_indices_.clear();
}


// Marks each space the laser passes through as free between the robot's position and
// using Bresenham's line algorithm the end of the laser position
void CostmapCore::raytraceFree(int x0, int y0, int x1, int y1) {
  const int dx = std::abs(x1 - x0);
  const int dy = -std::abs(y1 - y0);  // negative by convention of this Bresenham form
  const int step_x = (x0 < x1) ? 1 : -1;
  const int step_y = (y0 < y1) ? 1 : -1;
  int error = dx + dy;

  int x = x0;
  int y = y0;
  while (true) {
    // Stop at the end cell (don't free the cell the beam hit).
    if (x == x1 && y == y1) {
      break;
    }

    // Checking if the cell is still in bounds
    if (!inBounds(x, y)) {
      break;
    }

    // Marking the current grid coordinate free
    grid_[toIndex(x, y)] = kFree;

    // Decide whether to step in x, y, or both, based on the accumulated error.
    const int twice_error = 2 * error;
    if (twice_error >= dy) {
      error += dy;
      x += step_x;
    }
    if (twice_error <= dx) {
      error += dx;
      y += step_y;
    }
  }
}

void CostmapCore::markObstacle(int x, int y)
{
  const int index = toIndex(x, y);

  // Checking if this cell is already occupied
  if (grid_[index] != kOccupied) { // it wasn't, .'. needs to be marked
    grid_[index] = kOccupied;
    obstacle_indices_.push_back(index);
  }
}

// Marks the cells close to an obsticle as such
void CostmapCore::inflateObstacles() {
  for (const int obstacle_index : obstacle_indices_) {
    // Convert the flat index back to (x, y) so we can apply the (dx, dy) offsets.
    const int ox = obstacle_index % width_;
    const int oy = obstacle_index / width_;

    // Using the inflation stamp
    for (const auto& k : inflation_kernel_) {
      const int nx = ox + k.dx;
      const int ny = oy + k.dy;
      if (!inBounds(nx, ny)) {
        continue;
      }

      int8_t& cell = grid_[toIndex(nx, ny)];

      // Making sure the cost being assigned is more than the current cost of the cell
      if (k.cost > cell) {
        cell = k.cost;
      }
    }
  }
}

// Mapping the actual messurement units to grid spaces
bool CostmapCore::worldToGrid(double x, double y, int& gx, int& gy) const {
  gx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  gy = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  return inBounds(gx, gy);
}

// Returns true if the coordinates provided are in the bounds
bool CostmapCore::inBounds(int x, int y) const {
  return x >= 0 && x < width_ && y >= 0 && y < height_;
}

}  // namespace robot
