#include "map_memory_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
: logger_(logger) {}

void MapMemoryCore::configure(const std::string& frame_id, double resolution, int width,
                              int height, double origin_x, double origin_y) {

  global_map_.header.frame_id = frame_id;
  global_map_.info.resolution = static_cast<float>(resolution);
  global_map_.info.width = static_cast<uint32_t>(width);
  global_map_.info.height = static_cast<uint32_t>(height);

  // Aligning the map with the world
  global_map_.info.origin.position.x = origin_x;
  global_map_.info.origin.position.y = origin_y;
  global_map_.info.origin.position.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0;

  // Marking every cell UNKNOWN (-1)
  global_map_.data.assign(static_cast<size_t>(width) * height, -1);

  RCLCPP_INFO(logger_, "Global map configured: %dx%d cells @ %.2f m, origin (%.1f, %.1f) in '%s'",
              width, height, resolution, origin_x, origin_y, frame_id.c_str());
}

// Integrates the latest local costmap into the global map using inverse mapping:
// for each cell in the global map, transform its position into the robot's frame
// (undo the robot's pose) to find the corresponding costmap cell, and copy its value
// over if known. Iterating over the destination (not the source) avoids gaps that
// forward mapping would leave when the costmap is rotated relative to the global grid.
void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                                     const Pose2D& robot_pose) {

  // Pulling out the grid metadata
  const auto& g_info = global_map_.info;
  const auto& c_info = costmap.info;

  const double g_res = g_info.resolution;
  const double g_ox = g_info.origin.position.x;
  const double g_oy = g_info.origin.position.y;
  const int g_w = static_cast<int>(g_info.width);
  const int g_h = static_cast<int>(g_info.height);

  const double c_res = c_info.resolution;
  const double c_ox = c_info.origin.position.x;
  const double c_oy = c_info.origin.position.y;
  const int c_w = static_cast<int>(c_info.width);
  const int c_h = static_cast<int>(c_info.height);

  // Pre-compute sin/cos once instead of for every cell.
  const double cos_yaw = std::cos(robot_pose.yaw);
  const double sin_yaw = std::sin(robot_pose.yaw);

  // Transform the corners of the costmap into the global map and take the cells
  // bounded by them
  const std::array<std::array<double, 2>, 4> local_corners = {{
    {c_ox, c_oy},
    {c_ox + c_w * c_res, c_oy},
    {c_ox, c_oy + c_h * c_res},
    {c_ox + c_w * c_res, c_oy + c_h * c_res},
  }};

  double min_wx = std::numeric_limits<double>::max();
  double min_wy = std::numeric_limits<double>::max();
  double max_wx = std::numeric_limits<double>::lowest();
  double max_wy = std::numeric_limits<double>::lowest();
  for (const auto& corner : local_corners) {
    const double wx = robot_pose.x + cos_yaw * corner[0] - sin_yaw * corner[1];
    const double wy = robot_pose.y + sin_yaw * corner[0] + cos_yaw * corner[1];
    min_wx = std::min(min_wx, wx);
    min_wy = std::min(min_wy, wy);
    max_wx = std::max(max_wx, wx);
    max_wy = std::max(max_wy, wy);
  }

  // World bounding box -> global cell index range, clipped to the global map's edges.
  const int gx_min = std::max(0, static_cast<int>(std::floor((min_wx - g_ox) / g_res)));
  const int gy_min = std::max(0, static_cast<int>(std::floor((min_wy - g_oy) / g_res)));
  const int gx_max = std::min(g_w - 1, static_cast<int>(std::floor((max_wx - g_ox) / g_res)));
  const int gy_max = std::min(g_h - 1, static_cast<int>(std::floor((max_wy - g_oy) / g_res)));

  // Inverse mapping every cell in the created box
  for (int gy = gy_min; gy <= gy_max; ++gy) {
    for (int gx = gx_min; gx <= gx_max; ++gx) {
      // Getting the centre of this global cell, in world metres.
      const double wx = g_ox + (gx + 0.5) * g_res;
      const double wy = g_oy + (gy + 0.5) * g_res;

      // world -> robot frame.
      const double dx = wx - robot_pose.x;
      const double dy = wy - robot_pose.y;
      const double lx = cos_yaw * dx + sin_yaw * dy;
      const double ly = -sin_yaw * dx + cos_yaw * dy;

      // robot frame -> costmap cell.
      const int cx = static_cast<int>(std::floor((lx - c_ox) / c_res));
      const int cy = static_cast<int>(std::floor((ly - c_oy) / c_res));
      if (cx < 0 || cx >= c_w || cy < 0 || cy >= c_h) {
        continue;  // inside the bounding box, but outside the rotated costmap itself
      }

      // Fusing the global map and new map
      const int8_t value = costmap.data[cy * c_w + cx];
      if (value >= 0) { // Was not -1, .'. cell on costmap had value
        global_map_.data[gy * g_w + gx] = value;
      }
    }
  }
}

}  // namespace robot
