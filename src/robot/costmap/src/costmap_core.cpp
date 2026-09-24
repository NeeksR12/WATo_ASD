#include "costmap_core.hpp"

#include <algorithm>
#include <cmath>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::setParams(const CostmapParams& params) {
  params_ = params;
}

void CostmapCore::initializeGrid() {
  grid_.assign(static_cast<size_t>(params_.width * params_.height), 0);
  obstacles_.clear();
}

int CostmapCore::toIndex(int gx, int gy) const {
  return gy * params_.width + gx;
}

bool CostmapCore::inBounds(int gx, int gy) const {
  return gx >= 0 && gy >= 0 && gx < params_.width && gy < params_.height;
}

bool CostmapCore::worldToGrid(double x, double y, int& gx, int& gy) const {
  const double origin_x = -params_.width * params_.resolution / 2.0;
  const double origin_y = -params_.height * params_.resolution / 2.0;
  gx = static_cast<int>(std::floor((x - origin_x) / params_.resolution));
  gy = static_cast<int>(std::floor((y - origin_y) / params_.resolution));
  return inBounds(gx, gy);
}

void CostmapCore::markObstacle(int gx, int gy) {
  grid_[static_cast<size_t>(toIndex(gx, gy))] = params_.max_cost;
  obstacles_.emplace_back(gx, gy);
}

void CostmapCore::inflateObstacles() {
  const int radius_cells = static_cast<int>(
      std::ceil(params_.inflation_radius / params_.resolution));

  for (const auto& [ox, oy] : obstacles_) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        const int gx = ox + dx;
        const int gy = oy + dy;
        if (!inBounds(gx, gy)) {
          continue;
        }

        const double distance = std::hypot(dx * params_.resolution, dy * params_.resolution);
        if (distance > params_.inflation_radius) {
          continue;
        }

        const double scale = 1.0 - (distance / params_.inflation_radius);
        const int8_t cost = static_cast<int8_t>(
            std::clamp(params_.max_cost * scale, 0.0, static_cast<double>(params_.max_cost)));
        const size_t idx = static_cast<size_t>(toIndex(gx, gy));
        if (cost > grid_[idx]) {
          grid_[idx] = cost;
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid CostmapCore::processScan(const sensor_msgs::msg::LaserScan& scan) {
  initializeGrid();

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const float range = scan.ranges[i];
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      continue;
    }

    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double x = static_cast<double>(range) * std::cos(angle);
    const double y = static_cast<double>(range) * std::sin(angle);

    int gx = 0;
    int gy = 0;
    if (worldToGrid(x, y, gx, gy)) {
      markObstacle(gx, gy);
    }
  }

  inflateObstacles();

  nav_msgs::msg::OccupancyGrid msg;
  msg.header = scan.header;
  if (msg.header.frame_id.empty()) {
    msg.header.frame_id = "robot/chassis/lidar";
  }
  msg.info.resolution = static_cast<float>(params_.resolution);
  msg.info.width = static_cast<uint32_t>(params_.width);
  msg.info.height = static_cast<uint32_t>(params_.height);
  msg.info.origin.position.x = -params_.width * params_.resolution / 2.0;
  msg.info.origin.position.y = -params_.height * params_.resolution / 2.0;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(grid_.begin(), grid_.end());
  return msg;
}

}
