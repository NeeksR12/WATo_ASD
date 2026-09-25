#include "planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger) {}

// Decides how expensive it is to travel into a cell based on its cost and multipliers
double PlannerCore::traversalMultiplier(int8_t value) const {
  if (value >= 100) {
    return -1.0;  // wall: never enter
  }
  if (value < 0) {
    return 1.0;  // unknown: treat as free
  }
  if (value >= config_.lethal_cost) {
    return config_.lethal_multiplier;
  }
  return 1.0 + config_.cost_weight * (static_cast<double>(value) / 100.0);
}

// Uses A* to plan an algorithm
bool PlannerCore::planPath(const nav_msgs::msg::OccupancyGrid& map,
                           const geometry_msgs::msg::Point& start,
                           const geometry_msgs::msg::Point& goal,
                           nav_msgs::msg::Path& path) const {

  // Resetting path info
  path.poses.clear();
  path.header.frame_id = map.header.frame_id;

  // Collecting data from map
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const double resolution = map.info.resolution;

  // Ensuring map is valid
  if (width == 0 || height == 0 || map.data.empty()) {
    RCLCPP_WARN(logger_, "Cannot plan: map is empty");
    return false;
  }

  // Convert start and goal from metres to cells
  CellIndex start_cell;
  CellIndex goal_cell;
  if (!worldToCell(map, start.x, start.y, start_cell)) {
    RCLCPP_WARN(logger_, "Cannot plan: start (%.2f, %.2f) is outside the map", start.x, start.y);
    return false;
  }
  if (!worldToCell(map, goal.x, goal.y, goal_cell)) {
    RCLCPP_WARN(logger_, "Cannot plan: goal (%.2f, %.2f) is outside the map", goal.x, goal.y);
    return false;
  }

  // Flat index helper: same row-major layout as OccupancyGrid.data.
  auto to_index = [width](const CellIndex& c) { return c.y * width + c.x; };

  if (traversalMultiplier(map.data[to_index(goal_cell)]) < 0.0) {
    RCLCPP_WARN(logger_, "Cannot plan: goal (%.2f, %.2f) is inside an obstacle", goal.x, goal.y);
    return false;
  }

  // Heuristic: straight-line distance in metres between cell centres.
  auto heuristic = [resolution, &goal_cell](const CellIndex& c) {
    return std::hypot(c.x - goal_cell.x, c.y - goal_cell.y) * resolution;
  };

  // Search bookkeeping
  const size_t num_cells = static_cast<size_t>(width) * height;
  
  std::vector<double> g_score(num_cells, std::numeric_limits<double>::infinity());

  std::vector<int> came_from(num_cells, -1);

  std::vector<bool> closed(num_cells, false);

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;

  g_score[to_index(start_cell)] = 0.0;
  open_set.emplace(start_cell, heuristic(start_cell));

  // Neighbouring cells 
  static constexpr double kSqrt2 = 1.41421356237309504880;
  static const int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1}; // dx in that direction
  static const int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1}; // likewise
  static const double kStep[8] = {1.0, 1.0, 1.0, 1.0, kSqrt2, kSqrt2, kSqrt2, kSqrt2}; // Actual distance of the step

  bool found = false;

  // Generating path with A*
  while (!open_set.empty()) {
    // Take the most promising cell (smallest f).
    const CellIndex current = open_set.top().index;
    open_set.pop();

    const int current_idx = to_index(current);

    // Removes an index if it was a previous version that got updates (.'. this was the earlier with worse score)
    if (closed[current_idx]) {
      continue;
    }
    closed[current_idx] = true;

    // Reached the goal: because of the admissible heuristic, this is the cheapest path.
    if (current == goal_cell) {
      found = true;
      break;
    }

    // Checking if there is a better route to any neighbouring cells
    for (int i = 0; i < 8; ++i) {
      const CellIndex neighbour(current.x + kDx[i], current.y + kDy[i]);
      if (neighbour.x < 0 || neighbour.x >= width || neighbour.y < 0 || neighbour.y >= height) {
        continue;  // off the edge of the map
      }

      const int neighbour_idx = to_index(neighbour);
      if (closed[neighbour_idx]) {
        continue;  // already has its final, optimal g
      }

      const double multiplier = traversalMultiplier(map.data[neighbour_idx]);
      if (multiplier < 0.0) {
        continue;  // wall
      }

      // Cost to reach neighbour via current = cost to reach current + this one step.
      const double tentative_g = g_score[current_idx] + kStep[i] * resolution * multiplier;

      // Only keep it if it beats every route to neighbour we've found so far.
      if (tentative_g < g_score[neighbour_idx]) {
        g_score[neighbour_idx] = tentative_g;
        came_from[neighbour_idx] = current_idx;
        open_set.emplace(neighbour, tentative_g + heuristic(neighbour));
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(logger_, "A* found no path from (%.2f, %.2f) to (%.2f, %.2f)",
                start.x, start.y, goal.x, goal.y);
    return false;
  }

  // Reconstructing the path
  std::vector<CellIndex> cells;
  for (int idx = to_index(goal_cell); idx != -1; idx = came_from[idx]) {
    cells.emplace_back(idx % width, idx / width);
  }
  std::reverse(cells.begin(), cells.end());

  // Turning the path into a message
  path.poses.reserve(cells.size());
  for (size_t i = 0; i < cells.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = map.header.frame_id;

    // Use the exact start/goal for the endpoints rather than their cell centres, so the
    // path starts under the robot and ends precisely where the user clicked.
    if (i == 0) {
      pose.pose.position = start;
    } else if (i + 1 == cells.size()) {
      pose.pose.position = goal;
    } else {
      pose.pose.position = cellToWorld(map, cells[i]);
    }
    pose.pose.position.z = 0.0;
    path.poses.push_back(pose);
  }

  // Giving every pose an orientation facing the next pose
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const size_t next = std::min(i + 1, path.poses.size() - 1);
    const size_t prev = (next == i && i > 0) ? i - 1 : i;
    const auto& a = path.poses[prev].pose.position;
    const auto& b = path.poses[next].pose.position;
    const double yaw = std::atan2(b.y - a.y, b.x - a.x);
    path.poses[i].pose.orientation.z = std::sin(yaw / 2.0);
    path.poses[i].pose.orientation.w = std::cos(yaw / 2.0);
  }

  return true;
}

bool PlannerCore::worldToCell(const nav_msgs::msg::OccupancyGrid& map, double wx, double wy,
                              CellIndex& cell) {
  const double res = map.info.resolution;
  cell.x = static_cast<int>(std::floor((wx - map.info.origin.position.x) / res));
  cell.y = static_cast<int>(std::floor((wy - map.info.origin.position.y) / res));
  return cell.x >= 0 && cell.x < static_cast<int>(map.info.width) &&
         cell.y >= 0 && cell.y < static_cast<int>(map.info.height);
}

geometry_msgs::msg::Point PlannerCore::cellToWorld(const nav_msgs::msg::OccupancyGrid& map,
                                                   const CellIndex& cell) {
  // +0.5 -> the centre of the cell rather than its corner.
  geometry_msgs::msg::Point p;
  p.x = map.info.origin.position.x + (cell.x + 0.5) * map.info.resolution;
  p.y = map.info.origin.position.y + (cell.y + 0.5) * map.info.resolution;
  p.z = 0.0;
  return p;
}

}  // namespace robot
