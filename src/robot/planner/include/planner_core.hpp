#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <cstddef>
#include <functional>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot {

// Supporting Structures

// 2D grid index
struct CellIndex {
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

struct CellIndexHash {
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for the priority queue (min-heap by f_score).
// std::priority_queue is a MAX-heap by default: it keeps the "largest" element on top.
// By saying "a is less important than b when a.f > b.f", we flip it into a MIN-heap, so
// top() is always the node with the smallest f_score: the most promising one.
struct CompareF {
  bool operator()(const AStarNode &a, const AStarNode &b) {
    return a.f_score > b.f_score;
  }
};

// Tunables for how A* scores cells using params
struct PlannerConfig {
  int lethal_cost = 50;
  double lethal_multiplier = 100.0;
  double cost_weight = 5.0;
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    void configure(const PlannerConfig& config) { config_ = config; }

    // Runs A*. Returns true and fills `path` (in the map's frame) on success; returns
    // false if the start/goal is off the map or no route exists.
    bool planPath(const nav_msgs::msg::OccupancyGrid& map,
                  const geometry_msgs::msg::Point& start,
                  const geometry_msgs::msg::Point& goal,
                  nav_msgs::msg::Path& path) const;

  private:
    // World metres -> cell. Returns false if outside the map.
    static bool worldToCell(const nav_msgs::msg::OccupancyGrid& map, double wx, double wy,
                            CellIndex& cell);
    // Cell -> world metres (centre of the cell).
    static geometry_msgs::msg::Point cellToWorld(const nav_msgs::msg::OccupancyGrid& map,
                                                 const CellIndex& cell);

    // Multiplier applied to the distance of a step *into* a cell with this map value.
    // Returns a negative number if the cell cannot be entered at all.
    double traversalMultiplier(int8_t value) const;

    rclcpp::Logger logger_;
    PlannerConfig config_;
};

}  // namespace robot

#endif  // PLANNER_CORE_HPP_
