#include "control_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot {

ControlCore::ControlCore(const rclcpp::Logger& logger) : logger_(logger) {}

// Takes the path and pose and returns the control output needed to follow the path
ControlOutput ControlCore::computeCommand(const nav_msgs::msg::Path& path,
                                          const geometry_msgs::msg::Pose& robot_pose) const {
  ControlOutput out;  // Twist defaults to all zeros = "stop"

  // Edge case: nothing to follow
  if (path.poses.empty()) {
    return out; // Stays still
  }

  const double yaw = extractYaw(robot_pose.orientation);

  // Checking if the robot is at the goal
  const auto& goal = path.poses.back().pose.position;
  const double dist_to_goal =
    computeDistance(robot_pose.position.x, robot_pose.position.y, goal.x, goal.y);
  if (dist_to_goal < config_.goal_tolerance) {
    out.goal_reached = true;
    return out;  // zero velocity
  }

  // Calculating the point on the robot we are actually steering
  const double px = robot_pose.position.x - config_.pursuit_point_offset * std::cos(yaw);
  const double py = robot_pose.position.y - config_.pursuit_point_offset * std::sin(yaw);

  // Finding the lookahead point to follow
  const auto target = findLookaheadPoint(path, px, py);
  if (!target) {
    return out;
  }

  // Target -> Robot frame
  const double dx = target->x - px; 
  const double dy = target->y - py; 
  const double lx = std::cos(yaw) * dx + std::sin(yaw) * dy; // How far ahead of the robot
  const double ly = -std::sin(yaw) * dx + std::cos(yaw) * dy; // How far left of the robot

  // Calculating angle between direction robot is facing and target
  const double alpha = std::atan2(ly, lx);

  // If angle between robot and target is too large, spin in place as opposed to big loop
  if (std::abs(alpha) > config_.rotate_in_place_angle) {
    out.cmd.linear.x = 0.0;
    out.cmd.angular.z = std::copysign(config_.max_angular_speed, alpha);
    return out;
  }

  // Slow down as robot aproaches goal
  double v = std::min(config_.linear_speed, dist_to_goal);
  v = std::max(v, config_.min_linear_speed);

  // Determining curvature and angular velocity
  const double l_squared = lx * lx + ly * ly;
  // Guard against dividing by ~0 if the target is right on top of us.
  const double curvature = (l_squared > 1e-6) ? (2.0 * ly / l_squared) : 0.0;
  double omega = v * curvature;

  // Clamping turning rate (and turning speed to preserve curvature) if curve is too sharp
  if (std::abs(omega) > config_.max_angular_speed) {
    const double scale = config_.max_angular_speed / std::abs(omega);
    omega *= scale;
    v *= scale;
  }

  out.cmd.linear.x = v;
  out.cmd.angular.z = omega;
  return out;
}

// Determines the lookahead point and returns it as a message containing the point
std::optional<geometry_msgs::msg::Point> ControlCore::findLookaheadPoint(
  const nav_msgs::msg::Path& path, double px, double py) const {
  if (path.poses.empty()) {
    return std::nullopt;
  }

  // Determining which point on the path is the closest to the robot
  size_t closest = 0;
  double closest_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto& p = path.poses[i].pose.position;
    const double d = computeDistance(px, py, p.x, p.y);
    if (d < closest_dist) {
      closest_dist = d;
      closest = i;
    }
  }

  // Returning the closest point on the path that is >= to the lookahead distance
  for (size_t i = closest; i < path.poses.size(); ++i) {
    const auto& p = path.poses[i].pose.position;
    if (computeDistance(px, py, p.x, p.y) >= config_.lookahead_distance) {
      return p;
    }
  }

  // Returning the goal as the lookahead point if no point was found (.'. goal 
  // closer than lookahead distance)
  return path.poses.back().pose.position;
}

double ControlCore::computeDistance(double ax, double ay, double bx, double by) {
  return std::hypot(bx - ax, by - ay);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace robot
