#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

// Tunables for Pure Pursuit
struct ControlConfig {
  double lookahead_distance = 1.5;  // metres ahead on the path to aim for
  double goal_tolerance = 0.5;      // stop when this close (metres) to the path's end
  double linear_speed = 1.0;        // cruising forward speed, m/s
  double min_linear_speed = 0.2;    // never crawl slower than this while still moving
  double max_angular_speed = 1.5;   // rad/s cap on turning rate
  // Largest heading error (radians) at which we still drive forward. Beyond this the
  // target is off to the side or behind us, so we turn on the spot first.
  double rotate_in_place_angle = 1.2;
  // Distance (metres) from the odometry point back to the robot's centre of rotation.
  // See the explanation in control_core.cpp.
  double pursuit_point_offset = 1.3;
};

// Result of one control step
struct ControlOutput {
  geometry_msgs::msg::Twist cmd;  // velocity to send to the robot
  bool goal_reached = false;      // true once within goal_tolerance of the end
};

class ControlCore {
  public:
    // Constructor, takes the node's RCLCPP logger to enable logging to terminal
    explicit ControlCore(const rclcpp::Logger& logger);

    void configure(const ControlConfig& config) { config_ = config; }

    ControlOutput computeCommand(const nav_msgs::msg::Path& path,
                                 const geometry_msgs::msg::Pose& robot_pose) const;

  private:
    // Finds the point on the path the robot should steer toward.
    std::optional<geometry_msgs::msg::Point> findLookaheadPoint(
      const nav_msgs::msg::Path& path, double px, double py) const;

    static double computeDistance(double ax, double ay, double bx, double by);
    static double extractYaw(const geometry_msgs::msg::Quaternion& quat);

    rclcpp::Logger logger_;
    ControlConfig config_;
};

}  // namespace robot

#endif  // CONTROL_CORE_HPP_
