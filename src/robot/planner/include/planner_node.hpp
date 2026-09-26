#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "planner_core.hpp"


class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    // Runs A* from the current pose to the goal and publishes the result.
    void planPath();
    // Publishes a path with no poses. The control node interprets this as "stop".
    void publishEmptyPath();
    bool goalReached() const;

    robot::PlannerCore planner_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_ = State::WAITING_FOR_GOAL;

    // Latest data from each subscription, plus flags saying whether we've got any yet.
    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PointStamped goal_;
    geometry_msgs::msg::Pose robot_pose_;
    bool have_map_ = false;
    bool have_odom_ = false;

    // Timing for the timeout / periodic replanning.
    rclcpp::Time goal_start_time_;
    rclcpp::Time last_plan_time_;

    // Parameters
    std::string map_frame_;
    double goal_tolerance_ = 0.5;
    double goal_timeout_s_ = 120.0;
    double replan_period_s_ = 2.0;
};

#endif  // PLANNER_NODE_HPP_
