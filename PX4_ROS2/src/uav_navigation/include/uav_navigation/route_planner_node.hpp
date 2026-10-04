#ifndef UAV_NAVIGATION__ROUTE_PLANNER_NODE_HPP_
#define UAV_NAVIGATION__ROUTE_PLANNER_NODE_HPP_

#include <cstdint>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <uav_interfaces/msg/obstacle_array.hpp>
#include <uav_interfaces/msg/path3_d.hpp>

#include "uav_navigation/route_planner.hpp"

namespace uav_navigation
{

/// Publishes a global route toward the current goal, replanned on a timer.
///
/// Reads only odometry_fused (R4) and the world-frame obstacle map. A failed plan
/// is published as an invalid Path3D carrying plan_state + reason, never as
/// silence: a consumer that sees nothing cannot tell "no route" from "node died".
class RoutePlannerNode : public rclcpp::Node
{
public:
  explicit RoutePlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void tick();
  /// The route_goal stamp this node is currently planning for; zero when none.
  builtin_interfaces::msg::Time servedGoalStamp() const;

  void publishFailure(
    uint8_t plan_state, const std::string & reason, const std::string & planner_name = "");

  std::string uav_id_;
  bool require_obstacle_feed_ = false;
  double map_timeout_sec_ = 3.0;
  double plan_hz_ = 5.0;
  unsigned spline_tighten_rounds_ = 4;

  Costmap costmap_;
  RoutePlanner planner_;

  nav_msgs::msg::Odometry::ConstSharedPtr odometry_;
  uav_interfaces::msg::ObstacleArray::ConstSharedPtr obstacles_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr goal_;

  rclcpp::Publisher<uav_interfaces::msg::Path3D>::SharedPtr route_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::ObstacleArray>::SharedPtr obstacle_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace uav_navigation

#endif  // UAV_NAVIGATION__ROUTE_PLANNER_NODE_HPP_
