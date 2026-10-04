#ifndef UAV_NAVIGATION__LOCAL_PLANNER_NODE_HPP_
#define UAV_NAVIGATION__LOCAL_PLANNER_NODE_HPP_

#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <uav_interfaces/msg/avoidance_advice.hpp>
#include <uav_interfaces/msg/obstacle_array.hpp>
#include <uav_interfaces/msg/trajectory3_d.hpp>

#include "uav_navigation/local_avoidance.hpp"

namespace uav_navigation
{

/// Advises on the trajectory the aircraft is actually flying; never commands.
///
/// It watches /planning/trajectory rather than /planning/route on purpose: the
/// spline cuts corners the waypoints do not, and clearance proven on waypoints is
/// not clearance in flight (plan P6 section 2f).
///
/// Two behaviours are worth knowing before changing anything here:
///   1. The library answers Hold whenever it cannot see. This node is the ONE place
///      allowed to soften that into Clear, and only when require_obstacle_feed is
///      false, and only out loud.
///   2. With no plan to check it still publishes, with checked_horizon_m at zero.
///      Silence would be read by the consumer's timeout as a dead advisor.
class LocalPlannerNode : public rclcpp::Node
{
public:
  explicit LocalPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void tick();

  /// Trajectory points from the one nearest the aircraft to the end.
  std::vector<Vec3> routeAhead(const Vec3 & position) const;

  void publish(const AvoidanceResult & result);
  void publishNothingToCheck(const std::string & reason);

  /// Aged against our own clock, so it keeps running on the ticks that skip update().
  double mapAgeSeconds() const;

  std::string uav_id_;
  bool require_obstacle_feed_ = false;
  double advise_hz_ = 10.0;
  double odometry_timeout_ = 1.0;

  Costmap costmap_;
  LocalAvoidance advisor_;

  nav_msgs::msg::Odometry::ConstSharedPtr odometry_;
  uav_interfaces::msg::ObstacleArray::ConstSharedPtr obstacles_;
  uav_interfaces::msg::Trajectory3D::ConstSharedPtr trajectory_;

  rclcpp::Publisher<uav_interfaces::msg::AvoidanceAdvice>::SharedPtr advice_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::ObstacleArray>::SharedPtr obstacle_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::Trajectory3D>::SharedPtr trajectory_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace uav_navigation

#endif  // UAV_NAVIGATION__LOCAL_PLANNER_NODE_HPP_
