#include "uav_navigation/route_planner_node.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <string>
#include <vector>

#include "uav_interfaces/msg/obstacle.hpp"

namespace uav_navigation
{

using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;
using uav_interfaces::msg::ObstacleArray;
using uav_interfaces::msg::Path3D;

namespace
{
constexpr double kNanoToSecond = 1e-9;

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * kNanoToSecond;
}
}  // namespace

RoutePlannerNode::RoutePlannerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("route_planner_node", options)
{
  uav_id_ = declare_parameter<std::string>("uav_id", "uav0");

  CostmapLimits map_limits;
  map_limits.width_m = declare_parameter<double>("costmap.width_m", map_limits.width_m);
  map_limits.resolution_m =
    declare_parameter<double>("costmap.resolution_m", map_limits.resolution_m);
  map_limits.drone_radius_m =
    declare_parameter<double>("costmap.drone_radius_m", map_limits.drone_radius_m);
  map_limits.cost_scaling =
    declare_parameter<double>("costmap.cost_scaling", map_limits.cost_scaling);
  map_limits.corner_cut_m =
    declare_parameter<double>("costmap.corner_cut_m", map_limits.corner_cut_m);
  map_limits.obstacle_timeout_sec =
    declare_parameter<double>("costmap.obstacle_timeout_sec", map_limits.obstacle_timeout_sec);
  map_limits.flight_band_m =
    declare_parameter<double>("costmap.flight_band_m", map_limits.flight_band_m);

  RouteLimits route_limits;
  route_limits.soft_cost_gain =
    declare_parameter<double>("route.soft_cost_gain", route_limits.soft_cost_gain);
  route_limits.goal_tolerance_m =
    declare_parameter<double>("route.goal_tolerance_m", route_limits.goal_tolerance_m);
  route_limits.window_margin_m =
    declare_parameter<double>("route.window_margin_m", route_limits.window_margin_m);
  route_limits.projection_timeout_sec =
    declare_parameter<double>("route.projection_timeout_sec", route_limits.projection_timeout_sec);

  // Sim may fly with no map at all; real flight may not (plan P6 decision 4).
  require_obstacle_feed_ = declare_parameter<bool>("require_obstacle_feed", false);
  map_timeout_sec_ = declare_parameter<double>("map_timeout_sec", 3.0);
  plan_hz_ = declare_parameter<double>("plan_hz", 5.0);
  spline_tighten_rounds_ =
    static_cast<unsigned>(declare_parameter<int>("spline_tighten_rounds", 4));

  std::string reason;
  costmap_ = Costmap::build(map_limits, reason);
  if (!costmap_.valid()) {
    RCLCPP_FATAL(get_logger(), "costmap limits refused: %s", reason.c_str());
    throw std::runtime_error("costmap limits refused: " + reason);
  }
  planner_ = RoutePlanner(route_limits);

  const std::string base = "/uav/" + uav_id_;
  route_publisher_ = create_publisher<Path3D>(base + "/planning/route", rclcpp::QoS(1));

  odometry_subscription_ = create_subscription<Odometry>(
    base + "/state/odometry_fused", rclcpp::SensorDataQoS(),
    [this](Odometry::ConstSharedPtr message) {odometry_ = message;});

  obstacle_subscription_ = create_subscription<ObstacleArray>(
    base + "/world/obstacle_map_local", rclcpp::SensorDataQoS(),
    [this](ObstacleArray::ConstSharedPtr message) {obstacles_ = message;});

  goal_subscription_ = create_subscription<PoseStamped>(
    base + "/planning/route_goal", rclcpp::QoS(1),
    [this](PoseStamped::ConstSharedPtr message) {goal_ = message;});

  timer_ = rclcpp::create_timer(
    this, get_clock(),
    rclcpp::Duration::from_seconds(1.0 / std::max(0.1, plan_hz_)),
    [this]() {tick();});

  RCLCPP_INFO(
    get_logger(), "route planner up for %s, map %s",
    uav_id_.c_str(), require_obstacle_feed_ ? "REQUIRED" : "optional");
}

builtin_interfaces::msg::Time RoutePlannerNode::servedGoalStamp() const
{
  return goal_ ? goal_->header.stamp : builtin_interfaces::msg::Time();
}

void RoutePlannerNode::publishFailure(
  uint8_t plan_state, const std::string & reason, const std::string & planner_name)
{
  Path3D message;
  message.header.stamp = now();
  message.header.frame_id = "odom";
  message.uav_id = uav_id_;
  message.is_valid = false;
  message.plan_state = plan_state;
  message.reason = reason;
  message.planner_name = planner_name;
  message.goal_stamp = servedGoalStamp();
  route_publisher_->publish(message);

  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "no route: %s", reason.c_str());
}

void RoutePlannerNode::tick()
{
  if (!odometry_) {
    publishFailure(Path3D::PLAN_STATE_NO_GOAL, "waiting for odometry_fused");
    return;
  }
  if (!goal_) {
    return;  // no goal is not a fault, so it is not reported as one
  }

  const double now_seconds = now().seconds();
  const Vec3 start{
    odometry_->pose.pose.position.x,
    odometry_->pose.pose.position.y,
    odometry_->pose.pose.position.z};

  std::vector<CostmapObstacle> obstacles;
  // -inf means no report has ever arrived, which is NOT the same as a report that
  // carried nothing; the costmap needs the two told apart.
  double feed_stamp = -std::numeric_limits<double>::infinity();
  if (obstacles_) {
    feed_stamp = stampSeconds(obstacles_->header.stamp);
    obstacles.reserve(obstacles_->obstacles.size());
    for (const uav_interfaces::msg::Obstacle & entry : obstacles_->obstacles) {
      CostmapObstacle obstacle;
      obstacle.center = Vec3{entry.center.x, entry.center.y, entry.center.z};
      obstacle.size = Vec3{entry.size.x, entry.size.y, entry.size.z};
      obstacle.position_uncertainty = entry.position_uncertainty;
      obstacle.stamp_seconds = feed_stamp;
      obstacles.push_back(obstacle);
    }
  }

  std::string reason;
  costmap_.update(obstacles, start, now_seconds, feed_stamp, reason);

  // Silence is not free space: the map says how old it is, and on real hardware a
  // stale map is a stop, not a licence to keep planning through open air.
  if (require_obstacle_feed_ && costmap_.lastInputAge() > map_timeout_sec_) {
    publishFailure(Path3D::PLAN_STATE_FAILED, "obstacle map is stale or absent");
    return;
  }

  const Vec3 goal{
    goal_->pose.position.x, goal_->pose.position.y, goal_->pose.position.z};
  const RouteResult route = planner_.plan(costmap_, start, goal, now_seconds);
  if (!route.usable() && route.status != RouteStatus::AlreadyThere) {
    publishFailure(Path3D::PLAN_STATE_FAILED, route.reason, "a_star");
    return;
  }

  const std::string planner_name = route.goal_was_projected ? "a_star:projected" : "a_star";
  std::string tighten_reason;
  const std::vector<Vec3> waypoints = tightenForSpline(
    costmap_, route.waypoints, TrajectoryLimits(), spline_tighten_rounds_, tighten_reason);
  if (!tighten_reason.empty()) {
    publishFailure(
      Path3D::PLAN_STATE_FAILED, "spline would breach the map: " + tighten_reason, planner_name);
    return;
  }

  Path3D message;
  message.header.stamp = now();
  message.header.frame_id = "odom";
  message.uav_id = uav_id_;
  message.is_valid = true;
  message.plan_state = Path3D::PLAN_STATE_VALID;
  message.planner_name = planner_name;
  // Echoes which question this answers. Without it a consumer can only filter by
  // arrival time, and at 5 Hz a route for the PREVIOUS goal still lands inside the
  // window after a new one was asked.
  message.goal_stamp = servedGoalStamp();
  message.waypoints.reserve(waypoints.size());
  for (const Vec3 & waypoint : waypoints) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = waypoint.x;
    pose.position.y = waypoint.y;
    pose.position.z = waypoint.z;
    pose.orientation.w = 1.0;
    message.waypoints.push_back(pose);
  }
  route_publisher_->publish(message);
}

}  // namespace uav_navigation
