#include "uav_navigation/local_planner_node.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "uav_interfaces/msg/obstacle.hpp"

namespace uav_navigation
{

using nav_msgs::msg::Odometry;
using uav_interfaces::msg::AvoidanceAdvice;
using uav_interfaces::msg::ObstacleArray;
using uav_interfaces::msg::Trajectory3D;

namespace
{
constexpr double kNanoToSecond = 1e-9;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * kNanoToSecond;
}

uint8_t encode(Advice advice)
{
  switch (advice) {
    case Advice::Clear: return AvoidanceAdvice::ADVICE_CLEAR;
    case Advice::Escape: return AvoidanceAdvice::ADVICE_ESCAPE;
    default: return AvoidanceAdvice::ADVICE_HOLD;
  }
}
}  // namespace

LocalPlannerNode::LocalPlannerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("local_planner_node", options)
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

  AvoidanceLimits avoid_limits;
  avoid_limits.horizon_m = declare_parameter<double>("avoid.horizon_m", avoid_limits.horizon_m);
  avoid_limits.sample_step_m =
    declare_parameter<double>("avoid.sample_step_m", avoid_limits.sample_step_m);
  avoid_limits.clearance_scan_m =
    declare_parameter<double>("avoid.clearance_scan_m", avoid_limits.clearance_scan_m);
  avoid_limits.spiral_growth_m =
    declare_parameter<double>("avoid.spiral_growth_m", avoid_limits.spiral_growth_m);
  avoid_limits.max_escape_climb_m =
    declare_parameter<double>("avoid.max_escape_climb_m", avoid_limits.max_escape_climb_m);
  avoid_limits.allow_descent =
    declare_parameter<bool>("avoid.allow_descent", avoid_limits.allow_descent);
  avoid_limits.map_timeout_sec =
    declare_parameter<double>("avoid.map_timeout_sec", avoid_limits.map_timeout_sec);
  avoid_limits.max_spiral_steps =
    declare_parameter<int>("avoid.max_spiral_steps", avoid_limits.max_spiral_steps);

  require_obstacle_feed_ = declare_parameter<bool>("require_obstacle_feed", false);
  advise_hz_ = declare_parameter<double>("advise_hz", 10.0);
  // An advisor that keeps advising on a dead pose is worse than one that stops.
  odometry_timeout_ = declare_parameter<double>("odometry_timeout_sec", 1.0);
  if (!(std::isfinite(odometry_timeout_) && odometry_timeout_ > 0.0)) {
    RCLCPP_FATAL(get_logger(), "odometry_timeout_sec must be finite and positive");
    throw std::runtime_error("odometry_timeout_sec must be finite and positive");
  }

  std::string reason;
  costmap_ = Costmap::build(map_limits, reason);
  if (!costmap_.valid()) {
    RCLCPP_FATAL(get_logger(), "costmap limits refused: %s", reason.c_str());
    throw std::runtime_error("costmap limits refused: " + reason);
  }

  // A typo in yaml must stop the node, not quietly produce a no-op advisor.
  advisor_ = LocalAvoidance::build(avoid_limits, reason);
  if (!advisor_.valid()) {
    RCLCPP_FATAL(get_logger(), "avoidance limits refused: %s", reason.c_str());
    throw std::runtime_error("avoidance limits refused: " + reason);
  }

  const std::string base = "/uav/" + uav_id_;
  advice_publisher_ =
    create_publisher<AvoidanceAdvice>(base + "/planning/avoidance", rclcpp::QoS(1));

  odometry_subscription_ = create_subscription<Odometry>(
    base + "/state/odometry_fused", rclcpp::SensorDataQoS(),
    [this](Odometry::ConstSharedPtr message) {odometry_ = message;});

  obstacle_subscription_ = create_subscription<ObstacleArray>(
    base + "/world/obstacle_map_local", rclcpp::SensorDataQoS(),
    [this](ObstacleArray::ConstSharedPtr message) {obstacles_ = message;});

  trajectory_subscription_ = create_subscription<Trajectory3D>(
    base + "/planning/trajectory", rclcpp::QoS(1).transient_local(),
    [this](Trajectory3D::ConstSharedPtr message) {trajectory_ = message;});

  timer_ = rclcpp::create_timer(
    this, get_clock(),
    rclcpp::Duration::from_seconds(1.0 / std::max(0.1, advise_hz_)),
    [this]() {tick();});

  RCLCPP_INFO(
    get_logger(), "local planner up for %s, map %s",
    uav_id_.c_str(), require_obstacle_feed_ ? "REQUIRED" : "optional");
}

std::vector<Vec3> LocalPlannerNode::routeAhead(const Vec3 & position) const
{
  std::vector<Vec3> ahead;
  if (!trajectory_ || !trajectory_->is_valid || trajectory_->points.size() < 2) {
    return ahead;
  }

  std::size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < trajectory_->points.size(); ++index) {
    const auto & point = trajectory_->points[index].position;
    const double gap = distance3(position, Vec3{point.x, point.y, point.z});
    if (gap < best) {
      best = gap;
      nearest = index;
    }
  }

  ahead.reserve(trajectory_->points.size() - nearest);
  for (std::size_t index = nearest; index < trajectory_->points.size(); ++index) {
    const auto & point = trajectory_->points[index].position;
    ahead.push_back(Vec3{point.x, point.y, point.z});
  }
  return ahead;
}

double LocalPlannerNode::mapAgeSeconds() const
{
  const double age = now().seconds() - costmap_.lastInputStamp();
  return age >= 0.0 ? age : std::numeric_limits<double>::infinity();
}

void LocalPlannerNode::publishNothingToCheck(const std::string & reason)
{
  AvoidanceAdvice message;
  message.header.stamp = now();
  message.header.frame_id = "odom";
  message.uav_id = uav_id_;
  message.advice = AvoidanceAdvice::ADVICE_CLEAR;
  message.reason = reason;
  message.escape_point.x = kNaN;
  message.escape_point.y = kNaN;
  message.escape_point.z = kNaN;
  message.clearance_m = 0.0F;
  // Zero horizon is the honest way to say "nothing was checked"; a consumer that
  // is flying under avoidance must read this as no permission, not as clear air.
  message.checked_horizon_m = 0.0F;
  // Not lastInputAge(): this branch skipped update(), so that number stands still
  // and a dead feed would keep reporting the freshness of its last good tick.
  const double age = mapAgeSeconds();
  message.map_fresh = age <= advisor_.limits().map_timeout_sec;
  message.map_age_sec = static_cast<float>(age);
  advice_publisher_->publish(message);
}

void LocalPlannerNode::publish(const AvoidanceResult & result)
{
  AvoidanceAdvice message;
  message.header.stamp = now();
  message.header.frame_id = "odom";
  message.uav_id = uav_id_;
  message.advice = encode(result.advice);
  message.reason = result.reason;
  message.escape_point.x = result.escape_point.x;
  message.escape_point.y = result.escape_point.y;
  message.escape_point.z = result.escape_point.z;
  message.clearance_m = static_cast<float>(result.clearance_m);
  message.checked_horizon_m = static_cast<float>(result.checked_horizon_m);
  message.map_fresh = result.map_fresh;
  message.map_age_sec = static_cast<float>(result.map_age_sec);
  advice_publisher_->publish(message);

  if (result.advice != Advice::Clear) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "avoidance: %s", result.reason.c_str());
  }
}

void LocalPlannerNode::tick()
{
  if (!odometry_) {
    publishNothingToCheck("waiting for odometry_fused");
    return;
  }

  const double now_seconds = now().seconds();
  const double odometry_age = now_seconds - stampSeconds(odometry_->header.stamp);
  if (!(odometry_age >= 0.0) || odometry_age > odometry_timeout_) {
    // Advising on a pose nobody stands behind is how an advisor clears a wall.
    publishNothingToCheck("odometry_fused went stale");
    return;
  }

  const Vec3 position{
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
  costmap_.update(obstacles, position, now_seconds, feed_stamp, reason);
  if (costmap_.rejected() > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "costmap dropped %u obstacle(s): %s",
      costmap_.rejected(), reason.c_str());
  }

  const std::vector<Vec3> ahead = routeAhead(position);
  if (ahead.size() < 2) {
    publishNothingToCheck("no active plan to check");
    return;
  }

  AvoidanceResult result = advisor_.advise(costmap_, ahead, position);

  // The one place a Hold may be softened, and only for a map we never demanded.
  if (result.advice == Advice::Hold && !result.map_fresh && !require_obstacle_feed_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "flying unguarded: %s (require_obstacle_feed is false)", result.reason.c_str());
    result.advice = Advice::Clear;
    result.checked_horizon_m = 0.0;
    result.reason = "map not required, nothing checked: " + result.reason;
  }

  publish(result);
}

}  // namespace uav_navigation
