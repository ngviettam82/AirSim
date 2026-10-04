// G-N4a (P6-completion-run.md, task C1): the deterministic gate for the avoidance
// chain. No Gazebo, no PX4. Two REAL nodes are wired together -- local_planner_node
// and navigator_action_server_node -- through the exact topics they use in flight;
// a probe plays the aircraft (plant at PX4's real K_p 0.95, not the 8x-stiffer rig
// other unit tests use on purpose) plus the obstacle/vehicle/offboard sensors.
//
// What this proves that no existing test does: test_local_planner_node.cpp exercises
// the advisor alone against a fake trajectory, and test_navigator_action_server_node
// exercises the navigator alone against a fake advisor. Neither proves the two real
// nodes agree with each other end to end. This file is that missing proof.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <uav_interfaces/action/goto_pose.hpp>
#include <uav_interfaces/action/takeoff.hpp>
#include <uav_interfaces/msg/avoidance_advice.hpp>
#include <uav_interfaces/msg/control_command.hpp>
#include <uav_interfaces/msg/localization_status.hpp>
#include <uav_interfaces/msg/obstacle.hpp>
#include <uav_interfaces/msg/obstacle_array.hpp>
#include <uav_interfaces/msg/offboard_status.hpp>
#include <uav_interfaces/msg/result_code.hpp>
#include <uav_interfaces/msg/trajectory3_d.hpp>
#include <uav_interfaces/msg/vehicle_state.hpp>
#include <uav_interfaces/srv/arm.hpp>
#include <uav_interfaces/srv/disarm.hpp>
#include <uav_interfaces/srv/set_flight_mode.hpp>

#include "uav_navigation/carrot.hpp"
#include "uav_navigation/local_planner_node.hpp"
#include "uav_navigation/route_planner_node.hpp"
#include "uav_navigation/navigator_action_server_node.hpp"

using nav_msgs::msg::Odometry;
using uav_interfaces::action::GotoPose;
using uav_interfaces::action::Takeoff;
using uav_interfaces::msg::AvoidanceAdvice;
using uav_interfaces::msg::ControlCommand;
using uav_interfaces::msg::LocalizationStatus;
using uav_interfaces::msg::Obstacle;
using uav_interfaces::msg::ObstacleArray;
using uav_interfaces::msg::OffboardStatus;
using uav_interfaces::msg::ResultCode;
using uav_interfaces::msg::Trajectory3D;
using uav_interfaces::msg::VehicleState;
using uav_interfaces::srv::Arm;
using uav_interfaces::srv::Disarm;
using uav_interfaces::srv::SetFlightMode;
using uav_navigation::distance3;
using uav_navigation::Vec3;
using namespace std::chrono_literals;

namespace
{

constexpr double kGroundZ = 0.15;
constexpr double kTakeoffAltitude = 1.0;
constexpr double kCruiseZ = kGroundZ + kTakeoffAltitude;
constexpr double kTouchdownSeconds = 1.0;

// Mandated by the task brief: PX4's real position loop, not the 8.4x-stiffer rig
// test_navigator_action_server_node.cpp uses for its protocol tests.
constexpr double kPlantGain = 0.95;
constexpr double kPlantMaxSpeed = 3.0;
constexpr double kPlantMaxVerticalSpeed = 2.5;
constexpr double kPlantMaxTickSeconds = 0.2;

// README S:7's own margin for "still riding a trajectory, not the unbounded carrot"
// (c3: <=3 m/s^2 against max_acceleration 1.0). Reused here for the same reason: it
// is a shipped, already-justified number, not a fresh guess.
constexpr double kAccelerationEnvelopeSlack = 3.0;

// The pending navigator change (route re-asked after an escape, per the coordinator's
// note) will add a few more /planning/trajectory publishes per escape. This slack is
// deliberately generous so the replan-count assertion survives that change; it is a
// livelock guard, not a tight bound.
constexpr int kReplanSlack = 6;

struct CommandSample
{
  double seconds = 0.0;
  Vec3 position;
};

struct EmittedMotion
{
  double peak_speed = 0.0;
  double peak_acceleration = 0.0;
  std::size_t gaps = 0;
};

/// Copied from test_navigator_action_server_node.cpp on purpose (not shared through a
/// header): acceleration must come from the POSITIONS actually put on the wire, never
/// from a trajectory's own analytic field -- that field read 0.897 m/s^2 while the
/// real step was 55.7 (P6.2, memory S:1). This is the proven measurement, reused.
EmittedMotion measureEmitted(const std::vector<CommandSample> & trace, double max_gap)
{
  EmittedMotion motion;
  std::vector<double> speeds;
  std::vector<double> centres;
  std::vector<Vec3> velocities;

  for (std::size_t index = 1; index < trace.size(); ++index) {
    const double gap = trace[index].seconds - trace[index - 1].seconds;
    if (!(gap > 1e-3)) {
      continue;
    }
    const Vec3 velocity{
      (trace[index].position.x - trace[index - 1].position.x) / gap,
      (trace[index].position.y - trace[index - 1].position.y) / gap,
      (trace[index].position.z - trace[index - 1].position.z) / gap};
    velocities.push_back(velocity);
    speeds.push_back(
      std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z));
    centres.push_back(0.5 * (trace[index].seconds + trace[index - 1].seconds));
  }

  motion.gaps = speeds.size();
  for (const double speed : speeds) {
    motion.peak_speed = std::max(motion.peak_speed, speed);
  }
  for (std::size_t index = 1; index < velocities.size(); ++index) {
    const double gap = centres[index] - centres[index - 1];
    if (!(gap > 1e-3) || gap > max_gap) {
      continue;                         // a starved tick reads as acceleration; skip it
    }
    const Vec3 change{
      velocities[index].x - velocities[index - 1].x,
      velocities[index].y - velocities[index - 1].y,
      velocities[index].z - velocities[index - 1].z};
    motion.peak_acceleration = std::max(
      motion.peak_acceleration,
      std::sqrt(change.x * change.x + change.y * change.y + change.z * change.z) / gap);
  }
  return motion;
}

/// Distance from a point to the surface of an axis-aligned box; 0 if inside.
double boxSurfaceDistance(const Vec3 & point, const Vec3 & center, const Vec3 & half)
{
  const double dx = std::max(std::abs(point.x - center.x) - half.x, 0.0);
  const double dy = std::max(std::abs(point.y - center.y) - half.y, 0.0);
  const double dz = std::max(std::abs(point.z - center.z) - half.z, 0.0);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

struct AdviceLogEntry
{
  double stamp_seconds = 0.0;
  uint8_t advice = AvoidanceAdvice::ADVICE_CLEAR;
};

struct PlanLogEntry
{
  double stamp_seconds = 0.0;
  bool is_valid = false;
  Vec3 endpoint;
  std::size_t point_count = 0;
};

class AvoidanceChainFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // Own namespace per test instance: a lingering subscription from a previous test
    // must never see this test's obstacle/odometry traffic (same discipline as
    // test_navigator_action_server_node.cpp's NavigatorFixture).
    static std::atomic<int> instances{0};
    uav_id_ = "uav" + std::to_string(instances.fetch_add(1));
    const std::string prefix = "/uav/" + uav_id_;

    rclcpp::NodeOptions navigator_options;
    navigator_options.parameter_overrides({
      rclcpp::Parameter("use_sim_time", false),
      rclcpp::Parameter("uav_id", uav_id_),
      // Everything timing-relevant below is the SHIPPED value from
      // navigation_params.yaml: this gate measures the chain's real latency, so it
      // must run at the real rates, not a test-sped-up rig.
      rclcpp::Parameter("stream_hz", stream_hz_),
      rclcpp::Parameter("max_speed", max_speed_),
      rclcpp::Parameter("max_vertical_speed", max_vertical_speed_),
      rclcpp::Parameter("acceptance_radius", 0.3),
      rclcpp::Parameter("max_acceleration", max_acceleration_),
      rclcpp::Parameter("progress_rate_hz", progress_rate_hz_),
      rclcpp::Parameter("advice_timeout_sec", advice_timeout_sec_),
      rclcpp::Parameter("route_timeout_sec", route_timeout_sec_),
      rclcpp::Parameter("route_fresh_sec", route_fresh_sec_),
      rclcpp::Parameter("max_escape_deviation_m", max_escape_deviation_m_),
      rclcpp::Parameter("escape_replan_interval_sec", escape_replan_interval_sec_),
      rclcpp::Parameter("escape_refresh_m", escape_refresh_m_),
      rclcpp::Parameter("require_obstacle_feed", navigator_require_obstacle_feed_),
      // Loosened only where a busy CI machine, not the chain under test, would
      // otherwise flake the gate.
      rclcpp::Parameter("odometry_timeout_sec", 10.0),
      rclcpp::Parameter("startup_timeout_sec", 10.0),
      rclcpp::Parameter("takeoff_stall_grace_sec", 5.0),
      rclcpp::Parameter("leash_stall_fault_sec", 3.0),
      // Real ceiling is 240 s; bounded here so a livelocked gate fails in minutes,
      // not never. Does not change anything this test measures.
      rclcpp::Parameter("goto_timeout_sec", goto_timeout_sec_),
    });
    node_ = uav_navigation::createNavigatorActionServerNode(navigator_options);

    rclcpp::NodeOptions planner_options;
    planner_options.parameter_overrides({
      rclcpp::Parameter("use_sim_time", false),
      rclcpp::Parameter("uav_id", uav_id_),
      rclcpp::Parameter("costmap.width_m", 25.0),
      rclcpp::Parameter("costmap.resolution_m", 0.25),
      rclcpp::Parameter("costmap.drone_radius_m", drone_radius_m_),
      rclcpp::Parameter("costmap.cost_scaling", 1.0),
      rclcpp::Parameter("costmap.corner_cut_m", 0.0),
      rclcpp::Parameter("costmap.obstacle_timeout_sec", 3.0),
      rclcpp::Parameter("costmap.flight_band_m", flight_band_m_),
      rclcpp::Parameter("avoid.horizon_m", 6.0),
      rclcpp::Parameter("avoid.sample_step_m", 0.1),
      rclcpp::Parameter("avoid.clearance_scan_m", 2.0),
      rclcpp::Parameter("avoid.spiral_growth_m", 0.5),
      rclcpp::Parameter("avoid.max_escape_climb_m", 2.0),
      rclcpp::Parameter("avoid.allow_descent", false),
      rclcpp::Parameter("avoid.map_timeout_sec", 3.0),
      rclcpp::Parameter("avoid.max_spiral_steps", 24),
      rclcpp::Parameter("require_obstacle_feed", planner_require_obstacle_feed_),
      rclcpp::Parameter("advise_hz", advise_hz_),
    });
    planner_ = std::make_shared<uav_navigation::LocalPlannerNode>(planner_options);

    // Off unless a fixture asks. Claims 1 to 6 were measured without it and adding a
    // third node to those flights would change the latency they report.
    if (with_route_planner_) {
      rclcpp::NodeOptions route_options;
      route_options.parameter_overrides({
        rclcpp::Parameter("use_sim_time", false),
        rclcpp::Parameter("uav_id", uav_id_),
        rclcpp::Parameter("costmap.width_m", 25.0),
        rclcpp::Parameter("costmap.resolution_m", 0.25),
        rclcpp::Parameter("costmap.drone_radius_m", drone_radius_m_),
        rclcpp::Parameter("costmap.cost_scaling", 1.0),
        rclcpp::Parameter("costmap.corner_cut_m", 0.0),
        rclcpp::Parameter("costmap.obstacle_timeout_sec", 3.0),
        rclcpp::Parameter("costmap.flight_band_m", flight_band_m_),
        rclcpp::Parameter("require_obstacle_feed", planner_require_obstacle_feed_),
        rclcpp::Parameter("plan_hz", 5.0),
      });
      route_planner_ = std::make_shared<uav_navigation::RoutePlannerNode>(route_options);
    }

    probe_ = std::make_shared<rclcpp::Node>("avoidance_chain_probe");
    position_ = Vec3{0.0, 0.0, kGroundZ};

    sim_group_ = probe_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions command_options;
    command_options.callback_group =
      probe_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    command_subscription_ = probe_->create_subscription<ControlCommand>(
      prefix + "/control/cmd_mission", 50,
      [this](const ControlCommand::SharedPtr message) {onCommand(*message);}, command_options);

    plan_subscription_ = probe_->create_subscription<Trajectory3D>(
      prefix + "/planning/trajectory", rclcpp::QoS(5).reliable().transient_local(),
      [this](const Trajectory3D::SharedPtr message) {onPlan(*message);}, command_options);

    advice_subscription_ = probe_->create_subscription<AvoidanceAdvice>(
      prefix + "/planning/avoidance", rclcpp::QoS(10),
      [this](const AvoidanceAdvice::SharedPtr message) {onAdvice(*message);}, command_options);

    odometry_publisher_ = probe_->create_publisher<Odometry>(prefix + "/state/odometry_fused", 20);
    offboard_publisher_ =
      probe_->create_publisher<OffboardStatus>(prefix + "/backend/offboard_status", 10);
    vehicle_publisher_ = probe_->create_publisher<VehicleState>(prefix + "/state/vehicle", 10);
    localization_publisher_ = probe_->create_publisher<LocalizationStatus>(
      prefix + "/state/localization_status", 10);
    obstacle_publisher_ = probe_->create_publisher<ObstacleArray>(
      prefix + "/world/obstacle_map_local", rclcpp::SensorDataQoS());

    arm_service_ = probe_->create_service<Arm>(
      prefix + "/backend/arm",
      [this](const std::shared_ptr<Arm::Request>, std::shared_ptr<Arm::Response> response) {
        armed_ = true;
        flight_mode_ = VehicleState::FLIGHT_MODE_OFFBOARD;
        response->success = true;
        response->result_code = ResultCode::SUCCEEDED;
        response->message = "armed";
      });
    disarm_service_ = probe_->create_service<Disarm>(
      prefix + "/backend/disarm",
      [this](const std::shared_ptr<Disarm::Request>, std::shared_ptr<Disarm::Response> response) {
        armed_ = false;
        response->success = true;
        response->result_code = ResultCode::SUCCEEDED;
        response->message = "disarmed";
      });
    mode_service_ = probe_->create_service<SetFlightMode>(
      prefix + "/backend/set_mode",
      [this](
        const std::shared_ptr<SetFlightMode::Request> request,
        std::shared_ptr<SetFlightMode::Response> response) {
        flight_mode_ = request->mode;
        if (request->mode == VehicleState::FLIGHT_MODE_LAND) {
          land_commanded_at_ = probe_->now();
        }
        response->success = true;
        response->result_code = ResultCode::SUCCEEDED;
        response->message = "mode change confirmed";
        response->current_mode = request->mode;
      });

    sim_timer_ = probe_->create_wall_timer(20ms, [this]() {simulateVehicle();}, sim_group_);
    obstacle_timer_ = probe_->create_wall_timer(
      100ms, [this]() {publishObstacle();}, sim_group_);            // ~10 Hz, matches P6.3's "~7Hz" note in order of magnitude

    takeoff_client_ = rclcpp_action::create_client<Takeoff>(probe_, prefix + "/planning/takeoff");
    goto_client_ = rclcpp_action::create_client<GotoPose>(probe_, prefix + "/planning/goto_pose");

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 4);
    executor_->add_node(node_);
    executor_->add_node(planner_);
    if (route_planner_) {
      executor_->add_node(route_planner_);
    }
    executor_->add_node(probe_);
    spin_thread_ = std::thread([this]() {executor_->spin();});

    ASSERT_TRUE(takeoff_client_->wait_for_action_server(10s));
    ASSERT_TRUE(goto_client_->wait_for_action_server(10s));
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    takeoff_client_.reset();
    goto_client_.reset();
    executor_.reset();
    node_.reset();
    planner_.reset();
    probe_.reset();
  }

  // ------------------------------------------------------------- fake vehicle

  void onCommand(const ControlCommand & command)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    last_command_ = command;
    last_command_time_ = probe_->now();
    ++command_count_;
    if (tracing_) {
      trace_.push_back(
        CommandSample{
          rclcpp::Time(command.header.stamp).seconds(),
          Vec3{command.position.x, command.position.y, command.position.z}});
    }
  }

  void onPlan(const Trajectory3D & message)
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    PlanLogEntry entry;
    entry.stamp_seconds = rclcpp::Time(message.header.stamp).seconds();
    entry.is_valid = message.is_valid;
    entry.point_count = message.points.size();
    if (!message.points.empty()) {
      const auto & back = message.points.back().position;
      entry.endpoint = Vec3{back.x, back.y, back.z};
    }
    plans_.push_back(entry);
  }

  void onAdvice(const AvoidanceAdvice & message)
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    advice_log_.push_back(
      AdviceLogEntry{rclcpp::Time(message.header.stamp).seconds(), message.advice});
  }

  void startTrace()
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    trace_.clear();
    tracing_ = true;
  }

  std::vector<CommandSample> trace()
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return trace_;
  }

  std::vector<AdviceLogEntry> adviceLog()
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    return advice_log_;
  }

  std::vector<PlanLogEntry> plans()
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    return plans_;
  }

  Vec3 lastCommandedPosition()
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return Vec3{last_command_.position.x, last_command_.position.y, last_command_.position.z};
  }

  bool commandsFlowing()
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return command_count_.load() >= 5 && (probe_->now() - last_command_time_).seconds() < 0.3;
  }

  /// Rate-limited first-order lag at PX4's real K_p; caller holds position_mutex_.
  void advancePlant(const Vec3 & setpoint, double dt)
  {
    double vx = kPlantGain * (setpoint.x - position_.x);
    double vy = kPlantGain * (setpoint.y - position_.y);
    double vz = kPlantGain * (setpoint.z - position_.z);

    const double horizontal = std::hypot(vx, vy);
    if (horizontal > kPlantMaxSpeed) {
      vx *= kPlantMaxSpeed / horizontal;
      vy *= kPlantMaxSpeed / horizontal;
    }
    vz = std::max(-kPlantMaxVerticalSpeed, std::min(kPlantMaxVerticalSpeed, vz));

    position_.x += vx * dt;
    position_.y += vy * dt;
    position_.z += vz * dt;
  }

  void simulateVehicle()
  {
    const bool tracking = command_count_.load() > 0;
    const Vec3 setpoint = tracking ? lastCommandedPosition() : Vec3{};

    const rclcpp::Time tick = probe_->now();
    double dt = last_plant_tick_.nanoseconds() == 0 ? 0.02 : (tick - last_plant_tick_).seconds();
    last_plant_tick_ = tick;
    dt = std::min(std::max(dt, 0.001), kPlantMaxTickSeconds);

    Vec3 updated;
    {
      std::lock_guard<std::mutex> lock(position_mutex_);
      if (tracking) {
        advancePlant(setpoint, dt);
      }
      updated = position_;
    }

    Odometry odometry;
    odometry.header.stamp = tick;
    odometry.header.frame_id = "odom";
    odometry.pose.pose.position.x = updated.x;
    odometry.pose.pose.position.y = updated.y;
    odometry.pose.pose.position.z = updated.z;
    odometry.pose.pose.orientation.w = 1.0;
    odometry_publisher_->publish(odometry);

    VehicleState vehicle;
    vehicle.header.stamp = probe_->now();
    vehicle.uav_id = uav_id_;
    vehicle.connected = true;
    vehicle.armed = armed_.load();
    vehicle.flight_mode = flight_mode_.load();
    vehicle_publisher_->publish(vehicle);

    LocalizationStatus localization;
    localization.header.stamp = tick;
    localization.uav_id = uav_id_;
    localization.active_source = LocalizationStatus::SOURCE_FUSED;
    localization.is_valid = true;
    localization.position_uncertainty = 0.02F;
    localization_publisher_->publish(localization);

    OffboardStatus offboard;
    offboard.header.stamp = probe_->now();
    offboard.uav_id = uav_id_;
    offboard.state =
      commandsFlowing() ? OffboardStatus::STATE_ACTIVE : OffboardStatus::STATE_IDLE;
    offboard.setpoint_rate_hz = 50.0F;
    offboard.command_fresh = commandsFlowing();
    offboard.offboard_confirmed = commandsFlowing();
    offboard_publisher_->publish(offboard);

    if (flight_mode_.load() == VehicleState::FLIGHT_MODE_LAND && armed_.load() &&
      (probe_->now() - land_commanded_at_).seconds() > kTouchdownSeconds)
    {
      armed_ = false;
      flight_mode_ = VehicleState::FLIGHT_MODE_HOLD;
    }
  }

  // ------------------------------------------------------------- obstacle feed

  void publishObstacle()
  {
    if (!feed_enabled_.load()) {
      return;
    }
    ObstacleArray array;
    const rclcpp::Time stamp = probe_->now();
    array.header.stamp = stamp;
    array.header.frame_id = "odom";
    for (const Vec3 & centre : obstacleCentres()) {
      Obstacle obstacle;
      obstacle.shape = Obstacle::SHAPE_BOX;
      obstacle.center.x = centre.x;
      obstacle.center.y = centre.y;
      obstacle.center.z = centre.z;
      obstacle.size.x = 2.0 * obstacle_half_.x;
      obstacle.size.y = 2.0 * obstacle_half_.y;
      obstacle.size.z = 2.0 * obstacle_half_.z;
      obstacle.position_uncertainty = 0.0F;
      array.obstacles.push_back(obstacle);
    }
    obstacle_publisher_->publish(array);

    std::lock_guard<std::mutex> lock(log_mutex_);
    obstacle_stamps_.push_back(stamp.seconds());
  }

  /// One box unless a fixture built a wall out of several.
  std::vector<Vec3> obstacleCentres() const
  {
    std::vector<Vec3> centres =
      wall_centres_.empty() ? std::vector<Vec3>{obstacle_center_} : wall_centres_;
    if (surprise_enabled_.load()) {
      // A surprise is one box unless a fixture built a shape out of several
      // (CupFixture needs a concave one; a single box cannot trap anything).
      if (surprise_centres_.empty()) {
        centres.push_back(surprise_centre_);
      } else {
        centres.insert(centres.end(), surprise_centres_.begin(), surprise_centres_.end());
      }
    }
    return centres;
  }

  std::vector<double> obstacleStamps()
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    return obstacle_stamps_;
  }

  // ---------------------------------------------------------- action helpers

  template<typename ActionT>
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr sendGoal(
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
    const typename ActionT::Goal & goal, double timeout = 10.0)
  {
    auto future = client->async_send_goal(goal);
    if (future.wait_for(std::chrono::duration<double>(timeout)) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  template<typename ActionT>
  bool waitForResult(
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
    const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr & handle,
    typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & out, double timeout)
  {
    auto future = client->async_get_result(handle);
    if (future.wait_for(std::chrono::duration<double>(timeout)) != std::future_status::ready) {
      return false;
    }
    out = future.get();
    return true;
  }

  bool flyToCruise()
  {
    Takeoff::Goal goal;
    goal.target_altitude = static_cast<float>(kTakeoffAltitude);
    auto handle = sendGoal<Takeoff>(takeoff_client_, goal);
    if (handle == nullptr) {
      return false;
    }
    rclcpp_action::ClientGoalHandle<Takeoff>::WrappedResult result;
    if (!waitForResult<Takeoff>(takeoff_client_, handle, result, 60.0)) {
      return false;
    }
    return result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success;
  }

  GotoPose::Goal cruiseGoal(double x, double y, double speed) const
  {
    GotoPose::Goal goal;
    goal.target_pose.position.x = x;
    goal.target_pose.position.y = y;
    goal.target_pose.position.z = kCruiseZ;
    goal.target_pose.orientation.w = 1.0;
    goal.frame_id = "odom";
    goal.acceptance_radius = 0.3F;
    goal.max_speed = static_cast<float>(speed);
    return goal;
  }

  /// Anti-hang valve only (R21): the condition itself is the assertion.
  static bool waitFor(const std::function<bool()> & happened, double valve_seconds)
  {
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(valve_seconds);
    while (std::chrono::steady_clock::now() < deadline) {
      if (happened()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return happened();
  }

  // ---------------------------------------------------------------- fixture state

  std::string uav_id_;
  double stream_hz_ = 20.0;
  double max_speed_ = 0.55;
  double max_vertical_speed_ = 0.45;
  double max_acceleration_ = 1.0;
  double progress_rate_hz_ = 10.0;
  double advice_timeout_sec_ = 1.0;
  double route_timeout_sec_ = 2.0;
  double route_fresh_sec_ = 1.0;   // matches the node default; only StaleRouteFixture moves it
  double max_escape_deviation_m_ = 8.0;
  double escape_replan_interval_sec_ = 1.0;
  double escape_refresh_m_ = 0.5;
  double goto_timeout_sec_ = 90.0;
  double advise_hz_ = 10.0;
  double drone_radius_m_ = 0.35;
  double flight_band_m_ = 1.0;
  bool with_route_planner_ = false;
  bool navigator_require_obstacle_feed_ = false;
  bool planner_require_obstacle_feed_ = false;

  Vec3 obstacle_center_{2.0, 0.0, kCruiseZ};
  Vec3 obstacle_half_{0.4, 0.4, 0.4};
  std::vector<Vec3> wall_centres_;
  /// Appears mid-flight, on the way the route already chose: what perception
  /// finding something late looks like, and the only shape in this file that makes
  /// the reactive layer fire while a live route exists to rejoin.
  Vec3 surprise_centre_{2.5, 2.6, kCruiseZ};
  /// Non-empty replaces surprise_centre_ with a whole shape (CupFixture).
  std::vector<Vec3> surprise_centres_;
  /// How far along the route the surprise waits before appearing.
  double surprise_trigger_x_ = 0.8;
  double surprise_armed_seconds_ = 0.0;
  std::atomic<bool> surprise_enabled_{false};
  std::atomic<bool> feed_enabled_{true};

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<uav_navigation::LocalPlannerNode> planner_;
  std::shared_ptr<uav_navigation::RoutePlannerNode> route_planner_;
  std::shared_ptr<rclcpp::Node> probe_;
  rclcpp::CallbackGroup::SharedPtr sim_group_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;

  rclcpp::Subscription<ControlCommand>::SharedPtr command_subscription_;
  rclcpp::Subscription<Trajectory3D>::SharedPtr plan_subscription_;
  rclcpp::Subscription<AvoidanceAdvice>::SharedPtr advice_subscription_;
  rclcpp::Publisher<Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<OffboardStatus>::SharedPtr offboard_publisher_;
  rclcpp::Publisher<VehicleState>::SharedPtr vehicle_publisher_;
  rclcpp::Publisher<LocalizationStatus>::SharedPtr localization_publisher_;
  rclcpp::Publisher<ObstacleArray>::SharedPtr obstacle_publisher_;
  rclcpp::Service<Arm>::SharedPtr arm_service_;
  rclcpp::Service<Disarm>::SharedPtr disarm_service_;
  rclcpp::Service<SetFlightMode>::SharedPtr mode_service_;
  rclcpp::TimerBase::SharedPtr sim_timer_;
  rclcpp::TimerBase::SharedPtr obstacle_timer_;

  rclcpp_action::Client<Takeoff>::SharedPtr takeoff_client_;
  rclcpp_action::Client<GotoPose>::SharedPtr goto_client_;

  std::mutex command_mutex_;
  bool tracing_ = false;
  std::vector<CommandSample> trace_;
  ControlCommand last_command_;
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  std::atomic<int> command_count_{0};

  std::mutex log_mutex_;
  std::vector<AdviceLogEntry> advice_log_;
  std::vector<PlanLogEntry> plans_;
  std::vector<double> obstacle_stamps_;

  std::mutex position_mutex_;
  Vec3 position_;
  std::atomic<bool> armed_{false};
  std::atomic<uint8_t> flight_mode_{VehicleState::FLIGHT_MODE_HOLD};
  rclcpp::Time land_commanded_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_plant_tick_{0, 0, RCL_ROS_TIME};
};

/// Claim 6's rig. Config must live in the constructor, not the TEST_F body: SetUp()
/// (and therefore node construction) runs before the body gets a chance to touch
/// these members, so setting them there would silently configure nothing.
class RequiredFeedFixture : public AvoidanceChainFixture
{
protected:
  RequiredFeedFixture()
  {
    // Only the advisor's own require_obstacle_feed matters for this claim: it
    // decides whether a stale map is softened to CLEAR or forwarded as HOLD
    // (local_planner_node.cpp). The navigator's copy governs route-abort and
    // advice-SILENCE, and this scenario never goes silent -- the advisor keeps
    // publishing throughout, it just starts saying HOLD -- so the navigator's flag
    // is left at its sim default.
    planner_require_obstacle_feed_ = true;
    // Long enough that max_duration (0.8x this) comfortably clears the ~16.4 s a
    // straight 4 m leg needs at max_speed -- too short here trips a DIFFERENT
    // failure mode (trajectory build refuses, navigator falls back to the carrot,
    // which the local_planner_node never watches), not the one this test targets.
    goto_timeout_sec_ = 25.0;

    // Off the flight line on purpose: this fixture isolates map staleness from
    // avoidance, so the "keepalive" obstacle must never itself trigger an ESCAPE.
    obstacle_center_ = Vec3{0.0, 6.0, kCruiseZ};
  }
};

/// A wall, not a post: the goal sits behind a run of boxes with the only way past
/// offset to one side. This is the shape that makes a reactive layer livelock when
/// it throws the global route away, because an escape puts the aircraft back on a
/// line into the wall and escape/hold then alternate until the goal expires.
class WallFixture : public AvoidanceChainFixture
{
protected:
  WallFixture()
  {
    // Boxes 0.8 m wide with centres 0.8 m apart, so the run is solid from y -2.8
    // to y +1.2. With 0.35 m of drone radius inflated on top the gap opens above
    // y ~ 1.55, and the goal behind the wall can only be reached around the top.
    for (double y = -2.4; y <= 0.81; y += 0.8) {
      wall_centres_.push_back(Vec3{2.5, y, kCruiseZ});
    }
    // The whole point: with the global route in the loop, an escape must rejoin it
    // instead of pointing the aircraft back at the wall.
    with_route_planner_ = true;
    // Long enough that a livelock shows up as a livelock rather than as a plan the
    // core refused to build: 0.8 * this has to cover the way round.
    goto_timeout_sec_ = 60.0;
  }

  int countAdvice(uint8_t kind)
  {
    int total = 0;
    for (const AdviceLogEntry & entry : adviceLog()) {
      if (entry.advice == kind) {
        ++total;
      }
    }
    return total;
  }
};

/// A CONCAVE surprise: a cup that opens back toward the aircraft, with the goal
/// behind its closed end. This is the shape the debt ledger asked for -- the
/// existing wall proves the escape CONVERGES and that a rejoin HAPPENS, but the
/// wall is convex, so an escape that threw the route away would still reach the
/// goal. Only a cup makes "go straight at the goal after escaping" a trap, which
/// is what turns the rejoin from an observation into a load-bearing claim.
class CupFixture : public AvoidanceChainFixture
{
protected:
  CupFixture()
  {
    // Closed end at x = 3.4 (between the aircraft and the goal at x = 5) with
    // arms reaching all the way back to x = 1.0. Boxes are 0.8 m wide with
    // centres 0.8 m apart, so every run is solid. The arms have to be DEEP: a
    // shallow cup is escaped sideways in two moves and proves nothing (measured
    // -- the first version of this shape let the reactive layer out on its own).
    for (double y = -1.6; y <= 1.61; y += 0.8) {
      surprise_centres_.push_back(Vec3{3.4, y, kCruiseZ});
    }
    for (double x = 1.0; x <= 2.61; x += 0.8) {
      surprise_centres_.push_back(Vec3{x, -1.6, kCruiseZ});
      surprise_centres_.push_back(Vec3{x, 1.6, kCruiseZ});
    }
    // Close the cup only once the aircraft is DEEP inside its mouth.
    surprise_trigger_x_ = 2.0;
    with_route_planner_ = true;
    // Long enough that a livelock reads as a livelock, not as a plan the core
    // refused to build: 0.8 * this must cover the way around the cup.
    goto_timeout_sec_ = 60.0;
  }

  int countAdvice(uint8_t kind)
  {
    int total = 0;
    for (const AdviceLogEntry & entry : adviceLog()) {
      if (entry.advice == kind) {
        ++total;
      }
    }
    return total;
  }
};

/// The same cup with NO global route: the control that says whether this shape
/// is a real trap. Without it, "the stack escaped the cup" proves nothing about
/// what did the escaping.
class CupNoRouteFixture : public CupFixture
{
protected:
  CupNoRouteFixture()
  {
    with_route_planner_ = false;
  }
};

/// Same wall flight, but the route goes stale before the escape can rejoin it.
/// The freshness guard in routeTailAfter had no test of its own: nothing proved
/// it could fire, so a mutation that dropped the age check stayed green.
class StaleRouteFixture : public WallFixture
{
protected:
  StaleRouteFixture()
  {
    // Below one route publish period, so the route on hand is always older than
    // the ceiling by the time an escape asks for its tail.
    route_fresh_sec_ = 0.02;
  }
};

}  // namespace

// ===========================================================================
// Claims 1-5: one flight, because they are all read off the SAME trace/log --
// flying it twice would only double the wall-clock cost, not the evidence.
// ===========================================================================
TEST_F(
  AvoidanceChainFixture,
  ARealObstacleBendsTheRealChainAroundItWithMeasuredLatencyBoundedAccelAndConvergence)
{
  ASSERT_TRUE(flyToCruise()) << "FAILED TO MEASURE: takeoff never completed";

  // The obstacle feed has been running since SetUp, so by the time the goal is sent
  // the map is already fresh: the latency this test measures is the chain's own
  // reaction time, not a cold-start wait for the map.
  ASSERT_TRUE(
    waitFor([this]() {return !obstacleStamps().empty();}, 5.0))
    << "FAILED TO MEASURE: the obstacle feed itself never published";

  startTrace();
  const double flight_start_wall = rclcpp::Clock().now().seconds();

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(4.0, 0.0, max_speed_));
  ASSERT_NE(handle, nullptr) << "FAILED TO MEASURE: GotoPose was rejected";
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 120.0))
    << "FAILED TO MEASURE: goal never returned within the wall-clock valve";
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;
  const double flight_end_wall = rclcpp::Clock().now().seconds();

  const std::vector<CommandSample> flown = trace();
  ASSERT_GE(flown.size(), 10U)
    << "FAILED TO MEASURE: too few setpoint samples to say anything about the flight";

  // -------------------------------------------------------------- claim 1
  // Bends sideways/up, never ducks. Threshold is the obstacle's own half-width: a
  // detour smaller than that could not possibly have cleared it, so this rules out
  // "advice arrived but the flight barely reacted" as well as a vacuous pass.
  double peak_lateral = 0.0;
  double min_z = std::numeric_limits<double>::infinity();
  for (const CommandSample & sample : flown) {
    peak_lateral = std::max(peak_lateral, std::abs(sample.position.y));
    min_z = std::min(min_z, sample.position.z);
  }
  EXPECT_GT(peak_lateral, obstacle_half_.y)
    << "claim 1 (bend): peak lateral " << peak_lateral
    << " m did not clear even the obstacle's own half-width -- the escape was not flown";
  EXPECT_GE(min_z, kCruiseZ - 0.10)
    << "claim 1 (no descent): setpoint dipped to " << min_z << " m below cruise "
    << kCruiseZ << " m -- the chain avoided by ducking, which is forbidden";

  // -------------------------------------------------------------- claim 2
  // Collision-free measured against the SHIPPED setpoint stream, not the waypoints
  // the advisor validated (P6.3's own lesson: a spline can cut a corner a waypoint
  // check never sees). Tolerance is quantization only (costmap resolution 0.25 m /
  // carrot step), never a safety concession.
  double min_clearance = std::numeric_limits<double>::infinity();
  for (const CommandSample & sample : flown) {
    min_clearance =
      std::min(min_clearance, boxSurfaceDistance(sample.position, obstacle_center_, obstacle_half_));
  }
  constexpr double kQuantizationTolerance = 0.05;
  EXPECT_GE(min_clearance, drone_radius_m_ - kQuantizationTolerance)
    << "claim 2 (no collision): the flown setpoint came within " << min_clearance
    << " m of the obstacle surface, under the " << drone_radius_m_ << " m airframe radius";
  RecordProperty("min_clearance_m", std::to_string(min_clearance));

  // -------------------------------------------------------------- claim 3
  // Latency chain: map.stamp -> advice.stamp (first ESCAPE) -> the tick the
  // navigator visibly consumed it (first setpoint sample that actually left the
  // straight line). Converted to a distance at max_speed and checked against the
  // safety margin claim 2 already measured -- the question is whether the round
  // trip could plausibly have eaten into that margin.
  const std::vector<AdviceLogEntry> advice = adviceLog();
  const auto escape_it = std::find_if(
    advice.begin(), advice.end(),
    [](const AdviceLogEntry & entry) {return entry.advice == AvoidanceAdvice::ADVICE_ESCAPE;});
  ASSERT_NE(escape_it, advice.end())
    << "FAILED TO MEASURE: the real advisor never issued an ESCAPE for a blocked route";
  const double t_advice = escape_it->stamp_seconds;

  const std::vector<double> obstacle_stamps = obstacleStamps();
  double t_map = -std::numeric_limits<double>::infinity();
  for (double stamp : obstacle_stamps) {
    if (stamp <= t_advice && stamp > t_map) {
      t_map = stamp;
    }
  }
  ASSERT_GT(t_map, -std::numeric_limits<double>::infinity())
    << "FAILED TO MEASURE: no obstacle publish precedes the escape advice";

  // "tick navigator tieu thu" = the progress-loop tick that actually rebuilt and
  // republished the trajectory in response, i.e. the SECOND valid plan ever
  // published (the first is the original straight line). This is deliberately NOT
  // "when did the flown position visibly move 5 cm": the core is a clamped B-spline
  // with a zero-velocity start (README S:6c), so ANY new trajectory ramps up from
  // rest -- a position-threshold proxy would measure spline ramp time, not chain
  // latency, and conflate the two. Recorded separately below as a diagnostic.
  std::vector<PlanLogEntry> valid_plans_before_consume;
  for (const PlanLogEntry & entry : plans()) {
    // Two point legs are the carrot announcing its own leg; counting one here would
    // time a publish that rebuilt nothing and call it the chain latency.
    if (entry.is_valid && entry.point_count > 2) {
      valid_plans_before_consume.push_back(entry);
    }
  }
  ASSERT_GE(valid_plans_before_consume.size(), 2U)
    << "FAILED TO MEASURE: no rebuilt plan was ever published after the escape advice";
  const double t_consumed = valid_plans_before_consume[1].stamp_seconds;

  const double hop_map_to_advice = t_advice - t_map;
  const double hop_advice_to_consumed = t_consumed - t_advice;
  const double total_latency = t_consumed - t_map;
  const double reaction_distance_m = total_latency * max_speed_;
  const double clearance_margin_m = min_clearance - drone_radius_m_;

  // Secondary diagnostic only, not asserted: how long the smooth ramp takes before
  // the shipped setpoint stream shows a 5 cm bend. Useful context, different claim.
  constexpr double kBendThreshold = 0.05;
  const auto visible_bend_it = std::find_if(
    flown.begin(), flown.end(), [&](const CommandSample & sample) {
      return sample.seconds >= t_advice &&
      (std::abs(sample.position.y) > kBendThreshold ||
      sample.position.z - kCruiseZ > kBendThreshold);
    });
  if (visible_bend_it != flown.end()) {
    RecordProperty(
      "diagnostic_visible_bend_latency_sec", std::to_string(visible_bend_it->seconds - t_advice));
  }

  RecordProperty("hop_map_to_advice_sec", std::to_string(hop_map_to_advice));
  RecordProperty("hop_advice_to_consumed_sec", std::to_string(hop_advice_to_consumed));
  RecordProperty("total_advisor_latency_sec", std::to_string(total_latency));
  RecordProperty("reaction_distance_m", std::to_string(reaction_distance_m));

  EXPECT_GE(hop_map_to_advice, 0.0) << "an advice referenced a map sample from the future";
  EXPECT_LT(reaction_distance_m, clearance_margin_m)
    << "claim 3 (advisor lag): the chain's own round trip (" << total_latency
    << " s, " << reaction_distance_m << " m at " << max_speed_
    << " m/s) exceeds the " << clearance_margin_m << " m of clearance margin claim 2 measured";

  // -------------------------------------------------------------- claim 4
  // Acceleration through the mid-flight replan(s), read from shipped positions only
  // (never the trajectory's own acceleration field -- P6.2's 0.897-vs-55.7 lesson).
  const EmittedMotion motion = measureEmitted(flown, 1.5 / stream_hz_);
  RecordProperty("peak_acceleration_mps2", std::to_string(motion.peak_acceleration));
  EXPECT_GT(motion.gaps, 0U) << "FAILED TO MEASURE: no consecutive setpoint pair to differentiate";
  EXPECT_LE(motion.peak_acceleration, max_acceleration_ * kAccelerationEnvelopeSlack)
    << "claim 4 (rebuild step): peak emitted acceleration " << motion.peak_acceleration
    << " m/s^2 exceeds " << max_acceleration_ << " x " << kAccelerationEnvelopeSlack
    << " -- a mid-flight replan may be commanding a velocity step";

  // -------------------------------------------------------------- claim 5
  // Convergence: bounded, non-vacuous replanning, and the goal actually finished.
  // The pending navigator change (route re-asked after an escape) will add a few
  // more publishes per escape, so the ceiling below is deliberately loose -- a
  // livelock guard, not a tight prediction of the exact count.
  const std::vector<PlanLogEntry> published_plans = plans();
  int rebuild_count = 0;
  bool seen_first_valid = false;
  for (const PlanLogEntry & entry : published_plans) {
    // A two point leg is the carrot saying where it is going, not a rebuilt plan.
    if (!entry.is_valid || entry.point_count <= 2) {
      continue;
    }
    if (!seen_first_valid) {
      seen_first_valid = true;
      continue;                         // the original straight-line plan is not a rebuild
    }
    ++rebuild_count;
  }
  RecordProperty("replan_count", std::to_string(rebuild_count));
  EXPECT_GE(rebuild_count, 1)
    << "claim 5 (non-vacuous replan): no plan after the first was ever published -- "
    << "the escape was advised but never actually flown as a new trajectory";

  const double flight_duration_sec = flight_end_wall - flight_start_wall;
  const int replan_ceiling =
    static_cast<int>(std::ceil(flight_duration_sec / escape_replan_interval_sec_)) + kReplanSlack;
  EXPECT_LE(rebuild_count, replan_ceiling)
    << "claim 5 (convergence): " << rebuild_count << " replans in " << flight_duration_sec
    << " s against a throttle of one per " << escape_replan_interval_sec_
    << " s (ceiling " << replan_ceiling << ") -- the escape loop looks like it never settled";
}

// ===========================================================================
// Claim 6: a map that goes stale mid-flight, under a REQUIRED feed, holds the
// real navigator. Separate flight on purpose: this is a different failure mode,
// not a sub-case of the happy path above.
// ===========================================================================
TEST_F(RequiredFeedFixture, AMapThatDiesMidFlightUnderARequiredFeedHoldsTheRealChain)
{
  ASSERT_TRUE(flyToCruise()) << "FAILED TO MEASURE: takeoff never completed";
  ASSERT_TRUE(waitFor([this]() {return !obstacleStamps().empty();}, 5.0))
    << "FAILED TO MEASURE: the keepalive obstacle feed never published";

  startTrace();
  const Vec3 start = lastCommandedPosition();

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(4.0, 0.0, max_speed_));
  ASSERT_NE(handle, nullptr) << "FAILED TO MEASURE: GotoPose was rejected";

  ASSERT_TRUE(waitFor([&]() {return distance3(lastCommandedPosition(), start) > 0.3;}, 20.0))
    << "FAILED TO MEASURE: the flight never got moving, so freezing it proves nothing";

  feed_enabled_ = false;                 // the map now ages past obstacle_timeout_sec / map_timeout_sec

  ASSERT_TRUE(
    waitFor(
      [this]() {
        const std::vector<AdviceLogEntry> advice = adviceLog();
        return !advice.empty() && advice.back().advice == AvoidanceAdvice::ADVICE_HOLD;
      }, 10.0))
    << "FAILED TO MEASURE: no HOLD advice ever arrived after the feed was cut";

  const Vec3 held = lastCommandedPosition();
  const int commands_before = command_count_.load();
  std::this_thread::sleep_for(1500ms);
  const Vec3 after = lastCommandedPosition();

  EXPECT_LT(distance3(held, after), 0.05)
    << "claim 6 (freeze): the setpoint kept moving " << distance3(held, after)
    << " m after a HOLD advice arrived under a required feed";
  EXPECT_GT(command_count_.load() - commands_before, 0)
    << "claim 6 (no offboard loss): the setpoint stream stopped during the hold, "
    << "which is how offboard is actually lost in flight";

  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, goto_timeout_sec_ + 15.0))
    << "FAILED TO MEASURE: the held goal never returned at all";
  EXPECT_NE(result.code, rclcpp_action::ResultCode::SUCCEEDED)
    << "claim 6: a goal that spent the flight frozen under HOLD must not report success: "
    << result.result->message;
}


// ===========================================================================
// Claim 7: a goal BEHIND A WALL is flown around it and finishes, instead of the
// reactive layer and the setpoint stream arguing until the goal expires. This is
// the shape claim 5 could not challenge: one isolated box converges on its own
// whatever the replan policy is, so the throttle was never actually tested.
// ===========================================================================
TEST_F(WallFixture, AGoalBehindAWallIsFlownAroundItInsteadOfLivelockingAgainstIt)
{
  ASSERT_TRUE(flyToCruise()) << "FAILED TO MEASURE: takeoff never completed";
  ASSERT_TRUE(waitFor([this]() {return !obstacleStamps().empty();}, 5.0))
    << "FAILED TO MEASURE: the obstacle feed never started";

  startTrace();
  const double flight_start_wall = probe_->now().seconds();
  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(5.0, 0.0, max_speed_));
  ASSERT_NE(handle, nullptr) << "FAILED TO MEASURE: GotoPose was rejected";
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, goto_timeout_sec_ + 30.0))
    << "FAILED TO MEASURE: the goal never returned";
  const double flight_duration_sec = probe_->now().seconds() - flight_start_wall;

  const int escapes = countAdvice(AvoidanceAdvice::ADVICE_ESCAPE);
  const int holds = countAdvice(AvoidanceAdvice::ADVICE_HOLD);
  int replans = 0;
  bool seen_first = false;
  for (const PlanLogEntry & entry : plans()) {
    // A two point leg is the carrot saying where it is going, not a rebuilt plan.
    if (!entry.is_valid || entry.point_count <= 2) {
      continue;
    }
    if (!seen_first) {
      seen_first = true;
      continue;
    }
    ++replans;
  }
  RecordProperty("replan_count", std::to_string(replans));
  RecordProperty("escape_advice_count", std::to_string(escapes));
  RecordProperty("hold_advice_count", std::to_string(holds));
  RecordProperty("flight_duration_sec", std::to_string(flight_duration_sec));

  // The livelock signature is the goal running out the clock. Anything else, whether
  // succeeded or aborted for a stated reason, is not it.
  EXPECT_NE(result.result->result_code, ResultCode::ABORTED_TIMEOUT)
    << "claim 7 (livelock): the goal ran out its whole timeout with " << escapes
    << " escapes and " << holds << " holds advised: " << result.result->message;
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;
  EXPECT_EQ(result.result->message.find("carrot fallback"), std::string::npos)
    << "claim 7: the plan never built, so nothing here is about avoidance: "
    << result.result->message;

  const std::vector<CommandSample> flown = trace();
  ASSERT_GE(flown.size(), 10U) << "FAILED TO MEASURE: too few setpoints";

  // It went AROUND: the wall is solid to y +1.2 and inflated past that, so a flight
  // that stayed near the direct line went through cells it was told were blocked.
  double peak_lateral = 0.0;
  for (const CommandSample & sample : flown) {
    peak_lateral = std::max(peak_lateral, sample.position.y);
  }
  RecordProperty("peak_lateral_m", std::to_string(peak_lateral));
  EXPECT_GT(peak_lateral, 1.2)
    << "claim 7: the flight never cleared the top of the wall: " << peak_lateral << " m";

  // Ground truth, not the map: distance to the nearest box surface over the flight.
  double min_clearance = std::numeric_limits<double>::infinity();
  for (const CommandSample & sample : flown) {
    for (const Vec3 & centre : obstacleCentres()) {
      min_clearance =
        std::min(min_clearance, boxSurfaceDistance(sample.position, centre, obstacle_half_));
    }
  }
  RecordProperty("min_clearance_m", std::to_string(min_clearance));
  constexpr double kQuantizationTolerance = 0.05;
  EXPECT_GE(min_clearance, drone_radius_m_ - kQuantizationTolerance)
    << "claim 7: came within " << min_clearance << " m of a wall box surface";

  // Bounded replanning: the same throttle bound claim 5 uses, on a shape that can
  // actually break it.
  const int replan_ceiling =
    static_cast<int>(std::ceil(flight_duration_sec / escape_replan_interval_sec_)) + kReplanSlack;
  EXPECT_LE(replans, replan_ceiling)
    << "claim 7 (convergence): " << replans << " replans in " << flight_duration_sec
    << " s against one per " << escape_replan_interval_sec_ << " s (ceiling "
    << replan_ceiling << ")";
}


// ===========================================================================
// Claim 8: an obstacle that appears AFTER the route was planned, sitting on the
// way the route chose. This is the only shape in this file where the reactive
// layer fires while a global route is live, so it is the one that tests whether
// an escape rejoins the route or throws it away -- and the one that puts the
// replan throttle under any load at all.
// ===========================================================================
TEST_F(WallFixture, ASurpriseOnTheRouteIsEscapedWithoutThrowingTheRouteAway)
{
  ASSERT_TRUE(flyToCruise()) << "FAILED TO MEASURE: takeoff never completed";
  ASSERT_TRUE(waitFor([this]() {return !obstacleStamps().empty();}, 5.0))
    << "FAILED TO MEASURE: the obstacle feed never started";

  startTrace();
  const double flight_start_wall = probe_->now().seconds();
  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(5.0, 0.0, max_speed_));
  ASSERT_NE(handle, nullptr) << "FAILED TO MEASURE: GotoPose was rejected";

  // Let the route be planned and flown a little, then put something on it.
  ASSERT_TRUE(waitFor([this]() {return lastCommandedPosition().x > 0.8;}, 30.0))
    << "FAILED TO MEASURE: the aircraft never set off";
  surprise_enabled_ = true;

  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, goto_timeout_sec_ + 30.0))
    << "FAILED TO MEASURE: the goal never returned";
  const double flight_duration_sec = probe_->now().seconds() - flight_start_wall;

  const int escapes = countAdvice(AvoidanceAdvice::ADVICE_ESCAPE);
  const int holds = countAdvice(AvoidanceAdvice::ADVICE_HOLD);
  int replans = 0;
  bool seen_first = false;
  for (const PlanLogEntry & entry : plans()) {
    // A two point leg is the carrot saying where it is going, not a rebuilt plan.
    if (!entry.is_valid || entry.point_count <= 2) {
      continue;
    }
    if (!seen_first) {
      seen_first = true;
      continue;
    }
    ++replans;
  }
  RecordProperty("replan_count", std::to_string(replans));
  RecordProperty("escape_advice_count", std::to_string(escapes));
  RecordProperty("hold_advice_count", std::to_string(holds));
  RecordProperty("flight_duration_sec", std::to_string(flight_duration_sec));

  EXPECT_GT(escapes + holds, 0)
    << "claim 8: the surprise never disturbed the plan, so this flight tests nothing "
    << "about rejoining a route after an escape";
  EXPECT_NE(result.result->result_code, ResultCode::ABORTED_TIMEOUT)
    << "claim 8 (livelock): ran out the whole timeout after " << escapes << " escapes and "
    << holds << " holds: " << result.result->message;
  EXPECT_EQ(result.result->message.find("carrot fallback"), std::string::npos)
    << result.result->message;

  const std::vector<CommandSample> flown = trace();
  ASSERT_GE(flown.size(), 10U) << "FAILED TO MEASURE: too few setpoints";
  double min_clearance = std::numeric_limits<double>::infinity();
  for (const CommandSample & sample : flown) {
    for (const Vec3 & centre : obstacleCentres()) {
      min_clearance =
        std::min(min_clearance, boxSurfaceDistance(sample.position, centre, obstacle_half_));
    }
  }
  RecordProperty("min_clearance_m", std::to_string(min_clearance));
  constexpr double kSurpriseQuantizationTolerance = 0.05;
  EXPECT_GE(min_clearance, drone_radius_m_ - kSurpriseQuantizationTolerance)
    << "claim 8: came within " << min_clearance << " m of a box surface";

  // The point of the whole change: the escape kept the route it was already on.
  EXPECT_NE(result.result->message.find("route rejoined"), std::string::npos)
    << "claim 8: the escape dropped the global route: " << result.result->message;

  const int replan_ceiling =
    static_cast<int>(std::ceil(flight_duration_sec / escape_replan_interval_sec_)) + kReplanSlack;
  EXPECT_LE(replans, replan_ceiling)
    << "claim 8 (convergence): " << replans << " replans in " << flight_duration_sec
    << " s against one per " << escape_replan_interval_sec_ << " s (ceiling "
    << replan_ceiling << ")";
}

// ===========================================================================
// The route freshness ceiling in routeTailAfter: proven to FIRE, not assumed.
// Deliberately says nothing about how the flight ends -- dropping the tail is
// the pre-route behaviour, and pinning that here would test the wrong thing.
// ===========================================================================
TEST_F(StaleRouteFixture, AnEscapeRefusesToRejoinARouteOlderThanTheFreshnessCeiling)
{
  ASSERT_TRUE(flyToCruise()) << "FAILED TO MEASURE: takeoff never completed";
  ASSERT_TRUE(waitFor([this]() {return !obstacleStamps().empty();}, 5.0))
    << "FAILED TO MEASURE: the obstacle feed never started";

  startTrace();
  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(5.0, 0.0, max_speed_));
  ASSERT_NE(handle, nullptr) << "FAILED TO MEASURE: GotoPose was rejected";

  ASSERT_TRUE(waitFor([this]() {return lastCommandedPosition().x > 0.8;}, 30.0))
    << "FAILED TO MEASURE: the aircraft never set off";

  // The planner runs at plan_hz = 5, so a route lands every 200 ms while the ceiling under
  // test is 20 ms: ONE QUERY IN TEN would find a route still fresh and the guard would stay
  // quiet through no fault of its own. Measured 2026-08-25 -- that is exactly how this case
  // failed a workspace run. Silencing the publisher first makes the age only grow, so the
  // premise is built rather than gambled on, and the comparison stays a real one.
  if (route_planner_) {
    executor_->remove_node(route_planner_);
  }
  surprise_enabled_ = true;

  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, goto_timeout_sec_ + 30.0))
    << "FAILED TO MEASURE: the goal never returned";

  // Without an escape the guard is never reached, so the run proves nothing.
  ASSERT_GT(countAdvice(AvoidanceAdvice::ADVICE_ESCAPE) + countAdvice(AvoidanceAdvice::ADVICE_HOLD), 0)
    << "FAILED TO MEASURE: the surprise never disturbed the plan";

  const std::string message = result.result->message;
  // Says WHY it did not fire, so a future failure does not need this diagnosis again.
  EXPECT_NE(message.find("s old against a"), std::string::npos)
    << "the freshness ceiling never fired on a route it should have refused. If the message"
    << " says 'route rejoined', the route was younger than the "
    << route_fresh_sec_ << " s ceiling when the escape asked -- which would mean the"
    << " publisher was still running. Message: " << message;
  EXPECT_EQ(message.find("route rejoined"), std::string::npos)
    << "a route past its ceiling was rejoined anyway: " << message;
}

// ===========================================================================
// The debt this closes: "rejoining the route after an escape has never been
// shown to PREVENT a livelock -- the wall shape escapes itself." A cup cannot.
// Reaching the goal here is only possible by coming back OUT of the concavity,
// which is what the rejoined global route tells the aircraft to do; flying
// straight at the goal after each escape walks back into the closed end.
// ===========================================================================
TEST_F(CupFixture, AConcaveSurpriseIsEscapedByRejoiningTheRouteInsteadOfLivelocking)
{
  ASSERT_TRUE(flyToCruise()) << "FAILED TO MEASURE: takeoff never completed";
  ASSERT_TRUE(waitFor([this]() {return !obstacleStamps().empty();}, 5.0))
    << "FAILED TO MEASURE: the obstacle feed never started";

  startTrace();
  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(5.0, 0.0, max_speed_));
  ASSERT_NE(handle, nullptr) << "FAILED TO MEASURE: GotoPose was rejected";

  // Let the route be planned and flown into the mouth of the cup, THEN close it:
  // a cup present from the start is routed around by A* and never tests the
  // reactive layer at all (the wall flight already showed that).
  ASSERT_TRUE(
    waitFor([this]() {return lastCommandedPosition().x > surprise_trigger_x_;}, 30.0))
    << "FAILED TO MEASURE: the aircraft never reached the mouth of the cup";
  surprise_armed_seconds_ = probe_->now().seconds();
  surprise_enabled_ = true;

  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, goto_timeout_sec_ + 30.0))
    << "FAILED TO MEASURE: the goal never returned";

  ASSERT_GT(countAdvice(AvoidanceAdvice::ADVICE_ESCAPE) + countAdvice(AvoidanceAdvice::ADVICE_HOLD), 0)
    << "FAILED TO MEASURE: the cup never disturbed the plan, so nothing was escaped";

  EXPECT_NE(result.result->result_code, ResultCode::ABORTED_TIMEOUT)
    << "livelock: ran out the whole timeout inside a concave obstacle -- "
    << result.result->message;
  EXPECT_EQ(result.result->result_code, ResultCode::SUCCEEDED)
    << "did not get out of the cup: " << result.result->message;

  // Clearance is checked on what was FLOWN, not on waypoints: a spline cuts
  // corners that waypoints do not have (package-status S:8, P6.3 point 2).
  const std::vector<CommandSample> flown = trace();
  ASSERT_GE(flown.size(), 10U) << "FAILED TO MEASURE: too few setpoints";
  // R27-1: a sample can only be judged against obstacles that EXISTED when it
  // was flown. Scoring the whole trace against the cup scores the approach leg
  // against boxes that appeared later -- which is what the first run of this
  // test did, reporting a 0 m "violation" the aircraft could not have avoided.
  surprise_enabled_ = false;
  const std::vector<Vec3> before_cup = obstacleCentres();
  surprise_enabled_ = true;
  const std::vector<Vec3> after_cup = obstacleCentres();

  double min_clearance = std::numeric_limits<double>::infinity();
  Vec3 worst_at{0.0, 0.0, 0.0};
  std::size_t judged = 0;
  for (const CommandSample & sample : flown) {
    const std::vector<Vec3> & centres =
      sample.seconds >= surprise_armed_seconds_ ? after_cup : before_cup;
    ++judged;
    for (const Vec3 & centre : centres) {
      const double gap = boxSurfaceDistance(sample.position, centre, obstacle_half_);
      if (gap < min_clearance) {
        min_clearance = gap;
        worst_at = sample.position;
      }
    }
  }
  ASSERT_GT(judged, 0U) << "FAILED TO MEASURE: no sample was scored";
  RecordProperty("min_clearance_m", std::to_string(min_clearance));
  std::cout << "[cup, with route] worst clearance " << min_clearance
            << " m at (" << worst_at.x << ", " << worst_at.y << ")" << std::endl;
  constexpr double kSurpriseQuantizationTolerance = 0.05;
  EXPECT_GE(min_clearance, drone_radius_m_ - kSurpriseQuantizationTolerance)
    << "came within " << min_clearance << " m of a box surface while leaving the cup";
}

// EXPERIMENT (not a shipped claim): does the concavity trap a reactive-only
// stack? Recorded, never asserted -- the answer decides what the cup test above
// is allowed to claim, and an assertion here would pin whichever answer today's
// tuning happens to give.
TEST_F(CupNoRouteFixture, MeasureWhetherAConcaveSurpriseTrapsTheReactiveLayerAlone)
{
  ASSERT_TRUE(flyToCruise()) << "FAILED TO MEASURE: takeoff never completed";
  ASSERT_TRUE(waitFor([this]() {return !obstacleStamps().empty();}, 5.0))
    << "FAILED TO MEASURE: the obstacle feed never started";

  startTrace();
  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(5.0, 0.0, max_speed_));
  ASSERT_NE(handle, nullptr) << "FAILED TO MEASURE: GotoPose was rejected";
  ASSERT_TRUE(
    waitFor([this]() {return lastCommandedPosition().x > surprise_trigger_x_;}, 30.0))
    << "FAILED TO MEASURE: the aircraft never reached the mouth of the cup";
  surprise_enabled_ = true;

  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, goto_timeout_sec_ + 30.0))
    << "FAILED TO MEASURE: the goal never returned";

  RecordProperty("result_code", std::to_string(result.result->result_code));
  RecordProperty("escape_advice_count",
    std::to_string(countAdvice(AvoidanceAdvice::ADVICE_ESCAPE)));
  RecordProperty("hold_advice_count", std::to_string(countAdvice(AvoidanceAdvice::ADVICE_HOLD)));
  std::cout << "[cup, reactive only] result_code=" << int(result.result->result_code)
            << " escapes=" << countAdvice(AvoidanceAdvice::ADVICE_ESCAPE)
            << " holds=" << countAdvice(AvoidanceAdvice::ADVICE_HOLD)
            << " message=" << result.result->message << std::endl;
  SUCCEED();
}
