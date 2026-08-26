#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <uav_interfaces/action/follow_path.hpp>
#include <uav_interfaces/action/goto_pose.hpp>
#include <uav_interfaces/action/hold_position.hpp>
#include <uav_interfaces/action/land.hpp>
#include <uav_interfaces/action/recover.hpp>
#include <uav_interfaces/action/takeoff.hpp>
#include <uav_interfaces/action/track_target.hpp>
#include <uav_interfaces/msg/avoidance_advice.hpp>
#include <uav_interfaces/msg/control_authority.hpp>
#include <uav_interfaces/msg/control_command.hpp>
#include <uav_interfaces/msg/localization_status.hpp>
#include <uav_interfaces/msg/offboard_status.hpp>
#include <uav_interfaces/msg/path3_d.hpp>
#include <uav_interfaces/msg/result_code.hpp>
#include <uav_interfaces/msg/target_state.hpp>
#include <uav_interfaces/msg/target_track.hpp>
#include <uav_interfaces/msg/trajectory3_d.hpp>
#include <uav_interfaces/msg/vehicle_state.hpp>
#include <uav_interfaces/srv/arm.hpp>
#include <uav_interfaces/srv/disarm.hpp>
#include <uav_interfaces/srv/set_flight_mode.hpp>

#include "uav_navigation/carrot.hpp"
#include "uav_navigation/navigator_action_server_node.hpp"

using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;
using uav_interfaces::action::FollowPath;
using uav_interfaces::action::GotoPose;
using uav_interfaces::action::HoldPosition;
using uav_interfaces::action::Land;
using uav_interfaces::action::Recover;
using uav_interfaces::action::Takeoff;
using uav_interfaces::action::TrackTarget;
using uav_interfaces::msg::AvoidanceAdvice;
using uav_interfaces::msg::ControlAuthority;
using uav_interfaces::msg::ControlCommand;
using uav_interfaces::msg::LocalizationStatus;
using uav_interfaces::msg::OffboardStatus;
using uav_interfaces::msg::Path3D;
using uav_interfaces::msg::ResultCode;
using uav_interfaces::msg::TargetState;
using uav_interfaces::msg::TargetTrack;
using uav_interfaces::msg::Trajectory3D;
using uav_interfaces::msg::VehicleState;
using uav_interfaces::srv::Arm;
using uav_interfaces::srv::Disarm;
using uav_interfaces::srv::SetFlightMode;
using uav_navigation::distance3;
using uav_navigation::Vec3;
using uav_navigation::wrapAngle;
using namespace std::chrono_literals;

namespace
{

constexpr double kGroundZ = 0.15;
constexpr double kTakeoffAltitude = 1.0;
constexpr double kCruiseZ = kGroundZ + kTakeoffAltitude;
constexpr double kTouchdownSeconds = 1.0;

// First-order lag toward the setpoint. This rig is 8.4x stiffer than PX4's position
// loop (K_p 0.95), so nothing measured here says anything about clamping in the air;
// only the K_p 0.95 fixtures below do. Kept stiff on purpose: protocol tests want a
// plant that never interferes.
// MEASURED, not assumed (scripts/audit_plant_stiffness.sh, 2026-08-24): rerunning all
// 87 cases at K_p 0.95 leaves 85 verdicts unchanged. The two that flip both say so in
// their own names, and they flip because the leash clamps -- the behaviour the shipped
// 0.55/0.45 ceiling exists to avoid.
constexpr double kPlantGain = 8.0;              // 1/s
constexpr double kPlantMaxSpeed = 3.0;          // m/s
constexpr double kPlantMaxVerticalSpeed = 2.5;  // m/s
constexpr double kPlantMaxTickSeconds = 0.2;

// The node default max_acceleration, times the c3 jitter envelope proven in
// test_avoidance_chain. The old flat 8.0 bound would have passed a 7x regression.
constexpr double kMaxAcceleration = 1.0;        // m/s^2
constexpr double kAccelerationEnvelopeSlack = 3.0;

// The suspected G-N1 fault: the aircraft settles this far below the commanded
// altitude because the two vertical datums are not tied together.
constexpr double kFrameOffsetZ = 1.2;

/// What the fake route planner answers a /planning/route_goal with.
enum class RouteAnswer
{
  VALID,
  FAILED,
  SILENT,
};

/// What the fake advisor puts on /planning/avoidance.
enum class AdviceMode
{
  CLEAR,
  UNCHECKED,   // CLEAR with a zero horizon: it looked at nothing
  ESCAPE,
  HOLD,
  SILENT,
};

constexpr char kRouteRefusal[] = "no corridor in the costmap";
constexpr char kHoldReason[] = "map went stale under the plan";
constexpr char kEscapeReason[] = "obstacle 2.1 m ahead";

struct CommandSample
{
  double seconds = 0.0;
  Vec3 position;
  double yaw = 0.0;
};

/// Measures how badly the HOST oversleeps a 20 ms nap. A machine that cannot wake
/// this thread on time cannot deliver a setpoint tick on time either, so this is
/// the floor no claim about stream rate may sit below. Joins in the destructor:
/// a gtest ASSERT returns straight out of the enclosing function.
class HostSleepWitness
{
public:
  HostSleepWitness()
  {
    thread_ = std::thread(
      [this]() {
        auto previous = std::chrono::steady_clock::now();
        while (!stop_.load()) {
          std::this_thread::sleep_for(20ms);
          const auto now = std::chrono::steady_clock::now();
          const auto overshoot = static_cast<int>(
            std::chrono::duration_cast<std::chrono::microseconds>(now - previous).count() - 20000);
          if (overshoot > worst_us_.load()) {worst_us_.store(overshoot);}
          previous = now;
        }
      });
  }

  ~HostSleepWitness() {stopAndWorstSec();}

  HostSleepWitness(const HostSleepWitness &) = delete;
  HostSleepWitness & operator=(const HostSleepWitness &) = delete;

  double stopAndWorstSec()
  {
    stop_.store(true);
    if (thread_.joinable()) {
      thread_.join();
    }
    return worst_us_.load() / 1e6;
  }

private:
  std::atomic<bool> stop_{false};
  std::atomic<int> worst_us_{0};
  std::thread thread_;
};

struct EmittedMotion
{
  double peak_speed = 0.0;
  double peak_acceleration = 0.0;
  double peak_yaw_step = 0.0;
  double rise_seconds = -1.0;     // 10 % to 90 % of the fastest speed reached
  std::size_t gaps = 0;
  /// The two stream intervals that produced peak_acceleration, in emit-stamp seconds
  /// relative to the first sample. A bare number cannot say whether the plan stepped or
  /// the host starved a tick, and those need opposite responses.
  double peak_at_sec = -1.0;
  double peak_interval_a = 0.0;
  double peak_interval_b = 0.0;
  double peak_speed_a = 0.0;
  double peak_speed_b = 0.0;
};

std::string describePeak(const EmittedMotion & motion, double nominal_tick)
{
  std::ostringstream out;
  out << motion.peak_acceleration << " m/s2 at t+" << motion.peak_at_sec
      << " s: intervals " << motion.peak_interval_a << " s and " << motion.peak_interval_b
      << " s (nominal " << nominal_tick << " s), speeds " << motion.peak_speed_a
      << " -> " << motion.peak_speed_b << " m/s";
  return out.str();
}

/// Derived from the positions actually put on the wire. A trajectory's own
/// acceleration field is analytic inside a segment and blind to a step at its
/// joins, so only the shipped stream can answer what the aircraft was told.
EmittedMotion measureEmitted(const std::vector<CommandSample> & trace, double max_gap)
{
  EmittedMotion motion;
  std::vector<double> speeds;
  std::vector<double> centres;
  std::vector<double> spans;
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
    speeds.push_back(std::sqrt(
        velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z));
    centres.push_back(0.5 * (trace[index].seconds + trace[index - 1].seconds));
    spans.push_back(gap);
    motion.peak_yaw_step = std::max(
      motion.peak_yaw_step, std::abs(wrapAngle(trace[index].yaw - trace[index - 1].yaw)));
  }

  motion.gaps = speeds.size();
  for (const double speed : speeds) {
    motion.peak_speed = std::max(motion.peak_speed, speed);
  }
  for (std::size_t index = 1; index < velocities.size(); ++index) {
    const double gap = centres[index] - centres[index - 1];
    // A starved tick reads as acceleration; blame the scheduler, not the plan.
    if (!(gap > 1e-3) || gap > max_gap) {
      continue;
    }
    const Vec3 change{
      velocities[index].x - velocities[index - 1].x,
      velocities[index].y - velocities[index - 1].y,
      velocities[index].z - velocities[index - 1].z};
    const double acceleration =
      std::sqrt(change.x * change.x + change.y * change.y + change.z * change.z) / gap;
    if (acceleration > motion.peak_acceleration) {
      motion.peak_acceleration = acceleration;
      motion.peak_at_sec = centres[index] - trace.front().seconds;
      motion.peak_interval_a = spans[index - 1];
      motion.peak_interval_b = spans[index];
      motion.peak_speed_a = speeds[index - 1];
      motion.peak_speed_b = speeds[index];
    }
  }

  if (motion.peak_speed > 1e-3) {
    double moving_from = -1.0;
    for (std::size_t index = 0; index < speeds.size(); ++index) {
      if (moving_from < 0.0 && speeds[index] >= 0.1 * motion.peak_speed) {
        moving_from = centres[index];
      }
      if (moving_from >= 0.0 && speeds[index] >= 0.9 * motion.peak_speed) {
        motion.rise_seconds = centres[index] - moving_from;
        break;
      }
    }
  }
  return motion;
}

/// The longest stretch over which the plan clock barely moved.
double longestFlatStretch(const std::vector<std::pair<double, double>> & remaining, double slack)
{
  double longest = 0.0;
  for (std::size_t start = 0; start < remaining.size(); ++start) {
    for (std::size_t end = start + 1; end < remaining.size(); ++end) {
      if (remaining[start].second - remaining[end].second > slack) {
        break;
      }
      longest = std::max(longest, remaining[end].first - remaining[start].first);
    }
  }
  return longest;
}

class NavigatorFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // A graph of its own: a service request still in flight from the previous
    // test would otherwise be served by this test's fake vehicle.
    static std::atomic<int> instances{0};
    uav_id_ = "uav" + std::to_string(instances.fetch_add(1));
    const std::string prefix = "/uav/" + uav_id_;

    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("use_sim_time", false),
      rclcpp::Parameter("uav_id", uav_id_),
      rclcpp::Parameter("stream_hz", stream_hz_),
      rclcpp::Parameter("max_speed", max_speed_),
      rclcpp::Parameter("max_vertical_speed", max_vertical_speed_),
      rclcpp::Parameter("acceptance_radius", 0.3),
      rclcpp::Parameter("min_altitude_m", 0.2),
      rclcpp::Parameter("arrival_settle_sec", 0.2),
      rclcpp::Parameter("progress_rate_hz", 50.0),
      rclcpp::Parameter("feedback_hz", 10.0),
      // Loose on purpose: a loaded test machine starves the fake sensor, and the
      // node is right to abort then. NavigatorStalePoseFixture tests staleness.
      rclcpp::Parameter("odometry_timeout_sec", odometry_timeout_sec_),
      rclcpp::Parameter("startup_timeout_sec", startup_timeout_sec_),
      rclcpp::Parameter("takeoff_stall_grace_sec", takeoff_stall_grace_sec_),
      rclcpp::Parameter("leash_stall_fault_sec", leash_stall_fault_sec_),
      rclcpp::Parameter("offboard_engage_timeout_sec", 10.0),
      rclcpp::Parameter("service_timeout_sec", 3.0),
      rclcpp::Parameter("takeoff_timeout_sec", 20.0),
      rclcpp::Parameter("goto_timeout_sec", goto_timeout_sec_),
      rclcpp::Parameter("land_stream_stop_delay_sec", 0.2),
      rclcpp::Parameter("disarm_timeout_sec", 5.0),
      rclcpp::Parameter("require_obstacle_feed", require_obstacle_feed_),
      rclcpp::Parameter("route_timeout_sec", route_timeout_sec_),
      rclcpp::Parameter("route_fresh_sec", route_fresh_sec_),
      rclcpp::Parameter("advice_timeout_sec", advice_timeout_sec_),
      rclcpp::Parameter("max_escape_deviation_m", max_escape_deviation_m_),
      rclcpp::Parameter("avoidance_hold_timeout_sec", avoidance_hold_timeout_sec_),
      rclcpp::Parameter("escape_replan_interval_sec", escape_replan_interval_sec_),
      rclcpp::Parameter("min_standoff_m", min_standoff_m_),
      rclcpp::Parameter("target_velocity_error_mps", target_velocity_error_mps_),
      rclcpp::Parameter("target_reaction_sec", target_reaction_sec_),
      rclcpp::Parameter("target_lead_sec", target_lead_sec_),
      rclcpp::Parameter("target_state_timeout_sec", target_state_timeout_sec_),
    });

    node_ = uav_navigation::createNavigatorActionServerNode(options);
    probe_ = std::make_shared<rclcpp::Node>("navigator_probe");

    position_ = Vec3{0.0, 0.0, kGroundZ};
    frozen_stamp_ = probe_->now() - rclcpp::Duration::from_seconds(stamp_age_offset_sec_);

    // Own group: a real sensor keeps publishing however busy the flight computer is.
    sim_group_ = probe_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions command_options;
    command_options.callback_group =
      probe_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    command_subscription_ = probe_->create_subscription<ControlCommand>(
      prefix + "/control/cmd_mission", 50,
      [this](const ControlCommand::SharedPtr message) {onCommand(*message);},
      command_options);

    plan_subscription_ = probe_->create_subscription<Trajectory3D>(
      prefix + "/planning/trajectory", rclcpp::QoS(5).reliable().transient_local(),
      [this](const Trajectory3D::SharedPtr message) {
        std::lock_guard<std::mutex> lock(plan_mutex_);
        plans_.push_back(*message);
      },
      command_options);

    odometry_publisher_ =
      probe_->create_publisher<Odometry>(prefix + "/state/odometry_fused", 20);
    offboard_publisher_ =
      probe_->create_publisher<OffboardStatus>(prefix + "/backend/offboard_status", 10);
    vehicle_publisher_ =
      probe_->create_publisher<VehicleState>(prefix + "/state/vehicle", 10);
    localization_publisher_ = probe_->create_publisher<LocalizationStatus>(
      prefix + "/state/localization_status", 10);

    arm_service_ = probe_->create_service<Arm>(
      prefix + "/backend/arm",
      [this](
        const std::shared_ptr<Arm::Request>, std::shared_ptr<Arm::Response> response) {
        commands_before_arm_ = command_count_.load();
        offboard_active_at_arm_ = commandsFlowing();
        ++arm_calls_;
        armed_ = true;
        flight_mode_ = VehicleState::FLIGHT_MODE_OFFBOARD;
        response->success = true;
        response->result_code = ResultCode::SUCCEEDED;
        response->message = "armed";
      });

    disarm_service_ = probe_->create_service<Disarm>(
      prefix + "/backend/disarm",
      [this](
        const std::shared_ptr<Disarm::Request>, std::shared_ptr<Disarm::Response> response) {
        ++disarm_calls_;
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
        last_mode_request_ = request->mode;
        flight_mode_ = request->mode;
        if (request->mode == VehicleState::FLIGHT_MODE_LAND) {
          land_commanded_at_ = probe_->now();
          commands_at_land_mode_ = command_count_.load();
        }
        response->success = true;
        response->result_code = ResultCode::SUCCEEDED;
        response->message = "mode change confirmed";
        response->current_mode = request->mode;
      });

    // Both fakes are off by default. With no publisher on /planning/route the
    // navigator takes the straight line at once, and that is what the older tests
    // measure; switching them on for everyone would change every one of them.
    if (fake_route_) {
      route_publisher_ = probe_->create_publisher<Path3D>(prefix + "/planning/route", 1);
      // sim_group_, not the command group: the fake planner is a PEER NODE, and a
      // peer keeps answering however busy the flight computer is. Sharing the group
      // with the 50 Hz setpoint stream made its answers late, and a late answer ages
      // past route_fresh_sec -- which fires a guard the route tests were not aiming
      // at (measured 3.1 s old against the 1.0 s ceiling, 2026-08-20).
      rclcpp::SubscriptionOptions route_options;
      route_options.callback_group = sim_group_;
      route_goal_subscription_ = probe_->create_subscription<PoseStamped>(
        prefix + "/planning/route_goal", 1,
        [this](const PoseStamped::SharedPtr message) {answerRouteGoal(*message);},
        route_options);
    }
    if (fake_target_) {
      target_publisher_ =
        probe_->create_publisher<TargetState>(prefix + "/world/target_state", 10);
      target_timer_ = probe_->create_wall_timer(50ms, [this]() {publishTarget();}, sim_group_);
    }
    if (fake_advisor_) {
      advice_publisher_ =
        probe_->create_publisher<AvoidanceAdvice>(prefix + "/planning/avoidance", 1);
      advice_timer_ = probe_->create_wall_timer(100ms, [this]() {publishAdvice();}, sim_group_);
    }

    sim_timer_ = probe_->create_wall_timer(20ms, [this]() {simulateVehicle();}, sim_group_);

    takeoff_client_ = rclcpp_action::create_client<Takeoff>(probe_, prefix + "/planning/takeoff");
    goto_client_ = rclcpp_action::create_client<GotoPose>(probe_, prefix + "/planning/goto_pose");
    hold_client_ =
      rclcpp_action::create_client<HoldPosition>(probe_, prefix + "/planning/hold_position");
    land_client_ = rclcpp_action::create_client<Land>(probe_, prefix + "/planning/land");
    follow_client_ =
      rclcpp_action::create_client<FollowPath>(probe_, prefix + "/planning/follow_path");
    track_client_ =
      rclcpp_action::create_client<TrackTarget>(probe_, prefix + "/planning/track_target");
    recover_client_ = rclcpp_action::create_client<Recover>(probe_, prefix + "/planning/recover");

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 4);
    executor_->add_node(node_);
    executor_->add_node(probe_);
    spin_thread_ = std::thread([this]() {executor_->spin();});

    ASSERT_TRUE(takeoff_client_->wait_for_action_server(10s));
    ASSERT_TRUE(goto_client_->wait_for_action_server(10s));
    ASSERT_TRUE(hold_client_->wait_for_action_server(10s));
    ASSERT_TRUE(land_client_->wait_for_action_server(10s));
    ASSERT_TRUE(follow_client_->wait_for_action_server(10s));
    ASSERT_TRUE(track_client_->wait_for_action_server(10s));
    ASSERT_TRUE(recover_client_->wait_for_action_server(10s));
  }

  void TearDown() override
  {
    route_timer_.reset();
    target_timer_.reset();
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    takeoff_client_.reset();
    goto_client_.reset();
    hold_client_.reset();
    land_client_.reset();
    follow_client_.reset();
    track_client_.reset();
    recover_client_.reset();
    executor_.reset();
    node_.reset();
    probe_.reset();
  }

  // ------------------------------------------------------------- fake vehicle

  void onCommand(const ControlCommand & command)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    last_command_ = command;
    last_command_time_ = probe_->now();
    ++command_count_;
    max_arrival_lag_sec_ = std::max(
      max_arrival_lag_sec_,
      (last_command_time_ - rclcpp::Time(command.header.stamp)).seconds());
    if (tracing_) {
      trace_.push_back(
        CommandSample{
          rclcpp::Time(command.header.stamp).seconds(),
          Vec3{command.position.x, command.position.y, command.position.z},
          command.yaw});
    }
  }

  /// Starts a fresh trace: takeoff flies the carrot, and its step would swamp
  /// whatever the goto under test emits.
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

  void forgetPlans()
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    plans_.clear();
  }

  std::vector<Trajectory3D> plans()
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    return plans_;
  }

  bool commandsFlowing()
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return command_count_.load() >= 5 &&
           (probe_->now() - last_command_time_).seconds() < 0.3;
  }

  ControlCommand lastCommand()
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return last_command_;
  }

  Vec3 lastCommandedPosition()
  {
    const ControlCommand command = lastCommand();
    return Vec3{command.position.x, command.position.y, command.position.z};
  }

  Vec3 vehiclePosition()
  {
    std::lock_guard<std::mutex> lock(position_mutex_);
    return position_;
  }

  /// Rate-limited first-order lag; caller holds position_mutex_.
  void advancePlant(const Vec3 & setpoint, double dt)
  {
    const double goal_z = setpoint.z - plant_offset_z_.load();
    double vx = plant_gain_ * (setpoint.x - position_.x);
    double vy = plant_gain_ * (setpoint.y - position_.y);
    double vz = plant_gain_ * (goal_z - position_.z);

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

  /// Back-dates every future pose, so staleness does not depend on scheduling.
  void ageThePose(double seconds)
  {
    std::lock_guard<std::mutex> lock(position_mutex_);
    frozen_stamp_ = probe_->now() - rclcpp::Duration::from_seconds(seconds);
    freeze_stamp_ = true;
  }

  void simulateVehicle()
  {
    const bool tracking = command_count_.load() > 0;
    const Vec3 setpoint = tracking ? lastCommandedPosition() : Vec3{};

    const rclcpp::Time tick = probe_->now();
    double dt = last_plant_tick_.nanoseconds() == 0
      ? 0.02
      : (tick - last_plant_tick_).seconds();
    last_plant_tick_ = tick;
    dt = std::min(std::max(dt, 0.001), kPlantMaxTickSeconds);

    Vec3 updated;
    rclcpp::Time stamp = tick;
    {
      std::lock_guard<std::mutex> lock(position_mutex_);
      if (tracking && !plant_frozen_.load()) {
        advancePlant(setpoint, dt);
      }
      updated = position_;
      if (freeze_stamp_) {
        stamp = frozen_stamp_;
      }
    }

    Odometry odometry;
    odometry.header.stamp = stamp;
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
    localization.is_valid = localization_valid_.load();
    localization.position_uncertainty = static_cast<float>(reported_sigma_.load());
    localization_publisher_->publish(localization);

    OffboardStatus offboard;
    offboard.header.stamp = probe_->now();
    offboard.uav_id = uav_id_;
    offboard.state = commandsFlowing()
      ? (allow_offboard_.load() ? OffboardStatus::STATE_ACTIVE : OffboardStatus::STATE_STREAMING)
      : OffboardStatus::STATE_IDLE;
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

  // ------------------------------------------------- fake route planner and advisor

  /// The real planner keeps publishing for the goal in force at 5 Hz; an escape
  /// rejoins whatever is flowing, so a one-shot answer would never be fresh enough.
  void answerRouteGoal(const PoseStamped & question)
  {
    {
      std::lock_guard<std::mutex> lock(probe_mutex_);
      route_goals_.push_back(question);
      asked_route_ = question;
      has_route_question_ = true;
    }
    publishRoute(question);
    if (!route_timer_) {
      // 5 Hz like the real planner, and in sim_group_ so it keeps that rate: at a
      // 1.0 s route_fresh_sec ceiling this is five ticks of margin, the same margin
      // the fake advisor has. In the probe's default group it shared threads with
      // the action clients and went stale under load.
      route_timer_ = probe_->create_wall_timer(
        200ms, [this]() {
          PoseStamped question_now;
          {
            std::lock_guard<std::mutex> lock(probe_mutex_);
            if (!has_route_question_) {
              return;
            }
            question_now = asked_route_;
          }
          publishRoute(question_now);
        }, sim_group_);
    }
  }

  void publishRoute(const PoseStamped & question)
  {
    std::vector<Vec3> via;
    {
      std::lock_guard<std::mutex> lock(probe_mutex_);
      via = route_via_;
    }
    if (route_answer_.load() == RouteAnswer::SILENT) {
      return;
    }

    Path3D route;
    route.header.stamp = probe_->now();
    route.header.frame_id = "odom";
    route.uav_id = uav_id_;
    route.planner_name = "fake_a_star";
    // Echo of the question, unless a test is deliberately answering another goal.
    route.goal_stamp = stale_route_identity_.load()
      ? builtin_interfaces::msg::Time()
      : question.header.stamp;
    if (route_answer_.load() == RouteAnswer::FAILED) {
      route.is_valid = false;
      route.plan_state = Path3D::PLAN_STATE_FAILED;
      route.reason = kRouteRefusal;
      route_publisher_->publish(route);
      return;
    }

    route.is_valid = true;
    route.plan_state = Path3D::PLAN_STATE_VALID;
    const Vec3 here = vehiclePosition();
    // Every waypoint carries the flight band altitude, never the goal's: that is
    // what the real planner does, and the navigator has to fix it up.
    const double band_z = here.z;
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = here.x;
    pose.position.y = here.y;
    pose.position.z = band_z;
    route.waypoints.push_back(pose);
    for (const Vec3 & point : via) {
      pose.position.x = point.x;
      pose.position.y = point.y;
      pose.position.z = band_z;
      route.waypoints.push_back(pose);
    }
    pose.position.x = question.pose.position.x;
    pose.position.y = question.pose.position.y;
    pose.position.z = band_z;
    route.waypoints.push_back(pose);
    route_publisher_->publish(route);
  }

  void publishAdvice()
  {
    const AdviceMode mode = advice_mode_.load();
    if (mode == AdviceMode::SILENT) {
      return;
    }
    ++advice_published_;

    const double nan = std::numeric_limits<double>::quiet_NaN();
    AvoidanceAdvice advice;
    advice.header.stamp = probe_->now();
    advice.header.frame_id = "odom";
    advice.uav_id = uav_id_;
    advice.escape_point.x = nan;
    advice.escape_point.y = nan;
    advice.escape_point.z = nan;
    advice.clearance_m = 3.0F;
    advice.checked_horizon_m = 6.0F;
    advice.map_fresh = true;
    advice.map_age_sec = 0.1F;

    switch (mode) {
      case AdviceMode::UNCHECKED:
        advice.advice = AvoidanceAdvice::ADVICE_CLEAR;
        advice.checked_horizon_m = 0.0F;
        advice.reason = "no active plan to check";
        break;
      case AdviceMode::HOLD:
        advice.advice = AvoidanceAdvice::ADVICE_HOLD;
        advice.reason = kHoldReason;
        break;
      case AdviceMode::ESCAPE: {
          const Vec3 escape = escapePoint();
          advice.advice = AvoidanceAdvice::ADVICE_ESCAPE;
          advice.reason = kEscapeReason;
          advice.escape_point.x = escape.x;
          advice.escape_point.y = escape.y;
          advice.escape_point.z = escape.z;
          break;
        }
      case AdviceMode::CLEAR:
      default:
        advice.advice = AvoidanceAdvice::ADVICE_CLEAR;
        advice.reason = "horizon free";
        break;
    }
    advice_publisher_->publish(advice);
    ++advice_published_;
  }

  void publishTarget()
  {
    if (!target_alive_.load()) {
      return;
    }
    TargetState state;
    state.header.stamp = probe_->now();
    state.header.frame_id = "odom";
    state.uav_id = uav_id_;
    state.track_id = target_track_id_;
    {
      std::lock_guard<std::mutex> lock(probe_mutex_);
      state.pose.position.x = target_position_.x;
      state.pose.position.y = target_position_.y;
      state.pose.position.z = target_position_.z;
      state.velocity.x = target_velocity_.x;
      state.velocity.y = target_velocity_.y;
      state.velocity.z = target_velocity_.z;
    }
    state.pose.orientation.w = 1.0;
    state.status = target_status_.load();
    state.position_uncertainty = static_cast<float>(target_uncertainty_.load());
    state.time_since_seen_sec = static_cast<float>(target_seen_ago_.load());
    target_publisher_->publish(state);
  }

  void setTarget(const Vec3 & position, const Vec3 & velocity)
  {
    std::lock_guard<std::mutex> lock(probe_mutex_);
    target_position_ = position;
    target_velocity_ = velocity;
  }

  Vec3 targetPosition()
  {
    std::lock_guard<std::mutex> lock(probe_mutex_);
    return target_position_;
  }

  /// What the node is required to sit at: what was asked plus what decision 4 says
  /// the velocity estimate can be wrong by over one reaction time.
  double expectedStandoff(double requested) const
  {
    return std::max(min_standoff_m_, requested) +
           target_velocity_error_mps_ * target_reaction_sec_;
  }

  std::vector<PoseStamped> routeGoals()
  {
    std::lock_guard<std::mutex> lock(probe_mutex_);
    return route_goals_;
  }

  void setEscapePoint(const Vec3 & point)
  {
    std::lock_guard<std::mutex> lock(probe_mutex_);
    escape_point_ = point;
  }

  Vec3 escapePoint()
  {
    std::lock_guard<std::mutex> lock(probe_mutex_);
    return escape_point_;
  }

  /// Widest sideways excursion of the shipped setpoints; the goals below run on +x.
  double peakLateral()
  {
    double peak = 0.0;
    for (const CommandSample & sample : trace()) {
      peak = std::max(peak, std::abs(sample.position.y));
    }
    return peak;
  }

  // ---------------------------------------------------------- action helpers

  template<typename ActionT>
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr sendGoal(
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
    const typename ActionT::Goal & goal,
    typename rclcpp_action::Client<ActionT>::SendGoalOptions options =
    typename rclcpp_action::Client<ActionT>::SendGoalOptions(),
    double timeout = 10.0)
  {
    last_send_timed_out_ = false;
    auto future = client->async_send_goal(goal, options);
    if (future.wait_for(std::chrono::duration<double>(timeout)) != std::future_status::ready) {
      last_send_timed_out_ = true;
      return nullptr;
    }
    return future.get();
  }

  /// A null handle means REJECTED or a dead server; only the first is a pass.
  template<typename HandleT>
  ::testing::AssertionResult goalWasRejected(const HandleT & handle)
  {
    if (last_send_timed_out_) {
      return ::testing::AssertionFailure()
             << "the server never answered: a hang must not read as a rejection";
    }
    if (handle != nullptr) {
      return ::testing::AssertionFailure() << "the goal was accepted";
    }
    return ::testing::AssertionSuccess();
  }

  template<typename ActionT>
  bool waitForResult(
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
    const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr & handle,
    typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & out,
    double timeout = 30.0)
  {
    auto future = client->async_get_result(handle);
    if (future.wait_for(std::chrono::duration<double>(timeout)) != std::future_status::ready) {
      return false;
    }
    out = future.get();
    // UNKNOWN is rclcpp_action saying the server did not recognise the goal, so every
    // field below it is default-constructed. Judging those reads as a product verdict
    // on a result that was never delivered (seen 2026-08-25 under a 16-thread load,
    // with the aircraft provably holding station the whole time).
    if (out.code == rclcpp_action::ResultCode::UNKNOWN) {
      last_result_was_unknown_ = true;
      return false;
    }
    last_result_was_unknown_ = false;
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
    if (!waitForResult<Takeoff>(takeoff_client_, handle, result)) {
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

  Recover::Goal recoverGoal(uint8_t type, double safe_altitude) const
  {
    Recover::Goal goal;
    goal.recovery_type = type;
    goal.safe_altitude = static_cast<float>(safe_altitude);
    goal.trigger_reason = "probe";
    return goal;
  }

  /// The two point legs a carrot flight publishes; a trajectory carries hundreds.
  std::vector<Trajectory3D> carrotLegs()
  {
    std::vector<Trajectory3D> legs;
    for (const Trajectory3D & plan : plans()) {
      if (plan.is_valid && plan.points.size() == 2) {
        legs.push_back(plan);
      }
    }
    return legs;
  }

  /// The carrot's own timing: separate horizontal and vertical budgets, whichever
  /// runs out last. Derived from the limits this fixture configured, never typed in.
  double expectedLegSeconds(const Vec3 & from, const Vec3 & to) const
  {
    const double vertical_speed = std::min(max_vertical_speed_, max_speed_);
    return std::max(
      std::hypot(to.x - from.x, to.y - from.y) / max_speed_,
      std::abs(to.z - from.z) / vertical_speed);
  }

  /// Writer depth: past this the probe cannot tell a hole from its own loss, and it
  /// still sits well under the 0.5 s that actually drops offboard.
  static constexpr double kMaxStreamHoleTicks = 10.0;

  struct StreamWindow
  {
    double step = 0.0;         // biggest jump between two consecutive real ticks
    double gap = 0.0;          // longest hole between two received setpoints
    int samples = 0;
  };

  /// Read from the wire, never from a member of the node, and over a stated window.
  ///
  /// A pair separated by more than a tick and a half is a sample the PROBE missed,
  /// not a jump the node commanded: the fixture's ControlCommand writer is
  /// KeepLast(10) reliable at stream_hz_, so a fifth of a second of reader lag
  /// already loses samples. Same reasoning as measureEmitted's max_gap.
  StreamWindow measureStream(double from_seconds, double to_seconds)
  {
    StreamWindow window;
    bool have_previous = false;
    CommandSample previous;
    for (const CommandSample & sample : trace()) {
      if (sample.seconds < from_seconds || sample.seconds > to_seconds) {
        continue;
      }
      ++window.samples;
      if (have_previous) {
        const double gap = sample.seconds - previous.seconds;
        window.gap = std::max(window.gap, gap);
        if (gap <= 1.5 / stream_hz_) {
          window.step = std::max(window.step, distance3(sample.position, previous.position));
        }
      }
      previous = sample;
      have_previous = true;
    }
    return window;
  }

  /// The longest hole anywhere in the trace. Diagnostic only: it covers stretches
  /// far away from whatever a test is claiming, including probe-side losses.
  double longestTraceGap()
  {
    double gap = 0.0;
    const std::vector<CommandSample> samples = trace();
    for (std::size_t index = 1; index < samples.size(); ++index) {
      gap = std::max(gap, samples[index].seconds - samples[index - 1].seconds);
    }
    return gap;
  }

  /// Half of what the configured stream rate should deliver: proof it still runs.
  ///
  /// Measures ACHIEVED throughput against a CONFIGURED rate, so on a contended host
  /// it convicts the machine (2026-08-25: 11 Hz achieved against a 25 Hz floor with
  /// the probe only 0.0095 s behind). Sound only where the window is a short
  /// constant; anywhere the window itself stretches under load, judge the longest
  /// hole instead -- that is the thing that actually loses offboard.
  int expectedCommands(double seconds) const
  {
    return static_cast<int>(stream_hz_ * seconds * 0.5);
  }

  static void sleepFor(double seconds)
  {
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
  }

  /// Waits for an event; the wall clock is only the anti-hang valve (R21).
  static bool waitFor(const std::function<bool()> & happened, double valve_seconds)
  {
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(valve_seconds);
    while (std::chrono::steady_clock::now() < deadline) {
      if (happened()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return happened();
  }

  double tickSeconds() const {return 1.0 / stream_hz_;}

  std::string uav_id_;
  double goto_timeout_sec_ = 30.0;
  double odometry_timeout_sec_ = 10.0;
  double startup_timeout_sec_ = 10.0;
  double stamp_age_offset_sec_ = 0.0;
  double takeoff_stall_grace_sec_ = 5.0;
  double leash_stall_fault_sec_ = 3.0;
  double stream_hz_ = 50.0;
  double max_speed_ = 1.5;
  double max_vertical_speed_ = 1.5;
  double plant_gain_ = kPlantGain;
  bool last_send_timed_out_ = false;
  bool freeze_stamp_ = false;
  bool fake_route_ = false;
  bool fake_advisor_ = false;
  bool fake_target_ = false;
  double min_standoff_m_ = 1.0;
  double target_velocity_error_mps_ = 0.71;
  double target_reaction_sec_ = 1.0;
  double target_lead_sec_ = 0.0;
  double target_state_timeout_sec_ = 1.0;
  bool require_obstacle_feed_ = false;
  double route_timeout_sec_ = 2.0;
  double route_fresh_sec_ = 1.0;
  double advice_timeout_sec_ = 1.0;
  double max_escape_deviation_m_ = 8.0;
  double escape_replan_interval_sec_ = 1.0;
  double avoidance_hold_timeout_sec_ = 12.0;

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> probe_;
  rclcpp::CallbackGroup::SharedPtr sim_group_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;

  rclcpp::Subscription<ControlCommand>::SharedPtr command_subscription_;
  rclcpp::Subscription<Trajectory3D>::SharedPtr plan_subscription_;
  rclcpp::Publisher<Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<OffboardStatus>::SharedPtr offboard_publisher_;
  rclcpp::Publisher<VehicleState>::SharedPtr vehicle_publisher_;
  rclcpp::Publisher<LocalizationStatus>::SharedPtr localization_publisher_;
  rclcpp::Publisher<Path3D>::SharedPtr route_publisher_;
  rclcpp::TimerBase::SharedPtr route_timer_;
  rclcpp::Publisher<TargetState>::SharedPtr target_publisher_;
  rclcpp::TimerBase::SharedPtr target_timer_;
  rclcpp::Subscription<PoseStamped>::SharedPtr route_goal_subscription_;
  rclcpp::Publisher<AvoidanceAdvice>::SharedPtr advice_publisher_;
  rclcpp::TimerBase::SharedPtr advice_timer_;
  rclcpp::Service<Arm>::SharedPtr arm_service_;
  rclcpp::Service<Disarm>::SharedPtr disarm_service_;
  rclcpp::Service<SetFlightMode>::SharedPtr mode_service_;
  rclcpp::TimerBase::SharedPtr sim_timer_;

  rclcpp_action::Client<Takeoff>::SharedPtr takeoff_client_;
  rclcpp_action::Client<GotoPose>::SharedPtr goto_client_;
  rclcpp_action::Client<HoldPosition>::SharedPtr hold_client_;
  rclcpp_action::Client<Land>::SharedPtr land_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_client_;
  rclcpp_action::Client<TrackTarget>::SharedPtr track_client_;
  rclcpp_action::Client<Recover>::SharedPtr recover_client_;

  std::mutex command_mutex_;
  bool tracing_ = false;
  std::vector<CommandSample> trace_;
  std::mutex plan_mutex_;
  std::vector<Trajectory3D> plans_;
  ControlCommand last_command_;
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  std::atomic<int> command_count_{0};
  double max_arrival_lag_sec_ = 0.0;
  bool last_result_was_unknown_ = false;

  std::mutex probe_mutex_;
  std::vector<PoseStamped> route_goals_;
  std::vector<Vec3> route_via_;
  PoseStamped asked_route_;
  bool has_route_question_ = false;
  Vec3 escape_point_;
  std::atomic<RouteAnswer> route_answer_{RouteAnswer::VALID};
  std::atomic<AdviceMode> advice_mode_{AdviceMode::CLEAR};
  /// Witness for the STIMULUS: this fixture publishes advice from a 100 ms wall timer
  /// in its own executor, and a starved timer means the navigator was never offered
  /// the advice a test is about to judge it on. Measured 2026-08-26: the navigator
  /// reported "the obstacle advisor has not checked the way ahead" while a test
  /// concluded it had ignored a hold.
  std::atomic<int> advice_published_{0};
  std::atomic<bool> stale_route_identity_{false};
  Vec3 target_position_;
  Vec3 target_velocity_;
  std::atomic<double> target_uncertainty_{0.05};
  std::atomic<bool> target_alive_{true};
  // Message freshness and sighting freshness are different axes.
  std::atomic<double> target_seen_ago_{0.0};
  std::atomic<uint8_t> target_status_{TargetTrack::STATUS_TRACKING};
  int32_t target_track_id_ = 7;

  std::mutex position_mutex_;
  Vec3 position_;
  std::atomic<bool> armed_{false};
  std::atomic<bool> allow_offboard_{true};
  std::atomic<bool> localization_valid_{true};
  std::atomic<double> reported_sigma_{0.02};
  std::atomic<bool> plant_frozen_{false};
  std::atomic<double> plant_offset_z_{0.0};
  std::atomic<uint8_t> flight_mode_{VehicleState::FLIGHT_MODE_HOLD};
  std::atomic<int> arm_calls_{0};
  std::atomic<int> disarm_calls_{0};
  std::atomic<int> commands_before_arm_{0};
  std::atomic<bool> offboard_active_at_arm_{false};
  std::atomic<uint8_t> last_mode_request_{VehicleState::FLIGHT_MODE_UNKNOWN};
  std::atomic<int> commands_at_land_mode_{0};
  rclcpp::Time land_commanded_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time frozen_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_plant_tick_{0, 0, RCL_ROS_TIME};
};

/// Same rig, but the aircraft settles 1.2 m below every altitude it is given.
class NavigatorFrameOffsetFixture : public NavigatorFixture
{
protected:
  NavigatorFrameOffsetFixture()
  {
    plant_offset_z_ = kFrameOffsetZ;
    takeoff_stall_grace_sec_ = 1.0;
    // Slow stream on purpose: the gap between two tasks must be far shorter than a
    // tick, or an idle tick would clear the stall clock and hide an inherited one.
    stream_hz_ = 5.0;
  }
};

/// Same rig, but the stall fault is far away: this rig measures the plan clock,
/// not how fast the node gives up on an aircraft that cannot follow.
class NavigatorPlanClockFixture : public NavigatorFixture
{
protected:
  NavigatorPlanClockFixture()
  {
    leash_stall_fault_sec_ = 60.0;
  }
};

/// The real flight regime: PX4's position loop is a P controller with K_p 0.95,
/// so the aircraft trails the setpoint by v/K_p. The default rig is 8x stiffer,
/// which is why the leash-versus-lag conflict only ever showed up in the air.
class NavigatorTrackingEnvelopeFixture : public NavigatorFixture
{
protected:
  NavigatorTrackingEnvelopeFixture()
  {
    plant_gain_ = 0.95;
    max_speed_ = 0.55;
    max_vertical_speed_ = 0.45;
  }
};

/// Same aircraft, the speed ceiling we shipped before 2026-08-18: the control.
class NavigatorOverspeedEnvelopeFixture : public NavigatorTrackingEnvelopeFixture
{
protected:
  NavigatorOverspeedEnvelopeFixture()
  {
    max_speed_ = 1.5;
    max_vertical_speed_ = 1.5;
    // Riding the leash for a whole leg IS the stall signature; let the goal finish
    // so the clamp share reaches result->message instead of aborting first.
    leash_stall_fault_sec_ = 60.0;
  }
};

/// Same rig, but a goto gives up in seconds, so an unplannable goal is cheap to test.
class NavigatorShortGotoFixture : public NavigatorFixture
{
protected:
  NavigatorShortGotoFixture()
  {
    goto_timeout_sec_ = 3.0;
  }
};

/// Same rig, but the pose is half a minute old before the first goal arrives.
class NavigatorStalePoseFixture : public NavigatorFixture
{
protected:
  NavigatorStalePoseFixture()
  {
    stamp_age_offset_sec_ = 30.0;
    startup_timeout_sec_ = 2.0;
    freeze_stamp_ = true;
  }
};

/// A route planner that answers every question, with one corner off the direct line.
class NavigatorRouteFixture : public NavigatorFixture
{
protected:
  NavigatorRouteFixture()
  {
    fake_route_ = true;
    route_via_ = {Vec3{1.5, 2.0, 0.0}};
  }
};

/// A route planner that is subscribed but never answers.
class NavigatorSilentRouteFixture : public NavigatorRouteFixture
{
protected:
  NavigatorSilentRouteFixture()
  {
    route_answer_ = RouteAnswer::SILENT;
    route_timeout_sec_ = 0.5;
  }
};

/// The real-flight rule: no route, no flight.
class NavigatorRouteRequiredFixture : public NavigatorRouteFixture
{
protected:
  NavigatorRouteRequiredFixture()
  {
    require_obstacle_feed_ = true;
    route_answer_ = RouteAnswer::FAILED;
  }
};

/// An obstacle advisor on the wire, saying the horizon is free until told otherwise.
class NavigatorAdviceFixture : public NavigatorFixture
{
protected:
  NavigatorAdviceFixture()
  {
    fake_advisor_ = true;
  }
};

/// A held goal ends on the goto timeout, so the timeout has to be short enough to wait for.
class NavigatorAdviceHoldFixture : public NavigatorAdviceFixture
{
protected:
  NavigatorAdviceHoldFixture()
  {
    goto_timeout_sec_ = 8.0;
    advice_timeout_sec_ = 0.5;
  }

  /// The legs below must be plannable inside 0.8 * goto_timeout_sec. A leg that is
  /// not silently flies the carrot, and then the advisor is checking nothing and
  /// every hold in this file would pass for the wrong reason.
  static void expectTheTrajectoryWasFlown(const std::string & message)
  {
    EXPECT_EQ(message.find("carrot fallback"), std::string::npos)
      << "the plan never built, so this case proves nothing: " << message;
  }
};

/// Same rig, but nothing checking the way ahead is only a warning here.
class NavigatorAdviceGapFixture : public NavigatorAdviceFixture
{
protected:
  NavigatorAdviceGapFixture()
  {
    advice_timeout_sec_ = 0.5;
  }
};

/// The real-flight rule again: a required map needs a route as well as an advisor.
class NavigatorAdviceRequiredFixture : public NavigatorAdviceHoldFixture
{
protected:
  NavigatorAdviceRequiredFixture()
  {
    require_obstacle_feed_ = true;
    fake_route_ = true;
  }
};

/// A hold that never clears, with a goal timeout far too long to be what ends it.
class NavigatorEndlessHoldFixture : public NavigatorAdviceHoldFixture
{
protected:
  NavigatorEndlessHoldFixture()
  {
    goto_timeout_sec_ = 60.0;
    avoidance_hold_timeout_sec_ = 2.0;
  }
};

/// A required map and an advisor, sized for watching a recovery get held.
class NavigatorRecoveryAdviceFixture : public NavigatorAdviceRequiredFixture
{
protected:
  NavigatorRecoveryAdviceFixture()
  {
    // max_duration is 0.8 x this: the return home must PLAN, or the test would be
    // measuring a carrot fallback instead of the flight it claims.
    goto_timeout_sec_ = 20.0;
    // Twice the 0.4 s window the freeze detector below samples over, so "stopped"
    // and "silent past the deadline" cannot be confused, and still nine times the
    // measured 85 ms advisor round trip. Longer would push the hold so late in the
    // flight that the plan could finish first, and an arrival is not a hold.
    advice_timeout_sec_ = 0.8;
  }
};

/// A required map, a route that answers, and a goal too far to plan: the one way
/// into the carrot that needs no misconfiguration at all.
class NavigatorRequiredFeedShortPlanFixture : public NavigatorAdviceRequiredFixture
{
protected:
  NavigatorRequiredFeedShortPlanFixture()
  {
    // max_duration is 0.8 * this, so a distant goal cannot be planned.
    goto_timeout_sec_ = 6.0;
  }
};

/// Constructor-only rig: these configurations must never reach a running node.
class NavigatorConfigFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static std::shared_ptr<rclcpp::Node> build(const std::vector<rclcpp::Parameter> & overrides)
  {
    std::vector<rclcpp::Parameter> all{rclcpp::Parameter("use_sim_time", false)};
    all.insert(all.end(), overrides.begin(), overrides.end());
    rclcpp::NodeOptions options;
    options.parameter_overrides(all);
    return uav_navigation::createNavigatorActionServerNode(options);
  }
};

/// A route planner AND an advisor: what a Goto actually flies under in the air.
class NavigatorRouteAndAdviceFixture : public NavigatorFixture
{
protected:
  NavigatorRouteAndAdviceFixture()
  {
    fake_route_ = true;
    fake_advisor_ = true;
    route_via_ = {Vec3{1.5, 2.0, 0.0}};
    // Identity and rejoin are the guards under test here. The freshness guard
    // reads the WALL clock, so under a starved parallel ctest run it preempts
    // them (measured: 3.1 s age, target alone passes 2/2). Pin it out of range.
    route_fresh_sec_ = 60.0;
  }

  /// Plans published after the first: what a replan costs, and whether it settled.
  int replanCount()
  {
    int rebuilds = 0;
    bool seen_first = false;
    for (const Trajectory3D & plan : plans()) {
      // A two point leg is the carrot naming its own leg, not a rebuilt plan.
      if (!plan.is_valid || plan.points.size() <= 2) {
        continue;
      }
      if (!seen_first) {
        seen_first = true;
        continue;
      }
      ++rebuilds;
    }
    return rebuilds;
  }
};

/// Same, but the route planner never answers, so an escape has nothing to rejoin.
class NavigatorAdviceWithoutRouteFixture : public NavigatorRouteAndAdviceFixture
{
protected:
  NavigatorAdviceWithoutRouteFixture()
  {
    route_answer_ = RouteAnswer::SILENT;
    route_timeout_sec_ = 0.3;
  }
};

/// A target source on the wire; nothing else changes.
class NavigatorTargetFixture : public NavigatorFixture
{
protected:
  NavigatorTargetFixture()
  {
    fake_target_ = true;
  }

  /// The trajectory tripwire has no meaning here: tracking flies the carrot on
  /// purpose and publishes no plan at all.
  TrackTarget::Goal trackGoal(double standoff, double duration) const
  {
    TrackTarget::Goal goal;
    goal.target_id = target_track_id_;
    goal.standoff_distance = static_cast<float>(standoff);
    goal.duration_seconds = static_cast<float>(duration);
    goal.target_lost_timeout = 1.5F;
    return goal;
  }
};

/// Same rig with the lead switched on, which is off in the shipped configuration.
class NavigatorTargetLeadFixture : public NavigatorTargetFixture
{
protected:
  NavigatorTargetLeadFixture()
  {
    target_lead_sec_ = 1.0;
  }
};

/// Message liveness gets so much slack that only the SIGHTING axis can fire. Without
/// it a loaded machine starves the fake publisher, staleness wins the race, and the
/// test goes red on which correct guard bit first (R21).
class NavigatorTargetSightingFixture : public NavigatorTargetFixture
{
protected:
  NavigatorTargetSightingFixture()
  {
    target_state_timeout_sec_ = 30.0;
  }
};

/// A target AND an advisor: what tracking flies under now that the carrot leg it
/// rides is published for the advisor to check.
class NavigatorTargetAdviceFixture : public NavigatorTargetFixture
{
protected:
  NavigatorTargetAdviceFixture()
  {
    fake_advisor_ = true;
    advice_timeout_sec_ = 0.5;
  }
};

/// The real-flight rule: tracking is accepted, and the advice policy is what covers
/// it. Before 2026-08-20 the goal was refused at the door instead.
class NavigatorTargetRequiredFeedFixture : public NavigatorTargetAdviceFixture
{
protected:
  NavigatorTargetRequiredFeedFixture()
  {
    require_obstacle_feed_ = true;
    fake_route_ = true;
  }
};

}  // namespace

TEST_F(NavigatorFixture, GotoBeforeTakeoffIsRejectedAndNothingIsStreamed)
{
  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(1.0, 0.5, 1.0));
  EXPECT_TRUE(goalWasRejected(handle));
  sleepFor(0.5);
  EXPECT_EQ(command_count_.load(), 0) << "a rejected goal must not move the aircraft";
}

TEST_F(NavigatorFixture, TakeoffStreamsMissionSetpointsInOdomAtTheConfiguredRate)
{
  ASSERT_TRUE(flyToCruise());

  const ControlCommand command = lastCommand();
  EXPECT_EQ(command.control_mode, ControlCommand::MODE_POSITION);
  EXPECT_EQ(command.source, ControlAuthority::SOURCE_MISSION);
  EXPECT_EQ(command.header.frame_id, "odom");
  EXPECT_EQ(command.uav_id, uav_id_);
  EXPECT_NEAR(command.position.z, kCruiseZ, 0.1);

  const int before = command_count_.load();
  sleepFor(1.0);
  const int published = command_count_.load() - before;
  // Well above the 2 Hz PX4 floor; the exact rate is measured in sim by G-N1.
  EXPECT_GE(published, 20) << "50 Hz stream produced only " << published << " commands in 1 s";

  EXPECT_NEAR(vehiclePosition().z, kCruiseZ, 0.3);
}

TEST_F(NavigatorFixture, OffboardIsConfirmedBeforeTheAircraftIsArmed)
{
  ASSERT_TRUE(flyToCruise());
  EXPECT_EQ(arm_calls_.load(), 1);
  EXPECT_GT(commands_before_arm_.load(), 0) << "armed before any setpoint was streamed";
  EXPECT_TRUE(offboard_active_at_arm_.load()) << "armed before offboard was confirmed";
}

TEST_F(NavigatorFixture, GotoInAForeignFrameIsRejectedWhileTheSameGoalInOdomIsAccepted)
{
  ASSERT_TRUE(flyToCruise());

  GotoPose::Goal foreign = cruiseGoal(1.0, 0.5, 1.0);
  foreign.frame_id = "map";
  EXPECT_TRUE(goalWasRejected(sendGoal<GotoPose>(goto_client_, foreign)));

  auto accepted = sendGoal<GotoPose>(goto_client_, cruiseGoal(1.0, 0.5, 1.0));
  ASSERT_NE(accepted, nullptr) << "only the frame should have been at fault";

  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, accepted, result));
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_LE(result.result->final_distance_error, 0.3F);
}

TEST_F(NavigatorFixture, FeedbackReportsShrinkingDistance)
{
  ASSERT_TRUE(flyToCruise());

  std::mutex feedback_mutex;
  std::vector<float> distances;
  rclcpp_action::Client<GotoPose>::SendGoalOptions options;
  options.feedback_callback =
    [&](rclcpp_action::ClientGoalHandle<GotoPose>::SharedPtr,
      const std::shared_ptr<const GotoPose::Feedback> feedback) {
      std::lock_guard<std::mutex> lock(feedback_mutex);
      distances.push_back(feedback->distance_remaining);
    };

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(2.0, 0.0, 0.5), options);
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);

  std::lock_guard<std::mutex> lock(feedback_mutex);
  ASSERT_GE(distances.size(), 3u);
  EXPECT_LT(distances.back(), distances.front());
  EXPECT_LE(distances.back(), 0.3F);
}

TEST_F(NavigatorFixture, ASecondGoalIsRejectedWhileTheFirstIsStillFlying)
{
  ASSERT_TRUE(flyToCruise());

  auto flying = sendGoal<GotoPose>(goto_client_, cruiseGoal(6.0, 0.0, 0.2));
  ASSERT_NE(flying, nullptr);
  sleepFor(0.5);

  EXPECT_TRUE(goalWasRejected(sendGoal<GotoPose>(goto_client_, cruiseGoal(0.0, 0.0, 1.0))));

  Takeoff::Goal takeoff;
  takeoff.target_altitude = 2.0F;
  EXPECT_TRUE(goalWasRejected(sendGoal<Takeoff>(takeoff_client_, takeoff)));

  HoldPosition::Goal hold;
  hold.duration_seconds = 1.0F;
  EXPECT_TRUE(goalWasRejected(sendGoal<HoldPosition>(hold_client_, hold)));

  Land::Goal land;
  land.use_precision_landing = false;
  EXPECT_TRUE(goalWasRejected(sendGoal<Land>(land_client_, land)));

  goto_client_->async_cancel_goal(flying);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, flying, result));
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::CANCELED);
}

// Shipped envelope on purpose: the freeze must hold on an aircraft that lags.
TEST_F(NavigatorTrackingEnvelopeFixture, CancelFreezesTheSetpointAndKeepsStreaming)
{
  ASSERT_TRUE(flyToCruise());

  auto flying = sendGoal<GotoPose>(goto_client_, cruiseGoal(6.0, 0.0, 0.5));
  ASSERT_NE(flying, nullptr);
  sleepFor(1.0);

  goto_client_->async_cancel_goal(flying);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, flying, result));
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(result.result->result_code, ResultCode::CANCELED);

  const Vec3 at_cancel = lastCommandedPosition();
  const int before = command_count_.load();
  sleepFor(1.5);
  const Vec3 later = lastCommandedPosition();

  EXPECT_LT(distance3(at_cancel, later), 0.05) << "the setpoint kept travelling after cancel";
  EXPECT_GT(command_count_.load() - before, 10)
    << "the stream stopped after cancel, which drops offboard";
  EXPECT_LT(distance3(vehiclePosition(), later), 0.3)
    << "the aircraft did not settle on the hold point";
}

TEST_F(NavigatorFixture, HoldPositionRunsItsDurationAndKeepsTheAircraftParked)
{
  ASSERT_TRUE(flyToCruise());

  const Vec3 before_hold = lastCommandedPosition();
  HoldPosition::Goal goal;
  goal.duration_seconds = 1.0F;
  auto handle = sendGoal<HoldPosition>(hold_client_, goal);
  ASSERT_NE(handle, nullptr);

  rclcpp_action::ClientGoalHandle<HoldPosition>::WrappedResult result;
  ASSERT_TRUE(waitForResult<HoldPosition>(hold_client_, handle, result));
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_GE(result.result->held_seconds, 1.0F);
  EXPECT_LE(result.result->held_seconds, 10.0F) << "the hold never ended on its own";
  EXPECT_LT(distance3(before_hold, lastCommandedPosition()), 0.05);
}

TEST_F(NavigatorFixture, LandCommandsTheModeThenStopsStreaming)
{
  ASSERT_TRUE(flyToCruise());

  Land::Goal goal;
  goal.use_precision_landing = false;
  auto handle = sendGoal<Land>(land_client_, goal);
  ASSERT_NE(handle, nullptr);

  rclcpp_action::ClientGoalHandle<Land>::WrappedResult result;
  ASSERT_TRUE(waitForResult<Land>(land_client_, handle, result));
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(result.result->success);
  EXPECT_EQ(last_mode_request_.load(), VehicleState::FLIGHT_MODE_LAND);
  EXPECT_GT(commands_at_land_mode_.load(), 0) << "landing was commanded without offboard";
  EXPECT_EQ(disarm_calls_.load(), 0) << "the autopilot disarmed on its own";
  EXPECT_FALSE(armed_.load());

  const int after_land = command_count_.load();
  sleepFor(1.0);
  EXPECT_EQ(command_count_.load(), after_land)
    << "still streaming while the autopilot owns the flight";
}

TEST_F(NavigatorFixture, PrecisionLandingIsRejectedInsteadOfSilentlyIgnored)
{
  ASSERT_TRUE(flyToCruise());

  Land::Goal goal;
  goal.use_precision_landing = true;
  goal.marker_id = 7;
  EXPECT_EQ(sendGoal<Land>(land_client_, goal), nullptr);

  Land::Goal plain;
  plain.use_precision_landing = false;
  auto handle = sendGoal<Land>(land_client_, plain);
  ASSERT_NE(handle, nullptr) << "only precision landing should have been refused";
  rclcpp_action::ClientGoalHandle<Land>::WrappedResult result;
  ASSERT_TRUE(waitForResult<Land>(land_client_, handle, result));
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(NavigatorFixture, ASecondTakeoffIsAcceptedAfterLanding)
{
  ASSERT_TRUE(flyToCruise());

  Land::Goal land;
  land.use_precision_landing = false;
  auto land_handle = sendGoal<Land>(land_client_, land);
  ASSERT_NE(land_handle, nullptr);
  rclcpp_action::ClientGoalHandle<Land>::WrappedResult land_result;
  ASSERT_TRUE(waitForResult<Land>(land_client_, land_handle, land_result));
  ASSERT_EQ(land_result.code, rclcpp_action::ResultCode::SUCCEEDED);

  EXPECT_TRUE(flyToCruise()) << "the navigator did not return to IDLE after landing";
  EXPECT_EQ(arm_calls_.load(), 2);
}

TEST_F(NavigatorFixture, ArrivalWidensWhenTheSystemSaysItsPoseIsNoisy)
{
  ASSERT_TRUE(flyToCruise());

  // GPS-only fallback in SITL: sigma_z = 0.5 m, well past a 0.3 m acceptance.
  reported_sigma_ = 0.5;
  sleepFor(0.2);

  GotoPose::Goal goal = cruiseGoal(1.0, 0.5, 1.0);
  goal.acceptance_radius = 0.1F;
  auto handle = sendGoal<GotoPose>(goto_client_, goal);
  ASSERT_NE(handle, nullptr) << "a noisy pose is not a reason to refuse the goal outright";

  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result));
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  // max(0.1, 2.0 * 0.5) = 1.00 m, under the 1.5 m ceiling.
  EXPECT_NE(result.result->message.find("0.10 -> 1.00 m"), std::string::npos)
    << result.result->message;
  EXPECT_NE(result.result->message.find("0.50 m"), std::string::npos)
    << result.result->message;
}

TEST_F(NavigatorFixture, TheWidenedRadiusStopsAtItsCeiling)
{
  ASSERT_TRUE(flyToCruise());

  reported_sigma_ = 1.2;      // 2.0 * 1.2 = 2.4 m, past the 1.5 m ceiling
  sleepFor(0.2);

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(1.0, 0.5, 1.0));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result));
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_NE(result.result->message.find("-> 1.50 m"), std::string::npos)
    << result.result->message;
}

TEST_F(NavigatorFixture, AnUncertaintyPastTheCeilingRefusesTheGoal)
{
  reported_sigma_ = 2.0;      // worse than max_acceptance_radius
  sleepFor(0.2);

  Takeoff::Goal goal;
  goal.target_altitude = static_cast<float>(kTakeoffAltitude);
  EXPECT_EQ(sendGoal<Takeoff>(takeoff_client_, goal), nullptr);
  EXPECT_EQ(command_count_.load(), 0) << "a refused goal must not stream setpoints";

  reported_sigma_ = 0.02;
  sleepFor(0.2);
  EXPECT_TRUE(flyToCruise()) << "only the uncertainty should have been at fault";
}

TEST_F(NavigatorFixture, LocalizationDeclaredInvalidRefusesTheGoal)
{
  localization_valid_ = false;
  sleepFor(0.2);

  Takeoff::Goal goal;
  goal.target_altitude = static_cast<float>(kTakeoffAltitude);
  EXPECT_EQ(sendGoal<Takeoff>(takeoff_client_, goal), nullptr);
  EXPECT_EQ(command_count_.load(), 0);
}

TEST_F(NavigatorFixture, LocalizationGoingBadInFlightFaultsTheTaskAndHolds)
{
  ASSERT_TRUE(flyToCruise());

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(6.0, 0.0, 0.5));
  ASSERT_NE(handle, nullptr);
  sleepFor(0.3);
  localization_valid_ = false;

  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 20.0));
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.result->result_code, ResultCode::ABORTED_LOST_LOCALIZATION);

  const int before = command_count_.load();
  sleepFor(0.5);
  EXPECT_GT(command_count_.load() - before, expectedCommands(0.5))
    << "went silent instead of holding when localization was disowned";
}

TEST_F(NavigatorFixture, LandIsStillAllowedWhenLocalizationIsDisowned)
{
  ASSERT_TRUE(flyToCruise());

  localization_valid_ = false;
  sleepFor(0.3);

  Land::Goal goal;
  goal.use_precision_landing = false;
  auto handle = sendGoal<Land>(land_client_, goal);
  ASSERT_NE(handle, nullptr) << "landing must never be gated on localization quality";

  rclcpp_action::ClientGoalHandle<Land>::WrappedResult result;
  ASSERT_TRUE(waitForResult<Land>(land_client_, handle, result));
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_FALSE(armed_.load());
}

TEST_F(NavigatorFixture, AStiffRigNeverTripsTheLeashOnTakeoff)
{
  Takeoff::Goal goal;
  goal.target_altitude = static_cast<float>(kTakeoffAltitude);
  auto handle = sendGoal<Takeoff>(takeoff_client_, goal);
  ASSERT_NE(handle, nullptr);

  rclcpp_action::ClientGoalHandle<Takeoff>::WrappedResult result;
  ASSERT_TRUE(waitForResult<Takeoff>(takeoff_client_, handle, result));
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  // Scope: this rig is 8.4x stiffer than PX4, so it can only show that the leash
  // stays quiet when nothing challenges it. The real claim lives in the K_p 0.95
  // fixtures; arrival altitude is not asserted here because the node already gates
  // completion on the measured pose, which makes it a tautology.
  EXPECT_EQ(result.result->message.find("clamped"), std::string::npos)
    << "a stiff plant reported a leash stall: " << result.result->message;
}

TEST_F(NavigatorFrameOffsetFixture, AnAltitudeOffsetFaultsFastInsteadOfHangingUntilTimeout)
{
  Takeoff::Goal goal;
  goal.target_altitude = static_cast<float>(kTakeoffAltitude);
  auto handle = sendGoal<Takeoff>(takeoff_client_, goal);
  ASSERT_NE(handle, nullptr);

  const auto started = std::chrono::steady_clock::now();
  rclcpp_action::ClientGoalHandle<Takeoff>::WrappedResult result;
  ASSERT_TRUE(waitForResult<Takeoff>(takeoff_client_, handle, result, 40.0));
  const double seconds =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();

  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.result->result_code, ResultCode::ABORTED_PLANNER_FAILED);
  EXPECT_NE(result.result->message.find("clamped"), std::string::npos)
    << "the result carried no evidence: " << result.result->message;
  // takeoff_timeout_sec is 20 here: finishing well inside it is the whole point.
  EXPECT_LT(seconds, 12.0) << "hung on the stall instead of reporting it";
  EXPECT_GE(seconds, 2.0) << "faulted before a stall could possibly be real";

  const int before = command_count_.load();
  sleepFor(0.5);
  EXPECT_GT(command_count_.load() - before, expectedCommands(0.5))
    << "went silent instead of holding after the stall";
}

TEST_F(NavigatorFixture, SpoolUpOnTheGroundIsNotAStall)
{
  // The aircraft cannot move yet: motors spinning, PX4 land detector still holding.
  plant_frozen_ = true;

  Takeoff::Goal goal;
  goal.target_altitude = static_cast<float>(kTakeoffAltitude);
  auto handle = sendGoal<Takeoff>(takeoff_client_, goal);
  ASSERT_NE(handle, nullptr);

  // Longer than leash_stall_fault_sec, shorter than takeoff_stall_grace_sec.
  sleepFor(4.0);
  plant_frozen_ = false;

  rclcpp_action::ClientGoalHandle<Takeoff>::WrappedResult result;
  ASSERT_TRUE(waitForResult<Takeoff>(takeoff_client_, handle, result, 30.0));
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED)
    << "aborted while the aircraft was still on the ground: " << result.result->message;
  EXPECT_NEAR(vehiclePosition().z, kCruiseZ, 0.3);
}

TEST_F(NavigatorFrameOffsetFixture, StallCounterDoesNotCarryIntoTheNextTask)
{
  Takeoff::Goal takeoff;
  takeoff.target_altitude = static_cast<float>(kTakeoffAltitude);
  auto stalled = sendGoal<Takeoff>(takeoff_client_, takeoff);
  ASSERT_NE(stalled, nullptr);
  rclcpp_action::ClientGoalHandle<Takeoff>::WrappedResult stalled_result;
  ASSERT_TRUE(waitForResult<Takeoff>(takeoff_client_, stalled, stalled_result, 40.0));
  ASSERT_EQ(stalled_result.result->result_code, ResultCode::ABORTED_PLANNER_FAILED);

  // The aircraft can follow again; the next goal must start from a clean clock.
  plant_offset_z_ = 0.0;
  auto fresh = sendGoal<GotoPose>(goto_client_, cruiseGoal(1.0, 0.5, 1.0));
  ASSERT_NE(fresh, nullptr);

  const auto started = std::chrono::steady_clock::now();
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, fresh, result, 40.0));
  const double seconds =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();

  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED)
    << "inherited the previous stall: " << result.result->message;
  EXPECT_GT(seconds, 0.5) << "died instantly, which is the inherited-counter symptom";
}

TEST_F(NavigatorFixture, CancelDuringEngagementLeavesTheAircraftDisarmed)
{
  allow_offboard_ = false;      // the autopilot never confirms offboard

  Takeoff::Goal goal;
  goal.target_altitude = static_cast<float>(kTakeoffAltitude);
  auto handle = sendGoal<Takeoff>(takeoff_client_, goal);
  ASSERT_NE(handle, nullptr);
  sleepFor(0.5);
  EXPECT_GT(command_count_.load(), 0) << "the priming stream never started";

  takeoff_client_->async_cancel_goal(handle);
  rclcpp_action::ClientGoalHandle<Takeoff>::WrappedResult result;
  ASSERT_TRUE(waitForResult<Takeoff>(takeoff_client_, handle, result, 20.0));
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(arm_calls_.load(), 0) << "armed after the goal was canceled";

  const int after_cancel = command_count_.load();
  sleepFor(0.5);
  EXPECT_EQ(command_count_.load(), after_cancel)
    << "kept streaming after canceling on the ground";
}

TEST_F(NavigatorFixture, APoseThatGoesStaleAbortsTheTaskButKeepsHovering)
{
  ASSERT_TRUE(flyToCruise());

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(6.0, 0.0, 0.5));
  ASSERT_NE(handle, nullptr);
  sleepFor(0.3);
  const Vec3 before_loss = lastCommandedPosition();
  ageThePose(30.0);

  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 20.0));
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.result->result_code, ResultCode::ABORTED_LOST_LOCALIZATION);

  const int before = command_count_.load();
  sleepFor(0.5);
  EXPECT_GT(command_count_.load() - before, expectedCommands(0.5))
    << "losing localization silenced the stream instead of holding";
  EXPECT_LT(distance3(before_loss, lastCommandedPosition()), 0.5)
    << "the setpoint kept travelling on a pose we can no longer trust";
}

TEST_F(NavigatorStalePoseFixture, AStalePoseStopsTheFlightBeforeArming)
{
  Takeoff::Goal goal;
  goal.target_altitude = static_cast<float>(kTakeoffAltitude);
  auto handle = sendGoal<Takeoff>(takeoff_client_, goal);
  ASSERT_NE(handle, nullptr) << "the goal is valid; only its execution should fail";

  rclcpp_action::ClientGoalHandle<Takeoff>::WrappedResult result;
  ASSERT_TRUE(waitForResult<Takeoff>(takeoff_client_, handle, result, 30.0));
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.result->result_code, ResultCode::ABORTED_LOST_LOCALIZATION);
  EXPECT_EQ(arm_calls_.load(), 0) << "armed without a fresh pose";
  EXPECT_EQ(command_count_.load(), 0) << "streamed setpoints without a fresh pose";
}

// ------------------------------------------------- P6.2 trajectory integration

TEST_F(NavigatorFixture, GotoRampsTheSetpointInsteadOfSteppingToFullSpeedInAStiffRig)
{
  ASSERT_TRUE(flyToCruise());
  startTrace();

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(3.0, 0.0, 1.0));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 60.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;
  // The park transition is part of what the aircraft is told, so wait for the stream to
  // publish it. Reading the trace the instant the result lands stops one sample short of
  // the parked setpoint -- which is exactly how a 16.5 m/s2 step stayed invisible on an
  // idle machine and showed up only when the suite was busy enough to delay this thread.
  sleepFor(0.3);

  // The stiff rig never rides the leash, so the measurement is the plan's own.
  EXPECT_EQ(result.result->message.find("clamped"), std::string::npos)
    << "a clamped chain steps the stream: " << result.result->message;

  const EmittedMotion motion = measureEmitted(trace(), 1.5 * tickSeconds());
  ASSERT_GT(motion.gaps, 50U) << "too few setpoints to measure anything";
  EXPECT_LE(motion.peak_speed, 1.1) << "the setpoint outran the goal speed";
  RecordProperty("emitted_peak_acceleration", describePeak(motion, tickSeconds()));
  {
    // Diagnostic 2026-08-26: is the terminal stop present in every run, or only in the
    // runs where the trace happens to reach past arrival?
    const std::vector<CommandSample> tail = trace();
    std::ostringstream tail_out;
    tail_out << tail.size() << " samples; last speeds";
    for (std::size_t i = tail.size() > 6 ? tail.size() - 6 : 1; i < tail.size(); ++i) {
      const double gap = tail[i].seconds - tail[i - 1].seconds;
      const double d = distance3(tail[i].position, tail[i - 1].position);
      tail_out << " " << (gap > 1e-9 ? d / gap : -1.0);
    }
    RecordProperty("emitted_tail", tail_out.str());
  }
  EXPECT_LE(motion.peak_acceleration, kMaxAcceleration * kAccelerationEnvelopeSlack)
    << "the emitted stream jumped: " << describePeak(motion, tickSeconds());
  // The carrot reaches full speed in one tick; a trajectory has to accelerate.
  EXPECT_GE(motion.rise_seconds, 0.2)
    << "the setpoint stepped to speed in " << motion.rise_seconds << " s";
}

TEST_F(NavigatorPlanClockFixture, TheTrajectoryClockStopsWhileTheLeashIsClamped)
{
  ASSERT_TRUE(flyToCruise());

  std::mutex feedback_mutex;
  std::vector<std::pair<double, double>> remaining;
  const auto flat_stretch = [&]() {
      std::lock_guard<std::mutex> lock(feedback_mutex);
      return longestFlatStretch(remaining, 0.2);
    };

  rclcpp_action::Client<GotoPose>::SendGoalOptions options;
  options.feedback_callback =
    [&](rclcpp_action::ClientGoalHandle<GotoPose>::SharedPtr,
      const std::shared_ptr<const GotoPose::Feedback> feedback) {
      std::lock_guard<std::mutex> lock(feedback_mutex);
      remaining.emplace_back(probe_->now().seconds(), feedback->estimated_time_remaining);
    };

  const Vec3 start = lastCommandedPosition();
  startTrace();
  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(4.0, 0.0, 1.0), options);
  ASSERT_NE(handle, nullptr);

  // Anchor on the plan reaching cruise, not on a sleep: the tripled endpoints make
  // the first seconds crawl, and a slow setpoint never stretches the leash.
  ASSERT_TRUE(waitFor([&]() {return distance3(lastCommandedPosition(), start) > 0.8;}, 30.0))
    << "the setpoint never got moving";
  plant_frozen_ = true;
  const bool froze = waitFor([&]() {return flat_stretch() >= 1.0;}, 30.0);
  plant_frozen_ = false;

  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 120.0));
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;
  EXPECT_TRUE(froze)
    << "the plan kept running while the aircraft could not follow: longest flat stretch "
    << flat_stretch() << " s";

  // Evidence for the debt: the leash bounds position, not acceleration, so the
  // stream still steps when it clamps and releases. Measured, not assumed.
  const EmittedMotion motion = measureEmitted(trace(), 1.5 * tickSeconds());
  RecordProperty("emitted_acceleration_across_clamp", motion.peak_acceleration);
  std::cout << "[ EVIDENCE ] emitted acceleration across a leash clamp: "
            << motion.peak_acceleration << " m/s2" << std::endl;
}

TEST_F(NavigatorFixture, TheGotoPlanIsPublishedAndThenRetired)
{
  ASSERT_TRUE(flyToCruise());
  forgetPlans();

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(2.0, 1.0, 1.0));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 60.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  const std::vector<Trajectory3D> published = plans();
  // KeepLast(1) does not promise every sample arrives, only that the last one does.
  const auto valid = std::find_if(
    published.begin(), published.end(), [](const Trajectory3D & entry) {return entry.is_valid;});
  ASSERT_NE(valid, published.end()) << "no plan was ever published";

  const Trajectory3D & plan = *valid;
  EXPECT_TRUE(plan.is_valid);
  EXPECT_EQ(plan.plan_state, Trajectory3D::PLAN_STATE_VALID);
  EXPECT_TRUE(plan.reason.empty()) << "a valid plan carries no excuse: " << plan.reason;
  EXPECT_GE(plan.sequence, 1U) << "a new plan must advance the sequence";
  EXPECT_EQ(plan.header.frame_id, "odom");
  EXPECT_EQ(plan.uav_id, uav_id_);
  ASSERT_GE(plan.points.size(), 2U);
  for (std::size_t index = 1; index < plan.points.size(); ++index) {
    const double before = rclcpp::Duration(plan.points[index - 1].time_from_start).seconds();
    const double after = rclcpp::Duration(plan.points[index].time_from_start).seconds();
    ASSERT_GT(after, before) << "points are not ordered by time at index " << index;
  }
  const Vec3 last{
    plan.points.back().position.x, plan.points.back().position.y, plan.points.back().position.z};
  EXPECT_LT(distance3(last, Vec3{2.0, 1.0, kCruiseZ}), 0.05) << "the plan does not end on the goal";

  EXPECT_FALSE(published.back().is_valid) << "a finished plan is still offered as current";
  EXPECT_TRUE(published.back().points.empty());
  EXPECT_EQ(published.back().plan_state, Trajectory3D::PLAN_STATE_NO_GOAL);
  EXPECT_FALSE(published.back().reason.empty()) << "an invalid plan owes a reason";
  // Retirement announces the fate of that plan, not a new one.
  EXPECT_EQ(published.back().sequence, plan.sequence);
}

TEST_F(NavigatorFixture, CancelStopsTheSetpointWithNoPlanLeftRunning)
{
  ASSERT_TRUE(flyToCruise());

  auto flying = sendGoal<GotoPose>(goto_client_, cruiseGoal(6.0, 0.0, 1.0));
  ASSERT_NE(flying, nullptr);
  sleepFor(2.0);

  goto_client_->async_cancel_goal(flying);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, flying, result));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::CANCELED);

  const Vec3 at_cancel = lastCommandedPosition();
  const int before = command_count_.load();
  sleepFor(1.5);
  EXPECT_LT(distance3(at_cancel, lastCommandedPosition()), 0.05)
    << "a retired plan kept driving the setpoint after cancel";
  // A dead stream also holds the setpoint still, and that one drops offboard.
  EXPECT_GT(command_count_.load() - before, expectedCommands(1.5))
    << "the stream stopped after cancel";
  ASSERT_FALSE(plans().empty());
  EXPECT_FALSE(plans().back().is_valid);
}

TEST_F(NavigatorFixture, ALegTooShortForTheTurnSaysTheNoseFellShort)
{
  ASSERT_TRUE(flyToCruise());

  // 2 m of flight is ~4.3 s; pi at 0.5 rad/s needs 6.3 s. The nose cannot make it.
  const double requested = M_PI;
  GotoPose::Goal goal = cruiseGoal(2.0, 0.0, 1.0);
  goal.target_pose.orientation.z = std::sin(requested / 2.0);
  goal.target_pose.orientation.w = std::cos(requested / 2.0);

  auto handle = sendGoal<GotoPose>(goto_client_, goal);
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 60.0));

  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;
  EXPECT_NE(result.result->message.find("short of the requested yaw"), std::string::npos)
    << "arrival is a position test, so an unfinished turn must be stated: "
    << result.result->message;
}

TEST_F(NavigatorShortGotoFixture, AGoalTooFarToPlanFallsBackToTheCarrotAndSaysWhy)
{
  ASSERT_TRUE(flyToCruise());

  // Far enough that the trajectory would outlast max_duration.
  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(500.0, 0.0, 1.5));
  ASSERT_NE(handle, nullptr) << "admission is about frames and finiteness, not distance";

  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 30.0));
  EXPECT_EQ(result.result->result_code, ResultCode::ABORTED_TIMEOUT);
  EXPECT_NE(result.result->message.find("carrot fallback"), std::string::npos)
    << "the fallback was silent: " << result.result->message;
  // Anchored on the retire arriving, not on the result: they are different topics,
  // and the carrot leg republishes while the goal runs.
  ASSERT_TRUE(waitFor([&]() {return !plans().empty() && !plans().back().is_valid;}, 5.0));
  const std::vector<Trajectory3D> published = plans();
  ASSERT_FALSE(published.empty());
  EXPECT_FALSE(published.back().is_valid) << "a plan was advertised that does not exist";
  // The wire must say the build FAILED and why, not merely fall silent.
  const auto failed = std::find_if(
    published.begin(), published.end(), [](const Trajectory3D & entry) {
      return entry.plan_state == Trajectory3D::PLAN_STATE_FAILED;
    });
  ASSERT_NE(failed, published.end()) << "no FAILED plan_state ever reached the wire";
  EXPECT_FALSE(failed->reason.empty()) << "a failed plan owes its reason";
}

TEST_F(NavigatorFixture, TheNoseFollowsTheTrackThenTakesTheRequestedYaw)
{
  ASSERT_TRUE(flyToCruise());

  const double requested = M_PI / 4.0;
  GotoPose::Goal goal = cruiseGoal(4.0, 0.0, 1.0);
  goal.target_pose.orientation.z = std::sin(requested / 2.0);
  goal.target_pose.orientation.w = std::cos(requested / 2.0);

  startTrace();
  auto handle = sendGoal<GotoPose>(goto_client_, goal);
  ASSERT_NE(handle, nullptr);
  sleepFor(2.0);
  const double in_flight = lastCommand().yaw;

  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 60.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  // Travel runs along +x, so the track heading is 0 while the goal asks for 45 deg.
  EXPECT_LT(std::abs(wrapAngle(in_flight)), 0.3)
    << "the nose left the track early, at " << in_flight << " rad";
  EXPECT_LT(std::abs(wrapAngle(lastCommand().yaw - requested)), 0.4)
    << "the nose never took the requested yaw: " << lastCommand().yaw;
  EXPECT_LE(measureEmitted(trace(), 1.5 * tickSeconds()).peak_yaw_step, 0.1)
    << "yaw stepped instead of slewing under its rate limit";
}


TEST_F(NavigatorTrackingEnvelopeFixture, TheShippedSpeedStaysInsideWhatTheAircraftCanTrack)
{
  ASSERT_TRUE(flyToCruise());

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(3.0, 0.0, max_speed_));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 120.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  // The node only mentions clamping when it happened, so silence is the evidence.
  EXPECT_EQ(result.result->message.find("clamped"), std::string::npos)
    << "a healthy aircraft rode the leash: " << result.result->message;
  EXPECT_LT(max_speed_ / plant_gain_, 0.8)
    << "the shipped speed needs more lead than the leash budget allows";
}

TEST_F(NavigatorOverspeedEnvelopeFixture, TheOldSpeedCeilingRidesTheLeashTheWholeWay)
{
  ASSERT_TRUE(flyToCruise());

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(3.0, 0.0, max_speed_));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 120.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  // Positive control: without it, the test above proves only that nothing ran.
  EXPECT_NE(result.result->message.find("clamped"), std::string::npos)
    << "1.5 m/s needs 1.58 m of lead on 0.80 m of budget yet nothing clamped: "
    << result.result->message;
}


TEST_F(NavigatorTrackingEnvelopeFixture, TakeoffInsideTheShippedEnvelopeNeverTripsTheLeash)
{
  Takeoff::Goal goal;
  goal.target_altitude = static_cast<float>(kTakeoffAltitude);
  auto handle = sendGoal<Takeoff>(takeoff_client_, goal);
  ASSERT_NE(handle, nullptr);

  rclcpp_action::ClientGoalHandle<Takeoff>::WrappedResult result;
  ASSERT_TRUE(waitForResult<Takeoff>(takeoff_client_, handle, result, 60.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  // Vertical lead needed is 0.45 / 0.95 = 0.474 m against a 0.6 m budget, and the
  // stall detector is armed here, so a clamp would also have to not fire a fault.
  EXPECT_EQ(result.result->message.find("clamped"), std::string::npos)
    << "the shipped climb rate rode the leash: " << result.result->message;
}

TEST_F(NavigatorOverspeedEnvelopeFixture, TheOldClimbRateRidesTheLeashOnTakeoff)
{
  Takeoff::Goal goal;
  goal.target_altitude = static_cast<float>(kTakeoffAltitude);
  auto handle = sendGoal<Takeoff>(takeoff_client_, goal);
  ASSERT_NE(handle, nullptr);

  rclcpp_action::ClientGoalHandle<Takeoff>::WrappedResult result;
  ASSERT_TRUE(waitForResult<Takeoff>(takeoff_client_, handle, result, 60.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  // Positive control for the vertical axis, which had none: 1.5 / 0.95 = 1.58 m of
  // lead against 0.6 m of budget. Without this, the test above proves only that
  // nothing ran.
  EXPECT_NE(result.result->message.find("clamped"), std::string::npos)
    << "1.58 m of demanded lead on a 0.60 m budget yet nothing clamped: "
    << result.result->message;
}


// ----------------------------------------------------------------- route hop

TEST_F(NavigatorRouteFixture, RouteWaypointsBendTheFlightAwayFromTheStraightLine)
{
  ASSERT_TRUE(flyToCruise());
  startTrace();

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(3.0, 0.0, 1.0));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 90.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  const std::vector<PoseStamped> asked = routeGoals();
  ASSERT_FALSE(asked.empty()) << "the navigator never asked for a route";
  EXPECT_EQ(asked.back().header.frame_id, "odom");
  EXPECT_NEAR(asked.back().pose.position.x, 3.0, 1e-6) << "it asked about another goal";

  // The route corner sits 2 m off the direct line; the spline cuts it to ~1.33 m.
  EXPECT_GT(peakLateral(), 1.0)
    << "the flight went straight at the goal, so the route was ignored: peak lateral "
    << peakLateral() << " m";
  EXPECT_EQ(result.result->message.find("no route"), std::string::npos)
    << result.result->message;
}

TEST_F(NavigatorRouteFixture, ARefusedRouteFliesTheStraightLineAndSaysWhy)
{
  route_answer_ = RouteAnswer::FAILED;
  ASSERT_TRUE(flyToCruise());
  startTrace();

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(3.0, 0.0, 1.0));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 90.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  EXPECT_LT(peakLateral(), 0.25) << "a refused route still bent the flight";
  EXPECT_NE(result.result->message.find("straight line, no route"), std::string::npos)
    << "the fallback flew blind in silence: " << result.result->message;
  EXPECT_NE(result.result->message.find(kRouteRefusal), std::string::npos)
    << "the planner's own reason was dropped: " << result.result->message;
}

TEST_F(NavigatorRouteRequiredFixture, ARefusedRouteAbortsTheGoalWhenTheMapIsRequired)
{
  ASSERT_TRUE(flyToCruise());
  const Vec3 parked = lastCommandedPosition();

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(3.0, 0.0, 1.0));
  ASSERT_NE(handle, nullptr) << "the route is asked for while running, not at admission";
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 30.0));

  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED) << result.result->message;
  EXPECT_EQ(result.result->result_code, ResultCode::ABORTED_INTERNAL_ERROR);
  EXPECT_NE(result.result->message.find("map is required"), std::string::npos)
    << result.result->message;
  EXPECT_NE(result.result->message.find(kRouteRefusal), std::string::npos)
    << result.result->message;

  sleepFor(0.5);
  EXPECT_LT(distance3(parked, lastCommandedPosition()), 0.05)
    << "the goal was refused and the aircraft moved anyway";
}

TEST_F(NavigatorSilentRouteFixture, ASilentRoutePlannerIsWaitedOutThenNamed)
{
  ASSERT_TRUE(flyToCruise());
  startTrace();

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(2.5, 0.0, 1.0));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 90.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  EXPECT_FALSE(routeGoals().empty()) << "the navigator never asked";
  EXPECT_NE(result.result->message.find("route planner silent"), std::string::npos)
    << "waiting out a dead planner has to be stated: " << result.result->message;
  EXPECT_LT(peakLateral(), 0.25);
}

TEST_F(NavigatorRouteFixture, TheClimbIsSpreadAlongTheRouteInsteadOfLiftedAtTheGoal)
{
  ASSERT_TRUE(flyToCruise());
  forgetPlans();

  const double goal_z = kCruiseZ + 1.5;
  GotoPose::Goal goal = cruiseGoal(3.0, 0.0, 1.0);
  goal.target_pose.position.z = goal_z;

  auto handle = sendGoal<GotoPose>(goto_client_, goal);
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 90.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  const std::vector<Trajectory3D> published = plans();
  const auto valid = std::find_if(
    published.begin(), published.end(), [](const Trajectory3D & entry) {return entry.is_valid;});
  ASSERT_NE(valid, published.end()) << "no plan was ever published";
  ASSERT_GE(valid->points.size(), 2U);

  EXPECT_NEAR(valid->points.front().position.z, kCruiseZ, 0.1)
    << "the plan stepped in altitude at the start";
  EXPECT_NEAR(valid->points.back().position.z, goal_z, 0.05)
    << "the last waypoint did not take the goal altitude";

  // The route corner is halfway along the horizontal path, so half the climb is due.
  double at_corner = -1.0;
  double best = std::numeric_limits<double>::infinity();
  for (const auto & point : valid->points) {
    const double gap = std::hypot(point.position.x - 1.5, point.position.y - 2.0);
    if (gap < best) {
      best = gap;
      at_corner = point.position.z;
    }
  }
  EXPECT_GT(at_corner, kCruiseZ + 0.35)
    << "the whole climb was still to come at the corner: z " << at_corner;
  EXPECT_LT(at_corner, goal_z - 0.35)
    << "the whole climb was already done at the corner: z " << at_corner;
}

// ------------------------------------------------------------ avoidance advice

TEST_F(NavigatorAdviceFixture, ACheckedClearAdviceLeavesTheFlightAlone)
{
  ASSERT_TRUE(flyToCruise());
  startTrace();

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(3.0, 0.0, 1.0));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 90.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  EXPECT_GT(advice_published_.load(), 10) << "the advisor never spoke, so this proves nothing";
  EXPECT_LT(peakLateral(), 0.25) << "a clear horizon moved the flight off its line";
  EXPECT_EQ(result.result->message.find("avoidance"), std::string::npos)
    << "nothing happened, yet the result talks about avoidance: " << result.result->message;
}

TEST_F(NavigatorAdviceFixture, AnEscapePointBendsTheFlightThroughIt)
{
  ASSERT_TRUE(flyToCruise());
  setEscapePoint(Vec3{1.5, 2.0, kCruiseZ});
  advice_mode_ = AdviceMode::ESCAPE;
  startTrace();

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(3.0, 0.0, 1.0));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 90.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  EXPECT_GT(peakLateral(), 1.0)
    << "the escape point was accepted and never flown: peak lateral " << peakLateral() << " m";
  EXPECT_NE(result.result->message.find("avoidance escape"), std::string::npos)
    << result.result->message;
  EXPECT_NE(result.result->message.find(kEscapeReason), std::string::npos)
    << "the advisor's own reason was dropped: " << result.result->message;
}

TEST_F(NavigatorAdviceHoldFixture, AHoldAdviceFreezesTheSetpointWithoutStoppingTheStream)
{
  ASSERT_TRUE(flyToCruise());
  const Vec3 start = lastCommandedPosition();

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(2.0, 0.0, 1.0));
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(waitFor([&]() {return distance3(lastCommandedPosition(), start) > 0.4;}, 20.0))
    << "the setpoint never got moving, so freezing it would prove nothing";

  advice_mode_ = AdviceMode::HOLD;
  const Vec3 held = lastCommandedPosition();
  ASSERT_TRUE(
    waitFor(
      [&]() {
        const Vec3 first = lastCommandedPosition();
        sleepFor(0.5);
        return distance3(first, lastCommandedPosition()) < 0.02;
      }, 20.0))
    << "the setpoint kept running after a hold advice";
  EXPECT_GT(distance3(held, start), 0.3) << "the hold landed before the flight began";

  const Vec3 parked = lastCommandedPosition();
  const int before = command_count_.load();
  sleepFor(1.0);
  EXPECT_LT(distance3(parked, lastCommandedPosition()), 0.02) << "the hold did not hold";
  EXPECT_GT(command_count_.load() - before, expectedCommands(1.0))
    << "the stream stopped during an avoidance hold, which is how offboard is lost";

  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 60.0));
  EXPECT_EQ(result.result->result_code, ResultCode::ABORTED_TIMEOUT) << result.result->message;
  EXPECT_NE(result.result->message.find("avoidance hold"), std::string::npos)
    << result.result->message;
  EXPECT_NE(result.result->message.find(kHoldReason), std::string::npos)
    << "the advisor's own reason was dropped: " << result.result->message;
  expectTheTrajectoryWasFlown(result.result->message);
}

TEST_F(NavigatorAdviceFixture, AHoldThatClearsResumesTheSamePlan)
{
  ASSERT_TRUE(flyToCruise());

  std::mutex feedback_mutex;
  std::vector<std::pair<double, double>> remaining;
  const auto flat_stretch = [&]() {
      std::lock_guard<std::mutex> lock(feedback_mutex);
      return longestFlatStretch(remaining, 0.2);
    };

  rclcpp_action::Client<GotoPose>::SendGoalOptions options;
  options.feedback_callback =
    [&](rclcpp_action::ClientGoalHandle<GotoPose>::SharedPtr,
      const std::shared_ptr<const GotoPose::Feedback> feedback) {
      std::lock_guard<std::mutex> lock(feedback_mutex);
      remaining.emplace_back(probe_->now().seconds(), feedback->estimated_time_remaining);
    };

  const Vec3 start = lastCommandedPosition();
  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(3.0, 0.0, 1.0), options);
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(waitFor([&]() {return distance3(lastCommandedPosition(), start) > 0.5;}, 20.0))
    << "the setpoint never got moving";

  advice_mode_ = AdviceMode::HOLD;
  const int advice_before = advice_published_.load();
  const double hold_opened = probe_->now().seconds();
  const bool froze = waitFor([&]() {return flat_stretch() >= 1.0;}, 20.0);
  const double hold_window = probe_->now().seconds() - hold_opened;
  const int advice_during = advice_published_.load() - advice_before;
  advice_mode_ = AdviceMode::CLEAR;

  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 90.0));
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED)
    << "a cleared hold never resumed: " << result.result->message;

  // The stimulus has to have been APPLIED before the navigator can be judged on it:
  // half the nominal 10 Hz over the window is already generous.
  ASSERT_GE(advice_during, static_cast<int>(hold_window * 5.0))
    << "FAILED TO MEASURE: this fixture published only " << advice_during
    << " HOLD advices in " << hold_window << " s -- the advisor timer was starved, so the"
    << " navigator was never offered the hold this test judges it on";

  // The plan clock stopping IS the feedback channel: GotoPose has no text field.
  EXPECT_TRUE(froze)
    << "the plan clock counted down through an avoidance hold: longest flat stretch "
    << flat_stretch() << " s";
  EXPECT_NE(result.result->message.find("avoidance hold"), std::string::npos)
    << result.result->message;
}

TEST_F(NavigatorAdviceRequiredFixture, SilenceHoldsTheFlightWhenTheMapIsRequired)
{
  advice_mode_ = AdviceMode::SILENT;
  ASSERT_TRUE(flyToCruise());
  const Vec3 start = lastCommandedPosition();

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(1.5, 0.0, 1.0));
  ASSERT_NE(handle, nullptr) << "the route answered, so the goal must start";
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 60.0));

  EXPECT_EQ(result.result->result_code, ResultCode::ABORTED_TIMEOUT) << result.result->message;
  EXPECT_NE(result.result->message.find("avoidance hold"), std::string::npos)
    << result.result->message;
  EXPECT_NE(result.result->message.find("obstacle advisor"), std::string::npos)
    << result.result->message;
  EXPECT_LT(distance3(lastCommandedPosition(), start), 1.0)
    << "the flight ran on with nothing checking the way ahead";
  expectTheTrajectoryWasFlown(result.result->message);
}

TEST_F(NavigatorAdviceGapFixture, SilenceIsFlownThroughLoudlyWhenTheMapIsNotRequired)
{
  advice_mode_ = AdviceMode::SILENT;
  ASSERT_TRUE(flyToCruise());

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(2.5, 0.0, 1.0));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 90.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  EXPECT_NE(result.result->message.find("flying unguarded"), std::string::npos)
    << "flying with nothing checking ahead was silent: " << result.result->message;
  EXPECT_NE(result.result->message.find("obstacle advisor"), std::string::npos)
    << result.result->message;
}

TEST_F(NavigatorAdviceRequiredFixture, AZeroHorizonClearIsNotPermissionToFly)
{
  advice_mode_ = AdviceMode::UNCHECKED;
  ASSERT_TRUE(flyToCruise());
  const Vec3 start = lastCommandedPosition();

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(1.5, 0.0, 1.0));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 60.0));

  EXPECT_GT(advice_published_.load(), 10)
    << "the advisor said nothing, so this cannot tell a zero horizon from silence";
  EXPECT_EQ(result.result->result_code, ResultCode::ABORTED_TIMEOUT) << result.result->message;
  EXPECT_NE(result.result->message.find("avoidance hold"), std::string::npos)
    << result.result->message;
  EXPECT_LT(distance3(lastCommandedPosition(), start), 1.0)
    << "a CLEAR that checked nothing was flown as clear air";
  expectTheTrajectoryWasFlown(result.result->message);
}

TEST_F(NavigatorAdviceGapFixture, AZeroHorizonClearIsFlownThroughLoudlyWhenTheMapIsNotRequired)
{
  advice_mode_ = AdviceMode::UNCHECKED;
  ASSERT_TRUE(flyToCruise());

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(2.5, 0.0, 1.0));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 90.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  EXPECT_GT(advice_published_.load(), 10) << "the advisor never spoke";
  EXPECT_NE(result.result->message.find("flying unguarded"), std::string::npos)
    << "an advisor that checked nothing was treated as clear air: " << result.result->message;
}

TEST_F(NavigatorAdviceHoldFixture, AnEscapePointThatIsNotFiniteIsRefusedAndHeld)
{
  ASSERT_TRUE(flyToCruise());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  setEscapePoint(Vec3{nan, nan, nan});
  advice_mode_ = AdviceMode::ESCAPE;
  const Vec3 start = lastCommandedPosition();

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(2.0, 0.0, 1.0));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 60.0));

  EXPECT_EQ(result.result->result_code, ResultCode::ABORTED_TIMEOUT) << result.result->message;
  EXPECT_NE(result.result->message.find("escape refused"), std::string::npos)
    << result.result->message;
  EXPECT_NE(result.result->message.find("not finite"), std::string::npos)
    << result.result->message;
  // Without the finiteness check the plan fails to build, the carrot takes over and
  // the aircraft flies to the goal anyway. Only this guard parks it.
  EXPECT_LT(distance3(lastCommandedPosition(), start), 1.0)
    << "a NaN escape point was taken as advice and the flight went on";
  expectTheTrajectoryWasFlown(result.result->message);
}

TEST_F(NavigatorAdviceHoldFixture, AnEscapePointFarOffThePlanIsRefusedAndHeld)
{
  ASSERT_TRUE(flyToCruise());
  // Finite, inside the map, and 12 m off a 3 m leg: only the deviation ceiling
  // (5 m) can refuse this one.
  setEscapePoint(Vec3{1.0, 12.0, kCruiseZ});
  advice_mode_ = AdviceMode::ESCAPE;
  startTrace();

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(2.0, 0.0, 1.0));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 60.0));

  EXPECT_EQ(result.result->result_code, ResultCode::ABORTED_TIMEOUT) << result.result->message;
  EXPECT_NE(result.result->message.find("off the plan"), std::string::npos)
    << result.result->message;
  EXPECT_LT(peakLateral(), 0.3)
    << "a 12 m detour was flown on one advisor's word: peak lateral " << peakLateral() << " m";
  expectTheTrajectoryWasFlown(result.result->message);
}

// The altitude envelope is enforced at the goal door, and an escape never goes
// through that door. Without this guard the core smooths a descent the aircraft can
// track perfectly, so the leash never bites and nothing else says no.
TEST_F(NavigatorAdviceHoldFixture, AnEscapeBelowTheAltitudeFloorIsRefusedAndHeld)
{
  ASSERT_TRUE(flyToCruise());
  // Finite, and only 4.15 m off a 2 m leg, so the deviation ceiling (8 m) cannot be
  // what refuses it: this point is illegal purely because of its altitude.
  setEscapePoint(Vec3{1.0, 0.0, -3.0});
  advice_mode_ = AdviceMode::ESCAPE;
  startTrace();

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(2.0, 0.0, 1.0));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 60.0));

  EXPECT_EQ(result.result->result_code, ResultCode::ABORTED_TIMEOUT) << result.result->message;
  EXPECT_NE(result.result->message.find("min/max_altitude_m"), std::string::npos)
    << result.result->message;

  double lowest = kCruiseZ;
  for (const CommandSample & sample : trace()) {
    lowest = std::min(lowest, sample.position.z);
  }
  RecordProperty("lowest_commanded_z_m", std::to_string(lowest));
  EXPECT_GT(lowest, kCruiseZ - 0.3)
    << "the setpoint was walked toward the ground on one advisor's word: " << lowest << " m";
  expectTheTrajectoryWasFlown(result.result->message);
}

TEST_F(NavigatorEndlessHoldFixture, AHoldWithNoWayThroughEndsAsAHoldNotAsAGoalTimeout)
{
  advice_mode_ = AdviceMode::HOLD;
  ASSERT_TRUE(flyToCruise());

  const double sent = probe_->now().seconds();
  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(2.0, 0.0, 1.0));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 90.0));
  const double elapsed = probe_->now().seconds() - sent;
  RecordProperty("held_seconds", std::to_string(elapsed));

  EXPECT_EQ(result.result->result_code, ResultCode::ABORTED_PLANNER_FAILED)
    << result.result->message;
  EXPECT_NE(result.result->message.find("no way through"), std::string::npos)
    << result.result->message;
  // The number that matters: it ended as a hold, not as the goal running out.
  EXPECT_LT(elapsed, 0.5 * goto_timeout_sec_)
    << "the hold rode out the goal timeout instead: " << elapsed << " s";
  expectTheTrajectoryWasFlown(result.result->message);
}

TEST_F(NavigatorAdviceFixture, AnEscapeThatGoesDownIsFlownButNamed)
{
  ASSERT_TRUE(flyToCruise());
  // Inside the deviation ceiling, so it is accepted; the navigator has no map and
  // cannot refuse it, but a descent to get around something must not be silent.
  setEscapePoint(Vec3{1.5, 1.0, kCruiseZ - 0.8});
  advice_mode_ = AdviceMode::ESCAPE;

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(3.0, 0.0, 1.0));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 90.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  EXPECT_NE(result.result->message.find("descended"), std::string::npos)
    << "the escape dropped the aircraft 0.8 m and the result did not say so: "
    << result.result->message;
}

// ------------------------------------------------- configurations that must not run

TEST_F(NavigatorConfigFixture, EveryAvoidanceLimitThatWouldDisableItselfIsRefused)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  // Each one switches a guard off in silence: a zero timeout stops the whole watch,
  // and NaN makes every "deviation > ceiling" test false forever.
  const std::vector<std::pair<std::string, double>> cases{
    {"advice_timeout_sec", 0.0},
    {"advice_timeout_sec", nan},
    {"max_escape_deviation_m", 0.0},
    {"max_escape_deviation_m", nan},
    {"route_timeout_sec", 0.0},
    {"escape_replan_interval_sec", -1.0},
    {"escape_refresh_m", nan},
  };

  for (const std::pair<std::string, double> & entry : cases) {
    EXPECT_THROW(
      build({rclcpp::Parameter(entry.first, entry.second)}), std::invalid_argument)
      << entry.first << " = " << entry.second << " started the node anyway";
  }
}

TEST_F(NavigatorConfigFixture, TheSightingGateMayNotBeConfiguredIntoSilence)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::vector<std::pair<std::string, double>> unusable{
    {"target_sighting_timeout_sec", 0.0},
    {"target_sighting_timeout_sec", nan},
    {"target_sighting_period_copy_sec", 0.0},
    {"target_sighting_period_copy_sec", nan},
    {"tracker_lost_after_copy_sec", 0.0},
    {"tracker_lost_after_copy_sec", -1.0},
  };
  for (const std::pair<std::string, double> & entry : unusable) {
    EXPECT_THROW(
      build({rclcpp::Parameter(entry.first, entry.second)}), std::invalid_argument)
      << entry.first << " = " << entry.second << " started the node anyway";
  }

  // R29: a grace of one sampling period has no margin at all, so one late detection
  // would be read as a target nobody can see.
  EXPECT_THROW(
    build(
      {rclcpp::Parameter("target_sighting_timeout_sec", 0.1),
        rclcpp::Parameter("target_sighting_period_copy_sec", 0.0667)}),
    std::invalid_argument);

  // At or above the tracker's own give-up time the status branch always fires first
  // and this gate never bites: a guard that quietly stopped biting.
  EXPECT_THROW(
    build(
      {rclcpp::Parameter("target_sighting_timeout_sec", 3.0),
        rclcpp::Parameter("tracker_lost_after_copy_sec", 3.0)}),
    std::invalid_argument);

  // Positive control: the shipped pairing really does build, so the throws above
  // prove a refusal rather than proving the node never starts.
  EXPECT_NO_THROW(build({rclcpp::Parameter("target_sighting_timeout_sec", 1.0)}));
}

TEST_F(NavigatorConfigFixture, ARequiredMapRefusesEveryWayOfSwitchingTheWatchOff)
{
  EXPECT_THROW(
    build(
      {rclcpp::Parameter("require_obstacle_feed", true),
        rclcpp::Parameter("use_avoidance", false)}), std::invalid_argument)
    << "the map was demanded and the thing that reads it was switched off";

  // Not the self-lock any more (the carrot publishes its own leg since 2026-08-20):
  // the rollback gives up the acceleration ceiling of the core for every goal, and a
  // required map is the configuration that flies people.
  EXPECT_THROW(
    build(
      {rclcpp::Parameter("require_obstacle_feed", true),
        rclcpp::Parameter("use_trajectory", false)}), std::invalid_argument)
    << "the shaped-trajectory rollback must not be available on a required map";

  // The quietest of the three rollbacks: no route means every goal is flown as a
  // straight line, which is the local minimum the route planner exists to avoid.
  EXPECT_THROW(
    build(
      {rclcpp::Parameter("require_obstacle_feed", true),
        rclcpp::Parameter("use_route", false)}), std::invalid_argument)
    << "a demanded map that only ever sees a straight line is a map nobody used";

  EXPECT_NO_THROW(build({rclcpp::Parameter("require_obstacle_feed", true)}))
    << "the shipped real-flight configuration must still start";
}

TEST_F(NavigatorRequiredFeedShortPlanFixture, AGoalTooFarToPlanAbortsInsteadOfHoldingOutTheTimeout)
{
  ASSERT_TRUE(flyToCruise());

  // Far enough that the trajectory outlasts max_duration, so the carrot takes over
  // and the advisor is left with nothing to check.
  const rclcpp::Time asked = probe_->now();
  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(500.0, 0.0, 1.0));
  ASSERT_NE(handle, nullptr) << "admission is about frames and finiteness, not distance";

  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 60.0));
  const double elapsed = (probe_->now() - asked).seconds();

  EXPECT_EQ(result.result->result_code, ResultCode::ABORTED_PLANNER_FAILED)
    << "it fell into the timeout machine instead of saying why: " << result.result->message;
  EXPECT_NE(result.result->message.find("map is required"), std::string::npos)
    << result.result->message;
  RecordProperty("abort_seconds", elapsed);
  EXPECT_LT(elapsed, goto_timeout_sec_)
    << "the goal held for the whole goto timeout: " << elapsed << " s";
}


// ------------------------------------------------- an escape keeps the route (R-N1)

TEST_F(NavigatorRouteAndAdviceFixture, AnEscapeRejoinsTheRouteInsteadOfDroppingIt)
{
  ASSERT_TRUE(flyToCruise());
  // Beside the direct line, well inside the deviation ceiling, and BEFORE the route
  // corner: a replan that drops the route goes straight from here to the goal.
  setEscapePoint(Vec3{0.5, 0.8, kCruiseZ});
  advice_mode_ = AdviceMode::ESCAPE;
  startTrace();

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(3.0, 0.0, 1.0));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 120.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  RecordProperty("replan_count", replanCount());
  EXPECT_GE(replanCount(), 1) << "the escape was advised but never flown as a new plan";

  // Escape alone bends the flight to y ~ 0.53; the route corner at y = 2.0 pulls the
  // spline to y ~ 1.33. Only a rejoined route can get past 1.0.
  EXPECT_GT(peakLateral(), 1.0)
    << "the escape dropped the route and went straight at the goal: peak lateral "
    << peakLateral() << " m";
  EXPECT_NE(result.result->message.find("route rejoined"), std::string::npos)
    << result.result->message;
  EXPECT_EQ(result.result->message.find("carrot fallback"), std::string::npos)
    << result.result->message;
}

TEST_F(NavigatorAdviceWithoutRouteFixture, AnEscapeWithNoRouteToRejoinSaysSo)
{
  ASSERT_TRUE(flyToCruise());
  setEscapePoint(Vec3{0.5, 0.8, kCruiseZ});
  advice_mode_ = AdviceMode::ESCAPE;

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(3.0, 0.0, 1.0));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 120.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  EXPECT_NE(result.result->message.find("no route to rejoin"), std::string::npos)
    << "flying straight at the goal after an escape has to be stated: "
    << result.result->message;
}

// ------------------------------------------------------------------- follow path

TEST_F(NavigatorFixture, FollowPathFliesEveryWaypointInOrderAndReportsCompletion)
{
  ASSERT_TRUE(flyToCruise());
  startTrace();

  const std::vector<Vec3> waypoints{
    Vec3{1.0, 0.5, kCruiseZ}, Vec3{2.0, -0.5, kCruiseZ},
    Vec3{3.0, 0.5, kCruiseZ}, Vec3{4.0, 0.0, kCruiseZ}};

  FollowPath::Goal goal;
  goal.path.header.frame_id = "odom";
  goal.path.plan_state = Path3D::PLAN_STATE_VALID;
  goal.path.is_valid = true;
  for (const Vec3 & waypoint : waypoints) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = waypoint.x;
    pose.position.y = waypoint.y;
    pose.position.z = waypoint.z;
    pose.orientation.w = 1.0;
    goal.path.waypoints.push_back(pose);
  }
  goal.max_speed = 1.0F;
  // The spline cuts each corner by 0.33 m here, so a tighter radius would report
  // waypoints missed that the aircraft did fly past.
  const double acceptance = 0.5;
  goal.waypoint_acceptance_radius = static_cast<float>(acceptance);

  std::mutex feedback_mutex;
  std::vector<float> completion;
  rclcpp_action::Client<FollowPath>::SendGoalOptions options;
  options.feedback_callback =
    [&](rclcpp_action::ClientGoalHandle<FollowPath>::SharedPtr,
      const std::shared_ptr<const FollowPath::Feedback> feedback) {
      std::lock_guard<std::mutex> lock(feedback_mutex);
      completion.push_back(feedback->path_completion_percent);
    };

  auto handle = sendGoal<FollowPath>(follow_client_, goal, options);
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<FollowPath>::WrappedResult result;
  ASSERT_TRUE(waitForResult<FollowPath>(follow_client_, handle, result, 120.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  EXPECT_EQ(result.result->waypoints_reached, waypoints.size())
    << "a waypoint was never flown inside the acceptance radius: " << result.result->message;
  EXPECT_EQ(result.result->message.find("carrot fallback"), std::string::npos)
    << result.result->message;

  // Order, read off the setpoints that actually went out: the first tick within
  // acceptance of waypoint i must come before the first tick within it of i+1.
  const std::vector<CommandSample> flown = trace();
  ASSERT_GT(flown.size(), 50U) << "too few setpoints to say anything about order";
  std::vector<std::size_t> first_seen;
  for (const Vec3 & waypoint : waypoints) {
    std::size_t index = 0;
    for (; index < flown.size(); ++index) {
      if (distance3(flown[index].position, waypoint) <= acceptance) {
        break;
      }
    }
    ASSERT_LT(index, flown.size()) << "the flight never came near a waypoint";
    first_seen.push_back(index);
  }
  for (std::size_t index = 1; index < first_seen.size(); ++index) {
    EXPECT_GT(first_seen[index], first_seen[index - 1])
      << "waypoint " << index << " was reached before waypoint " << index - 1;
  }

  std::lock_guard<std::mutex> lock(feedback_mutex);
  ASSERT_FALSE(completion.empty()) << "no feedback arrived";
  for (std::size_t index = 1; index < completion.size(); ++index) {
    EXPECT_GE(completion[index], completion[index - 1]) << "completion went backwards";
  }
  EXPECT_FLOAT_EQ(completion.back(), 100.0F) << "the path finished short of 100 %";
}

// ------------------------------------------------------------------ track target

TEST_F(NavigatorTargetFixture, TheStandoffCarriesTheReactionMarginDecisionFourDemands)
{
  ASSERT_TRUE(flyToCruise());
  // Off the takeoff heading on purpose, so the nose has somewhere to turn to.
  setTarget(Vec3{4.0, 3.0, kCruiseZ}, Vec3{});

  const double requested = 2.0;
  auto handle = sendGoal<TrackTarget>(track_client_, trackGoal(requested, 6.0));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<TrackTarget>::WrappedResult result;
  ASSERT_TRUE(waitForResult<TrackTarget>(track_client_, handle, result, 60.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  const double held = distance3(vehiclePosition(), targetPosition());
  const double expected = expectedStandoff(requested);
  RecordProperty("standoff_held_m", std::to_string(held));
  EXPECT_NEAR(held, expected, 0.25)
    << "sat " << held << " m off a target it was told to hold " << requested
    << " m from; decision 4 requires " << expected << " m";
  EXPECT_GT(held, requested + 0.3)
    << "the reaction margin was never added: " << result.result->message;

  // The nose is slewed by the same rate limiter as everything else, so it lags in a
  // turn; here it has had seconds of straight flight to finish the turn.
  const Vec3 here = vehiclePosition();
  const Vec3 there = targetPosition();
  const double bearing = std::atan2(there.y - here.y, there.x - here.x);
  EXPECT_LT(std::abs(wrapAngle(lastCommand().yaw - bearing)), 0.3)
    << "the nose was commanded to " << lastCommand().yaw << " rad, not at the target at "
    << bearing << " rad";
}

TEST_F(NavigatorTargetLeadFixture, TargetLeadIsRefusedWhenNoUncertaintyIsStated)
{
  ASSERT_TRUE(flyToCruise());
  // Stationary in truth, but reporting a metre per second: exactly the 71 % error
  // decision 4 is about. Without a stated uncertainty this may not move the aim.
  target_uncertainty_ = -1.0;
  setTarget(Vec3{5.0, 0.0, kCruiseZ}, Vec3{0.0, 1.0, 0.0});

  auto handle = sendGoal<TrackTarget>(track_client_, trackGoal(2.0, 6.0));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<TrackTarget>::WrappedResult result;
  ASSERT_TRUE(waitForResult<TrackTarget>(track_client_, handle, result, 60.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  EXPECT_NE(result.result->message.find("target lead refused"), std::string::npos)
    << "an unusable velocity was used in silence: " << result.result->message;
  // A one second lead would have pulled the aim a whole metre along +y.
  EXPECT_LT(std::abs(vehiclePosition().y), 0.4)
    << "the aim was led by a velocity nobody vouched for: y " << vehiclePosition().y;
}

TEST_F(NavigatorTargetLeadFixture, AStatedUncertaintyLetsTheLeadThrough)
{
  ASSERT_TRUE(flyToCruise());
  target_uncertainty_ = 0.05;
  setTarget(Vec3{5.0, 0.0, kCruiseZ}, Vec3{0.0, 1.0, 0.0});

  auto handle = sendGoal<TrackTarget>(track_client_, trackGoal(2.0, 6.0));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<TrackTarget>::WrappedResult result;
  ASSERT_TRUE(waitForResult<TrackTarget>(track_client_, handle, result, 60.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  EXPECT_EQ(result.result->message.find("target lead refused"), std::string::npos)
    << result.result->message;
  // Positive control for the test above: with the lead allowed the aim really does
  // move, so that test proves a refusal rather than proving the lead never works.
  EXPECT_GT(std::abs(vehiclePosition().y), 0.4)
    << "the lead was allowed and changed nothing: y " << vehiclePosition().y;
}

TEST_F(NavigatorTargetFixture, ATargetThatStopsArrivingEndsTheGoalAsLostNotAsTimedOut)
{
  ASSERT_TRUE(flyToCruise());
  setTarget(Vec3{5.0, 0.0, kCruiseZ}, Vec3{});

  auto handle = sendGoal<TrackTarget>(track_client_, trackGoal(2.0, 0.0));
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(waitFor([this]() {return distance3(vehiclePosition(), Vec3{}) > 0.5;}, 20.0))
    << "the aircraft never started tracking";

  target_alive_ = false;
  rclcpp_action::ClientGoalHandle<TrackTarget>::WrappedResult result;
  ASSERT_TRUE(waitForResult<TrackTarget>(track_client_, handle, result, 60.0));

  EXPECT_EQ(result.result->result_code, ResultCode::ABORTED_LOST_TARGET)
    << result.result->message;
  EXPECT_NE(result.result->message.find("target lost"), std::string::npos)
    << result.result->message;
  EXPECT_GT(result.result->tracked_seconds, 0.0F);
}

// N2: the check above proves liveness; these three pin the sighting axis.
TEST_F(NavigatorTargetSightingFixture, ATargetNobodyHasSeenIsNotChasedJustBecauseTheMessageIsFresh)
{
  ASSERT_TRUE(flyToCruise());
  setTarget(Vec3{5.0, 0.0, kCruiseZ}, Vec3{});

  std::mutex feedback_mutex;
  std::vector<bool> visible;
  rclcpp_action::Client<TrackTarget>::SendGoalOptions options;
  options.feedback_callback =
    [&](rclcpp_action::ClientGoalHandle<TrackTarget>::SharedPtr,
      const std::shared_ptr<const TrackTarget::Feedback> feedback) {
      std::lock_guard<std::mutex> lock(feedback_mutex);
      visible.push_back(feedback->target_visible);
    };

  auto handle = sendGoal<TrackTarget>(track_client_, trackGoal(2.0, 0.0), options);
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(waitFor([this]() {return distance3(vehiclePosition(), Vec3{}) > 0.5;}, 20.0))
    << "the aircraft never started tracking";

  // The publisher keeps its full rate; only the sighting behind it ages out.
  target_seen_ago_ = 2.5;

  rclcpp_action::ClientGoalHandle<TrackTarget>::WrappedResult result;
  ASSERT_TRUE(waitForResult<TrackTarget>(track_client_, handle, result, 60.0));
  EXPECT_EQ(result.result->result_code, ResultCode::ABORTED_LOST_TARGET)
    << result.result->message;
  EXPECT_NE(result.result->message.find("has not been seen"), std::string::npos)
    << result.result->message;

  std::lock_guard<std::mutex> lock(feedback_mutex);
  ASSERT_FALSE(visible.empty());
  EXPECT_FALSE(visible.back())
    << "feedback still called a target visible that nobody had seen for 2.5 s";
}

TEST_F(NavigatorTargetSightingFixture, ATrackTheWorldModelCallsLostIsNotChasedWhileItKeepsArriving)
{
  ASSERT_TRUE(flyToCruise());
  setTarget(Vec3{5.0, 0.0, kCruiseZ}, Vec3{});

  auto handle = sendGoal<TrackTarget>(track_client_, trackGoal(2.0, 0.0));
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(waitFor([this]() {return distance3(vehiclePosition(), Vec3{}) > 0.5;}, 20.0))
    << "the aircraft never started tracking";

  // Sighting age stays zero: the status alone has to be enough.
  target_status_ = TargetTrack::STATUS_LOST;

  rclcpp_action::ClientGoalHandle<TrackTarget>::WrappedResult result;
  ASSERT_TRUE(waitForResult<TrackTarget>(track_client_, handle, result, 60.0));
  EXPECT_EQ(result.result->result_code, ResultCode::ABORTED_LOST_TARGET)
    << result.result->message;
  EXPECT_NE(result.result->message.find("reports the track as lost"), std::string::npos)
    << result.result->message;
}

/// Positive control for both tests above: a chase that keeps working must keep
/// working. Measured on the G-M3 chain: a healthy chase never held a sighting age
/// above 0.37 s, so 0.4 s of coasting is inside normal, not a loss.
TEST_F(NavigatorTargetFixture, ABrieflyCoastingTargetIsStillChasedToTheSameStandoff)
{
  ASSERT_TRUE(flyToCruise());
  setTarget(Vec3{4.0, 3.0, kCruiseZ}, Vec3{});
  target_status_ = TargetTrack::STATUS_COASTING;
  target_seen_ago_ = 0.4;

  const double requested = 2.0;
  auto handle = sendGoal<TrackTarget>(track_client_, trackGoal(requested, 6.0));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<TrackTarget>::WrappedResult result;
  ASSERT_TRUE(waitForResult<TrackTarget>(track_client_, handle, result, 60.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;
  EXPECT_EQ(result.result->message.find("has not been seen"), std::string::npos)
    << result.result->message;
  EXPECT_NEAR(
    distance3(vehiclePosition(), targetPosition()), expectedStandoff(requested), 0.25)
    << "a brief coast moved the standoff: " << result.result->message;
}

// Was: refused at the door, because tracking published nothing for the advisor to
// check. The carrot leg is published now, so the advice policy covers it instead.
TEST_F(NavigatorTargetRequiredFeedFixture, SilenceStopsTheChaseAndClearingItStartsAgain)
{
  advice_mode_ = AdviceMode::SILENT;
  ASSERT_TRUE(flyToCruise());
  setTarget(Vec3{5.0, 0.0, kCruiseZ}, Vec3{});
  const double standoff = expectedStandoff(2.0);

  auto handle = sendGoal<TrackTarget>(track_client_, trackGoal(2.0, 0.0));
  ASSERT_NE(handle, nullptr)
    << "the leg is published now, so a required map is no reason to refuse the goal";

  // The negative claim: with nothing checking ahead, the setpoint stops.
  ASSERT_TRUE(
    waitFor(
      [&]() {
        const Vec3 first = lastCommandedPosition();
        sleepFor(0.5);
        return distance3(first, lastCommandedPosition()) < 0.02;
      }, 20.0))
    << "the chase ran on with nothing checking the way ahead";
  const Vec3 held = lastCommandedPosition();
  EXPECT_GT(distance3(vehiclePosition(), targetPosition()), standoff + 0.5)
    << "the aircraft closed to its standoff while the advisor was silent";

  // Positive control (R21): the same rig, same goal, advisor speaking. Without this
  // the freeze above would also pass if tracking had simply stopped working.
  advice_mode_ = AdviceMode::CLEAR;
  EXPECT_TRUE(
    waitFor(
      [&]() {return distance3(lastCommandedPosition(), held) > 0.5;}, 20.0))
    << "a checked horizon never released the hold";
  EXPECT_TRUE(
    waitFor(
      [&]() {
        return std::abs(distance3(vehiclePosition(), targetPosition()) - standoff) < 0.35;
      }, 30.0))
    << "the chase never reached its standoff after the hold cleared";

  track_client_->async_cancel_goal(handle);
  rclcpp_action::ClientGoalHandle<TrackTarget>::WrappedResult result;
  ASSERT_TRUE(waitForResult<TrackTarget>(track_client_, handle, result, 30.0));
  EXPECT_NE(result.result->message.find("avoidance hold"), std::string::npos)
    << result.result->message;
  EXPECT_NE(result.result->message.find("obstacle advisor"), std::string::npos)
    << result.result->message;
}

TEST_F(NavigatorTargetAdviceFixture, AnEscapeIsHeldNotFlownWhileTheChaseRidesTheCarrot)
{
  ASSERT_TRUE(flyToCruise());
  setTarget(Vec3{5.0, 0.0, kCruiseZ}, Vec3{});
  // Far off to one side: flying it would be unmistakable in the shipped setpoints.
  setEscapePoint(Vec3{1.0, 3.0, kCruiseZ});

  auto handle = sendGoal<TrackTarget>(track_client_, trackGoal(2.0, 0.0));
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(waitFor([&]() {return lastCommandedPosition().x > 0.3;}, 20.0))
    << "the chase never got moving, so refusing an escape would prove nothing";

  startTrace();
  advice_mode_ = AdviceMode::ESCAPE;
  ASSERT_TRUE(
    waitFor(
      [&]() {
        const Vec3 first = lastCommandedPosition();
        sleepFor(0.5);
        return distance3(first, lastCommandedPosition()) < 0.02;
      }, 20.0))
    << "an escape advice kept the chase running";
  EXPECT_LT(peakLateral(), 0.5)
    << "the carrot leg was bent through an escape point: peak y " << peakLateral() << " m";

  // Positive control: the same advisor, saying the horizon is free, does move it.
  const Vec3 held = lastCommandedPosition();
  advice_mode_ = AdviceMode::CLEAR;
  EXPECT_TRUE(waitFor([&]() {return distance3(lastCommandedPosition(), held) > 0.3;}, 20.0))
    << "nothing moved even once the advisor cleared the way";

  track_client_->async_cancel_goal(handle);
  rclcpp_action::ClientGoalHandle<TrackTarget>::WrappedResult result;
  ASSERT_TRUE(waitForResult<TrackTarget>(track_client_, handle, result, 30.0));
  EXPECT_NE(result.result->message.find("escape is not flown on a carrot leg"), std::string::npos)
    << result.result->message;
}

TEST_F(NavigatorTargetAdviceFixture, TheLegTheChaseIsFlyingIsPublishedForTheAdvisor)
{
  // 🪤 BEFORE the takeoff, not after: the fake target publishes from SetUp, and its
  // default position is (0,0,0) -- which sits exactly under the aircraft. That is
  // precisely the degenerate case standoffPoint() guards, so a leg published in the
  // first 50 ms of tracking aims standoff metres along +x with no relation to the
  // real target. Setting it here gives the node seconds of correct target before
  // the goal, so the FIRST leg is about the target this test is asserting on.
  setTarget(Vec3{5.0, 0.0, kCruiseZ}, Vec3{});
  ASSERT_TRUE(flyToCruise());
  const Vec3 hover = lastCommandedPosition();
  forgetPlans();

  auto handle = sendGoal<TrackTarget>(track_client_, trackGoal(2.0, 0.0));
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(waitFor([&]() {return !carrotLegs().empty();}, 20.0))
    << "a carrot flight published no leg at all, so the advisor sees nothing";

  const Trajectory3D leg = carrotLegs().front();
  EXPECT_EQ(leg.plan_state, Trajectory3D::PLAN_STATE_VALID);
  EXPECT_TRUE(leg.is_valid);
  EXPECT_EQ(leg.header.frame_id, "odom");
  EXPECT_GE(leg.sequence, 1U);

  const Vec3 from{
    leg.points.front().position.x, leg.points.front().position.y,
    leg.points.front().position.z};
  const Vec3 to{
    leg.points.back().position.x, leg.points.back().position.y, leg.points.back().position.z};
  EXPECT_LT(distance3(from, hover), 0.3) << "the leg does not start where the setpoint is";

  // Where the leg ENDS is pinned by the two properties decision 4 is about, not by
  // an absolute point: standoffPoint() puts the aim exactly `standoff` from the
  // target along whatever bearing the aircraft happens to hold, so the radius is
  // exact for every leg while the point itself moves with the aircraft.
  const Vec3 target{5.0, 0.0, kCruiseZ};
  RecordProperty("leg_end_x", std::to_string(to.x));
  RecordProperty("leg_end_y", std::to_string(to.y));
  RecordProperty("leg_end_z", std::to_string(to.z));
  EXPECT_NEAR(std::hypot(to.x - target.x, to.y - target.y), expectedStandoff(2.0), 0.05)
    << "the leg does not end a standoff away from the target";
  // Its own altitude, never the target's: chasing that flies a ground target into
  // the ground. One tick of vertical travel is all the carrot can add in between.
  EXPECT_NEAR(to.z, from.z, max_vertical_speed_ / stream_hz_ + 1e-3)
    << "the chase leg changed altitude on its way to the standoff point";
  const double toward = (to.x - from.x) * (target.x - from.x) +
    (to.y - from.y) * (target.y - from.y);
  EXPECT_GT(toward, 0.0) << "the leg points away from the target it is chasing";

  EXPECT_DOUBLE_EQ(rclcpp::Duration(leg.points.front().time_from_start).seconds(), 0.0);
  const double seconds = rclcpp::Duration(leg.points.back().time_from_start).seconds();
  EXPECT_NEAR(seconds, expectedLegSeconds(from, to), 0.05)
    << "the leg claims a duration the carrot does not fly";
  const double speed = std::hypot(leg.points.front().velocity.x, leg.points.front().velocity.y);
  EXPECT_NEAR(speed, max_speed_, 0.05) << "the leg claims a speed the carrot may not fly";
  EXPECT_DOUBLE_EQ(leg.points.back().velocity.x, 0.0)
    << "the leg claims the carrot keeps going past its own end";

  track_client_->async_cancel_goal(handle);
  rclcpp_action::ClientGoalHandle<TrackTarget>::WrappedResult result;
  ASSERT_TRUE(waitForResult<TrackTarget>(track_client_, handle, result, 30.0));
}

TEST_F(NavigatorTargetAdviceFixture, AMovedTargetPutsAFreshLegOnTheWire)
{
  // Before the takeoff for the same reason as the test above: the first leg must be
  // about this target, not about the fake source's default (0,0,0).
  setTarget(Vec3{5.0, 0.0, kCruiseZ}, Vec3{});
  ASSERT_TRUE(flyToCruise());
  forgetPlans();

  auto handle = sendGoal<TrackTarget>(track_client_, trackGoal(2.0, 0.0));
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(waitFor([&]() {return !carrotLegs().empty();}, 20.0));
  const Trajectory3D first = carrotLegs().front();

  // The chase is a moving destination; the wire has to follow it.
  setTarget(Vec3{5.0, 4.0, kCruiseZ}, Vec3{});
  ASSERT_TRUE(
    waitFor(
      [&]() {
        const std::vector<Trajectory3D> legs = carrotLegs();
        return !legs.empty() && legs.back().points.back().position.y > 0.5;
      }, 20.0))
    << "the target moved and the published leg still pointed at where it had been";

  const Trajectory3D latest = carrotLegs().back();
  EXPECT_GT(latest.sequence, first.sequence) << "a new leg reused the old sequence";
  EXPECT_EQ(latest.points.size(), 2U);

  track_client_->async_cancel_goal(handle);
  rclcpp_action::ClientGoalHandle<TrackTarget>::WrappedResult result;
  ASSERT_TRUE(waitForResult<TrackTarget>(track_client_, handle, result, 30.0));
}


// The spline cuts each corner here by 0.33 m, so with a 0.15 m radius no interior
// waypoint is ever "achieved". Progress must still run to the end: if it stalled on
// the first cut corner, an escape would turn the aircraft round to collect it and
// the completion figure would freeze partway.
TEST_F(NavigatorFixture, ACutCornerStopsCountingButNeverStopsProgress)
{
  ASSERT_TRUE(flyToCruise());

  const std::vector<Vec3> waypoints{
    Vec3{1.0, 0.5, kCruiseZ}, Vec3{2.0, -0.5, kCruiseZ},
    Vec3{3.0, 0.5, kCruiseZ}, Vec3{4.0, 0.0, kCruiseZ}};

  FollowPath::Goal goal;
  goal.path.header.frame_id = "odom";
  goal.path.plan_state = Path3D::PLAN_STATE_VALID;
  goal.path.is_valid = true;
  for (const Vec3 & waypoint : waypoints) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = waypoint.x;
    pose.position.y = waypoint.y;
    pose.position.z = waypoint.z;
    pose.orientation.w = 1.0;
    goal.path.waypoints.push_back(pose);
  }
  goal.max_speed = 1.0F;
  goal.waypoint_acceptance_radius = 0.15F;

  std::mutex feedback_mutex;
  float best_completion = 0.0F;
  rclcpp_action::Client<FollowPath>::SendGoalOptions options;
  options.feedback_callback =
    [&](rclcpp_action::ClientGoalHandle<FollowPath>::SharedPtr,
      const std::shared_ptr<const FollowPath::Feedback> feedback) {
      std::lock_guard<std::mutex> lock(feedback_mutex);
      best_completion = std::max(best_completion, feedback->path_completion_percent);
    };

  auto handle = sendGoal<FollowPath>(follow_client_, goal, options);
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<FollowPath>::WrappedResult result;
  ASSERT_TRUE(waitForResult<FollowPath>(follow_client_, handle, result, 120.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  std::lock_guard<std::mutex> lock(feedback_mutex);
  EXPECT_FLOAT_EQ(best_completion, 100.0F)
    << "progress stalled on a cut corner at " << best_completion << " %";
  EXPECT_LT(result.result->waypoints_reached, waypoints.size())
    << "a 0.15 m radius against a 0.33 m corner cut should have missed waypoints, so "
    << "this case is not exercising the cut at all";
  EXPECT_NE(result.result->message.find("waypoints came inside"), std::string::npos)
    << "waypoints were missed in silence: " << result.result->message;
}


// ------------------------------------------------- the route must answer THIS goal

// Filtering by arrival time alone lets a route computed for the PREVIOUS goal in:
// at 5 Hz it lands up to 200 ms after the new question was asked.
TEST_F(NavigatorRouteFixture, ARouteAnsweringAnotherGoalIsNotFlown)
{
  stale_route_identity_ = true;
  ASSERT_TRUE(flyToCruise());
  startTrace();

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(3.0, 0.0, 1.0));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 90.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  EXPECT_FALSE(routeGoals().empty()) << "the navigator never asked";
  EXPECT_NE(result.result->message.find("route planner silent"), std::string::npos)
    << "a route about another goal was accepted as an answer: " << result.result->message;
  EXPECT_LT(peakLateral(), 0.25)
    << "the flight bent to a route that was never about this goal: " << peakLateral() << " m";
}

TEST_F(NavigatorRouteAndAdviceFixture, AnEscapeWillNotRejoinARouteAboutAnotherGoal)
{
  // Every route on the wire is about some other goal, from the first tick.
  stale_route_identity_ = true;
  ASSERT_TRUE(flyToCruise());
  setEscapePoint(Vec3{0.5, 0.8, kCruiseZ});
  advice_mode_ = AdviceMode::ESCAPE;
  startTrace();

  auto handle = sendGoal<GotoPose>(goto_client_, cruiseGoal(3.0, 0.0, 1.0));
  ASSERT_NE(handle, nullptr);

  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, handle, result, 120.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  EXPECT_NE(result.result->message.find("answers a different goal"), std::string::npos)
    << "the escape rejoined a route about another goal: " << result.result->message;
  // Only the escape point bends the flight, so it stays near the direct line.
  EXPECT_LT(peakLateral(), 0.9)
    << "a route about another goal shaped the flight: " << peakLateral() << " m";
}

// ------------------------------------------------------ decision 4 has a hard floor

TEST_F(NavigatorConfigFixture, TheMeasuredTargetVelocityErrorMayBeWidenedButNeverShrunk)
{
  // 0.708 m/s is what the sensor chain was measured to do, not a tuning choice, so
  // a smaller number is the system lying about what it knows.
  EXPECT_THROW(
    build({rclcpp::Parameter("target_velocity_error_mps", 0.001)}), std::invalid_argument);
  EXPECT_THROW(
    build({rclcpp::Parameter("target_velocity_error_mps", 0.7)}), std::invalid_argument);
  EXPECT_NO_THROW(build({rclcpp::Parameter("target_velocity_error_mps", 1.5)}))
    << "a wider margin than measured must still be allowed";
  EXPECT_NO_THROW(build({rclcpp::Parameter("target_velocity_error_mps", 0.71)}))
    << "the shipped value must start";

  // The margin is a PRODUCT, so a floor on one factor alone is no floor at all.
  EXPECT_THROW(
    build({rclcpp::Parameter("target_reaction_sec", 0.001)}), std::invalid_argument)
    << "the reaction time was the way round the velocity floor";
  EXPECT_NO_THROW(build({rclcpp::Parameter("target_reaction_sec", 2.0)}))
    << "a longer reaction time than decided must still be allowed";
}

TEST_F(NavigatorConfigFixture, ARecoveryMayNotBeAllowedToClimbWhereAGoalIsRefused)
{
  EXPECT_THROW(
    build(
      {rclcpp::Parameter("max_altitude_m", 20.0),
        rclcpp::Parameter("recovery.max_safe_altitude_m", 25.0)}), std::invalid_argument)
    << "two constants for one ceiling is how a recovery outflies the envelope";
  EXPECT_NO_THROW(
    build(
      {rclcpp::Parameter("max_altitude_m", 20.0),
        rclcpp::Parameter("recovery.max_safe_altitude_m", 20.0)}));
  EXPECT_THROW(
    build({rclcpp::Parameter("recovery.max_home_distance_m", 0.0)}), std::invalid_argument)
    << "a zero range is a return home that can never be planned";
}

TEST_F(NavigatorConfigFixture, ACarrotLegMayNotBeRepublishedFasterThanTheAdvisorAnswers)
{
  // 85 ms was measured for one advisor round trip; below the floor every answer is
  // about a leg already replaced, which the navigator reads as silence.
  EXPECT_THROW(
    build({rclcpp::Parameter("carrot_plan_period_sec", 0.05)}), std::invalid_argument);
  EXPECT_THROW(
    build({rclcpp::Parameter("carrot_plan_period_sec", 0.0)}), std::invalid_argument);
  EXPECT_NO_THROW(build({rclcpp::Parameter("carrot_plan_period_sec", 0.5)}))
    << "the shipped value must start";
}

// ------------------------------------------------------------------- recover

TEST_F(NavigatorFixture, TheThreeRecoveriesThisNodeMustNotChooseAreRefusedAtTheDoor)
{
  ASSERT_TRUE(flyToCruise());

  // Landing and handing the aircraft back are decisions, not trajectories: they
  // belong to the safety and authority layers (contract 2.5).
  EXPECT_TRUE(
    goalWasRejected(
      sendGoal<Recover>(recover_client_, recoverGoal(Recover::Goal::TYPE_LAND, kCruiseZ))));
  EXPECT_TRUE(
    goalWasRejected(
      sendGoal<Recover>(
        recover_client_, recoverGoal(Recover::Goal::TYPE_HANDOVER_TO_PILOT, kCruiseZ))));
  // The goal chooses; a goal that chose nothing must not be given a default.
  EXPECT_TRUE(
    goalWasRejected(
      sendGoal<Recover>(recover_client_, recoverGoal(Recover::Goal::TYPE_UNKNOWN, kCruiseZ))));

  // Positive control: the same door, the same moment, a type this node does fly.
  auto handle =
    sendGoal<Recover>(recover_client_, recoverGoal(Recover::Goal::TYPE_HOLD, kCruiseZ));
  ASSERT_NE(handle, nullptr) << "every recovery was refused, so the refusals prove nothing";
  rclcpp_action::ClientGoalHandle<Recover>::WrappedResult result;
  ASSERT_TRUE(waitForResult<Recover>(recover_client_, handle, result, 30.0));
  EXPECT_EQ(result.result->executed_type, Recover::Goal::TYPE_HOLD);
}

TEST_F(NavigatorFixture, ARefusedRecoveryDisturbsNeitherTheFlightNorTheNextGoal)
{
  ASSERT_TRUE(flyToCruise());

  auto flying = sendGoal<GotoPose>(goto_client_, cruiseGoal(2.0, 0.0, 1.0));
  ASSERT_NE(flying, nullptr);
  ASSERT_TRUE(waitFor([&]() {return lastCommandedPosition().x > 0.3;}, 20.0));

  EXPECT_TRUE(
    goalWasRejected(
      sendGoal<Recover>(recover_client_, recoverGoal(Recover::Goal::TYPE_LAND, kCruiseZ))));

  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult result;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, flying, result, 60.0));
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED)
    << "a recovery refused at the door still took the flight down: " << result.result->message;
}

TEST_F(NavigatorFixture, ARecoveryTheGeometryRefusesAbortsAndSaysWhy)
{
  ASSERT_TRUE(flyToCruise());

  // A safe altitude below the aircraft: recovery never descends.
  auto handle = sendGoal<Recover>(
    recover_client_, recoverGoal(Recover::Goal::TYPE_CLIMB_TO_SAFE_ALTITUDE, kCruiseZ - 1.0));
  ASSERT_NE(handle, nullptr) << "the geometry is judged against the anchor, not at the door";
  rclcpp_action::ClientGoalHandle<Recover>::WrappedResult result;
  ASSERT_TRUE(waitForResult<Recover>(recover_client_, handle, result, 30.0));

  EXPECT_EQ(result.result->result_code, ResultCode::ABORTED_INVALID_GOAL) << result.result->message;
  EXPECT_EQ(result.result->executed_type, Recover::Goal::TYPE_UNKNOWN)
    << "a plan that never flew still reported a type as flown";
  EXPECT_NE(result.result->message.find("refused"), std::string::npos) << result.result->message;

  // The navigator must still be usable: a refusal is not a wedged state machine.
  HoldPosition::Goal hold;
  hold.duration_seconds = 0.5F;
  EXPECT_NE(sendGoal<HoldPosition>(hold_client_, hold), nullptr);
}

TEST_F(NavigatorTrackingEnvelopeFixture, AHoldRecoveryStopsTheAircraftWhereItStands)
{
  ASSERT_TRUE(flyToCruise());
  startTrace();
  const Vec3 parked = lastCommandedPosition();
  const int before = command_count_.load();
  const double opened = probe_->now().seconds();

  // Witness for the HOST, not for the probe (N-g watched the reader). If the
  // machine cannot schedule a 20 ms sleep on time, it cannot deliver stream_hz
  // either, and a rate floor is then measuring the machine. RAII because an
  // ASSERT below returns straight out of this function.
  HostSleepWitness host_witness;

  auto handle =
    sendGoal<Recover>(recover_client_, recoverGoal(Recover::Goal::TYPE_HOLD, kCruiseZ));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<Recover>::WrappedResult result;
  ASSERT_TRUE(waitForResult<Recover>(recover_client_, handle, result, 30.0))
    << (last_result_was_unknown_
    ? "FAILED TO MEASURE: rclcpp_action returned UNKNOWN, no result was delivered"
    : "the recovery never returned a result");

  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;
  EXPECT_EQ(result.result->executed_type, Recover::Goal::TYPE_HOLD);
  EXPECT_LT(distance3(parked, lastCommandedPosition()), 0.05) << "a hold moved the aircraft";

  // A hold recovery is over in about arrival_settle_sec, so the window has to be
  // the one that was actually measured; half a second of it does not exist. The
  // extra wait is part of the claim: the stream must also survive the goal ending.
  sleepFor(0.5);
  const double elapsed = probe_->now().seconds() - opened;
  const double host_overshoot = host_witness.stopAndWorstSec();
  double arrival_lag = 0.0;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    arrival_lag = max_arrival_lag_sec_;
  }
  const StreamWindow window = measureStream(opened, opened + elapsed);
  RecordProperty("stream_longest_hole_sec", std::to_string(window.gap));
  RecordProperty("host_sleep_overshoot_sec", std::to_string(host_overshoot));

  // What loses offboard is the stream STOPPING, not the machine being slow: a rate
  // floor built from the configured stream_hz convicts a busy host, which is what
  // it did (measured 2026-08-25 under load: 37 arrivals in 3.30 s = 11 Hz against a
  // 25 Hz floor, while the probe was only 0.0095 s behind -- nothing was dropped,
  // the whole process was starved). Judge the longest hole instead, and let the
  // host's own scheduling error set the floor it cannot beat.
  ASSERT_GT(window.samples, 1) << "FAILED TO MEASURE: fewer than two setpoints traced";
  const double allowed_hole =
    std::max(kMaxStreamHoleTicks * tickSeconds(), host_overshoot + 2.0 * tickSeconds());
  EXPECT_LT(window.gap, allowed_hole)
    << "the stream stopped during a recovery, which is how offboard is lost"
    << "\n  elapsed " << elapsed << " s, arrivals " << (command_count_.load() - before)
    << ", traced " << window.samples << ", one tick " << tickSeconds() << " s"
    << "\n  worst probe arrival lag " << arrival_lag
    << " s, worst host 20 ms sleep overshoot " << host_overshoot << " s";
}

// The recovery the safety layer still has when localization goes. Every other task
// aborts on a disowned pose, by design; a hover must not, or the only escalation
// left from "lost localization" is LAND or HANDOVER (contract 2.5).
TEST_F(NavigatorTrackingEnvelopeFixture, AHoldRecoverySurvivesThePoseItCannotRead)
{
  ASSERT_TRUE(flyToCruise());
  const Vec3 parked = lastCommandedPosition();

  // Both localization faults at once: the status disowned AND the stamp long stale.
  localization_valid_ = false;
  ageThePose(30.0);
  const int settled = command_count_.load();
  ASSERT_TRUE(waitFor([&]() {return command_count_.load() - settled > 10;}, 10.0))
    << "the stream stopped before the disowned pose could even reach the node";

  // Positive control (R21): the same rig, the same moment, a task that DOES read the
  // pose is refused at the door. Without it a passing hover would not prove the
  // disowning ever landed.
  EXPECT_TRUE(goalWasRejected(sendGoal<GotoPose>(goto_client_, cruiseGoal(1.0, 0.0, 1.0))))
    << "a disowned pose did not even stop a goal that needs one";

  const int before = command_count_.load();
  const double opened = probe_->now().seconds();
  auto handle =
    sendGoal<Recover>(recover_client_, recoverGoal(Recover::Goal::TYPE_HOLD, kCruiseZ));
  ASSERT_NE(handle, nullptr) << "the one recovery that needs no pose was refused for want of one";
  rclcpp_action::ClientGoalHandle<Recover>::WrappedResult result;
  ASSERT_TRUE(waitForResult<Recover>(recover_client_, handle, result, 30.0));

  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;
  EXPECT_EQ(result.result->executed_type, Recover::Goal::TYPE_HOLD);
  // It knew, and it held anyway: the evidence has to say so, not hide it.
  EXPECT_NE(result.result->message.find("is_valid=false"), std::string::npos)
    << result.result->message;
  EXPECT_LT(distance3(parked, lastCommandedPosition()), 0.05)
    << "the hover moved the aircraft while it could not read where it was";

  sleepFor(0.5);
  const double elapsed = probe_->now().seconds() - opened;
  EXPECT_GT(command_count_.load() - before, expectedCommands(elapsed))
    << "the stream stopped, which is how offboard is lost while holding";
}

TEST_F(NavigatorTrackingEnvelopeFixture, AClimbRecoveryReachesTheSafeAltitudeItWasGiven)
{
  ASSERT_TRUE(flyToCruise());
  const double apex = kCruiseZ + 1.0;

  std::mutex stage_mutex;
  std::vector<std::pair<uint8_t, std::string>> stages;
  rclcpp_action::Client<Recover>::SendGoalOptions options;
  options.feedback_callback =
    [&](rclcpp_action::ClientGoalHandle<Recover>::SharedPtr,
      const std::shared_ptr<const Recover::Feedback> feedback) {
      std::lock_guard<std::mutex> lock(stage_mutex);
      stages.emplace_back(feedback->current_stage, feedback->stage_description);
    };

  auto handle = sendGoal<Recover>(
    recover_client_, recoverGoal(Recover::Goal::TYPE_CLIMB_TO_SAFE_ALTITUDE, apex), options);
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<Recover>::WrappedResult result;
  ASSERT_TRUE(waitForResult<Recover>(recover_client_, handle, result, 90.0));

  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;
  EXPECT_EQ(result.result->executed_type, Recover::Goal::TYPE_CLIMB_TO_SAFE_ALTITUDE);
  EXPECT_NEAR(vehiclePosition().z, apex, 0.35)
    << "the aircraft stopped short of the safe altitude it was sent to";
  EXPECT_LT(std::hypot(vehiclePosition().x, vehiclePosition().y), 0.35)
    << "a climb wandered off the spot it started from";

  std::lock_guard<std::mutex> lock(stage_mutex);
  ASSERT_FALSE(stages.empty()) << "a recovery reported no stage at all";
  EXPECT_EQ(stages.front().first, 1U) << "the first stage of a climb is climbing";
  EXPECT_FALSE(stages.front().second.empty()) << "a stage without a description";
  EXPECT_EQ(stages.back().first, 3U) << "the last stage of a climb is holding";
}

TEST_F(NavigatorTrackingEnvelopeFixture, AReturnHomeRecoveryEndsHoveringOverTheLaunchPoint)
{
  ASSERT_TRUE(flyToCruise());
  const Vec3 launch = vehiclePosition();

  auto away = sendGoal<GotoPose>(goto_client_, cruiseGoal(2.0, 0.0, max_speed_));
  ASSERT_NE(away, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult flown;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, away, flown, 90.0));
  ASSERT_EQ(flown.code, rclcpp_action::ResultCode::SUCCEEDED) << flown.result->message;

  const double cruise = kCruiseZ + 0.5;
  auto handle = sendGoal<Recover>(
    recover_client_, recoverGoal(Recover::Goal::TYPE_RETURN_HOME, cruise));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<Recover>::WrappedResult result;
  ASSERT_TRUE(waitForResult<Recover>(recover_client_, handle, result, 120.0));

  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;
  EXPECT_EQ(result.result->executed_type, Recover::Goal::TYPE_RETURN_HOME);
  EXPECT_LT(std::hypot(vehiclePosition().x - launch.x, vehiclePosition().y - launch.y), 0.35)
    << "the aircraft did not come back over where it left from";
  // Ends HOVERING above home, never at it: coming down is a separate, authorised act.
  EXPECT_NEAR(vehiclePosition().z, cruise, 0.35);
  EXPECT_GT(vehiclePosition().z - kGroundZ, 0.5)
    << "a return home came down on home, which is a landing nobody authorised";
}

TEST_F(NavigatorTrackingEnvelopeFixture, ARecoveryTakesOverAFlightWithoutSteppingTheSetpoint)
{
  ASSERT_TRUE(flyToCruise());

  startTrace();
  auto flying = sendGoal<GotoPose>(goto_client_, cruiseGoal(4.0, 0.0, max_speed_));
  ASSERT_NE(flying, nullptr);
  ASSERT_TRUE(waitFor([&]() {return lastCommandedPosition().x > 0.5;}, 30.0))
    << "the flight never got moving, so taking it over would prove nothing";

  const double takeover = probe_->now().seconds();
  auto handle =
    sendGoal<Recover>(recover_client_, recoverGoal(Recover::Goal::TYPE_HOLD, kCruiseZ));
  ASSERT_NE(handle, nullptr) << "a recovery must preempt, not be refused as busy";

  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult preempted;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, flying, preempted, 30.0));
  EXPECT_EQ(preempted.result->result_code, ResultCode::ABORTED_SAFETY) << preempted.result->message;
  EXPECT_NE(preempted.result->message.find("preempted by a recovery"), std::string::npos)
    << preempted.result->message;

  rclcpp_action::ClientGoalHandle<Recover>::WrappedResult result;
  ASSERT_TRUE(waitForResult<Recover>(recover_client_, handle, result, 60.0));
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  // Measured on the SHIPPED setpoints, never on a member of the node, and over the
  // window the claim is about: one second either side of the takeover, the same
  // window the G-N6 flight gate uses. A whole-trace measurement would also cover
  // the plan publish and the goal handshakes, which say nothing about continuity.
  sleepFor(1.0);
  const double edge = 1.0;
  const StreamWindow across = measureStream(takeover - edge, takeover + edge);
  const double ceiling = 3.0 * std::hypot(max_speed_, max_vertical_speed_) / stream_hz_;
  RecordProperty("takeover_step_m", std::to_string(across.step));
  RecordProperty("takeover_gap_s", std::to_string(across.gap));
  RecordProperty("whole_trace_gap_s", std::to_string(longestTraceGap()));
  ASSERT_GT(across.samples, 0) << "FAILED TO MEASURE: no setpoint landed in the window";
  EXPECT_LT(across.step, ceiling)
    << "the setpoint jumped " << across.step << " m at the takeover, ceiling " << ceiling;
  // PX4 leaves offboard below 2 Hz, so half a second of nothing is the real limit.
  EXPECT_LT(across.gap, 0.5)
    << "the stream stopped for " << across.gap << " s across the takeover";
  EXPECT_GT(across.samples, expectedCommands(2.0 * edge))
    << "the stream did not keep up across the takeover: " << across.samples << " setpoints";
}

TEST_F(NavigatorFixture, ARecoveryPublishesThePlanItFlies)
{
  ASSERT_TRUE(flyToCruise());
  forgetPlans();
  const double apex = kCruiseZ + 1.0;

  auto handle = sendGoal<Recover>(
    recover_client_, recoverGoal(Recover::Goal::TYPE_CLIMB_TO_SAFE_ALTITUDE, apex));
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ClientGoalHandle<Recover>::WrappedResult result;
  ASSERT_TRUE(waitForResult<Recover>(recover_client_, handle, result, 60.0));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << result.result->message;

  const std::vector<Trajectory3D> published = plans();
  const auto valid = std::find_if(
    published.begin(), published.end(), [](const Trajectory3D & entry) {return entry.is_valid;});
  ASSERT_NE(valid, published.end()) << "a recovery flew with nothing on the plan topic";
  EXPECT_EQ(valid->plan_state, Trajectory3D::PLAN_STATE_VALID);
  EXPECT_GT(valid->points.size(), 2U)
    << "the recovery did not go through the ordinary trajectory core";
  EXPECT_NEAR(valid->points.back().position.z, apex, 0.05)
    << "the published plan does not end at the safe altitude";
}

TEST_F(NavigatorRecoveryAdviceFixture, ASilentAdvisorHoldsARecoveryWhenTheMapIsRequired)
{
  ASSERT_TRUE(flyToCruise());
  auto away = sendGoal<GotoPose>(goto_client_, cruiseGoal(2.5, 0.0, 1.0));
  ASSERT_NE(away, nullptr);
  rclcpp_action::ClientGoalHandle<GotoPose>::WrappedResult flown;
  ASSERT_TRUE(waitForResult<GotoPose>(goto_client_, away, flown, 60.0));
  ASSERT_EQ(flown.code, rclcpp_action::ResultCode::SUCCEEDED) << flown.result->message;

  // The advisor keeps speaking through the first part of the recovery, and is cut
  // off only once the flight is visibly under way. Cutting it at second zero is a
  // legitimate hold from the start -- correct behaviour, but then the setpoint never
  // moves and there is nothing to distinguish the hold from the rest-start of the
  // plan (the trajectory core begins AND ends at rest). Same shape as the tracking
  // twin above, which has no rest-start and so needs no such care.
  const Vec3 start = lastCommandedPosition();
  auto handle = sendGoal<Recover>(
    recover_client_, recoverGoal(Recover::Goal::TYPE_RETURN_HOME, kCruiseZ + 0.4));
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(waitFor([&]() {return distance3(lastCommandedPosition(), start) > 0.3;}, 30.0))
    << "the recovery never got moving, so freezing it would prove nothing";

  const double silent_from = probe_->now().seconds();
  advice_mode_ = AdviceMode::SILENT;
  ASSERT_TRUE(
    waitFor(
      [&]() {
        const Vec3 first = lastCommandedPosition();
        sleepFor(0.4);
        return distance3(first, lastCommandedPosition()) < 0.02;
      }, 20.0))
    << "the recovery flew home with nothing checking the way";
  const Vec3 held = lastCommandedPosition();

  // The precondition of the whole claim, asserted rather than assumed: silence must
  // have outlasted advice_timeout_sec before a stopped setpoint can be the advisor's
  // doing. A trajectory starts and ends AT REST, so an early freeze is the plan
  // ramping up, and reading that as a hold is how this test passed for the wrong
  // reason. If this line fails, the freeze came too early - not the node's fault.
  EXPECT_GT(probe_->now().seconds() - silent_from, advice_timeout_sec_)
    << "the setpoint stopped before silence could even be called";
  EXPECT_GT(std::hypot(vehiclePosition().x, vehiclePosition().y), 0.3)
    << "the recovery got home before anything checked the way there";

  // Positive control (R21): the same advisor, speaking, lets the same recovery run.
  advice_mode_ = AdviceMode::CLEAR;
  EXPECT_TRUE(waitFor([&]() {return distance3(lastCommandedPosition(), held) > 0.3;}, 20.0))
    << "a checked horizon never released the recovery";

  recover_client_->async_cancel_goal(handle);
  rclcpp_action::ClientGoalHandle<Recover>::WrappedResult result;
  ASSERT_TRUE(waitForResult<Recover>(recover_client_, handle, result, 30.0));
  EXPECT_NE(result.result->message.find("avoidance hold"), std::string::npos)
    << result.result->message;
  EXPECT_NE(result.result->message.find("obstacle advisor"), std::string::npos)
    << result.result->message;
  expectTheTrajectoryWasFlown(result.result->message);
}
