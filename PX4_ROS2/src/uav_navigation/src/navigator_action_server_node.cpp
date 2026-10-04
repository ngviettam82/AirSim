// Action gateway the mission layer calls: takeoff, goto, hold, land.

#include "uav_navigation/navigator_action_server_node.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

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
#include <uav_interfaces/msg/trajectory_point.hpp>
#include <uav_interfaces/msg/vehicle_state.hpp>
#include <uav_interfaces/srv/arm.hpp>
#include <uav_interfaces/srv/disarm.hpp>
#include <uav_interfaces/srv/set_flight_mode.hpp>

#include "uav_navigation/carrot.hpp"
#include "uav_navigation/goal_admission.hpp"
#include "uav_navigation/recovery_planner.hpp"
#include "uav_navigation/trajectory.hpp"

namespace uav_navigation
{

namespace
{

using geometry_msgs::msg::PoseStamped;
// Duplicate waypoints make zero-length spline segments; well under costmap res.
constexpr double kMinWaypointSpacing = 0.05;

// Under this the advisor is riding the flight band, not routing the drone down.
constexpr double kEscapeDescentNotice = 0.5;

// A carrot whose target is itself cannot move, so a hover setpoint that drifted at
// all was driven by something else.
constexpr double kFrozenSetpointDrift = 0.01;

// MEASURED 2026-08-18: target velocity error on a 1 m/s target.
constexpr double kMeasuredTargetVelocityError = 0.708;

constexpr double kDecisionFourReactionSeconds = 1.0;

// MEASURED 2026-08-19 (G-N4a): one advisor round trip, 85.2 ms.
constexpr double kMinCarrotPlanPeriod = 0.2;

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
using uav_interfaces::msg::TrajectoryPoint;
using uav_interfaces::msg::VehicleState;
using uav_interfaces::srv::Arm;
using uav_interfaces::srv::Disarm;
using uav_interfaces::srv::SetFlightMode;

constexpr uint8_t kOffboardUnknown = 255;

// A grid this long is a two-minute flight; say so rather than ship it silently.
constexpr std::size_t kLoudTrajectoryPoints = 5000;

/// PX4 flies the aircraft itself in these modes; offboard must not fight it.
bool autopilotOwnsTheFlight(uint8_t flight_mode)
{
  return flight_mode == VehicleState::FLIGHT_MODE_LAND ||
         flight_mode == VehicleState::FLIGHT_MODE_RETURN ||
         flight_mode == VehicleState::FLIGHT_MODE_TAKEOFF ||
         flight_mode == VehicleState::FLIGHT_MODE_MISSION;
}

/// Distance from a point to the leg the aircraft is flying. Without a costmap this
/// is the only thing the navigator can check about an advisor's proposal.
double deviationFromLeg(const Vec3 & point, const Vec3 & from, const Vec3 & to)
{
  const Vec3 leg{to.x - from.x, to.y - from.y, to.z - from.z};
  const double length2 = leg.x * leg.x + leg.y * leg.y + leg.z * leg.z;
  if (!(length2 > 1e-9)) {
    return distance3(point, from);
  }
  const double along =
    ((point.x - from.x) * leg.x + (point.y - from.y) * leg.y + (point.z - from.z) * leg.z) /
    length2;
  const double clamped = std::max(0.0, std::min(1.0, along));
  const Vec3 nearest{
    from.x + clamped * leg.x, from.y + clamped * leg.y, from.z + clamped * leg.z};
  return distance3(point, nearest);
}

/// Where the altitude of an intermediate waypoint comes from.
enum class ViaAltitude
{
  PROFILE,   // route waypoints carry the flight band, so spread the climb here
  KEEP,      // the advisor chose this height on purpose; do not overwrite it
};

enum class LoopOutcome
{
  REACHED,
  CANCELED,
  PREEMPTED,   // a Recover goal took the aircraft over
  TIMEOUT,
  FAULT,
};

/// No stage constants exist in Recover.action, so these are the node's own, and
/// they are derived from the measured pose rather than from a step counter.
enum RecoveryStage : uint8_t
{
  RECOVERY_STAGE_CLIMBING = 1,
  RECOVERY_STAGE_CRUISING = 2,
  RECOVERY_STAGE_HOLDING = 3,
};

/// What a carrot leg has on /planning/trajectory right now.
enum class CarrotPlan
{
  NONE,
  SEGMENT,   // a two point leg the advisor can check
  PARKED,    // nothing planned; the setpoint is standing still
};

struct Fault
{
  bool present = false;
  bool hand_over = false;     // autopilot has the aircraft, so stop streaming
  uint8_t result_code = ResultCode::UNKNOWN;
  std::string detail;
};

/// Everything needed to rebuild the flight to the same goal mid-task.
struct GotoPlanRequest
{
  Vec3 target;
  std::optional<double> requested_yaw;
  double max_speed = 0.0;
  double acceptance = 0.0;
  /// Waypoints still ahead of an escape point. Whoever owns the shape of the flight
  /// supplies them: the live A* route for a Goto, the caller list for a path.
  std::function<std::vector<Vec3>(const Vec3 &)> tailAfter;
  ViaAltitude tail_altitude = ViaAltitude::PROFILE;
};

/// Task-thread only: how far along a caller-supplied path the aircraft has come.
struct PathProgress
{
  std::vector<bool> achieved;
  std::size_t ahead = 0;
};

struct TargetSample
{
  bool usable = false;
  Vec3 position;
  Vec3 velocity;
  double uncertainty = -1.0;
  std::string why;
};

struct PoseSample
{
  Vec3 position;
  double yaw = 0.0;
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  bool valid = false;
};

/// What the aircraft is being commanded right now, read under one lock so a
/// published leg cannot mix a setpoint from one tick with a speed from another.
struct CommandedMotion
{
  Vec3 setpoint;
  double yaw = 0.0;
  CarrotLimits limits;
  bool on_the_carrot = false;
};

class NavigatorActionServerNode : public rclcpp::Node
{
public:
  explicit NavigatorActionServerNode(const rclcpp::NodeOptions & options)
  : Node("navigator_action_server_node", options)
  {
    uav_id_ = declare_parameter<std::string>("uav_id", "uav0");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    stream_hz_ = std::max(2.0, declare_parameter<double>("stream_hz", 20.0));

    limits_.max_speed = declare_parameter<double>("max_speed", 0.55);
    limits_.max_vertical_speed = declare_parameter<double>("max_vertical_speed", 0.45);
    limits_.acceptance_radius = declare_parameter<double>("acceptance_radius", 0.3);
    acceptance_sigma_factor_ = declare_parameter<double>("acceptance_sigma_factor", 2.0);
    max_acceptance_radius_ = declare_parameter<double>("max_acceptance_radius", 1.5);
    limits_.max_takeoff_altitude = declare_parameter<double>("max_takeoff_altitude_m", 10.0);
    limits_.min_altitude = declare_parameter<double>("min_altitude_m", 0.5);
    limits_.max_altitude = declare_parameter<double>("max_altitude_m", 20.0);

    max_lead_horizontal_ = declare_parameter<double>("max_lead_horizontal_m", 0.8);
    max_lead_vertical_ = declare_parameter<double>("max_lead_vertical_m", 0.6);
    leash_stall_fault_ = declare_parameter<double>("leash_stall_fault_sec", 3.0);
    takeoff_stall_grace_ = declare_parameter<double>("takeoff_stall_grace_sec", 5.0);
    command_loss_debounce_ = declare_parameter<double>("command_loss_debounce_sec", 0.5);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate_rad_s", 0.5);
    max_acceleration_ = declare_parameter<double>("max_acceleration", 1.0);
    sample_hz_ = declare_parameter<double>("sample_hz", 50.0);
    // False flies the P6.1 carrot: the rollback, and the control the gate compares against.
    use_trajectory_ = declare_parameter<bool>("use_trajectory", true);
    // False flies the straight line P6.1-P6.2 flew: the rollback for the route hop.
    use_route_ = declare_parameter<bool>("use_route", true);
    route_timeout_ = declare_parameter<double>("route_timeout_sec", 2.0);
    // Real flight may not plan blind (plan P6 decision 4).
    require_obstacle_feed_ = declare_parameter<bool>("require_obstacle_feed", false);
    // The rollback for the advice hop, and deliberately a flag rather than a zero
    // timeout: a safety machine switched off by a number goes off unnoticed.
    use_avoidance_ = declare_parameter<bool>("use_avoidance", true);
    // Silence on the advice topic is never read as clear air; this is how long the
    // navigator waits before saying so. Ten missed ticks of a 10 Hz advisor.
    advice_timeout_ = declare_parameter<double>("advice_timeout_sec", 1.0);
    // The navigator has no costmap, so it cannot judge whether an escape point is
    // clear. It can judge whether the point is anywhere near the flight it planned.
    max_escape_deviation_ = declare_parameter<double>("max_escape_deviation_m", 8.0);
    escape_replan_interval_ = declare_parameter<double>("escape_replan_interval_sec", 1.0);
    escape_refresh_ = declare_parameter<double>("escape_refresh_m", 0.5);
    // How often a carrot leg reappears on /planning/trajectory, which is the only
    // thing the advisor can check. Not the stream rate: see kMinCarrotPlanPeriod.
    carrot_plan_period_ = declare_parameter<double>("carrot_plan_period_sec", 0.5);
    // A hold that never clears must end as a hold, not as a goal timeout.
    avoidance_hold_timeout_ = declare_parameter<double>("avoidance_hold_timeout_sec", 12.0);
    // How old the live route may be before an escape stops rejoining it. The route
    // planner runs at 5 Hz, so this is five ticks.
    route_fresh_ = declare_parameter<double>("route_fresh_sec", 1.0);
    min_standoff_ = declare_parameter<double>("min_standoff_m", 1.0);
    // Decision 4: the target velocity estimate is wrong by 0.71 m/s on a 1 m/s
    // target, an information limit no filter removes, so the gap pays for it.
    target_velocity_error_ = declare_parameter<double>("target_velocity_error_mps", 0.71);
    target_reaction_ = declare_parameter<double>("target_reaction_sec", 1.0);
    // Zero by default: a second of lead buys 0.71 m of error at measured numbers.
    target_lead_ = declare_parameter<double>("target_lead_sec", 0.0);
    target_state_timeout_ = declare_parameter<double>("target_state_timeout_sec", 1.0);
    // A publish-time stamp proves liveness, not that anyone saw the target.
    target_sighting_timeout_ = declare_parameter<double>("target_sighting_timeout_sec", 1.0);
    target_sighting_period_copy_ =
      declare_parameter<double>("target_sighting_period_copy_sec", 1.0 / 15.0);
    tracker_lost_after_copy_ = declare_parameter<double>("tracker_lost_after_copy_sec", 3.0);
    feedback_hz_ = std::max(0.1, declare_parameter<double>("feedback_hz", 2.0));
    progress_hz_ = std::max(1.0, declare_parameter<double>("progress_rate_hz", 10.0));
    arrival_settle_ = declare_parameter<double>("arrival_settle_sec", 1.0);

    odometry_timeout_ = declare_parameter<double>("odometry_timeout_sec", 1.0);
    startup_timeout_ = declare_parameter<double>("startup_timeout_sec", 30.0);
    offboard_timeout_ = declare_parameter<double>("offboard_engage_timeout_sec", 30.0);
    service_timeout_ = declare_parameter<double>("service_timeout_sec", 15.0);
    takeoff_timeout_ = declare_parameter<double>("takeoff_timeout_sec", 60.0);
    goto_timeout_ = declare_parameter<double>("goto_timeout_sec", 240.0);
    hold_timeout_margin_ = declare_parameter<double>("hold_timeout_margin_sec", 30.0);
    land_stream_stop_delay_ = declare_parameter<double>("land_stream_stop_delay_sec", 1.0);
    disarm_timeout_ = declare_parameter<double>("disarm_timeout_sec", 40.0);

    RecoveryLimits recovery_limits;
    recovery_limits.min_travel_m =
      declare_parameter<double>("recovery.min_travel_m", recovery_limits.min_travel_m);
    recovery_limits.max_climb_m =
      declare_parameter<double>("recovery.max_climb_m", recovery_limits.max_climb_m);
    // Defaulted from the airframe ceiling, never repeated: two constants for one
    // limit is how a recovery ends up allowed where a goal is not.
    recovery_limits.max_safe_altitude_m =
      declare_parameter<double>("recovery.max_safe_altitude_m", limits_.max_altitude);
    recovery_limits.min_home_clearance_m =
      declare_parameter<double>("recovery.min_home_clearance_m", recovery_limits.min_home_clearance_m);
    recovery_limits.max_home_distance_m =
      declare_parameter<double>("recovery.max_home_distance_m", recovery_limits.max_home_distance_m);

    active_limits_ = defaultCarrotLimits();
    rejectUnusableTrajectoryLimits();
    rejectUnusableAvoidanceLimits();
    buildRecoveryPlanner(recovery_limits);

    stream_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    action_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = stream_group_;

    const std::string prefix = "/uav/" + uav_id_;

    command_publisher_ =
      create_publisher<ControlCommand>(prefix + "/control/cmd_mission", 10);

    // Latched: the plan in force, so a consumer joining late still sees it.
    trajectory_publisher_ = create_publisher<Trajectory3D>(
      prefix + "/planning/trajectory", rclcpp::QoS(1).reliable().transient_local());

    route_goal_publisher_ =
      create_publisher<PoseStamped>(prefix + "/planning/route_goal", rclcpp::QoS(1));

    route_subscription_ = create_subscription<Path3D>(
      prefix + "/planning/route", rclcpp::QoS(1),
      [this](const Path3D::SharedPtr message) {
        const std::lock_guard<std::mutex> guard(route_mutex_);
        route_ = message;
        route_stamp_ = now();
      },
      subscription_options);

    advice_subscription_ = create_subscription<AvoidanceAdvice>(
      prefix + "/planning/avoidance", rclcpp::QoS(1),
      [this](const AvoidanceAdvice::SharedPtr message) {
        const std::lock_guard<std::mutex> guard(advice_mutex_);
        advice_ = message;
      },
      subscription_options);

    target_subscription_ = create_subscription<TargetState>(
      prefix + "/world/target_state", rclcpp::QoS(10),
      [this](const TargetState::SharedPtr message) {
        const std::lock_guard<std::mutex> guard(target_mutex_);
        target_state_ = message;
      },
      subscription_options);

    odometry_subscription_ = create_subscription<Odometry>(
      prefix + "/state/odometry_fused", rclcpp::QoS(10),
      [this](const Odometry::SharedPtr message) {onOdometry(*message);},
      subscription_options);

    offboard_subscription_ = create_subscription<OffboardStatus>(
      prefix + "/backend/offboard_status", rclcpp::QoS(10),
      [this](const OffboardStatus::SharedPtr message) {offboard_state_ = message->state;},
      subscription_options);

    localization_subscription_ = create_subscription<LocalizationStatus>(
      prefix + "/state/localization_status", rclcpp::QoS(10),
      [this](const LocalizationStatus::SharedPtr message) {
        position_uncertainty_ = message->position_uncertainty;
        localization_valid_ = message->is_valid;
        localization_seen_ = true;
      },
      subscription_options);

    vehicle_subscription_ = create_subscription<VehicleState>(
      prefix + "/state/vehicle", rclcpp::QoS(10),
      [this](const VehicleState::SharedPtr message) {
        connected_ = message->connected;
        armed_ = message->armed;
        flight_mode_ = message->flight_mode;
        failsafe_active_ = message->failsafe_active;
      },
      subscription_options);

    arm_client_ = create_client<Arm>(
      prefix + "/backend/arm", rmw_qos_profile_services_default, action_group_);
    disarm_client_ = create_client<Disarm>(
      prefix + "/backend/disarm", rmw_qos_profile_services_default, action_group_);
    mode_client_ = create_client<SetFlightMode>(
      prefix + "/backend/set_mode", rmw_qos_profile_services_default, action_group_);

    takeoff_server_ = rclcpp_action::create_server<Takeoff>(
      this, prefix + "/planning/takeoff",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Takeoff::Goal> goal) {
        return admit(
          TaskType::TAKEOFF, NavigatorState::TAKING_OFF, true,
          [&](NavigatorState state) {
            return admitTakeoff(state, TakeoffRequest{goal->target_altitude}, limits_);
          });
      },
      [this](std::shared_ptr<GoalHandleTakeoff>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<GoalHandleTakeoff> handle) {
        startWorker([this, handle]() {executeTakeoff(handle);});
      },
      rcl_action_server_get_default_options(), action_group_);

    goto_server_ = rclcpp_action::create_server<GotoPose>(
      this, prefix + "/planning/goto_pose",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const GotoPose::Goal> goal) {
        return admit(
          TaskType::GOTO_POSE, NavigatorState::GOING_TO_POSE, true,
          [&](NavigatorState state) {
            return admitGoto(state, toGotoRequest(*goal), limits_, odom_frame_);
          });
      },
      [this](std::shared_ptr<GoalHandleGoto>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<GoalHandleGoto> handle) {
        startWorker([this, handle]() {executeGoto(handle);});
      },
      rcl_action_server_get_default_options(), action_group_);

    hold_server_ = rclcpp_action::create_server<HoldPosition>(
      this, prefix + "/planning/hold_position",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const HoldPosition::Goal> goal) {
        return admit(
          TaskType::HOLD_POSITION, NavigatorState::HOLD_TASK, true,
          [&](NavigatorState state) {
            return admitHold(state, HoldRequest{goal->duration_seconds});
          });
      },
      [this](std::shared_ptr<GoalHandleHold>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<GoalHandleHold> handle) {
        startWorker([this, handle]() {executeHold(handle);});
      },
      rcl_action_server_get_default_options(), action_group_);

    follow_server_ = rclcpp_action::create_server<FollowPath>(
      this, prefix + "/planning/follow_path",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowPath::Goal> goal) {
        return admit(
          TaskType::FOLLOW_PATH, NavigatorState::FOLLOWING_PATH, true,
          [&](NavigatorState state) {
            return admitFollowPath(state, toFollowRequest(*goal), limits_, odom_frame_);
          });
      },
      [this](std::shared_ptr<GoalHandleFollow>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<GoalHandleFollow> handle) {
        startWorker([this, handle]() {executeFollowPath(handle);});
      },
      rcl_action_server_get_default_options(), action_group_);

    track_server_ = rclcpp_action::create_server<TrackTarget>(
      this, prefix + "/planning/track_target",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const TrackTarget::Goal> goal) {
        return admit(
          TaskType::TRACK_TARGET, NavigatorState::TRACKING_TARGET, true,
          [&](NavigatorState state) {
            return admitTrackTarget(state, toTrackRequest(*goal), limits_);
          });
      },
      [this](std::shared_ptr<GoalHandleTrack>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<GoalHandleTrack> handle) {
        startWorker([this, handle]() {executeTrackTarget(handle);});
      },
      rcl_action_server_get_default_options(), action_group_);

    recover_server_ = rclcpp_action::create_server<Recover>(
      this, prefix + "/planning/recover",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Recover::Goal> goal) {
        const RecoverRequest request = toRecoverRequest(*goal);
        // Hover needs no idea where the aircraft is, so a disowned pose must not be
        // what stops the one recovery that works without one.
        const rclcpp_action::GoalResponse response = admit(
          TaskType::RECOVER, NavigatorState::RECOVERING,
          goal->recovery_type != Recover::Goal::TYPE_HOLD,
          [&](NavigatorState state) {return admitRecover(state, request);});
        if (response == rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE) {
          requestPreemption();
        }
        return response;
      },
      [this](std::shared_ptr<GoalHandleRecover>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<GoalHandleRecover> handle) {
        startWorker([this, handle]() {executeRecover(handle);});
      },
      rcl_action_server_get_default_options(), action_group_);

    land_server_ = rclcpp_action::create_server<Land>(
      this, prefix + "/planning/land",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Land::Goal> goal) {
        return admit(
          TaskType::LAND, NavigatorState::LANDING, false,
          [&](NavigatorState state) {
            return admitLand(state, LandRequest{goal->use_precision_landing});
          });
      },
      // Refusing the handover back is the safe half: the autopilot already has it.
      [this](std::shared_ptr<GoalHandleLand>) {return rclcpp_action::CancelResponse::REJECT;},
      [this](std::shared_ptr<GoalHandleLand> handle) {
        startWorker([this, handle]() {executeLand(handle);});
      },
      rcl_action_server_get_default_options(), action_group_);

    stream_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / stream_hz_),
      [this]() {streamTick();}, stream_group_);

    publishNoTrajectory(Trajectory3D::PLAN_STATE_NO_GOAL, "no goal yet");

    RCLCPP_INFO(
      get_logger(), "navigator ready for %s: stream %.1f Hz, max_speed %.2f m/s, goto path %s",
      uav_id_.c_str(), stream_hz_, limits_.max_speed,
      use_trajectory_ ? "trajectory" : "carrot");
  }

  ~NavigatorActionServerNode() override
  {
    stopping_ = true;
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<Takeoff>;
  using GoalHandleGoto = rclcpp_action::ServerGoalHandle<GotoPose>;
  using GoalHandleHold = rclcpp_action::ServerGoalHandle<HoldPosition>;
  using GoalHandleLand = rclcpp_action::ServerGoalHandle<Land>;
  using GoalHandleFollow = rclcpp_action::ServerGoalHandle<FollowPath>;
  using GoalHandleTrack = rclcpp_action::ServerGoalHandle<TrackTarget>;
  using GoalHandleRecover = rclcpp_action::ServerGoalHandle<Recover>;

  // ---------------------------------------------------------------- streaming

  void onOdometry(const Odometry & message)
  {
    const auto & position = message.pose.pose.position;
    const auto & orientation = message.pose.pose.orientation;
    std::lock_guard<std::mutex> lock(motion_mutex_);
    position_ = Vec3{position.x, position.y, position.z};
    measured_yaw_ =
      yawFromQuaternion(orientation.x, orientation.y, orientation.z, orientation.w);
    odometry_stamp_ = message.header.stamp;
    odometry_valid_ = isFinite(position_);
  }

  // Never blocks: nothing under motion_mutex_ waits on anything.
  void streamTick()
  {
    ControlCommand command;
    CarrotStep step;
    rclcpp::Time tick_stamp{0, 0, RCL_ROS_TIME};
    double lead_horizontal = 0.0;
    double lead_vertical = 0.0;
    double position_z = 0.0;
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      const rclcpp::Time stamp = now();
      tick_stamp = stamp;
      if (!stream_active_) {
        last_stream_time_ = stamp;
        return;
      }

      const double nominal = 1.0 / stream_hz_;
      double dt = (stamp - last_stream_time_).seconds();
      if (!(dt > 0.0) || !std::isfinite(dt)) {
        dt = nominal;
      }
      dt = std::min(dt, 3.0 * nominal);
      last_stream_time_ = stamp;

      // Avoidance hold: the setpoint stands still and the plan clock stops with it,
      // so the plan resumes where it left off instead of being rebuilt from rest.
      if (motion_paused_) {
        step.setpoint = carrot_;
      } else {
        if (trajectory_active_) {
          // Leash clamped: the aircraft is behind, so the plan waits.
          if (!previous_tick_clamped_) {
            trajectory_time_ += dt;
          }
          const TrajectorySample planned = trajectory_.sample(trajectory_time_);
          target_ = planned.position;
          target_yaw_ = planned.yaw;
          if (requested_goal_yaw_) {
            // Leave the track late enough to fly it, early enough to land the turn
            // before arrival is declared, which happens before the plan runs out.
            if (!yaw_handover_engaged_) {
              const double slew =
                std::abs(wrapAngle(*requested_goal_yaw_ - commanded_yaw_)) /
                std::max(1e-3, max_yaw_rate_);
              // Latched: a clamped leash freezes remaining while slew keeps shrinking,
              // and an unlatched test would then flip back and dither the nose.
              yaw_handover_engaged_ =
                trajectory_.duration() - trajectory_time_ <= slew + yaw_handover_margin_;
            }
            if (yaw_handover_engaged_) {
              target_yaw_ = *requested_goal_yaw_;
            }
          }
        }
        step = advanceCarrot(carrot_, target_, position_, active_limits_, dt);
      }
      previous_tick_clamped_ = step.clamped();
      carrot_ = step.setpoint;
      commanded_yaw_ = stepYaw(
        commanded_yaw_, motion_paused_ ? commanded_yaw_ : target_yaw_, max_yaw_rate_, dt);

      lead_horizontal = horizontalDistance(position_, carrot_);
      lead_vertical = std::abs(carrot_.z - position_.z);
      position_z = position_.z;
      recordLeash(step, stamp, lead_horizontal, lead_vertical);

      command.header.stamp = stamp;
      command.header.frame_id = odom_frame_;
      command.uav_id = uav_id_;
      command.control_mode = ControlCommand::MODE_POSITION;
      command.position.x = carrot_.x;
      command.position.y = carrot_.y;
      command.position.z = carrot_.z;
      command.yaw = static_cast<float>(commanded_yaw_);
      command.source = ControlAuthority::SOURCE_MISSION;
    }
    command_publisher_->publish(command);

    if (step.clamped()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "leash clamping the setpoint, stall clock %.1f s (h %d v %d): lead h %.2f v %.2f m, "
        "commanded_z %.2f, position_z %.2f",
        leash_stall_seconds_.load(), static_cast<int>(step.clamped_horizontal),
        static_cast<int>(step.clamped_vertical),
        lead_horizontal, lead_vertical, command.position.z, position_z);
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "streaming: commanded_z %.2f, position_z %.2f, lead h %.2f v %.2f m, clamped %.0f%%",
        command.position.z, position_z, lead_horizontal, lead_vertical, clampedShare());
    }
  }

  /// Stall time is continuous clamping; a single clamped tick during pursuit is normal.
  /// Caller holds motion_mutex_.
  void recordLeash(
    const CarrotStep & step, const rclcpp::Time & stamp, double lead_horizontal,
    double lead_vertical)
  {
    stream_ticks_.fetch_add(1);
    leash_lead_horizontal_ = lead_horizontal;
    leash_lead_vertical_ = lead_vertical;
    if (step.clamped()) {
      clamped_ticks_.fetch_add(1);
    }
    // Spooling up on the ground clamps the setpoint with nothing wrong: the aircraft
    // is not refusing to follow, it is not allowed to move yet.
    if (!step.clamped() || stamp.seconds() < stall_watch_from_.load()) {
      leash_clamp_started_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      leash_stall_seconds_ = 0.0;
      return;
    }
    if (leash_clamp_started_.nanoseconds() == 0) {
      leash_clamp_started_ = stamp;
    }
    leash_stall_seconds_ = (stamp - leash_clamp_started_).seconds();
  }

  /// Every task starts with a clean stall clock; a new goal never inherits the old one.
  void beginTask()
  {
    planner_fault_.clear();
    planner_fault_code_ = ResultCode::ABORTED_PLANNER_FAILED;
    holding_without_localization_ = false;
    beginCarrotPlan();
    std::lock_guard<std::mutex> lock(motion_mutex_);
    leash_clamp_started_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    leash_stall_seconds_ = 0.0;
    stream_ticks_ = 0;
    clamped_ticks_ = 0;
    stall_watch_from_ = now().seconds() + takeoff_stall_grace_;
  }

  double clampedShare() const
  {
    const uint64_t ticks = stream_ticks_.load();
    if (ticks == 0) {
      return 0.0;
    }
    return 100.0 * static_cast<double>(clamped_ticks_.load()) / static_cast<double>(ticks);
  }

  std::string clampEvidence() const
  {
    if (clamped_ticks_.load() == 0) {
      return std::string();
    }
    char buffer[160];
    std::snprintf(
      buffer, sizeof(buffer), "; setpoint clamped %.0f%% of the task, lead h %.2f m v %.2f m",
      clampedShare(), leash_lead_horizontal_.load(), leash_lead_vertical_.load());
    return std::string(buffer);
  }

  CarrotLimits defaultCarrotLimits() const
  {
    return CarrotLimits{
      limits_.max_speed, limits_.max_vertical_speed, max_lead_horizontal_, max_lead_vertical_};
  }

  void beginStream(const Vec3 & anchor, double yaw)
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    clearTrajectory();
    carrot_ = anchor;
    target_ = anchor;
    commanded_yaw_ = yaw;
    target_yaw_ = yaw;
    active_limits_ = defaultCarrotLimits();
    last_stream_time_ = now();
    stream_active_ = true;
  }

  void stopStream()
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    stream_active_ = false;
  }

  bool streaming()
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    return stream_active_;
  }

  void setTarget(const Vec3 & target, double yaw, double max_speed)
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    clearTrajectory();
    target_ = target;
    target_yaw_ = yaw;
    active_limits_ = CarrotLimits{
      max_speed, std::min(limits_.max_vertical_speed, max_speed),
      max_lead_horizontal_, max_lead_vertical_};
  }

  /// Caller holds motion_mutex_. Every other task flies the carrot.
  void clearTrajectory()
  {
    motion_paused_ = false;
    trajectory_active_ = false;
    trajectory_time_ = 0.0;
    previous_tick_clamped_ = false;
    yaw_handover_margin_ = 0.0;
    yaw_handover_engaged_ = false;
    requested_goal_yaw_.reset();
  }

  /// Moved in: the sample grid is never allocated on the streaming path.
  void setTrajectory(
    Trajectory && plan, double max_speed, std::optional<double> goal_yaw, double yaw_margin,
    const Vec3 & planned_from)
  {
    Trajectory retired;
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      // Today the setpoint is frozen between tasks; a preempting caller would break that.
      const double drift = distance3(carrot_, planned_from);
      if (drift > 0.05) {
        RCLCPP_WARN(get_logger(), "setpoint moved %.3f m while the plan was built", drift);
      }
      clearTrajectory();
      retired = std::move(trajectory_);
      trajectory_ = std::move(plan);
      trajectory_active_ = true;
      requested_goal_yaw_ = goal_yaw;
      yaw_handover_margin_ = yaw_margin;
      const TrajectorySample start = trajectory_.sample(0.0);
      target_ = start.position;
      target_yaw_ = start.yaw;
      active_limits_ = CarrotLimits{
        max_speed, std::min(limits_.max_vertical_speed, max_speed),
        max_lead_horizontal_, max_lead_vertical_};
    }
    // The old grid dies here, with the stream timer free to run.
  }

  /// Suspends the plan without discarding it: a rebuilt plan would start from rest
  /// and step the commanded velocity, and the advisor still needs a plan to check.
  void pauseMotion()
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    motion_paused_ = true;
  }

  void resumeMotion()
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    motion_paused_ = false;
  }

  /// Stops the setpoint where it stands; the aircraft closes the gap without a step.
  Vec3 freezeSetpoint()
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    clearTrajectory();
    target_ = carrot_;
    target_yaw_ = commanded_yaw_;
    return carrot_;
  }

  Vec3 currentCarrot()
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    return carrot_;
  }

  /// One lock: a plan built from a tick-old origin and a fresh yaw is a step.
  std::pair<Vec3, double> motionAnchor()
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    return {carrot_, commanded_yaw_};
  }

  double commandedYaw()
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    return commanded_yaw_;
  }

  CommandedMotion commandedMotion()
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    CommandedMotion motion;
    motion.setpoint = carrot_;
    motion.yaw = commanded_yaw_;
    motion.limits = active_limits_;
    motion.on_the_carrot = stream_active_ && !trajectory_active_;
    return motion;
  }

  /// Negative when no plan is running. Stops counting down while the leash holds,
  /// which is what makes the frozen plan clock visible to whoever is watching.
  /// Ceiling on runOutPlanTail(). A plan longer than this after arrival means the plan
  /// disagrees with the aircraft, and waiting is then the wrong answer.
  static constexpr double kMaxPlanTailSec = 3.0;

  double trajectoryRemaining()
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    if (!trajectory_active_) {
      return -1.0;
    }
    return std::max(0.0, trajectory_.duration() - trajectory_time_);
  }

  PoseSample poseSample()
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    PoseSample sample;
    sample.position = position_;
    sample.yaw = measured_yaw_;
    sample.stamp = odometry_stamp_;
    sample.valid = odometry_valid_;
    return sample;
  }

  // ------------------------------------------------------------ published plan

  Trajectory3D trajectoryHeader() const
  {
    Trajectory3D message;
    message.header.stamp = now();
    message.header.frame_id = odom_frame_;
    message.uav_id = uav_id_;
    message.start_time = message.header.stamp;
    message.is_valid = false;
    message.sequence = plan_sequence_;
    return message;
  }

  Trajectory3D toMessage(const Trajectory & plan)
  {
    Trajectory3D message = trajectoryHeader();
    message.is_valid = true;
    message.plan_state = Trajectory3D::PLAN_STATE_VALID;
    message.sequence = ++plan_sequence_;

    const std::vector<TrajectorySample> & grid = plan.grid();
    message.points.reserve(grid.size());
    for (std::size_t index = 0; index < grid.size(); ++index) {
      const TrajectorySample & sample = grid[index];
      TrajectoryPoint point;
      point.time_from_start =
        rclcpp::Duration::from_seconds(static_cast<double>(index) * plan.gridPeriod());
      point.position.x = sample.position.x;
      point.position.y = sample.position.y;
      point.position.z = sample.position.z;
      point.velocity.x = sample.velocity.x;
      point.velocity.y = sample.velocity.y;
      point.velocity.z = sample.velocity.z;
      point.acceleration.x = sample.acceleration.x;
      point.acceleration.y = sample.acceleration.y;
      point.acceleration.z = sample.acceleration.z;
      point.yaw = static_cast<float>(sample.yaw);
      point.yaw_rate = static_cast<float>(sample.yaw_rate);
      message.points.push_back(point);
    }

    if (grid.size() > kLoudTrajectoryPoints) {
      RCLCPP_WARN(
        get_logger(), "publishing a %zu point trajectory, %.1f s long",
        grid.size(), plan.duration());
    }
    return message;
  }

  /// The two point leg a carrot flight is actually flying; see README section 6h.
  Trajectory3D toMessage(
    const Vec3 & from, const Vec3 & to, const CarrotLimits & limits, double yaw)
  {
    Trajectory3D message = trajectoryHeader();
    message.is_valid = true;
    message.plan_state = Trajectory3D::PLAN_STATE_VALID;
    message.sequence = ++plan_sequence_;

    const Vec3 span{to.x - from.x, to.y - from.y, to.z - from.z};
    const double horizontal = std::hypot(span.x, span.y);
    const double vertical = std::abs(span.z);
    const double seconds = std::max(
      horizontal / std::max(1e-3, limits.max_speed),
      vertical / std::max(1e-3, limits.max_vertical_speed));

    Vec3 velocity;
    if (seconds > 1e-6) {
      velocity = Vec3{span.x / seconds, span.y / seconds, span.z / seconds};
    }

    const Vec3 corners[2] = {from, to};
    for (std::size_t index = 0; index < 2; ++index) {
      TrajectoryPoint point;
      point.time_from_start =
        rclcpp::Duration::from_seconds(index == 0 ? 0.0 : seconds);
      point.position.x = corners[index].x;
      point.position.y = corners[index].y;
      point.position.z = corners[index].z;
      if (index == 0) {
        point.velocity.x = velocity.x;
        point.velocity.y = velocity.y;
        point.velocity.z = velocity.z;
      }
      point.yaw = static_cast<float>(yaw);
      message.points.push_back(point);
    }
    return message;
  }

  /// Without this the latched topic still offers the previous goal's plan.
  /// The sequence is not advanced: this announces the fate of the current era.
  void publishNoTrajectory(uint8_t plan_state, const std::string & reason)
  {
    Trajectory3D message = trajectoryHeader();
    message.plan_state = plan_state;
    message.reason = reason;
    trajectory_publisher_->publish(message);
  }

  // ------------------------------------------------------------- goal intake

  /// What "arrived" may mean given how well the system says it knows where it is.
  struct Acceptance
  {
    bool usable = true;
    double radius = 0.3;
    double sigma = -1.0;
    std::string detail;
  };

  Acceptance resolveAcceptance(double requested_radius) const
  {
    Acceptance acceptance;
    acceptance.radius = requested_radius;
    if (!localization_seen_.load()) {
      return acceptance;
    }
    acceptance.sigma = position_uncertainty_.load();
    if (!localization_valid_.load()) {
      acceptance.usable = false;
      acceptance.detail = "localization_status reports is_valid=false";
      return acceptance;
    }
    if (!std::isfinite(acceptance.sigma) || acceptance.sigma < 0.0) {
      return acceptance;
    }

    char buffer[160];
    if (acceptance.sigma > max_acceptance_radius_) {
      acceptance.usable = false;
      std::snprintf(
        buffer, sizeof(buffer),
        "position uncertainty %.2f m is worse than max_acceptance_radius %.2f m",
        acceptance.sigma, max_acceptance_radius_);
      acceptance.detail = buffer;
      return acceptance;
    }
    // Arriving inside the noise floor of our own pose is not something we can judge.
    if (acceptance.sigma >= requested_radius) {
      acceptance.radius = std::min(
        max_acceptance_radius_,
        std::max(requested_radius, acceptance_sigma_factor_ * acceptance.sigma));
      std::snprintf(
        buffer, sizeof(buffer),
        "acceptance widened %.2f -> %.2f m for position uncertainty %.2f m",
        requested_radius, acceptance.radius, acceptance.sigma);
      acceptance.detail = buffer;
    }
    return acceptance;
  }

  template<typename AdmissionFn>
  rclcpp_action::GoalResponse admit(
    TaskType task, NavigatorState running_state, bool needs_localization,
    AdmissionFn && evaluate)
  {
    std::lock_guard<std::mutex> lock(admission_mutex_);
    // The gap between accepting a recovery and the running task standing down is
    // the one window where the state alone would let a third goal in.
    if (recovery_pending_.load() && task != TaskType::RECOVER) {
      RCLCPP_WARN(
        get_logger(), "rejected %s goal: a recovery is taking the aircraft over",
        toString(task));
      return rclcpp_action::GoalResponse::REJECT;
    }
    const NavigatorState state = state_.load();
    const Admission verdict = evaluate(state);
    if (!verdict.accepted) {
      RCLCPP_WARN(
        get_logger(), "rejected %s goal in %s: %s (%s)",
        toString(task), toString(state), verdict.detail.c_str(), toString(verdict.reason));
      return rclcpp_action::GoalResponse::REJECT;
    }
    // Landing is the way down whatever localization says, so it is never gated.
    if (needs_localization) {
      const Acceptance acceptance = resolveAcceptance(limits_.acceptance_radius);
      if (!acceptance.usable) {
        RCLCPP_WARN(
          get_logger(), "rejected %s goal: %s", toString(task), acceptance.detail.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
    state_ = running_state;
    // Raised under the same lock that set the state: the running task will drop back
    // to HOLDING in a moment, and a goal arriving in that instant must not get in.
    if (task == TaskType::RECOVER) {
      recovery_pending_ = true;
    }
    RCLCPP_INFO(get_logger(), "accepted %s goal", toString(task));
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  static GotoRequest toGotoRequest(const GotoPose::Goal & goal)
  {
    GotoRequest request;
    request.x = goal.target_pose.position.x;
    request.y = goal.target_pose.position.y;
    request.z = goal.target_pose.position.z;
    request.frame_id = goal.frame_id;
    request.acceptance_radius = goal.acceptance_radius;
    request.max_speed = goal.max_speed;
    return request;
  }

  FollowPathRequest toFollowRequest(const FollowPath::Goal & goal) const
  {
    FollowPathRequest request;
    request.frame_id = goal.path.header.frame_id;
    // A hand-built Path3D leaves plan_state at UNKNOWN, so honour is_valid there.
    request.path_is_valid = goal.path.plan_state == Path3D::PLAN_STATE_VALID ||
      (goal.path.plan_state == Path3D::PLAN_STATE_UNKNOWN && goal.path.is_valid);
    request.max_speed = goal.max_speed;
    request.acceptance_radius = goal.waypoint_acceptance_radius;
    request.waypoints.reserve(goal.path.waypoints.size());
    for (const geometry_msgs::msg::Pose & pose : goal.path.waypoints) {
      request.waypoints.push_back(Vec3{pose.position.x, pose.position.y, pose.position.z});
    }
    return request;
  }

  TrackRequest toTrackRequest(const TrackTarget::Goal & goal) const
  {
    TrackRequest request;
    request.standoff_distance = goal.standoff_distance;
    request.duration_seconds = goal.duration_seconds;
    request.target_lost_timeout = goal.target_lost_timeout;
    return request;
  }

  /// Switched on explicitly so LAND and HANDOVER are refused where they are read,
  /// never cast into a type this planner would happily fly.
  static std::optional<RecoveryType> executableRecoveryType(
    uint8_t requested, std::string & refusal)
  {
    switch (requested) {
      case Recover::Goal::TYPE_HOLD:
        return RecoveryType::Hover;
      case Recover::Goal::TYPE_CLIMB_TO_SAFE_ALTITUDE:
        return RecoveryType::ClimbToSafe;
      case Recover::Goal::TYPE_RETURN_HOME:
        return RecoveryType::ReturnHome;
      case Recover::Goal::TYPE_LAND:
        refusal =
          "recovery landing is not this node's call: it executes motion, and where to "
          "put the aircraft down belongs to the safety layer (contract 2.5)";
        break;
      case Recover::Goal::TYPE_HANDOVER_TO_PILOT:
        refusal =
          "handing the aircraft back to the pilot is a control authority act, not a "
          "trajectory this node can fly";
        break;
      default:
        refusal =
          "recovery_type was never chosen; the goal states hold, climb_to_safe_altitude "
          "or return_home";
        break;
    }
    return std::nullopt;
  }

  static RecoverRequest toRecoverRequest(const Recover::Goal & goal)
  {
    RecoverRequest request;
    std::string refusal;
    request.type_executable = executableRecoveryType(goal.recovery_type, refusal).has_value();
    request.refusal = refusal;
    request.reads_altitude = goal.recovery_type != Recover::Goal::TYPE_HOLD;
    request.safe_altitude = goal.safe_altitude;
    return request;
  }

  /// Set the moment a recovery is accepted, so the running task stands down before
  /// the recovery worker even starts waiting for its thread.
  void requestPreemption()
  {
    preempt_requested_ = true;
  }

  /// The new worker joins the old one, so no executor thread ever waits on a task.
  void startWorker(std::function<void()> body)
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    std::thread previous = std::move(worker_);
    worker_ = std::thread(
      [previous = std::move(previous), body = std::move(body)]() mutable {
        if (previous.joinable()) {
          previous.join();
        }
        body();
      });
  }

  void setState(NavigatorState state)
  {
    state_ = state;
  }

  // --------------------------------------------------------------- execution

  /// Says out loud, once per task, what "arrived" will mean this time.
  double applyAcceptance(double requested_radius, const char * task)
  {
    const Acceptance acceptance = resolveAcceptance(requested_radius);
    acceptance_note_ = acceptance.detail.empty() ? std::string() : "; " + acceptance.detail;
    if (!acceptance.detail.empty()) {
      RCLCPP_WARN(get_logger(), "%s: %s", task, acceptance.detail.c_str());
    }
    return acceptance.radius;
  }

  bool sleepOneTick()
  {
    return rclcpp::ok() && !stopping_ &&
           get_clock()->sleep_for(rclcpp::Duration::from_seconds(1.0 / progress_hz_));
  }

  /// True only while the autopilot is actually taking our setpoints.
  bool inCommand() const
  {
    return offboard_state_.load() == OffboardStatus::STATE_ACTIVE &&
           flight_mode_.load() == VehicleState::FLIGHT_MODE_OFFBOARD;
  }

  Fault checkFaults()
  {
    Fault fault;
    if (!planner_fault_.empty()) {
      fault.present = true;
      fault.result_code = planner_fault_code_;
      fault.detail = planner_fault_;
      return fault;
    }
    const rclcpp::Time stamp = now();
    // A hover recovery reads no pose, so a dead pose may not end it: that is the one
    // task the safety layer still has left when localization goes (contract 2.5).
    if (!holding_without_localization_) {
      const PoseSample pose = poseSample();
      if (!pose.valid || (stamp - pose.stamp).seconds() > odometry_timeout_) {
        fault.present = true;
        fault.result_code = ResultCode::ABORTED_LOST_LOCALIZATION;
        fault.detail = "odometry_fused went stale";
        return fault;
      }
      // A pose the system itself no longer vouches for is not one to fly on.
      const Acceptance acceptance = resolveAcceptance(limits_.acceptance_radius);
      if (!acceptance.usable) {
        fault.present = true;
        fault.result_code = ResultCode::ABORTED_LOST_LOCALIZATION;
        fault.detail = acceptance.detail;
        return fault;
      }
    }
    if (failsafe_active_.load()) {
      fault.present = true;
      fault.hand_over = true;
      fault.result_code = ResultCode::ABORTED_SAFETY;
      fault.detail = "autopilot failsafe active";
      return fault;
    }
    if (autopilotOwnsTheFlight(flight_mode_.load())) {
      fault.present = true;
      fault.hand_over = true;
      fault.result_code = ResultCode::ABORTED_SAFETY;
      fault.detail = "autopilot took over the flight";
      return fault;
    }

    // Anything short of confirmed offboard means our setpoints are not flying the
    // aircraft. Stop commanding; never try to take it back.
    if (inCommand()) {
      not_commanding_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    } else {
      if (not_commanding_since_.nanoseconds() == 0) {
        not_commanding_since_ = stamp;
      }
      if ((stamp - not_commanding_since_).seconds() >= command_loss_debounce_) {
        const uint8_t offboard = offboard_state_.load();
        fault.present = true;
        fault.hand_over = true;
        fault.result_code = offboard == OffboardStatus::STATE_FAULT
          ? ResultCode::ABORTED_VEHICLE_REJECTED
          : ResultCode::ABORTED_SAFETY;
        char buffer[128];
        std::snprintf(
          buffer, sizeof(buffer),
          "no longer commanding: offboard state %u, flight mode %u", offboard,
          flight_mode_.load());
        fault.detail = buffer;
        return fault;
      }
    }

    // The aircraft cannot follow, so the setpoint has stopped moving. Say so and
    // stop, instead of hanging silently until the task times out.
    if (leash_stall_fault_ > 0.0 && leash_stall_seconds_.load() > leash_stall_fault_) {
      fault.present = true;
      fault.result_code = ResultCode::ABORTED_PLANNER_FAILED;
      char buffer[160];
      std::snprintf(
        buffer, sizeof(buffer),
        "setpoint clamped by leash for %.1f s, lead h %.2f m v %.2f m",
        leash_stall_seconds_.load(), leash_lead_horizontal_.load(),
        leash_lead_vertical_.load());
      fault.detail = buffer;
      return fault;
    }
    return fault;
  }

  template<typename GoalHandleT>
  LoopOutcome runProgressLoop(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const std::function<bool()> & reached,
    const std::function<void()> & publish_feedback,
    double timeout_sec, double settle_sec, Fault & fault,
    const std::function<void()> & watch = {})
  {
    const rclcpp::Time start = now();
    rclcpp::Time last_feedback = start;
    rclcpp::Time stable_since = start;
    bool stable = false;
    const double feedback_period = 1.0 / feedback_hz_;

    not_commanding_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    while (rclcpp::ok() && !stopping_) {
      if (goal_handle->is_canceling()) {
        return LoopOutcome::CANCELED;
      }
      // Stand down at once: the recovery worker is waiting on this thread to end,
      // and every tick spent here is a tick the aircraft keeps the old goal.
      if (preempt_requested_.load()) {
        return LoopOutcome::PREEMPTED;
      }
      fault = checkFaults();
      if (fault.present) {
        return LoopOutcome::FAULT;
      }
      if (watch) {
        watch();
      }
      const rclcpp::Time stamp = now();
      if (reached()) {
        if (!stable) {
          stable = true;
          stable_since = stamp;
        } else if ((stamp - stable_since).seconds() >= settle_sec) {
          publish_feedback();
          return LoopOutcome::REACHED;
        }
      } else {
        stable = false;
      }
      if ((stamp - last_feedback).seconds() >= feedback_period) {
        publish_feedback();
        last_feedback = stamp;
      }
      if ((stamp - start).seconds() > timeout_sec) {
        return LoopOutcome::TIMEOUT;
      }
      if (!sleepOneTick()) {
        fault.present = true;
        fault.result_code = ResultCode::ABORTED_INTERNAL_ERROR;
        fault.detail = "clock stopped or node is shutting down";
        return LoopOutcome::FAULT;
      }
    }
    fault.present = true;
    fault.result_code = ResultCode::ABORTED_INTERNAL_ERROR;
    fault.detail = "node is shutting down";
    return LoopOutcome::FAULT;
  }

  /// Arrival is judged on the AIRCRAFT -- it enters the acceptance ball while the plan
  /// still has tail left to decelerate through. Parking the setpoint at that instant put
  /// a step on the wire: measured 2026-08-26 on goto_pose, 0.331 m/s to 0 across one
  /// 20 ms tick, 16.5 m/s2, against a 0.43 m/s2 plan. It happened on every arrival; the
  /// trace only sometimes ran far enough to record it. Only REACHED waits: a cancel, a
  /// preempt or a fault all want the setpoint standing still now, not in 0.7 s.
  void runOutPlanTail()
  {
    const double remaining = trajectoryRemaining();
    if (!(remaining > 0.0)) {
      return;
    }
    const double budget = std::min(remaining + 2.0 / stream_hz_, kMaxPlanTailSec);
    const rclcpp::Time deadline = now() + rclcpp::Duration::from_seconds(budget);
    // Bounded in BOTH clocks: the deadline is ROS time, which stands still if /clock
    // stops, and the sleep is wall time, which does not. Either one alone can hang.
    const int max_ticks = static_cast<int>(budget * stream_hz_) + 4;
    for (int tick = 0; tick < max_ticks; ++tick) {
      if (!rclcpp::ok() || stopping_ || preempt_requested_.load() || now() >= deadline) {
        break;
      }
      if (!(trajectoryRemaining() > 0.0) || checkFaults().present) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / stream_hz_));
    }
    RCLCPP_WARN(
      get_logger(), "plan tail did not run out within %.2f s; parking the setpoint anyway",
      budget);
  }

  template<typename GoalHandleT, typename ResultT>
  void finishAirborne(
    const std::shared_ptr<GoalHandleT> & goal_handle, LoopOutcome outcome, const Fault & fault,
    const std::shared_ptr<ResultT> & result, const char * task)
  {
    if (outcome == LoopOutcome::REACHED) {
      runOutPlanTail();
    }
    freezeSetpoint();
    publishNoTrajectory(Trajectory3D::PLAN_STATE_NO_GOAL, std::string(task) + " ended");
    const std::string evidence = acceptance_note_ + evidence_note_ + clampEvidence();
    switch (outcome) {
      case LoopOutcome::REACHED:
        result->success = true;
        result->result_code = ResultCode::SUCCEEDED;
        result->message = std::string(task) + " complete" + evidence;
        setState(NavigatorState::HOLDING);
        goal_handle->succeed(result);
        return;
      case LoopOutcome::CANCELED:
        result->success = false;
        result->result_code = ResultCode::CANCELED;
        result->message = std::string(task) + " canceled, holding position" + evidence;
        setState(NavigatorState::HOLDING);
        goal_handle->canceled(result);
        return;
      case LoopOutcome::PREEMPTED:
        result->success = false;
        // Not CANCELED: nobody asked this caller to stop, a recovery took over.
        result->result_code = ResultCode::ABORTED_SAFETY;
        result->message = std::string(task) + " preempted by a recovery goal" + evidence;
        setState(NavigatorState::HOLDING);
        RCLCPP_WARN(get_logger(), "%s preempted by a recovery goal", task);
        goal_handle->abort(result);
        return;
      case LoopOutcome::TIMEOUT:
        result->success = false;
        result->result_code = ResultCode::ABORTED_TIMEOUT;
        result->message = std::string(task) + " timed out, holding position" + evidence;
        setState(NavigatorState::HOLDING);
        goal_handle->abort(result);
        return;
      case LoopOutcome::FAULT:
      default:
        result->success = false;
        result->result_code = fault.result_code;
        result->message = fault.detail + evidence;
        if (fault.hand_over) {
          stopStream();
          setState(NavigatorState::IDLE);
        } else {
          setState(NavigatorState::HOLDING);
        }
        RCLCPP_ERROR(get_logger(), "%s aborted: %s", task, fault.detail.c_str());
        goal_handle->abort(result);
        return;
    }
  }

  bool waitForVehicleAndOdometry(
    PoseSample & pose, std::string & detail, const std::function<bool()> & canceled)
  {
    const rclcpp::Time deadline = now() + rclcpp::Duration::from_seconds(startup_timeout_);
    while (rclcpp::ok() && !stopping_ && now() < deadline) {
      if (canceled()) {
        detail = "canceled before the aircraft was armed";
        return false;
      }
      pose = poseSample();
      const bool fresh = pose.valid && (now() - pose.stamp).seconds() <= odometry_timeout_;
      if (connected_.load() && fresh) {
        return true;
      }
      if (!sleepOneTick()) {
        break;
      }
    }
    detail = connected_.load() ? "no fresh odometry_fused" : "autopilot not connected";
    return false;
  }

  bool waitForOffboardActive(std::string & detail, const std::function<bool()> & canceled)
  {
    const rclcpp::Time deadline = now() + rclcpp::Duration::from_seconds(offboard_timeout_);
    while (rclcpp::ok() && !stopping_ && now() < deadline) {
      if (canceled()) {
        detail = "canceled before the aircraft was armed";
        return false;
      }
      if (offboard_state_.load() == OffboardStatus::STATE_ACTIVE) {
        return true;
      }
      if (!sleepOneTick()) {
        break;
      }
    }
    detail = "autopilot never confirmed offboard";
    return false;
  }

  /// Nothing is streaming and nothing is armed, so the ground is the safe end state.
  template<typename GoalHandleT, typename ResultT>
  void finishOnTheGround(
    const std::shared_ptr<GoalHandleT> & goal_handle, const std::shared_ptr<ResultT> & result,
    uint8_t result_code)
  {
    result->success = false;
    setState(NavigatorState::IDLE);
    if (goal_handle->is_canceling()) {
      result->result_code = ResultCode::CANCELED;
      goal_handle->canceled(result);
      return;
    }
    result->result_code = result_code;
    RCLCPP_ERROR(get_logger(), "takeoff aborted on the ground: %s", result->message.c_str());
    goal_handle->abort(result);
  }

  template<typename ClientT, typename RequestT>
  bool callService(
    const ClientT & client, const RequestT & request, const char * name, std::string & detail)
  {
    const auto timeout = std::chrono::duration<double>(service_timeout_);
    if (!client->wait_for_service(std::chrono::duration_cast<std::chrono::nanoseconds>(timeout))) {
      detail = std::string(name) + " service never appeared";
      return false;
    }
    auto future = client->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      detail = std::string(name) + " call timed out";
      return false;
    }
    const auto response = future.get();
    if (!response->success) {
      detail = std::string(name) + " rejected: " + response->message;
      return false;
    }
    return true;
  }

  void executeTakeoff(const std::shared_ptr<GoalHandleTakeoff> goal_handle)
  {
    auto result = std::make_shared<Takeoff::Result>();
    acceptance_note_.clear();
    evidence_note_.clear();
    const double requested = goal_handle->get_goal()->target_altitude;

    const auto canceled = [goal_handle]() {return goal_handle->is_canceling();};

    PoseSample pose;
    if (!waitForVehicleAndOdometry(pose, result->message, canceled)) {
      finishOnTheGround(goal_handle, result, ResultCode::ABORTED_LOST_LOCALIZATION);
      return;
    }

    const Vec3 origin = pose.position;
    ground_reference_z_ = origin.z;
    // The only home a recovery is ever given: in odom the origin looks like a
    // perfectly good coordinate, so it must be the place we actually left from.
    home_ = origin;
    home_known_ = true;
    beginStream(origin, pose.yaw);

    // A stale ACTIVE from the previous flight would satisfy the wait below.
    offboard_state_ = kOffboardUnknown;
    if (!waitForOffboardActive(result->message, canceled)) {
      stopStream();
      finishOnTheGround(goal_handle, result, ResultCode::ABORTED_TIMEOUT);
      return;
    }

    if (canceled()) {
      stopStream();
      result->message = "canceled before the aircraft was armed";
      finishOnTheGround(goal_handle, result, ResultCode::CANCELED);
      return;
    }

    // Offboard first, then arm; see docs/package-status.md Sec 2.
    if (!callService(arm_client_, std::make_shared<Arm::Request>(), "arm", result->message)) {
      stopStream();
      finishOnTheGround(goal_handle, result, ResultCode::ABORTED_VEHICLE_REJECTED);
      return;
    }

    const Vec3 target{origin.x, origin.y, origin.z + requested};
    setTarget(target, pose.yaw, limits_.max_speed);
    beginTask();
    const double acceptance = applyAcceptance(limits_.acceptance_radius, "takeoff");

    auto feedback = std::make_shared<Takeoff::Feedback>();
    const auto reached = [this, target, acceptance]() {
        return std::abs(poseSample().position.z - target.z) <= acceptance;
      };
    const auto publish = [this, goal_handle, feedback, origin, target]() {
        const Vec3 position = poseSample().position;
        feedback->current_altitude = static_cast<float>(position.z - origin.z);
        feedback->distance_to_target = static_cast<float>(std::abs(target.z - position.z));
        goal_handle->publish_feedback(feedback);
      };

    Fault fault;
    const LoopOutcome outcome = runProgressLoop(
      goal_handle, reached, publish, takeoff_timeout_, arrival_settle_, fault,
      [this, target]() {refreshCarrotPlan(target);});
    result->final_altitude = static_cast<float>(poseSample().position.z - origin.z);
    finishAirborne(goal_handle, outcome, fault, result, "takeoff");
  }

  /// A plan longer than the goto timeout would be abandoned in mid-air, so the
  /// timeout is a shape limit: too far to plan falls back to the carrot instead.
  TrajectoryLimits trajectoryLimits() const
  {
    TrajectoryLimits limits;
    limits.max_speed = limits_.max_speed;
    limits.max_vertical_speed = limits_.max_vertical_speed;
    limits.max_acceleration = max_acceleration_;
    limits.max_yaw_rate = max_yaw_rate_;
    limits.sample_hz = sample_hz_;
    limits.max_duration = std::min(limits.max_duration, 0.8 * goto_timeout_);
    return limits;
  }

  /// A mistyped shape parameter would degrade every goal to the carrot, which has
  /// no acceleration ceiling at all. Refuse to start instead.
  void rejectUnusableTrajectoryLimits() const
  {
    TrajectoryLimits shape = trajectoryLimits();
    // The timeout budget belongs to a goal, not to the shape being validated.
    shape.max_duration = TrajectoryLimits{}.max_duration;

    std::string reason;
    const Trajectory probe =
      Trajectory::build({Vec3{0.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}}, 0.0, shape, reason);
    if (!probe.valid()) {
      RCLCPP_FATAL(get_logger(), "trajectory parameters are unusable: %s", reason.c_str());
      throw std::invalid_argument("navigator trajectory parameters: " + reason);
    }
  }

  void refuseConfiguration(const std::string & detail) const
  {
    RCLCPP_FATAL(get_logger(), "navigator parameters are unusable: %s", detail.c_str());
    throw std::invalid_argument("navigator parameters: " + detail);
  }

  /// Refused, never clamped. Every value here switches off a safety behaviour when
  /// it is zero or NaN, and a guard that quietly stopped biting is the worst of the
  /// three outcomes. Same rule as Costmap::build and LocalAvoidance::build.
  void rejectUnusableAvoidanceLimits() const
  {
    const std::pair<const char *, double> positive[] = {
      {"route_timeout_sec", route_timeout_},
      {"advice_timeout_sec", advice_timeout_},
      {"max_escape_deviation_m", max_escape_deviation_},
      {"escape_replan_interval_sec", escape_replan_interval_},
      {"escape_refresh_m", escape_refresh_},
      {"route_fresh_sec", route_fresh_},
      {"carrot_plan_period_sec", carrot_plan_period_},
      {"avoidance_hold_timeout_sec", avoidance_hold_timeout_},
      {"min_standoff_m", min_standoff_},
      {"target_velocity_error_mps", target_velocity_error_},
      {"target_reaction_sec", target_reaction_},
      {"target_state_timeout_sec", target_state_timeout_},
      {"target_sighting_timeout_sec", target_sighting_timeout_},
      {"target_sighting_period_copy_sec", target_sighting_period_copy_},
      {"tracker_lost_after_copy_sec", tracker_lost_after_copy_},
    };
    for (const std::pair<const char *, double> & entry : positive) {
      if (!std::isfinite(entry.second) || entry.second <= 0.0) {
        refuseConfiguration(std::string(entry.first) + " must be finite and positive");
      }
    }
    if (target_velocity_error_ < kMeasuredTargetVelocityError) {
      char buffer[160];
      std::snprintf(
        buffer, sizeof(buffer),
        "target_velocity_error_mps %.3f is below the measured %.3f m/s (decision 4); "
        "the parameter may widen the margin, never shrink it",
        target_velocity_error_, kMeasuredTargetVelocityError);
      refuseConfiguration(buffer);
    }
    if (target_reaction_ < kDecisionFourReactionSeconds) {
      char buffer[160];
      std::snprintf(
        buffer, sizeof(buffer),
        "target_reaction_sec %.3f is below the %.3f s decision 4 fixed; the margin is a "
        "product, so shrinking either factor erases it",
        target_reaction_, kDecisionFourReactionSeconds);
      refuseConfiguration(buffer);
    }
    if (!std::isfinite(target_lead_) || target_lead_ < 0.0) {
      refuseConfiguration("target_lead_sec must be finite and not negative");
    }
    // R29: one sampling period of grace is zero margin.
    if (target_sighting_timeout_ < 2.0 * target_sighting_period_copy_) {
      char buffer[176];
      std::snprintf(
        buffer, sizeof(buffer),
        "target_sighting_timeout_sec %.3f is below 2 x target_sighting_period_copy_sec "
        "%.3f; one late detection would then be read as a lost target",
        target_sighting_timeout_, target_sighting_period_copy_);
      refuseConfiguration(buffer);
    }
    // Biting only after the tracker gave up is never biting at all.
    if (target_sighting_timeout_ >= tracker_lost_after_copy_) {
      char buffer[176];
      std::snprintf(
        buffer, sizeof(buffer),
        "target_sighting_timeout_sec %.3f is not below tracker_lost_after_copy_sec %.3f; "
        "the status branch would always fire first and this gate would never bite",
        target_sighting_timeout_, tracker_lost_after_copy_);
      refuseConfiguration(buffer);
    }
    if (carrot_plan_period_ < kMinCarrotPlanPeriod) {
      char buffer[176];
      std::snprintf(
        buffer, sizeof(buffer),
        "carrot_plan_period_sec %.3f is below the %.3f s floor; one advisor round trip "
        "was measured at 85 ms, and a leg replaced faster than that reads as silence",
        carrot_plan_period_, kMinCarrotPlanPeriod);
      refuseConfiguration(buffer);
    }
    if (require_obstacle_feed_ && !use_avoidance_) {
      refuseConfiguration("require_obstacle_feed needs use_avoidance");
    }
    // The third rollback, and the quietest: with no route the navigator flies the
    // straight line, which is the local minimum the route planner exists to avoid.
    // A demanded map that only ever sees a straight line is a map nobody used.
    if (require_obstacle_feed_ && !use_route_) {
      refuseConfiguration(
        "require_obstacle_feed needs use_route: without a route every goal is flown "
        "as a straight line, which is what walks into a local minimum");
    }
    // A carrot leg now publishes its own two point segment, so the advisor is no
    // longer blind to it. The rollback is still refused: use_trajectory=false gives
    // up the acceleration ceiling of the core for every goal, and a required map is
    // the configuration that flies people.
    if (require_obstacle_feed_ && !use_trajectory_) {
      refuseConfiguration(
        "require_obstacle_feed needs use_trajectory: the rollback gives up the "
        "acceleration ceiling of the trajectory core");
    }
  }

  /// Same rule as every limit above: refused, never clamped. And a recovery may not
  /// be allowed to climb where an ordinary goal is refused.
  void buildRecoveryPlanner(const RecoveryLimits & limits)
  {
    if (limits.max_safe_altitude_m > limits_.max_altitude) {
      char buffer[176];
      std::snprintf(
        buffer, sizeof(buffer),
        "recovery.max_safe_altitude_m %.2f is above max_altitude_m %.2f: a recovery may "
        "not fly where a goal may not go",
        limits.max_safe_altitude_m, limits_.max_altitude);
      refuseConfiguration(buffer);
    }
    std::string reason;
    recovery_ = RecoveryPlanner::build(limits, reason);
    if (!recovery_.valid()) {
      refuseConfiguration("recovery limits refused: " + reason);
    }
  }

  /// The plan time left once the aircraft is close enough to be called arrived.
  static double arrivalTail(const Trajectory & plan, double acceptance)
  {
    const std::vector<TrajectorySample> & grid = plan.grid();
    const Vec3 finish = plan.finalPosition();
    for (std::size_t index = grid.size(); index-- > 0; ) {
      if (distance3(grid[index].position, finish) > acceptance) {
        return plan.duration() - static_cast<double>(index) * plan.gridPeriod();
      }
    }
    return plan.duration();
  }

  /// Flies the trajectory when one builds, the carrot when it does not, and says which.
  /// Asks the route planner for a way to target and waits for ITS answer, not any
  /// answer: a route published before the question was asked is about another goal.
  /// Empty result plus an empty reason means "route hop switched off".
  std::vector<Vec3> requestRoute(const Vec3 & target, std::string & reason)
  {
    reason.clear();
    route_question_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    if (!use_route_) {
      return {};
    }
    // Nobody planning is not the same as a planner that went quiet, and waiting
    // out the timeout for an absent node delays every goal by it.
    if (route_subscription_->get_publisher_count() == 0) {
      reason = "no route planner is publishing";
      return {};
    }

    PoseStamped question;
    question.header.stamp = now();
    question.header.frame_id = odom_frame_;
    question.pose.position.x = target.x;
    question.pose.position.y = target.y;
    question.pose.position.z = target.z;
    question.pose.orientation.w = 1.0;
    const rclcpp::Time asked(question.header.stamp);
    route_question_stamp_ = asked;
    route_goal_publisher_->publish(question);

    const rclcpp::Time deadline = now() + rclcpp::Duration::from_seconds(route_timeout_);
    while (now() < deadline) {
      Path3D::SharedPtr answer;
      rclcpp::Time answered(0, 0, RCL_ROS_TIME);
      {
        const std::lock_guard<std::mutex> guard(route_mutex_);
        answer = route_;
        answered = route_stamp_;
      }
      // Identity, not arrival order: at 5 Hz a route computed for the PREVIOUS goal
      // still lands up to 200 ms after this question was asked.
      if (answer && rclcpp::Time(answer->goal_stamp) == asked) {
        (void)answered;
        if (answer->plan_state == Path3D::PLAN_STATE_VALID && answer->waypoints.size() >= 2) {
          std::vector<Vec3> waypoints;
          waypoints.reserve(answer->waypoints.size());
          for (const geometry_msgs::msg::Pose & pose : answer->waypoints) {
            waypoints.push_back(Vec3{pose.position.x, pose.position.y, pose.position.z});
          }
          return waypoints;
        }
        reason = answer->reason.empty() ? "route planner refused the goal" : answer->reason;
        return {};
      }
      get_clock()->sleep_for(std::chrono::milliseconds(20));
    }

    char text[64];
    std::snprintf(text, sizeof(text), "route planner silent for %.1f s", route_timeout_);
    reason = text;
    return {};
  }

  /// Route waypoints carry the altitude of the flight band, not the goal's: the
  /// height profile is the caller's job (P6.3 note 4). Spread the climb along the
  /// horizontal path so no single leg becomes a lift.
  static void applyAltitudeProfile(
    std::vector<Vec3> & waypoints, const Vec3 & origin, const Vec3 & target)
  {
    if (waypoints.size() < 2) {
      return;
    }
    std::vector<double> along(waypoints.size(), 0.0);
    double total = 0.0;
    for (std::size_t index = 1; index < waypoints.size(); ++index) {
      total += horizontalDistance(waypoints[index - 1], waypoints[index]);
      along[index] = total;
    }
    waypoints.front().z = origin.z;
    waypoints.back().z = target.z;
    if (!(total > 0.0)) {
      return;
    }
    for (std::size_t index = 1; index + 1 < waypoints.size(); ++index) {
      waypoints[index].z = origin.z + (target.z - origin.z) * (along[index] / total);
    }
  }

  /// False when no trajectory could be built and the carrot took the goal over.
  bool startGotoMotion(
    const Vec3 & target, const std::optional<double> & requested_yaw, double max_speed,
    double acceptance, std::vector<Vec3> via, ViaAltitude altitude)
  {
    // The plan starts at the setpoint in force, not at the measured pose: starting
    // from a noisy measurement steps the setpoint the moment the goal is accepted.
    const std::pair<Vec3, double> anchor = motionAnchor();
    const Vec3 origin = anchor.first;
    const double origin_yaw = anchor.second;
    const double carrot_yaw = requested_yaw ? *requested_yaw : origin_yaw;

    if (!use_trajectory_) {
      setTarget(target, carrot_yaw, max_speed);
      publishNoTrajectory(
        Trajectory3D::PLAN_STATE_NO_GOAL, "use_trajectory=false: flying the carrot");
      return false;
    }

    TrajectoryLimits limits = trajectoryLimits();
    limits.max_speed = max_speed;

    std::vector<Vec3> waypoints{origin};
    if (altitude == ViaAltitude::PROFILE) {
      applyAltitudeProfile(via, origin, target);
    }
    for (const Vec3 & point : via) {
      // The route starts at the aircraft, which the anchor already supplied.
      if (distance3(point, waypoints.back()) > kMinWaypointSpacing) {
        waypoints.push_back(point);
      }
    }
    if (distance3(waypoints.back(), target) > kMinWaypointSpacing) {
      waypoints.push_back(target);
    }

    std::string reason;
    Trajectory plan = Trajectory::build(waypoints, origin_yaw, limits, reason);
    if (!plan.valid()) {
      RCLCPP_WARN(
        get_logger(), "trajectory build failed (%s), flying the carrot", reason.c_str());
      evidence_note_ += "; carrot fallback: " + reason;
      setTarget(target, carrot_yaw, max_speed);
      publishNoTrajectory(Trajectory3D::PLAN_STATE_FAILED, reason);
      return false;
    }

    // Peaks describe the PLAN, not the wire: a clamped leash still steps the stream.
    RCLCPP_INFO(
      get_logger(), "trajectory %.1f s, plan peak %.2f m/s, %.2f m/s2, yaw lag %.0f deg",
      plan.duration(), plan.peakSpeed(), plan.peakAcceleration(),
      plan.peakYawLag() * 180.0 / M_PI);

    const double yaw_margin = arrivalTail(plan, acceptance) + arrival_settle_;
    // Advertise only what is already flying: install first, publish second.
    Trajectory3D message = toMessage(plan);
    setTrajectory(std::move(plan), max_speed, requested_yaw, yaw_margin, origin);
    trajectory_publisher_->publish(std::move(message));
    return true;
  }

  /// Arrival is judged on position, so a nose still turning has to say so.
  void noteYawShortfall(const std::optional<double> & requested_yaw)
  {
    if (!requested_yaw) {
      return;
    }
    const double shortfall = std::abs(wrapAngle(*requested_yaw - commandedYaw()));
    if (shortfall <= 0.1) {
      return;
    }
    char buffer[96];
    std::snprintf(
      buffer, sizeof(buffer), "; nose %.0f deg short of the requested yaw",
      shortfall * 180.0 / M_PI);
    evidence_note_ += buffer;
  }

  // ---------------------------------------------------------------- avoidance

  /// A new goal never inherits the permission, the hold or the escape of the last.
  void beginAvoidance()
  {
    advice_handled_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    escape_replanned_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    active_escape_.reset();
    avoidance_holding_ = false;
    avoidance_hold_started_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    route_rejoin_noted_ = false;
    target_lead_refused_ = false;
    avoidance_unguarded_noted_ = false;
    avoidance_descent_noted_ = false;
    avoidance_holds_ = 0;
    avoidance_escapes_ = 0;
  }

  /// Advice already in flight is about the plan just replaced, and the advisor gets
  /// a fresh window to look at the new one before silence is called.
  void notePlanInstalled()
  {
    plan_installed_ = now();
    advice_permission_ = plan_installed_;
  }

  /// A carrot leg starts with nothing of its own on the wire.
  void beginCarrotPlan()
  {
    carrot_plan_kind_ = CarrotPlan::NONE;
    carrot_plan_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  /// Publishes the leg the carrot is flying, task thread only and rate limited.
  void refreshCarrotPlan(const Vec3 & intended_target)
  {
    const CommandedMotion motion = commandedMotion();
    if (!motion.on_the_carrot) {
      carrot_plan_kind_ = CarrotPlan::NONE;
      return;
    }
    if (!isFinite(intended_target) || !isFinite(motion.setpoint)) {
      return;
    }

    // One message per period whichever way the leg changed, or a flicker storms.
    const rclcpp::Time stamp = now();
    if (carrot_plan_kind_ != CarrotPlan::NONE &&
      (stamp - carrot_plan_stamp_).seconds() < carrot_plan_period_)
    {
      return;
    }

    if (distance3(motion.setpoint, intended_target) < kMinWaypointSpacing) {
      if (carrot_plan_kind_ != CarrotPlan::PARKED) {
        carrot_plan_kind_ = CarrotPlan::PARKED;
        carrot_plan_stamp_ = stamp;
        publishNoTrajectory(
          Trajectory3D::PLAN_STATE_NO_GOAL, "holding the setpoint, no leg to check");
      }
      return;
    }

    if (carrot_plan_kind_ == CarrotPlan::SEGMENT &&
      distance3(motion.setpoint, carrot_plan_from_) < kMinWaypointSpacing &&
      distance3(intended_target, carrot_plan_to_) < kMinWaypointSpacing)
    {
      return;                           // same leg; the one on the wire still holds
    }

    Trajectory3D message =
      toMessage(motion.setpoint, intended_target, motion.limits, motion.yaw);
    carrot_plan_kind_ = CarrotPlan::SEGMENT;
    carrot_plan_from_ = motion.setpoint;
    carrot_plan_to_ = intended_target;
    carrot_plan_stamp_ = stamp;
    // advice_permission_ stays: a moving target must not mask a dead advisor.
    plan_installed_ = stamp;
    trajectory_publisher_->publish(std::move(message));
  }

  /// Said once per goal, whichever way it went.
  void noteRouteRejoin(const std::string & why)
  {
    if (route_rejoin_noted_) {
      return;
    }
    route_rejoin_noted_ = true;
    if (why.empty()) {
      evidence_note_ += "; route rejoined after the escape";
      return;
    }
    evidence_note_ += "; escape went straight at the goal, no route to rejoin: " + why;
    RCLCPP_WARN(get_logger(), "no route to rejoin after the escape: %s", why.c_str());
  }

  /// The freshest route already flowing, from just past the escape point onward.
  ///
  /// Never asks: a blocking re-ask would freeze the task thread for route_timeout
  /// while the aircraft keeps flying the OLD plan straight at the obstacle. At
  /// 0.55 m/s that is 1.1 m against a measured clearance of 0.941 m -- trading one
  /// fault for a worse one. The route planner republishes at 5 Hz anyway.
  std::vector<Vec3> routeTailAfter(const Vec3 & escape, const Vec3 & target)
  {
    if (!use_route_) {
      noteRouteRejoin("route hop is off");
      return {};
    }
    Path3D::SharedPtr latest;
    rclcpp::Time stamp(0, 0, RCL_ROS_TIME);
    {
      const std::lock_guard<std::mutex> guard(route_mutex_);
      latest = route_;
      stamp = route_stamp_;
    }
    if (!latest) {
      noteRouteRejoin("no route has arrived");
      return {};
    }

    char text[96];
    const double age = (now() - stamp).seconds();
    if (!(age >= 0.0) || age > route_fresh_) {
      std::snprintf(
        text, sizeof(text), "route is %.1f s old against a %.1f s ceiling", age, route_fresh_);
      noteRouteRejoin(text);
      return {};
    }
    if (rclcpp::Time(latest->goal_stamp) != route_question_stamp_) {
      noteRouteRejoin("route on the wire answers a different goal");
      return {};
    }
    if (latest->plan_state != Path3D::PLAN_STATE_VALID || latest->waypoints.size() < 2) {
      noteRouteRejoin(latest->reason.empty() ? "route is not valid" : latest->reason);
      return {};
    }

    std::vector<Vec3> route;
    route.reserve(latest->waypoints.size());
    for (const geometry_msgs::msg::Pose & pose : latest->waypoints) {
      const Vec3 point{pose.position.x, pose.position.y, pose.position.z};
      if (!isFinite(point)) {
        noteRouteRejoin("route carries a waypoint that is not finite");
        return {};
      }
      route.push_back(point);
    }
    // The planner clamps a distant goal to the window edge, so the route need not
    // end ON the goal. It must at least end closer to it than the escape point is.
    if (!(distance3(route.back(), target) < distance3(escape, target))) {
      noteRouteRejoin("route does not lead any closer to the goal");
      return {};
    }

    std::size_t nearest = 0;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < route.size(); ++index) {
      const double gap = distance3(route[index], escape);
      if (gap < best) {
        best = gap;
        nearest = index;
      }
    }
    std::vector<Vec3> tail(
      route.begin() + static_cast<std::ptrdiff_t>(nearest) + 1, route.end());
    if (tail.empty()) {
      noteRouteRejoin("the escape point is already past the end of the route");
      return {};
    }
    noteRouteRejoin(std::string());
    return tail;
  }

  void holdForAvoidance(const std::string & why)
  {
    if (avoidance_holding_) {
      return;
    }
    avoidance_holding_ = true;
    avoidance_hold_started_ = now();
    active_escape_.reset();
    pauseMotion();
    if (avoidance_holds_ == 0) {
      evidence_note_ += "; avoidance hold: " + why;
    }
    ++avoidance_holds_;
    RCLCPP_WARN(get_logger(), "avoidance hold: %s", why.c_str());
  }

  void releaseAvoidanceHold()
  {
    if (!avoidance_holding_) {
      return;
    }
    avoidance_holding_ = false;
    avoidance_hold_started_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    resumeMotion();
    RCLCPP_INFO(get_logger(), "avoidance cleared, the plan runs on");
  }

  /// Without a costmap the navigator cannot judge whether an escape point is clear.
  /// It can judge whether the point belongs to the flight it is actually planning.
  bool escapeIsUsable(const Vec3 & escape, const Vec3 & target, std::string & why)
  {
    if (!isFinite(escape)) {
      why = "escape point is not finite";
      return false;
    }
    char buffer[128];
    // The altitude envelope is enforced at the goal door, and an escape never goes
    // through that door. A recovery may not fly where a goal may not go, and neither
    // may an escape: the core would smooth a descent to -3 m the aircraft can track.
    if (escape.z < limits_.min_altitude || escape.z > limits_.max_altitude) {
      std::snprintf(
        buffer, sizeof(buffer),
        "escape point at %.2f m is outside min/max_altitude_m [%.2f, %.2f]",
        escape.z, limits_.min_altitude, limits_.max_altitude);
      why = buffer;
      return false;
    }
    const Vec3 from = currentCarrot();
    const double deviation = deviationFromLeg(escape, from, target);
    if (deviation > max_escape_deviation_) {
      std::snprintf(
        buffer, sizeof(buffer), "escape point is %.1f m off the plan, ceiling %.1f m",
        deviation, max_escape_deviation_);
      why = buffer;
      return false;
    }
    // Refusing a descent belongs to the advisor; saying it happened belongs here.
    if (from.z - escape.z > kEscapeDescentNotice && !avoidance_descent_noted_) {
      avoidance_descent_noted_ = true;
      std::snprintf(
        buffer, sizeof(buffer), "; avoidance escape descended %.2f m", from.z - escape.z);
      evidence_note_ += buffer;
      RCLCPP_WARN(get_logger(), "avoidance escape descends %.2f m", from.z - escape.z);
    }
    return true;
  }

  void flyAround(
    const Vec3 & escape, const std::string & reason, const GotoPlanRequest & request)
  {
    // Rebuilding at advice rate would restart the plan from rest ten times a second.
    if (active_escape_ && distance3(escape, *active_escape_) <= escape_refresh_) {
      return;
    }
    const rclcpp::Time stamp = now();
    if (escape_replanned_.nanoseconds() != 0 &&
      (stamp - escape_replanned_).seconds() < escape_replan_interval_)
    {
      return;
    }
    // Dropping the plan shape here is how a goal behind a wall livelocks: the first
    // obstacle advises an escape, the escape flies straight at the wall again, and
    // the pair alternate until the goal times out. Keep what is still ahead.
    std::vector<Vec3> via{escape};
    if (request.tailAfter) {
      const std::vector<Vec3> tail = request.tailAfter(escape);
      via.insert(via.end(), tail.begin(), tail.end());
    }
    // The escape altitude is the deliberate part; a route tail carries the flight
    // band, so the rest of the climb is spread from the escape onward.
    if (via.size() >= 2 && request.tail_altitude == ViaAltitude::PROFILE) {
      applyAltitudeProfile(via, escape, request.target);
    }
    const bool planned = startGotoMotion(
      request.target, request.requested_yaw, request.max_speed, request.acceptance,
      via, ViaAltitude::KEEP);
    notePlanInstalled();
    escape_replanned_ = plan_installed_;
    if (!planned && require_obstacle_feed_) {
      // With no trajectory the advisor checks nothing forever; end the goal now
      // instead of letting it hold out the whole goto timeout.
      planner_fault_ = "escape plan failed to build and the map is required";
      return;
    }
    active_escape_ = escape;
    avoidance_holding_ = false;
    if (avoidance_escapes_ == 0) {
      evidence_note_ += "; avoidance escape: " + reason;
    }
    ++avoidance_escapes_;
    RCLCPP_WARN(get_logger(), "flying around: %s", reason.c_str());
  }

  /// True when the advisor proved it is alive AND looked ahead. Refusing its escape
  /// still counts: the timeout below is about a dead advisor, not a wrong one.
  bool applyAdvice(const AvoidanceAdvice & advice, const GotoPlanRequest & request)
  {
    if (advice.advice == AvoidanceAdvice::ADVICE_HOLD) {
      holdForAvoidance(advice.reason);
      return true;
    }
    if (advice.advice == AvoidanceAdvice::ADVICE_ESCAPE) {
      // An escape is flown by rebuilding the trajectory. A carrot leg has no such
      // machine, so it stops instead of improvising one (README 6h).
      if (!commandedMotion().on_the_carrot) {
        const Vec3 escape{advice.escape_point.x, advice.escape_point.y, advice.escape_point.z};
        std::string why;
        if (!escapeIsUsable(escape, request.target, why)) {
          holdForAvoidance("escape refused, " + why);
          return true;
        }
        flyAround(escape, advice.reason, request);
        return true;
      }
      holdForAvoidance("escape is not flown on a carrot leg: " + advice.reason);
      return true;
    }
    // CLEAR with a zero horizon is how the advisor says it checked nothing at all.
    // That is not permission to fly, so it decays into the silence branch below.
    if (!(advice.checked_horizon_m > 0.0F)) {
      return false;
    }
    releaseAvoidanceHold();
    return true;
  }

  /// Runs on the task thread at progress_hz_, never on the stream timer.
  void watchAvoidance(const GotoPlanRequest & request)
  {
    // Publishing does not depend on use_avoidance_: the plan in force is a contract
    // with every consumer, not just with the advisor.
    refreshCarrotPlan(request.target);
    if (!use_avoidance_) {
      return;
    }
    endlessHold();

    AvoidanceAdvice::SharedPtr advice;
    {
      const std::lock_guard<std::mutex> guard(advice_mutex_);
      advice = advice_;
    }

    const rclcpp::Time stamp = now();
    if (advice) {
      const rclcpp::Time said(advice->header.stamp);
      if (said > advice_handled_ && said > plan_installed_) {
        advice_handled_ = said;
        if (applyAdvice(*advice, request)) {
          advice_permission_ = stamp;
          return;
        }
      }
    }

    if ((stamp - advice_permission_).seconds() <= advice_timeout_) {
      return;
    }
    const char * why = advice_subscription_->get_publisher_count() == 0
      ? "no obstacle advisor is publishing"
      : "the obstacle advisor has not checked the way ahead";
    if (require_obstacle_feed_) {
      holdForAvoidance(why);
      return;
    }
    if (!avoidance_unguarded_noted_) {
      avoidance_unguarded_noted_ = true;
      evidence_note_ += std::string("; flying unguarded: ") + why;
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "flying unguarded: %s", why);
  }

  /// A hold waits for the way to clear. One that never does is not waiting, and
  /// sitting on it until the goal timeout hides the reason behind the wrong code --
  /// the same lesson as the R-C2b patch, which cut 6988 ms of that to 972 ms.
  void endlessHold()
  {
    if (!avoidance_holding_ || avoidance_hold_started_.nanoseconds() == 0 ||
      !planner_fault_.empty())
    {
      return;
    }
    const double held = (now() - avoidance_hold_started_).seconds();
    if (held <= avoidance_hold_timeout_) {
      return;
    }
    char buffer[160];
    std::snprintf(
      buffer, sizeof(buffer),
      "held by the obstacle advisor for %.1f s with no way through, ceiling %.1f s",
      held, avoidance_hold_timeout_);
    planner_fault_ = buffer;
    RCLCPP_ERROR(get_logger(), "%s", buffer);
  }

  /// One line per goal however often the advisor spoke.
  void noteAvoidanceTally()
  {
    char buffer[96];
    if (avoidance_escapes_ > 1) {
      std::snprintf(buffer, sizeof(buffer), "; %d avoidance escapes flown", avoidance_escapes_);
      evidence_note_ += buffer;
    }
    if (avoidance_holds_ > 1) {
      std::snprintf(buffer, sizeof(buffer), "; %d avoidance holds", avoidance_holds_);
      evidence_note_ += buffer;
    }
  }

  void executeGoto(const std::shared_ptr<GoalHandleGoto> goal_handle)
  {
    auto result = std::make_shared<GotoPose::Result>();
    acceptance_note_.clear();
    evidence_note_.clear();
    const auto goal = goal_handle->get_goal();
    const GotoLimits resolved = resolveGotoLimits(toGotoRequest(*goal), limits_);

    const Vec3 target{
      goal->target_pose.position.x, goal->target_pose.position.y, goal->target_pose.position.z};
    const auto & orientation = goal->target_pose.orientation;
    const std::optional<double> requested_yaw =
      hasUsableOrientation(orientation.x, orientation.y, orientation.z, orientation.w)
      ? std::optional<double>(
        yawFromQuaternion(orientation.x, orientation.y, orientation.z, orientation.w))
      : std::nullopt;

    if (!streaming()) {
      result->success = false;
      result->result_code = ResultCode::ABORTED_INTERNAL_ERROR;
      result->message = "navigator is not streaming setpoints";
      setState(NavigatorState::IDLE);
      goal_handle->abort(result);
      return;
    }

    std::string route_reason;
    std::vector<Vec3> route = requestRoute(target, route_reason);
    if (route.empty() && !route_reason.empty()) {
      // Flying blind is a choice the airframe config makes, never a silent default.
      if (require_obstacle_feed_) {
        result->success = false;
        result->result_code = ResultCode::ABORTED_INTERNAL_ERROR;
        result->message = "no route and the map is required: " + route_reason;
        setState(NavigatorState::IDLE);
        goal_handle->abort(result);
        return;
      }
      RCLCPP_WARN(
        get_logger(), "no route (%s), flying the straight line", route_reason.c_str());
      evidence_note_ += "; straight line, no route: " + route_reason;
    }

    const double acceptance = applyAcceptance(resolved.acceptance_radius, "goto_pose");
    GotoPlanRequest plan_request;
    plan_request.target = target;
    plan_request.requested_yaw = requested_yaw;
    plan_request.max_speed = resolved.max_speed;
    plan_request.acceptance = acceptance;
    plan_request.tailAfter = [this, target](const Vec3 & escape) {
        return routeTailAfter(escape, target);
      };
    beginAvoidance();
    const bool planned = startGotoMotion(
      target, requested_yaw, resolved.max_speed, acceptance, std::move(route),
      ViaAltitude::PROFILE);
    notePlanInstalled();
    if (!planned && require_obstacle_feed_) {
      // The advisor only checks a published trajectory. Without one it reports a
      // zero horizon forever, so this goal could only hold until it timed out.
      freezeSetpoint();
      result->success = false;
      result->result_code = ResultCode::ABORTED_PLANNER_FAILED;
      result->message = "no trajectory and the map is required" + evidence_note_;
      setState(NavigatorState::HOLDING);
      RCLCPP_ERROR(get_logger(), "goto_pose aborted: %s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }
    beginTask();

    auto feedback = std::make_shared<GotoPose::Feedback>();
    const auto reached = [this, target, acceptance]() {
        return distance3(poseSample().position, target) <= acceptance;
      };
    const auto publish = [this, goal_handle, feedback, target, resolved]() {
        const Vec3 position = poseSample().position;
        const double remaining = distance3(position, target);
        feedback->current_position.x = position.x;
        feedback->current_position.y = position.y;
        feedback->current_position.z = position.z;
        feedback->distance_remaining = static_cast<float>(remaining);
        const double planned_remaining = trajectoryRemaining();
        feedback->estimated_time_remaining = static_cast<float>(
          planned_remaining > 0.0
          ? planned_remaining
          : remaining / std::max(0.01, resolved.max_speed));
        goal_handle->publish_feedback(feedback);
      };

    Fault fault;
    const LoopOutcome outcome = runProgressLoop(
      goal_handle, reached, publish, goto_timeout_, arrival_settle_, fault,
      [this, plan_request]() {watchAvoidance(plan_request);});
    result->final_distance_error =
      static_cast<float>(distance3(poseSample().position, target));
    noteAvoidanceTally();
    noteYawShortfall(requested_yaw);
    finishAirborne(goal_handle, outcome, fault, result, "goto_pose");
  }

  // -------------------------------------------------------------- follow path

  /// Marks how far along the caller list the aircraft has come. Task thread only.
  ///
  /// Two counters, because they answer two different questions. "achieved" is what
  /// the result reports and only ticks inside the acceptance radius. "ahead" is what
  /// a replan splices from, and also steps past a waypoint the spline cut the corner
  /// on -- otherwise an escape would turn the aircraft round to collect it.
  static void advancePathProgress(
    PathProgress & progress, const std::vector<Vec3> & waypoints, const Vec3 & position,
    double acceptance)
  {
    while (progress.ahead < waypoints.size()) {
      const double here = distance3(position, waypoints[progress.ahead]);
      if (here <= acceptance) {
        progress.achieved[progress.ahead] = true;
        ++progress.ahead;
        continue;
      }
      const bool nearer_next = progress.ahead + 1 < waypoints.size() &&
        distance3(position, waypoints[progress.ahead + 1]) < here;
      if (!nearer_next) {
        break;
      }
      ++progress.ahead;
    }
  }

  static std::size_t countAchieved(const PathProgress & progress)
  {
    std::size_t total = 0;
    for (const bool achieved : progress.achieved) {
      if (achieved) {
        ++total;
      }
    }
    return total;
  }

  /// Clamped projection keeps a detour from reading as leg progress.
  static double projectedShare(const Vec3 & from, const Vec3 & to, const Vec3 & position)
  {
    const Vec3 leg{to.x - from.x, to.y - from.y, to.z - from.z};
    const double length_squared = leg.x * leg.x + leg.y * leg.y + leg.z * leg.z;
    if (length_squared <= 1e-12) {
      return 1.0;
    }
    const double along =
      (position.x - from.x) * leg.x + (position.y - from.y) * leg.y +
      (position.z - from.z) * leg.z;
    return std::min(std::max(along / length_squared, 0.0), 1.0);
  }

  /// Arc length covered, not waypoint count: legs differ in length.
  static double percentAlongPath(
    const PathProgress & progress, const std::vector<Vec3> & waypoints,
    const std::vector<double> & milestones, const Vec3 & position)
  {
    if (progress.ahead >= waypoints.size()) {
      return 100.0;
    }
    const double total = milestones.back();
    if (total <= 1e-9) {
      return 0.0;
    }
    double covered = 0.0;
    if (progress.ahead > 0) {
      const Vec3 & from = waypoints[progress.ahead - 1];
      const Vec3 & to = waypoints[progress.ahead];
      covered = milestones[progress.ahead - 1] +
        distance3(from, to) * projectedShare(from, to, position);
    }
    return 100.0 * covered / total;
  }

  void executeFollowPath(const std::shared_ptr<GoalHandleFollow> goal_handle)
  {
    auto result = std::make_shared<FollowPath::Result>();
    acceptance_note_.clear();
    evidence_note_.clear();
    const auto goal = goal_handle->get_goal();

    std::vector<Vec3> waypoints;
    waypoints.reserve(goal->path.waypoints.size());
    for (const geometry_msgs::msg::Pose & pose : goal->path.waypoints) {
      waypoints.push_back(Vec3{pose.position.x, pose.position.y, pose.position.z});
    }

    if (!streaming() || waypoints.empty()) {
      result->success = false;
      result->result_code = ResultCode::ABORTED_INTERNAL_ERROR;
      result->message = "navigator is not streaming setpoints";
      setState(NavigatorState::IDLE);
      goal_handle->abort(result);
      return;
    }

    GotoRequest shape;
    shape.acceptance_radius = goal->waypoint_acceptance_radius;
    shape.max_speed = goal->max_speed;
    const GotoLimits resolved = resolveGotoLimits(shape, limits_);
    const double acceptance = applyAcceptance(resolved.acceptance_radius, "follow_path");

    const Vec3 target = waypoints.back();
    const std::vector<Vec3> via(waypoints.begin(), waypoints.end() - 1);

    auto progress = std::make_shared<PathProgress>();
    progress->achieved.assign(waypoints.size(), false);

    std::vector<double> milestones(waypoints.size(), 0.0);
    for (std::size_t index = 1; index < waypoints.size(); ++index) {
      milestones[index] = milestones[index - 1] +
        distance3(waypoints[index - 1], waypoints[index]);
    }

    GotoPlanRequest plan_request;
    plan_request.target = target;
    plan_request.max_speed = resolved.max_speed;
    plan_request.acceptance = acceptance;
    // The caller chose these altitudes; a replan must not re-profile them.
    plan_request.tail_altitude = ViaAltitude::KEEP;
    plan_request.tailAfter = [waypoints, progress](const Vec3 &) {
        std::vector<Vec3> tail;
        for (std::size_t index = progress->ahead; index + 1 < waypoints.size(); ++index) {
          tail.push_back(waypoints[index]);
        }
        return tail;
      };

    beginAvoidance();
    const bool planned = startGotoMotion(
      target, std::nullopt, resolved.max_speed, acceptance, via, ViaAltitude::KEEP);
    notePlanInstalled();
    if (!planned && require_obstacle_feed_) {
      freezeSetpoint();
      result->success = false;
      result->result_code = ResultCode::ABORTED_PLANNER_FAILED;
      result->message = "no trajectory and the map is required" + evidence_note_;
      setState(NavigatorState::HOLDING);
      RCLCPP_ERROR(get_logger(), "follow_path aborted: %s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }
    beginTask();

    auto feedback = std::make_shared<FollowPath::Feedback>();
    const auto reached = [this, waypoints, progress, target, acceptance]() {
        advancePathProgress(*progress, waypoints, poseSample().position, acceptance);
        return distance3(poseSample().position, target) <= acceptance;
      };
    const auto publish = [this, goal_handle, feedback, waypoints, progress, milestones]() {
        const std::size_t index = std::min(progress->ahead, waypoints.size() - 1);
        feedback->current_waypoint_index = static_cast<uint32_t>(index);
        feedback->distance_to_next_waypoint =
          static_cast<float>(distance3(poseSample().position, waypoints[index]));
        // Held at its peak: a detour must not read as progress lost.
        feedback->path_completion_percent = std::max(
          feedback->path_completion_percent,
          static_cast<float>(
            percentAlongPath(*progress, waypoints, milestones, poseSample().position)));
        goal_handle->publish_feedback(feedback);
      };

    Fault fault;
    const LoopOutcome outcome = runProgressLoop(
      goal_handle, reached, publish, goto_timeout_, arrival_settle_, fault,
      [this, plan_request]() {watchAvoidance(plan_request);});

    const std::size_t achieved = countAchieved(*progress);
    result->waypoints_reached = static_cast<uint32_t>(achieved);
    if (achieved < waypoints.size()) {
      char buffer[96];
      std::snprintf(
        buffer, sizeof(buffer), "; %zu of %zu waypoints came inside %.2f m",
        achieved, waypoints.size(), acceptance);
      evidence_note_ += buffer;
    }
    noteAvoidanceTally();
    finishAirborne(goal_handle, outcome, fault, result, "follow_path");
  }

  // ------------------------------------------------------------- track target

  /// Decision 4: the target velocity estimate is wrong by target_velocity_error_mps
  /// (0.71 m/s measured on a 1 m/s target, 71 %). That is an information limit, not
  /// a filter bug, so a reaction time of it is paid for out of the standoff.
  double effectiveStandoff(double requested) const
  {
    const double base = (std::isfinite(requested) && requested > 0.0)
      ? std::max(min_standoff_, requested)
      : min_standoff_;
    return base + target_velocity_error_ * target_reaction_;
  }

  TargetSample targetSample(int32_t wanted_id)
  {
    TargetSample sample;
    TargetState::SharedPtr state;
    {
      const std::lock_guard<std::mutex> guard(target_mutex_);
      state = target_state_;
    }
    if (!state) {
      sample.why = "no target_state has arrived";
      return sample;
    }
    if (wanted_id >= 0 && state->track_id != wanted_id) {
      sample.why = "target_state is about another track";
      return sample;
    }
    const double age = (now() - rclcpp::Time(state->header.stamp)).seconds();
    if (!(age >= 0.0) || age > target_state_timeout_) {
      sample.why = "target_state went stale";
      return sample;
    }
    // Above reads a publish-time stamp: liveness, not sighting.
    if (state->status == TargetTrack::STATUS_LOST) {
      sample.why = "target_state reports the track as lost";
      return sample;
    }
    const double seen_ago = state->time_since_seen_sec;
    // Negated: NaN loses every comparison and would read fresh.
    if (!(seen_ago >= 0.0) || seen_ago > target_sighting_timeout_) {
      char buffer[112];
      std::snprintf(
        buffer, sizeof(buffer), "target has not been seen for %.2f s, ceiling %.2f s",
        seen_ago, target_sighting_timeout_);
      sample.why = buffer;
      return sample;
    }
    sample.position =
      Vec3{state->pose.position.x, state->pose.position.y, state->pose.position.z};
    if (!isFinite(sample.position)) {
      sample.why = "target position is not finite";
      return sample;
    }
    sample.velocity = Vec3{state->velocity.x, state->velocity.y, state->velocity.z};
    sample.uncertainty = state->position_uncertainty;
    sample.usable = true;
    return sample;
  }

  /// Where to sit: standoff metres from the target, on our own side of it, at our
  /// own altitude. Chasing the target altitude would fly a ground target into the
  /// ground; the height profile stays the operator decision.
  Vec3 standoffPoint(const TargetSample & sample, const Vec3 & from, double standoff)
  {
    Vec3 aim = sample.position;
    if (target_lead_ > 0.0) {
      // The one rule that is not negotiable: without a stated uncertainty the
      // velocity is not evidence, so it may not move the aim point (decision 4).
      if (std::isfinite(sample.uncertainty) && sample.uncertainty >= 0.0) {
        aim.x += sample.velocity.x * target_lead_;
        aim.y += sample.velocity.y * target_lead_;
      } else if (!target_lead_refused_) {
        target_lead_refused_ = true;
        evidence_note_ += "; target lead refused: target_state carries no uncertainty";
        RCLCPP_WARN(get_logger(), "target lead refused: position_uncertainty is missing");
      }
    }
    double away_x = from.x - aim.x;
    double away_y = from.y - aim.y;
    double span = std::hypot(away_x, away_y);
    if (!(span > 1e-3)) {
      away_x = 1.0;
      away_y = 0.0;
      span = 1.0;
    }
    return Vec3{aim.x + away_x / span * standoff, aim.y + away_y / span * standoff, from.z};
  }

  void executeTrackTarget(const std::shared_ptr<GoalHandleTrack> goal_handle)
  {
    auto result = std::make_shared<TrackTarget::Result>();
    acceptance_note_.clear();
    evidence_note_.clear();
    const auto goal = goal_handle->get_goal();

    if (!streaming()) {
      result->success = false;
      result->result_code = ResultCode::ABORTED_INTERNAL_ERROR;
      result->message = "navigator is not streaming setpoints";
      setState(NavigatorState::IDLE);
      goal_handle->abort(result);
      return;
    }

    const double standoff = effectiveStandoff(goal->standoff_distance);
    const double max_speed = limits_.max_speed;
    char note[112];
    std::snprintf(
      note, sizeof(note), "; standoff %.2f m = %.2f requested plus %.2f m of reaction",
      standoff, goal->standoff_distance, target_velocity_error_ * target_reaction_);
    evidence_note_ += note;

    // Tracking flies the carrot, so what it publishes is the two point leg it is on.
    freezeSetpoint();
    beginAvoidance();
    beginTask();
    notePlanInstalled();
    const rclcpp::Time start = now();
    auto lost_since = std::make_shared<rclcpp::Time>(0, 0, RCL_ROS_TIME);
    auto visible = std::make_shared<bool>(false);

    const auto pursue = [this, goal, standoff, max_speed, lost_since, visible]() {
        const TargetSample sample = targetSample(goal->target_id);
        const rclcpp::Time stamp = now();
        *visible = sample.usable;
        if (!sample.usable) {
          if (lost_since->nanoseconds() == 0) {
            *lost_since = stamp;
            RCLCPP_WARN(get_logger(), "target lost: %s", sample.why.c_str());
          }
          if ((stamp - *lost_since).seconds() > goal->target_lost_timeout) {
            planner_fault_code_ = ResultCode::ABORTED_LOST_TARGET;
            planner_fault_ = "target lost for longer than target_lost_timeout: " + sample.why;
          }
          refreshCarrotPlan(freezeSetpoint());
          return;
        }
        *lost_since = rclcpp::Time(0, 0, RCL_ROS_TIME);
        const Vec3 from = currentCarrot();
        const Vec3 stand = standoffPoint(sample, from, standoff);
        const double bearing =
          std::atan2(sample.position.y - stand.y, sample.position.x - stand.x);

        GotoPlanRequest plan_request;
        plan_request.target = stand;
        plan_request.max_speed = max_speed;
        plan_request.tail_altitude = ViaAltitude::KEEP;
        // Publishes the leg, then reads what the advisor made of the last one. A
        // held leg is still published: it is how the advisor can ever clear it.
        watchAvoidance(plan_request);
        if (avoidance_holding_) {
          return;
        }
        setTarget(stand, bearing, max_speed);
      };

    auto feedback = std::make_shared<TrackTarget::Feedback>();
    const auto reached = [this, goal, start]() {
        return goal->duration_seconds > 0.0 &&
               (now() - start).seconds() >= goal->duration_seconds;
      };
    const auto publish = [this, goal, goal_handle, feedback, lost_since, visible]() {
        feedback->target_visible = *visible;
        const TargetSample sample = targetSample(goal->target_id);
        feedback->distance_to_target = sample.usable
          ? static_cast<float>(distance3(poseSample().position, sample.position))
          : std::numeric_limits<float>::infinity();
        feedback->target_lost_seconds = lost_since->nanoseconds() == 0
          ? 0.0F
          : static_cast<float>((now() - *lost_since).seconds());
        goal_handle->publish_feedback(feedback);
      };

    // An open-ended track ends on cancel or on losing the target, never on a clock.
    const double timeout = goal->duration_seconds > 0.0
      ? goal->duration_seconds + hold_timeout_margin_
      : std::numeric_limits<double>::max();

    Fault fault;
    const LoopOutcome outcome =
      runProgressLoop(goal_handle, reached, publish, timeout, 0.0, fault, pursue);
    result->tracked_seconds = static_cast<float>((now() - start).seconds());
    noteAvoidanceTally();
    finishAirborne(goal_handle, outcome, fault, result, "track_target");
  }

  // ------------------------------------------------------------------ recover

  /// Read from the pose, not from a step counter: a counter would keep reporting a
  /// climb the aircraft stopped making.
  static RecoveryStage recoveryStage(
    const RecoveryPlan & plan, const Vec3 & position, double acceptance)
  {
    const bool below = position.z < plan.hold_point.z - acceptance;
    if (plan.type == RecoveryType::ClimbToSafe) {
      return below ? RECOVERY_STAGE_CLIMBING : RECOVERY_STAGE_HOLDING;
    }
    if (plan.type == RecoveryType::ReturnHome) {
      if (plan.waypoints.size() >= 3 && below) {
        return RECOVERY_STAGE_CLIMBING;
      }
      return horizontalDistance(position, plan.hold_point) > acceptance
             ? RECOVERY_STAGE_CRUISING
             : RECOVERY_STAGE_HOLDING;
    }
    return RECOVERY_STAGE_HOLDING;
  }

  static std::string describeStage(
    RecoveryStage stage, const RecoveryPlan & plan, const Vec3 & position)
  {
    char buffer[128];
    switch (stage) {
      case RECOVERY_STAGE_CLIMBING:
        std::snprintf(buffer, sizeof(buffer), "climbing to %.2f m", plan.hold_point.z);
        break;
      case RECOVERY_STAGE_CRUISING:
        std::snprintf(
          buffer, sizeof(buffer), "crossing the last %.2f m to home",
          horizontalDistance(position, plan.hold_point));
        break;
      case RECOVERY_STAGE_HOLDING:
      default:
        std::snprintf(buffer, sizeof(buffer), "holding at %.2f m", plan.hold_point.z);
        break;
    }
    return std::string(buffer);
  }

  void executeRecover(const std::shared_ptr<GoalHandleRecover> goal_handle)
  {
    auto result = std::make_shared<Recover::Result>();
    result->executed_type = Recover::Goal::TYPE_UNKNOWN;
    acceptance_note_.clear();
    evidence_note_.clear();
    // The preempted task has stood down: the worker joined its thread to get here.
    preempt_requested_ = false;
    recovery_pending_ = false;
    setState(NavigatorState::RECOVERING);

    const auto goal = goal_handle->get_goal();
    std::string refusal;
    const std::optional<RecoveryType> type =
      executableRecoveryType(goal->recovery_type, refusal);
    if (!type) {
      result->success = false;
      result->result_code = ResultCode::ABORTED_INVALID_GOAL;
      result->message = refusal;
      setState(NavigatorState::HOLDING);
      goal_handle->abort(result);
      return;
    }

    if (!streaming()) {
      result->success = false;
      result->result_code = ResultCode::ABORTED_INTERNAL_ERROR;
      result->message = "navigator is not streaming setpoints";
      setState(NavigatorState::IDLE);
      goal_handle->abort(result);
      return;
    }

    RecoveryState state;
    // The anchor, not the measured pose: a recovery that steps the setpoint on its
    // first tick is a recovery that made things worse.
    state.position = motionAnchor().first;
    state.home = home_;
    state.home_known = home_known_;

    RecoveryRequest request;
    request.type = *type;
    request.safe_altitude_m = goal->safe_altitude;

    const RecoveryPlan plan = recovery_.plan(request, state);
    if (!plan.accepted) {
      result->success = false;
      result->result_code = ResultCode::ABORTED_INVALID_GOAL;
      result->message = std::string(recoveryTypeName(*type)) + " refused: " + plan.reason;
      setState(NavigatorState::HOLDING);
      RCLCPP_ERROR(get_logger(), "recover refused: %s", plan.reason.c_str());
      goal_handle->abort(result);
      return;
    }

    RCLCPP_WARN(
      get_logger(), "recovery taking the aircraft over: %s (%s)",
      plan.reason.c_str(), goal->trigger_reason.c_str());
    result->executed_type = goal->recovery_type;
    evidence_note_ += "; recovery " + std::string(recoveryTypeName(*type)) + ": " + plan.reason;

    const double acceptance = applyAcceptance(limits_.acceptance_radius, "recover");
    const Vec3 hold_point = plan.hold_point;
    const bool moves = plan.waypoints.size() >= 2;

    beginAvoidance();
    beginTask();
    if (moves) {
      std::vector<Vec3> via;
      if (plan.waypoints.size() > 2) {
        via.assign(plan.waypoints.begin() + 1, plan.waypoints.end() - 1);
      }
      // The ordinary flight path: same core, same leash, same single publisher. A
      // recovery does not get a second way into the setpoint stream.
      startGotoMotion(
        hold_point, std::nullopt, limits_.max_speed, acceptance, via, ViaAltitude::KEEP);
      notePlanInstalled();
    } else {
      refreshCarrotPlan(freezeSetpoint());
      notePlanInstalled();
      // The one task that must outlive the pose it cannot read (contract 2.5).
      holding_without_localization_ = true;
    }

    auto feedback = std::make_shared<Recover::Feedback>();
    // A hover proves itself from what it EMITS -- the setpoint standing still and
    // the stream still running for a settle time of it. Judging it by distance to a
    // measured pose would fail exactly when the pose is the thing that went.
    const uint64_t held_ticks =
      static_cast<uint64_t>(std::ceil(stream_hz_ * arrival_settle_));
    const auto reached = [this, hold_point, acceptance, moves, held_ticks]() {
        if (moves) {
          return distance3(poseSample().position, hold_point) <= acceptance;
        }
        return distance3(currentCarrot(), hold_point) <= kFrozenSetpointDrift &&
               stream_ticks_.load() >= held_ticks;
      };
    const auto publish = [this, goal_handle, feedback, plan, acceptance]() {
        const Vec3 position = poseSample().position;
        const RecoveryStage stage = recoveryStage(plan, position, acceptance);
        feedback->current_stage = static_cast<uint8_t>(stage);
        feedback->stage_description = describeStage(stage, plan, position);
        goal_handle->publish_feedback(feedback);
      };

    GotoPlanRequest plan_request;
    plan_request.target = hold_point;
    plan_request.max_speed = limits_.max_speed;
    plan_request.acceptance = acceptance;
    // The caller chose these altitudes; a replan must not re-profile them.
    plan_request.tail_altitude = ViaAltitude::KEEP;

    Fault fault;
    LoopOutcome outcome = LoopOutcome::FAULT;
    if (moves) {
      outcome = runProgressLoop(
        goal_handle, reached, publish, goto_timeout_, arrival_settle_, fault,
        [this, plan_request]() {watchAvoidance(plan_request);});
    } else {
      // No advisor watch: a hold goes nowhere, so there is no way ahead to clear.
      // No settle either: the tick count in reached() IS the settle, and it is
      // counted on the setpoints that went out rather than on a pose.
      outcome = runProgressLoop(goal_handle, reached, publish, goto_timeout_, 0.0, fault);
    }
    noteAvoidanceTally();
    finishAirborne(goal_handle, outcome, fault, result, "recover");
  }

  void executeHold(const std::shared_ptr<GoalHandleHold> goal_handle)
  {
    auto result = std::make_shared<HoldPosition::Result>();
    acceptance_note_.clear();
    evidence_note_.clear();
    const double duration = goal_handle->get_goal()->duration_seconds;

    if (!streaming()) {
      result->success = false;
      result->result_code = ResultCode::ABORTED_INTERNAL_ERROR;
      result->message = "navigator is not streaming setpoints";
      setState(NavigatorState::IDLE);
      goal_handle->abort(result);
      return;
    }

    const Vec3 hold_point = freezeSetpoint();
    beginTask();
    // Says on the wire what a hold is: a commanded point, and no leg to check.
    refreshCarrotPlan(hold_point);
    const rclcpp::Time start = now();

    auto feedback = std::make_shared<HoldPosition::Feedback>();
    const auto reached = [this, duration, start]() {
        return duration > 0.0 && (now() - start).seconds() >= duration;
      };
    const auto publish = [this, goal_handle, feedback, hold_point, start]() {
        feedback->elapsed_seconds = static_cast<float>((now() - start).seconds());
        feedback->position_error =
          static_cast<float>(distance3(poseSample().position, hold_point));
        goal_handle->publish_feedback(feedback);
      };

    // An indefinite hold ends only on cancel, so its timeout must never fire.
    const double timeout = duration > 0.0
      ? duration + hold_timeout_margin_
      : std::numeric_limits<double>::max();

    Fault fault;
    const LoopOutcome outcome =
      runProgressLoop(goal_handle, reached, publish, timeout, 0.0, fault);
    result->held_seconds = static_cast<float>((now() - start).seconds());
    finishAirborne(goal_handle, outcome, fault, result, "hold_position");
  }

  void executeLand(const std::shared_ptr<GoalHandleLand> goal_handle)
  {
    auto result = std::make_shared<Land::Result>();
    result->landed_on_marker = false;

    const double reference_z = ground_reference_z_.load();

    auto request = std::make_shared<SetFlightMode::Request>();
    request->mode = VehicleState::FLIGHT_MODE_LAND;
    if (!callService(mode_client_, request, "set_mode LAND", result->message)) {
      result->success = false;
      result->result_code = ResultCode::ABORTED_VEHICLE_REJECTED;
      setState(NavigatorState::HOLDING);
      goal_handle->abort(result);
      return;
    }

    // Mode first, stop streaming after; the reverse order trips failsafe.
    get_clock()->sleep_for(rclcpp::Duration::from_seconds(land_stream_stop_delay_));
    stopStream();
    // No leg of our own from here on: the autopilot flies the descent.
    publishNoTrajectory(
      Trajectory3D::PLAN_STATE_NO_GOAL, "landing: the autopilot flies the descent");

    const bool disarmed = waitForDisarm(goal_handle, reference_z);
    if (!disarmed) {
      RCLCPP_WARN(get_logger(), "no automatic disarm, commanding disarm");
      std::string detail;
      if (!callService(disarm_client_, std::make_shared<Disarm::Request>(), "disarm", detail)) {
        result->success = false;
        result->result_code = ResultCode::ABORTED_VEHICLE_REJECTED;
        result->message = detail;
        setState(NavigatorState::IDLE);
        goal_handle->abort(result);
        return;
      }
    }

    result->success = true;
    result->result_code = ResultCode::SUCCEEDED;
    result->message = disarmed ? "landed and disarmed" : "landed, disarm commanded";
    setState(NavigatorState::IDLE);
    goal_handle->succeed(result);
  }

  bool waitForDisarm(const std::shared_ptr<GoalHandleLand> & goal_handle, double reference_z)
  {
    auto feedback = std::make_shared<Land::Feedback>();
    feedback->marker_visible = false;

    const rclcpp::Time start = now();
    rclcpp::Time last_feedback = start;
    rclcpp::Time last_sample = start;
    double last_altitude = poseSample().position.z;
    const double feedback_period = 1.0 / feedback_hz_;

    while (rclcpp::ok() && !stopping_) {
      if (!armed_.load()) {
        return true;
      }
      const rclcpp::Time stamp = now();
      if ((stamp - last_feedback).seconds() >= feedback_period) {
        const double altitude = poseSample().position.z;
        const double elapsed = (stamp - last_sample).seconds();
        feedback->current_altitude = static_cast<float>(altitude - reference_z);
        feedback->descent_rate =
          elapsed > 0.0 ? static_cast<float>((last_altitude - altitude) / elapsed) : 0.0F;
        goal_handle->publish_feedback(feedback);
        last_feedback = stamp;
        last_sample = stamp;
        last_altitude = altitude;
      }
      if ((stamp - start).seconds() > disarm_timeout_) {
        return false;
      }
      if (!sleepOneTick()) {
        return !armed_.load();
      }
    }
    return !armed_.load();
  }

  // -------------------------------------------------------------------- state

  std::string uav_id_;
  std::string odom_frame_;
  double stream_hz_ = 20.0;
  MotionLimits limits_;
  double acceptance_sigma_factor_ = 2.0;
  double max_acceptance_radius_ = 1.5;
  double max_lead_horizontal_ = 0.8;
  double max_lead_vertical_ = 0.6;
  double leash_stall_fault_ = 3.0;
  double takeoff_stall_grace_ = 5.0;
  double command_loss_debounce_ = 0.5;
  double max_yaw_rate_ = 0.5;
  double max_acceleration_ = 1.0;
  double sample_hz_ = 50.0;
  bool use_trajectory_ = true;
  bool use_route_ = true;
  double route_timeout_ = 2.0;
  bool require_obstacle_feed_ = false;
  bool use_avoidance_ = true;
  double advice_timeout_ = 1.0;
  double max_escape_deviation_ = 8.0;
  double escape_replan_interval_ = 1.0;
  double escape_refresh_ = 0.5;
  double route_fresh_ = 1.0;
  double carrot_plan_period_ = 0.5;
  double avoidance_hold_timeout_ = 12.0;
  double min_standoff_ = 1.0;
  double target_velocity_error_ = 0.71;
  double target_reaction_ = 1.0;
  double target_lead_ = 0.0;
  double target_state_timeout_ = 1.0;
  double target_sighting_timeout_ = 1.0;
  double target_sighting_period_copy_ = 1.0 / 15.0;
  double tracker_lost_after_copy_ = 3.0;
  double feedback_hz_ = 2.0;
  double progress_hz_ = 10.0;
  double arrival_settle_ = 1.0;
  double odometry_timeout_ = 1.0;
  double startup_timeout_ = 30.0;
  double offboard_timeout_ = 30.0;
  double service_timeout_ = 15.0;
  double takeoff_timeout_ = 60.0;
  double goto_timeout_ = 120.0;
  double hold_timeout_margin_ = 30.0;
  double land_stream_stop_delay_ = 1.0;
  double disarm_timeout_ = 40.0;

  std::mutex motion_mutex_;
  bool stream_active_ = false;
  Vec3 carrot_;
  Vec3 target_;
  double commanded_yaw_ = 0.0;
  double target_yaw_ = 0.0;
  CarrotLimits active_limits_;
  Trajectory trajectory_;
  bool trajectory_active_ = false;
  bool motion_paused_ = false;
  double trajectory_time_ = 0.0;
  // The clamp verdict only exists after advanceCarrot, so the next tick reads it.
  bool previous_tick_clamped_ = false;
  double yaw_handover_margin_ = 0.0;
  bool yaw_handover_engaged_ = false;
  std::optional<double> requested_goal_yaw_;
  Vec3 position_;
  double measured_yaw_ = 0.0;
  rclcpp::Time odometry_stamp_{0, 0, RCL_ROS_TIME};
  bool odometry_valid_ = false;
  rclcpp::Time last_stream_time_{0, 0, RCL_ROS_TIME};

  // Guarded by motion_mutex_: the stream timer counts, a starting task clears.
  rclcpp::Time leash_clamp_started_{0, 0, RCL_ROS_TIME};

  // Touched only by the task thread.
  rclcpp::Time not_commanding_since_{0, 0, RCL_ROS_TIME};
  std::string acceptance_note_;
  std::string evidence_note_;
  std::string planner_fault_;
  uint8_t planner_fault_code_ = ResultCode::ABORTED_PLANNER_FAILED;
  bool route_rejoin_noted_ = false;
  bool target_lead_refused_ = false;
  rclcpp::Time advice_permission_{0, 0, RCL_ROS_TIME};
  rclcpp::Time advice_handled_{0, 0, RCL_ROS_TIME};
  rclcpp::Time plan_installed_{0, 0, RCL_ROS_TIME};
  rclcpp::Time escape_replanned_{0, 0, RCL_ROS_TIME};
  std::optional<Vec3> active_escape_;
  bool avoidance_holding_ = false;
  bool avoidance_unguarded_noted_ = false;
  bool avoidance_descent_noted_ = false;
  int avoidance_holds_ = 0;
  int avoidance_escapes_ = 0;
  CarrotPlan carrot_plan_kind_ = CarrotPlan::NONE;
  Vec3 carrot_plan_from_;
  Vec3 carrot_plan_to_;
  rclcpp::Time carrot_plan_stamp_{0, 0, RCL_ROS_TIME};
  Vec3 home_;
  bool home_known_ = false;
  bool holding_without_localization_ = false;
  rclcpp::Time avoidance_hold_started_{0, 0, RCL_ROS_TIME};
  RecoveryPlanner recovery_;

  std::mutex admission_mutex_;
  std::atomic<NavigatorState> state_{NavigatorState::IDLE};

  // Atomic because they cross the callback group boundary into the worker thread.
  std::atomic<uint8_t> offboard_state_{kOffboardUnknown};
  std::atomic<uint8_t> flight_mode_{VehicleState::FLIGHT_MODE_UNKNOWN};
  std::atomic<bool> connected_{false};
  std::atomic<bool> armed_{false};
  std::atomic<bool> failsafe_active_{false};
  std::atomic<bool> localization_seen_{false};
  std::atomic<bool> localization_valid_{true};
  std::atomic<double> position_uncertainty_{-1.0};
  std::atomic<double> ground_reference_z_{0.0};
  std::atomic<bool> stopping_{false};
  // Read by whatever task is flying, written when a recovery is accepted.
  std::atomic<bool> preempt_requested_{false};
  std::atomic<bool> recovery_pending_{false};

  // Leash telemetry: written by the stream timer, read by the task thread.
  std::atomic<double> leash_stall_seconds_{0.0};
  std::atomic<double> stall_watch_from_{std::numeric_limits<double>::max()};
  std::atomic<double> leash_lead_horizontal_{0.0};
  std::atomic<double> leash_lead_vertical_{0.0};
  std::atomic<uint64_t> stream_ticks_{0};
  std::atomic<uint64_t> clamped_ticks_{0};

  std::mutex worker_mutex_;
  std::thread worker_;

  rclcpp::CallbackGroup::SharedPtr stream_group_;
  rclcpp::CallbackGroup::SharedPtr action_group_;

  rclcpp::Publisher<ControlCommand>::SharedPtr command_publisher_;
  rclcpp::Publisher<Trajectory3D>::SharedPtr trajectory_publisher_;
  uint32_t plan_sequence_ = 0;
  rclcpp::Publisher<PoseStamped>::SharedPtr route_goal_publisher_;
  rclcpp::Subscription<Path3D>::SharedPtr route_subscription_;
  rclcpp::Subscription<AvoidanceAdvice>::SharedPtr advice_subscription_;
  rclcpp::Subscription<TargetState>::SharedPtr target_subscription_;
  rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<OffboardStatus>::SharedPtr offboard_subscription_;
  rclcpp::Subscription<LocalizationStatus>::SharedPtr localization_subscription_;
  rclcpp::Subscription<VehicleState>::SharedPtr vehicle_subscription_;

  std::mutex route_mutex_;
  Path3D::SharedPtr route_;
  rclcpp::Time route_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time route_question_stamp_{0, 0, RCL_ROS_TIME};
  std::mutex advice_mutex_;
  AvoidanceAdvice::SharedPtr advice_;
  std::mutex target_mutex_;
  TargetState::SharedPtr target_state_;
  rclcpp::Client<Arm>::SharedPtr arm_client_;
  rclcpp::Client<Disarm>::SharedPtr disarm_client_;
  rclcpp::Client<SetFlightMode>::SharedPtr mode_client_;
  rclcpp_action::Server<Takeoff>::SharedPtr takeoff_server_;
  rclcpp_action::Server<GotoPose>::SharedPtr goto_server_;
  rclcpp_action::Server<HoldPosition>::SharedPtr hold_server_;
  rclcpp_action::Server<Land>::SharedPtr land_server_;
  rclcpp_action::Server<FollowPath>::SharedPtr follow_server_;
  rclcpp_action::Server<TrackTarget>::SharedPtr track_server_;
  rclcpp_action::Server<Recover>::SharedPtr recover_server_;
  rclcpp::TimerBase::SharedPtr stream_timer_;
};

}  // namespace

std::shared_ptr<rclcpp::Node> createNavigatorActionServerNode(
  const rclcpp::NodeOptions & options)
{
  return std::make_shared<NavigatorActionServerNode>(options);
}

}  // namespace uav_navigation
