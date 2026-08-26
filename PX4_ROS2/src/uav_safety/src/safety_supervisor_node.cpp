#include "uav_safety/safety_supervisor_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <uav_interfaces/msg/result_code.hpp>

namespace uav_safety
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using nav_msgs::msg::Odometry;
using uav_interfaces::msg::ControlCommand;
using uav_interfaces::msg::LocalizationStatus;
using uav_interfaces::msg::ObstacleArray;
using uav_interfaces::msg::OffboardStatus;
using uav_interfaces::msg::SafetyState;
using uav_interfaces::msg::Trajectory3D;
using uav_interfaces::msg::VehicleHealth;
using uav_interfaces::msg::VehicleState;
using uav_interfaces::srv::ClearFault;
using uav_interfaces::srv::SetControlAuthority;

namespace
{

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// P8.1 left this to the caller (report point 7): how long since the last
// cmd_mission arrival still counts as "live" for ClearFault's handover-jump
// gate. Reuses the gateway's own stale-command cutoff (px4_command_gateway_
// node.cpp:58) as the most defensible existing number, rather than inventing
// a new one -- flagged in the P8.3 handoff report.
constexpr double kCmdMissionLiveWindowSec = 0.5;

// P8.5: guards against streaming a garbage frozen pose (R27-2) at the exact
// moment HOLD freezes it. Squared-norm floor, same convention as
// uav_navigation's carrot.hpp hasUsableOrientation() -- reimplemented
// locally (a few lines) rather than linking uav_navigation, which P8's plan
// forbids touching/depending on.
constexpr double kMinOrientationNormSq = 1e-6;

/// atan2 yaw extraction -- same formula as uav_navigation's
/// yawFromQuaternion (carrot.cpp:117), reimplemented locally for the same
/// reason as kMinOrientationNormSq above. Returns false (never writes
/// *yaw_out) on a non-finite or near-zero-norm quaternion.
bool tryYawFromQuaternion(double x, double y, double z, double w, double * yaw_out)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(w)) {
    return false;
  }
  if (x * x + y * y + z * z + w * w < kMinOrientationNormSq) {
    return false;
  }
  *yaw_out = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  return true;
}

KeyValue keyValue(const std::string & key, double value)
{
  KeyValue pair;
  pair.key = key;
  pair.value = std::to_string(value);
  return pair;
}

KeyValue keyValueStr(const std::string & key, const std::string & value)
{
  KeyValue pair;
  pair.key = key;
  pair.value = value;
  return pair;
}

/// P8.4/P8.5: both INHIBIT and HOLD are real enforcement when
/// enforcement_enabled; both stay dry-run-labelled otherwise -- this
/// function must never claim either happened for real when it did not.
std::string describeAction(FailsafeAction action, bool enforcement_enabled)
{
  switch (action) {
    case FailsafeAction::kInhibit:
      return enforcement_enabled ? "inhibit" : "would_inhibit (dry-run, enforcement_enabled=false)";
    case FailsafeAction::kHold:
      return enforcement_enabled ? "hold" : "would_hold (dry-run, enforcement_enabled=false)";
    case FailsafeAction::kReport:
      return "report";
    default:
      return "none";
  }
}

const char * stateName(SupervisorState state)
{
  switch (state) {
    case SupervisorState::kNormal: return "NORMAL";
    case SupervisorState::kEngagingHold: return "ENGAGING_HOLD";
    case SupervisorState::kHolding: return "HOLDING";
    case SupervisorState::kEngagingInhibit: return "ENGAGING_INHIBIT";
    case SupervisorState::kInhibited: return "INHIBITED";
    case SupervisorState::kClearing: return "CLEARING";
    default: return "UNKNOWN";
  }
}

const char * planStateName(uint8_t plan_state)
{
  switch (plan_state) {
    case Trajectory3D::PLAN_STATE_VALID: return "VALID";
    case Trajectory3D::PLAN_STATE_NO_GOAL: return "NO_GOAL";
    case Trajectory3D::PLAN_STATE_WITHDRAWN: return "WITHDRAWN";
    case Trajectory3D::PLAN_STATE_FAILED: return "FAILED";
    default: return "UNKNOWN";
  }
}

/// SafetyState::LEVEL_* (this node's own overall_level) -> diagnostic_msgs
/// level. DiagnosticStatus has no EMERGENCY value; EMERGENCY maps to its
/// highest, ERROR.
uint8_t levelToDiagnostic(uint8_t safety_level)
{
  switch (safety_level) {
    case SafetyState::LEVEL_WARNING: return DiagnosticStatus::WARN;
    case SafetyState::LEVEL_ERROR:
    case SafetyState::LEVEL_EMERGENCY:
      return DiagnosticStatus::ERROR;
    case SafetyState::LEVEL_OK: return DiagnosticStatus::OK;
    default: return DiagnosticStatus::STALE;
  }
}

uint8_t violationStatusToDiagnostic(ViolationStatus status, uint8_t severity)
{
  switch (status) {
    case ViolationStatus::kOk: return DiagnosticStatus::OK;
    case ViolationStatus::kCannotMeasure: return DiagnosticStatus::WARN;
    case ViolationStatus::kViolating: return levelToDiagnostic(severity);
    default: return DiagnosticStatus::STALE;
  }
}

double parseCountKey(const DiagnosticStatus & status, const std::string & key)
{
  for (const KeyValue & kv : status.values) {
    if (kv.key == key) {
      try {
        return std::stod(kv.value);
      } catch (const std::exception &) {
        return kNan;
      }
    }
  }
  return kNan;
}

/// B1/B4: reverse of authority_arbiter's sourceName() -- reimplemented
/// locally (uav_safety cannot link uav_control_authority).
uint8_t sourceFromName(const std::string & name)
{
  if (name == "TEST") {return kSourceTest;}
  if (name == "MISSION") {return kSourceMission;}
  if (name == "OPERATOR") {return kSourceOperator;}
  if (name == "SAFETY") {return kSourceSafety;}
  return kSourceNone;
}

/// B1: control_authority_manager_node's per-channel key naming
/// ("dropped_wrong_frame_<channel>"); "" for a channel with no such key
/// (kSourceNone, e.g. nobody currently holds authority).
std::string perChannelWrongFrameKey(uint8_t channel)
{
  switch (channel) {
    case kSourceTest: return "dropped_wrong_frame_test";
    case kSourceMission: return "dropped_wrong_frame_mission";
    case kSourceOperator: return "dropped_wrong_frame_operator";
    case kSourceSafety: return "dropped_wrong_frame_safety";
    default: return "";
  }
}

}  // namespace

SafetySupervisorNode::SafetySupervisorNode(const rclcpp::NodeOptions & options)
try
: Node("safety_supervisor_node", options),
  uav_id_(declare_parameter<std::string>("uav_id", "uav0")),
  prefix_("/uav/" + uav_id_),
  odom_frame_(declare_parameter<std::string>("odom_frame", "odom")),
  enforcement_enabled_(declare_parameter<bool>("enforcement_enabled", false)),
  params_(declareParams()),
  policy_(params_)
{
  // R24: ONE group for every sub, the 20 Hz tick timer, AND the service --
  // see header point 2.
  io_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = io_group_;

  localization_status_subscription_ = create_subscription<LocalizationStatus>(
    prefix_ + "/state/localization_status", rclcpp::QoS(10),
    [this](const LocalizationStatus::SharedPtr msg) {
      localization_status_ = *msg;
      localization_status_received_ = true;
      localization_status_arrival_ = now();
    }, sub_options);

  odometry_fused_subscription_ = create_subscription<Odometry>(
    prefix_ + "/state/odometry_fused", rclcpp::QoS(10),
    [this](const Odometry::SharedPtr msg) {
      odometry_fused_ = *msg;
      odometry_fused_received_ = true;
      odometry_fused_arrival_ = now();
    }, sub_options);

  // P8.3 handoff report point 1: jumps ONLY lives in the detailed
  // /diagnostics/localization array (see header field comment) -- corrected
  // from the /state/localization_health summary named in the original task.
  localization_diagnostics_subscription_ = create_subscription<DiagnosticArray>(
    prefix_ + "/diagnostics/localization", rclcpp::QoS(10),
    [this](const DiagnosticArray::SharedPtr msg) {
      for (const DiagnosticStatus & status : msg->status) {
        if (status.name == "localization: continuity") {
          localization_jump_count_ = parseCountKey(status, "jumps");
          localization_diagnostics_received_ = true;
          localization_diagnostics_arrival_ = now();   // R32
          break;
        }
      }
    }, sub_options);

  obstacle_map_subscription_ = create_subscription<ObstacleArray>(
    prefix_ + "/world/obstacle_map_local", rclcpp::QoS(10),
    [this](const ObstacleArray::SharedPtr msg) {
      obstacle_map_ = *msg;
      obstacle_map_received_ = true;
      obstacle_map_arrival_ = now();
    }, sub_options);

  vehicle_health_subscription_ = create_subscription<VehicleHealth>(
    prefix_ + "/state/health_px4", rclcpp::QoS(10),
    [this](const VehicleHealth::SharedPtr msg) {
      vehicle_health_ = *msg;
      vehicle_health_received_ = true;
      vehicle_health_arrival_ = now();   // B2
    }, sub_options);

  vehicle_state_subscription_ = create_subscription<VehicleState>(
    prefix_ + "/state/vehicle", rclcpp::QoS(10),
    [this](const VehicleState::SharedPtr msg) {
      vehicle_state_ = *msg;
      vehicle_state_received_ = true;
      vehicle_state_arrival_ = now();   // B2
    }, sub_options);

  offboard_status_subscription_ = create_subscription<OffboardStatus>(
    prefix_ + "/backend/offboard_status", rclcpp::QoS(10),
    [this](const OffboardStatus::SharedPtr msg) {
      offboard_status_ = *msg;
      offboard_status_received_ = true;
      offboard_status_arrival_ = now();   // B2/Y1
    }, sub_options);

  command_selected_subscription_ = create_subscription<ControlCommand>(
    prefix_ + "/control/command_selected", rclcpp::QoS(10),
    [this](const ControlCommand::SharedPtr msg) {
      command_selected_ = *msg;
      command_selected_received_ = true;
      command_selected_arrival_ = now();
    }, sub_options);

  cmd_mission_subscription_ = create_subscription<ControlCommand>(
    prefix_ + "/control/cmd_mission", rclcpp::QoS(10),
    [this](const ControlCommand::SharedPtr msg) {
      cmd_mission_ = *msg;
      cmd_mission_received_ = true;
      cmd_mission_arrival_ = now();
    }, sub_options);

  control_authority_diagnostics_subscription_ = create_subscription<DiagnosticArray>(
    prefix_ + "/diagnostics/control_authority", rclcpp::QoS(10),
    [this](const DiagnosticArray::SharedPtr msg) {
      authority_diagnostics_received_ = true;
      authority_diagnostics_arrival_ = now();   // R32
      bool latch_active = false;
      uint8_t latch_level = kSourceNone;
      uint8_t active_source = kSourceNone;
      const DiagnosticStatus * wrong_frame_status = nullptr;

      for (const DiagnosticStatus & status : msg->status) {
        if (status.name == "control_authority: arbitration") {
          for (const KeyValue & kv : status.values) {
            if (kv.key == "latch_active") {
              latch_active = (kv.value == "true");
            } else if (kv.key == "latch_level") {
              latch_level = sourceFromName(kv.value);
            } else if (kv.key == "active_source") {
              active_source = sourceFromName(kv.value);
            }
          }
        } else if (status.name == "control_authority: dropped wrong_frame") {
          wrong_frame_status = &status;
        }
      }

      // B4 + Y2: the arbiter's OWN diagnostics are the truth on the SAFETY
      // latch -- adopt them BOTH ways. Down (B4): latch cleared behind our
      // back -> onTick()'s "not yet granted" branch re-requests. Up (Y2):
      // THIS node restarted while the arbiter still held the latch (R5: it
      // never self-expires) -> ClearFault must release the orphan, not
      // answer "no fault latched" while command_selected stays cut forever.
      safety_latch_confirmed_granted_ = latch_active && latch_level == kSourceSafety;

      // B1 (real bug): read ONLY the currently HELD channel's own
      // per-channel wrong_frame count -- never the aggregate, which used to
      // let one stray wrong-frame message on an UNRELATED channel latch
      // FRAME_MISMATCH forever. Held = latch level if the latch is active
      // (it raises the floor regardless of who is momentarily streaming),
      // else whoever is the live active_source.
      const uint8_t held_channel = latch_active ? latch_level : active_source;
      if (wrong_frame_status != nullptr) {
        const std::string key = perChannelWrongFrameKey(held_channel);
        const double count = key.empty() ? kNan : parseCountKey(*wrong_frame_status, key);
        if (held_channel != previous_held_channel_ ||
          !std::isfinite(count) || !std::isfinite(previous_wrong_frame_drop_count_))
        {
          // New/changed held channel, or a genuinely unmeasurable reading:
          // establish a fresh baseline only, never claim "increasing" --
          // comparing across a channel CHANGE would compare apples to oranges.
          wrong_frame_drop_increasing_ = false;
        } else {
          wrong_frame_drop_increasing_ = count > previous_wrong_frame_drop_count_;
        }
        previous_wrong_frame_drop_count_ = count;
        previous_held_channel_ = held_channel;
      }
    }, sub_options);

  camera_health_subscription_ = create_subscription<DiagnosticStatus>(
    prefix_ + "/state/camera_health", rclcpp::QoS(10),
    [this](const DiagnosticStatus::SharedPtr msg) {
      camera_health_ = *msg;
      camera_health_received_ = true;
      camera_health_arrival_ = now();   // B2/Y1
    }, sub_options);

  // Publisher is QoS(1).reliable().transient_local() (navigator_action_
  // server_node.cpp:329) -- must match for this late-joining subscriber to
  // ever see the latched last plan.
  trajectory_subscription_ = create_subscription<Trajectory3D>(
    prefix_ + "/planning/trajectory", rclcpp::QoS(1).reliable().transient_local(),
    [this](const Trajectory3D::SharedPtr msg) {
      last_trajectory_ = *msg;
      trajectory_received_ = true;
    }, sub_options);

  safety_state_publisher_ = create_publisher<SafetyState>(
    prefix_ + "/safety/state", rclcpp::QoS(1).reliable().transient_local());
  violations_publisher_ = create_publisher<DiagnosticArray>(
    prefix_ + "/safety/violations", rclcpp::QoS(20));
  diagnostics_publisher_ = create_publisher<DiagnosticArray>(
    prefix_ + "/diagnostics/safety", rclcpp::QoS(10));

  // Structural, not a runtime branch elsewhere: with enforcement_enabled
  // false, NONE of these four are ever constructed -- `ros2 node info` shows
  // zero publishers on cmd_safety and zero service clients, a stronger proof
  // than any flag check (P8.3 brief, extended to the two P8.4 clients).
  if (enforcement_enabled_) {
    cmd_safety_publisher_ = create_publisher<ControlCommand>(
      prefix_ + "/control/cmd_safety", rclcpp::QoS(10));
    set_authority_client_ = create_client<SetControlAuthority>(
      prefix_ + "/control/set_authority", rmw_qos_profile_services_default, io_group_);
    clear_safety_latch_client_ = create_client<ClearFault>(
      prefix_ + "/control/clear_safety_latch", rmw_qos_profile_services_default, io_group_);

    // P8.5, header point 6: same io_group_ as tick_timer_ (R24) -- reads
    // params_.hold_stream_hz, not a literal, so the actual publish rate can
    // never silently drift from the value validate() checked (G6 discipline:
    // never let two constants encode one number).
    hold_stream_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / params_.hold_stream_hz),
      [this]() {onHoldStreamTick();}, io_group_);
  }

  // Same group as everything else touching policy_ (R24, header point 2).
  // Deferred-response overload (header point 5): rclcpp does NOT auto-send a
  // response here, respondClearFault()/finishClearFault() do that explicitly.
  clear_fault_service_ = create_service<ClearFault>(
    prefix_ + "/safety/clear_fault",
    [this](
      const std::shared_ptr<rmw_request_id_t> header,
      const std::shared_ptr<ClearFault::Request> request) {
      handleClearFault(header, request);
    },
    rmw_qos_profile_services_default, io_group_);

  // Not a wall timer (nợ #1): must track sim time.
  tick_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / 20.0),
    [this]() {onTick();}, io_group_);

  RCLCPP_INFO(
    get_logger(), "safety supervisor ready for %s (enforcement_enabled=%s)",
    uav_id_.c_str(), enforcement_enabled_ ? "true" : "false");
}
catch (const std::invalid_argument & error)
{
  RCLCPP_FATAL(
    rclcpp::get_logger("safety_supervisor_node"), "refusing to start: %s", error.what());
  throw;
}

FailsafePolicyParams SafetySupervisorNode::declareParams()
{
  FailsafePolicyParams params;
  params.odom_frame = odom_frame_;

  params.status_timeout_sec =
    declare_parameter<double>("status_timeout_sec", params.status_timeout_sec);
  params.fused_timeout_sec =
    declare_parameter<double>("fused_timeout_sec", params.fused_timeout_sec);
  params.localization_invalid_grace_sec = declare_parameter<double>(
    "localization_invalid_grace_sec", params.localization_invalid_grace_sec);
  params.localization_stale_grace_sec = declare_parameter<double>(
    "localization_stale_grace_sec", params.localization_stale_grace_sec);
  params.localization_jump_grace_sec = declare_parameter<double>(
    "localization_jump_grace_sec", params.localization_jump_grace_sec);
  params.blind_grace_sec = declare_parameter<double>("blind_grace_sec", params.blind_grace_sec);
  params.frozen_setpoint_epsilon_copy_m = declare_parameter<double>(
    "frozen_setpoint_epsilon_copy_m", params.frozen_setpoint_epsilon_copy_m);
  params.battery_warn_threshold =
    declare_parameter<double>("battery_warn_threshold", params.battery_warn_threshold);
  params.battery_critical_threshold =
    declare_parameter<double>("battery_critical_threshold", params.battery_critical_threshold);
  params.battery_warn_grace_sec =
    declare_parameter<double>("battery_warn_grace_sec", params.battery_warn_grace_sec);
  params.battery_critical_grace_sec =
    declare_parameter<double>("battery_critical_grace_sec", params.battery_critical_grace_sec);
  params.px4_action_grace_sec =
    declare_parameter<double>("px4_action_grace_sec", params.px4_action_grace_sec);
  params.min_obstacle_distance_copy_m =
    declare_parameter<double>("min_obstacle_distance_copy_m", params.min_obstacle_distance_copy_m);
  params.obstacle_grace_sec =
    declare_parameter<double>("obstacle_grace_sec", params.obstacle_grace_sec);
  params.obstacle_clear_margin_m =
    declare_parameter<double>("obstacle_clear_margin_m", params.obstacle_clear_margin_m);
  params.obstacle_map_timeout_copy_sec =
    declare_parameter<double>("obstacle_map_timeout_copy_sec", params.obstacle_map_timeout_copy_sec);
  params.offboard_min_rate_hz =
    declare_parameter<double>("offboard_min_rate_hz", params.offboard_min_rate_hz);
  params.offboard_grace_sec =
    declare_parameter<double>("offboard_grace_sec", params.offboard_grace_sec);
  params.selected_stale_sec =
    declare_parameter<double>("selected_stale_sec", params.selected_stale_sec);
  params.downstream_command_timeout_copy_sec = declare_parameter<double>(
    "downstream_command_timeout_copy_sec", params.downstream_command_timeout_copy_sec);
  params.estimator_grace_sec =
    declare_parameter<double>("estimator_grace_sec", params.estimator_grace_sec);
  params.camera_grace_sec = declare_parameter<double>("camera_grace_sec", params.camera_grace_sec);
  params.frame_mismatch_grace_sec =
    declare_parameter<double>("frame_mismatch_grace_sec", params.frame_mismatch_grace_sec);
  params.clear_stability_sec =
    declare_parameter<double>("clear_stability_sec", params.clear_stability_sec);
  params.battery_clear_margin =
    declare_parameter<double>("battery_clear_margin", params.battery_clear_margin);
  params.localization_clear_margin_sec = declare_parameter<double>(
    "localization_clear_margin_sec", params.localization_clear_margin_sec);
  params.handover_jump_limit_m =
    declare_parameter<double>("handover_jump_limit_m", params.handover_jump_limit_m);
  params.pose_max_age_sec =
    declare_parameter<double>("pose_max_age_sec", params.pose_max_age_sec);
  params.hold_stream_hz = declare_parameter<double>("hold_stream_hz", params.hold_stream_hz);

  params.odometry_timeout_copy_sec =
    declare_parameter<double>("odometry_timeout_copy_sec", params.odometry_timeout_copy_sec);
  params.progress_hz_copy = declare_parameter<double>("progress_hz_copy", params.progress_hz_copy);
  params.arbiter_source_timeout_copy_sec = declare_parameter<double>(
    "arbiter_source_timeout_copy_sec", params.arbiter_source_timeout_copy_sec);
  params.max_lead_copy_m =
    declare_parameter<double>("max_lead_copy_m", params.max_lead_copy_m);
  params.mux_publish_period_copy_sec = declare_parameter<double>(
    "mux_publish_period_copy_sec", params.mux_publish_period_copy_sec);   // Y8

  // B2/Y1: freshness ceilings for the 4 previously-unaged subscriptions.
  params.vehicle_health_timeout_sec = declare_parameter<double>(
    "vehicle_health_timeout_sec", params.vehicle_health_timeout_sec);
  params.vehicle_state_timeout_sec = declare_parameter<double>(
    "vehicle_state_timeout_sec", params.vehicle_state_timeout_sec);
  params.offboard_status_timeout_sec = declare_parameter<double>(
    "offboard_status_timeout_sec", params.offboard_status_timeout_sec);
  params.camera_health_timeout_sec = declare_parameter<double>(
    "camera_health_timeout_sec", params.camera_health_timeout_sec);

  // B3: real PX4 battery threshold copies.
  params.bat_low_thr_copy =
    declare_parameter<double>("bat_low_thr_copy", params.bat_low_thr_copy);
  params.bat_crit_thr_copy =
    declare_parameter<double>("bat_crit_thr_copy", params.bat_crit_thr_copy);
  params.bat_emergen_thr_copy =
    declare_parameter<double>("bat_emergen_thr_copy", params.bat_emergen_thr_copy);

  // B1: arbiter's own diagnostics cadence.
  params.arbiter_diagnostics_period_copy_sec = declare_parameter<double>(
    "arbiter_diagnostics_period_copy_sec", params.arbiter_diagnostics_period_copy_sec);
  // R32: localization_health_node's diagnostics cadence.
  params.localization_diag_period_copy_sec = declare_parameter<double>(
    "localization_diag_period_copy_sec", params.localization_diag_period_copy_sec);
  // Y2: world_model's obstacle map publish rate.
  params.world_model_publish_hz_copy = declare_parameter<double>(
    "world_model_publish_hz_copy", params.world_model_publish_hz_copy);

  // S:9d / config/safety_params.yaml footer: every *_copy printed loudly at
  // startup, same discipline as control_authority_manager_node's WARN for
  // downstream_command_timeout_sec. Y3: both numbers of every PAIRED
  // validate() rule appear in the SAME line, not just the *_copy side.
  RCLCPP_WARN(
    get_logger(),
    "blind_grace_sec=%.3f paired with odometry_timeout_copy_sec=%.3f + 2/progress_hz_copy=%.3f "
    "(must not race navigator's own layer-1 pose-staleness reaction)",
    params.blind_grace_sec, params.odometry_timeout_copy_sec, 2.0 / params.progress_hz_copy);
  RCLCPP_WARN(
    get_logger(),
    "fused_timeout_sec=%.3f paired with odometry_timeout_copy_sec=%.3f "
    "(safety must know before navigator does)",
    params.fused_timeout_sec, params.odometry_timeout_copy_sec);
  RCLCPP_WARN(
    get_logger(),
    "hold_stream_hz=%.3f paired with arbiter_source_timeout_copy_sec=%.3f "
    "(else the arbiter would see HOLD's own stream as not-live)",
    params.hold_stream_hz, params.arbiter_source_timeout_copy_sec);
  RCLCPP_WARN(
    get_logger(), "handover_jump_limit_m=%.3f paired with max_lead_copy_m=%.3f",
    params.handover_jump_limit_m, params.max_lead_copy_m);
  RCLCPP_WARN(
    get_logger(),
    "selected_stale_sec=%.3f paired with downstream_command_timeout_copy_sec=%.3f (P8.4b)",
    params.selected_stale_sec, params.downstream_command_timeout_copy_sec);
  RCLCPP_WARN(
    get_logger(),
    "frame_mismatch_grace_sec=%.3f paired with 2 x arbiter_diagnostics_period_copy_sec=%.3f (B1)",
    params.frame_mismatch_grace_sec, 2.0 * params.arbiter_diagnostics_period_copy_sec);
  RCLCPP_WARN(
    get_logger(),
    "obstacle_map_timeout_copy_sec=%.3f paired with 2/world_model_publish_hz_copy=%.3f (Y2)",
    params.obstacle_map_timeout_copy_sec, 2.0 / params.world_model_publish_hz_copy);
  RCLCPP_WARN(
    get_logger(),
    "pose_max_age_sec=%.3f paired with 2 x mux_publish_period_copy_sec=%.3f (Y8)",
    params.pose_max_age_sec, 2.0 * params.mux_publish_period_copy_sec);
  RCLCPP_WARN(
    get_logger(),
    "sample-and-hold expiry (R32): wrong-frame flag after 2 x arbiter_diagnostics_period_copy_sec"
    "=%.3f, jumps count after 2 x localization_diag_period_copy_sec=%.3f",
    2.0 * params.arbiter_diagnostics_period_copy_sec,
    2.0 * params.localization_diag_period_copy_sec);
  RCLCPP_WARN(
    get_logger(),
    "bat_crit_thr_copy=%.3f (real PX4 threshold, B3) vs battery_critical_threshold=%.3f "
    "(this package's own, REPORT-only) -- BATTERY_PX4_NO_ACTION gates on the FORMER, not the latter",
    params.bat_crit_thr_copy, params.battery_critical_threshold);
  RCLCPP_WARN(
    get_logger(),
    "enforcement_enabled=%s -- P8.3 ships false; P8.4 flips it",
    enforcement_enabled_ ? "true" : "false");

  return params;
}

double SafetySupervisorNode::ageSec(
  const rclcpp::Time & now_stamp, const rclcpp::Time & last_arrival, bool received) const
{
  if (!received) {
    return kInf;
  }
  return (now_stamp - last_arrival).seconds();
}

Measurements SafetySupervisorNode::buildMeasurements(const rclcpp::Time & now_stamp) const
{
  Measurements m;

  m.localization_status_received = localization_status_received_;
  m.localization_status_age_sec =
    ageSec(now_stamp, localization_status_arrival_, localization_status_received_);
  m.localization_is_valid = localization_status_received_ && localization_status_.is_valid;

  m.fused_odometry_age_sec = ageSec(now_stamp, odometry_fused_arrival_, odometry_fused_received_);

  // R32 (N3): a jumps reading held from a DEAD /diagnostics/localization
  // stream must expire to CANNOT_MEASURE, never stay trustworthy forever.
  const bool localization_diag_fresh = localization_diagnostics_received_ &&
    ageSec(now_stamp, localization_diagnostics_arrival_, localization_diagnostics_received_) <=
    2.0 * params_.localization_diag_period_copy_sec;
  m.localization_jump_count = localization_diag_fresh ? localization_jump_count_ : kNan;

  // Window displacement (S:0b R1): tick-sampled in onTick(), read here.
  bool window_full = command_window_.size() >= kWindowTicks;
  double displacement = kNan;
  if (window_full) {
    const PositionSample & newest = command_window_.back();
    const PositionSample & oldest = command_window_.front();
    if (newest.valid && oldest.valid) {
      displacement = std::hypot(
        std::hypot(newest.x - oldest.x, newest.y - oldest.y), newest.z - oldest.z);
    } else {
      window_full = false;    // not enough trustworthy samples yet
    }
  }
  m.command_setpoint_displacement_m = displacement;
  m.command_setpoint_window_full = window_full;
  m.command_selected_source = command_selected_received_ ? command_selected_.source : kSourceNone;

  // B2/Y1 (review 2026-08-21): a "received" flag alone let a dead publisher
  // leave the last GOOD reading frozen forever, read as still-trustworthy.
  // Fresh = received AND within its own timeout of the last ARRIVAL.
  const bool vehicle_health_fresh = vehicle_health_received_ &&
    ageSec(now_stamp, vehicle_health_arrival_, vehicle_health_received_) <=
      params_.vehicle_health_timeout_sec;
  const bool vehicle_state_fresh = vehicle_state_received_ &&
    ageSec(now_stamp, vehicle_state_arrival_, vehicle_state_received_) <=
      params_.vehicle_state_timeout_sec;
  const bool offboard_status_fresh = offboard_status_received_ &&
    ageSec(now_stamp, offboard_status_arrival_, offboard_status_received_) <=
      params_.offboard_status_timeout_sec;
  const bool camera_health_fresh = camera_health_received_ &&
    ageSec(now_stamp, camera_health_arrival_, camera_health_received_) <=
      params_.camera_health_timeout_sec;

  m.battery_remaining =
    vehicle_health_fresh ? static_cast<double>(vehicle_health_.battery_remaining) : kNan;
  m.vehicle_state_received = vehicle_state_fresh;
  m.failsafe_active = vehicle_state_fresh && vehicle_state_.failsafe_active;
  m.flight_mode = vehicle_state_fresh ? vehicle_state_.flight_mode : kFlightModeUnknown;
  // B5: meaningful iff vehicle_state_received (== vehicle_state_fresh).
  m.armed = vehicle_state_fresh && vehicle_state_.armed;

  if (!obstacle_map_received_) {
    m.obstacle_min_distance_m = kNan;
  } else if (obstacle_map_.obstacles.empty()) {
    // A genuinely EMPTY map is evidence ("nothing detected"), not a missing
    // measurement -- +inf, never NaN (R27-2: do not confuse "measured
    // clear" with "cannot measure").
    m.obstacle_min_distance_m = kInf;
  } else {
    // R27-2 audit fix (P8.5): std::min(closest, NaN) returns closest --
    // a corrupt entry anywhere but the very first would be silently dropped
    // from the min, and a corrupt FIRST entry would poison every subsequent
    // comparison instead (order-dependent, neither is safe). One non-finite
    // reading anywhere in the array makes the WHOLE min untrustworthy: we
    // cannot know whether the corrupt entry was the closest one.
    bool any_corrupt = false;
    float closest = std::numeric_limits<float>::infinity();
    for (const auto & obstacle : obstacle_map_.obstacles) {
      if (!std::isfinite(obstacle.distance)) {
        any_corrupt = true;
        break;
      }
      closest = std::min(closest, obstacle.distance);
    }
    m.obstacle_min_distance_m = any_corrupt ? kNan : static_cast<double>(closest);
  }
  m.obstacle_map_age_sec = ageSec(now_stamp, obstacle_map_arrival_, obstacle_map_received_);
  // Y5 (review 2026-08-21): a stale-but-cached LocalizationStatus used to
  // read as "good for obstacle registration" forever -- age-gated exactly
  // like LOCALIZATION_INVALID's own check (status_timeout_sec).
  m.localization_good_for_obstacles = localization_status_received_ &&
    ageSec(now_stamp, localization_status_arrival_, localization_status_received_) <=
      params_.status_timeout_sec &&
    localization_status_.is_valid;

  m.offboard_status_received = offboard_status_fresh;
  m.offboard_state = offboard_status_fresh ? offboard_status_.state : kOffboardStateIdle;
  m.offboard_setpoint_rate_hz =
    offboard_status_fresh ? static_cast<double>(offboard_status_.setpoint_rate_hz) : kNan;
  m.command_selected_age_sec =
    ageSec(now_stamp, command_selected_arrival_, command_selected_received_);
  // Y4: REAL freshness of command_selected -- feeds both SafetyState.
  // command_fresh (no longer a !BLIND_COMMAND proxy) and B5's action gate.
  m.command_fresh = m.command_selected_age_sec <= params_.selected_stale_sec;

  m.vehicle_health_received = vehicle_health_fresh;
  m.imu_ok = vehicle_health_fresh && vehicle_health_.imu_ok;
  m.gps_ok = vehicle_health_fresh && vehicle_health_.gps_ok;
  m.barometer_ok = vehicle_health_fresh && vehicle_health_.barometer_ok;
  m.magnetometer_ok = vehicle_health_fresh && vehicle_health_.magnetometer_ok;
  // Coordinator addition (post-B2 backend fix): overall_level==LEVEL_UNKNOWN
  // means the backend has no real failsafe_flags data yet -- the 4 bools
  // above default fabricated-true in the message struct in that case, never
  // to be trusted alone.
  m.estimator_flags_trustworthy = vehicle_health_fresh &&
    vehicle_health_.overall_level != VehicleHealth::LEVEL_UNKNOWN;

  m.camera_health_received = camera_health_fresh;
  m.camera_stream_ok = camera_health_fresh && camera_health_.level == DiagnosticStatus::OK;

  // R32 (Y1): stale arbiter diagnostics must read as "not increasing",
  // never hold the last true forever (a false FRAME_MISMATCH latch, or an
  // unclearable fault). One legitimate inter-cycle gap never expires a live
  // stream: the window spans 2 cycles, same B1 discipline as the grace.
  const bool authority_diag_fresh = authority_diagnostics_received_ &&
    ageSec(now_stamp, authority_diagnostics_arrival_, authority_diagnostics_received_) <=
    2.0 * params_.arbiter_diagnostics_period_copy_sec;
  m.active_channel_dropping_wrong_frame = authority_diag_fresh && wrong_frame_drop_increasing_;
  m.fused_frame_mismatch =
    odometry_fused_received_ && odometry_fused_.header.frame_id != odom_frame_;

  m.cmd_mission_live = cmd_mission_received_ &&
    ageSec(now_stamp, cmd_mission_arrival_, true) <= kCmdMissionLiveWindowSec;
  if (cmd_mission_received_ && odometry_fused_received_) {
    const auto & mission_pos = cmd_mission_.position;
    const auto & fused_pos = odometry_fused_.pose.pose.position;
    m.cmd_mission_setpoint_distance_from_fused_m = std::hypot(
      std::hypot(mission_pos.x - fused_pos.x, mission_pos.y - fused_pos.y),
      mission_pos.z - fused_pos.z);
  } else {
    m.cmd_mission_setpoint_distance_from_fused_m = kNan;
  }

  return m;
}

void SafetySupervisorNode::onTick()
{
  const rclcpp::Time stamp = now();

  // Tick-sampled window (P8.3 handoff report point 3): sample whatever
  // command_selected currently is, once per safety tick, regardless of
  // whether a new message arrived this tick.
  PositionSample sample;
  if (command_selected_received_ && command_selected_.control_mode == ControlCommand::MODE_POSITION) {
    sample.x = command_selected_.position.x;
    sample.y = command_selected_.position.y;
    sample.z = command_selected_.position.z;
    sample.valid = true;
  }
  command_window_.push_back(sample);
  while (command_window_.size() > kWindowTicks) {
    command_window_.pop_front();
  }

  const Measurements measurements = buildMeasurements(stamp);
  const EvaluationResult evaluated = policy_.evaluate(measurements, stamp.seconds());

  // "0 hop tre" (P8.1 report point 8): confirm ENGAGING_* immediately so the
  // FSM never sits stuck mid-engage.
  if (evaluated.state == SupervisorState::kEngagingHold) {
    // P8.5, header point 6: captures the frozen pose AND calls
    // confirmHoldEngaged() together -- the content check that may force a
    // synthetic +inf age lives inside this call.
    freezeHoldPoseAndConfirm(measurements, stamp);
  } else if (evaluated.state == SupervisorState::kEngagingInhibit) {
    policy_.confirmInhibitEngaged();
  }

  EvaluationResult reported = evaluated;
  reported.state = policy_.state();

  // P8.5: diagnostics-only -- was HOLDING last tick, is not HOLDING now, and
  // the lib's own pose-staleness ceiling explains it (evaluate()'s new
  // one-way transition, failsafe_policy.cpp). Never drives a decision here,
  // purely so the transition is visible on /safety/violations rather than
  // silently inferred from a state diff.
  if (previous_tick_state_ == SupervisorState::kHolding &&
    reported.state != SupervisorState::kHolding &&
    (!std::isfinite(measurements.fused_odometry_age_sec) ||
      measurements.fused_odometry_age_sec > params_.pose_max_age_sec))
  {
    publishHoldPoseLostEvent(measurements.fused_odometry_age_sec, stamp);
  }
  previous_tick_state_ = reported.state;

  // A stale frozen pose from a past episode must never survive to the next
  // one (S:4) -- clear it the instant ClearFault brings the FSM back to
  // NORMAL, not lazily on the next ENGAGING_HOLD.
  if (reported.state == SupervisorState::kNormal) {
    hold_frozen_pose_.valid = false;
  }

  // P8.4/P8.5 (header points 5+6): HOLDING or INHIBITED and not yet
  // confirmed granted on the arbiter -> request the SAFETY latch. Nothing is
  // published on cmd_safety for INHIBIT; for HOLD, the frozen-pose stream
  // (onHoldStreamTick()) runs independently of this request -- the latch
  // only makes the floor durable against a single missed stream tick.
  if (enforcement_enabled_ &&
    (reported.state == SupervisorState::kInhibited || reported.state == SupervisorState::kHolding) &&
    !safety_latch_confirmed_granted_)
  {
    requestSafetyLatchIfDue(stamp);
  }

  ++tick_count_;
  publishSafetyState(reported, measurements, stamp);
  publishViolationEdges(reported, stamp);
  if (tick_count_ % kDiagnosticsEveryTicks == 0) {
    publishDiagnostics(reported, stamp);
  }
}

void SafetySupervisorNode::freezeHoldPoseAndConfirm(
  const Measurements & measurements, const rclcpp::Time & now_stamp)
{
  // Node-owned geometry (header point 3/6 precedent): the lib only ever
  // sees an age, never the pose value itself -- same split as the
  // BLIND_COMMAND window and the handover-jump distance.
  const auto & position = odometry_fused_.pose.pose.position;
  const auto & orientation = odometry_fused_.pose.pose.orientation;
  double yaw = 0.0;
  const bool content_usable = std::isfinite(position.x) && std::isfinite(position.y) &&
    std::isfinite(position.z) &&
    tryYawFromQuaternion(orientation.x, orientation.y, orientation.z, orientation.w, &yaw);

  // A pose that is fresh by AGE but garbage in CONTENT must still escalate
  // straight to INHIBIT (R27-2) -- reuse confirmHoldEngaged()'s existing
  // "not fresh enough" path with a synthetic +inf age rather than widening
  // its signature; content validity is this node's job, the lib never
  // buffers position (S:4, header point 6).
  const double pose_age_for_confirm =
    content_usable ? measurements.fused_odometry_age_sec : kInf;

  policy_.confirmHoldEngaged(pose_age_for_confirm, now_stamp.seconds());

  if (policy_.state() == SupervisorState::kHolding) {
    hold_frozen_pose_.x = position.x;
    hold_frozen_pose_.y = position.y;
    hold_frozen_pose_.z = position.z;
    hold_frozen_pose_.yaw = static_cast<float>(yaw);
    hold_frozen_pose_.valid = true;
  } else {
    // Escalated straight to ENGAGING_INHIBIT -- never freeze on an
    // untrustworthy sample.
    hold_frozen_pose_.valid = false;
  }
}

void SafetySupervisorNode::onHoldStreamTick()
{
  // Defensive double gate: state() is the authority, hold_frozen_pose_.valid
  // guards the (should-be-impossible) case of reaching HOLDING without a
  // captured pose -- never publish from a default-constructed pose.
  if (policy_.state() != SupervisorState::kHolding || !hold_frozen_pose_.valid) {
    return;
  }

  ControlCommand cmd;
  cmd.header.stamp = now();     // restamp every tick (S:4) -- never let the arbiter see it age
  cmd.header.frame_id = odom_frame_;
  cmd.uav_id = uav_id_;
  cmd.control_mode = ControlCommand::MODE_POSITION;
  cmd.position.x = hold_frozen_pose_.x;
  cmd.position.y = hold_frozen_pose_.y;
  cmd.position.z = hold_frozen_pose_.z;
  cmd.yaw = hold_frozen_pose_.yaw;
  cmd.source = kSourceSafety;
  cmd_safety_publisher_->publish(cmd);
}

void SafetySupervisorNode::publishHoldPoseLostEvent(double pose_age_sec, const rclcpp::Time & stamp)
{
  DiagnosticArray array;
  array.header.stamp = stamp;
  DiagnosticStatus status;
  status.name = "safety: HOLD pose lost";
  status.hardware_id = uav_id_;
  status.level = DiagnosticStatus::ERROR;
  status.message = "fused pose aged past pose_max_age_sec while HOLDING -- escalating to INHIBIT";
  status.values.push_back(keyValue("measured", pose_age_sec));
  status.values.push_back(keyValue("threshold", params_.pose_max_age_sec));
  array.status.push_back(status);
  violations_publisher_->publish(array);
}

void SafetySupervisorNode::publishSafetyState(
  const EvaluationResult & result, const Measurements & measurements, const rclcpp::Time & stamp)
{
  const bool changed =
    result.state != last_published_state_ || result.overall_level != last_published_level_;
  const bool due = tick_count_ % kSafetyStateEveryTicks == 0;
  if (!changed && !due) {
    return;
  }

  const auto violating = [&result](ViolationCode code) {
      return result.violations[static_cast<size_t>(code)].status == ViolationStatus::kViolating;
    };

  SafetyState msg;
  msg.header.stamp = stamp;
  msg.uav_id = uav_id_;
  msg.level = result.overall_level;

  for (size_t i = 0; i < kViolationCodeCount; ++i) {
    if (result.violations[i].status != ViolationStatus::kViolating) {
      continue;
    }
    const ViolationCode code = static_cast<ViolationCode>(i);
    std::string detail = result.violations[i].detail;
    const FailsafeAction action = violationCodeAction(code);
    // HOLD and INHIBIT both get whatever describeAction() says is really
    // happening right now (P8.4/P8.5).
    if (action == FailsafeAction::kHold || action == FailsafeAction::kInhibit) {
      detail += " [" + describeAction(action, enforcement_enabled_) + "]";
    }
    msg.violation_codes.push_back(violationCodeName(code));
    msg.violation_details.push_back(detail);
  }

  // P8.4/P8.5: HOLD and INHIBIT are both real enforcement now -- any
  // ENGAGING_*/terminal ceiling state counts as active recovery.
  msg.recovery_active = enforcement_enabled_ &&
    (result.state == SupervisorState::kInhibited ||
    result.state == SupervisorState::kEngagingInhibit ||
    result.state == SupervisorState::kHolding ||
    result.state == SupervisorState::kEngagingHold);
  msg.recommended_action = describeAction(result.target_action, enforcement_enabled_);

  msg.localization_ok = !(
    violating(ViolationCode::kLocalizationInvalid) || violating(ViolationCode::kLocalizationStale) ||
    violating(ViolationCode::kLocalizationJump));
  msg.offboard_link_ok = !violating(ViolationCode::kOffboardUnhealthy);
  msg.battery_ok = !(
    violating(ViolationCode::kBatteryWarn) || violating(ViolationCode::kBatteryCritical) ||
    violating(ViolationCode::kBatteryPx4NoAction));
  msg.obstacle_clear = !violating(ViolationCode::kObstacleTooClose);
  // Y4: REAL freshness of command_selected -- no longer a !BLIND_COMMAND
  // proxy (a command can be perfectly fresh yet BLIND_COMMAND still latch
  // for an unrelated reason, and vice versa: those are different questions).
  msg.command_fresh = measurements.command_fresh;

  safety_state_publisher_->publish(msg);
  last_published_state_ = result.state;
  last_published_level_ = result.overall_level;
}

void SafetySupervisorNode::publishViolationEdges(
  const EvaluationResult & result, const rclcpp::Time & stamp)
{
  DiagnosticArray array;
  array.header.stamp = stamp;

  for (size_t i = 0; i < kViolationCodeCount; ++i) {
    if (result.violations[i].status == last_published_status_[i]) {
      continue;
    }
    const ViolationCode code = static_cast<ViolationCode>(i);
    const ViolationEntry & entry = result.violations[i];

    DiagnosticStatus status;
    status.name = std::string("safety: ") + violationCodeName(code);
    status.hardware_id = uav_id_;
    status.level = violationStatusToDiagnostic(entry.status, violationCodeSeverity(code));
    const std::string edge = entry.status == ViolationStatus::kViolating ? "entered" :
      entry.status == ViolationStatus::kCannotMeasure ? "cannot measure" : "cleared";
    status.message = edge + (entry.detail.empty() ? "" : (": " + entry.detail));
    status.values.push_back(keyValue("measured", entry.measured));
    status.values.push_back(keyValue("threshold", entry.threshold));
    status.values.push_back(
      keyValueStr("action", describeAction(violationCodeAction(code), enforcement_enabled_)));
    array.status.push_back(status);

    last_published_status_[i] = entry.status;
  }

  if (!array.status.empty()) {
    violations_publisher_->publish(array);
  }
}

void SafetySupervisorNode::publishDiagnostics(const EvaluationResult & result, const rclcpp::Time & stamp)
{
  DiagnosticArray array;
  array.header.stamp = stamp;

  DiagnosticStatus summary;
  summary.name = "safety";
  summary.hardware_id = uav_id_;
  summary.level = levelToDiagnostic(result.overall_level);
  summary.message = std::string("state=") + stateName(result.state) +
    ", enforcement_enabled=" + (enforcement_enabled_ ? "true" : "false");
  summary.values.push_back(keyValueStr("enforcement_enabled", enforcement_enabled_ ? "true" : "false"));
  summary.values.push_back(keyValueStr("state", stateName(result.state)));
  // B4: whether this node currently believes it holds the SAFETY latch --
  // re-confirmed every tick against the arbiter's own diagnostics (see
  // control_authority_diagnostics_subscription_), never a fire-and-forget flag.
  summary.values.push_back(
    keyValueStr("latch_confirmed_granted", safety_latch_confirmed_granted_ ? "true" : "false"));
  array.status.push_back(summary);

  for (size_t i = 0; i < kViolationCodeCount; ++i) {
    const ViolationCode code = static_cast<ViolationCode>(i);
    const ViolationEntry & entry = result.violations[i];
    DiagnosticStatus status;
    status.name = std::string("safety: ") + violationCodeName(code);
    status.hardware_id = uav_id_;
    status.level = violationStatusToDiagnostic(entry.status, violationCodeSeverity(code));
    status.message = entry.detail.empty() ?
      (entry.status == ViolationStatus::kOk ? "ok" : "cannot measure") : entry.detail;
    status.values.push_back(keyValue("measured", entry.measured));
    status.values.push_back(keyValue("threshold", entry.threshold));
    status.values.push_back(
      keyValueStr("action", describeAction(violationCodeAction(code), enforcement_enabled_)));
    array.status.push_back(status);
  }

  // Contract S:1.6/S:2.6: safety never DECIDES anything from plan_state --
  // diagnostics-only visibility.
  DiagnosticStatus plan;
  plan.name = "safety: planning/trajectory plan_state";
  plan.hardware_id = uav_id_;
  if (!trajectory_received_) {
    plan.level = DiagnosticStatus::WARN;
    plan.message = "never received";
  } else {
    plan.level = last_trajectory_.plan_state == Trajectory3D::PLAN_STATE_VALID ?
      DiagnosticStatus::OK : DiagnosticStatus::WARN;
    plan.message = std::string(planStateName(last_trajectory_.plan_state)) +
      (last_trajectory_.reason.empty() ? "" : (": " + last_trajectory_.reason));
  }
  array.status.push_back(plan);

  diagnostics_publisher_->publish(array);
}

void SafetySupervisorNode::handleClearFault(
  const std::shared_ptr<rmw_request_id_t> header,
  const std::shared_ptr<ClearFault::Request> request)
{
  const rclcpp::Time stamp = now();
  const Measurements measurements = buildMeasurements(stamp);
  const ClearFaultResult outcome = policy_.clearFault(request->fault_code, measurements, stamp.seconds());
  const SupervisorState state_after = policy_.state();

  RCLCPP_INFO(
    get_logger(), "clear_fault(fault_code=%s): %s",
    request->fault_code.empty() ? "(all)" : request->fault_code.c_str(), outcome.message.c_str());

  // Only release the arbiter's REAL SAFETY latch once the WHOLE FSM is back
  // to NORMAL. outcome.success is true even for a PARTIAL clear (S:4.3:
  // other latched codes keep the ceiling) -- releasing the arbiter latch
  // while this node's own policy still believes it should be cutting is
  // exactly the bug this gate exists to prevent.
  const bool should_release_arbiter_latch = outcome.success && enforcement_enabled_ &&
    clear_safety_latch_client_ && safety_latch_confirmed_granted_ &&
    state_after == SupervisorState::kNormal;

  if (!should_release_arbiter_latch) {
    respondClearFault(header, outcome);
    return;
  }

  if (!clear_safety_latch_client_->service_is_ready()) {
    ClearFaultResult failed = outcome;
    failed.success = false;
    failed.message += " [clear_safety_latch service not available -- cannot confirm release]";
    RCLCPP_ERROR(get_logger(), "%s", failed.message.c_str());
    respondClearFault(header, failed);
    return;
  }

  // Async + callback (header point 5) -- never spin_until_future_complete.
  // The lib already committed its own state (un-latched, FSM at NORMAL);
  // only the RESPONSE to the caller waits for the arbiter's confirmation.
  auto arbiter_request = std::make_shared<ClearFault::Request>();
  arbiter_request->fault_code = request->fault_code;
  clear_safety_latch_client_->async_send_request(
    arbiter_request,
    [this, header, outcome](rclcpp::Client<ClearFault>::SharedFuture future) {
      finishClearFault(header, outcome, future);
    });
}

void SafetySupervisorNode::respondClearFault(
  std::shared_ptr<rmw_request_id_t> header, const ClearFaultResult & outcome)
{
  auto response = std::make_shared<ClearFault::Response>();
  response->success = outcome.success;
  response->result_code = outcome.result_code;
  response->message = outcome.message +
    (enforcement_enabled_ ? "" : " [observe-only: enforcement_enabled=false, no external latch to release]");
  response->remaining_faults = outcome.remaining_faults;
  clear_fault_service_->send_response(*header, *response);
}

void SafetySupervisorNode::finishClearFault(
  std::shared_ptr<rmw_request_id_t> header, ClearFaultResult outcome,
  rclcpp::Client<ClearFault>::SharedFuture future)
{
  const auto arbiter_response = future.get();
  if (arbiter_response->success) {
    safety_latch_confirmed_granted_ = false;
    respondClearFault(header, outcome);
  } else {
    // clearSafetyLatch() never truly refuses (P8.2: "always cleared, a no-op
    // success if nothing is held") -- reaching here means the SERVICE CALL
    // itself failed, a genuinely rare edge case. The lib's own state has
    // already moved to NORMAL; this leaves it out of sync with the arbiter
    // (known, accepted limitation -- R26 point 3, "safety dead when latch"
    // diagnosed via Y15, PX4 failsafe is the fallback). Reported as failure,
    // loudly, rather than silently claiming success.
    outcome.success = false;
    outcome.message += " [arbiter refused to release the SAFETY latch: " + arbiter_response->message + "]";
    RCLCPP_ERROR(get_logger(), "%s", outcome.message.c_str());
    respondClearFault(header, outcome);
  }
}

void SafetySupervisorNode::requestSafetyLatchIfDue(const rclcpp::Time & now_stamp)
{
  if (latch_request_in_flight_) {
    // B4 (real bug): a lost/never-answered response used to wedge this true
    // forever, blocking every future retry. Treat an old-enough in-flight
    // request as abandoned and allow a fresh one -- SetControlAuthority is
    // idempotent (repeated "grant SAFETY" is harmless), so a stray late
    // response arriving after this retry is benign, never double-acted-on.
    if ((now_stamp - latch_request_sent_at_).seconds() < kLatchRequestTimeoutSec) {
      return;
    }
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "SAFETY latch request timed out waiting for a response (>= %.1f s) -- retrying",
      kLatchRequestTimeoutSec);
    latch_request_in_flight_ = false;
  }
  const bool never_attempted = last_latch_request_attempt_.nanoseconds() == 0;
  if (!never_attempted &&
    (now_stamp - last_latch_request_attempt_).seconds() < kLatchRetryPeriodSec)
  {
    return;    // retry <= 1 Hz
  }
  last_latch_request_attempt_ = now_stamp;

  if (!set_authority_client_->service_is_ready()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "set_authority service not available -- INHIBIT cannot latch yet, retrying");
    return;
  }

  latch_request_in_flight_ = true;
  latch_request_sent_at_ = now_stamp;
  auto request = std::make_shared<SetControlAuthority::Request>();
  request->requested_source = kSourceSafety;
  request->reason = std::string(stateName(policy_.state())) + " (P8.4/P8.5 enforcement)";
  set_authority_client_->async_send_request(
    request,
    [this](rclcpp::Client<SetControlAuthority>::SharedFuture future) {
      handleSetAuthorityResponse(future);
    });
}

void SafetySupervisorNode::handleSetAuthorityResponse(
  rclcpp::Client<SetControlAuthority>::SharedFuture future)
{
  latch_request_in_flight_ = false;
  const auto response = future.get();
  const bool granted = response->success && response->granted_source == kSourceSafety;
  safety_latch_confirmed_granted_ = granted;
  if (granted) {
    RCLCPP_WARN(get_logger(), "SAFETY latch granted: %s", response->message.c_str());
  } else {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "SAFETY latch request rejected: %s -- retrying at <= 1 Hz", response->message.c_str());
  }
  publishLatchEvent(granted, response->message);
}

void SafetySupervisorNode::publishLatchEvent(bool granted, const std::string & message)
{
  DiagnosticArray array;
  array.header.stamp = now();
  DiagnosticStatus status;
  status.name = "safety: SAFETY latch request";
  status.hardware_id = uav_id_;
  // WARN on grant: INHIBIT succeeding is the intended, severe outcome, not
  // an error. ERROR on rejection: that means we FAILED to cut.
  status.level = granted ? DiagnosticStatus::WARN : DiagnosticStatus::ERROR;
  status.message = message;
  status.values.push_back(keyValueStr("granted", granted ? "true" : "false"));
  array.status.push_back(status);
  violations_publisher_->publish(array);
}

}  // namespace uav_safety
