#ifndef UAV_SAFETY__SAFETY_SUPERVISOR_NODE_HPP_
#define UAV_SAFETY__SAFETY_SUPERVISOR_NODE_HPP_

#include <array>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>
#include <uav_interfaces/msg/control_command.hpp>
#include <uav_interfaces/msg/localization_status.hpp>
#include <uav_interfaces/msg/offboard_status.hpp>
#include <uav_interfaces/msg/safety_state.hpp>
#include <uav_interfaces/msg/trajectory3_d.hpp>
#include <uav_interfaces/msg/vehicle_health.hpp>
#include <uav_interfaces/msg/vehicle_state.hpp>
#include <uav_interfaces/msg/obstacle_array.hpp>
#include <uav_interfaces/srv/clear_fault.hpp>
#include <uav_interfaces/srv/set_control_authority.hpp>

#include "uav_safety/failsafe_policy.hpp"

namespace uav_safety
{

/// Measurement + enforcement shell around failsafe_policy. Rationale:
/// .claude/plan/P8-safety.md; operator contract: interface-contract §2.18;
/// state/traps + 2026-08-21 review fixes B1-B5: docs/package-status.md §10.
/// enforcement_enabled=false is STRUCTURAL: no cmd_safety publisher, no
/// enforcement clients, no HOLD timer exist (`ros2 node info` is the proof).
/// Numbered points referenced from the .cpp as "header point N":
///   1. Ages from ARRIVAL time only, never header.stamp.
///   2. R24: ONE MutuallyExclusive io_group_ for every callback -- policy_
///      and all last-known state are unsynchronised.
///   3. Node owns wire geometry (window-W displacement, handover jump);
///      the library never buffers position.
///   4. NaN in, NaN out (R27-2): unmeasured is never invented as 0.
///   5. INHIBIT = request the SAFETY latch (async, retry <= 1 Hz), never a
///      publish; ClearFault answers via deferred response only after the
///      arbiter's own clear_safety_latch confirms.
///   6. HOLD's frozen pose is node-owned geometry, captured once at
///      ENGAGING_HOLD; garbage content escalates straight to INHIBIT via a
///      synthetic +inf age.
class SafetySupervisorNode : public rclcpp::Node
{
public:
  explicit SafetySupervisorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  FailsafePolicyParams declareParams();
  void onTick();

  /// Deferred-response service callback (rclcpp's SharedPtrDeferResponseCallback
  /// overload -- header+request only, no response argument means rclcpp will
  /// NOT auto-send one; respondClearFault()/finishClearFault() call
  /// clear_fault_service_->send_response() explicitly, possibly after an
  /// async round trip to the arbiter). See header point 5.
  void handleClearFault(
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<uav_interfaces::srv::ClearFault::Request> request);
  void respondClearFault(
    std::shared_ptr<rmw_request_id_t> header, const ClearFaultResult & outcome);
  void finishClearFault(
    std::shared_ptr<rmw_request_id_t> header, ClearFaultResult outcome,
    rclcpp::Client<uav_interfaces::srv::ClearFault>::SharedFuture future);

  /// P8.4/P8.5 INHIBIT+HOLD enforcement: request the SAFETY latch, retry
  /// <= 1 Hz while not yet granted. No-op unless enforcement_enabled_.
  void requestSafetyLatchIfDue(const rclcpp::Time & now);
  void handleSetAuthorityResponse(
    rclcpp::Client<uav_interfaces::srv::SetControlAuthority>::SharedFuture future);
  void publishLatchEvent(bool granted, const std::string & message);

  /// P8.5, header point 6: captures the frozen pose and calls
  /// confirmHoldEngaged() in the same tick evaluate() reports ENGAGING_HOLD.
  void freezeHoldPoseAndConfirm(const Measurements & measurements, const rclcpp::Time & now);
  /// P8.5: HOLD stream timer callback, same io_group_ as onTick() (R24) --
  /// publishes the frozen pose on cmd_safety, restamped every tick, ONLY
  /// while policy_.state() == HOLDING.
  void onHoldStreamTick();
  /// P8.5: diagnostics-only event for the pose-lost-mid-hold escalation
  /// (evaluate()'s own new one-way transition) -- never drives a decision.
  void publishHoldPoseLostEvent(double pose_age_sec, const rclcpp::Time & stamp);

  Measurements buildMeasurements(const rclcpp::Time & now) const;
  /// Y4: takes `measurements` too, for SafetyState.command_fresh's REAL
  /// freshness value (no longer a !BLIND_COMMAND proxy).
  void publishSafetyState(
    const EvaluationResult & result, const Measurements & measurements, const rclcpp::Time & stamp);
  void publishViolationEdges(const EvaluationResult & result, const rclcpp::Time & stamp);
  void publishDiagnostics(const EvaluationResult & result, const rclcpp::Time & stamp);

  double ageSec(const rclcpp::Time & now, const rclcpp::Time & last_arrival, bool received) const;

  std::string uav_id_;
  std::string prefix_;
  std::string odom_frame_;
  bool enforcement_enabled_ = false;
  FailsafePolicyParams params_;
  FailsafePolicy policy_;

  // R24: the ONLY callback group for this node -- every sub, the tick timer
  // AND the service share it so failsafe_policy_ is never touched by two
  // threads at once (same rule as uav_control_authority's io_group_).
  rclcpp::CallbackGroup::SharedPtr io_group_;

  // ------------------------------------------------------- last-known state
  // Each "received" flag + "last_arrival" pair is the ONLY thing age/
  // measurability is computed from (point 1 above). Never read the stamp
  // embedded in the message itself for freshness.

  uav_interfaces::msg::LocalizationStatus localization_status_;
  bool localization_status_received_ = false;
  rclcpp::Time localization_status_arrival_{0, 0, RCL_ROS_TIME};

  nav_msgs::msg::Odometry odometry_fused_;
  bool odometry_fused_received_ = false;
  rclcpp::Time odometry_fused_arrival_{0, 0, RCL_ROS_TIME};

  // "jumps" only lives in the DETAILED /diagnostics/localization array
  // (localization_health_node.cpp addJumps()) -- NOT in the /state/
  // localization_health summary DiagnosticStatus, which collapses every
  // sub-check into one worst-level message with no jump count at all. See
  // the P8.3 handoff report point 1 for why this subscription was corrected.
  double localization_jump_count_ = std::numeric_limits<double>::quiet_NaN();
  bool localization_diagnostics_received_ = false;
  rclcpp::Time localization_diagnostics_arrival_{0, 0, RCL_ROS_TIME};   // R32 (N3)

  uav_interfaces::msg::ObstacleArray obstacle_map_;
  bool obstacle_map_received_ = false;
  rclcpp::Time obstacle_map_arrival_{0, 0, RCL_ROS_TIME};

  // B2/Y1 (review 2026-08-21): these 4 used to be cached with NO age --
  // once received, a dead publisher left the last GOOD reading frozen
  // forever, read as still-trustworthy indefinitely. Now arrival-tracked
  // exactly like every other subscription (point 1).
  uav_interfaces::msg::VehicleHealth vehicle_health_;
  bool vehicle_health_received_ = false;
  rclcpp::Time vehicle_health_arrival_{0, 0, RCL_ROS_TIME};

  uav_interfaces::msg::VehicleState vehicle_state_;
  bool vehicle_state_received_ = false;
  rclcpp::Time vehicle_state_arrival_{0, 0, RCL_ROS_TIME};

  uav_interfaces::msg::OffboardStatus offboard_status_;
  bool offboard_status_received_ = false;
  rclcpp::Time offboard_status_arrival_{0, 0, RCL_ROS_TIME};

  uav_interfaces::msg::ControlCommand command_selected_;
  bool command_selected_received_ = false;
  // P8.4b: OFFBOARD_UNHEALTHY's faster detection path -- raw arrival time,
  // never header.stamp (same discipline as every other age field here).
  rclcpp::Time command_selected_arrival_{0, 0, RCL_ROS_TIME};

  uav_interfaces::msg::ControlCommand cmd_mission_;
  bool cmd_mission_received_ = false;
  rclcpp::Time cmd_mission_arrival_{0, 0, RCL_ROS_TIME};

  // Y15: sample-and-hold between the (slower, ~1 Hz) diagnostics arrivals --
  // "is the HELD channel's own wrong-frame drop counter increasing" is only
  // re-judged when a new /diagnostics/control_authority message lands, then
  // held constant for every 20 Hz tick until the next one. B1 (review
  // 2026-08-21, real bug): reads the PER-CHANNEL count of whichever channel
  // currently holds authority (latch level if latched, else active_source)
  // -- the old AGGREGATE count let one stray wrong-frame message on an
  // UNRELATED channel latch FRAME_MISMATCH forever.
  double previous_wrong_frame_drop_count_ = std::numeric_limits<double>::quiet_NaN();
  bool wrong_frame_drop_increasing_ = false;
  // R32 (Y1): this sample-and-hold must carry an age -- a diagnostics stream
  // that dies right after reporting "increasing" must not leave the flag
  // stuck true (false FRAME_MISMATCH latch / an unclearable fault).
  bool authority_diagnostics_received_ = false;
  rclcpp::Time authority_diagnostics_arrival_{0, 0, RCL_ROS_TIME};
  // B1: which channel the previous reading's baseline belongs to -- a
  // held-channel CHANGE must reset the baseline, never compare across
  // channels (that would compare apples to oranges).
  uint8_t previous_held_channel_ = kSourceNone;

  diagnostic_msgs::msg::DiagnosticStatus camera_health_;
  bool camera_health_received_ = false;
  rclcpp::Time camera_health_arrival_{0, 0, RCL_ROS_TIME};   // B2/Y1

  uav_interfaces::msg::Trajectory3D last_trajectory_;
  bool trajectory_received_ = false;

  // Tick-sampled ring buffer (window W = 3 ticks, S:0b R1) of command_
  // selected's position -- sampled once per safety tick, not once per wire
  // arrival, so the window duration tracks the safety loop's own cadence
  // regardless of how often MISSION actually republishes (P8.1 report pt 5).
  struct PositionSample
  {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    bool valid = false;   // false if nothing received yet, or mode != POSITION
  };
  static constexpr size_t kWindowTicks = 3;
  std::deque<PositionSample> command_window_;

  rclcpp::Subscription<uav_interfaces::msg::LocalizationStatus>::SharedPtr
    localization_status_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_fused_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    localization_diagnostics_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::ObstacleArray>::SharedPtr obstacle_map_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::VehicleHealth>::SharedPtr vehicle_health_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::VehicleState>::SharedPtr vehicle_state_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::OffboardStatus>::SharedPtr offboard_status_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::ControlCommand>::SharedPtr command_selected_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::ControlCommand>::SharedPtr cmd_mission_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    control_authority_diagnostics_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr camera_health_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::Trajectory3D>::SharedPtr trajectory_subscription_;

  rclcpp::Publisher<uav_interfaces::msg::SafetyState>::SharedPtr safety_state_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr violations_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;

  // enforcement_enabled=false: left null, on purpose, forever for this node's
  // lifetime -- never created, never published/called on. Only constructed
  // inside `if (enforcement_enabled_)` in the constructor -- structural, not
  // a runtime branch (`ros2 node info`/`ros2 topic info` is the proof).
  // cmd_safety_publisher_ is published to ONLY by onHoldStreamTick(), ONLY
  // while HOLDING (P8.5, header point 6) -- INHIBIT still never publishes,
  // it cuts via the latch alone (header point 5).
  rclcpp::Publisher<uav_interfaces::msg::ControlCommand>::SharedPtr cmd_safety_publisher_;
  rclcpp::Client<uav_interfaces::srv::SetControlAuthority>::SharedPtr set_authority_client_;
  rclcpp::Client<uav_interfaces::srv::ClearFault>::SharedPtr clear_safety_latch_client_;

  // P8.4 latch bookkeeping.
  bool safety_latch_confirmed_granted_ = false;
  bool latch_request_in_flight_ = false;
  rclcpp::Time last_latch_request_attempt_{0, 0, RCL_ROS_TIME};
  static constexpr double kLatchRetryPeriodSec = 1.0;
  // B4 (review 2026-08-21, real bug): a lost/never-answered SetControlAuthority
  // response used to wedge latch_request_in_flight_ true forever, blocking
  // every future retry. Timeout treats an old-enough in-flight request as
  // abandoned; the request itself is idempotent (repeated "grant SAFETY" is
  // harmless), so a stray late response arriving after the retry is benign.
  rclcpp::Time latch_request_sent_at_{0, 0, RCL_ROS_TIME};
  static constexpr double kLatchRequestTimeoutSec = 2.0;

  rclcpp::Service<uav_interfaces::srv::ClearFault>::SharedPtr clear_fault_service_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  // P8.5: only ever created inside `if (enforcement_enabled_)`, same
  // structural gate as cmd_safety_publisher_/the two clients (header
  // point 6) -- `ros2 node info` is the proof, not a log line.
  rclcpp::TimerBase::SharedPtr hold_stream_timer_;

  // ------------------------------------------------------------- P8.5 HOLD
  // The ONE piece of geometry this node freezes -- captured once at
  // ENGAGING_HOLD -> HOLDING confirmation (freezeHoldPoseAndConfirm()),
  // never touched again until the next episode (S:4: no interpolation/
  // smoothing/drift). R24: touched only inside io_group_ callbacks
  // (onTick() writes it, onHoldStreamTick() only reads it) -- same
  // MutuallyExclusive group as every other piece of shared state in this
  // node, so never concurrent; no separate lock needed.
  struct HoldFrozenPose
  {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    float yaw = 0.0F;
    bool valid = false;
  };
  HoldFrozenPose hold_frozen_pose_;
  // R24: read+written only from io_group_'s onTick() -- diagnostics-only
  // (publishHoldPoseLostEvent trigger), never fed back into the FSM.
  SupervisorState previous_tick_state_ = SupervisorState::kNormal;

  // ------------------------------------------------------------- publishing
  unsigned tick_count_ = 0;
  static constexpr unsigned kSafetyStateEveryTicks = 4;      // 20 Hz / 4 = 5 Hz
  static constexpr unsigned kDiagnosticsEveryTicks = 20;     // 20 Hz / 20 = 1 Hz
  std::array<ViolationStatus, kViolationCodeCount> last_published_status_{};
  SupervisorState last_published_state_ = SupervisorState::kNormal;
  uint8_t last_published_level_ = uav_interfaces::msg::SafetyState::LEVEL_OK;
};

}  // namespace uav_safety

#endif  // UAV_SAFETY__SAFETY_SUPERVISOR_NODE_HPP_
