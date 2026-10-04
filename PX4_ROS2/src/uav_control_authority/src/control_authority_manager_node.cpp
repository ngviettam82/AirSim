#include "uav_control_authority/control_authority_manager_node.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <uav_interfaces/msg/control_authority.hpp>
#include <uav_interfaces/msg/result_code.hpp>

namespace uav_control_authority
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using uav_interfaces::msg::ControlAuthority;
using uav_interfaces::msg::ControlCommand;
using uav_interfaces::msg::ResultCode;
using uav_interfaces::srv::ClearFault;
using uav_interfaces::srv::SetControlAuthority;

namespace
{

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

CommandSnapshot toSnapshot(const ControlCommand & msg)
{
  CommandSnapshot snapshot;
  snapshot.stamp_sec = rclcpp::Time(msg.header.stamp).seconds();
  snapshot.frame_id = msg.header.frame_id;
  snapshot.control_mode = msg.control_mode;
  snapshot.source = msg.source;
  snapshot.position[0] = msg.position.x;
  snapshot.position[1] = msg.position.y;
  snapshot.position[2] = msg.position.z;
  snapshot.velocity[0] = msg.velocity.x;
  snapshot.velocity[1] = msg.velocity.y;
  snapshot.velocity[2] = msg.velocity.z;
  snapshot.acceleration[0] = msg.acceleration.x;
  snapshot.acceleration[1] = msg.acceleration.y;
  snapshot.acceleration[2] = msg.acceleration.z;
  snapshot.yaw = msg.yaw;
  snapshot.yaw_rate = msg.yaw_rate;
  return snapshot;
}

}  // namespace

// Function-try-block: AuthorityArbiter has no default constructor on purpose
// (S:3d, "refuse to start"), so it can only be built inside the member
// initializer list. This is the only place that can still log the exact
// reason before the exception unwinds the partially built node.
ControlAuthorityManagerNode::ControlAuthorityManagerNode(const rclcpp::NodeOptions & options)
try
: Node("control_authority_manager_node", options),
  uav_id_(declare_parameter<std::string>("uav_id", "uav0")),
  prefix_("/uav/" + uav_id_),
  params_(declareParams()),
  arbiter_(params_)
{
  // R24: ONE group for subs + timer + service -- see header point 4.
  io_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = io_group_;

  const rclcpp::QoS command_qos(10);

  safety_subscription_ = create_subscription<ControlCommand>(
    prefix_ + "/control/cmd_safety", command_qos,
    [this](const ControlCommand::SharedPtr msg) {onSourceMessage(kSourceSafety, msg);},
    sub_options);
  operator_subscription_ = create_subscription<ControlCommand>(
    prefix_ + "/control/cmd_operator", command_qos,
    [this](const ControlCommand::SharedPtr msg) {onSourceMessage(kSourceOperator, msg);},
    sub_options);
  mission_subscription_ = create_subscription<ControlCommand>(
    prefix_ + "/control/cmd_mission", command_qos,
    [this](const ControlCommand::SharedPtr msg) {onSourceMessage(kSourceMission, msg);},
    sub_options);
  test_subscription_ = create_subscription<ControlCommand>(
    prefix_ + "/control/cmd_test", command_qos,
    [this](const ControlCommand::SharedPtr msg) {onSourceMessage(kSourceTest, msg);},
    sub_options);

  command_publisher_ =
    create_publisher<ControlCommand>(prefix_ + "/control/command_selected", command_qos);
  authority_publisher_ = create_publisher<ControlAuthority>(
    prefix_ + "/control/authority", rclcpp::QoS(1).reliable().transient_local());
  diagnostics_publisher_ = create_publisher<DiagnosticArray>(
    prefix_ + "/diagnostics/control_authority", rclcpp::QoS(10));

  // Same group as the subs/timer (R24, B1 fix): the service never blocks
  // (unlike Arm's 5 s ack wait in the gateway), so sharing the group costs
  // nothing, and it is what keeps requestLatch() from racing onMessageArrived.
  set_authority_service_ = create_service<SetControlAuthority>(
    prefix_ + "/control/set_authority",
    [this](
      const std::shared_ptr<SetControlAuthority::Request> request,
      std::shared_ptr<SetControlAuthority::Response> response) {
      handleSetControlAuthority(request, response);
    },
    rmw_qos_profile_services_default, io_group_);

  // P8 R2: reuses ClearFault (no new interface). Same io_group_ as everything
  // else touching arbiter_ -- R24, no exceptions (header point 4). The only
  // legitimate caller is uav_safety, AFTER its own ClearFault accepted; this
  // service never judges the fault, only executes the release.
  clear_safety_latch_service_ = create_service<ClearFault>(
    prefix_ + "/control/clear_safety_latch",
    [this](
      const std::shared_ptr<ClearFault::Request> request,
      std::shared_ptr<ClearFault::Response> response) {
      handleClearSafetyLatch(request, response);
    },
    rmw_qos_profile_services_default, io_group_);

  // Not a wall timer: expiry/dwell/latch must track sim time (nợ #1).
  monitor_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(kMonitorPeriodSec),
    [this]() {onMonitorTick();}, io_group_);

  // Y17: registered AFTER declareParams() (member init list) already ran, so
  // this can never intercept the node's own startup declaration.
  param_callback_handle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return rejectParameterChange(parameters);
    });

  RCLCPP_INFO(get_logger(), "control authority manager ready for %s", uav_id_.c_str());
}
catch (const std::invalid_argument & error)
{
  RCLCPP_FATAL(
    rclcpp::get_logger("control_authority_manager_node"),
    "refusing to start: %s", error.what());
  throw;
}

ArbiterParams ControlAuthorityManagerNode::declareParams()
{
  ArbiterParams params;
  params.odom_frame = declare_parameter<std::string>("odom_frame", params.odom_frame);
  params.source_timeout_sec =
    declare_parameter<double>("source_timeout_sec", params.source_timeout_sec);
  params.downstream_command_timeout_sec = declare_parameter<double>(
    "downstream_command_timeout_sec", params.downstream_command_timeout_sec);
  params.release_dwell_sec =
    declare_parameter<double>("release_dwell_sec", params.release_dwell_sec);
  params.latch_grace_sec = declare_parameter<double>("latch_grace_sec", params.latch_grace_sec);
  params.latch_timeout_sec =
    declare_parameter<double>("latch_timeout_sec", params.latch_timeout_sec);
  params.check_command_age =
    declare_parameter<bool>("check_command_age", params.check_command_age);
  params.max_command_age_sec =
    declare_parameter<double>("max_command_age_sec", params.max_command_age_sec);
  params.enable_test_source =
    declare_parameter<bool>("enable_test_source", params.enable_test_source);
  params.enable_operator_source =
    declare_parameter<bool>("enable_operator_source", params.enable_operator_source);

  RCLCPP_WARN(
    get_logger(),
    "source_timeout_sec=%.3f paired with downstream_command_timeout_sec=%.3f "
    "(gateway's command_timeout_sec copy, px4_command_gateway_node.cpp:58)",
    params.source_timeout_sec, params.downstream_command_timeout_sec);
  return params;
}

rcl_interfaces::msg::SetParametersResult ControlAuthorityManagerNode::rejectParameterChange(
  const std::vector<rclcpp::Parameter> & parameters) const
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason =
    "arbitration parameters are validated once at startup (S:3d); "
    "runtime changes are refused, never silently applied unvalidated -- restart to reconfigure";
  for (const rclcpp::Parameter & parameter : parameters) {
    RCLCPP_ERROR(
      get_logger(), "refusing runtime change to '%s': %s",
      parameter.get_name().c_str(), result.reason.c_str());
  }
  return result;
}

void ControlAuthorityManagerNode::logDrop(uint8_t channel, const ArrivalResult & result)
{
  switch (result.drop_reason) {
    case DropReason::NONE:
    case DropReason::NOT_WINNER:
      break;
    case DropReason::NOT_FINITE:
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%s command dropped: non-finite field", sourceName(channel));
      break;
    case DropReason::WRONG_FRAME:
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%s command dropped: frame_id is not %s", sourceName(channel), params_.odom_frame.c_str());
      break;
    case DropReason::UNSUPPORTED_MODE:
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "%s command dropped: unsupported control_mode", sourceName(channel));
      break;
    case DropReason::STALE:
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "%s command dropped: header.stamp older than max_command_age_sec", sourceName(channel));
      break;
  }
}

void ControlAuthorityManagerNode::onSourceMessage(
  uint8_t channel, const ControlCommand::SharedPtr msg)
{
  const double now_sec = nowSeconds();
  const ArrivalResult result = arbiter_.onMessageArrived(channel, toSnapshot(*msg), now_sec);
  logDrop(channel, result);

  if (result.action == PublishAction::PUBLISH_RESTAMPED) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "%s command carried source=%s, restamped and forwarded",
      sourceName(channel), sourceName(msg->source));
  }

  if (result.action == PublishAction::PUBLISH || result.action == PublishAction::PUBLISH_RESTAMPED) {
    ControlCommand out = *msg;
    out.source = channel;              // topic decides authority, not the sender's claim
    command_publisher_->publish(out);
  }

  if (result.authority_changed) {
    publishAuthority(
      result.previous_source, result.active_source,
      std::string(sourceName(channel)) + " message arrived");
  }
}

void ControlAuthorityManagerNode::onMonitorTick()
{
  const double now_sec = nowSeconds();
  const TickResult result = arbiter_.onTick(now_sec);

  if (result.latch_revocation != LatchRevocation::NONE) {
    RCLCPP_ERROR(
      get_logger(), "latch auto-revoked: %s",
      result.latch_revocation == LatchRevocation::GRACE_EXPIRED
      ? "holder never published within latch_grace_sec"
      : "holder went silent past latch_timeout_sec");
  }

  if (result.authority_changed) {
    const std::string reason = result.active_source == kSourceNone
      ? "all sources silent past release_dwell_sec"
      : "release_dwell_sec elapsed, next best live source promoted";
    publishAuthority(result.previous_source, result.active_source, reason);
  }

  ++tick_count_;
  if (tick_count_ % diagnostics_every_ticks_ == 0) {
    // Y2: publisher-graph queries are not free; run them at 1 Hz, not
    // at the monitor's own 20 Hz.
    checkDuplicatePublishers();
    publishDiagnostics();
  }
  if (tick_count_ % heartbeat_every_ticks_ == 0) {
    publishAuthority(arbiter_.activeSource(), arbiter_.activeSource(), "heartbeat");
  }
}

void ControlAuthorityManagerNode::handleSetControlAuthority(
  const std::shared_ptr<SetControlAuthority::Request> request,
  std::shared_ptr<SetControlAuthority::Response> response)
{
  const uint8_t previous_active = arbiter_.activeSource();
  const LatchOutcome outcome = arbiter_.requestLatch(request->requested_source, nowSeconds());

  response->success = outcome.granted;
  response->granted_source = outcome.granted_source;
  response->message = outcome.message;
  response->result_code = outcome.granted ? ResultCode::SUCCEEDED : ResultCode::ABORTED_NO_AUTHORITY;

  // Y14: request->reason must never be silently dropped -- always logged,
  // and folded into the authority-change reason string below.
  const std::string caller_reason = request->reason.empty() ? "(none given)" : request->reason;
  RCLCPP_INFO(
    get_logger(), "SetControlAuthority(%s): %s -- caller reason: %s",
    sourceName(request->requested_source), outcome.message.c_str(), caller_reason.c_str());

  if (request->requested_source == kSourceNone) {
    // Y6: every RELEASE attempt is logged loudly, granted or not -- it is
    // always a notable authority event, never routine chatter.
    RCLCPP_ERROR(
      get_logger(), "SetControlAuthority release request: %s (caller reason: %s)",
      outcome.message.c_str(), caller_reason.c_str());
  }

  if (arbiter_.activeSource() != previous_active) {
    std::string reason = "SetControlAuthority: " + outcome.message;
    if (!request->reason.empty()) {
      reason += " (caller reason: " + request->reason + ")";
    }
    publishAuthority(previous_active, arbiter_.activeSource(), reason);
  }
}

void ControlAuthorityManagerNode::handleClearSafetyLatch(
  const std::shared_ptr<ClearFault::Request> request,
  std::shared_ptr<ClearFault::Response> response)
{
  const uint8_t previous_active = arbiter_.activeSource();
  const ClearSafetyLatchOutcome outcome = arbiter_.clearSafetyLatch(nowSeconds());

  response->success = outcome.cleared;
  response->result_code = ResultCode::SUCCEEDED;
  response->message = outcome.message;
  // remaining_faults left empty: this service only ever clears AUTHORITY's
  // own SAFETY latch, never uav_safety's fault bookkeeping.

  // P8 R2: every call is a notable authority event, logged loudly -- the
  // arbiter never judges the fault (that split belongs to uav_safety's own
  // ClearFault, upstream of this call), it only executes.
  const std::string caller_reason =
    request->fault_code.empty() ? "(none given)" : request->fault_code;
  RCLCPP_ERROR(
    get_logger(), "clear_safety_latch(fault_code=%s): %s",
    caller_reason.c_str(), outcome.message.c_str());

  if (arbiter_.activeSource() != previous_active) {
    publishAuthority(
      previous_active, arbiter_.activeSource(), "clear_safety_latch: " + outcome.message);
  }
}

void ControlAuthorityManagerNode::publishAuthority(
  uint8_t previous_source, uint8_t active_source, const std::string & reason)
{
  ControlAuthority msg;
  msg.header.stamp = now();
  msg.uav_id = uav_id_;
  msg.active_source = active_source;
  msg.previous_source = previous_source;
  msg.reason = reason;
  authority_publisher_->publish(msg);
}

void ControlAuthorityManagerNode::checkDuplicatePublishers()
{
  const std::array<std::string, 4> source_topics{{
      prefix_ + "/control/cmd_safety",
      prefix_ + "/control/cmd_operator",
      prefix_ + "/control/cmd_mission",
      prefix_ + "/control/cmd_test",
    }};

  duplicate_publisher_seen_ = false;
  size_t index = 0;
  for (const std::string & topic : source_topics) {
    const size_t count = get_publishers_info_by_topic(topic).size();
    publisher_counts_[index++] = {topic, count};
    if (count > 1) {
      duplicate_publisher_seen_ = true;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%zu publishers on %s, expected at most 1", count, topic.c_str());
    }
  }

  const std::string selected_topic = prefix_ + "/control/command_selected";
  const size_t selected_count = get_publishers_info_by_topic(selected_topic).size();
  publisher_counts_[index++] = {selected_topic, selected_count};
  if (selected_count > 1) {
    duplicate_publisher_seen_ = true;
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "%zu publishers on %s, only this node may write it", selected_count, selected_topic.c_str());
  }
}

void ControlAuthorityManagerNode::publishDiagnostics()
{
  DiagnosticArray array;
  array.header.stamp = now();

  DiagnosticStatus arbitration;
  arbitration.name = "control_authority: arbitration";
  arbitration.hardware_id = uav_id_;
  const bool inhibit_active = arbiter_.inhibitActive();
  // R26: INHIBIT's silence on command_selected is deliberate and unbounded --
  // the mandatory compensation is THIS status flipping to ERROR every single
  // diagnostics cycle for as long as it holds, never a one-time log line.
  arbitration.level = inhibit_active ? DiagnosticStatus::ERROR : DiagnosticStatus::OK;
  arbitration.message = std::string("active=") + sourceName(arbiter_.activeSource()) +
    (arbiter_.latchActive()
    ? std::string(", latch floor=") + sourceName(arbiter_.latchLevel()) +
    (arbiter_.latchEffective() ? "" : " (not yet effective, holder never live)")
    : std::string()) +
    (inhibit_active
    ? " -- INHIBIT: SAFETY latch effective with zero setpoints by design; "
    "escape routes: clear_safety_latch, PX4 failsafe, RC (P11)"
    : std::string());
  // Y15: latch state as key/value, for any consumer reading .values instead
  // of parsing .message.
  arbitration.values.push_back(keyValueStr("latch_active", arbiter_.latchActive() ? "true" : "false"));
  arbitration.values.push_back(keyValueStr("latch_level", sourceName(arbiter_.latchLevel())));
  arbitration.values.push_back(
    keyValueStr("latch_effective", arbiter_.latchEffective() ? "true" : "false"));
  arbitration.values.push_back(
    keyValueStr("latch_ever_alive", arbiter_.latchEverAlive() ? "true" : "false"));
  arbitration.values.push_back(keyValue("latch_age_sec", arbiter_.latchAgeSec(nowSeconds())));
  arbitration.values.push_back(keyValueStr("inhibit_active", inhibit_active ? "true" : "false"));
  // B1: exposed so a consumer (uav_safety) can identify the HELD channel
  // without parsing .message text -- needed to read that channel's OWN
  // per-channel wrong_frame count below, never the aggregate.
  arbitration.values.push_back(keyValueStr("active_source", sourceName(arbiter_.activeSource())));
  array.status.push_back(arbitration);

  // B2-d2: age since the last RAW message per channel (valid or not) --
  // distinguishes "channel dead" (age keeps growing) from "channel alive but
  // 100% garbage" (age stays small, yet the channel still never wins).
  DiagnosticStatus raw_age;
  raw_age.name = "control_authority: raw arrival age";
  raw_age.hardware_id = uav_id_;
  raw_age.level = DiagnosticStatus::OK;
  raw_age.message = "seconds since the last message per channel, valid or not";
  const double raw_age_now = nowSeconds();
  raw_age.values.push_back(
    keyValue("safety_sec", arbiter_.rawArrivalAgeSec(kSourceSafety, raw_age_now)));
  raw_age.values.push_back(
    keyValue("operator_sec", arbiter_.rawArrivalAgeSec(kSourceOperator, raw_age_now)));
  raw_age.values.push_back(
    keyValue("mission_sec", arbiter_.rawArrivalAgeSec(kSourceMission, raw_age_now)));
  raw_age.values.push_back(
    keyValue("test_sec", arbiter_.rawArrivalAgeSec(kSourceTest, raw_age_now)));
  array.status.push_back(raw_age);

  const auto dropStatus =
    [&array, this](const std::string & name, unsigned count, uint8_t level) {
      DiagnosticStatus status;
      status.name = "control_authority: dropped " + name;
      status.hardware_id = uav_id_;
      status.level = count > 0 ? level : DiagnosticStatus::OK;
      status.message = count > 0 ? (name + " command(s) dropped") : "none seen";
      status.values.push_back(keyValue("count", static_cast<double>(count)));
      array.status.push_back(status);
    };
  dropStatus("not_finite", arbiter_.droppedNotFiniteCount(), DiagnosticStatus::ERROR);
  dropStatus("unsupported_mode", arbiter_.droppedModeCount(), DiagnosticStatus::WARN);
  dropStatus("stale", arbiter_.droppedStaleCount(), DiagnosticStatus::WARN);

  // B1: wrong_frame carries the aggregate PLUS a per-channel breakdown --
  // uav_safety reads only the currently-HELD channel's own count, never
  // the aggregate (an unrelated channel's garbage must never latch it).
  {
    const unsigned count = arbiter_.droppedWrongFrameCount();
    DiagnosticStatus status;
    status.name = "control_authority: dropped wrong_frame";
    status.hardware_id = uav_id_;
    status.level = count > 0 ? DiagnosticStatus::ERROR : DiagnosticStatus::OK;
    status.message = count > 0 ? "wrong_frame command(s) dropped" : "none seen";
    status.values.push_back(keyValue("count", static_cast<double>(count)));
    status.values.push_back(
      keyValue(
        "dropped_wrong_frame_safety",
        static_cast<double>(arbiter_.droppedWrongFrameCount(kSourceSafety))));
    status.values.push_back(
      keyValue(
        "dropped_wrong_frame_operator",
        static_cast<double>(arbiter_.droppedWrongFrameCount(kSourceOperator))));
    status.values.push_back(
      keyValue(
        "dropped_wrong_frame_mission",
        static_cast<double>(arbiter_.droppedWrongFrameCount(kSourceMission))));
    status.values.push_back(
      keyValue(
        "dropped_wrong_frame_test",
        static_cast<double>(arbiter_.droppedWrongFrameCount(kSourceTest))));
    array.status.push_back(status);
  }

  DiagnosticStatus restamp;
  restamp.name = "control_authority: source field mismatch";
  restamp.hardware_id = uav_id_;
  restamp.level = arbiter_.restampedCount() > 0 ? DiagnosticStatus::ERROR : DiagnosticStatus::OK;
  restamp.message = arbiter_.restampedCount() > 0
    ? "a source published under the wrong source id"
    : "none seen";
  restamp.values.push_back(keyValue("count", static_cast<double>(arbiter_.restampedCount())));
  array.status.push_back(restamp);

  // Y6
  DiagnosticStatus release;
  release.name = "control_authority: latch release attempts";
  release.hardware_id = uav_id_;
  release.level =
    arbiter_.releaseAttemptCount() > 0 ? DiagnosticStatus::ERROR : DiagnosticStatus::OK;
  release.message = arbiter_.releaseAttemptCount() > 0
    ? "a RELEASE was requested (see rejected count for how many were refused)"
    : "none seen";
  release.values.push_back(keyValue("attempts", static_cast<double>(arbiter_.releaseAttemptCount())));
  release.values.push_back(
    keyValue("rejected", static_cast<double>(arbiter_.releaseRejectedCount())));
  array.status.push_back(release);

  // N-c (P10.8, Q-P10-7): a rewound sim clock (bag replay/loop) is diagnosed
  // here, not swallowed -- nonzero means at least one channel/latch was read
  // as "not measurable" (age +inf) purely because now_sec < a stored stamp,
  // never because anything actually went silent.
  DiagnosticStatus clock_regressions;
  clock_regressions.name = "control_authority: clock regressions";
  clock_regressions.hardware_id = uav_id_;
  const unsigned regression_count = arbiter_.clockRegressionsCount();
  clock_regressions.level = regression_count > 0 ? DiagnosticStatus::ERROR : DiagnosticStatus::OK;
  clock_regressions.message = regression_count > 0
    ? "sim clock moved backward -- affected channels read as not-LIVE, non-SAFETY "
    "latches may have released (SAFETY is unaffected, R5)"
    : "none seen";
  clock_regressions.values.push_back(
    keyValue("clock_regressions", static_cast<double>(regression_count)));
  array.status.push_back(clock_regressions);

  DiagnosticStatus publishers;
  publishers.name = "control_authority: publisher count";
  publishers.hardware_id = uav_id_;
  publishers.level = duplicate_publisher_seen_ ? DiagnosticStatus::ERROR : DiagnosticStatus::OK;
  publishers.message = duplicate_publisher_seen_
    ? "a topic this node owns has more than one publisher"
    : "single writer confirmed";
  for (const std::pair<std::string, size_t> & entry : publisher_counts_) {
    publishers.values.push_back(keyValue(entry.first, static_cast<double>(entry.second)));
  }
  array.status.push_back(publishers);

  diagnostics_publisher_->publish(array);
}

}  // namespace uav_control_authority
