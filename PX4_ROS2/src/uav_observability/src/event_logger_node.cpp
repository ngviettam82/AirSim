#include "uav_observability/event_logger_node.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <stdexcept>
#include <utility>

#include <fcntl.h>
#include <unistd.h>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

namespace uav_observability
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using uav_interfaces::msg::ControlAuthority;
using uav_interfaces::msg::LocalizationStatus;
using uav_interfaces::msg::MissionEvent;
using uav_interfaces::msg::MissionStatus;
using uav_interfaces::msg::OffboardStatus;
using uav_interfaces::msg::SafetyState;
using uav_interfaces::msg::VehicleState;

namespace
{

namespace fs = std::filesystem;

void require(bool condition, const std::string & detail)
{
  if (!condition) {
    throw std::invalid_argument("event_logger params: " + detail);
  }
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

std::string expandHome(const std::string & path)
{
  if (!path.empty() && path[0] == '~') {
    const char * home = std::getenv("HOME");
    if (home != nullptr) {
      return std::string(home) + path.substr(1);
    }
  }
  return path;
}

const char * missionEventTypeName(uint8_t type)
{
  switch (type) {
    case MissionEvent::EVENT_STARTED: return "STARTED";
    case MissionEvent::EVENT_STEP_STARTED: return "STEP_STARTED";
    case MissionEvent::EVENT_STEP_COMPLETED: return "STEP_COMPLETED";
    case MissionEvent::EVENT_PAUSED: return "PAUSED";
    case MissionEvent::EVENT_RESUMED: return "RESUMED";
    case MissionEvent::EVENT_FAILED: return "FAILED";
    case MissionEvent::EVENT_ABORTED: return "ABORTED";
    case MissionEvent::EVENT_COMPLETED: return "COMPLETED";
    default: return "UNKNOWN";
  }
}

// FAILED/ABORTED outrank a plain occurrence -- everything else in the
// mission timeline is informational, not itself a fault (plan taxonomy).
EventLevel missionEventTypeLevel(uint8_t type)
{
  if (type == MissionEvent::EVENT_FAILED) {return EventLevel::kError;}
  if (type == MissionEvent::EVENT_ABORTED) {return EventLevel::kWarn;}
  return EventLevel::kInfo;
}

const char * missionStateName(uint8_t state)
{
  switch (state) {
    case MissionStatus::STATE_IDLE: return "IDLE";
    case MissionStatus::STATE_RUNNING: return "RUNNING";
    case MissionStatus::STATE_PAUSED: return "PAUSED";
    case MissionStatus::STATE_ABORTED: return "ABORTED";
    case MissionStatus::STATE_COMPLETED: return "COMPLETED";
    default: return "UNKNOWN";
  }
}

const char * safetyLevelName(uint8_t level)
{
  switch (level) {
    case SafetyState::LEVEL_OK: return "OK";
    case SafetyState::LEVEL_WARNING: return "WARNING";
    case SafetyState::LEVEL_ERROR: return "ERROR";
    case SafetyState::LEVEL_EMERGENCY: return "EMERGENCY";
    default: return "UNKNOWN";
  }
}

EventLevel safetyLevelToEventLevel(uint8_t level)
{
  switch (level) {
    case SafetyState::LEVEL_WARNING: return EventLevel::kWarn;
    case SafetyState::LEVEL_ERROR:
    case SafetyState::LEVEL_EMERGENCY: return EventLevel::kError;
    default: return EventLevel::kInfo;
  }
}

// ControlAuthority.msg carries no separate "latch level" field -- the
// arbiter already publishes active_source as the EFFECTIVE (post-latch)
// holder, so tracking it alone already captures every latch transition
// (deviation from the plan's "active_source; latch level" wording -- see
// P10.5 report for the one-line reason).
const char * authoritySourceName(uint8_t source)
{
  switch (source) {
    case ControlAuthority::SOURCE_TEST: return "TEST";
    case ControlAuthority::SOURCE_MISSION: return "MISSION";
    case ControlAuthority::SOURCE_OPERATOR: return "OPERATOR";
    case ControlAuthority::SOURCE_SAFETY: return "SAFETY";
    default: return "NONE";
  }
}

const char * localizationSourceName(uint8_t source)
{
  switch (source) {
    case LocalizationStatus::SOURCE_GPS: return "GPS";
    case LocalizationStatus::SOURCE_VIO: return "VIO";
    case LocalizationStatus::SOURCE_OPTICAL_FLOW: return "OPTICAL_FLOW";
    case LocalizationStatus::SOURCE_FUSED: return "FUSED";
    default: return "NONE";
  }
}

const char * flightModeName(uint8_t mode)
{
  switch (mode) {
    case VehicleState::FLIGHT_MODE_MANUAL: return "MANUAL";
    case VehicleState::FLIGHT_MODE_ALTITUDE: return "ALTITUDE";
    case VehicleState::FLIGHT_MODE_POSITION: return "POSITION";
    case VehicleState::FLIGHT_MODE_OFFBOARD: return "OFFBOARD";
    case VehicleState::FLIGHT_MODE_HOLD: return "HOLD";
    case VehicleState::FLIGHT_MODE_RETURN: return "RETURN";
    case VehicleState::FLIGHT_MODE_TAKEOFF: return "TAKEOFF";
    case VehicleState::FLIGHT_MODE_LAND: return "LAND";
    case VehicleState::FLIGHT_MODE_MISSION: return "MISSION";
    default: return "UNKNOWN";
  }
}

const char * offboardStateName(uint8_t state)
{
  switch (state) {
    case OffboardStatus::STATE_IDLE: return "IDLE";
    case OffboardStatus::STATE_STREAMING: return "STREAMING";
    case OffboardStatus::STATE_ENGAGING: return "ENGAGING";
    case OffboardStatus::STATE_ACTIVE: return "ACTIVE";
    case OffboardStatus::STATE_FAULT: return "FAULT";
    default: return "UNKNOWN";
  }
}

const char * diagnosticLevelName(uint8_t level)
{
  switch (level) {
    case DiagnosticStatus::OK: return "OK";
    case DiagnosticStatus::WARN: return "WARN";
    case DiagnosticStatus::ERROR: return "ERROR";
    case DiagnosticStatus::STALE: return "STALE";
    default: return "UNKNOWN";
  }
}

EventLevel diagnosticLevelToEventLevel(uint8_t level)
{
  switch (level) {
    case DiagnosticStatus::WARN:
    case DiagnosticStatus::STALE: return EventLevel::kWarn;
    case DiagnosticStatus::ERROR: return EventLevel::kError;
    default: return EventLevel::kInfo;
  }
}

}  // namespace

EventLoggerNode::EventLoggerNode(const rclcpp::NodeOptions & options)
try
: Node("event_logger_node", options), ledger_(2.0)
{
  declareAndValidateParams();   // throws std::invalid_argument -- refuse to start (yaml sai)
  prefix_ = "/uav/" + uav_id_;
  ledger_ = EventLedger(fsync_period_sec_);   // real period, known only after params parsed

  // header point 1: ONE group for every sub AND the diagnostics timer.
  io_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  openLogFile();   // never throws -- failure just leaves file_state_ kDegraded (header point 3)

  diagnostics_publisher_ = create_publisher<DiagnosticArray>(
    prefix_ + "/diagnostics/observability_events", rclcpp::QoS(10));

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = io_group_;

  // QoS below matches each REAL publisher's reliability+durability class
  // (mission_executor_node.cpp, safety_supervisor_node.cpp,
  // control_authority_manager_node.cpp, localization_mux_node.cpp/
  // source_channel.cpp, px4_state_adapter_node.cpp,
  // offboard_session_manager_node.cpp, diagnostics_node.cpp) -- a TL
  // publisher paired with a Volatile subscriber never sees the latched
  // sample (same rule rosbag_manager_node's yaml comment states).
  mission_event_sub_ = create_subscription<MissionEvent>(
    prefix_ + "/mission/events", rclcpp::QoS(50).reliable(),
    [this](const MissionEvent::SharedPtr message) {onMissionEvent(message);}, sub_options);
  mission_status_sub_ = create_subscription<MissionStatus>(
    prefix_ + "/mission/status", rclcpp::QoS(1).reliable().transient_local(),
    [this](const MissionStatus::SharedPtr message) {onMissionStatus(message);}, sub_options);
  safety_state_sub_ = create_subscription<SafetyState>(
    prefix_ + "/safety/state", rclcpp::QoS(1).reliable().transient_local(),
    [this](const SafetyState::SharedPtr message) {onSafetyState(message);}, sub_options);
  safety_violations_sub_ = create_subscription<DiagnosticArray>(
    prefix_ + "/safety/violations", rclcpp::QoS(20).reliable(),
    [this](const DiagnosticArray::SharedPtr message) {onSafetyViolations(message);}, sub_options);
  authority_sub_ = create_subscription<ControlAuthority>(
    prefix_ + "/control/authority", rclcpp::QoS(1).reliable().transient_local(),
    [this](const ControlAuthority::SharedPtr message) {onAuthority(message);}, sub_options);
  estimator_source_sub_ = create_subscription<std_msgs::msg::String>(
    prefix_ + "/state/estimator_source", rclcpp::QoS(10).transient_local(),
    [this](const std_msgs::msg::String::SharedPtr message) {onEstimatorSource(message);},
    sub_options);
  localization_status_sub_ = create_subscription<LocalizationStatus>(
    prefix_ + "/state/localization_status", rclcpp::QoS(10),
    [this](const LocalizationStatus::SharedPtr message) {onLocalizationStatus(message);},
    sub_options);
  vehicle_sub_ = create_subscription<VehicleState>(
    prefix_ + "/state/vehicle", rclcpp::QoS(10),
    [this](const VehicleState::SharedPtr message) {onVehicle(message);}, sub_options);
  offboard_status_sub_ = create_subscription<OffboardStatus>(
    prefix_ + "/backend/offboard_status", rclcpp::QoS(10),
    [this](const OffboardStatus::SharedPtr message) {onOffboardStatus(message);}, sub_options);
  aggregated_sub_ = create_subscription<DiagnosticArray>(
    prefix_ + "/diagnostics/aggregated", rclcpp::QoS(10),
    [this](const DiagnosticArray::SharedPtr message) {onAggregated(message);}, sub_options);

  diagnostics_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0),
    [this]() {onDiagnosticsTick();}, io_group_);

  RCLCPP_INFO(
    get_logger(), "event_logger ready for %s: file_state=%s path=%s",
    uav_id_.c_str(), file_state_ == EventLoggerFileState::kOpen ? "OPEN" : "DEGRADED",
    log_path_.c_str());
}
catch (const std::invalid_argument & error)
{
  RCLCPP_FATAL(rclcpp::get_logger("event_logger_node"), "refusing to start: %s", error.what());
  throw;
}

EventLoggerNode::~EventLoggerNode()
{
  closeLogFile();
}

void EventLoggerNode::declareAndValidateParams()
{
  uav_id_ = declare_parameter<std::string>("uav_id", "uav0");
  log_root_ = expandHome(declare_parameter<std::string>("log_root", "~/uav_events"));
  fsync_period_sec_ = declare_parameter<double>("fsync_period_sec", 2.0);

  require(!log_root_.empty(), "log_root must not be empty");
  require(
    std::isfinite(fsync_period_sec_) && fsync_period_sec_ > 0.0,
    "fsync_period_sec must be finite and > 0");
}

std::string EventLoggerNode::makeLogFileName() const
{
  // header point 2: wall clock UTC, never sim time.
  const auto now_wall = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now_wall);
  std::tm utc_tm{};
  gmtime_r(&now_time, &utc_tm);
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%SZ", &utc_tm);
  return uav_id_ + "_" + std::string(buffer) + "_events.jsonl";
}

void EventLoggerNode::openLogFile()
{
  std::error_code create_ec;
  fs::create_directories(log_root_, create_ec);   // best-effort; open() below is the real gate

  log_path_ = log_root_ + "/" + makeLogFileName();
  const int fd = ::open(log_path_.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
  if (fd < 0) {
    RCLCPP_ERROR(
      get_logger(), "failed to open event log %s: %s", log_path_.c_str(), std::strerror(errno));
    log_fd_ = -1;
    file_state_ = EventLoggerFileState::kDegraded;
    return;
  }
  log_fd_ = fd;
  file_state_ = EventLoggerFileState::kOpen;
}

void EventLoggerNode::closeLogFile()
{
  if (log_fd_ < 0) {
    return;
  }
  ::fsync(log_fd_);
  ::close(log_fd_);
  log_fd_ = -1;
}

void EventLoggerNode::emit(
  const std::string & src, const std::string & field, const std::string & value,
  EventLevel level, const std::map<std::string, std::string> & detail)
{
  const double t_sim = nowSimSeconds();
  const double t_wall = nowWallSeconds();
  const auto line = ledger_.observe(src, field, value, level, t_sim, t_wall, detail);
  if (!line.has_value()) {
    return;   // no edge -- nothing to write, no seq spent (EventLedger's own rule)
  }
  writeLine(*line, level);
}

void EventLoggerNode::writeLine(const std::string & line, EventLevel level)
{
  if (log_fd_ < 0) {
    // header point 3: DEGRADED at boot, or the fd otherwise never opened --
    // the edge still happened (ledger_ already advanced seq), just no file.
    ++lines_dropped_;
    return;
  }
  const std::string with_newline = line + "\n";
  const ssize_t written = ::write(log_fd_, with_newline.data(), with_newline.size());
  if (written < 0 || static_cast<size_t>(written) != with_newline.size()) {
    ++lines_dropped_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "event line write failed: %s", std::strerror(errno));
    return;
  }
  ++lines_written_;
  bytes_written_ += with_newline.size();

  const double t_wall = nowWallSeconds();
  if (!ledger_.needsFsync(level, t_wall)) {
    return;
  }
  // level >= kError always needs fsync (EventLedger::needsFsync) -- this
  // runs synchronously, before the subscription callback that led here
  // returns, so an ERROR line is durable on disk before spin() moves on.
  if (::fsync(log_fd_) != 0) {
    ++fsync_errors_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "event log fsync failed: %s", std::strerror(errno));
    return;   // O1: never throw -- counted instead
  }
  ledger_.markFsynced(t_wall);
}

double EventLoggerNode::nowSimSeconds() const
{
  return now().seconds();
}

double EventLoggerNode::nowWallSeconds()
{
  return std::chrono::duration<double>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

void EventLoggerNode::onMissionEvent(const MissionEvent::SharedPtr message)
{
  // header point 4: every message is its own event -- unique field per call.
  const std::string field = std::to_string(mission_event_seq_++);
  const std::map<std::string, std::string> detail{
    {"mission_id", message->mission_id},
    {"step_name", message->step_name},
    {"description", message->description},
    {"result_code", std::to_string(message->result_code)},
  };
  emit(
    "mission/events", field, missionEventTypeName(message->event_type),
    missionEventTypeLevel(message->event_type), detail);
}

void EventLoggerNode::onMissionStatus(const MissionStatus::SharedPtr message)
{
  emit(
    "mission/status", "state", missionStateName(message->state), EventLevel::kInfo,
    {{"mission_id", message->mission_id}});
  emit(
    "mission/status", "current_step_index", std::to_string(message->current_step_index),
    EventLevel::kInfo, {{"current_step_name", message->current_step_name}});
}

void EventLoggerNode::onSafetyState(const SafetyState::SharedPtr message)
{
  const EventLevel level = safetyLevelToEventLevel(message->level);
  emit("safety/state", "level", safetyLevelName(message->level), level);
  emit(
    "safety/state", "recovery_active", message->recovery_active ? "true" : "false", level,
    {{"recommended_action", message->recommended_action}});
}

void EventLoggerNode::onSafetyViolations(const DiagnosticArray::SharedPtr message)
{
  std::set<std::string> current_names;
  for (const DiagnosticStatus & status : message->status) {
    current_names.insert(status.name);
    emit(
      "safety/violations", status.name, diagnosticLevelName(status.level),
      diagnosticLevelToEventLevel(status.level), {{"message", status.message}});
  }
  // header point 5: a name present last message but missing from this one
  // is the "cleared" edge -- DiagnosticArray never says so on its own.
  for (const std::string & name : last_violation_names_) {
    if (current_names.count(name) == 0) {
      emit("safety/violations", name, "CLEARED", EventLevel::kInfo);
    }
  }
  last_violation_names_ = std::move(current_names);
}

void EventLoggerNode::onAuthority(const ControlAuthority::SharedPtr message)
{
  emit(
    "control/authority", "active_source", authoritySourceName(message->active_source),
    EventLevel::kInfo, {{"reason", message->reason}});
}

void EventLoggerNode::onEstimatorSource(const std_msgs::msg::String::SharedPtr message)
{
  emit("state/estimator_source", "value", message->data, EventLevel::kInfo);
}

void EventLoggerNode::onLocalizationStatus(const LocalizationStatus::SharedPtr message)
{
  emit(
    "state/localization_status", "is_valid", message->is_valid ? "true" : "false",
    message->is_valid ? EventLevel::kInfo : EventLevel::kWarn, {{"detail", message->detail}});
  emit(
    "state/localization_status", "active_source",
    localizationSourceName(message->active_source), EventLevel::kInfo);
}

void EventLoggerNode::onVehicle(const VehicleState::SharedPtr message)
{
  emit("state/vehicle", "armed", message->armed ? "true" : "false", EventLevel::kInfo);
  emit("state/vehicle", "flight_mode", flightModeName(message->flight_mode), EventLevel::kInfo);
}

void EventLoggerNode::onOffboardStatus(const OffboardStatus::SharedPtr message)
{
  const EventLevel level =
    message->state == OffboardStatus::STATE_FAULT ? EventLevel::kError : EventLevel::kInfo;
  emit(
    "backend/offboard_status", "state", offboardStateName(message->state), level,
    {{"detail", message->detail}});
}

void EventLoggerNode::onAggregated(const DiagnosticArray::SharedPtr message)
{
  for (const DiagnosticStatus & status : message->status) {
    if (status.name != "system") {
      continue;
    }
    emit(
      "diagnostics/aggregated", "level", diagnosticLevelName(status.level),
      diagnosticLevelToEventLevel(status.level));
    for (const KeyValue & kv : status.values) {
      // P10.9b (design panel S:4.a D5, docs/interface-contract-v0.1.md
      // S:2.20): NO_GO now logs at ERROR (fsync'd before this callback
      // returns, EventLedger's own kError rule) -- it is the ONE go_no_go
      // value that must survive a process crash; UNKNOWN keeps its
      // pre-P10.9b WARN (R30: "cannot measure" is not the same severity as
      // "measured and bad"). worst_item/gate_mode piggyback on the SAME
      // edge-detected KeyValues so a reader of the JSONL timeline can see
      // WHAT blocked without cross-referencing the live wire.
      if (kv.key == "go_no_go") {
        const EventLevel level = kv.value == "GO" ? EventLevel::kInfo :
          kv.value == "NO_GO" ? EventLevel::kError : EventLevel::kWarn;
        emit("diagnostics/aggregated", "go_no_go", kv.value, level);
      } else if (kv.key == "worst_item") {
        emit("diagnostics/aggregated", "worst_item", kv.value, EventLevel::kInfo);
      } else if (kv.key == "gate_mode") {
        emit("diagnostics/aggregated", "gate_mode", kv.value, EventLevel::kInfo);
      }
    }
    return;
  }
}

void EventLoggerNode::onDiagnosticsTick()
{
  const bool open = file_state_ == EventLoggerFileState::kOpen;

  DiagnosticArray array;
  array.header.stamp = now();

  DiagnosticStatus status;
  status.name = "event_logger";
  status.hardware_id = uav_id_;
  status.level = open ? DiagnosticStatus::OK : DiagnosticStatus::ERROR;
  status.message = std::string("state=") + (open ? "OPEN" : "DEGRADED");
  status.values.push_back(keyValueStr("state", open ? "OPEN" : "DEGRADED"));
  status.values.push_back(keyValueStr("file_path", log_path_));
  status.values.push_back(keyValue("lines_written", static_cast<double>(lines_written_)));
  status.values.push_back(keyValue("lines_dropped", static_cast<double>(lines_dropped_)));
  status.values.push_back(keyValue("bytes_written", static_cast<double>(bytes_written_)));
  status.values.push_back(keyValue("fsync_errors", static_cast<double>(fsync_errors_)));
  array.status.push_back(status);

  diagnostics_publisher_->publish(array);
}

}  // namespace uav_observability
