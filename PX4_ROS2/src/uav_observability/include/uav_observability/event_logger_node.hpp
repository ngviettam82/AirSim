#ifndef UAV_OBSERVABILITY__EVENT_LOGGER_NODE_HPP_
#define UAV_OBSERVABILITY__EVENT_LOGGER_NODE_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <uav_interfaces/msg/control_authority.hpp>
#include <uav_interfaces/msg/localization_status.hpp>
#include <uav_interfaces/msg/mission_event.hpp>
#include <uav_interfaces/msg/mission_status.hpp>
#include <uav_interfaces/msg/offboard_status.hpp>
#include <uav_interfaces/msg/safety_state.hpp>
#include <uav_interfaces/msg/vehicle_state.hpp>

#include "uav_observability/event_ledger.hpp"

namespace uav_observability
{

/// Reported on /uav/<id>/diagnostics/observability_events KeyValue "state".
enum class EventLoggerFileState : uint8_t
{
  kOpen,
  kDegraded,
};

/// Unified event timeline node -- P10.5 (plan S:2.3). Typed sub on 10
/// sources -> EventLedger (edge-triggered) -> one JSONL file opened once at
/// boot. O2: this node NEVER publishes an event on the bus, only a file line
/// and a 1 Hz self-diagnostics DiagnosticArray describing the file itself.
/// Numbered points referenced from the .cpp as "header point N":
///   1. R24: ONE MutuallyExclusive io_group_ for every sub + the diagnostics
///      timer -- EventLedger and the raw file descriptor are not internally
///      synchronised (same discipline as the other two P10 nodes).
///   2. Log file NAME is wall-clock UTC (system_clock), never sim time --
///      same reason as rosbag_manager_node's bag directory name (sim time
///      can start at/near 0 or run backwards, which would collide names).
///   3. Boot-time open() failure never takes the node down: file_state_
///      becomes kDegraded, every edge still runs through EventLedger (seq
///      keeps advancing so downstream never sees a seq gap the lib itself
///      didn't cause) but writeLine() counts it as lines_dropped_ (O1).
///   4. mission/events: EVERY message is its own occurrence, never
///      deduplicated (plan S:2.3) -- each message gets a unique EventLedger
///      field (an incrementing counter), not a fixed one, so two identical-
///      content occurrences still both get a line. Every OTHER source here
///      is genuinely edge-triggered on a fixed field.
///   5. safety/violations (DiagnosticArray) has no explicit "cleared"
///      message -- a violation disappearing from the array IS the edge, so
///      onSafetyViolations() diffs the current status names against the
///      previous message's names to synthesize a CLEARED edge.
class EventLoggerNode : public rclcpp::Node
{
public:
  explicit EventLoggerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~EventLoggerNode() override;

private:
  void declareAndValidateParams();
  void openLogFile();
  void closeLogFile();
  std::string makeLogFileName() const;

  void onMissionEvent(const uav_interfaces::msg::MissionEvent::SharedPtr message);
  void onMissionStatus(const uav_interfaces::msg::MissionStatus::SharedPtr message);
  void onSafetyState(const uav_interfaces::msg::SafetyState::SharedPtr message);
  void onSafetyViolations(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message);
  void onAuthority(const uav_interfaces::msg::ControlAuthority::SharedPtr message);
  void onEstimatorSource(const std_msgs::msg::String::SharedPtr message);
  void onLocalizationStatus(const uav_interfaces::msg::LocalizationStatus::SharedPtr message);
  void onVehicle(const uav_interfaces::msg::VehicleState::SharedPtr message);
  void onOffboardStatus(const uav_interfaces::msg::OffboardStatus::SharedPtr message);
  void onAggregated(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message);

  void onDiagnosticsTick();

  void emit(
    const std::string & src, const std::string & field, const std::string & value,
    EventLevel level, const std::map<std::string, std::string> & detail = {});
  void writeLine(const std::string & line, EventLevel level);

  double nowSimSeconds() const;
  static double nowWallSeconds();

  // ------------------------------------------------------------- parameters
  std::string uav_id_;
  std::string prefix_;
  std::string log_root_;
  double fsync_period_sec_ = 0.0;

  // header point 1: the ONLY group -- every sub, the diagnostics timer.
  rclcpp::CallbackGroup::SharedPtr io_group_;

  rclcpp::Subscription<uav_interfaces::msg::MissionEvent>::SharedPtr mission_event_sub_;
  rclcpp::Subscription<uav_interfaces::msg::MissionStatus>::SharedPtr mission_status_sub_;
  rclcpp::Subscription<uav_interfaces::msg::SafetyState>::SharedPtr safety_state_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr safety_violations_sub_;
  rclcpp::Subscription<uav_interfaces::msg::ControlAuthority>::SharedPtr authority_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr estimator_source_sub_;
  rclcpp::Subscription<uav_interfaces::msg::LocalizationStatus>::SharedPtr
  localization_status_sub_;
  rclcpp::Subscription<uav_interfaces::msg::VehicleState>::SharedPtr vehicle_sub_;
  rclcpp::Subscription<uav_interfaces::msg::OffboardStatus>::SharedPtr offboard_status_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr aggregated_sub_;

  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;

  EventLedger ledger_;

  // Raw POSIX fd, not FILE*/ofstream -- fsync() needs the fd directly, and
  // O_APPEND write() is already unbuffered (satisfies "flush every line").
  int log_fd_ = -1;
  std::string log_path_;
  EventLoggerFileState file_state_ = EventLoggerFileState::kDegraded;

  // header point 4.
  uint64_t mission_event_seq_ = 0;
  // header point 5.
  std::set<std::string> last_violation_names_;

  // -------------------------------------------------------------- counters
  uint64_t lines_written_ = 0;
  uint64_t lines_dropped_ = 0;
  uint64_t bytes_written_ = 0;
  uint64_t fsync_errors_ = 0;
};

}  // namespace uav_observability

#endif  // UAV_OBSERVABILITY__EVENT_LOGGER_NODE_HPP_
