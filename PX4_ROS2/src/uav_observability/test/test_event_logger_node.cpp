// P10.5: the real node, driven through real topics by a probe (R20, hence
// the isolated domain ENV ROS_DOMAIN_ID=99, see CMakeLists.txt). Every wait
// is event-anchored (waitFor + a hang ceiling), never a bare sleep (R21).
#include <atomic>
#include <cctype>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <uav_interfaces/msg/control_authority.hpp>
#include <uav_interfaces/msg/localization_status.hpp>
#include <uav_interfaces/msg/mission_event.hpp>
#include <uav_interfaces/msg/mission_status.hpp>
#include <uav_interfaces/msg/offboard_status.hpp>
#include <uav_interfaces/msg/safety_state.hpp>
#include <uav_interfaces/msg/vehicle_state.hpp>

#include "uav_observability/event_logger_node.hpp"

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
using namespace std::chrono_literals;

namespace uav_observability
{
namespace
{

namespace fs = std::filesystem;

bool waitFor(const std::function<bool()> & happened, double ceiling_seconds)
{
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(ceiling_seconds);
  while (std::chrono::steady_clock::now() < deadline) {
    if (happened()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return happened();
}

std::string diagnosticStr(const DiagnosticStatus & status, const std::string & key)
{
  for (const KeyValue & kv : status.values) {
    if (kv.key == key) {
      return kv.value;
    }
  }
  return "";
}

double diagnosticNum(const DiagnosticStatus & status, const std::string & key)
{
  const std::string raw = diagnosticStr(status, key);
  if (raw.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  try {
    return std::stod(raw);
  } catch (const std::exception &) {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

std::vector<std::string> readAllLines(const std::string & path)
{
  std::vector<std::string> lines;
  std::ifstream file(path);
  std::string line;
  while (std::getline(file, line)) {
    if (!line.empty()) {
      lines.push_back(line);
    }
  }
  return lines;
}

/// Substring extraction, not a real JSON parser -- same convention the
/// EventLedger's own gtest (test_event_ledger.cpp) uses via contains().
/// Good enough here: this test controls every value it writes, so no field
/// value in these fixtures ever contains a literal `":"` sequence.
std::string jsonStringField(const std::string & line, const std::string & key)
{
  const std::string needle = "\"" + key + "\":\"";
  const size_t start = line.find(needle);
  if (start == std::string::npos) {
    return "";
  }
  const size_t value_start = start + needle.size();
  const size_t value_end = line.find('"', value_start);
  if (value_end == std::string::npos) {
    return "";
  }
  return line.substr(value_start, value_end - value_start);
}

long long jsonIntField(const std::string & line, const std::string & key)
{
  const std::string needle = "\"" + key + "\":";
  const size_t start = line.find(needle);
  if (start == std::string::npos) {
    return -1;
  }
  size_t value_start = start + needle.size();
  size_t value_end = value_start;
  while (value_end < line.size() && std::isdigit(static_cast<unsigned char>(line[value_end]))) {
    ++value_end;
  }
  if (value_end == value_start) {
    return -1;
  }
  return std::stoll(line.substr(value_start, value_end - value_start));
}

/// (f): fixed key set/order, same 9 keys event_ledger.hpp documents.
bool hasFixedKeyOrder(const std::string & line)
{
  if (line.empty() || line.front() != '{' || line.back() != '}') {
    return false;
  }
  static const std::vector<std::string> keys = {
    "\"t_sim\"", "\"t_wall\"", "\"seq\"", "\"src\"", "\"field\"", "\"from\"", "\"to\"", "\"level\"",
    "\"detail\"",
  };
  size_t last_pos = 0;
  for (const std::string & key : keys) {
    const size_t pos = line.find(key);
    if (pos == std::string::npos || pos < last_pos) {
      return false;
    }
    last_pos = pos;
  }
  return true;
}

/// Unique, empty, auto-removed temp dir per test -- never shared across
/// tests (parallel ctest runs), same shape as test_rosbag_manager_node.cpp's
/// TempDir (duplicated locally: no shared test-util header in this package).
class TempDir
{
public:
  explicit TempDir(const std::string & tag)
  {
    static std::atomic<uint64_t> counter{0};
    const auto salt = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = fs::temp_directory_path() /
      ("uav_observability_evlog_" + tag + "_" + std::to_string(counter.fetch_add(1)) + "_" +
      std::to_string(salt));
    fs::create_directories(path_);
  }
  ~TempDir() {std::error_code ec; fs::remove_all(path_, ec);}
  TempDir(const TempDir &) = delete;
  TempDir & operator=(const TempDir &) = delete;

  const fs::path & path() const {return path_;}

private:
  fs::path path_;
};

/// Node + probe + spinning executor + a diagnostics/observability_events
/// tap. The node subscribes to all 10 fixed topics unconditionally at
/// construction -- tests create their OWN typed publishers on probe() for
/// whichever source(s) they exercise (diagnostics_node test precedent).
class Harness
{
public:
  Harness(const std::string & uav_id, const std::vector<rclcpp::Parameter> & params)
  : prefix_("/uav/" + uav_id)
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    std::vector<rclcpp::Parameter> node_params = params;
    bool has_uav_id = false;
    for (rclcpp::Parameter & param : node_params) {
      if (param.get_name() == "uav_id") {
        param = rclcpp::Parameter("uav_id", uav_id);
        has_uav_id = true;
      }
    }
    if (!has_uav_id) {
      node_params.push_back(rclcpp::Parameter("uav_id", uav_id));
    }

    rclcpp::NodeOptions options;
    options.parameter_overrides(node_params);
    node_ = std::make_shared<EventLoggerNode>(options);

    probe_ = std::make_shared<rclcpp::Node>("event_logger_probe_" + uav_id);
    diagnostics_sub_ = probe_->create_subscription<DiagnosticArray>(
      prefix_ + "/diagnostics/observability_events", rclcpp::QoS(10),
      [this](const DiagnosticArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!msg->status.empty()) {
          last_diagnostics_ = msg->status.front();
          has_diagnostics_ = true;
        }
      });

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 4);
    executor_->add_node(node_);
    executor_->add_node(probe_);
    spin_thread_ = std::thread([this]() {executor_->spin();});
  }

  ~Harness() {shutdownNode();}
  Harness(const Harness &) = delete;
  Harness & operator=(const Harness &) = delete;

  /// Cancels the executor and joins BEFORE dropping the node -- an in-flight
  /// callback on another executor thread outliving the node is a
  /// use-after-free race (same discipline as the other two P10 node tests).
  void shutdownNode()
  {
    if (!executor_) {
      return;
    }
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    node_.reset();
    probe_.reset();
    executor_.reset();
  }

  rclcpp::Node::SharedPtr probe() {return probe_;}
  const std::string & prefix() const {return prefix_;}

  bool waitForDiagnostics(double ceiling_seconds)
  {
    return waitFor(
      [this]() {std::lock_guard<std::mutex> lock(mutex_); return has_diagnostics_;},
      ceiling_seconds);
  }

  DiagnosticStatus lastDiagnostics()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_diagnostics_;
  }

private:
  std::string prefix_;
  std::shared_ptr<EventLoggerNode> node_;
  rclcpp::Node::SharedPtr probe_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr diagnostics_sub_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;

  std::mutex mutex_;
  DiagnosticStatus last_diagnostics_;
  bool has_diagnostics_ = false;
};

std::string nextUavId()
{
  static std::atomic<int> counter{0};
  return "uav" + std::to_string(counter.fetch_add(1));
}

std::vector<rclcpp::Parameter> baseParams(const fs::path & log_root)
{
  return {
    rclcpp::Parameter("log_root", log_root.string()),
    rclcpp::Parameter("fsync_period_sec", 2.0),
  };
}

void overrideParam(std::vector<rclcpp::Parameter> & params, const rclcpp::Parameter & replacement)
{
  for (rclcpp::Parameter & param : params) {
    if (param.get_name() == replacement.get_name()) {
      param = replacement;
      return;
    }
  }
  params.push_back(replacement);
}

bool waitForLinesWritten(Harness & harness, double expected, double ceiling_seconds)
{
  return waitFor(
    [&harness, expected]() {
      return diagnosticNum(harness.lastDiagnostics(), "lines_written") >= expected;
    }, ceiling_seconds);
}

bool waitForLinesDropped(Harness & harness, double expected, double ceiling_seconds)
{
  return waitFor(
    [&harness, expected]() {
      return diagnosticNum(harness.lastDiagnostics(), "lines_dropped") >= expected;
    }, ceiling_seconds);
}

}  // namespace

// ===========================================================================
// mission/events (header point 4): EVERY message is its own event, even
// three back-to-back publishes with byte-identical content.
// ===========================================================================

TEST(EventLoggerNode, MissionEventsSourceLogsEveryMessageEvenWhenIdentical)
{
  TempDir log_root("mission_events");
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams(log_root.path()));

  auto pub = harness.probe()->create_publisher<MissionEvent>(
    harness.prefix() + "/mission/events", rclcpp::QoS(50).reliable());
  ASSERT_TRUE(waitFor([&]() {return pub->get_subscription_count() > 0;}, 10.0))
    << "FAILED TO MEASURE: node never subscribed to mission/events";

  MissionEvent msg;
  msg.event_type = MissionEvent::EVENT_STEP_STARTED;
  msg.mission_id = "m1";
  msg.step_name = "goto_wp1";
  for (int i = 0; i < 3; ++i) {
    pub->publish(msg);
    std::this_thread::sleep_for(20ms);
  }

  ASSERT_TRUE(waitForLinesWritten(harness, 3.0, 5.0))
    << "FAILED TO MEASURE: expected 3 lines from 3 identical mission events";

  const std::string log_path = diagnosticStr(harness.lastDiagnostics(), "file_path");
  harness.shutdownNode();
  const auto lines = readAllLines(log_path);
  EXPECT_GE(lines.size(), 3u);
}

// ===========================================================================
// mission/status: state + current_step_index are independent edges; a
// repeated identical message adds nothing (positive control: a real change
// does).
// ===========================================================================

TEST(EventLoggerNode, MissionStatusSourceDedupesRepeatsThenLogsChange)
{
  TempDir log_root("mission_status");
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams(log_root.path()));

  auto pub = harness.probe()->create_publisher<MissionStatus>(
    harness.prefix() + "/mission/status", rclcpp::QoS(1).reliable().transient_local());
  ASSERT_TRUE(waitFor([&]() {return pub->get_subscription_count() > 0;}, 10.0))
    << "FAILED TO MEASURE: node never subscribed to mission/status";

  MissionStatus msg;
  msg.state = MissionStatus::STATE_RUNNING;
  msg.mission_id = "m1";
  msg.current_step_index = 0;
  msg.current_step_name = "wp0";

  pub->publish(msg);   // first sighting: 2 lines (state + current_step_index)
  std::this_thread::sleep_for(50ms);
  for (int i = 0; i < 5; ++i) {
    pub->publish(msg);   // byte-identical repeats: 0 new lines each
    std::this_thread::sleep_for(20ms);
  }
  msg.state = MissionStatus::STATE_PAUSED;   // positive control: real change
  pub->publish(msg);

  ASSERT_TRUE(waitForLinesWritten(harness, 3.0, 5.0))
    << "FAILED TO MEASURE: expected exactly 3 lines (2 first-sighting + 1 change)";
  std::this_thread::sleep_for(300ms);   // let any stray extra writes settle
  EXPECT_EQ(diagnosticNum(harness.lastDiagnostics(), "lines_written"), 3.0)
    << "the 5 identical repeats must not have added a single extra line";
}

// ===========================================================================
// safety/state: level + recovery_active are independent edges.
// ===========================================================================

TEST(EventLoggerNode, SafetyStateSourceLevelAndRecoveryActiveAreIndependentEdges)
{
  TempDir log_root("safety_state");
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams(log_root.path()));

  auto pub = harness.probe()->create_publisher<SafetyState>(
    harness.prefix() + "/safety/state", rclcpp::QoS(1).reliable().transient_local());
  ASSERT_TRUE(waitFor([&]() {return pub->get_subscription_count() > 0;}, 10.0))
    << "FAILED TO MEASURE: node never subscribed to safety/state";

  SafetyState msg;
  msg.level = SafetyState::LEVEL_OK;
  msg.recovery_active = false;
  pub->publish(msg);   // first sighting: 2 lines
  std::this_thread::sleep_for(50ms);

  msg.level = SafetyState::LEVEL_EMERGENCY;   // level edge only
  pub->publish(msg);
  std::this_thread::sleep_for(50ms);

  msg.recovery_active = true;   // recovery_active edge only
  pub->publish(msg);

  ASSERT_TRUE(waitForLinesWritten(harness, 4.0, 5.0))
    << "FAILED TO MEASURE: expected 4 lines (2 first-sighting + level + recovery_active)";

  const std::string log_path = diagnosticStr(harness.lastDiagnostics(), "file_path");
  harness.shutdownNode();
  const auto lines = readAllLines(log_path);
  bool saw_emergency = false;
  for (const std::string & line : lines) {
    if (jsonStringField(line, "field") == "level" && jsonStringField(line, "to") == "EMERGENCY") {
      saw_emergency = true;
    }
  }
  EXPECT_TRUE(saw_emergency);
}

// ===========================================================================
// (c) ERROR-level event: already on disk right after publish+spin, read
// back by a SEPARATE file handle while the node still holds its own fd
// open -- evidence the fsync path actually ran (needsFsync() itself is
// pinned by the pure-lib test; this is the node-integration side of it).
// ===========================================================================

TEST(EventLoggerNode, ErrorLevelEventIsOnDiskRightAfterPublishAndSpin)
{
  TempDir log_root("error_fsync");
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams(log_root.path()));

  auto pub = harness.probe()->create_publisher<OffboardStatus>(
    harness.prefix() + "/backend/offboard_status", rclcpp::QoS(10));
  ASSERT_TRUE(waitFor([&]() {return pub->get_subscription_count() > 0;}, 10.0))
    << "FAILED TO MEASURE: node never subscribed to backend/offboard_status";

  OffboardStatus msg;
  msg.state = OffboardStatus::STATE_FAULT;
  msg.detail = "stream lost";
  pub->publish(msg);

  ASSERT_TRUE(waitForLinesWritten(harness, 1.0, 5.0))
    << "FAILED TO MEASURE: FAULT event never logged";

  const DiagnosticStatus diag = harness.lastDiagnostics();
  EXPECT_EQ(diagnosticNum(diag, "fsync_errors"), 0.0);

  const std::string log_path = diagnosticStr(diag, "file_path");
  const auto lines = readAllLines(log_path);   // node still running, fd still open
  bool saw_fault = false;
  for (const std::string & line : lines) {
    if (jsonStringField(line, "level") == "ERROR" && jsonStringField(line, "to") == "FAULT") {
      saw_fault = true;
    }
  }
  EXPECT_TRUE(saw_fault) << "ERROR-level FAULT event not found on disk after publish+spin";
}

// ===========================================================================
// safety/violations: appear, level-change, and clear (header point 5) are
// all distinct edges, keyed by violation name.
// ===========================================================================

TEST(EventLoggerNode, SafetyViolationsSourceLogsAppearLevelChangeAndClear)
{
  TempDir log_root("safety_violations");
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams(log_root.path()));

  auto pub = harness.probe()->create_publisher<DiagnosticArray>(
    harness.prefix() + "/safety/violations", rclcpp::QoS(20).reliable());
  ASSERT_TRUE(waitFor([&]() {return pub->get_subscription_count() > 0;}, 10.0))
    << "FAILED TO MEASURE: node never subscribed to safety/violations";

  DiagnosticArray array;
  DiagnosticStatus status;
  status.name = "OBSTACLE_TOO_CLOSE";
  status.level = DiagnosticStatus::WARN;
  status.message = "0.4m < 0.5m";
  array.status.push_back(status);
  pub->publish(array);   // appear: 1 line
  std::this_thread::sleep_for(50ms);

  array.status[0].level = DiagnosticStatus::ERROR;   // level change: 1 line
  pub->publish(array);
  std::this_thread::sleep_for(50ms);

  array.status.clear();   // clear: 1 line
  pub->publish(array);

  ASSERT_TRUE(waitForLinesWritten(harness, 3.0, 5.0))
    << "FAILED TO MEASURE: expected appear + level-change + clear = 3 lines";

  const std::string log_path = diagnosticStr(harness.lastDiagnostics(), "file_path");
  harness.shutdownNode();
  const auto lines = readAllLines(log_path);
  bool saw_clear = false;
  for (const std::string & line : lines) {
    if (jsonStringField(line, "field") == "OBSTACLE_TOO_CLOSE" &&
      jsonStringField(line, "to") == "CLEARED")
    {
      saw_clear = true;
    }
  }
  EXPECT_TRUE(saw_clear);
}

// ===========================================================================
// control/authority, state/estimator_source, state/localization_status,
// state/vehicle, diagnostics/aggregated: one edge test each (minimum bar).
// ===========================================================================

TEST(EventLoggerNode, AuthoritySourceLogsActiveSourceChange)
{
  TempDir log_root("authority");
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams(log_root.path()));

  auto pub = harness.probe()->create_publisher<ControlAuthority>(
    harness.prefix() + "/control/authority", rclcpp::QoS(1).reliable().transient_local());
  ASSERT_TRUE(waitFor([&]() {return pub->get_subscription_count() > 0;}, 10.0))
    << "FAILED TO MEASURE: node never subscribed to control/authority";

  ControlAuthority msg;
  msg.active_source = ControlAuthority::SOURCE_MISSION;
  pub->publish(msg);
  std::this_thread::sleep_for(50ms);
  msg.active_source = ControlAuthority::SOURCE_SAFETY;
  pub->publish(msg);

  ASSERT_TRUE(waitForLinesWritten(harness, 2.0, 5.0))
    << "FAILED TO MEASURE: expected 2 lines (first sighting + active_source change)";

  const std::string log_path = diagnosticStr(harness.lastDiagnostics(), "file_path");
  harness.shutdownNode();
  const auto lines = readAllLines(log_path);
  bool saw_safety = false;
  for (const std::string & line : lines) {
    if (jsonStringField(line, "to") == "SAFETY") {saw_safety = true;}
  }
  EXPECT_TRUE(saw_safety);
}

TEST(EventLoggerNode, EstimatorSourceValueChangeIsLogged)
{
  TempDir log_root("estimator_source");
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams(log_root.path()));

  auto pub = harness.probe()->create_publisher<std_msgs::msg::String>(
    harness.prefix() + "/state/estimator_source", rclcpp::QoS(10).transient_local());
  ASSERT_TRUE(waitFor([&]() {return pub->get_subscription_count() > 0;}, 10.0))
    << "FAILED TO MEASURE: node never subscribed to state/estimator_source";

  std_msgs::msg::String msg;
  msg.data = "GPS";
  pub->publish(msg);
  std::this_thread::sleep_for(50ms);
  msg.data = "VIO";
  pub->publish(msg);

  ASSERT_TRUE(waitForLinesWritten(harness, 2.0, 5.0))
    << "FAILED TO MEASURE: expected 2 lines (first sighting + value change)";
}

TEST(EventLoggerNode, LocalizationStatusSourceIsValidAndActiveSourceAreIndependentEdges)
{
  TempDir log_root("localization_status");
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams(log_root.path()));

  auto pub = harness.probe()->create_publisher<LocalizationStatus>(
    harness.prefix() + "/state/localization_status", rclcpp::QoS(10));
  ASSERT_TRUE(waitFor([&]() {return pub->get_subscription_count() > 0;}, 10.0))
    << "FAILED TO MEASURE: node never subscribed to state/localization_status";

  LocalizationStatus msg;
  msg.is_valid = true;
  msg.active_source = LocalizationStatus::SOURCE_GPS;
  pub->publish(msg);   // first sighting: 2 lines
  std::this_thread::sleep_for(50ms);
  msg.is_valid = false;   // is_valid edge only
  pub->publish(msg);
  std::this_thread::sleep_for(50ms);
  msg.active_source = LocalizationStatus::SOURCE_VIO;   // active_source edge only
  pub->publish(msg);

  ASSERT_TRUE(waitForLinesWritten(harness, 4.0, 5.0))
    << "FAILED TO MEASURE: expected 4 lines (2 first-sighting + is_valid + active_source)";
}

TEST(EventLoggerNode, VehicleSourceArmedAndFlightModeAreIndependentEdges)
{
  TempDir log_root("vehicle");
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams(log_root.path()));

  auto pub = harness.probe()->create_publisher<VehicleState>(
    harness.prefix() + "/state/vehicle", rclcpp::QoS(10));
  ASSERT_TRUE(waitFor([&]() {return pub->get_subscription_count() > 0;}, 10.0))
    << "FAILED TO MEASURE: node never subscribed to state/vehicle";

  VehicleState msg;
  msg.armed = false;
  msg.flight_mode = VehicleState::FLIGHT_MODE_POSITION;
  pub->publish(msg);   // first sighting: 2 lines
  std::this_thread::sleep_for(50ms);
  msg.armed = true;   // armed edge only
  pub->publish(msg);
  std::this_thread::sleep_for(50ms);
  msg.flight_mode = VehicleState::FLIGHT_MODE_OFFBOARD;   // flight_mode edge only
  pub->publish(msg);

  ASSERT_TRUE(waitForLinesWritten(harness, 4.0, 5.0))
    << "FAILED TO MEASURE: expected 4 lines (2 first-sighting + armed + flight_mode)";
}

TEST(EventLoggerNode, AggregatedSourceLogsSystemLevelAndGoNoGoIndependently)
{
  TempDir log_root("aggregated");
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams(log_root.path()));

  auto pub = harness.probe()->create_publisher<DiagnosticArray>(
    harness.prefix() + "/diagnostics/aggregated", rclcpp::QoS(10));
  ASSERT_TRUE(waitFor([&]() {return pub->get_subscription_count() > 0;}, 10.0))
    << "FAILED TO MEASURE: node never subscribed to diagnostics/aggregated";

  DiagnosticArray array;
  DiagnosticStatus system_status;
  system_status.name = "system";
  system_status.level = DiagnosticStatus::OK;
  KeyValue go_no_go;
  go_no_go.key = "go_no_go";
  go_no_go.value = "GO";
  system_status.values.push_back(go_no_go);
  array.status.push_back(system_status);
  pub->publish(array);   // first sighting: 2 lines (level + go_no_go)
  std::this_thread::sleep_for(50ms);

  array.status[0].level = DiagnosticStatus::ERROR;
  array.status[0].values[0].value = "NO_GO";
  pub->publish(array);   // both change: 2 lines

  ASSERT_TRUE(waitForLinesWritten(harness, 4.0, 5.0))
    << "FAILED TO MEASURE: expected 4 lines total (2 first-sighting + level + go_no_go)";
}

// ===========================================================================
// P10.9b (design panel S:4.a D5, docs/interface-contract-v0.1.md S:2.20):
// NO_GO logs at ERROR (must fsync before this callback returns -- the ONE
// go_no_go value that must survive a process crash); worst_item/gate_mode
// piggyback on the same aggregated "system" status.
// ===========================================================================

TEST(EventLoggerNode, AggregatedNoGoLogsAtErrorWithWorstItemAndGateMode)
{
  TempDir log_root("aggregated_no_go_error");
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams(log_root.path()));

  auto pub = harness.probe()->create_publisher<DiagnosticArray>(
    harness.prefix() + "/diagnostics/aggregated", rclcpp::QoS(10));
  ASSERT_TRUE(waitFor([&]() {return pub->get_subscription_count() > 0;}, 10.0))
    << "FAILED TO MEASURE: node never subscribed to diagnostics/aggregated";

  DiagnosticArray array;
  DiagnosticStatus system_status;
  system_status.name = "system";
  system_status.level = DiagnosticStatus::ERROR;
  KeyValue go_no_go;
  go_no_go.key = "go_no_go";
  go_no_go.value = "NO_GO";
  KeyValue worst_item;
  worst_item.key = "worst_item";
  worst_item.value = "diagnostics/safety:safety: OFFBOARD_UNHEALTHY";
  KeyValue gate_mode;
  gate_mode.key = "gate_mode";
  gate_mode.value = "flight";
  system_status.values.push_back(go_no_go);
  system_status.values.push_back(worst_item);
  system_status.values.push_back(gate_mode);
  array.status.push_back(system_status);
  pub->publish(array);

  ASSERT_TRUE(waitForLinesWritten(harness, 4.0, 5.0))
    << "FAILED TO MEASURE: expected 4 lines (level + go_no_go + worst_item + gate_mode)";

  const DiagnosticStatus diag = harness.lastDiagnostics();
  const std::string log_path = diagnosticStr(diag, "file_path");
  const auto lines = readAllLines(log_path);   // node still running, fd still open
  bool saw_no_go_error = false;
  bool saw_worst_item = false;
  bool saw_gate_mode = false;
  for (const std::string & line : lines) {
    if (jsonStringField(line, "field") == "go_no_go" && jsonStringField(line, "to") == "NO_GO") {
      EXPECT_EQ(jsonStringField(line, "level"), "ERROR")
        << "NO_GO must log at ERROR (fsync'd before the callback returns)";
      saw_no_go_error = true;
    }
    if (jsonStringField(line, "field") == "worst_item") {
      EXPECT_EQ(jsonStringField(line, "to"), "diagnostics/safety:safety: OFFBOARD_UNHEALTHY");
      saw_worst_item = true;
    }
    if (jsonStringField(line, "field") == "gate_mode") {
      EXPECT_EQ(jsonStringField(line, "to"), "flight");
      saw_gate_mode = true;
    }
  }
  EXPECT_TRUE(saw_no_go_error) << "go_no_go=NO_GO edge not found on disk";
  EXPECT_TRUE(saw_worst_item) << "worst_item edge not found on disk";
  EXPECT_TRUE(saw_gate_mode) << "gate_mode edge not found on disk";
}

// ===========================================================================
// (b) seq is a single counter, continuous across DIFFERENT sources -- each
// publish is confirmed processed (via lines_written) before the next one,
// so file order == publish order.
// ===========================================================================

TEST(EventLoggerNode, SeqIsContinuousAcrossDifferentSources)
{
  TempDir log_root("seq_continuity");
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams(log_root.path()));

  auto estimator_pub = harness.probe()->create_publisher<std_msgs::msg::String>(
    harness.prefix() + "/state/estimator_source", rclcpp::QoS(10).transient_local());
  auto authority_pub = harness.probe()->create_publisher<ControlAuthority>(
    harness.prefix() + "/control/authority", rclcpp::QoS(1).reliable().transient_local());
  auto vehicle_pub = harness.probe()->create_publisher<VehicleState>(
    harness.prefix() + "/state/vehicle", rclcpp::QoS(10));

  ASSERT_TRUE(
    waitFor(
      [&]() {
        return estimator_pub->get_subscription_count() > 0 &&
        authority_pub->get_subscription_count() > 0 && vehicle_pub->get_subscription_count() > 0;
      }, 10.0))
    << "FAILED TO MEASURE: node never subscribed to all 3 topics";

  std_msgs::msg::String estimator_msg;
  estimator_msg.data = "GPS";
  estimator_pub->publish(estimator_msg);
  ASSERT_TRUE(waitForLinesWritten(harness, 1.0, 5.0)) << "FAILED TO MEASURE: seq 0 never landed";

  ControlAuthority authority_msg;
  authority_msg.active_source = ControlAuthority::SOURCE_MISSION;
  authority_pub->publish(authority_msg);
  ASSERT_TRUE(waitForLinesWritten(harness, 2.0, 5.0)) << "FAILED TO MEASURE: seq 1 never landed";

  VehicleState vehicle_msg;
  vehicle_msg.armed = true;
  vehicle_msg.flight_mode = VehicleState::FLIGHT_MODE_OFFBOARD;
  vehicle_pub->publish(vehicle_msg);   // this single message emits 2 lines (armed + flight_mode)
  ASSERT_TRUE(waitForLinesWritten(harness, 4.0, 5.0)) << "FAILED TO MEASURE: seq 2/3 never landed";

  const std::string log_path = diagnosticStr(harness.lastDiagnostics(), "file_path");
  harness.shutdownNode();
  const auto lines = readAllLines(log_path);
  ASSERT_EQ(lines.size(), 4u);
  for (size_t i = 0; i < lines.size(); ++i) {
    EXPECT_EQ(jsonIntField(lines[i], "seq"), static_cast<long long>(i))
      << "line " << i << ": " << lines[i];
  }
}

// ===========================================================================
// (d) log_root pointing at an existing REGULAR FILE (not a directory) ->
// open() fails at boot, node stays alive, diagnostics reports DEGRADED, and
// every edge from then on counts as lines_dropped instead of writing.
// ===========================================================================

TEST(EventLoggerNode, LogRootPointingAtARegularFileDropsLinesAndReportsDegraded)
{
  TempDir parent("degraded_parent");
  const fs::path regular_file = parent.path() / "not_a_directory";
  {
    std::ofstream file(regular_file.string());
    file << "x";
  }

  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams(regular_file));   // log_root IS a plain file

  ASSERT_TRUE(harness.waitForDiagnostics(5.0)) << "FAILED TO MEASURE: no diagnostics tick";
  EXPECT_EQ(diagnosticStr(harness.lastDiagnostics(), "state"), "DEGRADED");

  auto pub = harness.probe()->create_publisher<std_msgs::msg::String>(
    harness.prefix() + "/state/estimator_source", rclcpp::QoS(10).transient_local());
  ASSERT_TRUE(waitFor([&]() {return pub->get_subscription_count() > 0;}, 10.0))
    << "FAILED TO MEASURE: node never subscribed to state/estimator_source";
  std_msgs::msg::String msg;
  msg.data = "GPS";
  pub->publish(msg);

  ASSERT_TRUE(waitForLinesDropped(harness, 1.0, 5.0))
    << "FAILED TO MEASURE: an edge with no open file must still count as lines_dropped";
}

// ===========================================================================
// (e) validate(): one test per rule -- each mutates ONE field off a known-
// good base and expects the constructor to refuse to start. A positive
// control proves the base config itself is NOT the thing throwing (R33).
// ===========================================================================

class EventLoggerValidateTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    log_root_ = std::make_unique<TempDir>("validate");
    params_ = baseParams(log_root_->path());
  }

  std::unique_ptr<TempDir> log_root_;
  std::vector<rclcpp::Parameter> params_;
};

TEST_F(EventLoggerValidateTest, KnownGoodConfigDoesNotThrow)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_NO_THROW({EventLoggerNode node(options);});
}

TEST_F(EventLoggerValidateTest, ZeroFsyncPeriodThrows)
{
  overrideParam(params_, rclcpp::Parameter("fsync_period_sec", 0.0));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({EventLoggerNode node(options);}, std::invalid_argument);
}

TEST_F(EventLoggerValidateTest, NegativeFsyncPeriodThrows)
{
  overrideParam(params_, rclcpp::Parameter("fsync_period_sec", -1.0));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({EventLoggerNode node(options);}, std::invalid_argument);
}

TEST_F(EventLoggerValidateTest, NanFsyncPeriodThrows)
{
  overrideParam(
    params_, rclcpp::Parameter("fsync_period_sec", std::numeric_limits<double>::quiet_NaN()));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({EventLoggerNode node(options);}, std::invalid_argument);
}

TEST_F(EventLoggerValidateTest, EmptyLogRootThrows)
{
  overrideParam(params_, rclcpp::Parameter("log_root", std::string("")));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({EventLoggerNode node(options);}, std::invalid_argument);
}

// ===========================================================================
// (f) every emitted line is valid JSONL: fixed key order, no raw newline
// leaked through the escaping path, on real messages carrying quotes/
// newlines in string fields (integration-level complement to
// test_event_ledger.cpp's escaping unit tests).
// ===========================================================================

TEST(EventLoggerNode, EveryEmittedLineIsValidJsonlWithFixedKeyOrder)
{
  TempDir log_root("jsonl_validity");
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams(log_root.path()));

  auto estimator_pub = harness.probe()->create_publisher<std_msgs::msg::String>(
    harness.prefix() + "/state/estimator_source", rclcpp::QoS(10).transient_local());
  auto violations_pub = harness.probe()->create_publisher<DiagnosticArray>(
    harness.prefix() + "/safety/violations", rclcpp::QoS(20).reliable());
  ASSERT_TRUE(
    waitFor(
      [&]() {
        return estimator_pub->get_subscription_count() > 0 &&
        violations_pub->get_subscription_count() > 0;
      }, 10.0))
    << "FAILED TO MEASURE: node never subscribed to both topics";

  std_msgs::msg::String estimator_msg;
  estimator_msg.data = "GPS \"quoted\"\nvalue";
  estimator_pub->publish(estimator_msg);

  DiagnosticArray array;
  DiagnosticStatus status;
  status.name = "TEST_CODE";
  status.level = DiagnosticStatus::WARN;
  status.message = "detail with \"quotes\"";
  array.status.push_back(status);
  violations_pub->publish(array);

  ASSERT_TRUE(waitForLinesWritten(harness, 2.0, 5.0)) << "FAILED TO MEASURE: expected 2 lines";

  const std::string log_path = diagnosticStr(harness.lastDiagnostics(), "file_path");
  harness.shutdownNode();
  const auto lines = readAllLines(log_path);
  ASSERT_GE(lines.size(), 2u);
  for (const std::string & line : lines) {
    EXPECT_TRUE(hasFixedKeyOrder(line)) << "malformed line: " << line;
    EXPECT_EQ(line.find('\n'), std::string::npos) << "raw newline leaked into a JSONL line";
  }
}

}  // namespace uav_observability
