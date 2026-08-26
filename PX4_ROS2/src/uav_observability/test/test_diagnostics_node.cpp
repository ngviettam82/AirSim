// P10.4: the real node, driven through real topics by a probe (R20, hence
// the isolated domain ENV ROS_DOMAIN_ID=99, see CMakeLists.txt). Every wait
// is event-anchored (waitFor + a hang ceiling), never a bare sleep (R21).
//
// P10.9b (go_no_go redesign, .claude/plan/P10-gonogo-design-panel.md S:4):
// baseParams() now REQUIRES an "state/vehicle" always-item (the phase
// predicate's own freshness source) -- every test below that expects GO
// runs a VehicleTicker to keep it fresh, otherwise state/vehicle alone
// reads UNKNOWN forever and go_no_go can never leave UNKNOWN, independent
// of anything else under test. G3 (folding in_air) was RULED OUT by the
// calling flow after verifying VehicleState.in_air never resets after the
// first flight (px4_state_adapter_node.cpp: in_air = takeoff_time > 0) --
// the phase predicate below uses `armed` alone, so there is no
// InAirAloneForcesFlightPhase test (it would test a feature that does not
// exist by design).
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <uav_interfaces/msg/vehicle_state.hpp>

#include "uav_observability/diagnostics_node.hpp"

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using uav_interfaces::msg::VehicleState;
using namespace std::chrono_literals;

namespace uav_observability
{
namespace
{

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

const DiagnosticStatus * findStatus(const DiagnosticArray & array, const std::string & name)
{
  for (const DiagnosticStatus & status : array.status) {
    if (status.name == name) {
      return &status;
    }
  }
  return nullptr;
}

/// Republishes a std_msgs::String on `topic` every `period` until stopped --
/// the background "this source is alive" ticker every GO/NO_GO test needs.
class Ticker
{
public:
  Ticker(rclcpp::Node::SharedPtr node, const std::string & topic, std::chrono::milliseconds period)
  : period_(period)
  {
    publisher_ = node->create_publisher<std_msgs::msg::String>(topic, rclcpp::QoS(10));
    thread_ = std::thread([this]() {run();});
  }
  ~Ticker() {stop();}
  Ticker(const Ticker &) = delete;
  Ticker & operator=(const Ticker &) = delete;

  void stop()
  {
    stop_.store(true);
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  void run()
  {
    std_msgs::msg::String msg;
    msg.data = "tick";
    while (!stop_.load()) {
      publisher_->publish(msg);
      std::this_thread::sleep_for(period_);
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::chrono::milliseconds period_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
};

/// One fake Sub-B child (P10.9b: DiagTicker upgraded from a single fixed
/// "child" status to N of these, each with its own action KeyValue --
/// needed to drive NW1/NW2/waiver-table scenarios).
struct FakeChild
{
  std::string name;
  uint8_t level;
  std::string action;   // "" -- no "action" KeyValue published at all
};

/// Republishes a DiagnosticArray (N children, given level/action each)
/// every `period` until stopped -- Sub-B's positive-control ticker.
class DiagTicker
{
public:
  /// Legacy convenience ctor -- one child named "child", no action key.
  /// Delegates to the multi-child ctor below.
  DiagTicker(
    rclcpp::Node::SharedPtr node, const std::string & topic, uint8_t level,
    std::chrono::milliseconds period)
  : DiagTicker(node, topic, std::vector<FakeChild>{{"child", level, ""}}, period) {}

  DiagTicker(
    rclcpp::Node::SharedPtr node, const std::string & topic,
    const std::vector<FakeChild> & children, std::chrono::milliseconds period)
  : period_(period)
  {
    publisher_ = node->create_publisher<DiagnosticArray>(topic, rclcpp::QoS(10));
    for (const FakeChild & child : children) {
      DiagnosticStatus status;
      status.name = child.name;
      status.level = child.level;
      if (!child.action.empty()) {
        KeyValue kv;
        kv.key = "action";
        kv.value = child.action;
        status.values.push_back(kv);
      }
      message_.status.push_back(status);
    }
    thread_ = std::thread([this]() {run();});
  }
  ~DiagTicker() {stop();}
  DiagTicker(const DiagTicker &) = delete;
  DiagTicker & operator=(const DiagTicker &) = delete;

  void stop()
  {
    stop_.store(true);
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  void run()
  {
    while (!stop_.load()) {
      publisher_->publish(message_);
      std::this_thread::sleep_for(period_);
    }
  }

  rclcpp::Publisher<DiagnosticArray>::SharedPtr publisher_;
  DiagnosticArray message_;
  std::chrono::milliseconds period_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
};

/// P10.9b: republishes VehicleState on `topic` every `period` until
/// stopped, `armed` settable live from the test thread -- the phase
/// predicate's own positive-control ticker (there is no more "gate_mode"
/// param to hand-set; every phase test drives this instead).
class VehicleTicker
{
public:
  VehicleTicker(
    rclcpp::Node::SharedPtr node, const std::string & topic, std::chrono::milliseconds period)
  : period_(period)
  {
    publisher_ = node->create_publisher<VehicleState>(topic, rclcpp::QoS(10));
    thread_ = std::thread([this]() {run();});
  }
  ~VehicleTicker() {stop();}
  VehicleTicker(const VehicleTicker &) = delete;
  VehicleTicker & operator=(const VehicleTicker &) = delete;

  void setArmed(bool armed) {armed_.store(armed);}

  /// N1 (review lượt 2): defaults to true -- every EXISTING test in this
  /// file predates the `connected_` gate and never touches this field, so
  /// the default must keep meaning "healthy link" (px4_state_adapter_node
  /// only ever reports armed=false while disconnected, never the reverse --
  /// see the test below that exercises the actual failure direction).
  void setConnected(bool connected) {connected_.store(connected);}

  void stop()
  {
    stop_.store(true);
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  void run()
  {
    while (!stop_.load()) {
      VehicleState msg;
      msg.connected = connected_.load();
      msg.armed = armed_.load();
      publisher_->publish(msg);
      std::this_thread::sleep_for(period_);
    }
  }

  rclcpp::Publisher<VehicleState>::SharedPtr publisher_;
  std::chrono::milliseconds period_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> armed_{false};
  std::atomic<bool> connected_{true};
};

/// Node + probe + spinning executor + taps on aggregated/system_health.
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
    node_ = std::make_shared<DiagnosticsNode>(options);

    probe_ = std::make_shared<rclcpp::Node>("diagnostics_probe_" + uav_id);
    system_health_sub_ = probe_->create_subscription<DiagnosticStatus>(
      prefix_ + "/state/system_health", rclcpp::QoS(1).reliable().transient_local(),
      [this](const DiagnosticStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_system_health_ = *msg;
        has_system_health_ = true;
      });
    aggregated_sub_ = probe_->create_subscription<DiagnosticArray>(
      prefix_ + "/diagnostics/aggregated", rclcpp::QoS(10),
      [this](const DiagnosticArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_aggregated_ = *msg;
        has_aggregated_ = true;
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
  /// use-after-free race (same discipline as test_rosbag_manager_node.cpp).
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

  /// P10.9b: replaces the old direct "gate_mode" write -- this only ever
  /// TIGHTENS the measured phase (auto|preflight are functionally
  /// identical no-ops; only "flight" forces anything).
  void setGateModeOverride(const std::string & mode)
  {
    node_->set_parameter(rclcpp::Parameter("gate_mode_override", mode));
  }

  bool waitForSystemHealth(double ceiling_seconds)
  {
    return waitFor(
      [this]() {std::lock_guard<std::mutex> lock(mutex_); return has_system_health_;},
      ceiling_seconds);
  }

  bool waitForGoNoGo(const std::string & expected, double ceiling_seconds)
  {
    return waitFor(
      [this, &expected]() {
        std::lock_guard<std::mutex> lock(mutex_);
        return has_system_health_ && diagnosticStr(last_system_health_, "go_no_go") == expected;
      }, ceiling_seconds);
  }

  DiagnosticStatus lastSystemHealth()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_system_health_;
  }

  bool waitForAggregated(double ceiling_seconds)
  {
    return waitFor(
      [this]() {std::lock_guard<std::mutex> lock(mutex_); return has_aggregated_;},
      ceiling_seconds);
  }

  DiagnosticArray lastAggregated()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_aggregated_;
  }

private:
  std::string prefix_;
  std::shared_ptr<DiagnosticsNode> node_;
  rclcpp::Node::SharedPtr probe_;
  rclcpp::Subscription<DiagnosticStatus>::SharedPtr system_health_sub_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr aggregated_sub_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;

  std::mutex mutex_;
  DiagnosticStatus last_system_health_;
  bool has_system_health_ = false;
  DiagnosticArray last_aggregated_;
  bool has_aggregated_ = false;
};

std::string nextUavId()
{
  static std::atomic<int> counter{0};
  return "uav" + std::to_string(counter.fetch_add(1));
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

/// 2 always items (0.3 s timeout) + "state/vehicle" (P10.9b's phase
/// predicate freshness source, ALSO 0.3 s timeout -- review lượt 2, N2: now
/// a SENTINEL row, topic/type both "", fed exclusively by the dedicated
/// typed subscription/onVehicleState(), same shape as the real yaml) + 1
/// flight_only item (0.3 s timeout) + 1 Sub-B source ("diagnostics/safety",
/// action_aware like the real one) -- small and fast, every test starts
/// here and overrides only what it needs (rosbag_manager test precedent).
/// waiver_* arrays default to empty (declare_parameter's own default) --
/// tests that need waivers add them via overrideParam().
std::vector<rclcpp::Parameter> baseParams()
{
  return {
    rclcpp::Parameter("gate_mode_override", std::string("auto")),
    rclcpp::Parameter("report_period_sec", 0.05),
    rclcpp::Parameter("publish_period_sec", 0.2),
    rclcpp::Parameter("sub_b_stale_after_sec", 2.0),   // V-O1 floor given the formal expected_hz

    rclcpp::Parameter(
      "always_names", std::vector<std::string>{"always_a", "always_b", "state/vehicle"}),
    rclcpp::Parameter(
      "always_topics", std::vector<std::string>{"test/always_a", "test/always_b", ""}),
    rclcpp::Parameter(
      "always_types",
      std::vector<std::string>{"std_msgs/msg/String", "std_msgs/msg/String", ""}),
    rclcpp::Parameter("always_qos", std::vector<std::string>{"be", "be", "be"}),
    rclcpp::Parameter(
      "always_depths", std::vector<int64_t>{10, 10, 10}),
    rclcpp::Parameter("always_expected_hz", std::vector<double>{10.0, 10.0, 10.0}),
    rclcpp::Parameter("always_timeout_sec", std::vector<double>{0.3, 0.3, 0.3}),
    rclcpp::Parameter(
      "always_level_when_missing", std::vector<std::string>{"error", "error", "error"}),
    rclcpp::Parameter("always_owner", std::vector<std::string>{"", "", ""}),

    rclcpp::Parameter("flight_only_names", std::vector<std::string>{"flight_a"}),
    rclcpp::Parameter("flight_only_topics", std::vector<std::string>{"test/flight_a"}),
    rclcpp::Parameter("flight_only_types", std::vector<std::string>{"std_msgs/msg/String"}),
    rclcpp::Parameter("flight_only_qos", std::vector<std::string>{"be"}),
    rclcpp::Parameter("flight_only_depths", std::vector<int64_t>{10}),
    rclcpp::Parameter("flight_only_expected_hz", std::vector<double>{10.0}),
    rclcpp::Parameter("flight_only_timeout_sec", std::vector<double>{0.3}),
    rclcpp::Parameter("flight_only_level_when_missing", std::vector<std::string>{"error"}),
    rclcpp::Parameter("flight_only_owner", std::vector<std::string>{""}),

    rclcpp::Parameter(
      "diag_source_names", std::vector<std::string>{"diagnostics/safety"}),
    rclcpp::Parameter(
      "diag_source_topics", std::vector<std::string>{"diagnostics/safety"}),
    rclcpp::Parameter(
      "diag_source_types", std::vector<std::string>{"diagnostic_msgs/msg/DiagnosticArray"}),
    rclcpp::Parameter("diag_source_qos", std::vector<std::string>{"be"}),
    rclcpp::Parameter("diag_source_depths", std::vector<int64_t>{10}),
    rclcpp::Parameter("diag_source_owner", std::vector<std::string>{""}),
    rclcpp::Parameter("diag_source_action_aware", std::vector<bool>{true}),
    rclcpp::Parameter("diag_source_summary_child", std::vector<std::string>{""}),
  };
}

}  // namespace

// ===========================================================================
// (a) boots with no publishers -> UNKNOWN everywhere. Positive control: feed
// every always item + the Sub-B source (+ state/vehicle) -> GO.
// ===========================================================================

TEST(DiagnosticsNode, NoPublishersIsUnknownEverywhere)
{
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams());

  ASSERT_TRUE(harness.waitForSystemHealth(5.0)) << "FAILED TO MEASURE: no system_health tick";
  const DiagnosticStatus health = harness.lastSystemHealth();
  EXPECT_EQ(diagnosticStr(health, "go_no_go"), "UNKNOWN");
  EXPECT_GT(diagnosticNum(health, "unknown_count"), 0.0);
  // P10.9b: unmeasured state/vehicle also means the phase predicate is
  // strict-flight, never a hand-set param anymore.
  EXPECT_EQ(diagnosticStr(health, "gate_mode"), "flight");
  EXPECT_EQ(diagnosticStr(health, "gate_mode_source"), "unmeasured_strict");

  ASSERT_TRUE(harness.waitForAggregated(5.0)) << "FAILED TO MEASURE: no aggregated tick";
  const DiagnosticArray aggregated = harness.lastAggregated();
  const DiagnosticStatus * item = findStatus(aggregated, "always_a");
  ASSERT_NE(item, nullptr);
  // UNKNOWN maps to STALE (the nearest diagnostic_msgs level -- R30: never OK).
  EXPECT_EQ(item->level, DiagnosticStatus::STALE);
  EXPECT_NE(item->level, DiagnosticStatus::OK);
}

TEST(DiagnosticsNode, EveryAlwaysItemAndSubBSourceFreshIsGo)
{
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams());

  Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
  Ticker tick_b(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
  DiagTicker tick_safety(
    harness.probe(), harness.prefix() + "/diagnostics/safety", DiagnosticStatus::OK, 200ms);
  VehicleTicker vehicle(harness.probe(), harness.prefix() + "/state/vehicle", 50ms);

  ASSERT_TRUE(harness.waitForGoNoGo("GO", 5.0))
    << "FAILED TO MEASURE: never reached GO with every always/Sub-B source fresh, last="
    << diagnosticStr(harness.lastSystemHealth(), "go_no_go");
  EXPECT_EQ(diagnosticNum(harness.lastSystemHealth(), "unknown_count"), 0.0);
  EXPECT_EQ(diagnosticNum(harness.lastSystemHealth(), "error_count"), 0.0);
}

// ===========================================================================
// (b) GO -> stop ONE always publisher -> STALE + NO_GO within timeout +
// report_period. Resume -> GO again.
// ===========================================================================

TEST(DiagnosticsNode, StoppingOneAlwaysPublisherFlipsToNoGoThenRecovers)
{
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams());

  Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
  auto tick_b = std::make_unique<Ticker>(
    harness.probe(), harness.prefix() + "/test/always_b", 50ms);
  DiagTicker tick_safety(
    harness.probe(), harness.prefix() + "/diagnostics/safety", DiagnosticStatus::OK, 200ms);
  VehicleTicker vehicle(harness.probe(), harness.prefix() + "/state/vehicle", 50ms);

  ASSERT_TRUE(harness.waitForGoNoGo("GO", 5.0)) << "FAILED TO MEASURE: never reached GO";

  tick_b->stop();
  tick_b.reset();

  ASSERT_TRUE(harness.waitForGoNoGo("NO_GO", 3.0))
    << "FAILED TO MEASURE: never flipped to NO_GO after always_b went silent, last="
    << diagnosticStr(harness.lastSystemHealth(), "go_no_go");
  // waitForAggregated() alone is NOT enough here -- it only proves "some
  // aggregated sample was EVER received" (the flag never resets), which
  // could be the one from BEFORE always_b went silent: aggregated republish
  // cadence (publish_period_sec_) is decoupled from system_health's
  // on-change publish, so poll until a FRESH one actually shows the change.
  ASSERT_TRUE(
    waitFor(
      [&]() {
        const DiagnosticStatus * item = findStatus(harness.lastAggregated(), "always_b");
        return item != nullptr && item->level == DiagnosticStatus::ERROR;
      }, 3.0))
    << "FAILED TO MEASURE: aggregated never caught up to show always_b as ERROR";

  Ticker tick_b_again(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
  ASSERT_TRUE(harness.waitForGoNoGo("GO", 3.0))
    << "FAILED TO MEASURE: never recovered to GO after always_b resumed";
}

// ===========================================================================
// (c) preflight: flight_only silence does NOT block GO. flight: the SAME
// silence pulls the system out of GO -- both directions of the tense
// predicate (P10.0 finding, R35). P10.9b: driven via gate_mode_override
// (auto|flight) instead of the old hand-set gate_mode, WITH a VehicleTicker
// kept armed=false throughout -- "auto" resolves to the raw measured phase,
// which by then has long since dwelled down to preflight.
// ===========================================================================

TEST(DiagnosticsNode, FlightOnlySilenceIsIgnoredPreflightAndCountsInFlight)
{
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams());

  Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
  Ticker tick_b(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
  DiagTicker tick_safety(
    harness.probe(), harness.prefix() + "/diagnostics/safety", DiagnosticStatus::OK, 200ms);
  VehicleTicker vehicle(harness.probe(), harness.prefix() + "/state/vehicle", 50ms);
  // flight_a is DELIBERATELY never published -- this is the "parked" case.

  ASSERT_TRUE(harness.waitForGoNoGo("GO", 5.0))
    << "FAILED TO MEASURE: preflight should be GO despite flight_a being silent";

  harness.setGateModeOverride("flight");
  ASSERT_TRUE(waitFor([&]() {return diagnosticStr(harness.lastSystemHealth(), "gate_mode") ==
    "flight";}, 3.0)) << "FAILED TO MEASURE: gate_mode never switched to flight";
  ASSERT_TRUE(
    waitFor(
      [&]() {
        const std::string state = diagnosticStr(harness.lastSystemHealth(), "go_no_go");
        return state == "NO_GO" || state == "UNKNOWN";
      }, 3.0))
    << "FAILED TO MEASURE: flight_a silence never counted once gate_mode=flight, last="
    << diagnosticStr(harness.lastSystemHealth(), "go_no_go");

  // "auto" (not "preflight" -- that override value is a documented no-op,
  // it can never LOOSEN a phase back toward preflight) -- armed_ has been
  // false via `vehicle` this whole time, so the raw measured phase has long
  // since dwelled down to preflight underneath the override.
  harness.setGateModeOverride("auto");
  ASSERT_TRUE(harness.waitForGoNoGo("GO", 3.0))
    << "FAILED TO MEASURE: never returned to GO after override back to auto";
}

// ===========================================================================
// (d) Sub-B: DiagnosticArray level ERROR on the source -> NO_GO. Silence >
// sub_b_stale_after_sec -> UNKNOWN contribution (R32), not a held-over ERROR.
// ===========================================================================

TEST(DiagnosticsNode, SubBErrorForcesNoGoThenSelfStalenessFoldsIntoUnknown)
{
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams());

  Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
  Ticker tick_b(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
  VehicleTicker vehicle(harness.probe(), harness.prefix() + "/state/vehicle", 50ms);

  auto tick_safety = std::make_unique<DiagTicker>(
    harness.probe(), harness.prefix() + "/diagnostics/safety", DiagnosticStatus::OK, 200ms);
  ASSERT_TRUE(harness.waitForGoNoGo("GO", 5.0)) << "FAILED TO MEASURE: never reached GO";

  tick_safety->stop();
  tick_safety.reset();
  auto tick_safety_error = std::make_unique<DiagTicker>(
    harness.probe(), harness.prefix() + "/diagnostics/safety", DiagnosticStatus::ERROR, 200ms);

  ASSERT_TRUE(harness.waitForGoNoGo("NO_GO", 3.0))
    << "FAILED TO MEASURE: Sub-B ERROR content never forced NO_GO, last="
    << diagnosticStr(harness.lastSystemHealth(), "go_no_go");

  tick_safety_error->stop();
  tick_safety_error.reset();

  // sub_b_stale_after_sec_ = 2.0s (baseParams) -- self-stale after that.
  ASSERT_TRUE(harness.waitForGoNoGo("UNKNOWN", 5.0))
    << "FAILED TO MEASURE: Sub-B self-staleness never folded into UNKNOWN (R32), last="
    << diagnosticStr(harness.lastSystemHealth(), "go_no_go");
}

// ===========================================================================
// C2: an item/source owned by an OPTIONAL subsystem must not permanently
// starve go/no-go into UNKNOWN when that subsystem is off -- but a REAL
// fault elsewhere must still be caught (not a global "always GO" bypass).
// ===========================================================================

TEST(DiagnosticsNode, PerceptionOwnedItemWithNoPublisherDoesNotBlockGoWhenPerceptionDisabled)
{
  const std::string uav_id = nextUavId();
  auto params = baseParams();
  overrideParam(
    params, rclcpp::Parameter("flight_only_owner", std::vector<std::string>{"perception"}));
  overrideParam(params, rclcpp::Parameter("perception_enabled", false));
  Harness harness(uav_id, params);

  Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
  Ticker tick_b(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
  DiagTicker tick_safety(
    harness.probe(), harness.prefix() + "/diagnostics/safety", DiagnosticStatus::OK, 200ms);
  VehicleTicker vehicle(harness.probe(), harness.prefix() + "/state/vehicle", 50ms);
  harness.setGateModeOverride("flight");
  // flight_a (now perception-owned) is DELIBERATELY never published.

  ASSERT_TRUE(harness.waitForGoNoGo("GO", 5.0))
    << "FAILED TO MEASURE: a perception-owned item with no publisher must not block GO when "
       "perception_enabled=false, last="
    << diagnosticStr(harness.lastSystemHealth(), "go_no_go");
  EXPECT_EQ(diagnosticNum(harness.lastSystemHealth(), "unknown_count"), 0.0);

  // Prove go/no-go is NOT just globally stuck GO -- a genuine fault
  // elsewhere must still flip it, exactly as before this fix.
  tick_b.stop();
  ASSERT_TRUE(harness.waitForGoNoGo("NO_GO", 3.0))
    << "FAILED TO MEASURE: a genuine fault must still flip go/no-go with perception disabled, "
       "last=" << diagnosticStr(harness.lastSystemHealth(), "go_no_go");
}

TEST(DiagnosticsNode, BlackboxOwnedDiagSourceNeverGatesGoNoGoEvenWhenReportingError)
{
  const std::string uav_id = nextUavId();
  auto params = baseParams();
  overrideParam(
    params, rclcpp::Parameter("diag_source_owner", std::vector<std::string>{"blackbox"}));
  Harness harness(uav_id, params);

  Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
  Ticker tick_b(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
  VehicleTicker vehicle(harness.probe(), harness.prefix() + "/state/vehicle", 50ms);
  // diagnostics/safety (now blackbox-owned) is DELIBERATELY never published
  // -- would normally self-stale into UNKNOWN and block GO forever (O1's
  // exact complaint: the evidence layer's own health must never gate a
  // flight).

  ASSERT_TRUE(harness.waitForGoNoGo("GO", 5.0))
    << "FAILED TO MEASURE: a blackbox-owned Sub-B source with no publisher must never block GO "
       "(O1), last=" << diagnosticStr(harness.lastSystemHealth(), "go_no_go");

  DiagTicker tick_safety_error(
    harness.probe(), harness.prefix() + "/diagnostics/safety", DiagnosticStatus::ERROR, 200ms);
  std::this_thread::sleep_for(500ms);   // give it time to actually be received
  EXPECT_EQ(diagnosticStr(harness.lastSystemHealth(), "go_no_go"), "GO")
    << "a blackbox-owned source reporting ERROR content must still never gate go/no-go (O1)";
}

// ===========================================================================
// C2 (VÀNG, contract S:2.20's own rule "GO iff every counted item is OK"):
// Sub-B WARN content must force NO_GO, not silently pass through as GO.
// ===========================================================================

TEST(DiagnosticsNode, SubBWarnContentForcesNoGoNotSilentGo)
{
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams());

  Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
  Ticker tick_b(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
  VehicleTicker vehicle(harness.probe(), harness.prefix() + "/state/vehicle", 50ms);
  auto tick_safety = std::make_unique<DiagTicker>(
    harness.probe(), harness.prefix() + "/diagnostics/safety", DiagnosticStatus::OK, 200ms);
  ASSERT_TRUE(harness.waitForGoNoGo("GO", 5.0)) << "FAILED TO MEASURE: never reached GO";

  tick_safety->stop();
  tick_safety.reset();
  auto tick_safety_warn = std::make_unique<DiagTicker>(
    harness.probe(), harness.prefix() + "/diagnostics/safety", DiagnosticStatus::WARN, 200ms);

  ASSERT_TRUE(harness.waitForGoNoGo("NO_GO", 3.0))
    << "FAILED TO MEASURE: Sub-B WARN content must force NO_GO (contract S:2.20: GO iff every "
       "counted item is OK), last=" << diagnosticStr(harness.lastSystemHealth(), "go_no_go");
}

// ===========================================================================
// C6: /state/system_health (a bare DiagnosticStatus, no header) must carry
// its own age -- stamp_sec (node/sim clock) + wall_stamp_sec (real UTC) --
// so a consumer can enforce R32 even if it only ever sees ONE sample
// (TransientLocal) and /clock later freezes.
// ===========================================================================

TEST(DiagnosticsNode, SystemHealthCarriesStampSecAndWallStampSecForConsumerSideStaleness)
{
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams());
  ASSERT_TRUE(harness.waitForSystemHealth(5.0)) << "FAILED TO MEASURE: no system_health tick";

  const DiagnosticStatus health = harness.lastSystemHealth();
  const double stamp_sec = diagnosticNum(health, "stamp_sec");
  const double wall_stamp_sec = diagnosticNum(health, "wall_stamp_sec");
  ASSERT_FALSE(std::isnan(stamp_sec))
    << "stamp_sec missing -- a bare DiagnosticStatus has no header, a consumer cannot enforce "
       "R32 without an in-payload stamp";
  ASSERT_FALSE(std::isnan(wall_stamp_sec));
  EXPECT_GT(stamp_sec, 0.0);
  EXPECT_GT(wall_stamp_sec, 1.7e9) << "wall_stamp_sec does not look like a real UTC epoch reading";
}

// ===========================================================================
// P10.9b Nhóm A -- measured phase (design panel S:4.c).
// ===========================================================================

TEST(DiagnosticsNode, PhaseFollowsArmedNotTheParameter)
{
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams());

  Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
  Ticker tick_b(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
  DiagTicker tick_safety(
    harness.probe(), harness.prefix() + "/diagnostics/safety", DiagnosticStatus::OK, 200ms);
  VehicleTicker vehicle(harness.probe(), harness.prefix() + "/state/vehicle", 50ms);
  vehicle.setArmed(false);

  ASSERT_TRUE(
    waitFor(
      [&]() {return diagnosticStr(harness.lastSystemHealth(), "gate_mode") == "preflight";},
      3.0))
    << "FAILED TO MEASURE: never settled to preflight with armed=false";
  EXPECT_EQ(diagnosticStr(harness.lastSystemHealth(), "gate_mode_source"), "measured");

  vehicle.setArmed(true);
  ASSERT_TRUE(
    waitFor(
      [&]() {return diagnosticStr(harness.lastSystemHealth(), "gate_mode") == "flight";}, 1.0))
    << "FAILED TO MEASURE: armed=true never upgraded phase to flight (should be IMMEDIATE, G4)";
  EXPECT_EQ(diagnosticStr(harness.lastSystemHealth(), "gate_mode_source"), "measured");
}

TEST(DiagnosticsNode, OverrideMayTightenNeverLoosen)
{
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams());

  Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
  Ticker tick_b(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
  DiagTicker tick_safety(
    harness.probe(), harness.prefix() + "/diagnostics/safety", DiagnosticStatus::OK, 200ms);
  VehicleTicker vehicle(harness.probe(), harness.prefix() + "/state/vehicle", 50ms);

  // armed=true + override=preflight -> still flight (override cannot loosen).
  vehicle.setArmed(true);
  ASSERT_TRUE(
    waitFor(
      [&]() {return diagnosticStr(harness.lastSystemHealth(), "gate_mode") == "flight";}, 3.0));
  harness.setGateModeOverride("preflight");
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(diagnosticStr(harness.lastSystemHealth(), "gate_mode"), "flight")
    << "override=preflight must NEVER loosen a measured flight phase";

  // armed=false (dwelled to preflight) + override=flight -> flight.
  harness.setGateModeOverride("auto");
  vehicle.setArmed(false);
  ASSERT_TRUE(
    waitFor(
      [&]() {return diagnosticStr(harness.lastSystemHealth(), "gate_mode") == "preflight";},
      3.0))
    << "FAILED TO MEASURE: never dwelled down to preflight with override=auto";
  harness.setGateModeOverride("flight");
  ASSERT_TRUE(
    waitFor(
      [&]() {return diagnosticStr(harness.lastSystemHealth(), "gate_mode") == "flight";}, 1.0))
    << "FAILED TO MEASURE: override=flight never forced flight despite armed=false";
}

// V3 (review lượt 3): every declared parameter EXCEPT gate_mode_override is
// read exactly once in declareAndValidateParams() and never re-read --
// "ros2 param set" on any of them must not report success while silently
// changing nothing (perception_enabled picked as the example the review
// flagged, but this is true of the whole watch/diag_source/waiver config).
TEST(DiagnosticsNode, SetParameterOnReadOnlyFieldReportsFailure)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto params = baseParams();
  params.push_back(rclcpp::Parameter("uav_id", nextUavId()));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params);
  DiagnosticsNode node(options);

  const auto result = node.set_parameter(rclcpp::Parameter("perception_enabled", false));
  EXPECT_FALSE(result.successful)
    << "V3: perception_enabled is read once at startup and never re-read -- a runtime "
       "set_parameter must be REFUSED, not silently accepted while doing nothing";

  const auto gate_result =
    node.set_parameter(rclcpp::Parameter("gate_mode_override", std::string("flight")));
  EXPECT_TRUE(gate_result.successful)
    << "gate_mode_override is the ONE deliberately runtime-settable param, must still work";
}

TEST(DiagnosticsNode, UnmeasurablePhaseIsStrictAndNeverGo)
{
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams());

  Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
  Ticker tick_b(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
  DiagTicker tick_safety(
    harness.probe(), harness.prefix() + "/diagnostics/safety", DiagnosticStatus::OK, 200ms);
  auto vehicle = std::make_unique<VehicleTicker>(
    harness.probe(), harness.prefix() + "/state/vehicle", 50ms);

  ASSERT_TRUE(harness.waitForGoNoGo("GO", 5.0)) << "FAILED TO MEASURE: never reached GO";

  vehicle->stop();
  vehicle.reset();

  // always_timeout_sec for state/vehicle = 0.3s (baseParams) -- strict past that.
  ASSERT_TRUE(
    waitFor(
      [&]() {
        return diagnosticStr(harness.lastSystemHealth(), "gate_mode_source") ==
        "unmeasured_strict";
      }, 3.0))
    << "FAILED TO MEASURE: never went strict after state/vehicle went silent";
  EXPECT_EQ(diagnosticStr(harness.lastSystemHealth(), "gate_mode"), "flight");
  EXPECT_NE(diagnosticStr(harness.lastSystemHealth(), "go_no_go"), "GO")
    << "an unmeasurable phase must never coincide with GO";
}

TEST(DiagnosticsNode, PhaseDowngradeIsDwelledUpgradeIsImmediate)
{
  // Wider report_period_sec (with matching wider timeouts, V-O3) than the
  // default fixture -- gives a reliable margin to hold armed=false for
  // LESS than one full eval tick without racing the node's own timer.
  auto params = baseParams();
  overrideParam(params, rclcpp::Parameter("report_period_sec", 0.2));
  overrideParam(params, rclcpp::Parameter("publish_period_sec", 0.2));
  overrideParam(
    params, rclcpp::Parameter("always_timeout_sec", std::vector<double>{0.5, 0.5, 0.5}));
  overrideParam(
    params, rclcpp::Parameter("flight_only_timeout_sec", std::vector<double>{0.5}));
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, params);

  Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
  Ticker tick_b(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
  DiagTicker tick_safety(
    harness.probe(), harness.prefix() + "/diagnostics/safety", DiagnosticStatus::OK, 200ms);
  VehicleTicker vehicle(harness.probe(), harness.prefix() + "/state/vehicle", 50ms);

  vehicle.setArmed(true);
  ASSERT_TRUE(
    waitFor(
      [&]() {return diagnosticStr(harness.lastSystemHealth(), "gate_mode") == "flight";}, 3.0));

  // A single glitchy tick of armed=false (kPreflightDwellTicks=7, V4 review
  // lượt 3; report_period=0.2s -> dwell threshold=1.4s; 100ms is well
  // under one full tick).
  vehicle.setArmed(false);
  bool saw_preflight = false;
  const auto glitch_deadline = std::chrono::steady_clock::now() + 100ms;
  while (std::chrono::steady_clock::now() < glitch_deadline) {
    if (diagnosticStr(harness.lastSystemHealth(), "gate_mode") == "preflight") {
      saw_preflight = true;
    }
    std::this_thread::sleep_for(10ms);
  }
  vehicle.setArmed(true);

  EXPECT_FALSE(saw_preflight)
    << "G4: a single glitchy armed=false tick must not re-open preflight waiver eligibility";
  ASSERT_TRUE(
    waitFor(
      [&]() {return diagnosticStr(harness.lastSystemHealth(), "gate_mode") == "flight";}, 1.0))
    << "armed=true again should keep/re-confirm flight immediately";
}

// ===========================================================================
// Review lượt 2 (N1/N2/N3, .claude/plan/P10-gonogo-design-panel.md's own
// "operator-checklist" is what this predicate implements) -- three ways the
// PREFLIGHT/FLIGHT predicate could tụt (drop) mid-flight, closing the go/no-
// go shield exactly when it matters most (R0).
// ===========================================================================

// N1: px4_state_adapter_node.cpp only assigns state.armed INSIDE
// `if (state.connected)` -- losing the PX4 link leaves the adapter
// publishing at full cadence (topic stays "fresh" by liveness alone) with
// armed defaulted back to false. Pre-fix, effectivePhase() never looked at
// `connected` at all, so this read as a genuine disarm and (after dwell)
// tụt to PREFLIGHT -- gate_mode_source stayed "measured", the worst
// possible label for a sample that is actually unusable.
TEST(DiagnosticsNode, LostPx4LinkForcesFlightStrictNeverPreflight)
{
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams());

  Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
  Ticker tick_b(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
  DiagTicker tick_safety(
    harness.probe(), harness.prefix() + "/diagnostics/safety", DiagnosticStatus::OK, 200ms);
  VehicleTicker vehicle(harness.probe(), harness.prefix() + "/state/vehicle", 50ms);
  vehicle.setConnected(true);
  vehicle.setArmed(true);   // mid-flight: armed, link healthy.

  ASSERT_TRUE(
    waitFor(
      [&]() {return diagnosticStr(harness.lastSystemHealth(), "gate_mode") == "flight";}, 3.0))
    << "FAILED TO MEASURE: never reached flight with armed=true, connected=true";
  EXPECT_EQ(diagnosticStr(harness.lastSystemHealth(), "gate_mode_source"), "measured");

  // Simulate losing the PX4 link WITHOUT the topic going silent -- exactly
  // what px4_state_adapter_node.cpp:121 actually publishes (armed reset to
  // its struct default, connected=false), never a plain topic dropout.
  vehicle.setConnected(false);
  vehicle.setArmed(false);

  ASSERT_TRUE(
    waitFor(
      [&]() {return diagnosticStr(harness.lastSystemHealth(), "gate_mode_source") ==
        "unmeasured_strict";}, 2.0))
    << "FAILED TO MEASURE: N1 -- losing connected_ never forced unmeasured_strict, last "
       "gate_mode=" << diagnosticStr(harness.lastSystemHealth(), "gate_mode") << " source="
    << diagnosticStr(harness.lastSystemHealth(), "gate_mode_source");
  EXPECT_EQ(diagnosticStr(harness.lastSystemHealth(), "gate_mode"), "flight")
    << "N1: a lost PX4 link must NEVER be read as a measured downgrade to preflight -- it must "
       "fall into the SAME strict-flight branch as a plain topic dropout";
  // V6 (review lượt 3): phase_source=unmeasured_strict must ITSELF block GO
  // -- this is the exact gap N1 left open. state/vehicle keeps arriving AT
  // CADENCE via the ticker (its OWN Sub-A liveness verdict reads kOk); only
  // `connected` says the link is gone. Every other watched item here
  // (always_a/always_b/diagnostics/safety) is fed and healthy -- pre-fix,
  // NONE of that would have stopped go_no_go from reading GO.
  EXPECT_NE(diagnosticStr(harness.lastSystemHealth(), "go_no_go"), "GO")
    << "V6: an unmeasurable phase must block GO even when state/vehicle's OWN liveness -- and "
       "every other item -- reads OK, last go_no_go=" <<
    diagnosticStr(harness.lastSystemHealth(), "go_no_go");

  // Other side: link comes back healthy+armed -> measured flight again, not
  // stuck strict forever (R30's "cannot measure" must self-heal).
  vehicle.setConnected(true);
  vehicle.setArmed(true);
  ASSERT_TRUE(
    waitFor(
      [&]() {return diagnosticStr(harness.lastSystemHealth(), "gate_mode_source") == "measured";},
      2.0))
    << "FAILED TO MEASURE: never recovered to measured after connected_ came back true";
  EXPECT_EQ(diagnosticStr(harness.lastSystemHealth(), "gate_mode"), "flight");
}

// N2: pre-fix, armed_ was written by a DEDICATED typed subscription while
// its own freshness was read off a SEPARATE GenericSubscription cadence
// entry on the identical topic -- two independent DDS reader objects that
// can see arrivals out of step with each other (worst case: diagnostics_
// node restarts mid-flight while px4_state_adapter_node keeps publishing;
// whichever reader matches that still-running publisher first drives
// "fresh" while the other reader's own state -- armed_ -- can still be
// sitting on its startup default). A literal reproduction of the DDS
// discovery-timing race itself is not something a fast, deterministic unit
// test can pin reliably (would be flaky by construction) -- instead this
// pins the STRUCTURAL precondition the race depends on: TWO independent
// node-side subscriptions existing on the same topic in the first place.
// The fix (single sentinel always-item, fed exclusively by the SAME
// onVehicleState() callback that also sets armed_/connected_) makes that
// precondition impossible, which this test measures directly.
TEST(DiagnosticsNode, VehicleStateTopicHasExactlyOneNodeSideSubscription)
{
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams());
  Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
  Ticker tick_b(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
  DiagTicker tick_safety(
    harness.probe(), harness.prefix() + "/diagnostics/safety", DiagnosticStatus::OK, 200ms);
  VehicleTicker vehicle(harness.probe(), harness.prefix() + "/state/vehicle", 50ms);
  ASSERT_TRUE(harness.waitForSystemHealth(5.0)) << "FAILED TO MEASURE: no system_health tick";

  size_t subscriber_count = 0;
  ASSERT_TRUE(
    waitFor(
      [&]() {
        subscriber_count = harness.probe()->get_subscriptions_info_by_topic(
          harness.prefix() + "/state/vehicle").size();
        return subscriber_count > 0;
      }, 3.0))
    << "FAILED TO MEASURE: no subscriber ever showed up on state/vehicle";
  EXPECT_EQ(subscriber_count, 1u)
    << "N2: state/vehicle must have exactly ONE node-side subscriber -- two independent "
       "readers (a GenericSubscription cadence entry plus a dedicated typed sub) can see "
       "arrivals out of step with each other, the exact structural precondition for the bug "
       "this fix removes";
}

// ===========================================================================
// P10.9b Nhóm B -- waivers, BOTH sides on the SAME child (design panel S:4.c).
// ===========================================================================

TEST(DiagnosticsNode, WaivedChildDoesNotBlockPreflightGo)
{
  auto params = baseParams();
  overrideParam(
    params, rclcpp::Parameter("waiver_source", std::vector<std::string>{"diagnostics/safety"}));
  overrideParam(
    params, rclcpp::Parameter(
      "waiver_child", std::vector<std::string>{"safety: OFFBOARD_UNHEALTHY"}));
  overrideParam(params, rclcpp::Parameter("waiver_when", std::vector<std::string>{"preflight"}));
  overrideParam(
    params, rclcpp::Parameter("waiver_reason", std::vector<std::string>{"parked, expected"}));
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, params);

  Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
  Ticker tick_b(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
  VehicleTicker vehicle(harness.probe(), harness.prefix() + "/state/vehicle", 50ms);
  vehicle.setArmed(false);
  DiagTicker tick_safety(
    harness.probe(), harness.prefix() + "/diagnostics/safety",
    std::vector<FakeChild>{
      {"safety: OFFBOARD_UNHEALTHY", DiagnosticStatus::ERROR, "report"},
      {"safety: LOCALIZATION_INVALID", DiagnosticStatus::OK, "report"},
    }, 200ms);

  ASSERT_TRUE(harness.waitForGoNoGo("GO", 5.0))
    << "FAILED TO MEASURE: waived child should not block preflight GO, last="
    << diagnosticStr(harness.lastSystemHealth(), "go_no_go");
  EXPECT_GE(diagnosticNum(harness.lastSystemHealth(), "waived_count"), 1.0);
}

TEST(DiagnosticsNode, SameWaivedChildBlocksOnceArmed)
{
  auto params = baseParams();
  overrideParam(
    params, rclcpp::Parameter("waiver_source", std::vector<std::string>{"diagnostics/safety"}));
  overrideParam(
    params, rclcpp::Parameter(
      "waiver_child", std::vector<std::string>{"safety: OFFBOARD_UNHEALTHY"}));
  overrideParam(params, rclcpp::Parameter("waiver_when", std::vector<std::string>{"preflight"}));
  overrideParam(
    params, rclcpp::Parameter("waiver_reason", std::vector<std::string>{"parked, expected"}));
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, params);

  Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
  Ticker tick_b(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
  // Now that armed=true makes flight_only items COUNT, flight_a needs its
  // own liveness too -- otherwise it alone would starve go_no_go at
  // UNKNOWN, masking the safety-child transition this test is about.
  Ticker tick_flight_a(harness.probe(), harness.prefix() + "/test/flight_a", 50ms);
  VehicleTicker vehicle(harness.probe(), harness.prefix() + "/state/vehicle", 50ms);
  vehicle.setArmed(true);   // <-- main difference from WaivedChildDoesNotBlockPreflightGo
  DiagTicker tick_safety(
    harness.probe(), harness.prefix() + "/diagnostics/safety",
    std::vector<FakeChild>{
      {"safety: OFFBOARD_UNHEALTHY", DiagnosticStatus::ERROR, "report"},
      {"safety: LOCALIZATION_INVALID", DiagnosticStatus::OK, "report"},
    }, 200ms);

  ASSERT_TRUE(harness.waitForGoNoGo("NO_GO", 3.0))
    << "FAILED TO MEASURE: a preflight-only waiver must stop applying once armed, last="
    << diagnosticStr(harness.lastSystemHealth(), "go_no_go");
  const std::string worst_item = diagnosticStr(harness.lastSystemHealth(), "worst_item");
  EXPECT_EQ(worst_item, "diagnostics/safety:safety: OFFBOARD_UNHEALTHY");
  EXPECT_NE(diagnosticStr(harness.lastSystemHealth(), "blocking").find(worst_item),
    std::string::npos) << "blocking list must contain the same child worst_item names";
}

TEST(DiagnosticsNode, UnlistedChildAlwaysCounts)
{
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams());   // no waiver rows at all

  Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
  Ticker tick_b(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
  VehicleTicker vehicle(harness.probe(), harness.prefix() + "/state/vehicle", 50ms);
  vehicle.setArmed(false);
  DiagTicker tick_safety(
    harness.probe(), harness.prefix() + "/diagnostics/safety",
    std::vector<FakeChild>{
      {"safety: LOCALIZATION_INVALID", DiagnosticStatus::ERROR, "report"},
    }, 200ms);

  ASSERT_TRUE(harness.waitForGoNoGo("NO_GO", 3.0))
    << "FAILED TO MEASURE: an unlisted (never-waived) ERROR child must always count, last="
    << diagnosticStr(harness.lastSystemHealth(), "go_no_go");
}

TEST(DiagnosticsNode, NewUnknownChildCountsByDefault)
{
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams());

  Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
  Ticker tick_b(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
  VehicleTicker vehicle(harness.probe(), harness.prefix() + "/state/vehicle", 50ms);
  vehicle.setArmed(false);
  DiagTicker tick_safety(
    harness.probe(), harness.prefix() + "/diagnostics/safety",
    std::vector<FakeChild>{
      {"safety: TOTALLY_NEW_CODE", DiagnosticStatus::ERROR, "report"},
    }, 200ms);

  ASSERT_TRUE(harness.waitForGoNoGo("NO_GO", 3.0))
    << "FAILED TO MEASURE: a brand new (P8-added) child must default to COUNTED (fail-loud), "
       "last=" << diagnosticStr(harness.lastSystemHealth(), "go_no_go");
}

TEST(DiagnosticsNode, PerceptionOffWaiverIsIndependentOfPhase)
{
  // Real-shaped scenario: CAMERA_STREAM_UNHEALTHY lives on diagnostics/
  // safety (owner="") -- it is the waiver table, not owner-gating, that has
  // to exempt it when perception_enabled_=false. Compared across TWO
  // harnesses (perception_enabled_ is not live-settable, same as the
  // pre-P10.9b node -- this was never a set_parameter-reconfigurable field).
  auto make_params = [](bool perception_enabled) {
      auto params = baseParams();
      overrideParam(params, rclcpp::Parameter("perception_enabled", perception_enabled));
      overrideParam(
        params, rclcpp::Parameter(
          "waiver_source", std::vector<std::string>{"diagnostics/safety"}));
      overrideParam(
        params, rclcpp::Parameter(
          "waiver_child", std::vector<std::string>{"safety: CAMERA_STREAM_UNHEALTHY"}));
      overrideParam(
        params, rclcpp::Parameter("waiver_when", std::vector<std::string>{"perception_off"}));
      overrideParam(
        params, rclcpp::Parameter(
          "waiver_reason", std::vector<std::string>{"no camera in this bringup"}));
      return params;
    };
  const auto safety_children = std::vector<FakeChild>{
    {"safety: CAMERA_STREAM_UNHEALTHY", DiagnosticStatus::WARN, "report"},
  };

  {
    const std::string uav_id = nextUavId();
    Harness harness(uav_id, make_params(false));
    Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
    Ticker tick_b(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
    // armed=true below makes flight_only items COUNT -- flight_a needs its
    // own liveness too, or it alone starves go_no_go at UNKNOWN.
    Ticker tick_flight_a(harness.probe(), harness.prefix() + "/test/flight_a", 50ms);
    VehicleTicker vehicle(harness.probe(), harness.prefix() + "/state/vehicle", 50ms);
    vehicle.setArmed(true);   // phase irrelevant -- perception_off does not check phase
    DiagTicker tick_safety(
      harness.probe(), harness.prefix() + "/diagnostics/safety", safety_children, 200ms);
    ASSERT_TRUE(harness.waitForGoNoGo("GO", 5.0))
      << "FAILED TO MEASURE: perception_off waiver should apply with perception_enabled=false "
         "regardless of phase, last=" << diagnosticStr(harness.lastSystemHealth(), "go_no_go");
  }
  {
    const std::string uav_id = nextUavId();
    Harness harness(uav_id, make_params(true));
    Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
    Ticker tick_b(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
    Ticker tick_flight_a(harness.probe(), harness.prefix() + "/test/flight_a", 50ms);
    VehicleTicker vehicle(harness.probe(), harness.prefix() + "/state/vehicle", 50ms);
    vehicle.setArmed(true);
    DiagTicker tick_safety(
      harness.probe(), harness.prefix() + "/diagnostics/safety", safety_children, 200ms);
    ASSERT_TRUE(harness.waitForGoNoGo("NO_GO", 3.0))
      << "FAILED TO MEASURE: perception_off waiver must stop applying once perception_enabled_"
         "=true, last=" << diagnosticStr(harness.lastSystemHealth(), "go_no_go");
  }
}

// ===========================================================================
// P10.9b Nhóm C -- NEVER-WAIVE (G1/G2), red-before-green evidence in the
// task report (this file did not exist on the pre-P10.9b logic, so these
// were run against a temporarily-restored copy of the OLD onEvalTick() --
// see report for the saved log).
// ===========================================================================

TEST(DiagnosticsNode, HoldClassErrorIsNeverWaivable)
{
  auto params = baseParams();
  overrideParam(
    params, rclcpp::Parameter("waiver_source", std::vector<std::string>{"diagnostics/safety"}));
  overrideParam(
    params, rclcpp::Parameter("waiver_child", std::vector<std::string>{"safety: BLIND_COMMAND"}));
  overrideParam(params, rclcpp::Parameter("waiver_when", std::vector<std::string>{"preflight"}));
  overrideParam(
    params, rclcpp::Parameter("waiver_reason", std::vector<std::string>{"latch dính, cố tình"}));
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, params);

  Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
  Ticker tick_b(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
  VehicleTicker vehicle(harness.probe(), harness.prefix() + "/state/vehicle", 50ms);
  vehicle.setArmed(false);   // preflight -- the waiver row's `when` DOES match
  DiagTicker tick_safety(
    harness.probe(), harness.prefix() + "/diagnostics/safety",
    std::vector<FakeChild>{
      {"safety: BLIND_COMMAND", DiagnosticStatus::ERROR, "hold"},
    }, 200ms);

  ASSERT_TRUE(harness.waitForGoNoGo("NO_GO", 3.0))
    << "FAILED TO MEASURE: NW1 must defeat a matching waiver row, last="
    << diagnosticStr(harness.lastSystemHealth(), "go_no_go");
  ASSERT_TRUE(harness.waitForAggregated(1.0));
  const DiagnosticStatus * child =
    findStatus(harness.lastAggregated(), "diagnostics/safety:safety: BLIND_COMMAND");
  ASSERT_NE(child, nullptr);
  EXPECT_EQ(diagnosticStr(*child, "counted"), "true");
  EXPECT_EQ(diagnosticStr(*child, "never_waivable"), "true");
  EXPECT_EQ(diagnosticStr(*child, "waived"), "false");
}

TEST(DiagnosticsNode, DryRunActionStringsClassifyAsHoldOrInhibit)
{
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams());

  Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
  Ticker tick_b(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
  VehicleTicker vehicle(harness.probe(), harness.prefix() + "/state/vehicle", 50ms);
  DiagTicker tick_safety(
    harness.probe(), harness.prefix() + "/diagnostics/safety",
    std::vector<FakeChild>{
      {"safety: OBSTACLE_TOO_CLOSE", DiagnosticStatus::ERROR,
        "would_hold (dry-run, enforcement_enabled=false)"},
      {"safety: BLIND_COMMAND", DiagnosticStatus::ERROR,
        "would_inhibit (dry-run, enforcement_enabled=false)"},
    }, 200ms);

  ASSERT_TRUE(harness.waitForAggregated(3.0)) << "FAILED TO MEASURE: no aggregated tick";
  const DiagnosticStatus * hold_child =
    findStatus(harness.lastAggregated(), "diagnostics/safety:safety: OBSTACLE_TOO_CLOSE");
  const DiagnosticStatus * inhibit_child =
    findStatus(harness.lastAggregated(), "diagnostics/safety:safety: BLIND_COMMAND");
  ASSERT_NE(hold_child, nullptr);
  ASSERT_NE(inhibit_child, nullptr);
  EXPECT_EQ(diagnosticStr(*hold_child, "never_waivable"), "true")
    << "dry-run \"would_hold (...)\" must classify as hold (PREFIX match, not full string)";
  EXPECT_EQ(diagnosticStr(*inhibit_child, "never_waivable"), "true")
    << "dry-run \"would_inhibit (...)\" must classify as inhibit (PREFIX match)";
}

TEST(DiagnosticsNode, ReportClassErrorRemainsWaivable)
{
  auto params = baseParams();
  overrideParam(
    params, rclcpp::Parameter("waiver_source", std::vector<std::string>{"diagnostics/safety"}));
  overrideParam(
    params, rclcpp::Parameter(
      "waiver_child", std::vector<std::string>{"safety: ESTIMATOR_INPUT_INVALID"}));
  overrideParam(params, rclcpp::Parameter("waiver_when", std::vector<std::string>{"preflight"}));
  overrideParam(
    params, rclcpp::Parameter("waiver_reason", std::vector<std::string>{"report-only, waivable"}));
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, params);

  Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
  Ticker tick_b(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
  VehicleTicker vehicle(harness.probe(), harness.prefix() + "/state/vehicle", 50ms);
  vehicle.setArmed(false);
  DiagTicker tick_safety(
    harness.probe(), harness.prefix() + "/diagnostics/safety",
    std::vector<FakeChild>{
      {"safety: ESTIMATOR_INPUT_INVALID", DiagnosticStatus::ERROR, "report"},
    }, 200ms);

  ASSERT_TRUE(harness.waitForGoNoGo("GO", 5.0))
    << "FAILED TO MEASURE: NW1 must be ONE-WAY -- a REPORT-class ERROR must still be waivable, "
       "last=" << diagnosticStr(harness.lastSystemHealth(), "go_no_go");
}

TEST(DiagnosticsNode, MissingActionKeyOnErrorChildIsNeverWaivable)
{
  auto params = baseParams();
  overrideParam(
    params, rclcpp::Parameter("waiver_source", std::vector<std::string>{"diagnostics/safety"}));
  overrideParam(
    params, rclcpp::Parameter(
      "waiver_child", std::vector<std::string>{"safety: LOCALIZATION_JUMP"}));
  overrideParam(params, rclcpp::Parameter("waiver_when", std::vector<std::string>{"preflight"}));
  overrideParam(
    params, rclcpp::Parameter("waiver_reason", std::vector<std::string>{"test NW2"}));
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, params);

  Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
  Ticker tick_b(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
  VehicleTicker vehicle(harness.probe(), harness.prefix() + "/state/vehicle", 50ms);
  vehicle.setArmed(false);
  DiagTicker tick_safety(
    harness.probe(), harness.prefix() + "/diagnostics/safety",
    std::vector<FakeChild>{
      {"safety: LOCALIZATION_JUMP", DiagnosticStatus::ERROR, ""},   // no "action" key at all
    }, 200ms);

  ASSERT_TRUE(harness.waitForGoNoGo("NO_GO", 3.0))
    << "FAILED TO MEASURE: NW2 fail-closed -- a missing action key on an action_aware source's "
       "ERROR child must always count, last=" << diagnosticStr(harness.lastSystemHealth(), "go_no_go");
}

TEST(DiagnosticsNode, UnparseableActionOnErrorChildIsNeverWaivable)
{
  auto params = baseParams();
  overrideParam(
    params, rclcpp::Parameter("waiver_source", std::vector<std::string>{"diagnostics/safety"}));
  overrideParam(
    params, rclcpp::Parameter(
      "waiver_child", std::vector<std::string>{"safety: LOCALIZATION_JUMP"}));
  overrideParam(params, rclcpp::Parameter("waiver_when", std::vector<std::string>{"preflight"}));
  overrideParam(
    params, rclcpp::Parameter("waiver_reason", std::vector<std::string>{"test NW2"}));
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, params);

  Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
  Ticker tick_b(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
  VehicleTicker vehicle(harness.probe(), harness.prefix() + "/state/vehicle", 50ms);
  vehicle.setArmed(false);
  DiagTicker tick_safety(
    harness.probe(), harness.prefix() + "/diagnostics/safety",
    std::vector<FakeChild>{
      {"safety: LOCALIZATION_JUMP", DiagnosticStatus::ERROR, "wibble"},
    }, 200ms);

  ASSERT_TRUE(harness.waitForGoNoGo("NO_GO", 3.0))
    << "FAILED TO MEASURE: NW2 fail-closed -- an unresolved action value must always count, "
       "last=" << diagnosticStr(harness.lastSystemHealth(), "go_no_go");
}

TEST(DiagnosticsNode, CannotMeasureWarnUnderPerceptionOffStaysWaivable)
{
  auto params = baseParams();
  overrideParam(
    params, rclcpp::Parameter("waiver_source", std::vector<std::string>{"diagnostics/safety"}));
  overrideParam(
    params, rclcpp::Parameter(
      "waiver_child", std::vector<std::string>{"safety: OBSTACLE_TOO_CLOSE"}));
  overrideParam(params, rclcpp::Parameter("waiver_when", std::vector<std::string>{"preflight"}));
  overrideParam(
    params, rclcpp::Parameter("waiver_reason", std::vector<std::string>{"cannot-measure WARN"}));
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, params);

  Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
  Ticker tick_b(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
  VehicleTicker vehicle(harness.probe(), harness.prefix() + "/state/vehicle", 50ms);
  vehicle.setArmed(false);
  DiagTicker tick_safety(
    harness.probe(), harness.prefix() + "/diagnostics/safety",
    std::vector<FakeChild>{
      // action="hold" (this code's REAL action class) but level=WARN
      // (kCannotMeasure) -- NW1 only fires at level==ERROR.
      {"safety: OBSTACLE_TOO_CLOSE", DiagnosticStatus::WARN, "hold"},
    }, 200ms);

  ASSERT_TRUE(harness.waitForGoNoGo("GO", 5.0))
    << "FAILED TO MEASURE: NW1 must not kick in below ERROR -- a WARN cannot-measure hold-class "
       "code must remain waivable, last=" << diagnosticStr(harness.lastSystemHealth(), "go_no_go");
}

// ===========================================================================
// (e) validate(): one test per rule (R33). Positive control proves the base
// config itself is not the thing throwing.
// ===========================================================================

class DiagnosticsNodeValidateTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    params_ = baseParams();
    params_.push_back(rclcpp::Parameter("uav_id", nextUavId()));
  }

  std::vector<rclcpp::Parameter> params_;
};

TEST_F(DiagnosticsNodeValidateTest, KnownGoodConfigDoesNotThrow)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_NO_THROW({DiagnosticsNode node(options);});
}

TEST_F(DiagnosticsNodeValidateTest, VO1TimeoutBelowTwoOverHzThrows)
{
  overrideParam(
    params_, rclcpp::Parameter("always_timeout_sec", std::vector<double>{0.01, 0.3, 0.3}));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({DiagnosticsNode node(options);}, std::invalid_argument);
}

TEST_F(DiagnosticsNodeValidateTest, VO2MismatchedArrayLengthThrows)
{
  overrideParam(params_, rclcpp::Parameter("always_topics", std::vector<std::string>{"test/a"}));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({DiagnosticsNode node(options);}, std::invalid_argument);
}

TEST_F(DiagnosticsNodeValidateTest, VO3ReportPeriodAboveHalfMinTimeoutThrows)
{
  // publish_period_sec raised TOO (VÀNG#9's separate publish>=report check
  // would otherwise fire first here and pre-empt V-O3) -- this test must
  // isolate V-O3 specifically, not just "some invalid_argument, any reason".
  overrideParam(params_, rclcpp::Parameter("report_period_sec", 1.0));
  overrideParam(params_, rclcpp::Parameter("publish_period_sec", 1.0));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({DiagnosticsNode node(options);}, std::invalid_argument);
}

TEST_F(DiagnosticsNodeValidateTest, VO4NonFiniteHzThrows)
{
  overrideParam(
    params_, rclcpp::Parameter(
      "always_expected_hz",
      std::vector<double>{std::numeric_limits<double>::quiet_NaN(), 10.0, 10.0}));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({DiagnosticsNode node(options);}, std::invalid_argument);
}

TEST_F(DiagnosticsNodeValidateTest, InvalidGateModeOverrideThrows)
{
  overrideParam(params_, rclcpp::Parameter("gate_mode_override", std::string("sometimes")));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({DiagnosticsNode node(options);}, std::invalid_argument);
}

TEST_F(DiagnosticsNodeValidateTest, MissingVehicleStateAlwaysItemThrows)
{
  // P10.9b: the phase predicate's own required "state/vehicle" always-item
  // -- drop it entirely (all 3 -> 2 items) and the node must refuse to
  // start, not silently run with no phase predicate.
  overrideParam(
    params_, rclcpp::Parameter("always_names", std::vector<std::string>{"always_a", "always_b"}));
  overrideParam(
    params_,
    rclcpp::Parameter("always_topics", std::vector<std::string>{"test/always_a", "test/always_b"}));
  overrideParam(
    params_,
    rclcpp::Parameter(
      "always_types", std::vector<std::string>{"std_msgs/msg/String", "std_msgs/msg/String"}));
  overrideParam(params_, rclcpp::Parameter("always_qos", std::vector<std::string>{"be", "be"}));
  overrideParam(params_, rclcpp::Parameter("always_depths", std::vector<int64_t>{10, 10}));
  overrideParam(
    params_, rclcpp::Parameter("always_expected_hz", std::vector<double>{10.0, 10.0}));
  overrideParam(
    params_, rclcpp::Parameter("always_timeout_sec", std::vector<double>{0.3, 0.3}));
  overrideParam(
    params_,
    rclcpp::Parameter("always_level_when_missing", std::vector<std::string>{"error", "error"}));
  overrideParam(params_, rclcpp::Parameter("always_owner", std::vector<std::string>{"", ""}));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({DiagnosticsNode node(options);}, std::invalid_argument);
}

TEST_F(DiagnosticsNodeValidateTest, SentinelTopicTypeMismatchThrows)
{
  // topic given but type left empty -- the both-empty-or-both-set pairing.
  overrideParam(
    params_, rclcpp::Parameter(
      "always_topics", std::vector<std::string>{"test/always_a", "", "state/vehicle"}));
  overrideParam(
    params_,
    rclcpp::Parameter("always_types", std::vector<std::string>{
      "std_msgs/msg/String", "std_msgs/msg/String", "uav_interfaces/msg/VehicleState"}));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({DiagnosticsNode node(options);}, std::invalid_argument);
}

// V5 (review lượt 3): guards the SENTINEL invariant itself (N2 -- state/
// vehicle fed EXCLUSIVELY by the dedicated typed sub, never a second
// GenericSubscription on the same topic). Give it a real topic+type pair
// (BOTH non-empty, so the earlier "topic/type must be both-empty or
// both-set" pairing rule above does NOT throw first) so the LATER, more
// specific `it->topic.empty()` check is what actually gets exercised.
TEST_F(DiagnosticsNodeValidateTest, VehicleStateEntryMustStaySentinelThrows)
{
  overrideParam(
    params_, rclcpp::Parameter(
      "always_topics",
      std::vector<std::string>{"test/always_a", "test/always_b", "state/vehicle"}));
  overrideParam(
    params_,
    rclcpp::Parameter(
      "always_types", std::vector<std::string>{
        "std_msgs/msg/String", "std_msgs/msg/String", "uav_interfaces/msg/VehicleState"}));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({DiagnosticsNode node(options);}, std::invalid_argument);
}

TEST_F(DiagnosticsNodeValidateTest, DiagSourceEmptyTopicThrows)
{
  overrideParam(params_, rclcpp::Parameter("diag_source_topics", std::vector<std::string>{""}));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({DiagnosticsNode node(options);}, std::invalid_argument);
}

// C2: owner must be one of ""|perception|blackbox -- a typo here would
// silently create a 4th, unrecognised bucket that ownerIsCounted() folds
// into "unowned" (always counted), the WRONG fail-direction for a subsystem
// meant to be gateable.
TEST_F(DiagnosticsNodeValidateTest, InvalidOwnerValueThrows)
{
  overrideParam(
    params_, rclcpp::Parameter("always_owner", std::vector<std::string>{"typo", "", ""}));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({DiagnosticsNode node(options);}, std::invalid_argument);
}

TEST_F(DiagnosticsNodeValidateTest, OwnerArrayLengthMismatchThrows)
{
  overrideParam(params_, rclcpp::Parameter("diag_source_owner", std::vector<std::string>{}));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({DiagnosticsNode node(options);}, std::invalid_argument);
}

// VÀNG#9: publish_period_sec must never be tighter than report_period_sec --
// a publish tick racing ahead of the first evaluation could publish a
// still-initial cached value.
TEST_F(DiagnosticsNodeValidateTest, PublishPeriodBelowReportPeriodThrows)
{
  // Must still satisfy V-O3 (report_period_sec <= min(timeout_sec)/2 = 0.15
  // given this fixture's 0.3s timeouts) -- 0.1 does; only the NEW check
  // (publish >= report) is what this test targets.
  overrideParam(params_, rclcpp::Parameter("report_period_sec", 0.1));
  overrideParam(params_, rclcpp::Parameter("publish_period_sec", 0.05));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({DiagnosticsNode node(options);}, std::invalid_argument);
}

// V4 (review lượt 3, replaces the old N3-only check): kPreflightDwellTicks *
// report_period_sec_ must outlast state/vehicle's OWN GRACE period
// (always_timeout_sec) -- NOT merely clear 2 of its sampling periods. The
// OLD invariant (2/expected_hz) is a formal SAMPLING bound, not the REAL
// timeout StalenessBoard actually applies (age<=timeout_sec,
// staleness_board.cpp): a dwell window that clears 2/expected_hz while
// still sitting BELOW always_timeout_sec used to pass validation silently
// -- exactly the shipped defaults before this fix (kPreflightDwellTicks=5 *
// 0.05s = 0,25s < always_timeout_sec=0,3s).
TEST_F(DiagnosticsNodeValidateTest, DwellNotExceedingVehicleStateGraceThrows)
{
  // dwell window = kPreflightDwellTicks(7) * report_period_sec_(0.05,
  // baseParams default) = 0.35s. 2/expected_hz(10Hz) = 0.2s -- the OLD
  // bound, cleared -- but always_timeout_sec raised to 0.4s here -- the NEW
  // bound, NOT cleared. A config the OLD check would have silently accepted
  // must throw under the fixed one.
  overrideParam(
    params_, rclcpp::Parameter("always_timeout_sec", std::vector<double>{0.3, 0.3, 0.4}));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({DiagnosticsNode node(options);}, std::invalid_argument);
}

// ===========================================================================
// P10.9b Nhóm D -- trung thực & cấu hình (R33), waiver table.
// ===========================================================================

TEST_F(DiagnosticsNodeValidateTest, WaiverReasonMustNotBeEmptyThrows)
{
  overrideParam(
    params_, rclcpp::Parameter("waiver_source", std::vector<std::string>{"diagnostics/safety"}));
  overrideParam(
    params_, rclcpp::Parameter("waiver_child", std::vector<std::string>{"safety: BATTERY_WARN"}));
  overrideParam(params_, rclcpp::Parameter("waiver_when", std::vector<std::string>{"preflight"}));
  overrideParam(params_, rclcpp::Parameter("waiver_reason", std::vector<std::string>{""}));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({DiagnosticsNode node(options);}, std::invalid_argument);
}

TEST_F(DiagnosticsNodeValidateTest, WaiverSourceMustExistThrows)
{
  overrideParam(
    params_, rclcpp::Parameter("waiver_source", std::vector<std::string>{"diagnostics/nope"}));
  overrideParam(
    params_, rclcpp::Parameter("waiver_child", std::vector<std::string>{"safety: BATTERY_WARN"}));
  overrideParam(params_, rclcpp::Parameter("waiver_when", std::vector<std::string>{"preflight"}));
  overrideParam(params_, rclcpp::Parameter("waiver_reason", std::vector<std::string>{"x"}));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({DiagnosticsNode node(options);}, std::invalid_argument);
}

TEST_F(DiagnosticsNodeValidateTest, WaiverArraysDifferentLengthThrows)
{
  overrideParam(
    params_, rclcpp::Parameter("waiver_source", std::vector<std::string>{"diagnostics/safety"}));
  overrideParam(params_, rclcpp::Parameter("waiver_child", std::vector<std::string>{}));
  overrideParam(params_, rclcpp::Parameter("waiver_when", std::vector<std::string>{"preflight"}));
  overrideParam(params_, rclcpp::Parameter("waiver_reason", std::vector<std::string>{"x"}));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({DiagnosticsNode node(options);}, std::invalid_argument);
}

TEST_F(DiagnosticsNodeValidateTest, WaiverWhenMustBePreflightOrPerceptionOffThrows)
{
  overrideParam(
    params_, rclcpp::Parameter("waiver_source", std::vector<std::string>{"diagnostics/safety"}));
  overrideParam(
    params_, rclcpp::Parameter("waiver_child", std::vector<std::string>{"safety: BATTERY_WARN"}));
  overrideParam(params_, rclcpp::Parameter("waiver_when", std::vector<std::string>{"always"}));
  overrideParam(params_, rclcpp::Parameter("waiver_reason", std::vector<std::string>{"x"}));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({DiagnosticsNode node(options);}, std::invalid_argument);
}

TEST_F(DiagnosticsNodeValidateTest, WaiverOnSummaryChildRefusesToStart)
{
  overrideParam(
    params_, rclcpp::Parameter(
      "diag_source_summary_child", std::vector<std::string>{"summary_child_x"}));
  overrideParam(
    params_, rclcpp::Parameter("waiver_source", std::vector<std::string>{"diagnostics/safety"}));
  overrideParam(
    params_, rclcpp::Parameter("waiver_child", std::vector<std::string>{"summary_child_x"}));
  overrideParam(params_, rclcpp::Parameter("waiver_when", std::vector<std::string>{"preflight"}));
  overrideParam(params_, rclcpp::Parameter("waiver_reason", std::vector<std::string>{"x"}));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({DiagnosticsNode node(options);}, std::invalid_argument);
}

// G8: summary=ERROR + every OTHER child OK -> GO (rollup is a SUBSET of
// children already counted individually -- proven for uav_safety's
// overall_level, failsafe_policy.cpp's std::max() over VIOLATING codes
// only). Replaces C's 2 dead summary-comparison tests (design panel S:3 G8).
TEST(DiagnosticsNode, SummaryChildIsNeverCountedByLevel)
{
  auto params = baseParams();
  overrideParam(
    params, rclcpp::Parameter(
      "diag_source_summary_child", std::vector<std::string>{"summary_child_x"}));
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, params);

  Ticker tick_a(harness.probe(), harness.prefix() + "/test/always_a", 50ms);
  Ticker tick_b(harness.probe(), harness.prefix() + "/test/always_b", 50ms);
  VehicleTicker vehicle(harness.probe(), harness.prefix() + "/state/vehicle", 50ms);
  vehicle.setArmed(false);
  DiagTicker tick_safety(
    harness.probe(), harness.prefix() + "/diagnostics/safety",
    std::vector<FakeChild>{
      {"summary_child_x", DiagnosticStatus::ERROR, ""},   // the rollup -- must be IGNORED
      {"safety: BATTERY_WARN", DiagnosticStatus::OK, "report"},
    }, 200ms);

  ASSERT_TRUE(harness.waitForGoNoGo("GO", 5.0))
    << "FAILED TO MEASURE: an ERROR-level summary child must NEVER be counted by level, last="
    << diagnosticStr(harness.lastSystemHealth(), "go_no_go");
}

// P8/uav_safety DRIFT detection (F2): every row in the shipped
// config/preflight_waivers.yaml targeting source "diagnostics/safety" must
// name a REAL child actually published by safety_supervisor_node.cpp's
// publishDiagnostics() (NOT violations_publisher_'s /safety/violations, a
// different topic, out of scope for this Sub-B source). Pure text scan of
// the checked-in file -- no node, no --params-file (an EMPTY yaml array,
// which this file legitimately has until D3 loads a real row, makes
// rclcpp's own declare_parameter/parameter_value_from throw "No parameter
// value set": a known type-inference gap on 0-element arrays, unrelated to
// this test's actual purpose).
std::vector<std::string> extractQuotedStringsAfterKey(
  const std::string & text, const std::string & key)
{
  const size_t key_pos = text.find(key + ":");
  if (key_pos == std::string::npos) {
    return {};
  }
  const size_t bracket_open = text.find('[', key_pos);
  const size_t bracket_close = text.find(']', bracket_open);
  if (bracket_open == std::string::npos || bracket_close == std::string::npos) {
    return {};
  }
  const std::string block = text.substr(bracket_open, bracket_close - bracket_open);
  std::vector<std::string> result;
  size_t pos = 0;
  while (true) {
    const size_t start = block.find('"', pos);
    if (start == std::string::npos) {
      break;
    }
    const size_t end = block.find('"', start + 1);
    if (end == std::string::npos) {
      break;
    }
    result.push_back(block.substr(start + 1, end - start - 1));
    pos = end + 1;
  }
  return result;
}

TEST(DiagnosticsNode, ShippedWaiverYamlMatchesRealSafetyChildNames)
{
  static const std::set<std::string> kRealSafetyChildNames = {
    "safety",   // summary (G8's verified rollup child)
    "safety: LOCALIZATION_INVALID", "safety: LOCALIZATION_STALE",
    "safety: LOCALIZATION_JUMP", "safety: BLIND_COMMAND",
    "safety: BATTERY_WARN", "safety: BATTERY_CRITICAL",
    "safety: BATTERY_PX4_NO_ACTION", "safety: OBSTACLE_TOO_CLOSE",
    "safety: OFFBOARD_UNHEALTHY", "safety: ESTIMATOR_INPUT_INVALID",
    "safety: CAMERA_STREAM_UNHEALTHY", "safety: FRAME_MISMATCH",
    "safety: planning/trajectory plan_state",
  };

  const std::string yaml_path =
    std::string(UAV_OBSERVABILITY_PACKAGE_SOURCE_DIR) + "/config/preflight_waivers.yaml";
  std::ifstream file(yaml_path);
  ASSERT_TRUE(file.is_open()) << "FAILED TO MEASURE: cannot open " << yaml_path;
  std::stringstream buffer;
  buffer << file.rdbuf();
  const std::string text = buffer.str();

  const std::vector<std::string> sources = extractQuotedStringsAfterKey(text, "waiver_source");
  const std::vector<std::string> children = extractQuotedStringsAfterKey(text, "waiver_child");
  ASSERT_EQ(sources.size(), children.size())
    << "FAILED TO MEASURE: waiver_source/waiver_child parsed to different counts -- malformed "
       "preflight_waivers.yaml, or this test's own naive text-scan assumption broke";

  for (size_t i = 0; i < sources.size(); ++i) {
    if (sources[i] != "diagnostics/safety") {
      continue;   // this fixture only has ground truth for the safety source
    }
    EXPECT_GT(kRealSafetyChildNames.count(children[i]), 0u)
      << "config/preflight_waivers.yaml row " << i << " targets child \"" << children[i]
      << "\" which is not a real /diagnostics/safety child name -- P8 likely renamed a "
         "violation code, sync this row";
  }
}

// ===========================================================================
// (f) /state/system_health publisher QoS is Reliable/KeepLast(1)/
// TransientLocal -- what actually guarantees a late subscriber sees the
// last value, independent of publish timing races.
// ===========================================================================

TEST(DiagnosticsNode, SystemHealthPublisherIsTransientLocal)
{
  const std::string uav_id = nextUavId();
  Harness harness(uav_id, baseParams());
  ASSERT_TRUE(harness.waitForSystemHealth(5.0)) << "FAILED TO MEASURE: no system_health tick";

  const auto endpoints =
    harness.probe()->get_publishers_info_by_topic(harness.prefix() + "/state/system_health");
  ASSERT_FALSE(endpoints.empty()) << "FAILED TO MEASURE: no publisher found on system_health";
  EXPECT_EQ(
    endpoints.front().qos_profile().durability(), rclcpp::DurabilityPolicy::TransientLocal);
  EXPECT_EQ(
    endpoints.front().qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);

  // Functional corroboration: a subscriber created well after boot still
  // gets a value almost immediately (durable re-delivery, not a timing race
  // against the periodic publish tick).
  DiagnosticStatus late_value;
  bool late_received = false;
  std::mutex late_mutex;
  auto late_subscriber = harness.probe()->create_subscription<DiagnosticStatus>(
    harness.prefix() + "/state/system_health", rclcpp::QoS(1).reliable().transient_local(),
    [&](const DiagnosticStatus::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(late_mutex);
      late_value = *msg;
      late_received = true;
    });
  ASSERT_TRUE(
    waitFor(
      [&]() {std::lock_guard<std::mutex> lock(late_mutex); return late_received;}, 2.0))
    << "FAILED TO MEASURE: late TransientLocal subscriber never received the latched value";
}

}  // namespace uav_observability
