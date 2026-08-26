// G-CA1 (P7-control-authority.md S:6): the real node, driven through the real
// 4 source topics by a probe, with a subscriber on command_selected/authority/
// diagnostics -- same shape as uav_navigation's test_avoidance_chain.cpp. This
// probe fabricates ControlCommand traffic (R20), hence the isolated domain.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <uav_interfaces/msg/control_authority.hpp>
#include <uav_interfaces/msg/control_command.hpp>
#include <uav_interfaces/srv/clear_fault.hpp>
#include <uav_interfaces/srv/set_control_authority.hpp>

#include "uav_control_authority/authority_arbiter.hpp"
#include "uav_control_authority/control_authority_manager_node.hpp"

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using uav_interfaces::msg::ControlAuthority;
using uav_interfaces::msg::ControlCommand;
using uav_interfaces::srv::ClearFault;
using uav_interfaces::srv::SetControlAuthority;
using namespace std::chrono_literals;

namespace uav_control_authority
{
namespace
{

struct CommandSample
{
  double wall_seconds = 0.0;
  /// When the SOURCE emitted it. The arbiter relays the stamp untouched, so this
  /// is the only clock a starved probe cannot stretch (same fix as CutChain Item8).
  double stamp_seconds = 0.0;
  uint8_t source = 0;
  double x = 0.0;
};

/// The witness ticks this often, so it can never resolve a stall finer than one tick.
constexpr double kWitnessTickSec = 0.020;

/// How many lower-priority samples the arbiter had NO right to relay.
///
/// A lower-priority sample on the wire is only a fault if the incumbent was live
/// when it landed: S:3b requires control to be handed down once the incumbent has
/// been silent for release_dwell_sec, so a sample after a longer silence is the
/// arbiter obeying, not breaking, its rule. Measured 2026-08-25 under a 16-way
/// parallel test: the fixture's own publisher thread starved for up to 2.51 s and
/// the arbiter correctly released -- judging that as a fault accused the product
/// of doing exactly what it was told to do. Anchored on stamp (emission), never on
/// probe arrival, so a slow reader cannot manufacture a silence.
std::size_t countPriorityViolations(
  const std::vector<CommandSample> & samples, uint8_t incumbent, uint8_t intruder,
  double release_dwell_sec)
{
  std::size_t violations = 0;
  double last_incumbent = -1.0;
  for (const CommandSample & sample : samples) {
    if (sample.source == incumbent) {
      last_incumbent = sample.stamp_seconds;
      continue;
    }
    if (sample.source != intruder || last_incumbent < 0.0) {
      continue;
    }
    if (sample.stamp_seconds - last_incumbent < release_dwell_sec) {
      ++violations;
    }
  }
  return violations;
}

/// The longest silence the incumbent's own publisher left on the wire. The claim
/// "two sources live at once" is only being tested while this stays under the
/// dwell -- otherwise there was no contest to arbitrate (R27-1).
double worstIncumbentSilenceSec(const std::vector<CommandSample> & samples, uint8_t incumbent)
{
  double worst = 0.0;
  double previous = -1.0;
  for (const CommandSample & sample : samples) {
    if (sample.source != incumbent) {continue;}
    if (previous >= 0.0) {
      worst = std::max(worst, sample.stamp_seconds - previous);
    }
    previous = sample.stamp_seconds;
  }
  return worst;
}

/// Size of the freeze the positive control injects into the probe's callback group.
constexpr double kInjectedStallSec = 0.60;

/// The N-g rule, hoisted out of the test body so it can be checked directly
/// (R27-3). A gap is only attributable to the arbiter when this probe's own
/// executor stayed livelier than the gap it claims to have observed.
///
/// Known blind zone, measured 2026-08-24: on an idle machine the witness floor is
/// one tick (0.0202 s observed), so this rule cannot adjudicate a gap under about
/// 0.04 s -- it would skip forever rather than pass. Unreachable today because the
/// probe streams at 20 Hz, making 0.05 s the smallest honest gap (observed 0.0505 -
/// 0.0509 s). Raise the stream rate and this test goes permanently silent.
bool gapIsMeasurable(double max_gap_sec, double probe_stall_sec)
{
  return probe_stall_sec < max_gap_sec - kWitnessTickSec;
}

struct AuthoritySample
{
  double wall_seconds = 0.0;
  /// When the arbiter emitted it -- the only clock a starved probe cannot stretch.
  double stamp_seconds = 0.0;
  uint8_t active_source = 0;
  uint8_t previous_source = 0;
  std::string reason;
};

/// How many times the arbiter reported NO source while one was in fact still live.
///
/// Reporting NONE is CORRECT once every source has gone quiet for source_timeout_sec
/// -- that is the liveness rule, not a fault. The only fault is reporting NONE while
/// a source was still publishing. Measured 2026-08-25/26: this fixture's own stream
/// threads are what go quiet under load, so judging every NONE accuses the arbiter of
/// obeying its own contract.
std::size_t countUnjustifiedNone(
  const std::vector<CommandSample> & commands, const std::vector<AuthoritySample> & authorities,
  uint8_t incumbent, double source_timeout_sec)
{
  std::size_t unjustified = 0;
  for (const AuthoritySample & sample : authorities) {
    if (sample.active_source != ControlAuthority::SOURCE_NONE) {continue;}
    double newest_incumbent = -1.0;
    for (const CommandSample & command : commands) {
      if (command.source != incumbent) {continue;}
      if (command.stamp_seconds <= sample.stamp_seconds) {
        newest_incumbent = std::max(newest_incumbent, command.stamp_seconds);
      }
    }
    if (newest_incumbent < 0.0) {continue;}      // it had never published at all
    if (sample.stamp_seconds - newest_incumbent < source_timeout_sec) {
      ++unjustified;
    }
  }
  return unjustified;
}

/// RAII: stops and joins a background stream even on an early ASSERT_* return
/// -- a joinable std::thread destroyed without join() calls std::terminate(),
/// which silently takes the whole test BINARY down with it (every other
/// test's result lost, not just this one). Only used where a test needs to
/// assert partway through a still-running stream; tests whose thread already
/// self-completes before any assertion do not need it.
class StreamGuard
{
public:
  StreamGuard(std::thread thread, std::atomic<bool> & stop_flag)
  : thread_(std::move(thread)), stop_flag_(stop_flag) {}
  ~StreamGuard()
  {
    stop_flag_.store(true);
    if (thread_.joinable()) {
      thread_.join();
    }
  }
  StreamGuard(const StreamGuard &) = delete;
  StreamGuard & operator=(const StreamGuard &) = delete;

private:
  std::thread thread_;
  std::atomic<bool> & stop_flag_;
};

class ControlAuthorityFixture : public ::testing::Test
{
protected:
  /// Override in a subclass to widen shipped timing for a test that would
  /// otherwise chase real scheduling jitter instead of the mechanism under
  /// test -- the exact boundary arithmetic is already pinned, jitter-free,
  /// by test_authority_arbiter.cpp.
  virtual std::vector<rclcpp::Parameter> extraParameterOverrides() {return {};}

  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // Own namespace per test instance: a lingering subscription from a
    // previous test must never see this test's fabricated traffic (same
    // discipline as uav_navigation's NavigatorFixture / AvoidanceChainFixture).
    static std::atomic<int> instances{0};
    uav_id_ = "uav" + std::to_string(instances.fetch_add(1));
    prefix_ = "/uav/" + uav_id_;

    std::vector<rclcpp::Parameter> overrides{rclcpp::Parameter("uav_id", uav_id_)};
    for (const rclcpp::Parameter & extra : extraParameterOverrides()) {
      overrides.push_back(extra);
    }
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    node_ = std::make_shared<ControlAuthorityManagerNode>(options);

    probe_ = std::make_shared<rclcpp::Node>("control_authority_probe_" + uav_id_);

    safety_publisher_ =
      probe_->create_publisher<ControlCommand>(prefix_ + "/control/cmd_safety", rclcpp::QoS(10));
    operator_publisher_ = probe_->create_publisher<ControlCommand>(
      prefix_ + "/control/cmd_operator", rclcpp::QoS(10));
    mission_publisher_ = probe_->create_publisher<ControlCommand>(
      prefix_ + "/control/cmd_mission", rclcpp::QoS(10));
    test_publisher_ =
      probe_->create_publisher<ControlCommand>(prefix_ + "/control/cmd_test", rclcpp::QoS(10));

    command_subscription_ = probe_->create_subscription<ControlCommand>(
      prefix_ + "/control/command_selected", rclcpp::QoS(50),
      [this](const ControlCommand::SharedPtr msg) {onCommand(*msg);});
    authority_subscription_ = probe_->create_subscription<ControlAuthority>(
      prefix_ + "/control/authority", rclcpp::QoS(20).reliable().transient_local(),
      [this](const ControlAuthority::SharedPtr msg) {onAuthority(*msg);});
    diagnostics_subscription_ = probe_->create_subscription<DiagnosticArray>(
      prefix_ + "/diagnostics/control_authority", rclcpp::QoS(10),
      [this](const DiagnosticArray::SharedPtr msg) {onDiagnostics(*msg);});

    authority_client_ =
      probe_->create_client<SetControlAuthority>(prefix_ + "/control/set_authority");
    // P8 R2: the only path that releases a SAFETY latch.
    clear_safety_latch_client_ =
      probe_->create_client<ClearFault>(prefix_ + "/control/clear_safety_latch");

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 4);
    executor_->add_node(node_);
    executor_->add_node(probe_);
    spin_thread_ = std::thread([this]() {executor_->spin();});

    // Discovery must complete before a test starts asserting on delivery.
    ASSERT_TRUE(
      waitFor(
        [this]() {
          return probe_->count_subscribers(prefix_ + "/control/cmd_mission") > 0 &&
          command_subscription_->get_publisher_count() > 0;
        }, 10.0))
      << "FAILED TO MEASURE: discovery never completed";
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    node_.reset();
    probe_.reset();
    executor_.reset();
  }

  // ------------------------------------------------------------- callbacks

  void onCommand(const ControlCommand & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    commands_.push_back(
      CommandSample{
        rclcpp::Clock().now().seconds(), rclcpp::Time(msg.header.stamp).seconds(), msg.source,
        msg.position.x});
  }

  void onAuthority(const ControlAuthority & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    authorities_.push_back(
      AuthoritySample{
        rclcpp::Clock().now().seconds(), rclcpp::Time(msg.header.stamp).seconds(),
        msg.active_source, msg.previous_source, msg.reason});
  }

  void onDiagnostics(const DiagnosticArray & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_diagnostics_ = msg;
    has_diagnostics_ = true;
  }

  std::vector<CommandSample> commands()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return commands_;
  }

  /// A stall witness living in the SAME executor as the subscriptions. Gaps here
  /// are measured on ARRIVAL at this probe, so a probe that itself stopped for
  /// 0.8 s is indistinguishable from an arbiter that went quiet for 0.8 s. Under
  /// a 12-package parallel `colcon test` the former happens; without this witness
  /// it reads as a product fault (N-g, 2026-08-24).
  void startStallWitness()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stall_max_sec_ = 0.0;
      last_witness_ = std::chrono::steady_clock::now();
    }
    witness_timer_ = probe_->create_wall_timer(
      20ms, [this]() {
        const auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(mutex_);
        stall_max_sec_ = std::max(
          stall_max_sec_, std::chrono::duration<double>(now - last_witness_).count());
        last_witness_ = now;
      });
  }

  double witnessedStallSec()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return stall_max_sec_;
  }

  std::vector<AuthoritySample> authorities()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return authorities_;
  }

  DiagnosticArray diagnostics()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_diagnostics_;
  }

  // Y9: every access to shared state goes through a locked accessor -- the
  // executor runs 4 threads, so a bare `has_diagnostics_` read/write is a race.
  bool hasDiagnostics()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_diagnostics_;
  }

  /// Resets the recorded logs so a test can establish a steady state (e.g.
  /// "MISSION is already active") before measuring what happens next --
  /// without this, the legitimate startup transition into that steady state
  /// would be counted as part of the thing under test.
  void clearLogs()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    commands_.clear();
    authorities_.clear();
  }

  // --------------------------------------------------------------- helpers

  static ControlCommand makeCommand(uint8_t source, double x, uint8_t mode = ControlCommand::MODE_POSITION)
  {
    ControlCommand msg;
    msg.header.frame_id = "odom";
    msg.control_mode = mode;
    msg.source = source;
    msg.position.x = x;
    msg.yaw = 0.0F;
    return msg;
  }

  void publishOn(
    const rclcpp::Publisher<ControlCommand>::SharedPtr & publisher, ControlCommand msg)
  {
    msg.header.stamp = probe_->now();
    publisher->publish(msg);
  }

  /// Streams source at ~20 Hz for duration_sec on a background thread, so a
  /// test can interleave two live sources without hand-rolled sleeps.
  std::thread streamSource(
    const rclcpp::Publisher<ControlCommand>::SharedPtr & publisher, uint8_t source, double x,
    double duration_sec, std::atomic<bool> * stop_flag)
  {
    return std::thread(
      [this, publisher, source, x, duration_sec, stop_flag]() {
        const auto deadline =
          std::chrono::steady_clock::now() + std::chrono::duration<double>(duration_sec);
        while (std::chrono::steady_clock::now() < deadline &&
          (stop_flag == nullptr || !stop_flag->load()))
        {
          publishOn(publisher, makeCommand(source, x));
          std::this_thread::sleep_for(50ms);
        }
      });
  }

  /// True once a TEST sample has landed AFTER MISSION's last one -- the event
  /// the handover claim is about. "any TEST sample" is already satisfied by the
  /// samples from before MISSION ever started (R21: anchor on the event).
  bool handoverToTestObserved()
  {
    const std::vector<CommandSample> samples = commands();
    std::size_t last_mission = samples.size();
    for (std::size_t i = 0; i < samples.size(); ++i) {
      if (samples[i].source == kSourceMission) {last_mission = i;}
    }
    if (last_mission == samples.size()) {return false;}
    for (std::size_t i = last_mission + 1; i < samples.size(); ++i) {
      if (samples[i].source == kSourceTest) {return true;}
    }
    return false;
  }

  std::size_t countBySource(uint8_t source)
  {
    std::size_t total = 0;
    for (const CommandSample & sample : commands()) {
      if (sample.source == source) {
        ++total;
      }
    }
    return total;
  }

  /// Where an intruder landed relative to the incumbent, so a failure says WHICH
  /// mechanism let it through instead of only that one got through.
  std::string describeIntruders(uint8_t intruder, uint8_t incumbent)
  {
    const std::vector<CommandSample> samples = commands();
    double first_incumbent = -1.0;
    double previous_incumbent = -1.0;
    std::ostringstream out;
    for (std::size_t i = 0; i < samples.size(); ++i) {
      if (samples[i].source == incumbent) {
        if (first_incumbent < 0.0) {first_incumbent = samples[i].stamp_seconds;}
        previous_incumbent = samples[i].stamp_seconds;
        continue;
      }
      if (samples[i].source != intruder) {continue;}
      out << "\n  intruder at index " << i << " of " << samples.size()
          << ", incumbent samples before it: " << (first_incumbent < 0.0 ? "NONE" : "some")
          << ", gap since last incumbent: "
          << (previous_incumbent < 0.0
        ? -1.0
        : samples[i].stamp_seconds - previous_incumbent)
          << " s";
    }
    out << "\n  probe stall witnessed: " << witnessedStallSec() << " s";
    return out.str();
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

  std::string uav_id_;
  std::string prefix_;
  std::shared_ptr<ControlAuthorityManagerNode> node_;
  std::shared_ptr<rclcpp::Node> probe_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;

  rclcpp::Publisher<ControlCommand>::SharedPtr safety_publisher_;
  rclcpp::Publisher<ControlCommand>::SharedPtr operator_publisher_;
  rclcpp::Publisher<ControlCommand>::SharedPtr mission_publisher_;
  rclcpp::Publisher<ControlCommand>::SharedPtr test_publisher_;
  rclcpp::Subscription<ControlCommand>::SharedPtr command_subscription_;
  rclcpp::Subscription<ControlAuthority>::SharedPtr authority_subscription_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr diagnostics_subscription_;
  rclcpp::Client<SetControlAuthority>::SharedPtr authority_client_;
  rclcpp::Client<ClearFault>::SharedPtr clear_safety_latch_client_;

  std::mutex mutex_;
  double stall_max_sec_ = 0.0;
  std::chrono::steady_clock::time_point last_witness_;
  rclcpp::TimerBase::SharedPtr witness_timer_;
  std::vector<CommandSample> commands_;
  std::vector<AuthoritySample> authorities_;
  DiagnosticArray last_diagnostics_;
  bool has_diagnostics_ = false;
};

}  // namespace

// ===========================================================================
// G-CA1 item 1: two sources live at once, only the higher priority reaches
// the wire, WITH a positive counter-check (R21: a negative claim needs proof
// the pipe was not simply broken).
// ===========================================================================
TEST_F(ControlAuthorityFixture, OnlyTheHigherPrioritySourceReachesCommandSelected)
{
  startStallWitness();

  // Build the premise instead of hoping for it. Both streams started together used
  // to let TEST's first message land before MISSION had ever been heard from -- the
  // arbiter then had exactly one live source and relayed it, correctly. Measured
  // 2026-08-25: 5 of 6 failures under load were that race, at sample index 0.
  std::atomic<bool> stop{false};
  std::thread mission_thread = streamSource(mission_publisher_, kSourceMission, 1.0, 4.0, &stop);
  ASSERT_TRUE(
    waitFor(
      [this]() {
        const std::vector<AuthoritySample> log = authorities();
        return !log.empty() && log.back().active_source == kSourceMission;
      }, 5.0))
    << "FAILED TO MEASURE: MISSION never became the active source";
  clearLogs();    // Y9: never touch commands_ directly, it is executor-shared state

  std::thread test_thread = streamSource(test_publisher_, kSourceTest, 2.0, 2.0, &stop);
  test_thread.join();
  stop.store(true);
  mission_thread.join();

  const double dwell = uav_control_authority::ArbiterParams{}.release_dwell_sec;
  ASSERT_GE(countBySource(kSourceMission), 5U) << "FAILED TO MEASURE: too few MISSION samples";
  ASSERT_LT(witnessedStallSec(), dwell / 2.0)
    << "FAILED TO MEASURE: the probe itself stalled " << witnessedStallSec()
    << " s, long enough to fake a silence the arbiter was entitled to act on";
  EXPECT_EQ(countPriorityViolations(commands(), kSourceMission, kSourceTest, dwell), 0U)
    << "TEST reached command_selected while MISSION was live"
    << describeIntruders(kSourceTest, kSourceMission);
  RecordProperty(
    "worst_incumbent_silence_sec",
    std::to_string(worstIncumbentSilenceSec(commands(), kSourceMission)));

  // Positive counter-check: turn MISSION off, TEST must now get through.
  clearLogs();
  std::thread test_only = streamSource(test_publisher_, kSourceTest, 2.0, 1.0, nullptr);
  test_only.join();
  EXPECT_GT(countBySource(kSourceTest), 0U)
    << "FAILED TO MEASURE: TEST never got through even with MISSION silent -- "
    << "the zero count above may have been a broken pipe, not arbitration";
}

// ===========================================================================
// The rule the test above judges by, exercised on its own. Without this the
// silence allowance is only ever seen excusing samples, never convicting one --
// which is how a guard quietly becomes an amnesty (N-g, 2026-08-24).
// ===========================================================================
TEST(PriorityViolationRule, ConvictsAnIntruderThatBeatTheDwell)
{
  const double dwell = uav_control_authority::ArbiterParams{}.release_dwell_sec;
  const std::vector<CommandSample> samples{
    {0.0, 0.00, kSourceMission, 1.0},
    {0.0, 0.05, kSourceMission, 1.0},
    {0.0, 0.10, kSourceTest, 2.0},      // 0.05 s after MISSION: the arbiter had no right
  };
  EXPECT_EQ(countPriorityViolations(samples, kSourceMission, kSourceTest, dwell), 1U);
}

TEST(PriorityViolationRule, AcquitsAnIntruderAfterTheIncumbentWentTrulySilent)
{
  const double dwell = uav_control_authority::ArbiterParams{}.release_dwell_sec;
  const std::vector<CommandSample> samples{
    {0.0, 0.00, kSourceMission, 1.0},
    {0.0, 2.51, kSourceTest, 2.0},      // the 2.51 s starvation measured under load
  };
  EXPECT_EQ(countPriorityViolations(samples, kSourceMission, kSourceTest, dwell), 0U);
}

TEST(PriorityViolationRule, AcquitsAnIntruderThatArrivedBeforeTheIncumbentEverDid)
{
  const double dwell = uav_control_authority::ArbiterParams{}.release_dwell_sec;
  const std::vector<CommandSample> samples{
    {0.0, 0.00, kSourceTest, 2.0},      // index 0: 5 of 6 failures measured 2026-08-25
    {0.0, 0.02, kSourceMission, 1.0},
  };
  EXPECT_EQ(countPriorityViolations(samples, kSourceMission, kSourceTest, dwell), 0U);
}

// The NONE rule, exercised on its own -- same reason as the priority rule above:
// an allowance that is only ever seen excusing samples is an amnesty (N-g).
TEST(JustifiedNoneRule, ConvictsANoneEmittedWhileTheIncumbentWasStillPublishing)
{
  const double timeout = uav_control_authority::ArbiterParams{}.source_timeout_sec;
  const std::vector<CommandSample> commands{
    {0.0, 0.00, kSourceMission, 1.0},
    {0.0, 0.05, kSourceMission, 1.0},
  };
  const std::vector<AuthoritySample> authorities{
    {0.0, 0.10, ControlAuthority::SOURCE_NONE, kSourceMission, "stale?"},
  };
  EXPECT_EQ(countUnjustifiedNone(commands, authorities, kSourceMission, timeout), 1U);
}

TEST(JustifiedNoneRule, AcquitsANoneAfterTheIncumbentTrulyWentQuiet)
{
  const double timeout = uav_control_authority::ArbiterParams{}.source_timeout_sec;
  const std::vector<CommandSample> commands{{0.0, 0.00, kSourceMission, 1.0}};
  const std::vector<AuthoritySample> authorities{
    {0.0, 2.51, ControlAuthority::SOURCE_NONE, kSourceMission, "source timed out"},
  };
  EXPECT_EQ(countUnjustifiedNone(commands, authorities, kSourceMission, timeout), 0U);
}

TEST(JustifiedNoneRule, IgnoresSamplesThatAreNotNone)
{
  const double timeout = uav_control_authority::ArbiterParams{}.source_timeout_sec;
  const std::vector<CommandSample> commands{{0.0, 0.00, kSourceMission, 1.0}};
  const std::vector<AuthoritySample> authorities{
    {0.0, 0.02, kSourceMission, kSourceNone, "mission live"},
    {0.0, 0.04, kSourceOperator, kSourceMission, "operator took over"},
  };
  EXPECT_EQ(countUnjustifiedNone(commands, authorities, kSourceMission, timeout), 0U);
}

TEST(PriorityViolationRule, ConvictsEveryIntruderNotJustTheFirst)
{
  const double dwell = uav_control_authority::ArbiterParams{}.release_dwell_sec;
  const std::vector<CommandSample> samples{
    {0.0, 0.00, kSourceMission, 1.0},
    {0.0, 0.05, kSourceTest, 2.0},
    {0.0, 0.10, kSourceMission, 1.0},
    {0.0, 0.12, kSourceTest, 2.0},
  };
  EXPECT_EQ(countPriorityViolations(samples, kSourceMission, kSourceTest, dwell), 2U);
}

// ===========================================================================
// N-b (Q3's invariant, named): total silence on all 4 source topics must
// never produce a single message on command_selected -- the arbiter is a
// pure relay (S:1), it never synthesizes a setpoint. Positive counter-check
// afterward proves this was silence, not a broken pipe (R21).
// ===========================================================================
TEST_F(ControlAuthorityFixture, AllSourcesSilentProducesZeroMessagesOnCommandSelected)
{
  // Several monitor ticks' worth of real time with nothing published at all.
  std::this_thread::sleep_for(1500ms);
  EXPECT_EQ(commands().size(), 0U)
    << "the arbiter published a setpoint with every source silent -- S:1 violation";

  // Positive counter-check: the pipe itself works once something IS sent.
  publishOn(mission_publisher_, makeCommand(kSourceMission, 1.0));
  ASSERT_TRUE(waitFor([this]() {return !commands().empty();}, 5.0))
    << "FAILED TO MEASURE: command_selected never carried anything, even after a real publish -- "
    << "the zero count above may have been a broken pipe, not the silence invariant";
}

// ===========================================================================
// G-CA1 item 2: SAFETY injected mid-MISSION switches immediately. Measures
// the real hop latency (Q4: measure, then decide -- proposed ceiling 100 ms).
// ===========================================================================
TEST_F(ControlAuthorityFixture, SafetyInjectedMidMissionSwitchesWithinTheProposedCeiling)
{
  std::atomic<bool> stop{false};
  StreamGuard mission_thread(
    streamSource(mission_publisher_, kSourceMission, 0.0, 3.0, &stop), stop);
  ASSERT_TRUE(waitFor([this]() {return countBySource(kSourceMission) >= 5U;}, 5.0))
    << "FAILED TO MEASURE: MISSION never got moving";

  const double injected_at = rclcpp::Clock().now().seconds();
  publishOn(safety_publisher_, makeCommand(kSourceSafety, 9.0));

  ASSERT_TRUE(waitFor([this]() {return countBySource(kSourceSafety) > 0;}, 5.0))
    << "FAILED TO MEASURE: SAFETY never reached command_selected";

  double first_safety_seconds = -1.0;
  for (const CommandSample & sample : commands()) {
    if (sample.source == kSourceSafety) {
      first_safety_seconds = sample.wall_seconds;
      break;
    }
  }
  ASSERT_GT(first_safety_seconds, 0.0);
  const double latency_sec = first_safety_seconds - injected_at;
  RecordProperty("switch_latency_sec", std::to_string(latency_sec));
  EXPECT_GE(latency_sec, 0.0);
  EXPECT_LE(latency_sec, 0.1)
    << "claim 2: " << latency_sec << " s exceeds the 100 ms ceiling -- S:6 says this means "
    << "the arbiter is waiting on a timer instead of passing the message through";
}

// ===========================================================================
// G-CA1 item 3: no gap in the setpoint stream through a live switch --
// max inter-arrival gap on command_selected, measured at the subscriber.
// ===========================================================================
TEST_F(ControlAuthorityFixture, NoGapInCommandSelectedThroughAHandover)
{
  startStallWitness();
  std::atomic<bool> stop{false};
  {
    StreamGuard mission_thread(
      streamSource(mission_publisher_, kSourceMission, 0.0, 1.0, &stop), stop);
    ASSERT_TRUE(waitFor([this]() {return countBySource(kSourceMission) >= 5U;}, 5.0));
  }
  stop = false;    // StreamGuard's destructor set it for MISSION; SAFETY needs a clean flag

  std::thread safety_thread = streamSource(safety_publisher_, kSourceSafety, 5.0, 1.0, &stop);
  safety_thread.join();

  const std::vector<CommandSample> flown = commands();
  ASSERT_GE(flown.size(), 10U) << "FAILED TO MEASURE: too few samples to bound a gap";

  double max_gap = 0.0;
  for (std::size_t index = 1; index < flown.size(); ++index) {
    max_gap = std::max(max_gap, flown[index].wall_seconds - flown[index - 1].wall_seconds);
  }
  RecordProperty("max_gap_sec", std::to_string(max_gap));

  // R27-1: check the measurement before the object. The witness ticks in the same
  // executor as the subscription, so if it also missed by ~the gap under test, the
  // stall was here and this run cannot say anything about the arbiter. Reported as
  // skipped, never folded into a PASS and never blamed on the product.
  const double stall = witnessedStallSec();
  RecordProperty("probe_stall_max_sec", std::to_string(stall));
  std::cout << "[ EVIDENCE ] command_selected max gap " << max_gap
            << " s, probe stall " << stall << " s" << std::endl;
  if (!gapIsMeasurable(max_gap, stall)) {
    GTEST_SKIP() << "FAILED TO MEASURE: this probe's own executor stalled " << stall
                 << " s while measuring a " << max_gap << " s gap on command_selected -- "
                 << "the two are indistinguishable, so no verdict is given (N-g)";
  }

  EXPECT_LE(max_gap, 0.15) << "claim 3: a gap of " << max_gap << " s reached command_selected"
                           << " (probe stalled at most " << stall << " s, so this is real)";
  EXPECT_LT(max_gap, 0.50) << "claim 3 (absolute): would already have starved the gateway";
}

// ===========================================================================
// R27-3 positive control for the guard above. A shield never seen to bite is
// not evidence, and the stall that produced N-g (12 packages under a parallel
// ctest) cannot be summoned on demand. So reproduce its MECHANISM: the witness
// timer and the command subscription share probe_'s default, MUTUALLY EXCLUSIVE
// callback group, so one sleeping callback freezes both -- which is what a
// scheduler stall does to this probe. Two halves, and the second matters more:
//   (a) the witness actually SEES a stall of the size injected, and
//   (b) the rule suppresses a verdict ONLY when the probe is the suspect --
//       a real gap measured by a healthy probe must still be judged, or the
//       guard would be a blanket amnesty that hides every product fault.
// ===========================================================================
TEST_F(ControlAuthorityFixture, TheStallWitnessSeesAnInjectedStallAndStillJudgesAHealthyProbe)
{
  // (a) the sensor half -- inject a stall and check the witness reports it.
  startStallWitness();

  std::atomic<bool> fired{false};
  auto stall_timer = probe_->create_wall_timer(
    50ms, [&fired]() {
      if (fired.exchange(true)) {
        return;                          // one-shot: only the first tick sleeps
      }
      std::this_thread::sleep_for(std::chrono::duration<double>(kInjectedStallSec));
    });
  ASSERT_TRUE(waitFor([&fired]() {return fired.load();}, 5.0))
    << "FAILED TO MEASURE: the injecting timer never ran";
  ASSERT_TRUE(
    waitFor([this]() {return witnessedStallSec() >= 0.5 * kInjectedStallSec;}, 5.0))
    << "the witness never noticed a " << kInjectedStallSec << " s freeze of its own "
    << "executor -- the guard in NoGapInCommandSelectedThroughAHandover cannot fire";
  stall_timer->cancel();

  const double witnessed = witnessedStallSec();
  RecordProperty("injected_stall_sec", std::to_string(kInjectedStallSec));
  RecordProperty("witnessed_stall_sec", std::to_string(witnessed));
  // RecordProperty only reaches the XML; print it so any plain run shows the number.
  std::cout << "[ EVIDENCE ] injected " << kInjectedStallSec << " s freeze, witness saw "
            << witnessed << " s" << std::endl;
  EXPECT_GE(witnessed, kInjectedStallSec - 0.10)
    << "the witness under-reported the freeze it was designed to catch";

  // (b) the rule half. The N-g run itself: a 0.80 s gap seen by a probe that
  // froze 0.80 s says nothing about the arbiter.
  EXPECT_FALSE(gapIsMeasurable(0.80, 0.80));
  EXPECT_FALSE(gapIsMeasurable(0.80, 0.79));
  // ...but these must still be judged, or the guard buries real faults:
  EXPECT_TRUE(gapIsMeasurable(0.80, 0.02)) << "a real 0.80 s outage would be excused";
  EXPECT_TRUE(gapIsMeasurable(0.16, 0.01)) << "a gap just over the 0.15 ceiling would be excused";
}

// ===========================================================================
// G-CA1 item 3, DOWN direction (Y1): the test above is an UP handover --
// SAFETY beats MISSION on priority alone, no dwell involved. The real
// dwell-gated silence happens when a HIGHER channel goes quiet and a LOWER,
// still-live one has to wait release_dwell_sec before taking over. Real
// (shipped) timing on purpose: the number that matters is the absolute one
// against the gateway's 0.5 s cutoff, not a widened test artifact.
// Expected order of magnitude: release_dwell_sec + monitor tick + 1/stream_hz
// = 0.20 + 0.05 + 0.05 = 0.30 s.
// ===========================================================================
TEST_F(ControlAuthorityFixture, ReleaseHandoverGapStaysUnderTheGatewayTimeout)
{
  // Discovery BEFORE the streams: starting TEST first let its own window be
  // eaten by the subscriber wait, so under load TEST was already dead by the
  // time MISSION went quiet -- that was the flake, not the arbiter (R21).
  ASSERT_TRUE(
    waitFor(
      [this]() {
        return probe_->count_subscribers(prefix_ + "/control/cmd_test") > 0 &&
        probe_->count_subscribers(prefix_ + "/control/cmd_mission") > 0;
      }, 5.0)) << "FAILED TO MEASURE: arbiter never subscribed to both channels";

  std::atomic<bool> stop{false};
  // TEST must outlive MISSION(1 s) + release_dwell_sec + the observation wait;
  // sized from those terms, not tuned against a red run.
  StreamGuard test_thread(streamSource(test_publisher_, kSourceTest, 2.0, 8.0, &stop), stop);

  // MISSION streams for exactly 1 s, on its own thread with no stop flag, then
  // goes silent by itself -- release_dwell_sec must be what hands control down.
  std::thread mission_thread = streamSource(mission_publisher_, kSourceMission, 1.0, 1.0, nullptr);
  mission_thread.join();
  ASSERT_GE(countBySource(kSourceMission), 5U) << "FAILED TO MEASURE: MISSION never got moving";

  ASSERT_TRUE(waitFor([this]() {return handoverToTestObserved();}, 5.0))
    << "FAILED TO MEASURE: TEST never took over once MISSION went silent";

  const std::vector<CommandSample> flown = commands();
  double last_mission_seconds = -1.0;
  double first_test_after_mission_seconds = -1.0;
  for (const CommandSample & sample : flown) {
    if (sample.source == kSourceMission) {
      last_mission_seconds = sample.wall_seconds;
    } else if (sample.source == kSourceTest && last_mission_seconds > 0.0 &&
      first_test_after_mission_seconds < 0.0)
    {
      first_test_after_mission_seconds = sample.wall_seconds;
    }
  }
  ASSERT_GT(last_mission_seconds, 0.0) << "FAILED TO MEASURE: no MISSION sample seen";
  ASSERT_GT(first_test_after_mission_seconds, 0.0)
    << "FAILED TO MEASURE: no TEST sample seen after MISSION's last one";

  const double gap_sec = first_test_after_mission_seconds - last_mission_seconds;
  RecordProperty("release_handover_gap_sec", std::to_string(gap_sec));
  EXPECT_LT(gap_sec, 0.50)
    << "claim 3 (DOWN direction): " << gap_sec << " s of silence on command_selected -- "
    << "the gateway's own stale-command cutoff is 0.5 s";
}

// ===========================================================================
// G-CA1 item 4: handover position jump, MEASURED ONLY (S:6 item 4 -- no
// threshold until P8 has a real SAFETY producer to tune against).
// ===========================================================================
TEST_F(ControlAuthorityFixture, HandoverPositionJumpIsMeasuredOnly)
{
  publishOn(mission_publisher_, makeCommand(kSourceMission, 0.0));
  ASSERT_TRUE(waitFor([this]() {return countBySource(kSourceMission) > 0;}, 5.0));

  publishOn(safety_publisher_, makeCommand(kSourceSafety, 5.0));
  ASSERT_TRUE(waitFor([this]() {return countBySource(kSourceSafety) > 0;}, 5.0));

  double last_mission_x = std::numeric_limits<double>::quiet_NaN();
  double first_safety_x = std::numeric_limits<double>::quiet_NaN();
  for (const CommandSample & sample : commands()) {
    if (sample.source == kSourceMission) {
      last_mission_x = sample.x;
    } else if (sample.source == kSourceSafety && std::isnan(first_safety_x)) {
      first_safety_x = sample.x;
    }
  }
  ASSERT_FALSE(std::isnan(last_mission_x));
  ASSERT_FALSE(std::isnan(first_safety_x));
  RecordProperty("handover_jump_m", std::to_string(std::abs(first_safety_x - last_mission_x)));
}

// ===========================================================================
// G-CA1 item 5: the active source never flips on a single missed tick from
// itself while a lower-priority channel keeps streaming underneath it.
//
// Timing is widened (source_timeout_sec/release_dwell_sec 1.0 s instead of
// the shipped 0.20 s) so this integration test measures the MECHANISM, not
// WSL executor scheduling jitter -- the shipped boundary is already pinned,
// jitter-free, by test_authority_arbiter.cpp's TimeBoundary suite.
// ===========================================================================
class WidenedHysteresisFixture : public ControlAuthorityFixture
{
protected:
  std::vector<rclcpp::Parameter> extraParameterOverrides() override
  {
    return {
      rclcpp::Parameter("source_timeout_sec", 1.0),
      rclcpp::Parameter("release_dwell_sec", 1.0),
      rclcpp::Parameter("downstream_command_timeout_sec", 2.0),
    };
  }
};

TEST_F(WidenedHysteresisFixture, ActiveSourceNeverFlipsOnASingleMissedTick)
{
  std::atomic<bool> stop{false};
  StreamGuard test_thread(streamSource(test_publisher_, kSourceTest, 0.0, 4.0, &stop), stop);
  ASSERT_TRUE(waitFor([this]() {return countBySource(kSourceTest) >= 3U;}, 5.0));

  // Establish MISSION as the steady-state active source FIRST -- that
  // takeover is itself a legitimate, expected flip and must not be counted
  // as part of what this test measures.
  publishOn(mission_publisher_, makeCommand(kSourceMission, 1.0));
  ASSERT_TRUE(
    waitFor(
      [this]() {
        const std::vector<AuthoritySample> log = authorities();
        return !log.empty() && log.back().active_source == kSourceMission;
      }, 5.0))
    << "FAILED TO MEASURE: MISSION never took over from TEST";
  clearLogs();

  // MISSION streams at 20 Hz for 2 s but skips exactly ONE tick midway --
  // a single missed publish, not a real outage. TEST keeps streaming underneath.
  for (int tick = 0; tick < 40; ++tick) {
    if (tick != 20) {
      publishOn(mission_publisher_, makeCommand(kSourceMission, 1.0));
    }
    std::this_thread::sleep_for(50ms);
  }

  int flips = 0;
  uint8_t previous = ControlAuthority::SOURCE_NONE;
  bool first = true;
  for (const AuthoritySample & sample : authorities()) {
    if (!first && sample.active_source != previous) {
      ++flips;
    }
    previous = sample.active_source;
    first = false;
  }
  RecordProperty("active_source_flips", std::to_string(flips));
  EXPECT_EQ(flips, 0) << "MISSION's own jitter displaced it as active_source";
  EXPECT_GT(countBySource(kSourceMission), 0U) << "FAILED TO MEASURE: MISSION never got through";
  EXPECT_EQ(countBySource(kSourceTest), 0U) << "TEST must never have won while MISSION jittered";
}

// ===========================================================================
// G-CA1 item 6: NaN / wrong frame / unsupported mode are all blocked, none
// reach the wire, and diagnostics reflects each (S:3c: NaN and frame are
// ERROR; an unsupported mode is WARN -- the gateway already logs it that way).
// ===========================================================================
TEST_F(ControlAuthorityFixture, MalformedMissionCommandsAreBlockedAndDiagnosed)
{
  ControlCommand nan_command = makeCommand(kSourceMission, 0.0);
  nan_command.position.y = std::numeric_limits<float>::quiet_NaN();
  publishOn(mission_publisher_, nan_command);

  ControlCommand wrong_frame = makeCommand(kSourceMission, 0.0);
  wrong_frame.header.frame_id = "map";
  publishOn(mission_publisher_, wrong_frame);

  ControlCommand weird_mode = makeCommand(kSourceMission, 0.0, ControlCommand::MODE_ATTITUDE);
  publishOn(mission_publisher_, weird_mode);

  // Y8: wait for the diagnostics EVENT the 3 bad messages must have produced,
  // never a wall-clock sleep window (R21).
  ASSERT_TRUE(waitFor([this]() {return hasDiagnostics();}, 5.0))
    << "FAILED TO MEASURE: diagnostics never published";
  const DiagnosticArray report = diagnostics();

  auto levelFor = [&report](const std::string & suffix) -> int {
      for (const DiagnosticStatus & status : report.status) {
        if (status.name.size() >= suffix.size() &&
          status.name.compare(status.name.size() - suffix.size(), suffix.size(), suffix) == 0)
        {
          return status.level;
        }
      }
      return -1;
    };

  EXPECT_EQ(levelFor("dropped not_finite"), DiagnosticStatus::ERROR);
  EXPECT_EQ(levelFor("dropped wrong_frame"), DiagnosticStatus::ERROR);
  EXPECT_EQ(levelFor("dropped unsupported_mode"), DiagnosticStatus::WARN);

  // Y8 (R21): the diagnostics event already proves the 3 bad messages were
  // SEEN; a wall clock still cannot prove they never reached command_selected
  // -- only a positive counter-check on the SAME writer/topic can (DDS keeps
  // writer->reader order). Publish one distinctly-marked VALID command and
  // wait for it: once it arrives, everything the same publisher sent earlier
  // is guaranteed already resolved.
  constexpr double kSentinelX = 999.0;
  publishOn(mission_publisher_, makeCommand(kSourceMission, kSentinelX));
  ASSERT_TRUE(
    waitFor(
      [this]() {
        for (const CommandSample & sample : commands()) {
          if (sample.source == kSourceMission && sample.x == kSentinelX) {
            return true;
          }
        }
        return false;
      }, 5.0))
    << "FAILED TO MEASURE: the sentinel valid command never reached command_selected";

  const std::vector<CommandSample> flown = commands();
  EXPECT_EQ(flown.size(), 1U)
    << "a malformed command reached command_selected ahead of the sentinel";
  ASSERT_FALSE(flown.empty());
  EXPECT_EQ(flown.front().x, kSentinelX);
}

// ===========================================================================
// G-CA1 item 7 (Y4 chốt, rewritten 2026-08-20): a latch whose holder never
// publishes has NO EFFECT AT ALL -- it must NEVER exclude a mission that is
// actually flying. This replaces the pre-Y4 expectation (grant used to
// exclude MISSION immediately); see test_authority_arbiter.cpp's
// Latch.NeverAliveDoesNotExcludeALowerLiveChannel for the arbiter-level pin.
// The latch still quietly self-destructs after latch_grace_sec regardless.
// ===========================================================================
TEST_F(ControlAuthorityFixture, ALatchThatNeverPublishesNeverExcludesMissionAndStillExpires)
{
  std::atomic<bool> stop{false};
  StreamGuard mission_thread(
    streamSource(mission_publisher_, kSourceMission, 0.0, 10.0, &stop), stop);
  ASSERT_TRUE(waitFor([this]() {return countBySource(kSourceMission) >= 3U;}, 5.0));

  ASSERT_TRUE(authority_client_->wait_for_service(5s));
  auto request = std::make_shared<SetControlAuthority::Request>();
  request->requested_source = ControlAuthority::SOURCE_OPERATOR;
  request->reason = "G-CA1 item 7 (Y4)";
  auto future = authority_client_->async_send_request(request);
  ASSERT_EQ(future.wait_for(5s), std::future_status::ready);
  EXPECT_TRUE(future.get()->success);

  // Y4: granted, but OPERATOR never publishes. Watch for the whole grace
  // window (shipped 2.0 s) plus margin -- MISSION must never be excluded.
  // Wall-clock valve for a NEGATIVE claim (R21); the positive counter-check
  // right after is what actually proves the pipe was alive throughout.
  clearLogs();
  startStallWitness();
  constexpr double kWatchSeconds = 2.5;    // > shipped latch_grace_sec (2.0 s)
  std::this_thread::sleep_for(std::chrono::duration<double>(kWatchSeconds));

  // A NONE sample is only a fault if MISSION was STILL PUBLISHING when it was
  // emitted. Once MISSION has been quiet for source_timeout_sec the arbiter is
  // required to report NONE -- and under load it is this fixture's own stream
  // thread that goes quiet (measured 2.24-2.51 s elsewhere in this suite), not
  // the aircraft's. Judging every NONE convicted the arbiter of obeying S:3b.
  const double timeout = uav_control_authority::ArbiterParams{}.source_timeout_sec;
  ASSERT_GT(countBySource(kSourceMission), 0U)
    << "FAILED TO MEASURE: MISSION traffic never reached command_selected during the watch";
  ASSERT_LT(witnessedStallSec(), timeout / 2.0)
    << "FAILED TO MEASURE: the probe itself stalled " << witnessedStallSec()
    << " s, long enough to fake the silence the arbiter is entitled to act on";
  RecordProperty(
    "worst_mission_silence_sec",
    std::to_string(worstIncumbentSilenceSec(commands(), kSourceMission)));

  EXPECT_EQ(countUnjustifiedNone(commands(), authorities(), kSourceMission, timeout), 0U)
    << "Y4: a latch whose holder never published excluded MISSION anyway"
    << "\n  worst silence in this fixture's own MISSION stream: "
    << worstIncumbentSilenceSec(commands(), kSourceMission) << " s (timeout " << timeout << " s)";
}

// ===========================================================================
// G-CA1 item 8: a second publisher on command_selected is detected and
// reported as an ERROR diagnostic. Y2 moved the publisher-graph query off
// the 20 Hz monitor tick onto the 1 Hz diagnostics cadence (kMonitorPeriodSec
// * diagnostics_every_ticks_ = 1.0 s), so the worst case detection latency is
// now bounded by ONE diagnostics period, not by the old 10/20 Hz tick -- the
// valve below reflects that real trade-off instead of the pre-Y2 "<1 s".
// ===========================================================================
TEST_F(ControlAuthorityFixture, ASecondPublisherOnCommandSelectedIsDetected)
{
  auto rogue = probe_->create_publisher<ControlCommand>(
    prefix_ + "/control/command_selected", rclcpp::QoS(10));
  ControlCommand rogue_command = makeCommand(kSourceTest, 42.0);
  rogue_command.header.stamp = probe_->now();

  const double published_at = rclcpp::Clock().now().seconds();
  rogue->publish(rogue_command);

  ASSERT_TRUE(
    waitFor(
      [this]() {
        for (const DiagnosticStatus & status : diagnostics().status) {
          if (status.name.find("publisher count") != std::string::npos &&
            status.level == DiagnosticStatus::ERROR)
          {
            return true;
          }
        }
        return false;
      }, 2.0))
    << "FAILED TO MEASURE: the second publisher on command_selected was never flagged";
  RecordProperty(
    "duplicate_publisher_detection_sec",
    std::to_string(rclcpp::Clock().now().seconds() - published_at));
}

// ===========================================================================
// G-CA1 item 9 (P8 R2): a SAFETY latch that has NEVER published still
// excludes a live MISSION -- the deliberate INHIBIT mode. Zero NEW messages
// reach command_selected while it holds, WITH a positive counter-check
// (R21): MISSION keeps streaming the whole time, and resumes reaching
// command_selected the moment clear_safety_latch releases the latch -- proof
// the zero above was the INHIBIT floor, not a broken pipe.
// ===========================================================================
TEST_F(ControlAuthorityFixture, SafetyLatchNeverPublishedStillExcludesALiveMissionInhibit)
{
  std::atomic<bool> stop{false};
  StreamGuard mission_thread(
    streamSource(mission_publisher_, kSourceMission, 0.0, 10.0, &stop), stop);
  ASSERT_TRUE(waitFor([this]() {return countBySource(kSourceMission) >= 3U;}, 5.0))
    << "FAILED TO MEASURE: MISSION never got moving";

  ASSERT_TRUE(authority_client_->wait_for_service(5s));
  auto latch_request = std::make_shared<SetControlAuthority::Request>();
  latch_request->requested_source = ControlAuthority::SOURCE_SAFETY;
  latch_request->reason = "G-CA1 item 9 (P8 R2, INHIBIT)";
  auto latch_future = authority_client_->async_send_request(latch_request);
  ASSERT_EQ(latch_future.wait_for(5s), std::future_status::ready);
  EXPECT_TRUE(latch_future.get()->success);

  // INHIBIT: watch a window comfortably past the shipped source_timeout_sec/
  // release_dwell_sec (0.20 s each) while MISSION keeps streaming underneath.
  clearLogs();
  std::this_thread::sleep_for(1000ms);
  EXPECT_EQ(commands().size(), 0U)
    << "P8 R2: a never-published SAFETY latch let a command through command_selected";

  ASSERT_TRUE(clear_safety_latch_client_->wait_for_service(5s));
  auto clear_request = std::make_shared<ClearFault::Request>();
  clear_request->fault_code = "G-CA1 item 9";
  auto clear_future = clear_safety_latch_client_->async_send_request(clear_request);
  ASSERT_EQ(clear_future.wait_for(5s), std::future_status::ready);
  EXPECT_TRUE(clear_future.get()->success);

  ASSERT_TRUE(waitFor([this]() {return countBySource(kSourceMission) > 0;}, 5.0))
    << "FAILED TO MEASURE: MISSION never resumed reaching command_selected after "
    << "clear_safety_latch -- the zero count above may have been a broken pipe, not INHIBIT";
}

// ===========================================================================
// G-CA1 item 10 (P8 R2 regression guard): the SAFETY-only exception must
// never leak to OPERATOR -- a latch that has never published still does NOT
// cut a live MISSION, and its grace-expiry still runs exactly as before P8.
// Deliberately close to item 7 above; kept as its own explicit pin so a
// future change near the SAFETY branch cannot silently widen scope without a
// red test right here.
// ===========================================================================
TEST_F(ControlAuthorityFixture, OperatorLatchNeverPublishedStillDoesNotCutMissionRegressionGuard)
{
  std::atomic<bool> stop{false};
  StreamGuard mission_thread(
    streamSource(mission_publisher_, kSourceMission, 0.0, 10.0, &stop), stop);
  ASSERT_TRUE(waitFor([this]() {return countBySource(kSourceMission) >= 3U;}, 5.0))
    << "FAILED TO MEASURE: MISSION never got moving";

  ASSERT_TRUE(authority_client_->wait_for_service(5s));
  auto request = std::make_shared<SetControlAuthority::Request>();
  request->requested_source = ControlAuthority::SOURCE_OPERATOR;
  request->reason = "G-CA1 item 10 (P8 R2 regression guard)";
  auto future = authority_client_->async_send_request(request);
  ASSERT_EQ(future.wait_for(5s), std::future_status::ready);
  EXPECT_TRUE(future.get()->success);

  clearLogs();
  std::this_thread::sleep_for(1000ms);
  EXPECT_GT(countBySource(kSourceMission), 0U)
    << "P8 R2 regression: an OPERATOR latch that never published cut MISSION -- "
    << "the SAFETY-only exception must never widen to OPERATOR";

  // Grace-expiry must still run for OPERATOR exactly as before P8 -- wait
  // past the shipped latch_grace_sec (2.0 s) and confirm the Y15 diagnostics
  // key reports the latch gone.
  ASSERT_TRUE(
    waitFor(
      [this]() {
        for (const DiagnosticStatus & status : diagnostics().status) {
          if (status.name.find("arbitration") == std::string::npos) {
            continue;
          }
          for (const diagnostic_msgs::msg::KeyValue & kv : status.values) {
            if (kv.key == "latch_active" && kv.value == "false") {
              return true;
            }
          }
        }
        return false;
      }, 4.0))
    << "FAILED TO MEASURE: OPERATOR's never-alive latch never grace-expired (Y4 regression)";
}

// ===========================================================================
// G-CA1 item 11 (P8 R2): clear_safety_latch is the only path that releases a
// SAFETY latch -- MISSION reclaims within about one MISSION stream tick, and
// a plain SetControlAuthority(NONE) release is still rejected at SAFETY (Y6
// unchanged).
// ===========================================================================
TEST_F(
  ControlAuthorityFixture,
  ClearSafetyLatchReleasesInhibitAndSetControlAuthorityReleaseStillRejected)
{
  std::atomic<bool> stop{false};
  StreamGuard mission_thread(
    streamSource(mission_publisher_, kSourceMission, 0.0, 10.0, &stop), stop);
  ASSERT_TRUE(waitFor([this]() {return countBySource(kSourceMission) >= 3U;}, 5.0))
    << "FAILED TO MEASURE: MISSION never got moving";

  ASSERT_TRUE(authority_client_->wait_for_service(5s));
  auto latch_request = std::make_shared<SetControlAuthority::Request>();
  latch_request->requested_source = ControlAuthority::SOURCE_SAFETY;
  latch_request->reason = "G-CA1 item 11";
  auto latch_future = authority_client_->async_send_request(latch_request);
  ASSERT_EQ(latch_future.wait_for(5s), std::future_status::ready);
  ASSERT_TRUE(latch_future.get()->success);

  clearLogs();
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(commands().size(), 0U) << "FAILED TO MEASURE: INHIBIT never actually took hold";

  // Y6 unchanged: a plain RELEASE via SetControlAuthority is still rejected
  // at SAFETY -- clear_safety_latch is the only path, exactly as before P8.
  auto release_request = std::make_shared<SetControlAuthority::Request>();
  release_request->requested_source = ControlAuthority::SOURCE_NONE;
  release_request->reason = "G-CA1 item 11 (Y6 regression)";
  auto release_future = authority_client_->async_send_request(release_request);
  ASSERT_EQ(release_future.wait_for(5s), std::future_status::ready);
  EXPECT_FALSE(release_future.get()->success)
    << "Y6 regression: SetControlAuthority(NONE) released a SAFETY latch";
  EXPECT_TRUE(commands().empty())
    << "the rejected release must not have let anything through either";

  // The real release path.
  ASSERT_TRUE(clear_safety_latch_client_->wait_for_service(5s));
  const double cleared_at = rclcpp::Clock().now().seconds();
  auto clear_request = std::make_shared<ClearFault::Request>();
  clear_request->fault_code = "G-CA1 item 11";
  auto clear_future = clear_safety_latch_client_->async_send_request(clear_request);
  ASSERT_EQ(clear_future.wait_for(5s), std::future_status::ready);
  EXPECT_TRUE(clear_future.get()->success);

  ASSERT_TRUE(waitFor([this]() {return countBySource(kSourceMission) > 0;}, 5.0))
    << "FAILED TO MEASURE: MISSION never resumed after clear_safety_latch";
  double first_after_clear = -1.0;
  for (const CommandSample & sample : commands()) {
    if (sample.source == kSourceMission) {
      first_after_clear = sample.wall_seconds;
      break;
    }
  }
  ASSERT_GT(first_after_clear, 0.0);
  const double gap_sec = first_after_clear - cleared_at;
  RecordProperty("clear_safety_latch_reclaim_gap_sec", std::to_string(gap_sec));
  EXPECT_GE(gap_sec, 0.0);
  EXPECT_LT(gap_sec, 0.50)
    << "claim (item 11): " << gap_sec << " s to reclaim after clear_safety_latch -- "
    << "should track ~1 MISSION stream tick (0.05 s), well under the gateway's 0.5 s cutoff";
}

}  // namespace uav_control_authority
