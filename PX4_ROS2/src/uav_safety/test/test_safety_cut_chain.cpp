// G-S1 (P8-safety.md S:8): the REAL safety_supervisor_node (enforcement_
// enabled=true) driving the REAL control_authority_manager_node, in ONE
// process, through the ACTUAL /control/cmd_mission -> arbiter ->
// /control/command_selected pipe (R27-4: through the mach it protects, not
// a stand-in). A probe fabricates the 10 raw input topics + cmd_mission
// (R20), hence the isolated domain.
//
// Item numbering follows the P8.4 task brief: 0,1,2,3,4,5,6,12,13 are
// implemented here; 7,8 (HOLD) are P8.5; 9,10,11 (Y4 exception, clear_
// safety_latch mechanics) are already pinned by uav_control_authority's own
// G-CA1 suite and are referenced, not repeated (P8.2).
#include <iostream>
#include <atomic>
#include <chrono>
#include <cmath>
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
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <uav_interfaces/msg/control_command.hpp>
#include <uav_interfaces/msg/localization_status.hpp>
#include <uav_interfaces/msg/obstacle_array.hpp>
#include <uav_interfaces/msg/offboard_status.hpp>
#include <uav_interfaces/msg/safety_state.hpp>
#include <uav_interfaces/msg/trajectory3_d.hpp>
#include <uav_interfaces/msg/vehicle_health.hpp>
#include <uav_interfaces/msg/vehicle_state.hpp>
#include <uav_interfaces/srv/clear_fault.hpp>

#include "uav_control_authority/control_authority_manager_node.hpp"
#include "uav_safety/failsafe_policy.hpp"
#include "uav_safety/safety_supervisor_node.hpp"

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
using namespace std::chrono_literals;

namespace uav_safety
{
namespace
{

/// Mutable, thread-safe "what the world currently looks like", republished
/// continuously so age-based measurements stay fresh (same technique as
/// test_safety_supervisor_node.cpp's WorldState).
struct WorldState
{
  std::mutex mutex;
  bool localization_valid = true;
  double battery_remaining = 0.80;
  bool failsafe_active = false;
  uint8_t flight_mode = VehicleState::FLIGHT_MODE_OFFBOARD;
  bool armed = true;              // B5: disarmed must gate the 4 LATCHING codes off
  double mission_x = 0.0;         // cmd_mission position -- moves for BLIND_COMMAND tests
  std::string fused_frame_id = "odom";   // "wrong" triggers FRAME_MISMATCH instantly
  // B1: publish one WRONG-FRAME ControlCommand this round on the named
  // channel (dropped by the arbiter before it can win authority) -- MISSION
  // keeps winning via its own separate, always-valid publish right after.
  bool wrong_frame_on_operator = false;
  bool wrong_frame_on_mission = false;
  // P8.5 (items 7/8): one obstacle inside min_obstacle_distance_copy_m when true.
  bool obstacle_close = false;
  // P8.5 (item 8): drop the odometry_fused publish entirely -- the
  // real-world analogue of a dead VIO/mux, isolating pose loss as the ONLY
  // thing driving the HOLDING -> INHIBITED transition under test.
  bool suppress_odometry = false;
  // P8.5 (item 7b): publish odometry with the degenerate all-zero
  // quaternion (never set to identity) -- pins freezeHoldPoseAndConfirm()'s
  // content-usable gate (finite position + tryYawFromQuaternion).
  bool garbage_orientation = false;
};

DiagnosticStatus keyValueStatus(const std::string & name, double count_value)
{
  DiagnosticStatus status;
  status.name = name;
  KeyValue kv;
  kv.key = "count";
  kv.value = std::to_string(count_value);
  status.values.push_back(kv);
  return status;
}

struct CommandSample
{
  double wall_seconds = 0.0;      // when the probe's callback ran -- OBSERVATION
  double publish_seconds = 0.0;   // header.stamp -- EMISSION, the one that attributes blame
  uint8_t source = 0;
  double x = 0.0;
};

class CutChainFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    static std::atomic<int> instances{0};
    uav_id_ = "uav" + std::to_string(instances.fetch_add(1));
    prefix_ = "/uav/" + uav_id_;

    rclcpp::NodeOptions authority_options;
    authority_options.parameter_overrides({rclcpp::Parameter("uav_id", uav_id_)});
    authority_node_ =
      std::make_shared<uav_control_authority::ControlAuthorityManagerNode>(authority_options);

    rclcpp::NodeOptions safety_options;
    safety_options.parameter_overrides(
      {rclcpp::Parameter("uav_id", uav_id_), rclcpp::Parameter("enforcement_enabled", true)});
    safety_node_ = std::make_shared<SafetySupervisorNode>(safety_options);

    probe_ = std::make_shared<rclcpp::Node>("cut_chain_probe_" + uav_id_);

    localization_status_pub_ =
      probe_->create_publisher<LocalizationStatus>(prefix_ + "/state/localization_status", rclcpp::QoS(10));
    odometry_fused_pub_ = probe_->create_publisher<Odometry>(prefix_ + "/state/odometry_fused", rclcpp::QoS(10));
    localization_diag_pub_ =
      probe_->create_publisher<DiagnosticArray>(prefix_ + "/diagnostics/localization", rclcpp::QoS(10));
    obstacle_map_pub_ =
      probe_->create_publisher<ObstacleArray>(prefix_ + "/world/obstacle_map_local", rclcpp::QoS(10));
    health_px4_pub_ = probe_->create_publisher<VehicleHealth>(prefix_ + "/state/health_px4", rclcpp::QoS(10));
    vehicle_state_pub_ = probe_->create_publisher<VehicleState>(prefix_ + "/state/vehicle", rclcpp::QoS(10));
    offboard_status_pub_ =
      probe_->create_publisher<OffboardStatus>(prefix_ + "/backend/offboard_status", rclcpp::QoS(10));
    cmd_mission_pub_ = probe_->create_publisher<ControlCommand>(prefix_ + "/control/cmd_mission", rclcpp::QoS(10));
    // B1: only used to inject wrong-frame drops on a channel that never
    // holds authority (OPERATOR outranks MISSION by priority, but a
    // WRONG-FRAME message is dropped before it can count as a valid
    // arrival, so it never actually wins).
    cmd_operator_pub_ =
      probe_->create_publisher<ControlCommand>(prefix_ + "/control/cmd_operator", rclcpp::QoS(10));
    camera_health_pub_ =
      probe_->create_publisher<DiagnosticStatus>(prefix_ + "/state/camera_health", rclcpp::QoS(10));
    trajectory_pub_ = probe_->create_publisher<Trajectory3D>(
      prefix_ + "/planning/trajectory", rclcpp::QoS(1).reliable().transient_local());

    command_selected_sub_ = probe_->create_subscription<ControlCommand>(
      prefix_ + "/control/command_selected", rclcpp::QoS(50),
      [this](const ControlCommand::SharedPtr msg) {onCommand(*msg);});
    authority_diag_sub_ = probe_->create_subscription<DiagnosticArray>(
      prefix_ + "/diagnostics/control_authority", rclcpp::QoS(10),
      [this](const DiagnosticArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_authority_diagnostics_ = *msg;
        has_authority_diagnostics_ = true;
      });
    // B4: latch_confirmed_granted key.
    safety_diag_sub_ = probe_->create_subscription<DiagnosticArray>(
      prefix_ + "/diagnostics/safety", rclcpp::QoS(10),
      [this](const DiagnosticArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_safety_diagnostics_ = *msg;
        has_safety_diagnostics_ = true;
      });
    safety_state_sub_ = probe_->create_subscription<SafetyState>(
      prefix_ + "/safety/state", rclcpp::QoS(1).reliable().transient_local(),
      [this](const SafetyState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        safety_states_.push_back(*msg);
      });
    violations_sub_ = probe_->create_subscription<DiagnosticArray>(
      prefix_ + "/safety/violations", rclcpp::QoS(20),
      [this](const DiagnosticArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const DiagnosticStatus & status : msg->status) {
          violation_edges_.push_back(status);
        }
      });

    clear_fault_client_ = probe_->create_client<ClearFault>(prefix_ + "/safety/clear_fault");
    // B4: calls the ARBITER's clear_safety_latch DIRECTLY, bypassing
    // uav_safety's own ClearFault -- simulates "the latch got cleared by
    // someone/something else" (arbiter restart, manual intervention) to pin
    // that safety re-confirms its grant against /diagnostics/control_authority
    // rather than trusting a fire-and-forget local flag forever.
    arbiter_clear_safety_latch_client_ =
      probe_->create_client<ClearFault>(prefix_ + "/control/clear_safety_latch");

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 8);
    executor_->add_node(authority_node_);
    executor_->add_node(safety_node_);
    executor_->add_node(probe_);
    spin_thread_ = std::thread([this]() {executor_->spin();});

    // #0 (S:8): every piece of the chain must be measurably alive before any
    // cut-chain assertion runs -- FAILED TO MEASURE, not a silent pass/fail.
    // service_is_ready() is the correct check for a SERVICE (clear_fault);
    // count_publishers()/count_subscribers() only apply to topics.
    ASSERT_TRUE(
      waitFor(
        [this]() {
          return probe_->count_subscribers(prefix_ + "/control/cmd_mission") > 0 &&
          clear_fault_client_->service_is_ready();
        }, 10.0))
      << "FAILED TO MEASURE: discovery never completed (arbiter or safety service missing)";

    stream_stop_.store(false);
    stream_thread_ = std::thread([this]() {streamWorld();});

    ASSERT_TRUE(waitFor([this]() {return countBySource(kSourceMission) > 0;}, 5.0))
      << "FAILED TO MEASURE: cmd_mission never reached command_selected through the real arbiter";
    ASSERT_TRUE(waitFor([this]() {return !safetyStates().empty();}, 5.0))
      << "FAILED TO MEASURE: safety/state never published";
    std::this_thread::sleep_for(300ms);   // let the command window fill (W=3 ticks)
  }

  void TearDown() override
  {
    stream_stop_.store(true);
    if (stream_thread_.joinable()) {
      stream_thread_.join();
    }
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    authority_node_.reset();
    safety_node_.reset();
    probe_.reset();
    executor_.reset();
  }

  // ------------------------------------------------------------ streaming

  void streamWorld()
  {
    while (!stream_stop_.load()) {
      publishOneRound();
      std::this_thread::sleep_for(20ms);   // ~50 Hz, comfortably above the 20 Hz safety tick
    }
  }

  void publishOneRound()
  {
    bool localization_valid;
    double battery_remaining;
    bool failsafe_active;
    uint8_t flight_mode;
    double mission_x;
    std::string fused_frame_id;
    bool obstacle_close;
    bool suppress_odometry;
    bool garbage_orientation;
    bool armed;
    bool wrong_frame_on_operator;
    bool wrong_frame_on_mission;
    {
      std::lock_guard<std::mutex> lock(world_.mutex);
      localization_valid = world_.localization_valid;
      battery_remaining = world_.battery_remaining;
      failsafe_active = world_.failsafe_active;
      flight_mode = world_.flight_mode;
      mission_x = world_.mission_x;
      fused_frame_id = world_.fused_frame_id;
      obstacle_close = world_.obstacle_close;
      suppress_odometry = world_.suppress_odometry;
      garbage_orientation = world_.garbage_orientation;
      armed = world_.armed;
      wrong_frame_on_operator = world_.wrong_frame_on_operator;
      wrong_frame_on_mission = world_.wrong_frame_on_mission;
    }
    const rclcpp::Time stamp = probe_->now();

    LocalizationStatus localization_status;
    localization_status.header.stamp = stamp;
    localization_status.header.frame_id = "odom";
    localization_status.is_valid = localization_valid;
    localization_status.quality = LocalizationStatus::QUALITY_GOOD;
    localization_status.active_source = LocalizationStatus::SOURCE_FUSED;
    localization_status_pub_->publish(localization_status);

    if (!suppress_odometry) {
      Odometry odometry;
      odometry.header.stamp = stamp;
      odometry.header.frame_id = fused_frame_id;
      odometry.pose.pose.position.z = 2.0;
      // geometry_msgs/Quaternion.msg defaults w=1 (identity) -- a fresh
      // Odometry is ALREADY a valid orientation, not degenerate, so item 7b
      // must explicitly zero w to build a genuinely all-zero quaternion
      // (norm 0) instead of relying on "leave it unset".
      if (garbage_orientation) {
        odometry.pose.pose.orientation.w = 0.0;
      }
      odometry_fused_pub_->publish(odometry);
    }

    DiagnosticArray localization_diag;
    localization_diag.header.stamp = stamp;
    localization_diag.status.push_back(keyValueStatus("localization: continuity", 0.0));
    localization_diag_pub_->publish(localization_diag);

    ObstacleArray obstacles;
    obstacles.header.stamp = stamp;
    obstacles.header.frame_id = "odom";
    if (obstacle_close) {
      uav_interfaces::msg::Obstacle obstacle;
      obstacle.shape = uav_interfaces::msg::Obstacle::SHAPE_SPHERE;
      obstacle.distance = 0.20F;   // < min_obstacle_distance_copy_m (0.35)
      obstacles.obstacles.push_back(obstacle);
    }
    obstacle_map_pub_->publish(obstacles);   // empty unless obstacle_close -- "nothing detected"

    VehicleHealth health;
    health.header.stamp = stamp;
    health.battery_remaining = static_cast<float>(battery_remaining);
    health.imu_ok = true;
    health.gps_ok = true;
    health.barometer_ok = true;
    health.magnetometer_ok = true;
    health.overall_level = VehicleHealth::LEVEL_OK;
    health_px4_pub_->publish(health);

    VehicleState vehicle;
    vehicle.header.stamp = stamp;
    vehicle.armed = armed;   // B5
    vehicle.flight_mode = flight_mode;
    vehicle.failsafe_active = failsafe_active;
    vehicle_state_pub_->publish(vehicle);

    OffboardStatus offboard;
    offboard.header.stamp = stamp;
    offboard.state = OffboardStatus::STATE_ACTIVE;
    offboard.setpoint_rate_hz = 20.0F;
    offboard.command_fresh = true;
    offboard.offboard_confirmed = true;
    offboard_status_pub_->publish(offboard);

    // B1: a WRONG-FRAME message on the HELD channel (dropped, but counted
    // per-channel by the arbiter) published BEFORE the real, valid mission
    // command below -- MISSION never loses authority (its own valid publish
    // right after keeps it live), only its per-channel drop counter rises.
    if (wrong_frame_on_mission) {
      ControlCommand garbage;
      garbage.header.stamp = stamp;
      garbage.header.frame_id = "not_odom";   // WRONG_FRAME drop
      garbage.control_mode = ControlCommand::MODE_POSITION;
      garbage.source = kSourceMission;
      cmd_mission_pub_->publish(garbage);
    }

    ControlCommand mission;
    mission.header.stamp = stamp;
    mission.header.frame_id = "odom";
    mission.control_mode = ControlCommand::MODE_POSITION;
    mission.source = kSourceMission;
    mission.position.x = mission_x;
    mission.position.z = 2.0;
    mission.yaw = 0.0F;
    cmd_mission_pub_->publish(mission);

    // B1: a WRONG-FRAME message on a channel that NEVER holds authority here
    // (OPERATOR outranks MISSION by priority, but a dropped WRONG_FRAME
    // message never counts as a valid arrival, so it can never actually
    // win) -- proves the per-channel fix: this must NEVER affect FRAME_MISMATCH.
    if (wrong_frame_on_operator) {
      ControlCommand garbage;
      garbage.header.stamp = stamp;
      garbage.header.frame_id = "not_odom";
      garbage.control_mode = ControlCommand::MODE_POSITION;
      garbage.source = kSourceOperator;
      cmd_operator_pub_->publish(garbage);
    }

    DiagnosticStatus camera;
    camera.name = "camera health";
    camera.level = DiagnosticStatus::OK;
    camera_health_pub_->publish(camera);

    Trajectory3D trajectory;
    trajectory.header.stamp = stamp;
    trajectory.header.frame_id = "odom";
    trajectory.plan_state = Trajectory3D::PLAN_STATE_VALID;
    trajectory_pub_->publish(trajectory);
  }

  // --------------------------------------------------------------- helpers

  void onCommand(const ControlCommand & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    commands_.push_back(
      CommandSample{
        rclcpp::Clock().now().seconds(),
        rclcpp::Time(msg.header.stamp).seconds(),
        msg.source,
        msg.position.x});
  }

  std::vector<CommandSample> commands()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return commands_;
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

  void clearCommandLog()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    commands_.clear();
  }

  std::vector<SafetyState> safetyStates()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return safety_states_;
  }

  std::vector<DiagnosticStatus> violationEdges()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return violation_edges_;
  }

  void clearLogs()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    commands_.clear();
    safety_states_.clear();
    violation_edges_.clear();
  }

  bool latchActiveTrue()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_authority_diagnostics_) {
      return false;
    }
    for (const DiagnosticStatus & status : last_authority_diagnostics_.status) {
      if (status.name.find("arbitration") == std::string::npos) {
        continue;
      }
      for (const KeyValue & kv : status.values) {
        if (kv.key == "latch_active" && kv.value == "true") {
          return true;
        }
      }
    }
    return false;
  }

  /// B4: safety_supervisor_node's OWN belief (latch_confirmed_granted key,
  /// /diagnostics/safety), re-confirmed every tick against the arbiter's
  /// diagnostics -- distinct from latchActiveTrue() (the arbiter's own truth).
  bool latchConfirmedGrantedTrue()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_safety_diagnostics_) {
      return false;
    }
    for (const DiagnosticStatus & status : last_safety_diagnostics_.status) {
      if (status.name != "safety") {
        continue;
      }
      for (const KeyValue & kv : status.values) {
        if (kv.key == "latch_confirmed_granted") {
          return kv.value == "true";
        }
      }
    }
    return false;
  }

  static const DiagnosticStatus * findEdge(
    const std::vector<DiagnosticStatus> & edges, const std::string & code, const std::string & needle)
  {
    for (const DiagnosticStatus & status : edges) {
      if (status.name == "safety: " + code && status.message.find(needle) != std::string::npos) {
        return &status;
      }
    }
    return nullptr;
  }

  /// Anti-hang valve only (R21): the condition itself is the assertion.
  static bool waitFor(const std::function<bool()> & happened, double valve_seconds)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(valve_seconds);
    while (std::chrono::steady_clock::now() < deadline) {
      if (happened()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return happened();
  }

  /// Synchronous helper: send one clear_fault request, return the response
  /// (blocks the TEST thread on a future, never a callback -- the node's own
  /// callback path stays strictly async/callback per header point 5).
  ClearFault::Response::SharedPtr callClearFault(const std::string & fault_code, double timeout_sec = 2.0)
  {
    auto request = std::make_shared<ClearFault::Request>();
    request->fault_code = fault_code;
    auto future = clear_fault_client_->async_send_request(request);
    if (future.wait_for(std::chrono::duration<double>(timeout_sec)) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  std::string uav_id_;
  std::string prefix_;
  std::shared_ptr<uav_control_authority::ControlAuthorityManagerNode> authority_node_;
  std::shared_ptr<SafetySupervisorNode> safety_node_;
  std::shared_ptr<rclcpp::Node> probe_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;

  WorldState world_;
  std::atomic<bool> stream_stop_{false};
  std::thread stream_thread_;

  rclcpp::Publisher<LocalizationStatus>::SharedPtr localization_status_pub_;
  rclcpp::Publisher<Odometry>::SharedPtr odometry_fused_pub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr localization_diag_pub_;
  rclcpp::Publisher<ObstacleArray>::SharedPtr obstacle_map_pub_;
  rclcpp::Publisher<VehicleHealth>::SharedPtr health_px4_pub_;
  rclcpp::Publisher<VehicleState>::SharedPtr vehicle_state_pub_;
  rclcpp::Publisher<OffboardStatus>::SharedPtr offboard_status_pub_;
  rclcpp::Publisher<ControlCommand>::SharedPtr cmd_mission_pub_;
  rclcpp::Publisher<ControlCommand>::SharedPtr cmd_operator_pub_;   // B1
  rclcpp::Publisher<DiagnosticStatus>::SharedPtr camera_health_pub_;
  rclcpp::Publisher<Trajectory3D>::SharedPtr trajectory_pub_;

  rclcpp::Subscription<ControlCommand>::SharedPtr command_selected_sub_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr authority_diag_sub_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr safety_diag_sub_;   // B4
  rclcpp::Subscription<SafetyState>::SharedPtr safety_state_sub_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr violations_sub_;
  rclcpp::Client<ClearFault>::SharedPtr clear_fault_client_;
  rclcpp::Client<ClearFault>::SharedPtr arbiter_clear_safety_latch_client_;   // B4

  std::mutex mutex_;
  std::vector<CommandSample> commands_;
  std::vector<SafetyState> safety_states_;
  std::vector<DiagnosticStatus> violation_edges_;
  DiagnosticArray last_authority_diagnostics_;
  bool has_authority_diagnostics_ = false;
  DiagnosticArray last_safety_diagnostics_;   // B4
  bool has_safety_diagnostics_ = false;
};

}  // namespace

// ===========================================================================
// #0: measurement preflight -- every piece of the chain proven alive.
// ===========================================================================
TEST_F(CutChainFixture, Item0_PreflightAllChannelsMeasurable)
{
  // SetUp() already asserted this once; re-assert explicitly as its own
  // named, reportable item (R27-1: a cofnt must check the measurement
  // before checking the object).
  EXPECT_GT(countBySource(kSourceMission), 0U) << "FAILED TO MEASURE: cmd_mission not flowing";
  EXPECT_FALSE(safetyStates().empty()) << "FAILED TO MEASURE: safety/state never published";
  EXPECT_TRUE(clear_fault_client_->service_is_ready())
    << "FAILED TO MEASURE: safety/clear_fault not ready";
}

// ===========================================================================
// #1 / #13: BATTERY_PX4_NO_ACTION -- cuts once PX4 has NOT acted past grace;
// does NOT cut if PX4 already acted (failsafe_active or RETURN/LAND).
// Mutation: px4_action_grace_sec=0 must turn this red (run by hand, P8.4
// handoff report).
// ===========================================================================
TEST_F(CutChainFixture, Item1_BatteryPx4NoActionCutsPastGraceWhenPx4HasNotActed)
{
  clearLogs();
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.battery_remaining = 0.05;   // < battery_critical_threshold (0.10)
  }
  const double grace = FailsafePolicyParams{}.px4_action_grace_sec;
  ASSERT_TRUE(
    waitFor(
      [this]() {return findEdge(violationEdges(), "BATTERY_PX4_NO_ACTION", "entered") != nullptr;},
      grace + 3.0))
    << "FAILED TO MEASURE: BATTERY_PX4_NO_ACTION never entered";

  clearCommandLog();
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(countBySource(kSourceMission), 0U)
    << "command_selected kept carrying MISSION after BATTERY_PX4_NO_ACTION latched";
  // latch_active rides /diagnostics/control_authority, which publishes at 1 Hz by
  // contract. Reading it at a fixed instant asked a 1 Hz channel to have spoken
  // inside a 500 ms window: measured 2026-08-25 under load, the latch WAS set and
  // the flag arrived 0.390 s after this line used to judge. The cut itself is the
  // assertion above; this one corroborates it, so wait for the event (R21) instead
  // of for a sample to happen to land.
  const auto probe_opened = std::chrono::steady_clock::now();
  const bool latch_seen = waitFor([this]() {return latchActiveTrue();}, 5.0);
  RecordProperty(
    "latch_flag_observed_sec",
    std::to_string(
      std::chrono::duration<double>(std::chrono::steady_clock::now() - probe_opened).count()));
  EXPECT_TRUE(latch_seen) << "arbiter never reported latch_active=true";
}

TEST_F(CutChainFixture, Item13_Px4AlreadyActingExemptsEvenPastGrace)
{
  clearLogs();
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.battery_remaining = 0.05;
    world_.failsafe_active = true;   // PX4 already acting -- must NOT cut
  }
  const double grace = FailsafePolicyParams{}.px4_action_grace_sec;
  std::this_thread::sleep_for(std::chrono::duration<double>(grace + 1.0));

  clearCommandLog();
  std::this_thread::sleep_for(500ms);
  EXPECT_GT(countBySource(kSourceMission), 0U)
    << "PX4 already acting (failsafe_active) but safety cut anyway";
  EXPECT_FALSE(latchActiveTrue()) << "a latch was requested despite the PX4-acting exemption";
}

// ===========================================================================
// #2: BLIND_COMMAND (bad localization + moving setpoint) cuts MISSION off
// command_selected. Event-anchored claims: violation enters, latch raised,
// cut is ABSOLUTE. Latency is measured and RECORDED, not thresholded here
// (N-safety-d, 2026-08-22): the old grace+2ticks+0.30 budget was derived
// correctly for the product chain but implicitly assumed zero scheduler
// stall -- an idle WSL host stalled the harness 1.2 s (elapsed 3.13 vs
// budget 1.9) while the SAME product measured 0.56 s on-wire at G-S3.
// R21 + the audit-item-5 precedent: threshold only where the rig is
// stall-free; latency budgets stay enforced by the sim gates (G-S2
// blind_distance, G-S3 onset). Mutation: blind_grace_sec=0 still turns
// this red (validate() refuses to start, S:9d pairing rule).
// ===========================================================================
TEST_F(CutChainFixture, Item2_BlindCommandCutsAndItsLatencyIsRecorded)
{
  clearLogs();
  const double onset_wall = rclcpp::Clock().now().seconds();
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.localization_valid = false;
  }
  std::atomic<bool> nudge_stop{false};
  std::thread nudger(
    [this, &nudge_stop]() {
      bool high = false;
      while (!nudge_stop.load()) {
        high = !high;
        {
          std::lock_guard<std::mutex> lock(world_.mutex);
          world_.mission_x = high ? 0.20 : 0.0;   // bounded ping-pong, see P8.3 handoff report
        }
        std::this_thread::sleep_for(20ms);
      }
    });

  const double grace = FailsafePolicyParams{}.blind_grace_sec;
  ASSERT_TRUE(
    waitFor(
      [this]() {return findEdge(violationEdges(), "BLIND_COMMAND", "entered") != nullptr;},
      grace + 5.0))
    << "FAILED TO MEASURE: BLIND_COMMAND never entered";
  ASSERT_TRUE(waitFor([this]() {return latchActiveTrue();}, 5.0))
    << "FAILED TO MEASURE: arbiter never reported latch_active=true after BLIND_COMMAND";

  const std::vector<CommandSample> before_stop = commands();
  nudge_stop.store(true);
  nudger.join();
  ASSERT_TRUE(!before_stop.empty()) << "FAILED TO MEASURE: no MISSION samples ever seen";

  // Absolute cut (same shape as items 3/5): the probe keeps publishing
  // cmd_mission the whole time, yet nothing more reaches command_selected.
  clearCommandLog();
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(countBySource(kSourceMission), 0U)
    << "BLIND_COMMAND latched but MISSION still got through command_selected";

  double last_mission_wall = onset_wall;
  for (const CommandSample & sample : before_stop) {
    if (sample.source == kSourceMission) {
      last_mission_wall = sample.wall_seconds;
    }
  }
  RecordProperty(
    "blind_command_cut_elapsed_sec", std::to_string(last_mission_wall - onset_wall));
}

// ===========================================================================
// #4 (R1): stationary setpoint on bad localization does NOT cut, even well
// past blind_grace_sec. Mutation: frozen_setpoint_epsilon_copy_m=0 must turn this
// red.
// ===========================================================================
TEST_F(CutChainFixture, Item4_StationarySetpointOnBadLocalizationDoesNotCut)
{
  clearLogs();
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.localization_valid = false;
    world_.mission_x = 3.0;   // fixed value -- never changes from here on
  }
  const double grace = FailsafePolicyParams{}.blind_grace_sec;
  std::this_thread::sleep_for(std::chrono::duration<double>(grace + 1.0));

  clearCommandLog();
  std::this_thread::sleep_for(500ms);
  EXPECT_GT(countBySource(kSourceMission), 0U)
    << "R1: a stationary setpoint on bad localization must be a valid frozen hold, not a cut";
  EXPECT_TRUE(findEdge(violationEdges(), "LOCALIZATION_INVALID", "") != nullptr ||
    findEdge(violationEdges(), "LOCALIZATION_STALE", "") != nullptr)
    << "the underlying cause must still be reported, even though it does not cut";
  EXPECT_FALSE(latchActiveTrue()) << "R1: no INHIBIT latch should exist for a stationary hold";
}

// ===========================================================================
// #3 + #5: absolute cut (0 new messages) WITH a positive counter-check
// (cmd_mission itself keeps flowing on the probe side), then ClearFault's
// two directions -- reject while not stably clear, accept once it is, and
// MISSION resumes reaching command_selected.
// ===========================================================================
TEST_F(CutChainFixture, Item3And5_AbsoluteCutThenClearFaultRejectsThenAcceptsAndTrafficResumes)
{
  clearLogs();
  {
    // FRAME_MISMATCH: instantaneous trigger, fastest reliable path to
    // INHIBITED for this test (its own grace/timing is separately pinned by
    // item 2/13's tests and by test_failsafe_policy.cpp's boundary suite).
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.fused_frame_id = "map";   // wrong frame -> fused_frame_mismatch=true
  }
  ASSERT_TRUE(
    waitFor([this]() {return findEdge(violationEdges(), "FRAME_MISMATCH", "entered") != nullptr;}, 3.0))
    << "FAILED TO MEASURE: FRAME_MISMATCH never entered";
  ASSERT_TRUE(waitFor([this]() {return latchActiveTrue();}, 3.0))
    << "FAILED TO MEASURE: arbiter never reported latch_active=true";

  // Absolute cut, positive counter-check: cmd_mission itself never stops
  // (the probe keeps publishing throughout), yet ZERO new command_selected
  // messages carry MISSION while INHIBITED.
  clearCommandLog();
  std::this_thread::sleep_for(800ms);
  EXPECT_EQ(countBySource(kSourceMission), 0U) << "INHIBIT let a message through command_selected";

  // Restore the frame so the cause CAN eventually clear, then reject: not
  // yet stable for clear_stability_sec.
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.fused_frame_id = "odom";
  }
  auto reject_response = callClearFault("FRAME_MISMATCH");
  ASSERT_NE(reject_response, nullptr) << "FAILED TO MEASURE: clear_fault call never completed";
  EXPECT_FALSE(reject_response->success) << "accepted before clear_stability_sec elapsed";

  // Poll until stable-clear, then accept (S:3.1 polling model, P8.1 report
  // point 7).
  const double clear_stability_sec = FailsafePolicyParams{}.clear_stability_sec;
  ClearFault::Response::SharedPtr accept_response;
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(clear_stability_sec + 3.0);
  while (std::chrono::steady_clock::now() < deadline) {
    accept_response = callClearFault("FRAME_MISMATCH");
    if (accept_response && accept_response->success) {
      break;
    }
    std::this_thread::sleep_for(500ms);
  }
  ASSERT_NE(accept_response, nullptr) << "FAILED TO MEASURE: clear_fault stopped responding";
  EXPECT_TRUE(accept_response->success) << "ClearFault never accepted once stably clear";

  ASSERT_TRUE(waitFor([this]() {return countBySource(kSourceMission) > 0;}, 5.0))
    << "FAILED TO MEASURE: MISSION never resumed reaching command_selected after ClearFault";
  // Diagnostics republish at only 1 Hz (control_authority_manager_node.cpp
  // diagnostics_every_ticks_) -- a bare latchActiveTrue() right after the
  // ClearFault response races that cadence and reads a stale sample. Poll.
  EXPECT_TRUE(waitFor([this]() {return !latchActiveTrue();}, 2.0))
    << "arbiter latch still active after a confirmed release";
}

// ===========================================================================
// #6: ClearFault's handover-jump gate -- 2.0 m rejected even when otherwise
// stably clear, 0.3 m accepted.
// ===========================================================================
TEST_F(CutChainFixture, Item6_HandoverJumpRejectsLargeJumpAcceptsSmallOne)
{
  clearLogs();
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.fused_frame_id = "map";
  }
  ASSERT_TRUE(
    waitFor([this]() {return findEdge(violationEdges(), "FRAME_MISMATCH", "entered") != nullptr;}, 3.0))
    << "FAILED TO MEASURE: FRAME_MISMATCH never entered";

  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.fused_frame_id = "odom";     // cause clear
    world_.mission_x = 2.0;             // fused pose is fixed at x=0 -> 2.0 m jump
  }

  // clear_stability_sec only accumulates ACROSS CALLS to clear_fault() (S:
  // 3.1 is a polling model, P8.1 report point 7) -- a single call after a
  // sleep never builds it, it only ever samples "stable_for=0". Poll until
  // the rejection reason itself becomes HANDOVER_JUMP, proving the FRAME_
  // MISMATCH cause's OWN stability gate has already been satisfied and the
  // handover gate is the one now blocking.
  const double clear_stability_sec = FailsafePolicyParams{}.clear_stability_sec;
  ClearFault::Response::SharedPtr big_jump_response;
  bool found_jump_reason = false;
  const auto jump_deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(clear_stability_sec + 5.0);
  while (std::chrono::steady_clock::now() < jump_deadline && !found_jump_reason) {
    big_jump_response = callClearFault("FRAME_MISMATCH");
    ASSERT_NE(big_jump_response, nullptr) << "FAILED TO MEASURE: clear_fault call never completed";
    ASSERT_FALSE(big_jump_response->success) << "2.0 m handover jump was accepted";
    for (const std::string & detail : big_jump_response->remaining_faults) {
      if (detail.find("HANDOVER_JUMP") != std::string::npos) {
        found_jump_reason = true;
      }
    }
    if (!found_jump_reason) {
      std::this_thread::sleep_for(500ms);
    }
  }
  EXPECT_TRUE(found_jump_reason)
    << "FAILED TO MEASURE: never saw a HANDOVER_JUMP rejection -- stability gate never cleared "
    << "within the polling window, so the handover gate itself was never actually exercised";
  EXPECT_TRUE(latchActiveTrue()) << "latch must still be held after a rejected clear";

  // Same FRAME_MISMATCH stability already satisfied above (mission_x plays
  // no part in that tracker) -- switching to a small jump should let the
  // very next call through the handover gate too.
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.mission_x = 0.3;   // small jump -- within handover_jump_limit_m (0.80)
  }
  ClearFault::Response::SharedPtr small_jump_response;
  const auto accept_deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < accept_deadline) {
    small_jump_response = callClearFault("FRAME_MISMATCH");
    if (small_jump_response && small_jump_response->success) {
      break;
    }
    std::this_thread::sleep_for(300ms);
  }
  ASSERT_NE(small_jump_response, nullptr) << "FAILED TO MEASURE: clear_fault stopped responding";
  EXPECT_TRUE(small_jump_response->success) << "0.3 m handover jump was rejected";
}

// ===========================================================================
// #12: NaN battery takes no action -- never cuts, never reads as healthy.
// ===========================================================================
TEST_F(CutChainFixture, Item12_NanBatteryTakesNoAction)
{
  clearLogs();
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.battery_remaining = std::numeric_limits<double>::quiet_NaN();
  }
  ASSERT_TRUE(
    waitFor(
      [this]() {return findEdge(violationEdges(), "BATTERY_CRITICAL", "cannot measure") != nullptr;}, 3.0))
    << "FAILED TO MEASURE: NaN battery never reached safety/violations";

  clearCommandLog();
  std::this_thread::sleep_for(1500ms);   // well past px4_action_grace_sec's own instant no-op
  EXPECT_GT(countBySource(kSourceMission), 0U) << "NaN battery cut command_selected";
  EXPECT_FALSE(latchActiveTrue()) << "NaN battery must never latch the arbiter";
}

// ===========================================================================
// #7 (P8.5): OBSTACLE_TOO_CLOSE seizes control via HOLD -- command_selected
// carries a FROZEN pose (SOURCE_SAFETY) streamed >= 15 Hz on the wire, drift
// -free (fused pose is constant at x=0,z=2.0 throughout, see
// publishOneRound()), while cmd_mission keeps flowing on its own topic
// without ever reaching command_selected (positive counter-check, same
// shape as items 3/5's absolute-cut proof -- the probe's streamer never
// stops publishing cmd_mission regardless of world state).
// ===========================================================================
TEST_F(CutChainFixture, Item7_ObstacleHoldFreezesPoseAndStreamsOnTheWire)
{
  clearLogs();
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.obstacle_close = true;
  }
  ASSERT_TRUE(
    waitFor([this]() {return findEdge(violationEdges(), "OBSTACLE_TOO_CLOSE", "entered") != nullptr;}, 3.0))
    << "FAILED TO MEASURE: OBSTACLE_TOO_CLOSE never entered";
  ASSERT_TRUE(waitFor([this]() {return latchActiveTrue();}, 3.0))
    << "FAILED TO MEASURE: arbiter never reported latch_active=true";
  ASSERT_TRUE(waitFor([this]() {return countBySource(kSourceSafety) > 0;}, 3.0))
    << "FAILED TO MEASURE: no SOURCE_SAFETY sample ever reached command_selected";

  clearCommandLog();
  const auto rate_window_start = std::chrono::steady_clock::now();
  std::this_thread::sleep_for(1000ms);
  const double elapsed_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - rate_window_start).count();

  EXPECT_EQ(countBySource(kSourceMission), 0U) << "HOLD let a MISSION message through command_selected";
  const std::size_t safety_count = countBySource(kSourceSafety);
  const double rate_hz = static_cast<double>(safety_count) / elapsed_sec;
  RecordProperty("hold_stream_rate_hz", std::to_string(rate_hz));
  EXPECT_GE(rate_hz, 15.0)
    << "wire rate " << rate_hz << " Hz over " << elapsed_sec << " s, budget is hold_stream_hz>=15 (V3)";

  // No interpolation/smoothing/drift (S:4): every SAFETY sample on the wire
  // carries the exact same frozen x.
  for (const CommandSample & sample : commands()) {
    if (sample.source == kSourceSafety) {
      EXPECT_NEAR(sample.x, 0.0, 1e-6) << "frozen pose drifted on the wire";
    }
  }
}

// ===========================================================================
// #7b (P8.5): a fresh-by-AGE but garbage-by-CONTENT pose (degenerate
// all-zero quaternion) at the exact moment HOLD would engage must escalate
// straight to INHIBIT -- SOURCE_SAFETY must NEVER reach command_selected
// (freezeHoldPoseAndConfirm()'s content-usable gate, R27-2).
// ===========================================================================
TEST_F(CutChainFixture, Item7b_GarbageOrientationAtEngageEscalatesStraightToInhibitNeverStreams)
{
  clearLogs();
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.obstacle_close = true;
    world_.garbage_orientation = true;
  }
  ASSERT_TRUE(
    waitFor([this]() {return findEdge(violationEdges(), "OBSTACLE_TOO_CLOSE", "entered") != nullptr;}, 3.0))
    << "FAILED TO MEASURE: OBSTACLE_TOO_CLOSE never entered";
  ASSERT_TRUE(waitFor([this]() {return latchActiveTrue();}, 3.0))
    << "FAILED TO MEASURE: arbiter never reported latch_active=true (INHIBIT path)";

  clearCommandLog();
  std::this_thread::sleep_for(800ms);
  EXPECT_EQ(countBySource(kSourceSafety), 0U)
    << "HOLD streamed on a garbage (degenerate all-zero) orientation";
  EXPECT_EQ(countBySource(kSourceMission), 0U)
    << "INHIBIT let a MISSION message through command_selected";
}

// ===========================================================================
// #8 (P8.5): pose lost mid-HOLD -> one-way escalation to INHIBIT (stream
// stops, latch survives), then ClearFault recovers once the obstacle cause
// clears too, and MISSION resumes reaching command_selected.
// ===========================================================================
TEST_F(CutChainFixture, Item8_PoseLostMidHoldEscalatesToInhibitThenClearFaultRecovers)
{
  clearLogs();
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.obstacle_close = true;
  }
  ASSERT_TRUE(waitFor([this]() {return countBySource(kSourceSafety) > 0;}, 3.0))
    << "FAILED TO MEASURE: HOLD never started streaming";

  // Kill the fused pose stream entirely (dead VIO/mux is the real-world
  // analogue) -- everything else (obstacle, cmd_mission, ...) keeps
  // flowing, isolating pose loss as the ONLY thing driving this transition.
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.suppress_odometry = true;
  }
  ASSERT_TRUE(
    waitFor(
      [this]() {return findEdge(violationEdges(), "HOLD pose lost", "") != nullptr;},
      FailsafePolicyParams{}.pose_max_age_sec + 1.0))
    << "FAILED TO MEASURE: HOLD pose lost event never published";

  // Positive counter-check (R21): a SAFETY sample DID exist before
  // suppression (the stream was really running), and after a comfortable
  // settle window past the cutover, NO further SAFETY sample arrives.
  double last_safety_wall_before = 0.0;
  for (const CommandSample & sample : commands()) {
    if (sample.source == kSourceSafety && sample.wall_seconds > last_safety_wall_before) {
      last_safety_wall_before = sample.wall_seconds;
    }
  }
  ASSERT_GT(last_safety_wall_before, 0.0) << "FAILED TO MEASURE: no SAFETY sample ever seen before suppression";

  const double cutover_wall = rclcpp::Clock().now().seconds();
  clearCommandLog();
  std::this_thread::sleep_for(800ms);
  // Evidence before verdict: if a SAFETY sample did arrive, say whether it was EMITTED
  // before or after the cutover. Arriving late is not the same as being sent late, and
  // only the second one accuses the supervisor.
  for (const CommandSample & sample : commands()) {
    if (sample.source != kSourceSafety) {
      continue;
    }
    std::cout << "[ EVIDENCE ] SAFETY sample: published " << (sample.publish_seconds - cutover_wall)
              << " s relative to cutover, observed " << (sample.wall_seconds - cutover_wall)
              << " s relative to cutover"
              << (sample.publish_seconds < cutover_wall
                  ? "  -> EMITTED BEFORE the cutover (late delivery, not a late publish)"
                  : "  -> EMITTED AFTER the cutover (the supervisor really did publish)")
              << std::endl;
  }
  // R21: blame follows the EMISSION stamp. A command already in flight when the pose was
  // lost arrives after the cutover under load without the supervisor having published it
  // late -- counting arrivals accused the supervisor of the middleware's latency, and did
  // so on roughly two runs in three whenever the machine was busy.
  size_t emitted_after_cutover = 0;
  for (const CommandSample & sample : commands()) {
    if (sample.source == kSourceSafety && sample.publish_seconds >= cutover_wall) {
      ++emitted_after_cutover;
    }
  }
  EXPECT_EQ(emitted_after_cutover, 0U)
    << "HOLD stream kept publishing after pose was lost (" << countBySource(kSourceSafety)
    << " SAFETY sample(s) arrived after the cutover, " << emitted_after_cutover
    << " of them emitted after it)";
  EXPECT_EQ(countBySource(kSourceMission), 0U) << "INHIBIT let a MISSION message through after escalation";
  EXPECT_TRUE(latchActiveTrue()) << "latch must survive the HOLDING -> INHIBITED escalation";

  // R5 (coordinator decision, this exact race is what G-S1 #8 found): wait
  // PAST the arbiter's own latch_timeout_sec while still silent. Before R5,
  // this is exactly where the arbiter used to log "latch auto-revoked:
  // holder went silent past latch_timeout_sec" and let MISSION back in --
  // the SAFETY latch must never do that, unlike every other level.
  clearCommandLog();
  const double latch_timeout_sec = uav_control_authority::ArbiterParams{}.latch_timeout_sec;
  std::this_thread::sleep_for(std::chrono::duration<double>(latch_timeout_sec + 1.0));
  EXPECT_EQ(countBySource(kSourceMission), 0U)
    << "R5 regression: latch auto-revoked past latch_timeout_sec, MISSION leaked through";
  EXPECT_TRUE(latchActiveTrue())
    << "R5 regression: SAFETY latch auto-revoked past latch_timeout_sec (must persist until ClearFault)";

  // Restore pose AND clear the obstacle (both causes must be gone), then
  // poll ClearFault until it accepts (S:3.1 polling model). This recovery
  // succeeding, with MISSION flowing again on the SAME command_selected
  // subscription used for the negative checks above, is the R21 positive
  // counter-check anchoring the whole silent window: it proves the probe/
  // arbiter pipe was alive throughout, so "0 messages" meant genuinely
  // blocked, not a dead observer.
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.suppress_odometry = false;
    world_.obstacle_close = false;
  }
  const double clear_stability_sec = FailsafePolicyParams{}.clear_stability_sec;
  ClearFault::Response::SharedPtr accept_response;
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(clear_stability_sec + 5.0);
  while (std::chrono::steady_clock::now() < deadline) {
    accept_response = callClearFault("");
    if (accept_response && accept_response->success) {
      break;
    }
    std::this_thread::sleep_for(500ms);
  }
  ASSERT_NE(accept_response, nullptr) << "FAILED TO MEASURE: clear_fault stopped responding";
  EXPECT_TRUE(accept_response->success) << "ClearFault never accepted once stably clear";

  ASSERT_TRUE(waitFor([this]() {return countBySource(kSourceMission) > 0;}, 5.0))
    << "FAILED TO MEASURE: MISSION never resumed reaching command_selected after ClearFault";
  EXPECT_TRUE(waitFor([this]() {return !latchActiveTrue();}, 2.0))
    << "arbiter latch still active after a confirmed release";
}

// ===========================================================================
// B1 (review 2026-08-21, real bug): FRAME_MISMATCH's sustained-drops path
// must read ONLY the currently HELD channel's own per-channel wrong_frame
// count -- never the arbiter's aggregate.
// ===========================================================================
TEST_F(CutChainFixture, Item_B1_WrongFrameOnAnUnrelatedChannelNeverLatchesFrameMismatch)
{
  clearLogs();
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    // MISSION stays the held channel throughout (OPERATOR's messages are
    // always WRONG_FRAME, so they are dropped before ever counting as a
    // valid arrival -- they can never win authority away from MISSION).
    world_.wrong_frame_on_operator = true;
  }
  const double grace = FailsafePolicyParams{}.frame_mismatch_grace_sec;
  std::this_thread::sleep_for(std::chrono::duration<double>(grace + 3.0));

  clearCommandLog();
  std::this_thread::sleep_for(500ms);
  EXPECT_GT(countBySource(kSourceMission), 0U)
    << "B1 regression: wrong-frame drops on an UNRELATED (never-held) channel latched FRAME_MISMATCH";
  EXPECT_FALSE(latchActiveTrue())
    << "B1 regression: an unrelated channel's errors must never raise the latch";
  EXPECT_EQ(findEdge(violationEdges(), "FRAME_MISMATCH", "entered"), nullptr)
    << "FRAME_MISMATCH must never even REPORT from an unrelated channel's errors";
}

TEST_F(CutChainFixture, Item_B1_WrongFrameOnTheHeldChannelDoesLatchFrameMismatch)
{
  clearLogs();
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    // MISSION IS the held channel -- its own per-channel counter must rise.
    world_.wrong_frame_on_mission = true;
  }
  const double grace = FailsafePolicyParams{}.frame_mismatch_grace_sec;
  ASSERT_TRUE(
    waitFor(
      [this]() {return findEdge(violationEdges(), "FRAME_MISMATCH", "entered") != nullptr;},
      grace + 5.0))
    << "FAILED TO MEASURE: FRAME_MISMATCH never entered from the HELD channel's own errors";
  ASSERT_TRUE(waitFor([this]() {return latchActiveTrue();}, 3.0))
    << "FAILED TO MEASURE: arbiter never reported latch_active=true";

  clearCommandLog();
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(countBySource(kSourceMission), 0U) << "FRAME_MISMATCH latched but MISSION still got through";
}

// ===========================================================================
// B5 (plan S:2, review 2026-08-21): armed && OFFBOARD && command_fresh gates
// the 4 LATCHING codes through the WHOLE real chain (arbiter + safety).
// ===========================================================================
TEST_F(CutChainFixture, Item_B5_DisarmedNeverSeizesControlEvenWithObstacleClose)
{
  clearLogs();
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.armed = false;
    world_.obstacle_close = true;
  }
  const double grace = FailsafePolicyParams{}.obstacle_grace_sec;
  std::this_thread::sleep_for(std::chrono::duration<double>(grace + 2.0));

  clearCommandLog();
  std::this_thread::sleep_for(500ms);
  EXPECT_GT(countBySource(kSourceMission), 0U)
    << "B5 regression: HOLD seized control while disarmed (parked next to an obstacle)";
  EXPECT_EQ(countBySource(kSourceSafety), 0U) << "B5 regression: HOLD streamed while disarmed";
  EXPECT_FALSE(latchActiveTrue()) << "B5 regression: a latch was requested while disarmed";

  // Positive counter-check (R21): arm, confirm the SAME obstacle condition
  // NOW does latch -- proves the gate suppressed the action, not that
  // OBSTACLE_TOO_CLOSE itself was somehow never wired up in this test.
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.armed = true;
  }
  ASSERT_TRUE(waitFor([this]() {return latchActiveTrue();}, grace + 5.0))
    << "FAILED TO MEASURE: arming never let the SAME obstacle condition latch";
}

// ===========================================================================
// B4 (review 2026-08-21, real bug): a SAFETY latch cleared by someone else
// (arbiter restart, manual intervention) must be re-confirmed, not trusted
// on a fire-and-forget local flag -- safety re-requests until granted again.
// ===========================================================================
TEST_F(CutChainFixture, Item_B4_LatchClearedExternallyIsReconfirmedAndReacquired)
{
  clearLogs();
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.fused_frame_id = "map";   // instantaneous FRAME_MISMATCH -> INHIBIT
  }
  ASSERT_TRUE(
    waitFor([this]() {return findEdge(violationEdges(), "FRAME_MISMATCH", "entered") != nullptr;}, 3.0))
    << "FAILED TO MEASURE: FRAME_MISMATCH never entered";
  ASSERT_TRUE(waitFor([this]() {return latchConfirmedGrantedTrue();}, 3.0))
    << "FAILED TO MEASURE: safety never confirmed the SAFETY latch granted";
  ASSERT_TRUE(latchActiveTrue());

  // Simulate "cleared by someone else": call the ARBITER's clear_safety_latch
  // DIRECTLY (bypassing uav_safety's own ClearFault entirely) -- the arbiter
  // executes it unconditionally (P8.2: "always cleared, a no-op success if
  // nothing is held"), exactly as if an operator or a restarted arbiter had
  // done it without safety's knowledge.
  ASSERT_TRUE(arbiter_clear_safety_latch_client_->wait_for_service(3s));
  {
    auto request = std::make_shared<ClearFault::Request>();
    request->fault_code = "external test intervention";
    auto future = arbiter_clear_safety_latch_client_->async_send_request(request);
    ASSERT_EQ(future.wait_for(3s), std::future_status::ready)
      << "FAILED TO MEASURE: direct clear_safety_latch call never completed";
  }
  ASSERT_TRUE(waitFor([this]() {return !latchActiveTrue();}, 3.0))
    << "FAILED TO MEASURE: arbiter never reported the latch gone after the direct clear";

  // B4's fix: safety must notice (via /diagnostics/control_authority) and
  // downgrade its own belief, then re-request -- the latch comes back,
  // because FRAME_MISMATCH is still latched inside safety's own FSM
  // (clearing the ARBITER's latch does not clear uav_safety's own fault).
  ASSERT_TRUE(waitFor([this]() {return latchActiveTrue();}, 5.0))
    << "FAILED TO MEASURE: safety never re-requested and reacquired the SAFETY latch "
       "after noticing the external clear (B4 regression)";
  EXPECT_TRUE(waitFor([this]() {return latchConfirmedGrantedTrue();}, 3.0))
    << "safety re-acquired the arbiter latch but never updated its own latch_confirmed_granted";
}

// ===========================================================================
// Y2 (review 2026-08-21): safety RESTARTS while the arbiter still holds the
// SAFETY latch (R5: it never self-expires). The fresh policy boots NORMAL
// knowing nothing of the latch -- ClearFault used to answer success=true
// ("no fault latched") while command_selected stayed cut forever. The B4
// re-parse must also RAISE latch_confirmed_granted from the arbiter's own
// diagnostics so ClearFault releases the orphan instead of lying.
// ===========================================================================
TEST_F(CutChainFixture, Item_Y2_RestartedSafetyAdoptsTheOrphanLatchSoClearFaultReallyReleasesIt)
{
  clearLogs();
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.fused_frame_id = "map";   // instantaneous FRAME_MISMATCH -> INHIBIT
  }
  ASSERT_TRUE(waitFor([this]() {return latchActiveTrue();}, 5.0))
    << "FAILED TO MEASURE: arbiter never latched before the restart";

  // Cause gone BEFORE the restart -- the fresh node must boot into a clean
  // world (policy NORMAL) with only the ORPHANED arbiter latch left behind.
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.fused_frame_id = "odom";
  }

  // Restart ONLY the safety node: stop the executor so the old node is
  // destroyed with no callback in flight, boot a fresh one (same params),
  // spin again. Arbiter + probe keep their in-process state -- exactly a
  // companion-computer safety process crash/restart.
  executor_->cancel();
  spin_thread_.join();
  executor_.reset();   // a cancelled executor keeps DEAD guard conditions -- never respin it
  safety_node_.reset();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    has_safety_diagnostics_ = false;     // never read the DEAD node's belief
    has_authority_diagnostics_ = false;  // only trust POST-restart arbiter truth
  }
  rclcpp::NodeOptions safety_options;
  safety_options.parameter_overrides(
    {rclcpp::Parameter("uav_id", uav_id_), rclcpp::Parameter("enforcement_enabled", true)});
  safety_node_ = std::make_shared<SafetySupervisorNode>(safety_options);
  executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions(), 8);
  executor_->add_node(authority_node_);
  executor_->add_node(safety_node_);
  executor_->add_node(probe_);
  spin_thread_ = std::thread([this]() {executor_->spin();});

  ASSERT_TRUE(waitFor([this]() {return clear_fault_client_->service_is_ready();}, 10.0))
    << "FAILED TO MEASURE: restarted safety never offered clear_fault";
  // 10 s valve, not 5: arbiter diagnostics are 1 Hz and a batched in-process
  // run (16 fixtures) once burned a 5 s valve on backlog alone (isolated
  // runs: 6/6 well under 1 s). Anti-hang valve only, R21.
  ASSERT_TRUE(waitFor([this]() {return latchActiveTrue();}, 10.0))
    << "FAILED TO MEASURE: R5 latch did not survive the restart (or arbiter diagnostics stopped)";

  // The fix itself: the fresh node adopts the arbiter's live truth.
  ASSERT_TRUE(waitFor([this]() {return latchConfirmedGrantedTrue();}, 5.0))
    << "Y2 regression: restarted safety never adopted the arbiter's live SAFETY latch";

  // ClearFault must now RELEASE the orphan -- and MISSION must actually flow
  // again on the SAME subscription, not just a polite success string.
  ClearFault::Response::SharedPtr response;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(5.0);
  while (std::chrono::steady_clock::now() < deadline) {
    response = callClearFault("");
    if (response && response->success) {
      break;
    }
    std::this_thread::sleep_for(200ms);
  }
  ASSERT_NE(response, nullptr) << "FAILED TO MEASURE: clear_fault stopped responding";
  ASSERT_TRUE(response->success) << "ClearFault never accepted on a clean world: " << response->message;
  EXPECT_TRUE(waitFor([this]() {return !latchActiveTrue();}, 3.0))
    << "Y2 regression: ClearFault answered success while the arbiter latch stayed cut";
  clearCommandLog();
  EXPECT_TRUE(waitFor([this]() {return countBySource(kSourceMission) > 0;}, 5.0))
    << "FAILED TO MEASURE: MISSION never resumed after the orphan latch release";
}

}  // namespace uav_safety
