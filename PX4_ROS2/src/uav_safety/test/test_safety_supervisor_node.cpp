// P8.3: the real node, driven through the 12 real input topics by a probe,
// with subscribers on safety/state, safety/violations, diagnostics/safety --
// same shape as uav_control_authority's test_control_authority_manager_node.
// This probe fabricates traffic on real topics (R20), hence the isolated
// domain (ENV ROS_DOMAIN_ID=97, see CMakeLists.txt).
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

/// Mutable, thread-safe "what the world currently looks like" -- the
/// background streamer republishes this at ~20 Hz on all 12 input topics so
/// age-based measurements stay fresh for the whole test, while a test can
/// mutate one field under lock to inject a specific fault.
struct WorldState
{
  std::mutex mutex;

  bool localization_valid = true;
  bool battery_nan = false;
  double command_x = 0.0;   // moves when a test wants BLIND_COMMAND's displacement > epsilon
  // R27-2 audit fix (P8.5): [5.0, NaN] when true -- pins that ONE corrupt
  // obstacle entry makes the whole min-distance unmeasurable, never silently
  // computed from the surviving finite entry (std::min swallowing NaN).
  bool obstacle_array_corrupt = false;
  // Coordinator addition (post-B2 backend fix): VehicleHealth.overall_level
  // == LEVEL_UNKNOWN when true (backend has no real failsafe_flags data).
  bool estimator_level_unknown = false;
  // B2 (review 2026-08-21): drop the VehicleHealth publish entirely when
  // true -- pins that a dead publisher's LAST GOOD reading goes stale
  // (CANNOT_MEASURE), never stays frozen-trustworthy forever.
  bool suppress_vehicle_health = false;
  // Y5 (review 2026-08-21): drop the LocalizationStatus publish entirely
  // when true -- last cached is_valid=true reading must go stale, not stay
  // "good for obstacle registration" forever.
  bool suppress_localization_status = false;
  // Y5 test fixture: a single obstacle inside min_obstacle_distance_copy_m,
  // independent of obstacle_array_corrupt above.
  bool obstacle_close = false;
  // R32 (N3): value published under the "jumps" key of "localization:
  // continuity"; the suppress flag kills the whole diag publish (dead
  // localization_health_node analogue).
  double localization_jumps = 0.0;
  bool suppress_localization_diag = false;
  // R32 (Y1): 0 = plain zero-count aggregate (baseline), 1 = arbitration +
  // RISING per-channel wrong-frame count on the held (MISSION) channel,
  // 2 = suppress the authority diagnostics publish entirely (dead arbiter
  // diagnostics analogue).
  int authority_diag_mode = 0;
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

class SafetySupervisorFixture : public ::testing::Test
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

    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("uav_id", uav_id_)});
    node_ = std::make_shared<SafetySupervisorNode>(options);

    probe_ = std::make_shared<rclcpp::Node>("safety_probe_" + uav_id_);

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
    command_selected_pub_ =
      probe_->create_publisher<ControlCommand>(prefix_ + "/control/command_selected", rclcpp::QoS(10));
    cmd_mission_pub_ = probe_->create_publisher<ControlCommand>(prefix_ + "/control/cmd_mission", rclcpp::QoS(10));
    control_authority_diag_pub_ =
      probe_->create_publisher<DiagnosticArray>(prefix_ + "/diagnostics/control_authority", rclcpp::QoS(10));
    camera_health_pub_ =
      probe_->create_publisher<DiagnosticStatus>(prefix_ + "/state/camera_health", rclcpp::QoS(10));
    trajectory_pub_ = probe_->create_publisher<Trajectory3D>(
      prefix_ + "/planning/trajectory", rclcpp::QoS(1).reliable().transient_local());

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
    diagnostics_sub_ = probe_->create_subscription<DiagnosticArray>(
      prefix_ + "/diagnostics/safety", rclcpp::QoS(10),
      [this](const DiagnosticArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_diagnostics_ = *msg;
        has_diagnostics_ = true;
      });

    clear_fault_client_ = probe_->create_client<ClearFault>(prefix_ + "/safety/clear_fault");

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 4);
    executor_->add_node(node_);
    executor_->add_node(probe_);
    spin_thread_ = std::thread([this]() {executor_->spin();});

    ASSERT_TRUE(
      waitFor(
        [this]() {
          return probe_->count_publishers(prefix_ + "/safety/state") > 0 &&
          command_selected_pub_->get_subscription_count() > 0;
        }, 10.0))
      << "FAILED TO MEASURE: discovery never completed";

    stream_stop_.store(false);
    stream_thread_ = std::thread([this]() {streamWorld();});

    // Give the streamer a couple of node ticks to fill the command window
    // and establish a healthy baseline before any test starts asserting.
    ASSERT_TRUE(waitFor([this]() {return !safetyStates().empty();}, 5.0))
      << "FAILED TO MEASURE: safety/state never arrived";
    std::this_thread::sleep_for(300ms);
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
    node_.reset();
    probe_.reset();
    executor_.reset();
  }

  // ------------------------------------------------------------ streaming

  void streamWorld()
  {
    while (!stream_stop_.load()) {
      publishOneRound();
      std::this_thread::sleep_for(20ms);   // ~50 Hz, comfortably above the node's 20 Hz tick
    }
  }

  void publishOneRound()
  {
    bool localization_valid;
    bool battery_nan;
    double command_x;
    bool obstacle_array_corrupt;
    bool estimator_level_unknown;
    bool suppress_vehicle_health;
    bool suppress_localization_status;
    bool obstacle_close;
    double localization_jumps;
    bool suppress_localization_diag;
    int authority_diag_mode;
    {
      std::lock_guard<std::mutex> lock(world_.mutex);
      localization_valid = world_.localization_valid;
      battery_nan = world_.battery_nan;
      command_x = world_.command_x;
      obstacle_array_corrupt = world_.obstacle_array_corrupt;
      estimator_level_unknown = world_.estimator_level_unknown;
      suppress_vehicle_health = world_.suppress_vehicle_health;
      suppress_localization_status = world_.suppress_localization_status;
      obstacle_close = world_.obstacle_close;
      localization_jumps = world_.localization_jumps;
      suppress_localization_diag = world_.suppress_localization_diag;
      authority_diag_mode = world_.authority_diag_mode;
    }
    const rclcpp::Time stamp = probe_->now();

    if (!suppress_localization_status) {
      LocalizationStatus localization_status;
      localization_status.header.stamp = stamp;
      localization_status.header.frame_id = "odom";
      localization_status.is_valid = localization_valid;
      localization_status.quality = LocalizationStatus::QUALITY_GOOD;
      localization_status.active_source = LocalizationStatus::SOURCE_FUSED;
      localization_status_pub_->publish(localization_status);
    }

    Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = "odom";
    odometry.pose.pose.position.z = 2.0;
    odometry_fused_pub_->publish(odometry);

    if (!suppress_localization_diag) {
      DiagnosticArray localization_diag;
      localization_diag.header.stamp = stamp;
      DiagnosticStatus continuity = keyValueStatus("localization: continuity", 0.0);
      KeyValue jumps_kv;
      jumps_kv.key = "jumps";   // the key the node actually parses (R32/N3)
      jumps_kv.value = std::to_string(localization_jumps);
      continuity.values.push_back(jumps_kv);
      localization_diag.status.push_back(continuity);
      localization_diag_pub_->publish(localization_diag);
    }

    ObstacleArray obstacles;
    obstacles.header.stamp = stamp;
    obstacles.header.frame_id = "odom";
    if (obstacle_array_corrupt) {
      uav_interfaces::msg::Obstacle far;
      far.shape = uav_interfaces::msg::Obstacle::SHAPE_SPHERE;
      far.distance = 5.0F;
      obstacles.obstacles.push_back(far);
      uav_interfaces::msg::Obstacle corrupt;
      corrupt.shape = uav_interfaces::msg::Obstacle::SHAPE_SPHERE;
      corrupt.distance = std::numeric_limits<float>::quiet_NaN();
      obstacles.obstacles.push_back(corrupt);
    } else if (obstacle_close) {
      // Y5 fixture: well inside min_obstacle_distance_copy_m (default 0.35).
      uav_interfaces::msg::Obstacle close;
      close.shape = uav_interfaces::msg::Obstacle::SHAPE_SPHERE;
      close.distance = 0.10F;
      obstacles.obstacles.push_back(close);
    }
    obstacle_map_pub_->publish(obstacles);   // empty unless corrupt/close -- "nothing detected"

    if (!suppress_vehicle_health) {
      VehicleHealth health;
      health.header.stamp = stamp;
      health.battery_remaining =
        battery_nan ? std::numeric_limits<float>::quiet_NaN() : 0.8F;
      health.imu_ok = true;
      health.gps_ok = true;
      health.barometer_ok = true;
      health.magnetometer_ok = true;
      health.overall_level =
        estimator_level_unknown ? VehicleHealth::LEVEL_UNKNOWN : VehicleHealth::LEVEL_OK;
      health_px4_pub_->publish(health);
    }

    VehicleState vehicle;
    vehicle.header.stamp = stamp;
    vehicle.armed = true;
    vehicle.flight_mode = VehicleState::FLIGHT_MODE_OFFBOARD;
    vehicle.failsafe_active = false;
    vehicle_state_pub_->publish(vehicle);

    OffboardStatus offboard;
    offboard.header.stamp = stamp;
    offboard.state = OffboardStatus::STATE_ACTIVE;
    offboard.setpoint_rate_hz = 20.0F;
    offboard.command_fresh = true;
    offboard.offboard_confirmed = true;
    offboard_status_pub_->publish(offboard);

    ControlCommand command;
    command.header.stamp = stamp;
    command.header.frame_id = "odom";
    command.control_mode = ControlCommand::MODE_POSITION;
    command.source = kSourceMission;
    command.position.x = command_x;
    command.position.z = 2.0;
    command.yaw = 0.0F;
    command_selected_pub_->publish(command);
    cmd_mission_pub_->publish(command);

    if (authority_diag_mode != 2) {
      DiagnosticArray authority_diag;
      authority_diag.header.stamp = stamp;
      if (authority_diag_mode == 1) {
        // R32/Y1: arbitration says MISSION holds authority, and MISSION's own
        // per-channel drop count rises every round -- the node's increasing
        // flag turns (and stays) true for as long as this stream lives.
        DiagnosticStatus arbitration;
        arbitration.name = "control_authority: arbitration";
        KeyValue kv;
        kv.key = "latch_active";
        kv.value = "false";
        arbitration.values.push_back(kv);
        kv.key = "latch_level";
        kv.value = "NONE";
        arbitration.values.push_back(kv);
        kv.key = "active_source";
        kv.value = "MISSION";
        arbitration.values.push_back(kv);
        authority_diag.status.push_back(arbitration);

        DiagnosticStatus drops;
        drops.name = "control_authority: dropped wrong_frame";
        KeyValue rising;
        rising.key = "dropped_wrong_frame_mission";
        rising.value = std::to_string(++wrong_frame_counter_);
        drops.values.push_back(rising);
        authority_diag.status.push_back(drops);
      } else {
        authority_diag.status.push_back(keyValueStatus("control_authority: dropped wrong_frame", 0.0));
      }
      control_authority_diag_pub_->publish(authority_diag);
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

  DiagnosticArray diagnostics()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_diagnostics_;
  }

  bool hasDiagnostics()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_diagnostics_;
  }

  void clearLogs()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    safety_states_.clear();
    violation_edges_.clear();
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

  static std::string valueOf(const DiagnosticStatus & status, const std::string & key)
  {
    for (const KeyValue & kv : status.values) {
      if (kv.key == key) {
        return kv.value;
      }
    }
    return "";
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

  std::string uav_id_;
  std::string prefix_;
  std::shared_ptr<SafetySupervisorNode> node_;
  std::shared_ptr<rclcpp::Node> probe_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;

  WorldState world_;
  std::atomic<bool> stream_stop_{false};
  std::thread stream_thread_;
  double wrong_frame_counter_ = 0.0;   // stream-thread only (R32/Y1)

  rclcpp::Publisher<LocalizationStatus>::SharedPtr localization_status_pub_;
  rclcpp::Publisher<Odometry>::SharedPtr odometry_fused_pub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr localization_diag_pub_;
  rclcpp::Publisher<ObstacleArray>::SharedPtr obstacle_map_pub_;
  rclcpp::Publisher<VehicleHealth>::SharedPtr health_px4_pub_;
  rclcpp::Publisher<VehicleState>::SharedPtr vehicle_state_pub_;
  rclcpp::Publisher<OffboardStatus>::SharedPtr offboard_status_pub_;
  rclcpp::Publisher<ControlCommand>::SharedPtr command_selected_pub_;
  rclcpp::Publisher<ControlCommand>::SharedPtr cmd_mission_pub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr control_authority_diag_pub_;
  rclcpp::Publisher<DiagnosticStatus>::SharedPtr camera_health_pub_;
  rclcpp::Publisher<Trajectory3D>::SharedPtr trajectory_pub_;

  rclcpp::Subscription<SafetyState>::SharedPtr safety_state_sub_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr violations_sub_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::Client<ClearFault>::SharedPtr clear_fault_client_;

  std::mutex mutex_;
  std::vector<SafetyState> safety_states_;
  std::vector<DiagnosticStatus> violation_edges_;
  DiagnosticArray last_diagnostics_;
  bool has_diagnostics_ = false;
};

}  // namespace

// ===========================================================================
// (a) Healthy signals throughout -> 0 violation, level OK.
// ===========================================================================
TEST_F(SafetySupervisorFixture, HealthySignalsProduceZeroViolations)
{
  clearLogs();
  ASSERT_TRUE(waitFor([this]() {return !safetyStates().empty();}, 5.0))
    << "FAILED TO MEASURE: safety/state never published";

  const SafetyState latest = safetyStates().back();
  EXPECT_EQ(latest.level, SafetyState::LEVEL_OK);
  EXPECT_TRUE(latest.violation_codes.empty()) << "unexpected violation on a fully healthy tick";
  EXPECT_TRUE(latest.localization_ok);
  EXPECT_TRUE(latest.offboard_link_ok);
  EXPECT_TRUE(latest.battery_ok);
  EXPECT_TRUE(latest.obstacle_clear);
  EXPECT_TRUE(latest.command_fresh);
  EXPECT_FALSE(latest.recovery_active) << "P8.3 must never actually recover -- enforcement_enabled is false";
}

// ===========================================================================
// (d) Structural: enforcement_enabled=false means NO publisher on cmd_safety
// exists at all -- proven by graph query, not a flag read.
// ===========================================================================
TEST_F(SafetySupervisorFixture, NoPublisherExistsOnCmdSafety)
{
  const auto publishers = probe_->get_publishers_info_by_topic(prefix_ + "/control/cmd_safety");
  EXPECT_EQ(publishers.size(), 0U)
    << "enforcement_enabled=false must mean safety_supervisor_node never creates this publisher";
}

// ===========================================================================
// (b) is_valid=false + setpoint moving -> BLIND_COMMAND, dry-run "would_
// inhibit" label, WITH a correct entering edge, followed by a clearing edge
// once conditions are stable-clear and ClearFault is called.
// ===========================================================================
TEST_F(SafetySupervisorFixture, BlindCommandEntersAsDryRunInhibitThenClearsViaClearFault)
{
  clearLogs();
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.localization_valid = false;
  }

  // Keep the setpoint moving (> frozen_setpoint_epsilon_copy_m across the 3-tick
  // window) for the whole grace window -- a background nudger, independent
  // of the ~50 Hz world streamer's own cadence. Ping-ponged, NOT a runaway
  // ramp: an unbounded ramp would also drag cmd_mission's absolute position
  // away from odometry_fused, which ClearFault's OWN handover-jump gate
  // (S:3.1) would then correctly keep rejecting forever -- that is the
  // gate working as designed, not something this test should trigger by
  // accident. Bounded within [0, 0.20] m keeps it well under
  // handover_jump_limit_m (0.80 default) for the whole test.
  std::atomic<bool> nudge_stop{false};
  std::thread nudger(
    [this, &nudge_stop]() {
      bool high = false;
      while (!nudge_stop.load()) {
        high = !high;
        {
          std::lock_guard<std::mutex> lock(world_.mutex);
          world_.command_x = high ? 0.20 : 0.0;
        }
        std::this_thread::sleep_for(20ms);
      }
    });

  const double default_blind_grace_sec = FailsafePolicyParams{}.blind_grace_sec;
  const bool entered = waitFor(
    [this]() {return findEdge(violationEdges(), "BLIND_COMMAND", "entered") != nullptr;},
    default_blind_grace_sec + 3.0);
  nudge_stop.store(true);
  nudger.join();

  ASSERT_TRUE(entered) << "FAILED TO MEASURE: BLIND_COMMAND never entered";
  // Snapshot into a NAMED local first: findEdge() returns a pointer into
  // whatever it was given, and a pointer into a temporary vector (e.g.
  // findEdge(violationEdges(), ...) used directly) dangles the instant the
  // statement ends -- this bit the first draft (garbage bytes, then a
  // cross-test SIGSEGV once heap reuse made it worse).
  const std::vector<DiagnosticStatus> edges_after_entry = violationEdges();
  const DiagnosticStatus * enter_edge = findEdge(edges_after_entry, "BLIND_COMMAND", "entered");
  ASSERT_NE(enter_edge, nullptr);
  EXPECT_EQ(valueOf(*enter_edge, "action"), "would_inhibit (dry-run, enforcement_enabled=false)");

  const bool seen_in_state = waitFor(
    [this]() {
      const std::vector<SafetyState> states = safetyStates();
      for (const SafetyState & state : states) {
        for (const std::string & code : state.violation_codes) {
          if (code == "BLIND_COMMAND") {
            return true;
          }
        }
      }
      return false;
    }, 2.0);
  EXPECT_TRUE(seen_in_state) << "FAILED TO MEASURE: BLIND_COMMAND never reached safety/state";

  // Stop moving the setpoint and restore localization; wait for clear_
  // stability_sec, then release through the real service (S:3.1).
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.localization_valid = true;
    world_.command_x = world_.command_x;    // frozen at its last value from here on
  }

  const double clear_stability_sec = FailsafePolicyParams{}.clear_stability_sec;
  ASSERT_TRUE(clear_fault_client_->wait_for_service(5s));
  ClearFault::Response::SharedPtr response;
  const bool cleared = waitFor(
    [this, &response]() {
      auto request = std::make_shared<ClearFault::Request>();
      request->fault_code = "BLIND_COMMAND";
      auto future = clear_fault_client_->async_send_request(request);
      if (future.wait_for(1s) != std::future_status::ready) {
        return false;
      }
      response = future.get();
      return response->success;
    }, clear_stability_sec + 3.0);
  ASSERT_TRUE(cleared) << "FAILED TO MEASURE: ClearFault never accepted once stably clear";

  const bool cleared_edge_seen = waitFor(
    [this]() {return findEdge(violationEdges(), "BLIND_COMMAND", "cleared") != nullptr;}, 2.0);
  EXPECT_TRUE(cleared_edge_seen) << "FAILED TO MEASURE: BLIND_COMMAND clearing edge never published";
}

// ===========================================================================
// (c) NaN battery -> "cannot measure", never read as healthy OR as violating.
// ===========================================================================
TEST_F(SafetySupervisorFixture, NanBatteryIsCannotMeasureNotAViolation)
{
  clearLogs();
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.battery_nan = true;
  }

  const bool cannot_measure_seen = waitFor(
    [this]() {return findEdge(violationEdges(), "BATTERY_CRITICAL", "cannot measure") != nullptr;}, 3.0);
  ASSERT_TRUE(cannot_measure_seen) << "FAILED TO MEASURE: NaN battery never reached safety/violations";
  // Named snapshot first -- see the comment on edges_after_entry above.
  const std::vector<DiagnosticStatus> edges_after_nan = violationEdges();
  const DiagnosticStatus * edge = findEdge(edges_after_nan, "BATTERY_CRITICAL", "cannot measure");
  ASSERT_NE(edge, nullptr);
  EXPECT_EQ(edge->level, DiagnosticStatus::WARN) << "cannot-measure must never read as OK or ERROR";

  // CANNOT_MEASURE needs no grace (instantaneous), but safety/state only
  // republishes on its own 5 Hz/on-change cadence -- clearLogs() emptied it,
  // so it can genuinely still be empty here; .back() on an empty vector is
  // UB (this crashed before this wait was added).
  ASSERT_TRUE(waitFor([this]() {return !safetyStates().empty();}, 3.0))
    << "FAILED TO MEASURE: safety/state never republished after clearLogs()";
  const SafetyState latest = safetyStates().back();
  EXPECT_TRUE(latest.battery_ok) << "NaN must never be silently read as a battery violation";
  for (const std::string & code : latest.violation_codes) {
    EXPECT_NE(code, "BATTERY_WARN");
    EXPECT_NE(code, "BATTERY_CRITICAL");
    EXPECT_NE(code, "BATTERY_PX4_NO_ACTION");
  }

  // Positive counter-check (R21): restore a real number and confirm the
  // pipe still reports a normal reading afterward -- the WARN above was
  // "cannot measure", not a broken subscriber.
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.battery_nan = false;
  }
  ASSERT_TRUE(
    waitFor(
      [this]() {
        const DiagnosticArray report = diagnostics();
        for (const DiagnosticStatus & status : report.status) {
          if (status.name == "safety: BATTERY_CRITICAL") {
            return valueOf(status, "measured") != "" &&
            valueOf(status, "measured").find("nan") == std::string::npos;
          }
        }
        return false;
      }, 3.0))
    << "FAILED TO MEASURE: diagnostics/safety never reported a real battery value after recovery";
}

// ===========================================================================
// (e) R27-2 audit fix (P8.5): a corrupt (NaN) obstacle mixed with a valid,
// finite one must make OBSTACLE_TOO_CLOSE unmeasurable -- never silently
// computed from the surviving finite entry (std::min(closest, NaN) would
// otherwise return closest, quietly discarding the corrupt reading).
// ===========================================================================
TEST_F(SafetySupervisorFixture, CorruptObstacleEntryIsCannotMeasureNotSilentlyIgnored)
{
  clearLogs();
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.obstacle_array_corrupt = true;
  }

  const bool cannot_measure_seen = waitFor(
    [this]() {return findEdge(violationEdges(), "OBSTACLE_TOO_CLOSE", "cannot measure") != nullptr;}, 3.0);
  ASSERT_TRUE(cannot_measure_seen)
    << "FAILED TO MEASURE: corrupt obstacle entry never reached safety/violations as cannot-measure";
  const std::vector<DiagnosticStatus> edges_after_corrupt = violationEdges();
  const DiagnosticStatus * edge = findEdge(edges_after_corrupt, "OBSTACLE_TOO_CLOSE", "cannot measure");
  ASSERT_NE(edge, nullptr);
  EXPECT_EQ(edge->level, DiagnosticStatus::WARN) << "cannot-measure must never read as OK or ERROR";

  ASSERT_TRUE(waitFor([this]() {return !safetyStates().empty();}, 3.0))
    << "FAILED TO MEASURE: safety/state never republished after clearLogs()";
  const SafetyState latest = safetyStates().back();
  for (const std::string & code : latest.violation_codes) {
    EXPECT_NE(code, "OBSTACLE_TOO_CLOSE")
      << "cannot-measure must never be silently upgraded into a violation either";
  }

  // Positive counter-check (R21): restore a clean, single finite entry and
  // confirm the pipe reports a real measured value afterward -- the WARN
  // above was "cannot measure", not a broken subscriber.
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.obstacle_array_corrupt = false;
  }
  ASSERT_TRUE(
    waitFor(
      [this]() {
        const DiagnosticArray report = diagnostics();
        for (const DiagnosticStatus & status : report.status) {
          if (status.name == "safety: OBSTACLE_TOO_CLOSE") {
            return valueOf(status, "measured") != "" &&
            valueOf(status, "measured").find("nan") == std::string::npos;
          }
        }
        return false;
      }, 3.0))
    << "FAILED TO MEASURE: diagnostics/safety never reported a real obstacle distance after recovery";
}

// ===========================================================================
// B2 (review 2026-08-21, real bug): a dead VehicleHealth publisher must go
// CANNOT_MEASURE once stale (vehicle_health_timeout_sec 0.3 s), never stay
// frozen on the LAST GOOD battery reading forever. Representative of all 4
// newly-arrival-tracked subscriptions (vehicle_state_/offboard_status_/
// camera_health_ share the exact same ageSec() pattern).
// ===========================================================================
TEST_F(SafetySupervisorFixture, DeadVehicleHealthPublisherGoesCannotMeasureAfterTimeout)
{
  clearLogs();
  // Confirm a real (non-NaN) battery reading first -- the stale state that
  // follows must be a genuine transition, not "never measured at all".
  ASSERT_TRUE(
    waitFor(
      [this]() {
        const DiagnosticArray report = diagnostics();
        for (const DiagnosticStatus & status : report.status) {
          if (status.name == "safety: BATTERY_CRITICAL") {
            return valueOf(status, "measured") != "" &&
            valueOf(status, "measured").find("nan") == std::string::npos;
          }
        }
        return false;
      }, 3.0))
    << "FAILED TO MEASURE: no real battery reading seen before suppression";

  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.suppress_vehicle_health = true;
  }
  const bool cannot_measure_seen = waitFor(
    [this]() {return findEdge(violationEdges(), "BATTERY_CRITICAL", "cannot measure") != nullptr;},
    FailsafePolicyParams{}.vehicle_health_timeout_sec + 2.0);
  ASSERT_TRUE(cannot_measure_seen)
    << "FAILED TO MEASURE: dead VehicleHealth publisher never went stale/CANNOT_MEASURE";

  // Positive counter-check (R21): resume publishing, confirm a real reading
  // comes back -- the CANNOT_MEASURE above was staleness, not a dead probe.
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.suppress_vehicle_health = false;
  }
  ASSERT_TRUE(
    waitFor(
      [this]() {
        const DiagnosticArray report = diagnostics();
        for (const DiagnosticStatus & status : report.status) {
          if (status.name == "safety: BATTERY_CRITICAL") {
            return valueOf(status, "measured") != "" &&
            valueOf(status, "measured").find("nan") == std::string::npos;
          }
        }
        return false;
      }, 3.0))
    << "FAILED TO MEASURE: battery reading never resumed after publisher came back";
}

// ===========================================================================
// Coordinator addition (post-B2 backend fix): VehicleHealth.overall_level ==
// LEVEL_UNKNOWN must make ESTIMATOR_INPUT_INVALID CANNOT_MEASURE, never read
// the fabricated-true imu_ok/gps_ok/barometer_ok/magnetometer_ok as healthy.
// ===========================================================================
TEST_F(SafetySupervisorFixture, EstimatorLevelUnknownIsCannotMeasureNotOk)
{
  clearLogs();
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.estimator_level_unknown = true;
  }
  const bool cannot_measure_seen = waitFor(
    [this]() {return findEdge(violationEdges(), "ESTIMATOR_INPUT_INVALID", "cannot measure") != nullptr;},
    3.0);
  ASSERT_TRUE(cannot_measure_seen)
    << "FAILED TO MEASURE: overall_level==LEVEL_UNKNOWN never reached safety/violations as cannot-measure";

  ASSERT_TRUE(waitFor([this]() {return !safetyStates().empty();}, 3.0))
    << "FAILED TO MEASURE: safety/state never republished after clearLogs()";
  const SafetyState latest = safetyStates().back();
  for (const std::string & code : latest.violation_codes) {
    EXPECT_NE(code, "ESTIMATOR_INPUT_INVALID")
      << "fabricated-true flags under LEVEL_UNKNOWN must never read as a healthy OK, "
         "but must also never be silently upgraded into a violation";
  }

  // Positive counter-check (R21): restore a real level, confirm the CURRENT
  // diagnostics entry reads OK (not stuck on the earlier cannot-measure
  // reading) -- checked via /diagnostics/safety's live snapshot, not the
  // accumulated edge log (which would still contain the earlier edge).
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.estimator_level_unknown = false;
  }
  ASSERT_TRUE(
    waitFor(
      [this]() {
        const DiagnosticArray report = diagnostics();
        for (const DiagnosticStatus & status : report.status) {
          if (status.name == "safety: ESTIMATOR_INPUT_INVALID") {
            return status.level == DiagnosticStatus::OK;
          }
        }
        return false;
      }, 3.0))
    << "FAILED TO MEASURE: ESTIMATOR_INPUT_INVALID never recovered after overall_level became real";
}

// ===========================================================================
// Y5 (review 2026-08-21): a stale-but-cached LocalizationStatus (last value
// is_valid=true) must not keep reading as "good for obstacle registration"
// forever -- age-gated exactly like LOCALIZATION_INVALID's own check
// (status_timeout_sec). OBSTACLE_TOO_CLOSE must never HOLD on a pose it can
// no longer trust for registration.
// ===========================================================================
TEST_F(SafetySupervisorFixture, ObstacleTooCloseNeverLatchesWhileLocalizationStatusIsStale)
{
  clearLogs();
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.suppress_localization_status = true;
  }

  // Let the last (is_valid=true) LocalizationStatus reading actually go
  // stale BEFORE introducing a close obstacle -- otherwise
  // localization_good_for_obstacles would still read true off the still-
  // fresh last arrival, and OBSTACLE_TOO_CLOSE would legitimately latch
  // before the age-gate this test targets ever engages (a real ordering
  // hazard, not a hypothetical one: this bit the first draft).
  const double status_timeout_sec = FailsafePolicyParams{}.status_timeout_sec;
  std::this_thread::sleep_for(std::chrono::duration<double>(status_timeout_sec + 0.3));

  clearLogs();
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.obstacle_close = true;
  }

  const bool not_trusted_seen = waitFor(
    [this]() {
      const DiagnosticArray report = diagnostics();
      for (const DiagnosticStatus & status : report.status) {
        if (status.name == "safety: OBSTACLE_TOO_CLOSE") {
          return status.message.find("localization not trusted for registration") != std::string::npos;
        }
      }
      return false;
    }, 3.0);
  ASSERT_TRUE(not_trusted_seen)
    << "FAILED TO MEASURE: stale LocalizationStatus never made OBSTACLE_TOO_CLOSE distrust it";

  // Hold well past obstacle_grace_sec while localization stays stale --
  // proves it never latches, not just that it has not latched YET.
  const double obstacle_grace_sec = FailsafePolicyParams{}.obstacle_grace_sec;
  std::this_thread::sleep_for(std::chrono::duration<double>(obstacle_grace_sec + 0.5));

  ASSERT_TRUE(waitFor([this]() {return !safetyStates().empty();}, 3.0))
    << "FAILED TO MEASURE: safety/state never republished after clearLogs()";
  for (const SafetyState & state : safetyStates()) {
    for (const std::string & code : state.violation_codes) {
      EXPECT_NE(code, "OBSTACLE_TOO_CLOSE")
        << "must never HOLD on an obstacle registered against untrusted localization";
    }
  }

  // Positive counter-check (R21): restore LocalizationStatus publishing --
  // the obstacle stays close -- and confirm OBSTACLE_TOO_CLOSE now DOES
  // enter, proving the earlier silence was the age-gate, not a broken probe.
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.suppress_localization_status = false;
  }
  const bool entered = waitFor(
    [this]() {return findEdge(violationEdges(), "OBSTACLE_TOO_CLOSE", "entered") != nullptr;},
    obstacle_grace_sec + 3.0);
  EXPECT_TRUE(entered)
    << "FAILED TO MEASURE: OBSTACLE_TOO_CLOSE never entered once localization was fresh again";
}

// ===========================================================================
// R32/Y1 (review 2026-08-21): the wrong-frame "increasing" flag is a sample-
// and-hold from the arbiter's diagnostics -- a stream that dies right after
// reporting "increasing" must expire the flag (2 source cycles), never hold
// it true forever (false FRAME_MISMATCH latch / an unclearable fault).
// ===========================================================================
TEST_F(SafetySupervisorFixture, StaleAuthorityDiagnosticsExpireTheWrongFrameIncreasingFlag)
{
  clearLogs();
  const auto frameMismatchMeasured = [this]() -> double {
      const DiagnosticArray report = diagnostics();
      for (const DiagnosticStatus & status : report.status) {
        if (status.name == "safety: FRAME_MISMATCH") {
          try {
            return std::stod(valueOf(status, "measured"));
          } catch (const std::exception &) {
            return -1.0;
          }
        }
      }
      return -1.0;
    };

  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.authority_diag_mode = 1;   // rising per-channel count on MISSION
  }
  ASSERT_TRUE(waitFor([&]() {return frameMismatchMeasured() == 1.0;}, 3.0))
    << "FAILED TO MEASURE: rising per-channel drops never raised the increasing flag";

  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.authority_diag_mode = 2;   // arbiter diagnostics stream dies
  }
  const double expiry_sec = 2.0 * FailsafePolicyParams{}.arbiter_diagnostics_period_copy_sec;
  ASSERT_TRUE(waitFor([&]() {return frameMismatchMeasured() == 0.0;}, expiry_sec + 2.0))
    << "R32/Y1 regression: the increasing flag stayed stuck true after the "
       "arbiter diagnostics stream died";

  // Positive counter-check (R21): resume the rising stream -- the flag comes
  // back, proving the earlier 0 was the expiry, not a dead probe/path.
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.authority_diag_mode = 1;
  }
  ASSERT_TRUE(waitFor([&]() {return frameMismatchMeasured() == 1.0;}, 3.0))
    << "FAILED TO MEASURE: the increasing flag never resumed after the stream came back";
}

// ===========================================================================
// R32/N3: the jumps count is a sample-and-hold from /diagnostics/
// localization -- a stream that dies right after reporting jumps>0 must
// expire to CANNOT_MEASURE (2 source cycles), never stay a trusted,
// still-violating reading forever.
// ===========================================================================
TEST_F(SafetySupervisorFixture, StaleLocalizationDiagnosticsExpireTheJumpCountToCannotMeasure)
{
  clearLogs();
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.localization_jumps = 3.0;
  }
  ASSERT_TRUE(
    waitFor([this]() {return findEdge(violationEdges(), "LOCALIZATION_JUMP", "entered") != nullptr;}, 3.0))
    << "FAILED TO MEASURE: jumps>0 never entered LOCALIZATION_JUMP";

  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.suppress_localization_diag = true;
  }
  const double expiry_sec = 2.0 * FailsafePolicyParams{}.localization_diag_period_copy_sec;
  ASSERT_TRUE(
    waitFor(
      [this]() {
        return findEdge(violationEdges(), "LOCALIZATION_JUMP", "cannot measure") != nullptr;
      }, expiry_sec + 2.0))
    << "R32/N3 regression: a dead localization diagnostics stream left jumps>0 "
       "frozen as a trusted, still-violating reading";

  // Positive counter-check (R21): resume the stream (jumps still 3) -- the
  // violation re-enters, proving the earlier transition was the expiry.
  clearLogs();
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.suppress_localization_diag = false;
  }
  ASSERT_TRUE(
    waitFor([this]() {return findEdge(violationEdges(), "LOCALIZATION_JUMP", "entered") != nullptr;}, 3.0))
    << "FAILED TO MEASURE: LOCALIZATION_JUMP never re-entered after the stream resumed";
}

}  // namespace uav_safety
