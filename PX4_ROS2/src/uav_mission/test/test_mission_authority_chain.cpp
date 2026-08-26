// P9.4b (plan P9-mission.md S:5): the REAL mission_executor_node driving
// the REAL control_authority_manager_node, in ONE process, through the
// ACTUAL /control/cmd_mission -> arbiter -> /control/authority pipe (R27-4:
// through the mach it protects, not a stand-in -- same shape as
// uav_safety's test_safety_cut_chain.cpp). A probe fabricates cmd_mission
// (standing in for the real navigator, which is not run here) plus the raw
// world topics mission_executor_node reads directly (health_px4/
// localization_status/odometry_fused), and seizes the SAFETY latch via the
// REAL /control/set_authority service (standing in for uav_safety, which is
// also not run here -- P8's own test suites already pin that node's half of
// this chain). Fake GotoPose/Takeoff/Land servers stand in for the rest of
// the navigator (R20: fabricates rclcpp_action traffic on real topic names
// -- isolated domain).
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <uav_interfaces/action/execute_mission.hpp>
#include <uav_interfaces/action/follow_path.hpp>
#include <uav_interfaces/action/goto_pose.hpp>
#include <uav_interfaces/action/hold_position.hpp>
#include <uav_interfaces/action/land.hpp>
#include <uav_interfaces/action/recover.hpp>
#include <uav_interfaces/action/takeoff.hpp>
#include <uav_interfaces/action/track_target.hpp>
#include <uav_interfaces/msg/control_authority.hpp>
#include <uav_interfaces/msg/control_command.hpp>
#include <uav_interfaces/msg/localization_status.hpp>
#include <uav_interfaces/msg/mission_status.hpp>
#include <uav_interfaces/msg/vehicle_health.hpp>
#include <uav_interfaces/srv/clear_fault.hpp>
#include <uav_interfaces/srv/resume_mission.hpp>
#include <uav_interfaces/srv/set_control_authority.hpp>

#include "uav_control_authority/control_authority_manager_node.hpp"
#include "uav_mission/mission_executor_node.hpp"

using namespace std::chrono_literals;

namespace uav_mission
{
namespace
{

using ExecuteMission = uav_interfaces::action::ExecuteMission;
using GotoPose = uav_interfaces::action::GotoPose;
using Takeoff = uav_interfaces::action::Takeoff;
using Land = uav_interfaces::action::Land;
using HoldPosition = uav_interfaces::action::HoldPosition;
using FollowPath = uav_interfaces::action::FollowPath;
using TrackTarget = uav_interfaces::action::TrackTarget;
using Recover = uav_interfaces::action::Recover;
using ControlAuthority = uav_interfaces::msg::ControlAuthority;
using ControlCommand = uav_interfaces::msg::ControlCommand;
using LocalizationStatus = uav_interfaces::msg::LocalizationStatus;
using VehicleHealth = uav_interfaces::msg::VehicleHealth;
using MissionStatusMsg = uav_interfaces::msg::MissionStatus;

/// Minimal fake-navigator plumbing, trimmed down from
/// test_mission_executor_node.cpp (duplicated rather than shared -- same
/// precedent as uav_safety's test_safety_supervisor_node.cpp vs.
/// test_safety_cut_chain.cpp each keeping their own small WorldState/probe):
/// this file only needs enough traffic to prove the AUTHORITY chain, not to
/// re-pin the broker's own FSM (that is P9.4's job).
struct FakeNavigatorState
{
  std::mutex mutex;
  std::vector<std::pair<std::string, std::string>> events;   // (action, kind)

  bool hasEvent(const std::string & action, const std::string & kind)
  {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & e : events) {
      if (e.first == action && e.second == kind) {
        return true;
      }
    }
    return false;
  }

  void log(const std::string & action, const std::string & kind)
  {
    std::lock_guard<std::mutex> lock(mutex);
    events.emplace_back(action, kind);
  }
};

template<typename ActionT>
class FakeSimpleServer
{
public:
  FakeSimpleServer(
    const rclcpp::Node::SharedPtr & node, const std::string & name, FakeNavigatorState & state,
    std::string label, std::chrono::milliseconds delay = 20ms)
  : state_(state), label_(std::move(label)), delay_(delay)
  {
    server_ = rclcpp_action::create_server<ActionT>(
      node, name,
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename ActionT::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle) {
        state_.log(label_, "accepted");
        std::thread([this, goal_handle]() {
          std::this_thread::sleep_for(delay_);
          auto result = std::make_shared<typename ActionT::Result>();
          if (goal_handle->is_canceling()) {
            result->success = false;
            result->result_code = kResultCanceled;
            result->message = "canceled";
            goal_handle->canceled(result);
            state_.log(label_, "canceled");
            return;
          }
          result->success = true;
          result->result_code = kResultSucceeded;
          goal_handle->succeed(result);
          state_.log(label_, "succeeded");
        }).detach();
      });
  }

private:
  FakeNavigatorState & state_;
  std::string label_;
  std::chrono::milliseconds delay_;
  typename rclcpp_action::Server<ActionT>::SharedPtr server_;
};

/// GotoPose only: hold-controlled, same technique as
/// test_mission_executor_node.cpp -- lets the test keep a body goal
/// deliberately in flight while it seizes the SAFETY latch.
class FakeGotoPoseServer
{
public:
  FakeGotoPoseServer(const rclcpp::Node::SharedPtr & node, const std::string & name, FakeNavigatorState & state)
  : state_(state)
  {
    server_ = rclcpp_action::create_server<GotoPose>(
      node, name,
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const GotoPose::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<GotoPose>>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<GotoPose>> goal_handle) {
        state_.log("GotoPose", "accepted");
        std::thread([this, goal_handle]() {
          while (rclcpp::ok() && hold_.load() && !goal_handle->is_canceling()) {
            std::this_thread::sleep_for(10ms);
          }
          if (!rclcpp::ok()) {
            return;
          }
          auto result = std::make_shared<GotoPose::Result>();
          if (goal_handle->is_canceling()) {
            result->success = false;
            result->result_code = kResultCanceled;
            result->message = "canceled";
            goal_handle->canceled(result);
            state_.log("GotoPose", "canceled");
            return;
          }
          result->success = true;
          result->result_code = kResultSucceeded;
          result->final_distance_error = 0.05f;
          goal_handle->succeed(result);
          state_.log("GotoPose", "succeeded");
        }).detach();
      });
  }

  void setHold(bool hold) {hold_.store(hold);}

private:
  FakeNavigatorState & state_;
  std::atomic<bool> hold_{false};
  rclcpp_action::Server<GotoPose>::SharedPtr server_;
};

class AuthorityChainFixture : public ::testing::Test
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

    rclcpp::NodeOptions mission_options;
    mission_options.parameter_overrides(
      {rclcpp::Parameter("uav_id", uav_id_),
        rclcpp::Parameter("missions_dir", std::string(UAV_MISSION_CONFIG_DIR) + "/missions"),
        rclcpp::Parameter("tick_hz", 20.0)});
    mission_node_ = std::make_shared<MissionExecutorNode>(mission_options);

    fake_navigator_node_ = std::make_shared<rclcpp::Node>("fake_navigator_" + uav_id_);
    goto_pose_server_ =
      std::make_unique<FakeGotoPoseServer>(fake_navigator_node_, prefix_ + "/planning/goto_pose", fake_state_);
    takeoff_server_ = std::make_unique<FakeSimpleServer<Takeoff>>(
      fake_navigator_node_, prefix_ + "/planning/takeoff", fake_state_, "Takeoff");
    land_server_ = std::make_unique<FakeSimpleServer<Land>>(
      fake_navigator_node_, prefix_ + "/planning/land", fake_state_, "Land");
    hold_stub_ = std::make_unique<FakeSimpleServer<HoldPosition>>(
      fake_navigator_node_, prefix_ + "/planning/hold_position", fake_state_, "HoldPosition");
    follow_stub_ = std::make_unique<FakeSimpleServer<FollowPath>>(
      fake_navigator_node_, prefix_ + "/planning/follow_path", fake_state_, "FollowPath");
    track_stub_ = std::make_unique<FakeSimpleServer<TrackTarget>>(
      fake_navigator_node_, prefix_ + "/planning/track_target", fake_state_, "TrackTarget");
    recover_stub_ = std::make_unique<FakeSimpleServer<Recover>>(
      fake_navigator_node_, prefix_ + "/planning/recover", fake_state_, "Recover");

    probe_ = std::make_shared<rclcpp::Node>("authority_chain_probe_" + uav_id_);

    // Stands in for the navigator's own OWN command stream -- without this,
    // the real arbiter never has a live MISSION channel to select in the
    // first place (mission_executor_node itself never publishes
    // ControlCommand, per the 4 CẤM).
    cmd_mission_pub_ = probe_->create_publisher<ControlCommand>(prefix_ + "/control/cmd_mission", rclcpp::QoS(10));
    health_pub_ = probe_->create_publisher<VehicleHealth>(prefix_ + "/state/health_px4", rclcpp::QoS(10));
    localization_pub_ =
      probe_->create_publisher<LocalizationStatus>(prefix_ + "/state/localization_status", rclcpp::QoS(10));
    odometry_pub_ =
      probe_->create_publisher<nav_msgs::msg::Odometry>(prefix_ + "/state/odometry_fused", rclcpp::QoS(10));

    authority_sub_ = probe_->create_subscription<ControlAuthority>(
      prefix_ + "/control/authority", rclcpp::QoS(1).reliable().transient_local(),
      [this](ControlAuthority::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        last_authority_ = *msg;
        has_authority_ = true;
      });
    status_sub_ = probe_->create_subscription<MissionStatusMsg>(
      prefix_ + "/mission/status", rclcpp::QoS(1).reliable().transient_local(),
      [this](MissionStatusMsg::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        last_status_ = *msg;
        has_status_ = true;
      });

    set_authority_client_ =
      probe_->create_client<uav_interfaces::srv::SetControlAuthority>(prefix_ + "/control/set_authority");
    clear_safety_latch_client_ =
      probe_->create_client<uav_interfaces::srv::ClearFault>(prefix_ + "/control/clear_safety_latch");
    resume_client_ = probe_->create_client<uav_interfaces::srv::ResumeMission>(prefix_ + "/mission/resume");
    execute_client_ = rclcpp_action::create_client<ExecuteMission>(probe_, prefix_ + "/mission/execute_mission");

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 10);
    executor_->add_node(authority_node_);
    executor_->add_node(mission_node_);
    executor_->add_node(fake_navigator_node_);
    executor_->add_node(probe_);
    spin_thread_ = std::thread([this]() {executor_->spin();});

    ASSERT_TRUE(waitFor([this]() {return execute_client_->action_server_is_ready();}, 10.0))
      << "FAILED TO MEASURE: discovery never completed (mission node's ExecuteMission server missing)";
    ASSERT_TRUE(waitFor([this]() {return set_authority_client_->service_is_ready();}, 10.0))
      << "FAILED TO MEASURE: discovery never completed (arbiter's set_authority service missing)";

    stream_stop_.store(false);
    stream_thread_ = std::thread([this]() {streamWorld();});

    ASSERT_TRUE(waitFor([this]() {return authorityKnown();}, 5.0))
      << "FAILED TO MEASURE: /control/authority never published";
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
    mission_node_.reset();
    fake_navigator_node_.reset();
    probe_.reset();
    executor_.reset();
  }

  void streamWorld()
  {
    while (!stream_stop_.load()) {
      // G-M1 review round 1 (2026-08-23): toggle-able -- simulates a real
      // PX4-native LAND taking the leg (nobody streams cmd_mission once
      // LANDING begins, the navigator hands off to PX4's own LAND mode).
      if (stream_cmd_mission_.load()) {
        ControlCommand cmd;
        cmd.header.stamp = probe_->now();
        cmd.header.frame_id = "odom";
        cmd.uav_id = uav_id_;
        cmd.control_mode = ControlCommand::MODE_POSITION;
        cmd.position.x = 0.0;
        cmd.position.y = 0.0;
        cmd.position.z = 1.5;
        cmd.yaw = 0.0f;
        cmd.source = kSourceMission;
        cmd_mission_pub_->publish(cmd);
      }

      VehicleHealth health;
      health.header.stamp = probe_->now();
      health.uav_id = uav_id_;
      health.battery_remaining = static_cast<float>(battery_remaining_.load());
      health.overall_level = VehicleHealth::LEVEL_OK;
      health_pub_->publish(health);

      LocalizationStatus loc;
      loc.header.stamp = probe_->now();
      loc.uav_id = uav_id_;
      loc.is_valid = true;
      loc.active_source = LocalizationStatus::SOURCE_FUSED;
      localization_pub_->publish(loc);

      nav_msgs::msg::Odometry odom;
      odom.header.stamp = probe_->now();
      odom.header.frame_id = "odom";
      odom.pose.pose.orientation.w = 1.0;
      odometry_pub_->publish(odom);

      std::this_thread::sleep_for(20ms);
    }
  }

  static bool waitFor(const std::function<bool()> & happened, double valve_seconds)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(static_cast<int64_t>(valve_seconds * 1000));
    while (std::chrono::steady_clock::now() < deadline) {
      if (happened()) {
        return true;
      }
      std::this_thread::sleep_for(20ms);
    }
    return happened();
  }

  bool authorityKnown()
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return has_authority_;
  }

  uint8_t latestAuthoritySource()
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return has_authority_ ? last_authority_.active_source : kSourceNone;
  }

  MissionStatusMsg latestStatus()
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return last_status_;
  }

  void setStreamCmdMission(bool enable) {stream_cmd_mission_.store(enable);}

  bool sendExecuteMissionGoal(const std::string & mission_id, const std::string & mission_params = "")
  {
    ExecuteMission::Goal goal;
    goal.mission_id = mission_id;
    goal.mission_params = mission_params;
    goal.loop = false;
    auto send_future = execute_client_->async_send_goal(goal);
    if (send_future.wait_for(5s) != std::future_status::ready) {
      return false;
    }
    goal_handle_ = send_future.get();
    return goal_handle_ != nullptr;
  }

  bool seizeSafetyLatch()
  {
    auto request = std::make_shared<uav_interfaces::srv::SetControlAuthority::Request>();
    request->requested_source = kSourceSafety;
    request->reason = "P9.4b probe: simulate uav_safety seizing authority";
    if (!set_authority_client_->wait_for_service(3s)) {
      return false;
    }
    auto future = set_authority_client_->async_send_request(request);
    if (future.wait_for(3s) != std::future_status::ready) {
      return false;
    }
    return future.get()->success;
  }

  bool clearSafetyLatch()
  {
    auto request = std::make_shared<uav_interfaces::srv::ClearFault::Request>();
    request->fault_code = "";
    if (!clear_safety_latch_client_->wait_for_service(3s)) {
      return false;
    }
    auto future = clear_safety_latch_client_->async_send_request(request);
    if (future.wait_for(3s) != std::future_status::ready) {
      return false;
    }
    return future.get()->success;
  }

  bool callResumeMission(const std::string & mission_id)
  {
    auto request = std::make_shared<uav_interfaces::srv::ResumeMission::Request>();
    request->mission_id = mission_id;
    if (!resume_client_->wait_for_service(3s)) {
      return false;
    }
    auto future = resume_client_->async_send_request(request);
    if (future.wait_for(3s) != std::future_status::ready) {
      return false;
    }
    return future.get()->success;
  }

  // ------------------------------------------------------------- members

  std::string uav_id_;
  std::string prefix_;

  std::shared_ptr<uav_control_authority::ControlAuthorityManagerNode> authority_node_;
  std::shared_ptr<MissionExecutorNode> mission_node_;
  rclcpp::Node::SharedPtr fake_navigator_node_;
  rclcpp::Node::SharedPtr probe_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;

  FakeNavigatorState fake_state_;
  std::unique_ptr<FakeGotoPoseServer> goto_pose_server_;
  std::unique_ptr<FakeSimpleServer<Takeoff>> takeoff_server_;
  std::unique_ptr<FakeSimpleServer<Land>> land_server_;
  std::unique_ptr<FakeSimpleServer<HoldPosition>> hold_stub_;
  std::unique_ptr<FakeSimpleServer<FollowPath>> follow_stub_;
  std::unique_ptr<FakeSimpleServer<TrackTarget>> track_stub_;
  std::unique_ptr<FakeSimpleServer<Recover>> recover_stub_;

  std::atomic<bool> stream_stop_{true};
  std::atomic<double> battery_remaining_{0.9};
  std::atomic<bool> stream_cmd_mission_{true};
  std::thread stream_thread_;
  rclcpp::Publisher<ControlCommand>::SharedPtr cmd_mission_pub_;
  rclcpp::Publisher<VehicleHealth>::SharedPtr health_pub_;
  rclcpp::Publisher<LocalizationStatus>::SharedPtr localization_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  std::mutex status_mutex_;
  ControlAuthority last_authority_;
  bool has_authority_ = false;
  MissionStatusMsg last_status_;
  bool has_status_ = false;
  rclcpp::Subscription<ControlAuthority>::SharedPtr authority_sub_;
  rclcpp::Subscription<MissionStatusMsg>::SharedPtr status_sub_;

  rclcpp::Client<uav_interfaces::srv::SetControlAuthority>::SharedPtr set_authority_client_;
  rclcpp::Client<uav_interfaces::srv::ClearFault>::SharedPtr clear_safety_latch_client_;
  rclcpp::Client<uav_interfaces::srv::ResumeMission>::SharedPtr resume_client_;
  rclcpp_action::Client<ExecuteMission>::SharedPtr execute_client_;
  rclcpp_action::ClientGoalHandle<ExecuteMission>::SharedPtr goal_handle_;
};

TEST_F(AuthorityChainFixture, SafetySeizeCancelsGoalAndPausesThenClearAndResumeContinue)
{
  ASSERT_TRUE(sendExecuteMissionGoal("indoor_patrol"));
  goto_pose_server_->setHold(true);
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("GotoPose", "accepted");}, 15.0))
    << "FAILED TO MEASURE: the mission never sent a body GotoPose goal";

  // R27-3 positive witness: mission is genuinely ACTIVE through the REAL
  // arbiter reporting MISSION, before ever seizing the latch.
  ASSERT_TRUE(waitFor([this]() {return latestAuthoritySource() == kSourceMission;}, 5.0))
    << "FAILED TO MEASURE: the real arbiter never selected MISSION before the latch";
  EXPECT_EQ(latestStatus().state, MissionStatusMsg::STATE_RUNNING);

  ASSERT_TRUE(seizeSafetyLatch()) << "the real SetControlAuthority(SAFETY) call was refused/unreachable";
  // NOT kSourceSafety: the arbiter's active_source only ever reports a
  // channel that has actually published a VALID message (authority_arbiter.
  // cpp's computeWinner() requires isLive()); the SAFETY latch itself is
  // structurally exempt from THAT liveness requirement (contract S2.16f),
  // but nobody in this test ever publishes on cmd_safety (uav_safety is not
  // run here -- P8's own suite pins that half). So the real, observable
  // effect of the latch is active_source dropping to SOURCE_NONE (MISSION is
  // now below the floor) -- exactly the "!= MISSION" signal mission_policy's
  // priority 2 actually keys off, which is what this waits for.
  ASSERT_TRUE(waitFor([this]() {return latestAuthoritySource() != kSourceMission;}, 5.0))
    << "FAILED TO MEASURE: the real arbiter never took MISSION out of active_source after the latch";

  ASSERT_TRUE(waitFor([this]() {return latestStatus().state == MissionStatusMsg::STATE_PAUSED;}, 6.0))
    << "the mission never canceled its goal and paused once SAFETY held authority THROUGH THE REAL ARBITER";
  EXPECT_TRUE(fake_state_.hasEvent("GotoPose", "canceled"))
    << "the safety-seize PAUSE did not actually cancel the in-flight navigator goal";
  EXPECT_EQ(latestStatus().last_result_code, kResultAbortedNoAuthority);

  ASSERT_TRUE(clearSafetyLatch()) << "the real clear_safety_latch call was refused/unreachable";
  // cmd_mission is still streaming continuously (probe), so once the floor
  // drops the arbiter should pick MISSION back up as the live winner.
  ASSERT_TRUE(waitFor([this]() {return latestAuthoritySource() == kSourceMission;}, 5.0))
    << "FAILED TO MEASURE: the real arbiter never released the SAFETY latch";

  std::this_thread::sleep_for(1s);
  EXPECT_EQ(latestStatus().state, MissionStatusMsg::STATE_PAUSED)
    << "clearing the SAFETY latch alone must NOT resume the mission -- only ResumeMission may (plan P9 policy 3)";

  goto_pose_server_->setHold(false);
  ASSERT_TRUE(callResumeMission(""));
  ASSERT_TRUE(waitFor([this]() {return latestStatus().state == MissionStatusMsg::STATE_RUNNING;}, 5.0))
    << "ResumeMission did not bring the mission back to RUNNING";
  ASSERT_TRUE(
    waitFor(
      [this]() {
        const auto s = latestStatus();
        return s.current_step_index >= 1 || s.state == MissionStatusMsg::STATE_COMPLETED;
      }, 20.0))
    << "FAILED TO MEASURE: mission made no further progress after resume";
}

// ---------------------------------------------------------------- (B1-fix)
// G-M1 review round 1 regression (2026-08-23): a REAL gate run showed
// indoor_patrol spuriously PAUSE (EVENT_PAUSED ABORTED_NO_AUTHORITY)
// several seconds into Finish's GotoPose(home) leg, with the REAL
// control_authority_manager_node reporting 0 log lines and
// safety_supervisor_node reporting 0 violations around that time -- nobody
// actually seized authority. test_mission_executor_node.cpp's OWN B1 test
// (AuthoritySeizeDuringFinishCancelsPausesAndResumesTheSameLeg) never
// caught this because its fixture publishes /control/authority DIRECTLY at
// 50 Hz (a raw fake probe), bypassing the REAL arbiter's 2 Hz HEARTBEAT
// throttling entirely -- authority_arrival_ never got a chance to look
// anything but maximally fresh. THIS fixture instead drives the REAL
// control_authority_manager_node (2 Hz heartbeat, matching
// authority_heartbeat_copy_sec's actual source), the only way to exercise
// the real timing this bug lives in.
TEST_F(AuthorityChainFixture, BodyToFinishTransitionStaysRunningWithHealthyRealAuthority)
{
  // Reach Finish deterministically and FAST via battery END_EARLY (same
  // technique test_mission_executor_node.cpp's B1 test uses) rather than
  // waiting out several real indoor_patrol waypoints -- authority stays
  // kSourceMission (streamed continuously via cmd_mission) throughout,
  // exactly the "healthy" case this test is pinning.
  ASSERT_TRUE(sendExecuteMissionGoal("indoor_patrol"));
  goto_pose_server_->setHold(true);
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("GotoPose", "accepted");}, 15.0))
    << "FAILED TO MEASURE: the mission never sent a body GotoPose goal";
  ASSERT_TRUE(waitFor([this]() {return latestAuthoritySource() == kSourceMission;}, 5.0))
    << "FAILED TO MEASURE: the real arbiter never selected MISSION";

  battery_remaining_.store(0.2);   // below battery_warn_threshold (0.35)
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("GotoPose", "canceled");}, 8.0))
    << "FAILED TO MEASURE: battery END_EARLY never canceled the body leg";
  const auto countGotoPoseAccepted = [this]() {
    std::lock_guard<std::mutex> lock(fake_state_.mutex);
    int count = 0;
    for (const auto & e : fake_state_.events) {
      if (e.first == "GotoPose" && e.second == "accepted") {
        ++count;
      }
    }
    return count;
  };
  ASSERT_TRUE(waitFor([&]() {return countGotoPoseAccepted() >= 2;}, 8.0))
    << "FAILED TO MEASURE: Finish's own GotoPose(home) leg never got dispatched "
       "(2nd GotoPose accept)";

  // Finish's GotoPose(home) leg is now held (RUNNING) with battery STILL
  // below WARN (irrelevant -- Finish already decided; the livelock-
  // suppression block should keep battery inert here) and authority
  // continuously healthy through the REAL arbiter's ACTUAL 2 Hz heartbeat
  // cadence, for a real several seconds -- long enough for any heartbeat-
  // timing edge case to show up, matching the multi-second real gap the
  // G-M1 gate log showed between waypoint completion and the spurious
  // PAUSE.
  const auto deadline = std::chrono::steady_clock::now() + 10s;
  while (std::chrono::steady_clock::now() < deadline) {
    ASSERT_NE(latestStatus().state, MissionStatusMsg::STATE_PAUSED)
      << "B1 regression: mission spuriously PAUSED during Finish with authority continuously "
      << "healthy through the REAL arbiter -- last_reason=" << latestStatus().last_reason;
    std::this_thread::sleep_for(50ms);
  }

  goto_pose_server_->setHold(false);   // let Finish/Land finally run free
  ASSERT_TRUE(
    waitFor(
      [this]() {
        const auto s = latestStatus();
        return s.state == MissionStatusMsg::STATE_ABORTED || s.state == MissionStatusMsg::STATE_COMPLETED;
      }, 15.0))
    << "FAILED TO MEASURE: mission never reached a terminal state";
  EXPECT_EQ(latestStatus().state, MissionStatusMsg::STATE_ABORTED);
  EXPECT_EQ(latestStatus().last_result_code, kResultAbortedLowBattery)
    << "got last_reason=" << latestStatus().last_reason;
}

// Same hypothesis, but via the NATURAL SUCCESS transition (no battery
// trigger, no kCanceling detour) -- the EXACT path the real G-M1 gate log
// showed (8/8 waypoints -> straight beginFinish(), no cancel involved).
TEST_F(AuthorityChainFixture, NaturalBodySuccessIntoFinishStaysRunningWithHealthyRealAuthority)
{
  goto_pose_server_->setHold(true);
  ASSERT_TRUE(sendExecuteMissionGoal("indoor_patrol", "loops: 1\ndwell_sec: 0.2\n"));
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("GotoPose", "accepted");}, 15.0))
    << "FAILED TO MEASURE: the mission never sent the body GotoPose goal";
  ASSERT_TRUE(waitFor([this]() {return latestAuthoritySource() == kSourceMission;}, 5.0))
    << "FAILED TO MEASURE: the real arbiter never selected MISSION";

  goto_pose_server_->setHold(false);
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("GotoPose", "succeeded");}, 5.0))
    << "FAILED TO MEASURE: the body GotoPose leg never completed";
  goto_pose_server_->setHold(true);   // re-arm BEFORE Finish's own GotoPose(home) can slip through

  const auto countGotoPoseAccepted = [this]() {
    std::lock_guard<std::mutex> lock(fake_state_.mutex);
    int count = 0;
    for (const auto & e : fake_state_.events) {
      if (e.first == "GotoPose" && e.second == "accepted") {
        ++count;
      }
    }
    return count;
  };
  ASSERT_TRUE(waitFor([&]() {return countGotoPoseAccepted() >= 2;}, 5.0))
    << "FAILED TO MEASURE: Finish's own GotoPose(home) leg never got dispatched after natural "
       "body SUCCESS (dwell_sec=0.2 elapsed, Repeat(1) should have completed)";

  const auto deadline = std::chrono::steady_clock::now() + 10s;
  while (std::chrono::steady_clock::now() < deadline) {
    ASSERT_NE(latestStatus().state, MissionStatusMsg::STATE_PAUSED)
      << "B1 regression (natural SUCCESS path): mission spuriously PAUSED during Finish with "
      << "authority continuously healthy through the REAL arbiter -- last_reason="
      << latestStatus().last_reason;
    std::this_thread::sleep_for(50ms);
  }

  goto_pose_server_->setHold(false);
  ASSERT_TRUE(
    waitFor([this]() {return latestStatus().state == MissionStatusMsg::STATE_COMPLETED;}, 15.0))
    << "FAILED TO MEASURE: mission never reached STATE_COMPLETED, got state="
    << static_cast<int>(latestStatus().state) << " reason=" << latestStatus().last_reason;
}

// ---------------------------------------------------------------- (G-M1)
// ROOT CAUSE CONFIRMED on the wire (2026-08-23, gm1_diag1.log lines
// 171-199): a real G-M1 gate run PAUSEd spuriously exactly on entering
// kFinishLanding, active_source=NONE, a clean 2 Hz heartbeat sawtooth (NOT
// executor starvation), seize_elapsed climbing to authority_loss_grace_sec
// then firing. Mechanism: the navigator hands the LANDING leg to PX4's OWN
// native LAND mode -- nobody streams cmd_mission once landing starts, so
// the REAL arbiter correctly reports NONE (an EXPECTED safe state while
// landing, not a seize). This test drives the REAL authority chain through
// EXACTLY that mechanism: let indoor_patrol complete its body and GotoHome
// leg naturally, dispatch Land, THEN stop streaming cmd_mission (simulating
// the navigator handing off to PX4-native-LAND) -- old code (kFinishLanding
// NOT exempt) spuriously PAUSEs; new code (kFinishLanding exempt) must
// finish landing and reach COMPLETED.
TEST_F(AuthorityChainFixture, LandingWithNoCommandStreamStaysRunningNotPaused)
{
  // Longer Land delay than the 20 ms default -- the mission must sit in
  // kFinishLanding for several real seconds, long enough for
  // authority_loss_grace_sec (1.5 s) to matter, matching the real G-M1
  // gate's multi-second LANDING window. Safe to swap here: no Land goal
  // exists yet (this runs before sendExecuteMissionGoal below).
  land_server_.reset();
  land_server_ = std::make_unique<FakeSimpleServer<Land>>(
    fake_navigator_node_, prefix_ + "/planning/land", fake_state_, "Land", 5000ms);

  ASSERT_TRUE(sendExecuteMissionGoal("indoor_patrol", "loops: 1\ndwell_sec: 0.2\n"));
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("GotoPose", "accepted");}, 15.0))
    << "FAILED TO MEASURE: the mission never sent the body GotoPose goal";
  ASSERT_TRUE(waitFor([this]() {return latestAuthoritySource() == kSourceMission;}, 5.0))
    << "FAILED TO MEASURE: the real arbiter never selected MISSION";

  // Body (1 waypoint, unheld) -> dwell 0.2s -> natural SUCCESS ->
  // beginFinish() -> GotoPose(home) (unheld) -> Land dispatched.
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("Land", "accepted");}, 10.0))
    << "FAILED TO MEASURE: Land was never dispatched after the body completed";

  // PX4-native-LAND simulation: nobody streams cmd_mission once LANDING
  // begins.
  setStreamCmdMission(false);
  ASSERT_TRUE(waitFor([this]() {return latestAuthoritySource() != kSourceMission;}, 5.0))
    << "FAILED TO MEASURE: the real arbiter never dropped MISSION out of active_source once "
       "cmd_mission stopped";

  // Old code: sits here up to authority_loss_grace_sec (1.5 s) then PAUSEs
  // spuriously. New code: kFinishLanding is exempt from the authority-
  // seize guard, must NOT pause here at all.
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline) {
    ASSERT_NE(latestStatus().state, MissionStatusMsg::STATE_PAUSED)
      << "G-M1 regression: kFinishLanding spuriously PAUSED once cmd_mission stopped (PX4-native-"
      << "LAND simulation, active_source=NONE) -- last_reason=" << latestStatus().last_reason;
    std::this_thread::sleep_for(50ms);
  }

  ASSERT_TRUE(
    waitFor([this]() {return latestStatus().state == MissionStatusMsg::STATE_COMPLETED;}, 10.0))
    << "FAILED TO MEASURE: mission never reached STATE_COMPLETED, got state="
    << static_cast<int>(latestStatus().state) << " reason=" << latestStatus().last_reason;
}

}  // namespace
}  // namespace uav_mission
