// P9.4 gate (plan P9-mission.md S:5): the REAL mission_executor_node driving
// the REAL NavGoalBroker against 7 in-process FAKE navigator action servers
// standing in for uav_navigation (R20: fabricates rclcpp_action traffic on
// real topic names -- isolated domain). Criteria (a)-(f) each get one
// TEST_F below; every assertion is anchored on an EVENT (R21), wall time is
// only ever a timeout valve.
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/quaternion.hpp>
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
#include <uav_interfaces/msg/localization_status.hpp>
#include <uav_interfaces/msg/mission_event.hpp>
#include <uav_interfaces/msg/mission_status.hpp>
#include <uav_interfaces/msg/semantic_landmark_array.hpp>
#include <uav_interfaces/msg/target_state.hpp>
#include <uav_interfaces/msg/target_track.hpp>
#include <uav_interfaces/msg/vehicle_health.hpp>
#include <uav_interfaces/srv/abort_mission.hpp>
#include <uav_interfaces/srv/pause_mission.hpp>
#include <uav_interfaces/srv/resume_mission.hpp>

#include "uav_mission/mission_executor_node.hpp"

using namespace std::chrono_literals;

namespace uav_mission
{
namespace
{

// COPY of mission_policy.hpp: the executor calls authority stale at twice this.
constexpr double kAuthorityHeartbeatCopySec = 0.5;

using AuthorityRound = std::pair<double, uint8_t>;

/// The last instant, at or before `until_sec`, when the executor would still have counted
/// authority as held -- reconstructed from the rounds a publisher actually sent.
///
/// A MISSION round grants authority from its own stamp until whichever comes first: the
/// next round replaces it, or it goes stale. Anything else grants nothing.
double authorityHeldEnd(
  const std::vector<AuthorityRound> & rounds, double until_sec, uint8_t mission_source)
{
  const double stale_after = 2.0 * kAuthorityHeartbeatCopySec;
  double held_end = 0.0;
  for (size_t i = 0; i < rounds.size(); ++i) {
    if (rounds[i].second != mission_source || rounds[i].first > until_sec) {
      continue;
    }
    const double replaced_at = (i + 1 < rounds.size())
      ? rounds[i + 1].first
      : std::numeric_limits<double>::infinity();
    held_end = std::max(
      held_end, std::min({replaced_at, rounds[i].first + stale_after, until_sec}));
  }
  return held_end;
}

using ExecuteMission = uav_interfaces::action::ExecuteMission;
using GotoPose = uav_interfaces::action::GotoPose;
using Takeoff = uav_interfaces::action::Takeoff;
using Land = uav_interfaces::action::Land;
using HoldPosition = uav_interfaces::action::HoldPosition;
using FollowPath = uav_interfaces::action::FollowPath;
using TrackTarget = uav_interfaces::action::TrackTarget;
using Recover = uav_interfaces::action::Recover;
using ControlAuthority = uav_interfaces::msg::ControlAuthority;
using LocalizationStatus = uav_interfaces::msg::LocalizationStatus;
using VehicleHealth = uav_interfaces::msg::VehicleHealth;
using TargetStateMsg = uav_interfaces::msg::TargetState;
using MissionStatusMsg = uav_interfaces::msg::MissionStatus;
using MissionEventMsg = uav_interfaces::msg::MissionEvent;
using SemanticLandmarkArray = uav_interfaces::msg::SemanticLandmarkArray;

// Same formula as mission_executor_node.cpp's tryYawFromQuaternion/
// yawOnlyQuaternion (which itself follows the uav_navigation carrot.cpp /
// uav_safety safety_supervisor_node.cpp precedent) -- reimplemented locally
// on the TEST side so the assertion below does not depend on the SUT's own
// (anonymous-namespace, unexported) helpers.
double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion yawOnlyQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

/// Runs on_exit at end of scope, REGARDLESS of how that scope was left
/// (normal fall-through OR an early `return` from a fatal ASSERT_*) -- C++
/// still runs local destructors on an early return, unlike a raw local
/// struct capturing `this` by pointer, which cannot reach the fixture's
/// protected members from outside the fixture's own class hierarchy. A
/// lambda captured here, defined INSIDE a TEST_F body, does not have that
/// problem (its enclosing scope already has access) -- used by the G-M4.4
/// identity-anchor test below to settle a long-running fake goal before
/// TearDown() destroys the objects its background thread still touches.
class ScopeGuard
{
public:
  explicit ScopeGuard(std::function<void()> on_exit)
  : on_exit_(std::move(on_exit)) {}
  ~ScopeGuard() {if (on_exit_) {on_exit_();}}

private:
  std::function<void()> on_exit_;
};

/// Copies the 3 real shipped XML files into a scratch directory, then lets
/// the caller overwrite one -- same helper as test_mission_registry.cpp's
/// ScratchMissionsDir, reimplemented locally (small, ROS-free-ish helper,
/// same reason the codebase already reimplements yaw-extraction locally
/// rather than sharing across test binaries). G-M3 gate (2026-08-22): lets
/// the crash-isolation test inject a real port-type mismatch through an
/// actual MissionExecutorNode instance, not a synthetic stand-in.
class ScratchMissionsDir
{
public:
  ScratchMissionsDir()
  {
    dir_ = std::filesystem::temp_directory_path() /
      ("uav_mission_executor_test_scratch_" +
      std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
    std::filesystem::create_directories(dir_);
    for (const char * name : {"indoor_patrol.xml", "follow_target.xml", "inspect_point.xml"}) {
      std::filesystem::copy_file(
        std::filesystem::path(UAV_MISSION_CONFIG_DIR) / "missions" / name, dir_ / name,
        std::filesystem::copy_options::overwrite_existing);
    }
  }

  ~ScratchMissionsDir()
  {
    std::error_code ec;
    std::filesystem::remove_all(dir_, ec);
  }

  void overwrite(const std::string & filename, const std::string & content) const
  {
    std::ofstream out(dir_ / filename, std::ios::trunc);
    out << content;
  }

  std::string path() const {return dir_.string();}

private:
  std::filesystem::path dir_;
};

/// Shared across all 7 fake navigator action servers -- the ONE thing that
/// lets criterion (a) be checked: a live-goal counter that is GLOBAL
/// (across all 7 action types), matching NavGoalBroker's node-wide "at most
/// one goal navigator ever alive" invariant, not a per-type one.
struct FakeNavigatorState
{
  std::mutex mutex;
  int live_goal_count = 0;
  bool multiple_live_goals_observed = false;

  struct Event
  {
    std::string action;
    std::string kind;
    uint64_t seq = 0;
  };
  std::vector<Event> events;
  uint64_t next_seq = 1;

  // G-M2 yaw fallout (2026-08-22): every GotoPose goal FakeGotoPoseServer
  // ever received, in order -- lets tests assert on what orientation the
  // mission actually sent, not just whether/when a goal landed.
  std::vector<geometry_msgs::msg::Quaternion> goto_pose_orientations;

  void recordGotoPoseGoal(const geometry_msgs::msg::Quaternion & orientation)
  {
    std::lock_guard<std::mutex> lock(mutex);
    goto_pose_orientations.push_back(orientation);
  }

  // G-M4.4 (2026-08-23): every TrackTarget goal FakeTrackTargetServer ever
  // received, in order -- the identity-anchor fix's whole point is WHICH
  // id each dispatched goal carries, not just that a goal landed.
  std::vector<int32_t> track_target_ids;

  void recordTrackTargetGoal(int32_t target_id)
  {
    std::lock_guard<std::mutex> lock(mutex);
    track_target_ids.push_back(target_id);
  }

  // G-M4.4: the world's CURRENT track sighting, test-controlled -- shared
  // between the TargetState stream (MissionExecutorFixture::publishOneRound(),
  // what TargetSeen/latestTarget() reads) and FakeTrackTargetServer's own
  // watchdog (what simulates navigator's targetSample(wanted_id) filter,
  // navigator_action_server_node.cpp:2597) so both agree on "what's
  // actually visible right now" without a second copy drifting out of sync.
  // Separate mutex from the event-bookkeeping one above: streamed at 50 Hz
  // from a different thread, no reason to contend on `mutex`.
  std::mutex world_mutex;
  bool world_target_visible = false;
  int32_t world_track_id = -1;
  double world_target_x = 0.0;
  double world_target_y = 0.0;

  struct WorldTarget
  {
    bool visible = false;
    int32_t track_id = -1;
    double x = 0.0;
    double y = 0.0;
  };

  void setWorldTarget(bool visible, int32_t track_id, double x, double y)
  {
    std::lock_guard<std::mutex> lock(world_mutex);
    world_target_visible = visible;
    world_track_id = track_id;
    world_target_x = x;
    world_target_y = y;
  }

  WorldTarget worldTarget()
  {
    std::lock_guard<std::mutex> lock(world_mutex);
    return {world_target_visible, world_track_id, world_target_x, world_target_y};
  }

  bool hasEvent(const std::string & action, const std::string & kind)
  {
    return findFirstSeq(action, kind) != 0;
  }

  /// Everything the navigator was actually asked to do, in order.
  std::string describeEvents()
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (events.empty()) {
      return "(the navigator was never asked for anything)";
    }
    std::string out;
    for (const Event & e : events) {
      out += " " + e.action + ":" + e.kind;
    }
    return out;
  }

  uint64_t findFirstSeq(const std::string & action, const std::string & kind)
  {
    std::lock_guard<std::mutex> lock(mutex);
    for (const Event & e : events) {
      if (e.action == action && e.kind == kind) {
        return e.seq;
      }
    }
    return 0;
  }

  uint64_t findFirstSeqAfter(const std::string & action, const std::string & kind, uint64_t after_seq)
  {
    std::lock_guard<std::mutex> lock(mutex);
    for (const Event & e : events) {
      if (e.action == action && e.kind == kind && e.seq > after_seq) {
        return e.seq;
      }
    }
    return 0;
  }
};

void countGoalStart(FakeNavigatorState & state, const std::string & action)
{
  std::lock_guard<std::mutex> lock(state.mutex);
  state.live_goal_count += 1;
  if (state.live_goal_count > 1) {
    state.multiple_live_goals_observed = true;
  }
  state.events.push_back({action, "accepted", state.next_seq++});
}

void countGoalEnd(FakeNavigatorState & state, const std::string & action, const std::string & kind)
{
  std::lock_guard<std::mutex> lock(state.mutex);
  state.live_goal_count -= 1;
  state.events.push_back({action, kind, state.next_seq++});
}

/// Immediate-ish success, used for Takeoff and the 3 action types none of
/// the 3 shipped missions or gate criteria (a)-(f) exercise (HoldPosition/
/// FollowPath/Recover) -- present only so the node's 7-client wiring always
/// has a live server to talk to. TrackTarget moved to its own
/// FakeTrackTargetServer below (G-M4.4, 2026-08-23): it needs id-aware
/// behaviour a fixed-delay-then-succeed stub cannot give it.
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
        countGoalStart(state_, label_);
        std::thread([this, goal_handle]() {
          std::this_thread::sleep_for(delay_);
          auto result = std::make_shared<typename ActionT::Result>();
          if (goal_handle->is_canceling()) {
            result->success = false;
            result->result_code = kResultCanceled;
            result->message = "canceled";
            goal_handle->canceled(result);
            countGoalEnd(state_, label_, "canceled");
            return;
          }
          result->success = true;
          result->result_code = kResultSucceeded;
          goal_handle->succeed(result);
          countGoalEnd(state_, label_, "succeeded");
        }).detach();
      });
  }

private:
  FakeNavigatorState & state_;
  std::string label_;
  std::chrono::milliseconds delay_;
  typename rclcpp_action::Server<ActionT>::SharedPtr server_;
};

/// TrackTarget, id-aware (G-M4.4, 2026-08-23): simulates navigator_action_
/// server_node.cpp:2597's targetSample(wanted_id) filter -- a TargetState
/// sample that is invisible OR belongs to a DIFFERENT track_id than the
/// goal's own target_id is "unusable" for that goal. Unusable for >= the
/// goal's OWN target_lost_timeout aborts ABORTED_LOST_TARGET, exactly like
/// the real navigator's watchdog. wanted_id < 0 ("any") never goes unusable
/// while something is visible -- this is precisely what let a ghost track
/// defeat detection before the mission-side anchor fix (bt_nodes.cpp
/// TargetSeen -> {acquired_track_id} -> NavAction's target_id port).
class FakeTrackTargetServer
{
public:
  FakeTrackTargetServer(
    const rclcpp::Node::SharedPtr & node, const std::string & name, FakeNavigatorState & state)
  : state_(state)
  {
    server_ = rclcpp_action::create_server<TrackTarget>(
      node, name,
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const TrackTarget::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<TrackTarget>>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<TrackTarget>> goal_handle) {
        const auto goal = goal_handle->get_goal();
        const int32_t wanted_id = goal->target_id;
        const double lost_timeout = static_cast<double>(goal->target_lost_timeout);
        state_.recordTrackTargetGoal(wanted_id);
        countGoalStart(state_, "TrackTarget");
        std::thread([this, goal_handle, wanted_id, lost_timeout]() {
          const auto usable = [this, wanted_id]() {
            const FakeNavigatorState::WorldTarget w = state_.worldTarget();
            return w.visible && (wanted_id < 0 || w.track_id == wanted_id);
          };
          std::optional<std::chrono::steady_clock::time_point> unusable_since;
          while (rclcpp::ok() && !goal_handle->is_canceling()) {
            if (usable()) {
              unusable_since.reset();
            } else if (!unusable_since.has_value()) {
              unusable_since = std::chrono::steady_clock::now();
            } else if (
              std::chrono::duration<double>(
                std::chrono::steady_clock::now() - *unusable_since).count() >= lost_timeout)
            {
              break;   // LOST_TARGET, matching the real navigator's own watchdog
            }
            std::this_thread::sleep_for(10ms);
          }
          if (!rclcpp::ok()) {
            return;
          }
          auto result = std::make_shared<TrackTarget::Result>();
          if (goal_handle->is_canceling()) {
            result->success = false;
            result->result_code = kResultCanceled;
            result->message = "canceled";
            goal_handle->canceled(result);
            countGoalEnd(state_, "TrackTarget", "canceled");
            return;
          }
          // Loop only exits non-canceled once unusable_since aged past
          // lost_timeout -- always a loss here (duration_seconds=0.0 in
          // every shipped mission means "track until told otherwise", so
          // this fake never self-succeeds, matching real usage).
          result->success = false;
          result->result_code = kResultAbortedLostTarget;
          result->message = "target lost (fake :2597 watchdog)";
          goal_handle->abort(result);
          countGoalEnd(state_, "TrackTarget", "lost");
        }).detach();
      });
  }

private:
  FakeNavigatorState & state_;
  rclcpp_action::Server<TrackTarget>::SharedPtr server_;
};

/// GotoPose, hold-controlled: while hold_ is true, the goal sits RUNNING
/// (polled every 10 ms) until setHold(false)/release() -- lets tests pin
/// down exactly when a body-step "arrives" without racing real navigator
/// timing. Defaults to hold_=false (near-instant SUCCESS), so tests that
/// do not care about timing can just let a whole mission run free.
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
        state_.recordGotoPoseGoal(goal_handle->get_goal()->target_pose.orientation);
        countGoalStart(state_, "GotoPose");
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
            countGoalEnd(state_, "GotoPose", "canceled");
            return;
          }
          result->success = true;
          result->result_code = kResultSucceeded;
          result->final_distance_error = 0.05f;
          goal_handle->succeed(result);
          countGoalEnd(state_, "GotoPose", "succeeded");
        }).detach();
      });
  }

  void setHold(bool hold) {hold_.store(hold);}

private:
  FakeNavigatorState & state_;
  std::atomic<bool> hold_{false};
  rclcpp_action::Server<GotoPose>::SharedPtr server_;
};

/// Land: cancel refusable (gate criterion (f), uav_navigation/README.md
/// "Cancel khi dang LANDING bi TU CHOI") -- always completes for real
/// ~300 ms later regardless of any refused cancel attempt.
class FakeLandServer
{
public:
  FakeLandServer(const rclcpp::Node::SharedPtr & node, const std::string & name, FakeNavigatorState & state)
  : state_(state)
  {
    server_ = rclcpp_action::create_server<Land>(
      node, name,
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Land::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Land>>) {
        return refuse_cancel_.load() ?
          rclcpp_action::CancelResponse::REJECT : rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Land>> goal_handle) {
        countGoalStart(state_, "Land");
        std::thread([this, goal_handle]() {
          std::this_thread::sleep_for(300ms);
          auto result = std::make_shared<Land::Result>();
          if (goal_handle->is_canceling()) {
            result->success = false;
            result->result_code = kResultCanceled;
            result->message = "canceled";
            result->landed_on_marker = false;
            goal_handle->canceled(result);
            countGoalEnd(state_, "Land", "canceled");
            return;
          }
          result->success = true;
          result->result_code = kResultSucceeded;
          result->landed_on_marker = false;
          goal_handle->succeed(result);
          countGoalEnd(state_, "Land", "succeeded");
        }).detach();
      });
  }

  void setRefuseCancel(bool refuse) {refuse_cancel_.store(refuse);}

private:
  FakeNavigatorState & state_;
  std::atomic<bool> refuse_cancel_{false};
  rclcpp_action::Server<Land>::SharedPtr server_;
};

class MissionExecutorFixture : public ::testing::Test
{
protected:
  /// Factored out of SetUp() (G-M3 gate, 2026-08-22) so the crash-isolation
  /// test can build a SECOND node against a scratch missions_dir (a crafted
  /// port-type mismatch) and swap it in mid-test, still on uav_id_'s SAME
  /// topic namespace -- the existing execute_client_/status_sub_ pick it up
  /// transparently once it is added to executor_ and the old one removed.
  // extra_params (G-M4.4b, 2026-08-23): lets a test override e.g.
  // max_step_retries/retry_backoff_sec/track_progress_reset_sec without
  // touching every OTHER test's node -- default empty preserves every
  // existing call site's behavior exactly.
  std::shared_ptr<MissionExecutorNode> buildMissionNode(
    const std::string & missions_dir, const std::vector<rclcpp::Parameter> & extra_params = {})
  {
    rclcpp::NodeOptions node_options;
    std::vector<rclcpp::Parameter> params = {
      rclcpp::Parameter("uav_id", uav_id_),
      rclcpp::Parameter("missions_dir", missions_dir),
      rclcpp::Parameter("tick_hz", 20.0)};
    params.insert(params.end(), extra_params.begin(), extra_params.end());
    node_options.parameter_overrides(params);
    return std::make_shared<MissionExecutorNode>(node_options);
  }

  /// Replaces node_ with a fresh one built against a different missions_dir.
  /// Two hazards found by gdb while diagnosing this exact helper (2026-08-22,
  /// both real environment/library limits, not a logic bug on our side):
  ///   1. add_node()/remove_node() on a MultiThreadedExecutor that still has
  ///      OTHER threads blocked inside rmw_wait() segfaults inside
  ///      rmw_fastrtps_shared_cpp::__rmw_wait() -- so spin must be FULLY
  ///      stopped (cancel + join) before the node set changes at all.
  ///   2. Re-spinning the SAME MultiThreadedExecutor object after a
  ///      cancel() is not reliable either (hit "guard condition
  ///      implementation is invalid" from a stale wait set) -- so the
  ///      executor itself is rebuilt fresh here, identical to SetUp()'s own
  ///      construction, rather than reused.
  /// Also reset() the OLD node BEFORE constructing the new one: "node_ =
  /// buildMissionNode(...)" evaluates the RHS (constructing the new node,
  /// same uav_id_ => same topic/action names) BEFORE releasing the old one,
  /// which briefly doubles up every server name (a THIRD hazard hit first).
  void swapMissionNode(
    const std::string & missions_dir, const std::vector<rclcpp::Parameter> & extra_params = {})
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    node_.reset();
    node_ = buildMissionNode(missions_dir, extra_params);

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 8);
    executor_->add_node(node_);
    executor_->add_node(fake_navigator_node_);
    executor_->add_node(probe_);
    spin_thread_ = std::thread([this]() {executor_->spin();});
  }

  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    static std::atomic<int> instances{0};
    uav_id_ = "uav" + std::to_string(instances.fetch_add(1));
    prefix_ = "/uav/" + uav_id_;

    node_ = buildMissionNode(std::string(UAV_MISSION_CONFIG_DIR) + "/missions");

    fake_navigator_node_ = std::make_shared<rclcpp::Node>("fake_navigator_" + uav_id_);
    goto_pose_server_ =
      std::make_unique<FakeGotoPoseServer>(fake_navigator_node_, prefix_ + "/planning/goto_pose", fake_state_);
    takeoff_server_ = std::make_unique<FakeSimpleServer<Takeoff>>(
      fake_navigator_node_, prefix_ + "/planning/takeoff", fake_state_, "Takeoff");
    land_server_ =
      std::make_unique<FakeLandServer>(fake_navigator_node_, prefix_ + "/planning/land", fake_state_);
    hold_stub_ = std::make_unique<FakeSimpleServer<HoldPosition>>(
      fake_navigator_node_, prefix_ + "/planning/hold_position", fake_state_, "HoldPosition");
    follow_stub_ = std::make_unique<FakeSimpleServer<FollowPath>>(
      fake_navigator_node_, prefix_ + "/planning/follow_path", fake_state_, "FollowPath");
    track_target_server_ = std::make_unique<FakeTrackTargetServer>(
      fake_navigator_node_, prefix_ + "/planning/track_target", fake_state_);
    recover_stub_ = std::make_unique<FakeSimpleServer<Recover>>(
      fake_navigator_node_, prefix_ + "/planning/recover", fake_state_, "Recover");

    probe_ = std::make_shared<rclcpp::Node>("mission_probe_" + uav_id_);
    authority_pub_ = probe_->create_publisher<ControlAuthority>(
      prefix_ + "/control/authority", rclcpp::QoS(1).reliable().transient_local());
    health_pub_ = probe_->create_publisher<VehicleHealth>(prefix_ + "/state/health_px4", rclcpp::QoS(10));
    localization_pub_ =
      probe_->create_publisher<LocalizationStatus>(prefix_ + "/state/localization_status", rclcpp::QoS(10));
    odometry_pub_ =
      probe_->create_publisher<nav_msgs::msg::Odometry>(prefix_ + "/state/odometry_fused", rclcpp::QoS(10));
    landmarks_pub_ = probe_->create_publisher<SemanticLandmarkArray>(
      prefix_ + "/world/semantic_landmarks", rclcpp::QoS(10));
    target_pub_ = probe_->create_publisher<TargetStateMsg>(
      prefix_ + "/world/target_state", rclcpp::QoS(10));

    status_sub_ = probe_->create_subscription<MissionStatusMsg>(
      prefix_ + "/mission/status", rclcpp::QoS(1).reliable().transient_local(),
      [this](MissionStatusMsg::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        statuses_.push_back(*msg);
      });
    events_sub_ = probe_->create_subscription<MissionEventMsg>(
      prefix_ + "/mission/events", rclcpp::QoS(50),
      [this](MissionEventMsg::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        events_.push_back(*msg);
      });

    pause_client_ = probe_->create_client<uav_interfaces::srv::PauseMission>(prefix_ + "/mission/pause");
    resume_client_ = probe_->create_client<uav_interfaces::srv::ResumeMission>(prefix_ + "/mission/resume");
    abort_client_ = probe_->create_client<uav_interfaces::srv::AbortMission>(prefix_ + "/mission/abort");
    execute_client_ = rclcpp_action::create_client<ExecuteMission>(probe_, prefix_ + "/mission/execute_mission");

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 8);
    executor_->add_node(node_);
    executor_->add_node(fake_navigator_node_);
    executor_->add_node(probe_);
    spin_thread_ = std::thread([this]() {executor_->spin();});

    ASSERT_TRUE(waitFor([this]() {return execute_client_->action_server_is_ready();}, 10.0))
      << "FAILED TO MEASURE: mission_executor_node's ExecuteMission server never came up";

    stream_stop_.store(false);
    stream_thread_ = std::thread([this]() {streamWorld();});

    ASSERT_TRUE(waitFor([this]() {return !statuses().empty();}, 5.0))
      << "FAILED TO MEASURE: mission/status never published";
  }

  void TearDown() override
  {
    expectEveryStatusNamedItsGoal();
    stream_stop_.store(true);
    if (stream_thread_.joinable()) {
      stream_thread_.join();
    }
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    node_.reset();
    fake_navigator_node_.reset();
    probe_.reset();
    executor_.reset();
  }

  // ------------------------------------------------------------ streaming

  /// 🔴 This ONE sleeping thread is the whole stimulus, and it can be left
  /// unscheduled far past the rules the executor judges by -- measured 2026-08-25
  /// under load, a 2.242 s gap against a 1.0 s staleness threshold. Landing during
  /// setup pauses the mission, and PAUSED never self-resumes, so every later claim
  /// in that run is about a mission that stopped for the RIG's reason. Every wait on
  /// a premise here therefore reports worstAuthorityGap() when it fails.
  /// Do NOT "fix" this by adding a second publisher: tried 2026-08-25, it doubles
  /// every round in publishOneRound() and turned 10 other tests red.
  void streamWorld()
  {
    while (!stream_stop_.load()) {
      const double stall = stream_stall_sec_.exchange(0.0);
      if (stall > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(stall));
        continue;
      }
      publishOneRound();
      std::this_thread::sleep_for(20ms);
    }
  }

  /// Test-only: silence the stream for `seconds`, so a test can PROVE the executor
  /// reacts to a real authority lapse instead of trusting that it would.
  void injectStreamStall(double seconds) {stream_stall_sec_.store(seconds);}

  void publishOneRound()
  {
    uint8_t authority_source;
    double battery;
    bool localization_valid;
    bool landmark_visible;
    int32_t landmark_marker_id;
    double landmark_x;
    double landmark_y;
    double odometry_yaw;
    {
      std::lock_guard<std::mutex> lock(world_mutex_);
      authority_source = authority_source_;
      battery = battery_remaining_;
      localization_valid = localization_valid_;
      landmark_visible = landmark_visible_;
      landmark_marker_id = landmark_marker_id_;
      landmark_x = landmark_x_;
      landmark_y = landmark_y_;
      odometry_yaw = odometry_yaw_;
    }

    ControlAuthority auth;
    auth.header.stamp = probe_->now();
    auth.uav_id = uav_id_;
    auth.active_source = authority_source;
    authority_pub_->publish(auth);
    {
      std::lock_guard<std::mutex> lock(authority_log_mutex_);
      authority_log_.push_back({rclcpp::Time(auth.header.stamp).seconds(), authority_source});
    }

    VehicleHealth health;
    health.header.stamp = probe_->now();
    health.uav_id = uav_id_;
    health.battery_remaining = static_cast<float>(battery);
    health.overall_level = VehicleHealth::LEVEL_OK;
    health_pub_->publish(health);

    LocalizationStatus loc;
    loc.header.stamp = probe_->now();
    loc.uav_id = uav_id_;
    loc.is_valid = localization_valid;
    loc.active_source = LocalizationStatus::SOURCE_FUSED;
    localization_pub_->publish(loc);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = probe_->now();
    odom.header.frame_id = "odom";
    odom.pose.pose.orientation = yawOnlyQuaternion(odometry_yaw);
    odometry_pub_->publish(odom);

    SemanticLandmarkArray landmarks;
    landmarks.header.stamp = probe_->now();
    landmarks.uav_id = uav_id_;
    if (landmark_visible) {
      uav_interfaces::msg::SemanticLandmark landmark;
      landmark.marker_id = landmark_marker_id;
      landmark.pose.position.x = landmark_x;
      landmark.pose.position.y = landmark_y;
      landmark.pose.orientation.w = 1.0;
      landmark.time_since_seen_sec = 0.0f;
      landmarks.landmarks.push_back(landmark);
    }
    landmarks_pub_->publish(landmarks);

    // G-M4.4: same worldTarget() the fake TrackTarget server's watchdog
    // reads (FakeNavigatorState), so both sides of the fake navigator agree
    // with what latestTarget()/TargetSeen sees on the REAL node under test.
    const FakeNavigatorState::WorldTarget world_target = fake_state_.worldTarget();
    TargetStateMsg target;
    target.header.stamp = probe_->now();
    target.header.frame_id = "odom";
    target.uav_id = uav_id_;
    if (world_target.visible) {
      target.track_id = world_target.track_id;
      target.status = uav_interfaces::msg::TargetTrack::STATUS_TRACKING;
      target.pose.position.x = world_target.x;
      target.pose.position.y = world_target.y;
      target.pose.orientation.w = 1.0;
      target.time_since_seen_sec = 0.0f;
    } else {
      target.time_since_seen_sec = std::numeric_limits<float>::infinity();
    }
    target_pub_->publish(target);
  }

  // B2/Y3 (2026-08-23, review round 1): authority/health/localization/
  // odometry deliberately kept FRESH while landmarks/target stay frozen
  // (the caller stops streamWorld() first) -- isolates the world_model-
  // died scenario from ALSO tripping the unrelated pre-existing battery-
  // unmeasured start gate, authority/localization staleness, or (Y3) the
  // NEW odometry-freshness goal-acceptance gate, any of which would
  // otherwise reject/pause the mission for the WRONG reason before the
  // behavior actually under test ever gets a chance to run.
  void publishFreshEverythingExceptLandmarksAndTarget()
  {
    uint8_t authority_source;
    double battery;
    bool localization_valid;
    double odometry_yaw;
    {
      std::lock_guard<std::mutex> lock(world_mutex_);
      authority_source = authority_source_;
      battery = battery_remaining_;
      localization_valid = localization_valid_;
      odometry_yaw = odometry_yaw_;
    }
    ControlAuthority auth;
    auth.header.stamp = probe_->now();
    auth.uav_id = uav_id_;
    auth.active_source = authority_source;
    authority_pub_->publish(auth);
    {
      std::lock_guard<std::mutex> lock(authority_log_mutex_);
      authority_log_.push_back({rclcpp::Time(auth.header.stamp).seconds(), authority_source});
    }

    VehicleHealth health;
    health.header.stamp = probe_->now();
    health.uav_id = uav_id_;
    health.battery_remaining = static_cast<float>(battery);
    health.overall_level = VehicleHealth::LEVEL_OK;
    health_pub_->publish(health);

    LocalizationStatus loc;
    loc.header.stamp = probe_->now();
    loc.uav_id = uav_id_;
    loc.is_valid = localization_valid;
    loc.active_source = LocalizationStatus::SOURCE_FUSED;
    localization_pub_->publish(loc);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = probe_->now();
    odom.header.frame_id = "odom";
    odom.pose.pose.orientation = yawOnlyQuaternion(odometry_yaw);
    odometry_pub_->publish(odom);
  }

  void setAuthoritySource(uint8_t source)
  {
    std::lock_guard<std::mutex> lock(world_mutex_);
    authority_source_ = source;
  }
  void setBattery(double remaining)
  {
    std::lock_guard<std::mutex> lock(world_mutex_);
    battery_remaining_ = remaining;
  }
  void setLocalizationValid(bool valid)
  {
    std::lock_guard<std::mutex> lock(world_mutex_);
    localization_valid_ = valid;
  }
  /// G-M2 regression: streams a visible landmark (matches inspect_point.xml's
  /// MarkerSeen leaves) -- marker_id=-1 (any) unless a test needs a specific one.
  void setLandmarkVisible(double x, double y, int32_t marker_id = -1)
  {
    std::lock_guard<std::mutex> lock(world_mutex_);
    landmark_visible_ = true;
    landmark_marker_id_ = marker_id;
    landmark_x_ = x;
    landmark_y_ = y;
  }
  /// G-M2 yaw fallout: sets the yaw streamed on /state/odometry_fused --
  /// sendGotoPose() (mission_executor_node.cpp) must echo this back on every
  /// GotoPose goal it dispatches, never a hardcoded yaw 0.
  void setOdometryYaw(double yaw)
  {
    std::lock_guard<std::mutex> lock(world_mutex_);
    odometry_yaw_ = yaw;
  }
  /// G-M4.4: sets the world's CURRENT target sighting -- streamed on
  /// /world/target_state (what the REAL node's TargetSeen/latestTarget()
  /// reads) AND read by FakeTrackTargetServer's own watchdog (what decides
  /// whether an in-flight goal's anchored id is still "usable"). One
  /// source of truth (FakeNavigatorState::worldTarget()) for both.
  void setWorldTarget(bool visible, int32_t track_id, double x, double y)
  {
    fake_state_.setWorldTarget(visible, track_id, x, y);
  }
  void hideWorldTarget() {fake_state_.setWorldTarget(false, -1, 0.0, 0.0);}

  // -------------------------------------------------------------- helpers

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

  std::vector<MissionStatusMsg> statuses()
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return statuses_;
  }

  // Contract 2.19, checked over EVERY sample of EVERY test rather than only the last one
  // of a few. The two unforced publish paths (mission_executor_node.cpp:746 and :897) emit
  // their bad sample only on the 2 Hz `due` beat, so a per-test terminal assertion sees it
  // roughly one run in five -- that intermittency is what debt #15 recorded as flakiness.
  // Twenty-three missions each contributing their whole stream is a far better detector,
  // and it costs nothing: the samples are already collected.
  void expectEveryStatusNamedItsGoal()
  {
    size_t offenders = 0;
    uint8_t first_bad_state = 0;
    for (const auto & s : statuses()) {
      if (s.state != MissionStatusMsg::STATE_IDLE && s.goal_id.empty()) {
        if (offenders == 0) {
          first_bad_state = s.state;
        }
        ++offenders;
      }
    }
    EXPECT_EQ(offenders, 0U)
      << "contract 2.19: " << offenders << " MissionStatus sample(s) carried an empty "
      << "goal_id while state was " << static_cast<int>(first_bad_state) << " (not IDLE). "
      << "A sample published after settleGoalHandle() reads the id off a handle that is "
      << "already gone, and the last such sample is what every late joiner reads out of "
      << "the TransientLocal cache.";
  }

  MissionStatusMsg latestStatus()
  {
    auto s = statuses();
    return s.empty() ? MissionStatusMsg{} : s.back();
  }

  bool hasMissionEvent(uint8_t event_type)
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    for (const MissionEventMsg & e : events_) {
      if (e.event_type == event_type) {
        return true;
      }
    }
    return false;
  }

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

  bool callPauseMission(const std::string & mission_id)
  {
    auto request = std::make_shared<uav_interfaces::srv::PauseMission::Request>();
    request->mission_id = mission_id;
    if (!pause_client_->wait_for_service(3s)) {
      return false;
    }
    auto future = pause_client_->async_send_request(request);
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

  bool callAbortMission(const std::string & reason)
  {
    auto request = std::make_shared<uav_interfaces::srv::AbortMission::Request>();
    request->mission_id = "";
    request->reason = reason;
    if (!abort_client_->wait_for_service(3s)) {
      return false;
    }
    auto future = abort_client_->async_send_request(request);
    if (future.wait_for(3s) != std::future_status::ready) {
      return false;
    }
    return future.get()->success;
  }

  /// Matches mission_executor_node.cpp's own (anonymous-namespace, not
  /// exported) uuidToHex() byte-for-byte -- used to prove a terminal
  /// MissionStatus.goal_id is not just non-empty but the SETTLING goal's
  /// own UUID.
  static std::string uuidHex(const rclcpp_action::GoalUUID & uuid)
  {
    std::ostringstream oss;
    for (uint8_t byte : uuid) {
      oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
    }
    return oss.str();
  }

  // ------------------------------------------------------------- members

  std::string uav_id_;
  std::string prefix_;

  std::shared_ptr<MissionExecutorNode> node_;
  rclcpp::Node::SharedPtr fake_navigator_node_;
  rclcpp::Node::SharedPtr probe_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;

  FakeNavigatorState fake_state_;
  std::unique_ptr<FakeGotoPoseServer> goto_pose_server_;
  std::unique_ptr<FakeSimpleServer<Takeoff>> takeoff_server_;
  std::unique_ptr<FakeLandServer> land_server_;
  std::unique_ptr<FakeSimpleServer<HoldPosition>> hold_stub_;
  std::unique_ptr<FakeSimpleServer<FollowPath>> follow_stub_;
  std::unique_ptr<FakeTrackTargetServer> track_target_server_;
  std::unique_ptr<FakeSimpleServer<Recover>> recover_stub_;

  /// When the executor would have STOPPED seeing authority as held, before `until_sec`.
  ///
  /// The test used to time the grace from its own call to setAuthoritySource(), which is
  /// only the same instant when the world stream is on time. It is a plain thread doing
  /// sleep_for(20ms), so under a 12-package parallel ctest it can starve past the node's
  /// staleness window (2 x authority_heartbeat_copy_sec = 1.0 s) -- and then the grace has
  /// ALREADY been running when the seize is published. Measured 2026-08-25: elapsed read
  /// 0.281 s against a 1.5 s grace, which accuses the executor of a rule it kept.
  ///
  /// So reconstruct the executor's own predicate from the stream this fixture published:
  /// authority counts as held from a MISSION round until either the next round replaces it
  /// or it goes stale, whichever comes first.
  double authorityHeldEndBefore(double until_sec)
  {
    std::vector<AuthorityRound> log;
    {
      std::lock_guard<std::mutex> lock(authority_log_mutex_);
      log = authority_log_;
    }
    return authorityHeldEnd(log, until_sec, kSourceMission);
  }

  /// Longest silence in the fixture's own authority stream, for evidence lines.
  double worstAuthorityGap()
  {
    std::lock_guard<std::mutex> lock(authority_log_mutex_);
    double worst = 0.0;
    for (size_t i = 1; i < authority_log_.size(); ++i) {
      worst = std::max(worst, authority_log_[i].first - authority_log_[i - 1].first);
    }
    return worst;
  }

  std::mutex authority_log_mutex_;
  std::vector<AuthorityRound> authority_log_;

  std::mutex world_mutex_;
  uint8_t authority_source_ = kSourceMission;
  double battery_remaining_ = 0.9;
  bool localization_valid_ = true;
  bool landmark_visible_ = false;
  int32_t landmark_marker_id_ = -1;
  double landmark_x_ = 0.0;
  double landmark_y_ = 0.0;
  double odometry_yaw_ = 0.0;
  std::atomic<bool> stream_stop_{true};
  std::thread stream_thread_;
  std::atomic<double> stream_stall_sec_{0.0};

  rclcpp::Publisher<ControlAuthority>::SharedPtr authority_pub_;
  rclcpp::Publisher<VehicleHealth>::SharedPtr health_pub_;
  rclcpp::Publisher<SemanticLandmarkArray>::SharedPtr landmarks_pub_;
  rclcpp::Publisher<TargetStateMsg>::SharedPtr target_pub_;
  rclcpp::Publisher<LocalizationStatus>::SharedPtr localization_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  std::mutex status_mutex_;
  std::vector<MissionStatusMsg> statuses_;
  std::vector<MissionEventMsg> events_;
  rclcpp::Subscription<MissionStatusMsg>::SharedPtr status_sub_;
  rclcpp::Subscription<MissionEventMsg>::SharedPtr events_sub_;

  rclcpp::Client<uav_interfaces::srv::PauseMission>::SharedPtr pause_client_;
  rclcpp::Client<uav_interfaces::srv::ResumeMission>::SharedPtr resume_client_;
  rclcpp::Client<uav_interfaces::srv::AbortMission>::SharedPtr abort_client_;
  rclcpp_action::Client<ExecuteMission>::SharedPtr execute_client_;
  rclcpp_action::ClientGoalHandle<ExecuteMission>::SharedPtr goal_handle_;
};

// ---------------------------------------------------------------- (a)

TEST_F(MissionExecutorFixture, NeverMoreThanOneLiveNavigatorGoal)
{
  ASSERT_TRUE(sendExecuteMissionGoal("indoor_patrol"));
  ASSERT_TRUE(
    waitFor(
      [this]() {
        const auto s = latestStatus();
        return s.state == MissionStatusMsg::STATE_COMPLETED || s.state == MissionStatusMsg::STATE_ABORTED;
      }, 40.0))
    << "FAILED TO MEASURE: mission never reached a terminal state";
  EXPECT_FALSE(fake_state_.multiple_live_goals_observed)
    << "more than one navigator goal was alive at the same time";
  EXPECT_GT(fake_state_.next_seq, 1u) << "FAILED TO MEASURE: no navigator goal traffic observed at all";
  EXPECT_EQ(latestStatus().last_result_code, kResultSucceeded);
}

// ---------------------------------------------------------------- (b)

TEST_F(MissionExecutorFixture, CancelHappensBeforeTheNextGoalIsSent)
{
  goto_pose_server_->setHold(true);
  ASSERT_TRUE(sendExecuteMissionGoal("indoor_patrol"));
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("GotoPose", "accepted");}, 10.0))
    << "FAILED TO MEASURE: first body GotoPose never reached the fake navigator";

  setAuthoritySource(kSourceOperator);
  ASSERT_TRUE(waitFor([this]() {return latestStatus().state == MissionStatusMsg::STATE_PAUSED;}, 5.0));
  const uint64_t cancel_seq = fake_state_.findFirstSeq("GotoPose", "canceled");
  ASSERT_NE(cancel_seq, 0u) << "FAILED TO MEASURE: the held GotoPose was never actually canceled";

  goto_pose_server_->setHold(false);
  setAuthoritySource(kSourceMission);
  ASSERT_TRUE(callResumeMission(""));

  ASSERT_TRUE(
    waitFor([this, cancel_seq]() {return fake_state_.findFirstSeqAfter("GotoPose", "accepted", cancel_seq) != 0;}, 10.0))
    << "FAILED TO MEASURE: no new GotoPose goal was ever sent after resume";
  const uint64_t next_accept_seq = fake_state_.findFirstSeqAfter("GotoPose", "accepted", cancel_seq);
  EXPECT_GT(next_accept_seq, cancel_seq)
    << "a new GotoPose goal was accepted before the previous one's cancel landed";
}

// ------------------------------------------- the anchor rule, pinned on its own

TEST(AuthorityGraceAnchor, AnUninterruptedStreamGrantsAuthorityUntilTheSeize)
{
  std::vector<AuthorityRound> rounds;
  for (int tick = 0; tick < 100; ++tick) {          // 2 s of 50 Hz MISSION
    rounds.push_back({tick * 0.02, kSourceMission});
  }
  rounds.push_back({2.00, kSourceOperator});

  EXPECT_NEAR(authorityHeldEnd(rounds, 3.5, kSourceMission), 2.00, 1e-9);
}

TEST(AuthorityGraceAnchor, AStalledStreamStopsGrantingAuthorityBeforeTheSeize)
{
  // The measured case, 2026-08-25: the fixture's own thread starved, the executor's grace
  // began at the staleness edge, and timing from the seize read 0.281 s of a 1.5 s grace.
  std::vector<AuthorityRound> rounds;
  for (int tick = 0; tick < 50; ++tick) {
    rounds.push_back({tick * 0.02, kSourceMission});
  }
  rounds.push_back({4.00, kSourceOperator});        // 3 s of silence, then the seize

  // Held ends 1.0 s after the last MISSION round, NOT at the seize 3 s later.
  EXPECT_NEAR(authorityHeldEnd(rounds, 5.0, kSourceMission), 0.98 + 1.0, 1e-9);
}

TEST(AuthorityGraceAnchor, ARoundThatArrivesAfterTheWindowCannotGrantAuthority)
{
  const std::vector<AuthorityRound> rounds{{0.0, kSourceMission}, {9.0, kSourceMission}};

  EXPECT_NEAR(authorityHeldEnd(rounds, 2.0, kSourceMission), 1.0, 1e-9);
}

TEST(AuthorityGraceAnchor, NothingIsGrantedWhenNoMissionRoundWasEverSent)
{
  const std::vector<AuthorityRound> rounds{{0.0, kSourceOperator}, {0.5, kSourceOperator}};

  // 0.0 is the sentinel the fixture turns into FAILED TO MEASURE.
  EXPECT_EQ(authorityHeldEnd(rounds, 2.0, kSourceMission), 0.0);
}

// ---------------------------------------------------------------- (c)

TEST_F(MissionExecutorFixture, AuthoritySeizeSustainedCancelsAndPauses)
{
  goto_pose_server_->setHold(true);
  ASSERT_TRUE(sendExecuteMissionGoal("indoor_patrol"));
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("GotoPose", "accepted");}, 10.0))
    << "FAILED TO MEASURE: the mission never reached GotoPose"
    << "\n  navigator saw:" << fake_state_.describeEvents()
    << "\n  mission state " << static_cast<int>(latestStatus().state)
    << ", last_result_code " << static_cast<int>(latestStatus().last_result_code)
    << "\n  worst gap in this fixture's own authority stream: " << worstAuthorityGap()
    << " s (the executor calls authority stale after 1.0 s, and PAUSED never self-resumes)";

  const double seize_called_sec = probe_->now().seconds();
  setAuthoritySource(kSourceOperator);

  ASSERT_TRUE(waitFor([this]() {return latestStatus().state == MissionStatusMsg::STATE_PAUSED;}, 6.0))
    << "FAILED TO MEASURE: mission never reached PAUSED after a sustained authority seize";

  // Both ends come off the wire: the pause is stamped by the executor when it PUBLISHED,
  // and the start is when this fixture's own stream stopped granting authority.
  const double paused_at_sec = rclcpp::Time(latestStatus().header.stamp).seconds();
  const double grace_started_sec = authorityHeldEndBefore(paused_at_sec);
  ASSERT_GT(grace_started_sec, 0.0)
    << "FAILED TO MEASURE: no MISSION authority round was ever published before the pause";
  const double elapsed = paused_at_sec - grace_started_sec;

  std::cout << "[ EVIDENCE ] grace ran " << elapsed << " s; the seize was published "
            << (seize_called_sec - grace_started_sec)
            << " s after authority stopped being held; worst gap in this fixture's own "
            << "authority stream was " << worstAuthorityGap() << " s (stale after 1.0 s)"
            << std::endl;

  EXPECT_GE(elapsed, 1.5) << "paused before authority_loss_grace_sec (1.5 s) had elapsed";
  EXPECT_TRUE(fake_state_.hasEvent("GotoPose", "canceled"))
    << "the sustained authority seize did not actually cancel the in-flight navigator goal";
  EXPECT_EQ(latestStatus().last_result_code, kResultAbortedNoAuthority);
}

// The mechanism behind the load-sensitive failures of this fixture, reproduced on
// purpose instead of waited for (2026-08-25). Silence the authority stream past the
// staleness rule and the mission must pause with NO_AUTHORITY -- that is what a
// starved rig does to every later claim in the run, and it is why each premise wait
// here reports worstAuthorityGap() when it fails.
TEST_F(MissionExecutorFixture, ASustainedAuthorityLapsePausesTheMission)
{
  goto_pose_server_->setHold(true);
  ASSERT_TRUE(sendExecuteMissionGoal("indoor_patrol"));
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("GotoPose", "accepted");}, 10.0))
    << "FAILED TO MEASURE: the mission never reached GotoPose"
    << "\n  navigator saw:" << fake_state_.describeEvents()
    << "\n  worst gap in this fixture's own authority stream: " << worstAuthorityGap() << " s";

  injectStreamStall(3.5);   // > 2 x heartbeat staleness (1.0 s) + grace (1.5 s)

  ASSERT_TRUE(waitFor([this]() {return latestStatus().state == MissionStatusMsg::STATE_PAUSED;}, 8.0))
    << "the executor did not react to a real authority lapse -- worst gap seen was "
    << worstAuthorityGap() << " s";
  EXPECT_EQ(latestStatus().last_result_code, kResultAbortedNoAuthority);
  // The gap only becomes visible once the stream publishes again: reading the log
  // while the thread is still asleep measured 0.02 s of a 3.5 s silence.
  EXPECT_TRUE(waitFor([this]() {return worstAuthorityGap() >= 1.0;}, 6.0))
    << "FAILED TO MEASURE: the injected stall never reached the wire, worst gap "
    << worstAuthorityGap() << " s";
}

// ---------------------------------------------------------------- (d)

TEST_F(MissionExecutorFixture, ResumeContinuesAtTheSameStepIndex)
{
  ASSERT_TRUE(sendExecuteMissionGoal("indoor_patrol"));
  ASSERT_TRUE(waitFor([this]() {return latestStatus().current_step_index >= 1;}, 15.0))
    << "FAILED TO MEASURE: no waypoint step ever completed before the pause";

  goto_pose_server_->setHold(true);
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("GotoPose", "accepted");}, 5.0));

  const uint32_t index_before_pause = latestStatus().current_step_index;
  ASSERT_TRUE(callPauseMission(""));
  ASSERT_TRUE(waitFor([this]() {return latestStatus().state == MissionStatusMsg::STATE_PAUSED;}, 5.0));
  EXPECT_EQ(latestStatus().current_step_index, index_before_pause)
    << "pausing itself must not advance/reset the step index";

  goto_pose_server_->setHold(false);
  ASSERT_TRUE(callResumeMission(""));
  ASSERT_TRUE(waitFor([this]() {return latestStatus().state == MissionStatusMsg::STATE_RUNNING;}, 5.0));

  ASSERT_TRUE(waitFor([this, index_before_pause]() {return latestStatus().current_step_index > index_before_pause;}, 20.0))
    << "FAILED TO MEASURE: current_step_index_ never advanced further after resume "
    << "(it must continue counting up, never reset to 0)";
}

// ---------------------------------------------------------------- (e)

TEST_F(MissionExecutorFixture, BatteryEndEarlyOnlyAfterAuthorityReturns)
{
  goto_pose_server_->setHold(true);
  ASSERT_TRUE(sendExecuteMissionGoal("indoor_patrol"));
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("GotoPose", "accepted");}, 10.0))
    << "FAILED TO MEASURE: the mission never reached GotoPose"
    << "\n  navigator saw:" << fake_state_.describeEvents()
    << "\n  worst gap in this fixture's own authority stream: " << worstAuthorityGap() << " s";

  // Both triggers armed at once. mission_policy checks authority (priority
  // 2) BEFORE battery (priority 3): the mission must PAUSE, never
  // end_early, while authority stays absent, no matter how long battery
  // has been below WARN.
  setAuthoritySource(kSourceOperator);
  setBattery(0.2);
  ASSERT_TRUE(waitFor([this]() {return latestStatus().state == MissionStatusMsg::STATE_PAUSED;}, 6.0));

  std::this_thread::sleep_for(4s);   // outlast battery_warn_dwell_sec (3.0 s) while still paused
  EXPECT_EQ(latestStatus().state, MissionStatusMsg::STATE_PAUSED)
    << "battery-driven end_early fired while authority was still absent";
  EXPECT_EQ(latestStatus().last_result_code, kResultAbortedNoAuthority)
    << "PAUSED reason must still read as the authority seize, not battery, while authority is absent";

  goto_pose_server_->setHold(false);   // let Finish's legs (GotoPose(home), Land) run free
  setAuthoritySource(kSourceMission);
  ASSERT_TRUE(callResumeMission(""));

  ASSERT_TRUE(
    waitFor(
      [this]() {
        const auto s = latestStatus();
        return s.state == MissionStatusMsg::STATE_ABORTED && s.last_result_code == kResultAbortedLowBattery;
      }, 15.0))
    << "FAILED TO MEASURE/FAIL: mission never ended ABORTED_LOW_BATTERY once authority returned";
}

// ---------------------------------------------------------------- (f)

TEST_F(MissionExecutorFixture, CancelRefusedDuringLandingDoesNotHangAndEndsOnTheRealResult)
{
  ASSERT_TRUE(sendExecuteMissionGoal("indoor_patrol"));

  land_server_->setRefuseCancel(true);
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("Land", "accepted");}, 30.0))
    << "FAILED TO MEASURE: mission never reached its Finish/Land leg";

  ASSERT_TRUE(callAbortMission("operator abort during landing"));

  ASSERT_TRUE(
    waitFor([this]() {return latestStatus().state == MissionStatusMsg::STATE_ABORTED;}, 10.0))
    << "FAILED TO MEASURE/FAIL: mission hung instead of settling once Land's real result arrived";
  EXPECT_TRUE(fake_state_.hasEvent("Land", "succeeded"))
    << "Land's cancel was refused (LANDING) -- it must still be allowed to finish for real, "
    << "not be force-ended";
}

// ---------------------------------------------------------------- (g)
// G-M1 gate bug (2026-08-22, gm1_probe.log): probe saw ExecuteMission
// SUCCEEDED (goal_status=4, result_code=SUCCEEDED, steps=8) yet
// /mission/status terminal sample was still STATE_RUNNING across all 162
// samples collected. Root cause (mission_executor_node.cpp): finishAsAborted()
// and finishAsCompleted() call settleGoalHandle() -- which sends the
// ExecuteMission action RESULT and resets goal_handle_ to nullptr -- BEFORE
// the caller (onTick()) ever publishes the terminal MissionStatus. Two
// observable, DETERMINISTIC (non-racy) consequences of that ordering, both
// pinned below without relying on wall-clock timing (R21: an actual client-
// side wire race was tried first here and discarded -- see plan/changelog
// 2026-08-22 -- because rclcpp entity-priority scheduling and in-process
// SHM-transport jitter made it too fast/unreliable to force in a fast
// synthetic test; these two properties hold on EVERY run regardless of
// timing, and pin the exact same root cause):
//   (1) msg.goal_id ("" if goal_handle_ is null) is read from goal_handle_
//       AFTER it was already reset -- so the terminal status ALWAYS ships
//       an empty goal_id, violating contract 2.19 ("goal_id ... rong khi
//       state == IDLE" implies non-empty for the settling goal otherwise).
//   (2) a late-joining TransientLocal subscriber -- the contract's whole
//       reason for existing -- must receive the terminal sample; that part
//       already works (this is the durability half of 2.19, not the
//       ordering half) and is pinned here as a regression guard.

TEST_F(MissionExecutorFixture, TerminalStatusGoalIdMatchesSettlingGoal_Completed)
{
  ASSERT_TRUE(sendExecuteMissionGoal("indoor_patrol"));
  const std::string expected_goal_id = uuidHex(goal_handle_->get_goal_id());

  ASSERT_TRUE(waitFor([this]() {return latestStatus().state == MissionStatusMsg::STATE_COMPLETED;}, 60.0))
    << "FAILED TO MEASURE: mission never reached STATE_COMPLETED";

  const MissionStatusMsg terminal = latestStatus();
  EXPECT_EQ(terminal.last_result_code, kResultSucceeded);
  EXPECT_FLOAT_EQ(terminal.progress_percent, 100.0f);
  EXPECT_FALSE(terminal.goal_id.empty())
    << "contract 2.19: goal_id is only empty when state == IDLE -- the terminal COMPLETED "
    << "sample must still carry the settling goal's UUID (it is published BEFORE "
    << "settleGoalHandle() resets goal_handle_, not after)";
  EXPECT_EQ(terminal.goal_id, expected_goal_id)
    << "terminal goal_id must be the SAME goal that just completed, not garbage";

  // (2) durability half of 2.19: a subscriber created AFTER the mission is
  // already done must still get the terminal sample immediately.
  auto late_node = std::make_shared<rclcpp::Node>("mission_late_join_probe_" + uav_id_);
  std::mutex late_mutex;
  std::vector<MissionStatusMsg> late_samples;
  auto late_sub = late_node->create_subscription<MissionStatusMsg>(
    prefix_ + "/mission/status", rclcpp::QoS(1).reliable().transient_local(),
    [&](MissionStatusMsg::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(late_mutex);
      late_samples.push_back(*msg);
    });
  executor_->add_node(late_node);   // fixture's spin_thread_ now drains it too
  ASSERT_TRUE(
    waitFor(
      [&]() {
        std::lock_guard<std::mutex> lock(late_mutex);
        return !late_samples.empty();
      }, 5.0))
    << "FAILED TO MEASURE: late-joining TransientLocal subscriber got nothing";
  {
    std::lock_guard<std::mutex> lock(late_mutex);
    EXPECT_EQ(late_samples.back().state, MissionStatusMsg::STATE_COMPLETED)
      << "late joiner must see the CURRENT (terminal) state immediately, contract 2.19 TransientLocal";
    EXPECT_EQ(late_samples.back().goal_id, expected_goal_id);
  }
  executor_->remove_node(late_node);
}

// ---------------------------------------------------------------- (g3)
// The third goal_id path, and the only deterministic one. Aborting while the mission is
// in its Finish/Land leg lands in onTick()'s kFinishGotoHome/kFinishLanding/kRecovering
// branch, where finishAsAborted() is followed by publishStatus(TRUE) -- forced, so it
// fires every time rather than on the 2 Hz beat. Before the settling_goal_id_ fix this
// shipped a terminal ABORTED sample with no goal_id on EVERY run of this path.
//
// The scenario is deliberately the same one CancelRefusedDuringLandingDoesNotHangAndEnds
// OnTheRealResult already drives; the only difference is that this one reads goal_id. The
// path was never untested, just unwatched.

TEST_F(MissionExecutorFixture, TerminalStatusGoalIdSurvivesOperatorAbortWhileLanding)
{
  ASSERT_TRUE(sendExecuteMissionGoal("indoor_patrol"));
  const std::string expected_goal_id = uuidHex(goal_handle_->get_goal_id());

  land_server_->setRefuseCancel(true);
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("Land", "accepted");}, 30.0))
    << "FAILED TO MEASURE: mission never reached its Finish/Land leg";

  ASSERT_TRUE(callAbortMission("operator abort during landing"));
  ASSERT_TRUE(
    waitFor([this]() {return latestStatus().state == MissionStatusMsg::STATE_ABORTED;}, 10.0))
    << "FAILED TO MEASURE: mission never settled after the operator abort";

  const MissionStatusMsg terminal = latestStatus();
  EXPECT_FALSE(terminal.goal_id.empty())
    << "contract 2.19: the terminal ABORTED sample lost its goal_id on the forced publish "
    << "that follows settleGoalHandle() (mission_executor_node.cpp:795)";
  EXPECT_EQ(terminal.goal_id, expected_goal_id)
    << "the durable sample must name the goal that actually aborted";
}

TEST_F(MissionExecutorFixture, TerminalStatusGoalIdMatchesSettlingGoal_Aborted)
{
  ASSERT_TRUE(sendExecuteMissionGoal("indoor_patrol"));
  const std::string expected_goal_id = uuidHex(goal_handle_->get_goal_id());

  ASSERT_TRUE(callAbortMission("goal_id regression probe: operator abort"));
  ASSERT_TRUE(waitFor([this]() {return latestStatus().state == MissionStatusMsg::STATE_ABORTED;}, 60.0))
    << "FAILED TO MEASURE: mission never reached STATE_ABORTED";

  const MissionStatusMsg terminal = latestStatus();
  EXPECT_EQ(terminal.last_result_code, kResultCanceled);
  EXPECT_FALSE(terminal.last_reason.empty()) << "last_reason must be filled when state == ABORTED";
  EXPECT_FALSE(terminal.goal_id.empty())
    << "contract 2.19: the terminal ABORTED sample must still carry the settling goal's UUID";
  EXPECT_EQ(terminal.goal_id, expected_goal_id)
    << "terminal goal_id must be the SAME goal that just aborted, not garbage";
}

// ---------------------------------------------------------------- (h)
// G-M2 gate bug (2026-08-22, gm2_bringup.log): inspect_point e2e aborted
// ABORTED_TIMEOUT at goto_over_marker after a spurious ~48 s Finish (fly
// home + land) detour. Root cause, pinned via an instrumented reproduction
// run (scripts/verify_mission.sh m2): inspect_point.xml's TWO <MarkerSeen>
// leaves never remapped their marker_x/marker_y OUTPUT ports to the
// blackboard (missing marker_x="{marker_x}" marker_y="{marker_y}"
// attributes) -- MarkerSeen::tick()'s setOutput() had nowhere shared to
// write, so goto_over_marker's x="{marker_x}" y="{marker_y}" input ports
// were never bound and NavAction::onStart()'s readPorts() genuinely failed.
// A SEPARATE static bug (mission_policy.cpp) compounded it: ABORTED_INVALID_
// GOAL was retryable at retry_count==0 (always true here, freshly reset per
// step), so evaluateStepFailure() returned kContinue, pending_terminal_
// verdict_ stayed empty, and tickBody()'s ambiguous catch-all fired instead
// of an immediate, correctly-labeled abort. Both are fixed; this exercises
// the REAL shipped inspect_point.xml (UAV_MISSION_CONFIG_DIR), not a
// synthetic tree, so it pins the mission-content fix directly.
TEST_F(MissionExecutorFixture, InspectPointReachesMarkerWithoutInvalidGoalAbort)
{
  setLandmarkVisible(2.0, 0.0, 7);   // matches inspect_point's default approach (2,0,2); marker_id default is -1 (any)
  ASSERT_TRUE(sendExecuteMissionGoal("inspect_point"));

  ASSERT_TRUE(
    waitFor(
      [this]() {
        const auto s = latestStatus();
        return s.state == MissionStatusMsg::STATE_COMPLETED || s.state == MissionStatusMsg::STATE_ABORTED;
      }, 30.0))
    << "FAILED TO MEASURE: inspect_point mission never reached a terminal state";

  const MissionStatusMsg terminal = latestStatus();
  EXPECT_EQ(terminal.state, MissionStatusMsg::STATE_COMPLETED)
    << "last_result_code=" << static_cast<int>(terminal.last_result_code)
    << " last_reason=" << terminal.last_reason;
  EXPECT_EQ(terminal.last_result_code, kResultSucceeded);
  EXPECT_NE(terminal.last_result_code, kResultAbortedInvalidGoal)
    << "goto_over_marker's x=\"{marker_x}\" y=\"{marker_y}\" ports were never bound "
    << "(inspect_point.xml <MarkerSeen> missing marker_x/marker_y output port remap)";
}

// ---------------------------------------------------------------- (i)
// G-M2 fallout (coordinator ruling 2026-08-22, same day as (h) above): every
// GotoPose goal mission_executor_node ever sends -- from a NavAction leaf
// AND from the Finish/GotoHome leg -- hardcoded orientation.w=1.0 (yaw 0
// absolute), regardless of the drone's actual heading. A near-zero-distance
// leg with the drone already facing away from yaw 0 turned into a
// gratuitous ~130 deg yaw maneuver that alone stretched a leg to 52.3 s
// (~3x a leg that only needed to translate). Fix: sendGotoPose()
// (mission_executor_node.cpp) now echoes the CURRENT odometry_fused yaw,
// covering every call site (NavAction's GotoPose ports, search legs, and
// tickFinishGotoHome()) through the single shared implementation.
TEST_F(MissionExecutorFixture, GotoPoseGoalsCarryCurrentYawNotZero)
{
  const double kYaw = M_PI / 2.0;   // 90 deg -- unambiguously distinct from the old hardcoded yaw 0
  setOdometryYaw(kYaw);
  ASSERT_TRUE(
    sendExecuteMissionGoal(
      "indoor_patrol", "waypoints:\n  - {x: 1.0, y: 0.0, z: 1.5}\nloops: 1\ndwell_sec: 0.1\n"))
    << "FAILED TO MEASURE: ExecuteMission goal was not accepted";

  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("GotoPose", "accepted");}, 10.0))
    << "FAILED TO MEASURE: the NavAction leg's GotoPose goal never reached the fake navigator";
  ASSERT_TRUE(
    waitFor(
      [this]() {
        const auto s = latestStatus();
        return s.state == MissionStatusMsg::STATE_COMPLETED || s.state == MissionStatusMsg::STATE_ABORTED;
      }, 30.0))
    << "FAILED TO MEASURE: mission never reached a terminal state (need the Finish/GotoHome leg too)";
  EXPECT_EQ(latestStatus().state, MissionStatusMsg::STATE_COMPLETED)
    << "last_result_code=" << static_cast<int>(latestStatus().last_result_code);

  std::vector<geometry_msgs::msg::Quaternion> orientations;
  {
    std::lock_guard<std::mutex> lock(fake_state_.mutex);
    orientations = fake_state_.goto_pose_orientations;
  }
  // >= 2: the patrol_leg's own GotoPose (NavAction) + Finish/GotoHome's
  // GotoPose (dispatched after the 1-loop body succeeds) -- both call sites
  // sendGotoPose() covers.
  ASSERT_GE(orientations.size(), 2u)
    << "FAILED TO MEASURE: expected >= 2 GotoPose goals (NavAction leg + Finish/GotoHome leg), got "
    << orientations.size();

  for (std::size_t i = 0; i < orientations.size(); ++i) {
    const double yaw = yawFromQuaternion(orientations[i]);
    EXPECT_NEAR(yaw, kYaw, 1e-6)
      << "GotoPose goal #" << i << " did not carry the current odometry yaw -- got quaternion z="
      << orientations[i].z << " w=" << orientations[i].w
      << " (yaw=" << yaw << "), a hardcoded yaw 0 would read exactly 0 here";
  }
}

// ---------------------------------------------------------------- (j)
// G-M3 gate bug (2026-08-22, gm3_bringup.log): mission_executor_node
// std::terminate()'d (SIGABRT) the FIRST time follow_target ever ran.
// populateBlackboard()'s bb->set("timeout_msec", static_cast<int>(...))
// disagreed with BT.CPP's built-in Timeout decorator, whose msec port is
// declared `unsigned` (decorators/timeout_node.h) -- BT::Blackboard::set()
// throws BT::LogicError on a port type change, and NOTHING caught it.
// indoor_patrol/inspect_point never tripped this because their only
// built-in-typed port, Repeat::num_cycles, happens to also be `int`. This
// test runs the REAL shipped follow_target.xml (UAV_MISSION_CONFIG_DIR,
// same pattern as InspectPointReachesMarkerWithoutInvalidGoalAbort above)
// end to end -- red on the unfixed cast (node dies before EVENT_STARTED's
// subscriber even finishes flushing), green after.
TEST_F(MissionExecutorFixture, FollowTargetMissionStartsWithoutCrashingTheNode)
{
  ASSERT_TRUE(sendExecuteMissionGoal("follow_target"))
    << "FAILED TO MEASURE: ExecuteMission goal was not accepted";
  ASSERT_TRUE(waitFor([this]() {return hasMissionEvent(MissionEventMsg::EVENT_STARTED);}, 10.0))
    << "FAILED TO MEASURE: EVENT_STARTED never arrived -- node likely crashed on goal accept";
  ASSERT_TRUE(waitFor([this]() {return latestStatus().state == MissionStatusMsg::STATE_RUNNING;}, 10.0))
    << "FAILED TO MEASURE: mission never reached STATE_RUNNING -- node likely crashed "
    << "(BT::LogicError from timeout_msec's type mismatch, uncaught, std::terminate())";
  EXPECT_TRUE(
    waitFor(
      [this]() {
        return fake_state_.hasEvent("GotoPose", "accepted") ||
        fake_state_.hasEvent("TrackTarget", "accepted");
      }, 10.0))
    << "FAILED TO MEASURE: neither the search leg's GotoPose nor a TrackTarget goal ever reached "
    << "the fake navigator -- node accepted the goal but the BT tree never actually ticked";
}

// ---------------------------------------------------------------- (k)
// R0 structural crash-isolation (coordinator ruling 2026-08-22, same day as
// (j)): a broken mission XML/params must NEVER take the whole node down --
// verified here with a DIFFERENT, deliberately injected port-type mismatch
// (not the one already fixed above), through a REAL MissionExecutorNode
// pointed at a scratch missions_dir. A crafted indoor_patrol.xml swaps its
// outer <Repeat num_cycles="{loops}"> for <Timeout msec="{loops}"> --
// Timeout::msec is `unsigned`, populateBlackboard() still sets "loops" as
// `int` (unrelated to today's follow_target fix), so this throws in
// EXACTLY the same place, on a DIFFERENT node/data. Proves: (1) the goal
// aborts cleanly with ABORTED_INTERNAL_ERROR carrying what(), (2) the node
// keeps serving requests, (3) a subsequent, valid mission on the SAME node
// still runs to STATE_COMPLETED -- true fault isolation, not a lucky crash.
TEST_F(MissionExecutorFixture, PopulateBlackboardThrowAbortsCleanlyAndNodeServesTheNextMission)
{
  ScratchMissionsDir scratch;
  scratch.overwrite(
    "indoor_patrol.xml",
    "<?xml version=\"1.0\"?>\n"
    "<root main_tree_to_execute=\"MainTree\">\n"
    "  <BehaviorTree ID=\"MainTree\">\n"
    "    <Timeout msec=\"{loops}\">\n"
    "      <Sequence name=\"patrol_leg\">\n"
    "        <WaypointCursor name=\"next_waypoint\" waypoints=\"{waypoints}\" "
    "waypoint_out=\"{leg_target}\"/>\n"
    "        <NavAction name=\"goto_waypoint\" nav_type=\"GotoPose\" target=\"{leg_target}\" "
    "frame_id=\"odom\" acceptance_radius=\"0.3\" max_speed=\"0.55\"/>\n"
    "        <Dwell name=\"dwell_at_waypoint\" duration_sec=\"{dwell_sec}\"/>\n"
    "      </Sequence>\n"
    "    </Timeout>\n"
    "  </BehaviorTree>\n"
    "</root>\n");

  swapMissionNode(scratch.path());
  ASSERT_TRUE(waitFor([this]() {return execute_client_->action_server_is_ready();}, 10.0))
    << "FAILED TO MEASURE: swapped-in node's ExecuteMission server never came up";
  // R21: anchor readiness on an EVENT, not a blind sleep -- the swapped
  // node's OWN initial-IDLE publishStatus(true) (its constructor) landing
  // on status_sub_ proves its pub/sub discovery is live end to end, so its
  // OTHER same-constructor subscriptions (incl. /state/health_px4, which
  // gates handleAccepted() BEFORE populateBlackboard() ever runs) are
  // likely live too -- action_server_is_ready() alone races this: it only
  // proves the ACTION entity, a different one, is discovered.
  const std::size_t statuses_before_swap = statuses().size();
  ASSERT_TRUE(waitFor([&]() {return statuses().size() > statuses_before_swap;}, 5.0))
    << "FAILED TO MEASURE: swapped-in node never published its own initial status";

  ASSERT_TRUE(sendExecuteMissionGoal("indoor_patrol"))
    << "FAILED TO MEASURE: the crafted-XML goal was not even ACCEPTED "
    << "(handleGoal() only checks mission_id/phase_, never parses the XML)";

  // NOTE: this failure is a "pre-start" gate (same family as the existing
  // battery-too-low and malformed-XML gates just above it in
  // handleAccepted()) -- mission_state_ never reaches STATE_RUNNING, so
  // mission/status is NEVER updated at all here (by design, matching those
  // two pre-existing gates). The only observable signal is the ACTION
  // RESULT itself, not the status topic -- checking latestStatus() here
  // would hang forever (caught mid-diagnosis, 2026-08-22: 10 s timeouts
  // with no crash, because the assertion was watching the wrong channel).
  auto result_future = execute_client_->async_get_result(goal_handle_);
  ASSERT_TRUE(waitFor([&]() {return result_future.wait_for(0s) == std::future_status::ready;}, 10.0))
    << "FAILED TO MEASURE: ExecuteMission result never arrived -- node may have crashed "
    << "(instead of the expected clean abort) or hung";
  const auto wrapped = result_future.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(wrapped.result != nullptr);
  EXPECT_FALSE(wrapped.result->success);
  EXPECT_EQ(wrapped.result->result_code, kResultAbortedInternalError)
    << "message=" << wrapped.result->message;
  EXPECT_NE(wrapped.result->message.find("populate"), std::string::npos)
    << "message should name the failing step (populateBlackboard) -- got: "
    << wrapped.result->message;

  // (3) node still alive AND still able to run a DIFFERENT, valid mission
  // to completion -- true isolation, not just "didn't crash yet".
  ASSERT_TRUE(sendExecuteMissionGoal("follow_target"))
    << "FAILED TO MEASURE: node did not accept a fresh goal after the crash-isolated one";
  ASSERT_TRUE(waitFor([this]() {return hasMissionEvent(MissionEventMsg::EVENT_STARTED);}, 10.0))
    << "FAILED TO MEASURE: node stopped responding after the isolated populateBlackboard() throw";
  EXPECT_TRUE(waitFor([this]() {return latestStatus().state == MissionStatusMsg::STATE_RUNNING;}, 10.0));
}

// ---------------------------------------------------------------- (l)
// G-M4.4 gate bug: tracker's own M/N ghost filter (3-of-5) cuts ghost
// tracks 67% but still lets enough through to reset target_lost_timeout's
// clock, because mission dispatched every TrackTarget goal with
// target_id=-1 ("any") -- navigator's OWN identity filter
// (targetSample(wanted_id), navigator_action_server_node.cpp:2597) is a
// no-op for wanted_id<0, so BOTH detection layers were defeated at once.
// Root cause was in mission, not navigator: TargetSeen never told NavAction
// which id it had actually acquired. Fix: TargetSeen writes track_id out,
// follow_target.xml rebinds NavAction's target_id port to read THAT
// instead of the raw -1 filter (bt_nodes.cpp, follow_target.xml).
// FakeTrackTargetServer simulates :2597's watchdog for real: unusable
// (invisible OR wrong id) for >= the goal's OWN target_lost_timeout aborts
// LOST_TARGET, exactly like the real navigator would.
TEST_F(MissionExecutorFixture, TrackTargetGoalAnchorsIdAndRebindsAfterReacquire)
{
  // RAII cleanup: no matter how this test exits (pass, an EXPECT failure
  // that keeps running, or an early ASSERT_TRUE return), make sure any
  // TrackTarget goal is settled before the fixture starts destroying
  // fake_state_/goal_handle_ right after. An orphaned FakeTrackTargetServer
  // watchdog thread still looping past that point is a genuine
  // use-after-free race (segfault / std::system_error, both hit while
  // writing this test, 2026-08-23) -- TearDown() does not wait for
  // fake-server threads, because every OTHER test's fake goals always
  // resolve on their own within the test body; this is the first one that
  // does not. Best-effort only, not itself an assertion under test. The
  // lambda captures [this] (the TEST_F-generated fixture subclass), so it
  // can reach protected callAbortMission()/latestStatus()/waitFor() the
  // same way the rest of this test body already does.
  ScopeGuard settle_on_exit(
    [this]() {
      callAbortMission("test cleanup: no orphaned navigator goal past this test");
      waitFor([this]() {return latestStatus().state != MissionStatusMsg::STATE_RUNNING;}, 10.0);
    });

  setWorldTarget(true, 7, 1.0, 0.0);   // a real target, track_id=7
  ASSERT_TRUE(sendExecuteMissionGoal("follow_target"))
    << "FAILED TO MEASURE: ExecuteMission goal was not accepted";

  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("TrackTarget", "accepted");}, 15.0))
    << "FAILED TO MEASURE: TrackTarget goal never reached the fake navigator";
  {
    std::lock_guard<std::mutex> lock(fake_state_.mutex);
    ASSERT_FALSE(fake_state_.track_target_ids.empty());
    EXPECT_EQ(fake_state_.track_target_ids.back(), 7)
      << "the dispatched TrackTarget goal must carry the id TargetSeen actually acquired (7), "
      << "never -1 ('any') -- target_id=-1 on the wire defeats both the tracker's M/N ghost "
      << "filter and navigator's own targetSample(wanted_id) id-pinning "
      << "(navigator_action_server_node.cpp:2597), letting a ghost track reset the "
      << "target_lost_timeout clock forever";
  }

  // Ghost swap: world flips to a DIFFERENT track_id (99) while the goal
  // anchored to 7 is still in flight -- an anchored goal must NOT accept
  // it. Real mechanism: unusable for >= target_lost_timeout (4.0 s,
  // hardcoded in follow_target.xml) -> navigator aborts LOST_TARGET. This
  // is the assertion that is RED on the unfixed code: with target_id=-1
  // actually dispatched, the fake's usable() check (wanted_id < 0 ...) is
  // permanently true while anything is visible, so this event NEVER fires
  // and the wait below times out.
  goto_pose_server_->setHold(true);   // hold whichever search leg dispatches, so it can be inspected below
  setWorldTarget(true, 99, 1.0, 0.0);
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("TrackTarget", "lost");}, 8.0))
    << "FAILED TO MEASURE/FAIL: goal anchored to track_id=7 never went LOST_TARGET despite the "
    << "world switching to a different id (99) for longer than target_lost_timeout -- on the "
    << "unfixed code (target_id=-1 actually dispatched) this NEVER happens, the ghost is "
    << "accepted forever and both detection layers stay defeated";

  // Stop offering the ghost once detected -- otherwise TargetSeen's own
  // target_id filter (-1 = any) would immediately re-acquire the SAME
  // ghost the moment track_when_seen resets (its own id-blindness is by
  // design, see README.md), racing the search leg this test wants to
  // inspect. mission_policy's generic terminal-failure retry budget
  // (max_step_retries=2, retry_backoff_sec=3.0, mission_policy.hpp) means
  // this specific anchored goal actually retries the SAME id (7) up to 3x
  // before the leaf itself finally reports FAILURE and the tree falls
  // through to search -- NavAction's retry path re-dispatches from its own
  // member state, never re-reading the blackboard, so the anchor holds
  // through every retry too (same architecture that anchors the ORIGINAL
  // dispatch -- onStart() reads ports exactly once).
  hideWorldTarget();
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("GotoPose", "accepted");}, 30.0))
    << "FAILED TO MEASURE: mission never fell through to the search branch after the anchored "
    << "goal's retry budget (>= 3 LOST_TARGET occurrences) was exhausted";
  // Anchor on the SEARCH dispatch, not the FIRST "lost" -- with a retry
  // budget > 1, NavAction's OWN retry redispatches (still id=7, still
  // "accepted") fire BETWEEN the first "lost" and the eventual fall-through
  // to search, so "first accepted after first lost" would false-positive
  // match one of those retries instead of the real post-reacquire goal.
  const uint64_t search_dispatch_seq = fake_state_.findFirstSeq("GotoPose", "accepted");

  // Re-acquire with a NEW real id (55, simulating the tracker assigning a
  // fresh id after reacquisition) while search is still in flight (held).
  setWorldTarget(true, 55, 2.0, 0.0);
  ASSERT_TRUE(
    waitFor(
      [this, search_dispatch_seq]() {
        return fake_state_.findFirstSeqAfter("TrackTarget", "accepted", search_dispatch_seq) != 0;
      },
      10.0))
    << "FAILED TO MEASURE: no second TrackTarget goal was ever dispatched after reacquiring "
    << "with a new id -- rebind-on-reacquire (README.md 'target_id (follow_target)') is broken";
  {
    std::lock_guard<std::mutex> lock(fake_state_.mutex);
    EXPECT_EQ(fake_state_.track_target_ids.back(), 55)
      << "after search re-acquires, the NEW goal must anchor to the id JUST seen (55), not the "
      << "stale 7 or the ghost 99 -- 'bam vat vua tim duoc sau search' is the designed rebind "
      << "semantics, not a bug";
  }
  goto_pose_server_->setHold(false);
  // settle_on_exit's destructor (RAII, declared at the top) settles the
  // still-RUNNING id=55 goal now, regardless of which assertion above (if
  // any) ended this test early.
}

// ---------------------------------------------------------------- (m)
// G-M4.4b (2026-08-23): plan P9 S:2.5 "search 4 diem (2 luot) -> Recover
// (CLIMB) -> abort". A real gate run (~/gate_logs/gm4d_full_run_final3.log)
// showed follow_target looping track<->search 5x over ~90s+ before the
// OUTER Timeout finally cut it off with the WRONG result code
// (ABORTED_TIMEOUT, not ABORTED_LOST_TARGET) -- a single-tick TargetSeen
// flicker re-enters track_when_seen and (via NavAction::onStart()'s
// unconditional resetStepRetryState()) wipes pending_terminal_verdict_/
// step_retry_count_, exactly the SAME "reset at step boundary" mechanism
// that is CORRECT for a genuine reacquisition, defeating a budget meant to
// survive across a mere flicker.
//
// PERMANENT regression guard: a scratch follow_target.xml identical to the
// shipped one MINUS <SearchBudgetAvailable/> reproduces the OLD symptom on
// the CURRENT (already-fixed) C++ machinery -- proving the fix lives in the
// XML wiring just as much as in the counter logic, and that removing the
// gate tag alone reopens the bug.
TEST_F(MissionExecutorFixture, SearchWithoutBudgetGateLoopsForeverToWrongResultCode)
{
  ScopeGuard settle_on_exit(
    [this]() {
      goto_pose_server_->setHold(false);
      callAbortMission("test cleanup: no orphaned navigator goal past this test");
      waitFor([this]() {return latestStatus().state != MissionStatusMsg::STATE_RUNNING;}, 10.0);
    });

  ScratchMissionsDir scratch;
  scratch.overwrite(
    "follow_target.xml",
    R"XML(<?xml version="1.0"?>
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Timeout msec="{timeout_msec}">
      <ReactiveFallback name="follow_or_search">
        <Sequence name="track_when_seen">
          <TargetSeen name="target_visible_check" target_id="{target_id}" track_id="{acquired_track_id}"/>
          <NavAction name="track_target" nav_type="TrackTarget" target_id="{acquired_track_id}"
                     standoff_distance="{standoff_m}" duration_seconds="0.0"
                     target_lost_timeout="0.5"/>
        </Sequence>
        <Sequence name="search_when_lost">
          <NavAction name="search_point_1" nav_type="GotoPose" x="{search_x1}" y="{search_y1}"
                     z="{search_z}" frame_id="odom" acceptance_radius="0.5" max_speed="0.4"/>
          <NavAction name="search_point_2" nav_type="GotoPose" x="{search_x2}" y="{search_y2}"
                     z="{search_z}" frame_id="odom" acceptance_radius="0.5" max_speed="0.4"/>
        </Sequence>
      </ReactiveFallback>
    </Timeout>
  </BehaviorTree>
</root>
)XML");

  swapMissionNode(
    scratch.path(),
    {rclcpp::Parameter("max_step_retries", 1), rclcpp::Parameter("retry_backoff_sec", 0.2)});
  ASSERT_TRUE(waitFor([this]() {return execute_client_->action_server_is_ready();}, 10.0))
    << "FAILED TO MEASURE: swapped-in node's ExecuteMission server never came up";
  const std::size_t statuses_before_swap = statuses().size();
  ASSERT_TRUE(waitFor([&]() {return statuses().size() > statuses_before_swap;}, 5.0))
    << "FAILED TO MEASURE: swapped-in node never published its own initial status";

  goto_pose_server_->setHold(true);   // search legs must never self-complete -- keeps the proof clean
  setWorldTarget(true, 1, 1.0, 0.0);
  ASSERT_TRUE(sendExecuteMissionGoal("follow_target", "timeout_sec: 10.0"))
    << "FAILED TO MEASURE: ExecuteMission goal was not accepted";
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("TrackTarget", "accepted");}, 10.0))
    << "FAILED TO MEASURE: initial TrackTarget goal never reached the fake navigator";

  hideWorldTarget();
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("GotoPose", "accepted");}, 10.0))
    << "FAILED TO MEASURE: search never got dispatched after the target went out of view";
  const uint64_t search1_seq = fake_state_.findFirstSeq("GotoPose", "accepted");

  // A momentary flicker (a ghost, one tick true) still interrupts search on
  // this unpatched XML -- same reactive mechanism the fix relies on, just
  // with nothing left to eventually stop it.
  setWorldTarget(true, 2, 1.0, 0.0);
  ASSERT_TRUE(
    waitFor(
      [this, search1_seq]() {
        return fake_state_.findFirstSeqAfter("TrackTarget", "accepted", search1_seq) != 0;
      }, 8.0))
    << "FAILED TO MEASURE: flicker never re-dispatched TrackTarget";
  hideWorldTarget();
  ASSERT_TRUE(
    waitFor(
      [this, search1_seq]() {
        return fake_state_.findFirstSeqAfter("GotoPose", "accepted", search1_seq) != 0;
      }, 10.0))
    << "FAILED TO MEASURE: search never resumed after the flicker-triggered TrackTarget detour";
  const uint64_t search2_seq = fake_state_.findFirstSeqAfter("GotoPose", "accepted", search1_seq);

  // A second flicker -- this is the one search_attempts_max=2 should have
  // refused, had this XML carried the gate.
  setWorldTarget(true, 3, 1.0, 0.0);
  ASSERT_TRUE(
    waitFor(
      [this, search2_seq]() {
        return fake_state_.findFirstSeqAfter("TrackTarget", "accepted", search2_seq) != 0;
      }, 8.0))
    << "FAILED TO MEASURE: second flicker never re-dispatched TrackTarget";
  hideWorldTarget();
  ASSERT_TRUE(
    waitFor(
      [this, search2_seq]() {
        return fake_state_.findFirstSeqAfter("GotoPose", "accepted", search2_seq) != 0;
      }, 10.0))
    << "FAILED TO MEASURE: search never resumed a THIRD time -- exceeds what search_attempts_max=2 "
    << "should ever allow, proving this unpatched XML has no budget enforcement at all";
  const uint64_t search3_seq = fake_state_.findFirstSeqAfter("GotoPose", "accepted", search2_seq);

  // No gate exists in this XML -- nothing left to ever trigger Recover. Let
  // the OUTER Timeout (10s) be the only thing that ends this mission. Stay
  // held through this wait -- releasing the hold here would let search3
  // COMPLETE (both waypoints "reached") instead of being cut off by the
  // Timeout, which (via ReactiveFallback's own OR semantics) reports the
  // whole tree SUCCESS instead, masking the very symptom under test.
  // Timeout halting the tree cancels this in-flight leg regardless of
  // hold_ (FakeGotoPoseServer checks is_canceling() even while held) --
  // that cancellation is the signal the Timeout actually fired.
  ASSERT_TRUE(
    waitFor(
      [this, search3_seq]() {
        return fake_state_.findFirstSeqAfter("GotoPose", "canceled", search3_seq) != 0;
      }, 15.0))
    << "FAILED TO MEASURE: the outer Timeout never halted the still-running search leg";
  // Finish's OWN separate GotoPose-home leg (dispatched fresh now that the
  // tree failed) is a NEW goal with no cancellation racing it -- it would
  // sit held forever if hold_ were still true.
  goto_pose_server_->setHold(false);
  ASSERT_TRUE(
    waitFor([this]() {return latestStatus().state != MissionStatusMsg::STATE_RUNNING;}, 15.0))
    << "FAILED TO MEASURE: mission never reached a terminal state at all";
  EXPECT_EQ(latestStatus().state, MissionStatusMsg::STATE_ABORTED);
  EXPECT_EQ(latestStatus().last_result_code, kResultAbortedTimeout)
    << "unpatched XML has no budget enforcement, so the ONLY thing that ever ends this mission is "
    << "the outer Timeout -- the WRONG result code (should be ABORTED_LOST_TARGET on the real fix), "
    << "got reason=" << latestStatus().last_reason;
  EXPECT_FALSE(fake_state_.hasEvent("Recover", "accepted"))
    << "Recover must never be dispatched on this unpatched XML -- nothing in it can trigger it";
}

// ---------------------------------------------------------------- (n)
// The fix, exercised end to end: exactly search_attempts_max (2) search
// episodes survive repeated single-tick flickers, then the THIRD loss
// episode routes to Recover(CLIMB) instead of a 3rd search dispatch, ending
// ABORTED_LOST_TARGET well before the outer Timeout (12s, generously
// unreached) -- the mirror image of test (m) above, same flicker choreography,
// only difference is the shipped follow_target.xml structure (gate present).
TEST_F(MissionExecutorFixture, FollowTargetExhaustsSearchBudgetThenRecoversAndAbortsLostTarget)
{
  ScopeGuard settle_on_exit(
    [this]() {
      goto_pose_server_->setHold(false);
      callAbortMission("test cleanup: no orphaned navigator goal past this test");
      waitFor([this]() {return latestStatus().state != MissionStatusMsg::STATE_RUNNING;}, 10.0);
    });

  ScratchMissionsDir scratch;
  scratch.overwrite(
    "follow_target.xml",
    R"XML(<?xml version="1.0"?>
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Timeout msec="{timeout_msec}">
      <ReactiveFallback name="follow_or_search">
        <Sequence name="track_when_seen">
          <TargetSeen name="target_visible_check" target_id="{target_id}" track_id="{acquired_track_id}"/>
          <NavAction name="track_target" nav_type="TrackTarget" target_id="{acquired_track_id}"
                     standoff_distance="{standoff_m}" duration_seconds="0.0"
                     target_lost_timeout="0.5"/>
        </Sequence>
        <Sequence name="search_when_lost">
          <SearchBudgetAvailable name="search_budget_check"/>
          <NavAction name="search_point_1" nav_type="GotoPose" x="{search_x1}" y="{search_y1}"
                     z="{search_z}" frame_id="odom" acceptance_radius="0.5" max_speed="0.4"/>
          <NavAction name="search_point_2" nav_type="GotoPose" x="{search_x2}" y="{search_y2}"
                     z="{search_z}" frame_id="odom" acceptance_radius="0.5" max_speed="0.4"/>
        </Sequence>
      </ReactiveFallback>
    </Timeout>
  </BehaviorTree>
</root>
)XML");

  swapMissionNode(
    scratch.path(),
    {rclcpp::Parameter("max_step_retries", 1), rclcpp::Parameter("retry_backoff_sec", 0.2)});
  ASSERT_TRUE(waitFor([this]() {return execute_client_->action_server_is_ready();}, 10.0))
    << "FAILED TO MEASURE: swapped-in node's ExecuteMission server never came up";
  const std::size_t statuses_before_swap = statuses().size();
  ASSERT_TRUE(waitFor([&]() {return statuses().size() > statuses_before_swap;}, 5.0))
    << "FAILED TO MEASURE: swapped-in node never published its own initial status";

  goto_pose_server_->setHold(true);
  setWorldTarget(true, 1, 1.0, 0.0);
  ASSERT_TRUE(sendExecuteMissionGoal("follow_target", "timeout_sec: 12.0"))
    << "FAILED TO MEASURE: ExecuteMission goal was not accepted";
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("TrackTarget", "accepted");}, 10.0))
    << "FAILED TO MEASURE: initial TrackTarget goal never reached the fake navigator";

  hideWorldTarget();
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("GotoPose", "accepted");}, 10.0))
    << "FAILED TO MEASURE: search episode #1 never got dispatched -- search_attempts_max=2 should "
    << "still allow it";
  const uint64_t search1_seq = fake_state_.findFirstSeq("GotoPose", "accepted");

  setWorldTarget(true, 2, 1.0, 0.0);   // flicker: a ghost, one tick true, must NOT spend the budget
  ASSERT_TRUE(
    waitFor(
      [this, search1_seq]() {
        return fake_state_.findFirstSeqAfter("TrackTarget", "accepted", search1_seq) != 0;
      }, 8.0))
    << "FAILED TO MEASURE: flicker never re-dispatched TrackTarget";
  hideWorldTarget();
  ASSERT_TRUE(
    waitFor(
      [this, search1_seq]() {
        return fake_state_.findFirstSeqAfter("GotoPose", "accepted", search1_seq) != 0;
      }, 10.0))
    << "FAILED TO MEASURE: search episode #2 never got dispatched -- search_attempts_max=2 should "
    << "still allow it";
  const uint64_t search2_seq = fake_state_.findFirstSeqAfter("GotoPose", "accepted", search1_seq);

  setWorldTarget(true, 3, 1.0, 0.0);   // second flicker -- must NOT buy a 3rd episode
  ASSERT_TRUE(
    waitFor(
      [this, search2_seq]() {
        return fake_state_.findFirstSeqAfter("TrackTarget", "accepted", search2_seq) != 0;
      }, 8.0))
    << "FAILED TO MEASURE: second flicker never re-dispatched TrackTarget";
  hideWorldTarget();

  // Budget is now exhausted (2 search episodes already spent) -- this THIRD
  // loss episode must route to Recover(CLIMB) instead of a 3rd search
  // dispatch.
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("Recover", "accepted");}, 10.0))
    << "FAILED TO MEASURE: Recover(CLIMB) was never dispatched once the search-attempts budget "
    << "(search_attempts_max=2) was exhausted";
  EXPECT_EQ(fake_state_.findFirstSeqAfter("GotoPose", "accepted", search2_seq), 0u)
    << "a THIRD search episode must never be dispatched once the budget is exhausted -- this is "
    << "exactly the flicker-defeats-the-counter bug (2026-08-23 gate run, ~90s+ of looping ending "
    << "in the wrong result code) this fix exists to close";

  ASSERT_TRUE(
    waitFor([this]() {return latestStatus().state != MissionStatusMsg::STATE_RUNNING;}, 10.0))
    << "FAILED TO MEASURE: mission never reached a terminal state after Recover";
  EXPECT_EQ(latestStatus().state, MissionStatusMsg::STATE_ABORTED);
  EXPECT_EQ(latestStatus().last_result_code, kResultAbortedLostTarget)
    << "the terminal result must reflect the ROOT CAUSE (search budget exhausted while tracking a "
    << "lost target), never ABORTED_TIMEOUT -- got reason=" << latestStatus().last_reason;
}

// ---------------------------------------------------------------- (o)
// Requirement (b): a GENUINE loss followed by a GENUINE reacquisition
// (tracking sustained past track_progress_reset_sec, not a single-tick
// flicker) must reset the search-attempts budget -- the mission must NOT
// abort early the next time it loses the target, it gets its full 2
// episodes again.
TEST_F(MissionExecutorFixture, FollowTargetSustainedReacquisitionResetsSearchBudget)
{
  ScopeGuard settle_on_exit(
    [this]() {
      goto_pose_server_->setHold(false);
      callAbortMission("test cleanup: no orphaned navigator goal past this test");
      waitFor([this]() {return latestStatus().state != MissionStatusMsg::STATE_RUNNING;}, 10.0);
    });

  ScratchMissionsDir scratch;
  scratch.overwrite(
    "follow_target.xml",
    R"XML(<?xml version="1.0"?>
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Timeout msec="{timeout_msec}">
      <ReactiveFallback name="follow_or_search">
        <Sequence name="track_when_seen">
          <TargetSeen name="target_visible_check" target_id="{target_id}" track_id="{acquired_track_id}"/>
          <NavAction name="track_target" nav_type="TrackTarget" target_id="{acquired_track_id}"
                     standoff_distance="{standoff_m}" duration_seconds="0.0"
                     target_lost_timeout="0.5"/>
        </Sequence>
        <Sequence name="search_when_lost">
          <SearchBudgetAvailable name="search_budget_check"/>
          <NavAction name="search_point_1" nav_type="GotoPose" x="{search_x1}" y="{search_y1}"
                     z="{search_z}" frame_id="odom" acceptance_radius="0.5" max_speed="0.4"/>
          <NavAction name="search_point_2" nav_type="GotoPose" x="{search_x2}" y="{search_y2}"
                     z="{search_z}" frame_id="odom" acceptance_radius="0.5" max_speed="0.4"/>
        </Sequence>
      </ReactiveFallback>
    </Timeout>
  </BehaviorTree>
</root>
)XML");

  swapMissionNode(
    scratch.path(),
    {rclcpp::Parameter("max_step_retries", 1), rclcpp::Parameter("retry_backoff_sec", 0.2),
      rclcpp::Parameter("track_progress_reset_sec", 0.6)});
  ASSERT_TRUE(waitFor([this]() {return execute_client_->action_server_is_ready();}, 10.0))
    << "FAILED TO MEASURE: swapped-in node's ExecuteMission server never came up";
  const std::size_t statuses_before_swap = statuses().size();
  ASSERT_TRUE(waitFor([&]() {return statuses().size() > statuses_before_swap;}, 5.0))
    << "FAILED TO MEASURE: swapped-in node never published its own initial status";

  goto_pose_server_->setHold(true);
  setWorldTarget(true, 1, 1.0, 0.0);
  ASSERT_TRUE(sendExecuteMissionGoal("follow_target", "timeout_sec: 20.0"))
    << "FAILED TO MEASURE: ExecuteMission goal was not accepted";
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("TrackTarget", "accepted");}, 10.0))
    << "FAILED TO MEASURE: initial TrackTarget goal never reached the fake navigator";

  // Spend episode #1 of the budget.
  hideWorldTarget();
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("GotoPose", "accepted");}, 10.0))
    << "FAILED TO MEASURE: search episode #1 never got dispatched";
  const uint64_t search1_seq = fake_state_.findFirstSeq("GotoPose", "accepted");

  // GENUINE reacquisition: track_id=2, held visible continuously (not a
  // flicker) well past track_progress_reset_sec (0.6s here).
  setWorldTarget(true, 2, 1.0, 0.0);
  ASSERT_TRUE(
    waitFor(
      [this, search1_seq]() {
        return fake_state_.findFirstSeqAfter("TrackTarget", "accepted", search1_seq) != 0;
      }, 8.0))
    << "FAILED TO MEASURE: reacquisition never re-dispatched TrackTarget";
  std::this_thread::sleep_for(900ms);   // > track_progress_reset_sec (0.6s): budget must reset to 0

  // Lose it again for real -- if the reset above worked, this is budget
  // slot #1 of a FRESH 2, not slot #3 of the ORIGINAL 2 (which would abort
  // prematurely).
  hideWorldTarget();
  ASSERT_TRUE(
    waitFor(
      [this, search1_seq]() {
        return fake_state_.findFirstSeqAfter("GotoPose", "accepted", search1_seq) != 0;
      }, 10.0))
    << "FAILED TO MEASURE: search never resumed after the genuine second loss";
  const uint64_t search2_seq = fake_state_.findFirstSeqAfter("GotoPose", "accepted", search1_seq);

  // A single-tick flicker to id=3, then a THIRD search dispatch -- only
  // possible if the reset above genuinely freed up a second full slot.
  setWorldTarget(true, 3, 1.0, 0.0);
  ASSERT_TRUE(
    waitFor(
      [this, search2_seq]() {
        return fake_state_.findFirstSeqAfter("TrackTarget", "accepted", search2_seq) != 0;
      }, 8.0))
    << "FAILED TO MEASURE: flicker never re-dispatched TrackTarget";
  hideWorldTarget();
  ASSERT_TRUE(
    waitFor(
      [this, search2_seq]() {
        return fake_state_.findFirstSeqAfter("GotoPose", "accepted", search2_seq) != 0;
      }, 10.0))
    << "FAILED TO MEASURE: search episode #2 of the RESET budget never got dispatched -- the "
    << "budget was not actually reset by the sustained reacquisition above";

  // Must still be RUNNING -- no premature Recover/abort from a budget that
  // never actually reset.
  EXPECT_FALSE(fake_state_.hasEvent("Recover", "accepted"))
    << "Recover must not fire yet -- the sustained reacquisition should have bought 2 fresh "
    << "episodes, and only the second of those has been spent so far";
  EXPECT_EQ(latestStatus().state, MissionStatusMsg::STATE_RUNNING)
    << "mission ended early -- the search-attempts budget was not reset by the sustained "
    << "reacquisition, got last_result_code=" << static_cast<int>(latestStatus().last_result_code)
    << " reason=" << latestStatus().last_reason;
}

// ---------------------------------------------------------------- (p)
// bug #12 (2026-08-23, coordinator report): a REAL G-M4.4 gate run with a
// COMPLETELY clean arena (no target ever existed) reported mission
// SUCCEEDED after 66.1 s -- search_when_lost = Sequence[SearchBudgetAvail-
// able, search1, search2] had no reacquire confirmation, so reaching both
// waypoints (navigation success) was silently read as "target found".
// This is the mirror case of test (n)/(o) above: THOSE always went through
// a real TrackTarget dispatch first (a target existed, then vanished);
// THIS one never dispatches TrackTarget at all (TargetSeen fails from the
// very first tick), which the ORIGINAL G-M4.4b counter hook (tied to
// TrackTarget's own terminal failure) could never observe or bound. Uses
// the REAL shipped follow_target.xml (UAV_MISSION_CONFIG_DIR) -- this is
// the money test for bug #12's fix.
TEST_F(MissionExecutorFixture, FollowTargetNeverAcquiredExhaustsSearchThenRecoversAndAbortsLostTarget)
{
  hideWorldTarget();   // arena hoan toan sach -- target never exists at all
  ASSERT_TRUE(sendExecuteMissionGoal("follow_target"))
    << "FAILED TO MEASURE: ExecuteMission goal was not accepted";

  // search_attempts_max=2 (node default) -- exactly 2 full rounds (4
  // GotoPose dispatches: search1+search2 twice) must run before Recover.
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("Recover", "accepted");}, 15.0))
    << "FAILED TO MEASURE: Recover(CLIMB) was never dispatched -- either the mission wrongly "
    << "reported SUCCESS (bug #12's exact symptom) or it never converged at all";
  EXPECT_FALSE(fake_state_.hasEvent("TrackTarget", "accepted"))
    << "TrackTarget must NEVER be dispatched in this scenario -- the target was never visible "
    << "even once, so track_when_seen's TargetSeen must fail before ever reaching NavAction";
  {
    std::lock_guard<std::mutex> lock(fake_state_.mutex);
    int goto_pose_accepted = 0;
    for (const auto & e : fake_state_.events) {
      if (e.action == "GotoPose" && e.kind == "accepted") {
        ++goto_pose_accepted;
      }
    }
    EXPECT_EQ(goto_pose_accepted, 4)
      << "expected exactly 2 search ROUNDS (search_point_1+search_point_2 each) before the "
      << "budget-exhausted 3rd entry is blocked BEFORE dispatching anything -- got "
      << goto_pose_accepted << " GotoPose dispatches total";
  }

  ASSERT_TRUE(
    waitFor([this]() {return latestStatus().state != MissionStatusMsg::STATE_RUNNING;}, 10.0))
    << "FAILED TO MEASURE: mission never reached a terminal state after Recover";
  EXPECT_EQ(latestStatus().state, MissionStatusMsg::STATE_ABORTED);
  EXPECT_EQ(latestStatus().last_result_code, kResultAbortedLostTarget)
    << "bug #12: the terminal result must be ABORTED_LOST_TARGET (the root cause), never "
    << "SUCCEEDED (the original symptom) or ABORTED_TIMEOUT -- got reason="
    << latestStatus().last_reason;

  // Never once reported COMPLETED anywhere in its whole history -- the
  // exact symptom (mission SUCCEEDED 66.1s "mission completed") must never
  // occur even transiently.
  for (const MissionStatusMsg & s : statuses()) {
    EXPECT_NE(s.state, MissionStatusMsg::STATE_COMPLETED)
      << "bug #12 regression: mission reported STATE_COMPLETED at some point despite the target "
      << "never once being visible";
  }
}

// ---------------------------------------------------------------- (q)
// Requirement (b) of bug #12: a GENUINE reacquisition DURING a search round
// (target held visible, not a single-tick flicker) must rebind
// {acquired_track_id} to whatever is ACTUALLY visible and resume TRACKING,
// while search_point_2 is still in flight -- never read as mission SUCCESS.
// Mechanistically this resolves through track_when_seen's OWN TargetSeen,
// not search_when_lost's recheck: ReactiveFallback re-ticks track_when_seen
// from index 0 on EVERY external tick, and both leaves read the SAME
// state_mutex_-protected world snapshot, so track_when_seen always sees a
// reacquisition first and wins the race, canceling search_point_2 before
// the recheck itself ever gets a chance to tick -- the recheck's own
// SUCCESS path (tickBody()'s kFollowTarget retry-on-SUCCESS branch,
// mission_executor_node.cpp) is a defensive backstop for a tree shape this
// one does not exercise, not something an integration test can force
// through THIS specific tree without changing it. What this test DOES
// prove, and must not regress: a genuine (sustained) reacquisition mid-
// search never gets misread as mission SUCCESS, and correctly rebinds +
// resumes tracking -- same outcome the coordinator asked for, reached via
// track_when_seen rather than the recheck specifically.
TEST_F(MissionExecutorFixture, FollowTargetGenuineReacquireDuringSearchRebindsAndResumesTracking)
{
  ScopeGuard settle_on_exit(
    [this]() {
      goto_pose_server_->setHold(false);
      callAbortMission("test cleanup: no orphaned navigator goal past this test");
      waitFor([this]() {return latestStatus().state != MissionStatusMsg::STATE_RUNNING;}, 10.0);
    });

  goto_pose_server_->setHold(true);
  hideWorldTarget();
  ASSERT_TRUE(sendExecuteMissionGoal("follow_target"))
    << "FAILED TO MEASURE: ExecuteMission goal was not accepted";

  // search_point_1: release just long enough for it to complete, then
  // re-hold BEFORE search_point_2 can also complete unattended -- the test
  // needs precise control over exactly when the target becomes visible
  // relative to search_point_2, not search_point_1.
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("GotoPose", "accepted");}, 10.0))
    << "FAILED TO MEASURE: search_point_1 never got dispatched";
  const uint64_t sp1_accepted_seq = fake_state_.findFirstSeq("GotoPose", "accepted");
  goto_pose_server_->setHold(false);
  ASSERT_TRUE(
    waitFor(
      [this, sp1_accepted_seq]() {
        return fake_state_.findFirstSeqAfter("GotoPose", "succeeded", sp1_accepted_seq) != 0;
      }, 10.0))
    << "FAILED TO MEASURE: search_point_1 never completed";
  goto_pose_server_->setHold(true);   // re-arm BEFORE search_point_2 can slip through unheld

  ASSERT_TRUE(
    waitFor(
      [this, sp1_accepted_seq]() {
        return fake_state_.findFirstSeqAfter("GotoPose", "accepted", sp1_accepted_seq) != 0;
      }, 10.0))
    << "FAILED TO MEASURE: search_point_2 never got dispatched";

  // Target genuinely reappears -- held visible (not a single-tick flicker)
  // -- while search_point_2 is held/RUNNING. On the NEXT external tick,
  // ReactiveFallback re-ticks track_when_seen FIRST and it wins the race,
  // canceling search_point_2 (same mechanism as test (n)'s reacquire, just
  // triggered mid-search-round instead of mid-track).
  setWorldTarget(true, 42, 3.0, 0.0);
  goto_pose_server_->setHold(false);   // let search_point_2 resolve either way (succeed or canceled)

  ASSERT_TRUE(
    waitFor(
      [this]() {return fake_state_.hasEvent("TrackTarget", "accepted");}, 10.0))
    << "FAILED TO MEASURE: the reacquisition never led to a fresh TrackTarget dispatch";
  {
    std::lock_guard<std::mutex> lock(fake_state_.mutex);
    ASSERT_FALSE(fake_state_.track_target_ids.empty());
    EXPECT_EQ(fake_state_.track_target_ids.back(), 42)
      << "the resumed TrackTarget goal must anchor to the id ACTUALLY seen (42), same "
      << "rebind-on-reacquire semantics as target_visible_check (G-M4.4)";
  }

  // bug #12's symptom, checked here too: reaching search_when_lost's
  // waypoints/SUCCESS must NEVER read as mission SUCCESS, even when a
  // reacquisition happens to land mid-round.
  EXPECT_EQ(latestStatus().state, MissionStatusMsg::STATE_RUNNING)
    << "mission must still be RUNNING (tracking again), not COMPLETED, after a genuine "
    << "reacquisition during search -- got last_result_code="
    << static_cast<int>(latestStatus().last_result_code) << " reason=" << latestStatus().last_reason;
  for (const MissionStatusMsg & s : statuses()) {
    EXPECT_NE(s.state, MissionStatusMsg::STATE_COMPLETED)
      << "bug #12 regression: a genuine reacquisition during search must never be read as "
      << "mission SUCCESS";
  }
}

// ---------------------------------------------------------------- (r)
// B1 (2026-08-23, review round 1): authority-seize during Finish must
// still cancel + PAUSE (priority 2 is an EDGE condition, the livelock
// suppression only ever applied to LEVEL conditions like battery). Drives
// the mission into kFinishGotoHome via the SAME battery-END_EARLY path
// test (e) (BatteryEndEarlyOnlyAfterAuthorityReturns) already uses, then
// seizes authority WHILE the (held) GotoPose(home) leg is in flight.
TEST_F(MissionExecutorFixture, AuthoritySeizeDuringFinishCancelsPausesAndResumesTheSameLeg)
{
  goto_pose_server_->setHold(true);
  ASSERT_TRUE(sendExecuteMissionGoal("indoor_patrol"))
    << "FAILED TO MEASURE: ExecuteMission goal was not accepted";
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("GotoPose", "accepted");}, 10.0))
    << "FAILED TO MEASURE: first body GotoPose never reached the fake navigator";

  // Battery END_EARLY -> beginFinish(ABORTED_LOW_BATTERY) -> kFinishGotoHome,
  // whose OWN GotoPose(home) leg dispatches next (still held).
  setBattery(0.2);
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("GotoPose", "canceled");}, 8.0))
    << "FAILED TO MEASURE: battery END_EARLY never canceled the body leg";
  const uint64_t body_cancel_seq = fake_state_.findFirstSeq("GotoPose", "canceled");
  ASSERT_TRUE(
    waitFor(
      [this, body_cancel_seq]() {
        return fake_state_.findFirstSeqAfter("GotoPose", "accepted", body_cancel_seq) != 0;
      }, 8.0))
    << "FAILED TO MEASURE: Finish's own GotoPose(home) leg never got dispatched";
  const uint64_t finish_dispatch_seq =
    fake_state_.findFirstSeqAfter("GotoPose", "accepted", body_cancel_seq);

  // Authority seized WHILE stuck mid-Finish (held) -- this is the exact
  // scenario B1 fixes: previously this priority-2 condition was suppressed
  // by the same block that (correctly) suppresses battery/pause here.
  setAuthoritySource(kSourceOperator);
  ASSERT_TRUE(waitFor([this]() {return latestStatus().state == MissionStatusMsg::STATE_PAUSED;}, 6.0))
    << "FAILED TO MEASURE: authority-seize during Finish never paused the mission -- B1 regression "
    << "(the livelock-suppression block is swallowing priority 2 again)";
  EXPECT_TRUE(fake_state_.findFirstSeqAfter("GotoPose", "canceled", finish_dispatch_seq) != 0)
    << "the held Finish leg was never actually canceled once authority was seized";
  EXPECT_EQ(latestStatus().last_result_code, kResultAbortedNoAuthority)
    << "PAUSED reason must read as the authority seize, same code AuthoritySeizeSustainedCancels"
    << "AndPauses(test (c)) already pins for a kBody pause";

  // Resume must continue INTO Finish (redispatch GotoPose(home)/Land), not
  // restart the body -- proven end to end: the mission must still end
  // ABORTED_LOW_BATTERY (the ORIGINAL Finish reason, from BEFORE the
  // authority-seize interruption), never something else.
  setAuthoritySource(kSourceMission);
  ASSERT_TRUE(callResumeMission(""));
  ASSERT_TRUE(waitFor([this]() {return latestStatus().state == MissionStatusMsg::STATE_RUNNING;}, 5.0))
    << "FAILED TO MEASURE: ResumeMission never took effect";
  goto_pose_server_->setHold(false);   // let Finish's legs (GotoPose(home), Land) finally run free

  ASSERT_TRUE(
    waitFor(
      [this]() {
        const auto s = latestStatus();
        return s.state == MissionStatusMsg::STATE_ABORTED &&
        s.last_result_code == kResultAbortedLowBattery;
      }, 15.0))
    << "FAILED TO MEASURE/FAIL: mission never ended ABORTED_LOW_BATTERY once resumed -- the "
    << "ORIGINAL Finish reason (pending_termination_) was lost across the authority-seize pause, "
    << "got last_result_code=" << static_cast<int>(latestStatus().last_result_code)
    << " reason=" << latestStatus().last_reason;
}

// ---------------------------------------------------------------- (s)
// B2 (2026-08-23, review round 1): a landmark entry's OWN time_since_seen_
// sec never advances again once world_model dies (the LAST message is
// simply replayed forever by latestLandmark() without a message-level
// arrival check) -- inspect_point would incorrectly COMPLETE on dead data.
// Stops the whole world stream (odometry/authority/health/localization/
// landmarks/target all freeze at once -- the simplest reliable way to
// simulate "world_model died" with this fixture) after a marker has
// already been seen once, then confirms inspect_point does NOT complete.
TEST_F(MissionExecutorFixture, DeadWorldModelStreamMakesInspectPointNeverComplete)
{
  setLandmarkVisible(2.0, 0.0);
  ASSERT_TRUE(waitFor([this]() {return !statuses().empty();}, 5.0));
  std::this_thread::sleep_for(100ms);   // let at least one landmarks message with it actually land

  stream_stop_.store(true);
  if (stream_thread_.joinable()) {
    stream_thread_.join();
  }
  // last_landmarks_ is now frozen forever, marker_id=-1 still "visible" per
  // its own time_since_seen_sec=0.0f (which never advances again either) --
  // ONLY the B2 arrival-of-the-MESSAGE check can catch this. Keep
  // authority/health/localization ALIVE on a separate thread -- otherwise
  // the pre-existing battery-unmeasured start gate (or authority/
  // localization staleness) would reject/pause the mission for an
  // unrelated reason before this scenario ever gets to run.
  std::atomic<bool> keep_alive_stop{false};
  std::thread keep_alive_thread([this, &keep_alive_stop]() {
    while (!keep_alive_stop.load()) {
      publishFreshEverythingExceptLandmarksAndTarget();
      std::this_thread::sleep_for(20ms);
    }
  });
  std::this_thread::sleep_for(100ms);   // let a few keep-alive messages land before the goal

  ASSERT_TRUE(sendExecuteMissionGoal("inspect_point"))
    << "FAILED TO MEASURE: ExecuteMission goal was not accepted";
  // handleAccepted()'s early-gate abort paths (battery/odometry/registry)
  // settle the ACTION RESULT directly WITHOUT ever touching mission/status
  // (mission_state_/publishStatus() are only reached further down) -- so
  // waiting on mission/status alone cannot distinguish "still IDLE because
  // handleAccepted() has not run yet" from "genuinely rejected early".
  // Polling the actual result here (found empirically necessary for this
  // test to reliably observe the REAL post-accept outcome, not a stale
  // pre-accept status snapshot) makes the wait below anchor on a real
  // signal either way.
  auto early_result_future = execute_client_->async_get_result(goal_handle_);
  early_result_future.wait_for(500ms);
  ASSERT_TRUE(
    waitFor([this]() {return latestStatus().state != MissionStatusMsg::STATE_RUNNING;}, 20.0))
    << "FAILED TO MEASURE: mission never reached a terminal state";
  EXPECT_EQ(latestStatus().state, MissionStatusMsg::STATE_ABORTED)
    << "B2 regression: inspect_point COMPLETED on a marker sighting from a dead world_model "
    << "stream -- the whole /world/semantic_landmarks MESSAGE must itself expire (R32), not just "
    << "each landmark's own time_since_seen_sec (which a dead publisher never advances again), "
    << "got last_reason=" << latestStatus().last_reason;
  for (const MissionStatusMsg & s : statuses()) {
    EXPECT_NE(s.state, MissionStatusMsg::STATE_COMPLETED)
      << "B2 regression: must never read as COMPLETED on dead world_model data, even transiently";
  }

  keep_alive_stop.store(true);
  keep_alive_thread.join();
  // Restart full streaming so TearDown()/other fixture expectations stay sane.
  stream_stop_.store(false);
  stream_thread_ = std::thread([this]() {streamWorld();});
}

// ---------------------------------------------------------------- (t)
// Y1 (2026-08-23, review round 1): step_timeout_sec was declared,
// validated, and never enforced -- a goal that NEVER resolves (fake never
// calls succeed/abort/cancel-completes) used to hang NavAction in RUNNING
// forever, with nothing to ever cut it off from below. Overrides
// navigator_goto_timeout_copy_sec/step_timeout_sec/retry_backoff_sec down
// so the whole retry-then-terminal cascade (max_step_retries=2 default)
// finishes in a reasonable test budget.
TEST_F(MissionExecutorFixture, StepTimeoutCancelsAStuckGoalAndMissionExitsCorrectly)
{
  // navigator_goto_timeout_copy_sec/goto_reach_copy_m must both move
  // together (registry's own goto_reach_copy_m <=
  // navigator_max_speed_copy_mps x navigator_goto_timeout_copy_sec rule) --
  // indoor_patrol's DEFAULT waypoints (mission_registry.cpp
  // defaultIndoorPatrol()) are a 1 m unit square, max leg 1.0 m, so 1.5 m
  // keeps comfortable margin without needing to override waypoints too.
  swapMissionNode(
    std::string(UAV_MISSION_CONFIG_DIR) + "/missions",
    {rclcpp::Parameter("navigator_goto_timeout_copy_sec", 3.0),
      rclcpp::Parameter("goto_reach_copy_m", 1.5), rclcpp::Parameter("step_timeout_sec", 4.0),
      rclcpp::Parameter("retry_backoff_sec", 0.3),
      // Y4's new ctor validation (2 x search_offset_m <= goto_reach_copy_m)
      // needs this shrunk to match goto_reach_copy_m above -- indoor_patrol
      // itself never uses search_offset_m, but the check runs regardless.
      rclcpp::Parameter("search_offset_m", 0.5)});
  ASSERT_TRUE(waitFor([this]() {return execute_client_->action_server_is_ready();}, 10.0))
    << "FAILED TO MEASURE: swapped-in node's ExecuteMission server never came up";
  const std::size_t statuses_before_swap = statuses().size();
  ASSERT_TRUE(waitFor([&]() {return statuses().size() > statuses_before_swap;}, 5.0))
    << "FAILED TO MEASURE: swapped-in node never published its own initial status";

  goto_pose_server_->setHold(true);   // NEVER resolves on its own -- Y1 must be what cuts it off
  ASSERT_TRUE(sendExecuteMissionGoal("indoor_patrol"))
    << "FAILED TO MEASURE: ExecuteMission goal was not accepted";
  ASSERT_TRUE(waitFor([this]() {return fake_state_.hasEvent("GotoPose", "accepted");}, 10.0))
    << "FAILED TO MEASURE: body GotoPose never reached the fake navigator";

  // Only 2 dispatches, not 3 (max_step_retries=2 would otherwise allow a
  // 3rd) -- mission_policy's OWN "same failure reason twice in a row stops
  // retrying" rule (last_failure_reason_repeated) kicks in on the SECOND
  // Y1 timeout, since every Y1 timeout reports the identical literal
  // reason string. Confirms Y1 integrates correctly with that existing
  // safety net rather than bypassing it. Each cancels via step_timeout_sec
  // on its own, with NO external pause/abort/authority-seize ever
  // happening -- purely the step's own timeout budget.
  uint64_t last_cancel_seq = 0;
  for (int attempt = 0; attempt < 2; ++attempt) {
    ASSERT_TRUE(
      waitFor(
        [this, last_cancel_seq]() {
          return fake_state_.findFirstSeqAfter("GotoPose", "canceled", last_cancel_seq) != 0;
        }, 8.0))
      << "FAILED TO MEASURE: step_timeout_sec never canceled attempt #" << (attempt + 1)
      << " -- the goal is still held with nothing ever cutting it off";
    last_cancel_seq = fake_state_.findFirstSeqAfter("GotoPose", "canceled", last_cancel_seq);
  }

  goto_pose_server_->setHold(false);   // release so Finish's own legs (untouched by Y1) can run free
  ASSERT_TRUE(
    waitFor([this]() {return latestStatus().state != MissionStatusMsg::STATE_RUNNING;}, 15.0))
    << "FAILED TO MEASURE: mission never reached a terminal state after the step-timeout budget "
    << "was exhausted";
  EXPECT_EQ(latestStatus().state, MissionStatusMsg::STATE_ABORTED);
  EXPECT_EQ(latestStatus().last_result_code, kResultAbortedTimeout)
    << "the terminal result must reflect the step's OWN timeout (ABORTED_TIMEOUT), routed through "
    << "evaluateStepFailure() like any other step failure -- got reason="
    << latestStatus().last_reason;
}

// ---------------------------------------------------------------- (u)
// Y2 (2026-08-23, review round 1): handleGoal()'s phase_ != kIdle check
// alone has a TOCTOU window -- phase_ only leaves kIdle inside
// handleAccepted(), a LATER, separate io_group_ callback (Reentrant: can
// run concurrently with a SECOND handleGoal() call). Fires 2 goals back to
// back via the raw action client (NOT the blocking sendExecuteMissionGoal()
// helper, which would serialize them and never exercise the race at all).
TEST_F(MissionExecutorFixture, TwoGoalsSentBackToBackYieldExactlyOneAcceptOneReject)
{
  ExecuteMission::Goal goal1;
  goal1.mission_id = "indoor_patrol";
  goal1.loop = false;
  ExecuteMission::Goal goal2;
  goal2.mission_id = "indoor_patrol";
  goal2.loop = false;

  auto future1 = execute_client_->async_send_goal(goal1);
  auto future2 = execute_client_->async_send_goal(goal2);   // fired immediately, no wait in between

  ASSERT_EQ(future1.wait_for(5s), std::future_status::ready)
    << "FAILED TO MEASURE: goal 1's accept/reject response never arrived";
  ASSERT_EQ(future2.wait_for(5s), std::future_status::ready)
    << "FAILED TO MEASURE: goal 2's accept/reject response never arrived";
  auto handle1 = future1.get();
  auto handle2 = future2.get();
  const int accepted_count = (handle1 != nullptr ? 1 : 0) + (handle2 != nullptr ? 1 : 0);
  EXPECT_EQ(accepted_count, 1)
    << "expected EXACTLY one of two back-to-back goals accepted (accepting_ flag should close "
    << "the TOCTOU window) -- got " << accepted_count;

  // Settle whichever goal WAS accepted so it does not leak into the next test.
  goal_handle_ = (handle1 != nullptr) ? handle1 : handle2;
  if (goal_handle_) {
    callAbortMission("test cleanup: settle the accepted goal");
    waitFor([this]() {return latestStatus().state != MissionStatusMsg::STATE_RUNNING;}, 10.0);
  }
}

// ---------------------------------------------------------------- (v)
// Y3 (2026-08-23, review round 1, R30): a goal must be REFUSED (at the
// ExecuteMission RESULT level -- handleGoal() cannot know this, it has no
// odometry checks of its own) when odometry_fused is not fresh at
// acceptance time, instead of silently seeding mission_home_ at (0,0,0), a
// perfectly plausible-looking real position.
TEST_F(MissionExecutorFixture, GoalRejectedWithoutFreshOdometry)
{
  stream_stop_.store(true);
  if (stream_thread_.joinable()) {
    stream_thread_.join();
  }
  std::this_thread::sleep_for(300ms);   // odometry_fused now stale (default gate: 2 x 0.1 s = 0.2 s)

  // Refresh JUST battery/authority/localization right before sending --
  // isolates "odometry specifically is stale" from the unrelated
  // pre-existing battery-unmeasured start gate, which stopping the WHOLE
  // stream would otherwise trip first (battery goes stale in the same
  // 300 ms window). Deliberately does NOT touch odometry_pub_.
  {
    ControlAuthority auth;
    auth.header.stamp = probe_->now();
    auth.uav_id = uav_id_;
    auth.active_source = kSourceMission;
    authority_pub_->publish(auth);
    VehicleHealth health;
    health.header.stamp = probe_->now();
    health.uav_id = uav_id_;
    health.battery_remaining = 0.9f;
    health.overall_level = VehicleHealth::LEVEL_OK;
    health_pub_->publish(health);
    LocalizationStatus loc;
    loc.header.stamp = probe_->now();
    loc.uav_id = uav_id_;
    loc.is_valid = true;
    loc.active_source = LocalizationStatus::SOURCE_FUSED;
    localization_pub_->publish(loc);
    std::this_thread::sleep_for(50ms);   // let it land before the goal is sent
  }

  ASSERT_TRUE(sendExecuteMissionGoal("indoor_patrol"))
    << "FAILED TO MEASURE: goal was not even ACCEPTED (handleGoal() has no odometry check of its "
    << "own -- this gate lives in handleAccepted(), the RESULT, not the accept/reject response)";
  auto result_future = execute_client_->async_get_result(goal_handle_);
  ASSERT_TRUE(waitFor([&]() {return result_future.wait_for(0s) == std::future_status::ready;}, 10.0))
    << "FAILED TO MEASURE: ExecuteMission result never arrived";
  const auto wrapped = result_future.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(wrapped.result != nullptr);
  EXPECT_FALSE(wrapped.result->success);
  EXPECT_EQ(wrapped.result->result_code, kResultAbortedLostLocalization)
    << "message=" << wrapped.result->message;
  EXPECT_NE(wrapped.result->message.find("odometry"), std::string::npos)
    << "message should name the reason -- got: " << wrapped.result->message;

  // Node must recover once odometry resumes -- not permanently wedged.
  stream_stop_.store(false);
  stream_thread_ = std::thread([this]() {streamWorld();});
  ASSERT_TRUE(waitFor([this]() {return sendExecuteMissionGoal("indoor_patrol");}, 5.0))
    << "FAILED TO MEASURE: node never accepted a fresh goal again once odometry resumed";
  callAbortMission("test cleanup");
  waitFor([this]() {return latestStatus().state != MissionStatusMsg::STATE_RUNNING;}, 10.0);
}

}  // namespace
}  // namespace uav_mission
