#ifndef UAV_MISSION__MISSION_EXECUTOR_NODE_HPP_
#define UAV_MISSION__MISSION_EXECUTOR_NODE_HPP_

#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <uav_interfaces/action/execute_mission.hpp>
#include <uav_interfaces/action/goto_pose.hpp>
#include <uav_interfaces/action/land.hpp>
#include <uav_interfaces/action/takeoff.hpp>
#include <uav_interfaces/action/track_target.hpp>
#include <uav_interfaces/action/hold_position.hpp>
#include <uav_interfaces/action/follow_path.hpp>
#include <uav_interfaces/action/recover.hpp>
#include <uav_interfaces/msg/control_authority.hpp>
#include <uav_interfaces/msg/localization_status.hpp>
#include <uav_interfaces/msg/mission_event.hpp>
#include <uav_interfaces/msg/mission_status.hpp>
#include <uav_interfaces/msg/semantic_landmark_array.hpp>
#include <uav_interfaces/msg/target_state.hpp>
#include <uav_interfaces/msg/vehicle_health.hpp>
#include <uav_interfaces/srv/abort_mission.hpp>
#include <uav_interfaces/srv/load_mission.hpp>
#include <uav_interfaces/srv/pause_mission.hpp>
#include <uav_interfaces/srv/resume_mission.hpp>

#include "uav_mission/bt_nodes.hpp"
#include "uav_mission/continuity_tracker.hpp"
#include "uav_mission/mission_context.hpp"
#include "uav_mission/mission_policy.hpp"
#include "uav_mission/mission_registry.hpp"
#include "uav_mission/nav_goal_broker.hpp"

namespace uav_mission
{

/// The internal execution phase -- distinct from MissionStatus.state
/// (mission_state_): a mission that is STATE_RUNNING can be in any of
/// kTakeoffSending/kBody/kFinishGotoHome/kFinishLanding depending on which
/// leg of the flight it is actually flying.
enum class ExecPhase : uint8_t
{
  kIdle,
  kTakeoffSending,
  kBody,
  kCanceling,        // waiting for broker_ to reach kIdle before settling pending_termination_
  kPaused,           // BT tree not ticked; only ResumeMission (or a paused-timeout verdict) moves on
  kFinishGotoHome,   // "Finish": shared by SUCCESS, END_EARLY and ABORT_LAND (plan P9 S:2)
  kFinishLanding,
  kRecovering,       // G-M4.4b: follow_target's search budget exhausted -> Recover(CLIMB) -> abort
};

/// What tickCanceling() does once broker_ reaches kIdle.
struct PendingTermination
{
  enum class Kind : uint8_t { kNone, kPause, kAbortHold, kFinish };
  Kind kind = Kind::kNone;
  uint8_t result_code = kResultUnknown;
  std::string reason;
};

/// BT engine + NavGoalBroker + action server + 4 services (P9.4, plan
/// P9-mission.md S:1/S:5). MissionContext is implemented directly on this
/// class (not a nested object) so the 6 custom BT leaf nodes (bt_nodes.hpp)
/// call straight back into it without an extra indirection.
///
/// Full architecture rationale -> uav_mission/README.md ("mission_executor_
/// node (P9.4) -- kien truc that"); Y8/Y9 (2026-08-23, review round 1)
/// trimmed this block to what changing THIS file specifically needs, the
/// rest was duplicating README.md and drifting out of sync with it.
///
///   1. R24: tick_group_ (MutuallyExclusive, the ONE 10 Hz timer) +
///      io_group_ (Reentrant, everything else -- so io_group_ callbacks
///      can run concurrently with EACH OTHER, not just with tick_group_).
///
///   2. state_mutex_ covers every field below except broker_ (own mutex,
///      nav_goal_broker.hpp) and cancel_fn_mutex_/current_cancel_fn_ (own
///      tiny mutex). onTick() holds state_mutex_ for the WHOLE tick,
///      including the nested BT tick and every MissionContext call it
///      triggers. Every io_group_ subscription/service/action-goal/
///      cancel/accepted callback takes it too.
///      🔴 Y8/Y9 fix (2026-08-23): this point used to ALSO claim
///      sendTypedGoal<>()'s action CLIENT goal-response/result/cancel-
///      response callbacks take state_mutex_ -- FALSE, verified against
///      the actual code. Those 3 callbacks touch ONLY broker_ (its own
///      mutex) and cancel_fn_mutex_, never state_mutex_/phase_/tree_/any
///      cached world field. A change that reads/writes state_mutex_-
///      protected state from inside one of those 3 without taking
///      state_mutex_ itself IS a real race -- the stale claim would have
///      led a future reader straight into it.
///
///   3. Priority 5 (step failure) is evaluated inside NavAction
///      (onRunning() -> evaluateStepFailure()), never at the top of
///      onTick() -- see README.md for the priorities-1/3/4-forced-inert
///      mechanics evaluateStepFailure() relies on.
///
///   4. "Finish" (GotoPose(home) -> Land) is plain C++ state
///      (kFinishGotoHome/kFinishLanding), NOT a BT subtree -- one shared
///      landing sequence for SUCCESS/END_EARLY/ABORT_LAND. See README.md.
///
///   5. The 4 CẤM (uav_mission/README.md): this class never publishes
///      ControlCommand/control/*, never calls clear_fault/
///      clear_safety_latch, never calls arm/disarm/set_mode, never calls
///      SetControlAuthority. /control/authority is READ ONLY.
class MissionExecutorNode : public rclcpp::Node, public MissionContext
{
public:
  explicit MissionExecutorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // ---- MissionContext overrides -----------------------------------------
  // PRECONDITION for every method below except the two trivial getters:
  // called only from tick_group_'s thread, while onTick() already holds
  // state_mutex_ (point 2 above) -- never lock state_mutex_ again inside
  // these bodies.
  double rosNowSeconds() const override;
  double retryBackoffSec() const override;
  double stepTimeoutSec() const override;
  bool sendGotoPose(
    double x, double y, double z, const std::string & frame_id,
    double acceptance_radius, double max_speed) override;
  bool sendTrackTarget(
    int32_t target_id, double standoff_distance, double duration_seconds,
    double target_lost_timeout) override;
  std::optional<GoalOutcome> pollNavActionOutcome() override;
  void cancelNavActionGoal() override;
  void resetStepRetryState() override;
  GuardVerdict evaluateStepFailure(uint8_t result_code, const std::string & reason) override;
  void reportStepStarted(const std::string & step_name) override;
  void reportStepCompleted(const std::string & step_name, bool succeeded) override;
  std::optional<SightedLandmark> latestLandmark(int32_t marker_id) const override;
  std::optional<SightedTarget> latestTarget(int32_t target_id) const override;
  void noteTrackTargetProgress(double elapsed_this_attempt_sec) override;
  void noteSearchEpisodeBegins() override;
  bool searchBudgetAvailable() const override;
  void noteSearchBudgetExhausted() override;

private:
  using ExecuteMission = uav_interfaces::action::ExecuteMission;
  using GoalHandleExecuteMission = rclcpp_action::ServerGoalHandle<ExecuteMission>;

  // ---- setup --------------------------------------------------------------
  MissionPolicyParams declarePolicyParams();
  MissionRegistryParams declareRegistryParams();

  // ---- ExecuteMission action server (io_group_) ---------------------------
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteMission::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleExecuteMission> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandleExecuteMission> goal_handle);
  /// Writes every blackboard key the 3 shipped mission XML bodies read
  /// (config/missions/*.xml) from plan. target_id/search points have no
  /// mission_registry field (P9.2/P9.3 froze that schema without them) --
  /// this is the node-level default documented in README.md's "P9.4 lech
  /// plan": target_id=-1 (track whichever target is first seen),
  /// search points = home/approach +/- search_offset_m_.
  void populateBlackboard(BT::Tree & tree, const MissionPlan & plan);
  /// Settles goal_handle_ (succeed/canceled/abort, per ROS2 action server
  /// rules -- canceled() only if is_canceling(), never abort() for a
  /// client-initiated cancel) and clears it. A no-op if no goal is active.
  /// Callers MUST publishStatus(true) BEFORE calling this (contract 2.19:
  /// goal_id must still be non-empty on the terminal status, and the
  /// terminal state must be on the wire no later than the action result --
  /// G-M1 gate bug, 2026-08-22, mission_executor_node.cpp finishAsAborted/
  /// finishAsCompleted).
  void settleGoalHandle(bool success, uint8_t result_code, const std::string & message);

  // ---- 4 services (io_group_) ----------------------------------------------
  void handleLoadMission(
    const std::shared_ptr<uav_interfaces::srv::LoadMission::Request> request,
    std::shared_ptr<uav_interfaces::srv::LoadMission::Response> response);
  void handlePauseMission(
    const std::shared_ptr<uav_interfaces::srv::PauseMission::Request> request,
    std::shared_ptr<uav_interfaces::srv::PauseMission::Response> response);
  void handleResumeMission(
    const std::shared_ptr<uav_interfaces::srv::ResumeMission::Request> request,
    std::shared_ptr<uav_interfaces::srv::ResumeMission::Response> response);
  void handleAbortMission(
    const std::shared_ptr<uav_interfaces::srv::AbortMission::Request> request,
    std::shared_ptr<uav_interfaces::srv::AbortMission::Response> response);
  /// mission_id validation shared by Pause/Resume/Abort (contract §2.19-
  /// adjacent .srv comment: "empty targets the running mission"). Returns
  /// false (with *out_message set) if mission_id is non-empty and does not
  /// match current_mission_id_, or if no mission is running/paused at all.
  bool targetsRunningMission(const std::string & mission_id, std::string * out_message) const;

  // ---- tick loop (tick_group_) ----------------------------------------------
  void onTick();
  MissionWorldView buildTopLevelView(double now_sec);
  void beginCancelFor(const GuardVerdict & verdict, double now_sec);
  void tickCanceling(double now_sec);
  void enterPaused(const std::string & reason, uint8_t result_code, double now_sec);
  void tickTakeoff(double now_sec);
  void tickBody(double now_sec);
  void beginFinish(uint8_t result_code, const std::string & reason, double now_sec);
  void tickFinishGotoHome(double now_sec);
  void tickFinishLanding(double now_sec);
  /// Terminal, non-Finish ending (ABORT_HOLD, or a Takeoff/Finish leg that
  /// itself failed) -- settles the goal handle, resets phase_ to kIdle.
  void finishAsAborted(uint8_t result_code, const std::string & reason, double now_sec);
  /// Terminal, successful ending (after Finish's Land leg succeeds).
  void finishAsCompleted(double now_sec);

  /// G-M4.4b (2026-08-23): follow_target's search-attempts budget just ran
  /// out (searchBudgetAvailable() went false the SAME tick tickBody() saw
  /// the tree return FAILURE with a stored ABORTED_LOST_TARGET verdict) --
  /// dispatch Recover(CLIMB) instead of the normal Finish/abort path.
  /// Mirrors beginFinish()/tickFinishLanding(): a phase_ + a single
  /// goal-dispatched flag, no BT subtree (point 4 of the class doc).
  void beginRecovering(const std::string & reason, double now_sec);
  void tickRecovering(double now_sec);
  /// Reuses the SAME sendTypedGoal<>()/broker_ every other leg dispatch
  /// goes through (point after finishAsCompleted below) -- "at most one
  /// navigator goal alive" holds for Recover too.
  bool dispatchRecover();

  // ---- navigator goal dispatch, shared by NavAction (via MissionContext)
  // and this node's own Takeoff/Finish legs. Template lives in this header
  // (multiple .cpp call sites, 4 instantiations: GotoPose/TrackTarget/
  // Takeoff/Land). ----
  template<typename ActionT>
  bool sendTypedGoal(
    typename rclcpp_action::Client<ActionT>::SharedPtr client,
    const typename ActionT::Goal & goal);
  /// Node-owned dispatch for the two legs NavAction never drives (Takeoff
  /// happens before the mission body tree exists; Land is the tail of
  /// "Finish", point 4 above) -- both go through the SAME sendTypedGoal<>()
  /// and therefore the SAME broker_, honoring "at most one goal navigator
  /// ever alive" node-wide, not just within the BT tree.
  bool dispatchTakeoff(double target_altitude);
  bool dispatchLand();
  void cancelCurrentGoal();
  /// Yaw-only quaternion holding the CURRENT odometry heading -- the "don't
  /// care about heading" neutral for GotoPose is "keep facing where you
  /// already face", not "snap to yaw 0" (G-M2 fallout, 2026-08-22: yaw=0
  /// absolute cost every GotoPose leg a gratuitous turn, e.g. 130 deg/52s on
  /// a near-zero-distance Finish/GotoHome leg). Falls back to identity
  /// (yaw 0) + logs WARN once if odometry is unavailable/non-finite -- never
  /// blocks the goal on missing yaw, only position is safety-critical here.
  geometry_msgs::msg::Quaternion currentHeadingOrientation();
  /// R32: battery_remaining age-gated against 2 x
  /// battery_health_period_copy_sec, same discipline as the authority
  /// staleness check -- a reading held from a dead /state/health_px4
  /// stream must expire to NaN ("not measured"), never stay trustworthy
  /// forever. The ONE place all 3 battery readers (buildTopLevelView(),
  /// evaluateStepFailure(), handleAccepted()'s isBatteryOkToStart() gate)
  /// go through.
  double freshBatteryOrNan(double now_sec) const;
  /// B2/Y11 (2026-08-23): odometry_received_ ANDed with an R32 age gate
  /// (2 x odometry_publish_period_copy_sec_) -- the ONE place every
  /// odometry reader (handleAccepted()'s Y3 gate, mission_home_/search-
  /// point seeding, tickFinishGotoHome()'s home-z, currentHeadingOrientation()
  /// 's yaw) goes through, so a dead /state/odometry_fused stream falls
  /// back exactly like "never received" instead of staying trustworthy on
  /// a stale sample forever.
  bool odometryFresh() const;

  // ---- publishing (tick_group_ AND io_group_ -- handleAccepted()/
  // handleResumeMission() etc also call these; always under state_mutex_) --
  void publishStatus(bool force);
  void publishEvent(
    uint8_t event_type, const std::string & step_name, const std::string & description,
    uint8_t result_code);

  // ------------------------------------------------------------- parameters
  std::string uav_id_;
  std::string prefix_;
  std::string odom_frame_;
  double tick_hz_ = 10.0;
  double takeoff_altitude_m_ = 1.5;
  double finish_goto_speed_mps_ = 0.55;
  double finish_acceptance_radius_m_ = 0.3;
  double marker_seen_freshness_sec_ = 1.0;
  double target_seen_freshness_sec_ = 1.0;
  double search_offset_m_ = 2.0;
  // G-M4.4b (2026-08-23): follow_target's loss-episode search budget --
  // plan P9 S:2.5 "2 lượt x 20s -> Recover(CLIMB) -> abort".
  int search_attempts_max_ = 2;
  double track_progress_reset_sec_ = 10.0;   // R29: validated >= 2 x tick period in ctor
  // B2 (2026-08-23, review round 1): recover_climb_altitude_m_ MUST stay
  // <= registry_params_.max_alt_copy_m -- validated in the ctor (Y5).
  // 7.5, not 10.0: the OLD default sat ABOVE max_alt_copy_m's shipped 8.0,
  // i.e. "climb to a safe altitude" would have flown THROUGH an indoor
  // ceiling. Not simply clamped to 8.0 -- kept a margin under the ceiling
  // in case max_alt_copy_m is ever tightened without this default in mind.
  double recover_climb_altitude_m_ = 7.5;
  // B2/Y11 (2026-08-23, review round 1): R32 age-gates for world_model's
  // 2 topics (landmarks/target) and localization_mux's odometry_fused --
  // a reading held from a dead publisher must expire to "not seen", never
  // stay trustworthy forever. *_copy because the true period lives in
  // ANOTHER package's config (world_model_params.yaml publish_rate_hz=10.0,
  // uav_localization's localization_mux_node.cpp publish_rate_hz default
  // 10.0) -- validated positive+finite in the ctor.
  double world_model_publish_period_copy_sec_ = 0.1;
  double odometry_publish_period_copy_sec_ = 0.1;

  MissionPolicyParams policy_params_;
  MissionRegistryParams registry_params_;
  MissionPolicy policy_;
  MissionRegistry registry_;
  NavGoalBroker broker_;

  // ---------------------------------------------------------- R24 point 2
  mutable std::mutex state_mutex_;

  // -- mission/phase bookkeeping (state_mutex_) --
  ExecPhase phase_ = ExecPhase::kIdle;
  ExecPhase paused_from_phase_ = ExecPhase::kBody;
  PendingTermination pending_termination_;
  bool phase_goal_dispatched_ = false;
  // Y2 (2026-08-23, review round 1): closes the handleGoal()/handleAccepted()
  // TOCTOU window -- see handleGoal()'s doc comment.
  bool accepting_ = false;

  MissionPlan current_plan_;
  bool loop_mission_ = false;
  Waypoint mission_home_;

  uint8_t mission_state_ = uav_interfaces::msg::MissionStatus::STATE_IDLE;
  uint8_t last_published_state_ = uav_interfaces::msg::MissionStatus::STATE_IDLE;
  std::string current_mission_id_;
  std::string current_step_name_;
  uint32_t current_step_index_ = 0;
  uint32_t total_steps_ = 0;
  uint8_t last_result_code_ = kResultUnknown;
  std::string last_reason_;
  rclcpp::Time mission_start_time_{0, 0, RCL_ROS_TIME};

  uint32_t step_retry_count_ = 0;
  bool have_last_failure_reason_ = false;
  std::string last_failure_reason_;
  std::optional<GuardVerdict> pending_terminal_verdict_;

  // G-M4.4b: follow_target's search-attempts budget -- deliberately NOT
  // reset by resetStepRetryState() (see mission_context.hpp). Reset to 0 in
  // handleAccepted() (mission start) and inside noteTrackTargetProgress()
  // once track_progress_reset_sec_ of unbroken tracking is observed.
  uint32_t follow_search_attempts_ = 0;
  std::string recovering_reason_;   // set by beginRecovering(), used by tickRecovering()

  bool operator_abort_requested_ = false;
  std::string operator_abort_reason_;
  bool operator_pause_requested_ = false;
  // Y6 (2026-08-23, review round 1): edge-latch for the battery-WARN-
  // while-PAUSED MissionEvent -- true once notified for the CURRENT
  // continuous WARN episode, reset when battery clears WARN/unknown or the
  // mission leaves kPaused. See onTick()'s kPaused branch.
  bool battery_warn_notified_while_paused_ = false;
  // G-M1 review round 1 (2026-08-23): edge-latch for the Finish/Recovering
  // authority-seize diagnostic WARN -- logs ONCE per continuous seize
  // episode (not every tick), same pattern as the Y6 latch above. See
  // onTick()'s kFinishGotoHome/kFinishLanding/kRecovering branch.
  bool authority_seize_warned_this_episode_ = false;

  std::shared_ptr<GoalHandleExecuteMission> goal_handle_;

  // Outlives goal_handle_ on purpose: settleGoalHandle() drops the handle as soon as
  // the action result is sent, and the terminal MissionStatus is published after that
  // on three paths (contract 2.19 -- see the fix note in mission_executor_node.cpp).
  std::string settling_goal_id_;

  ContinuityTracker authority_seize_tracker_;
  ContinuityTracker battery_warn_tracker_;
  ContinuityTracker battery_unknown_tracker_;
  ContinuityTracker paused_tracker_;

  // -- cached world state, "received + arrival" pattern (uav_safety style,
  // state_mutex_ instead of a dedicated io_group_) --
  uav_interfaces::msg::ControlAuthority last_authority_;
  bool authority_received_ = false;
  rclcpp::Time authority_arrival_{0, 0, RCL_ROS_TIME};

  uav_interfaces::msg::VehicleHealth last_health_;
  bool health_received_ = false;
  rclcpp::Time health_arrival_{0, 0, RCL_ROS_TIME};

  uav_interfaces::msg::LocalizationStatus last_localization_;
  bool localization_received_ = false;
  rclcpp::Time localization_arrival_{0, 0, RCL_ROS_TIME};

  nav_msgs::msg::Odometry last_odometry_;
  bool odometry_received_ = false;
  rclcpp::Time odometry_arrival_{0, 0, RCL_ROS_TIME};
  bool yaw_fallback_warned_ = false;   // currentHeadingOrientation(): log the w=1 fallback once

  uav_interfaces::msg::SemanticLandmarkArray last_landmarks_;
  bool landmarks_received_ = false;
  rclcpp::Time landmarks_arrival_{0, 0, RCL_ROS_TIME};
  uav_interfaces::msg::TargetState last_target_;
  bool target_received_ = false;
  rclcpp::Time target_arrival_{0, 0, RCL_ROS_TIME};

  unsigned status_tick_count_ = 0;
  static constexpr unsigned kStatusEveryTicks = 5;   // 10 Hz tick / 5 = 2 Hz (contract §2.19)

  // -------------------------------------------------- broker cancel wiring
  // Separate small mutex (not state_mutex_): set by whichever
  // sendTypedGoal<ActionT>() goal-response callback most recently landed
  // (io_group_), read by cancelCurrentGoal() (tick_group_, under
  // state_mutex_ already) -- kept apart from state_mutex_ so a cancel can
  // be issued without the send-side callback ever needing state_mutex_.
  std::mutex cancel_fn_mutex_;
  std::function<void()> current_cancel_fn_;

  // --------------------------------------------------------- ROS interfaces
  rclcpp::CallbackGroup::SharedPtr tick_group_;
  rclcpp::CallbackGroup::SharedPtr io_group_;

  rclcpp::Subscription<uav_interfaces::msg::ControlAuthority>::SharedPtr authority_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::VehicleHealth>::SharedPtr health_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::LocalizationStatus>::SharedPtr localization_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::SemanticLandmarkArray>::SharedPtr landmarks_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::TargetState>::SharedPtr target_subscription_;

  rclcpp::Publisher<uav_interfaces::msg::MissionStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<uav_interfaces::msg::MissionEvent>::SharedPtr events_publisher_;

  rclcpp::Service<uav_interfaces::srv::LoadMission>::SharedPtr load_service_;
  rclcpp::Service<uav_interfaces::srv::PauseMission>::SharedPtr pause_service_;
  rclcpp::Service<uav_interfaces::srv::ResumeMission>::SharedPtr resume_service_;
  rclcpp::Service<uav_interfaces::srv::AbortMission>::SharedPtr abort_service_;

  rclcpp_action::Server<ExecuteMission>::SharedPtr execute_mission_server_;

  rclcpp_action::Client<uav_interfaces::action::Takeoff>::SharedPtr takeoff_client_;
  rclcpp_action::Client<uav_interfaces::action::GotoPose>::SharedPtr goto_pose_client_;
  rclcpp_action::Client<uav_interfaces::action::HoldPosition>::SharedPtr hold_position_client_;
  rclcpp_action::Client<uav_interfaces::action::FollowPath>::SharedPtr follow_path_client_;
  rclcpp_action::Client<uav_interfaces::action::TrackTarget>::SharedPtr track_target_client_;
  rclcpp_action::Client<uav_interfaces::action::Recover>::SharedPtr recover_client_;
  rclcpp_action::Client<uav_interfaces::action::Land>::SharedPtr land_client_;

  rclcpp::TimerBase::SharedPtr tick_timer_;

  // ---------------------------------------------------------------- R0/R24
  // Declared LAST on purpose -- C++ destroys members in REVERSE declaration
  // order, so these are destroyed FIRST, before cancel_fn_mutex_/
  // current_cancel_fn_/broker_/every ROS interface above. ~BT::Tree() halts
  // any still-RUNNING leaf, which calls NavAction::onHalted() ->
  // cancelNavActionGoal() -> cancelCurrentGoal() -> back into THIS class
  // (cancel_fn_mutex_, current_cancel_fn_, broker_). Declaring tree_/
  // bt_factory_ any earlier means that callback runs against
  // already-destroyed members -- a real crash caught by gdb during P9.4
  // (SIGSEGV in ~MissionExecutorNode(), captured shared_ptr<ClientGoalHandle>
  // freed twice): ~Tree() must run while everything it can call back into
  // is still alive.
  BT::BehaviorTreeFactory bt_factory_;
  std::unique_ptr<BT::Tree> tree_;
};

}  // namespace uav_mission

#endif  // UAV_MISSION__MISSION_EXECUTOR_NODE_HPP_
