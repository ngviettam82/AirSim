#include "uav_mission/mission_executor_node.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace uav_mission
{

namespace
{

std::string uuidToHex(const rclcpp_action::GoalUUID & uuid)
{
  std::ostringstream oss;
  for (uint8_t byte : uuid) {
    oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
  }
  return oss.str();
}

// Same formula as uav_navigation's yawFromQuaternion (carrot.cpp:117) /
// uav_safety's tryYawFromQuaternion (safety_supervisor_node.cpp:55) --
// reimplemented locally (a few lines) rather than depending on either
// package. Returns false (never writes *yaw_out) on a non-finite or
// near-zero-norm quaternion.
bool tryYawFromQuaternion(double x, double y, double z, double w, double * yaw_out)
{
  constexpr double kMinOrientationNormSq = 1e-6;
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(w)) {
    return false;
  }
  if (x * x + y * y + z * z + w * w < kMinOrientationNormSq) {
    return false;
  }
  *yaw_out = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  return true;
}

geometry_msgs::msg::Quaternion yawOnlyQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

// G-M4.4b (2026-08-23): follow_target's search-attempts budget params --
// fail fast (caught by the ctor's try/catch) rather than let a bad value
// silently produce nonsense episode semantics at runtime.
void validateFollowSearchParams(
  int search_attempts_max, double track_progress_reset_sec, double tick_hz)
{
  if (search_attempts_max < 1) {
    throw std::invalid_argument("search_attempts_max must be >= 1");
  }
  if (tick_hz <= 0.0) {
    throw std::invalid_argument("tick_hz must be > 0");
  }
  // R29: a discrete/slow-sampled signal's grace period must be >= 2x its
  // sampling period. "Sustained tracking" here is sampled once per
  // onRunning() tick (tick_hz_), so track_progress_reset_sec_ must clear
  // that bar or it could reset on noise from a SINGLE lucky tick.
  const double tick_period_sec = 1.0 / tick_hz;
  if (track_progress_reset_sec < 2.0 * tick_period_sec) {
    throw std::invalid_argument(
      "track_progress_reset_sec must be >= 2x tick period (R29)");
  }
}

// B2/Y11 (2026-08-23): R32 age-gate periods for the 3 topics
// odometryFresh()/latestLandmark()/latestTarget() key off of.
void validateFreshnessPeriods(
  double world_model_publish_period_copy_sec, double odometry_publish_period_copy_sec)
{
  if (!(std::isfinite(world_model_publish_period_copy_sec) &&
    world_model_publish_period_copy_sec > 0.0))
  {
    throw std::invalid_argument("world_model_publish_period_copy_sec must be finite and positive");
  }
  if (!(std::isfinite(odometry_publish_period_copy_sec) && odometry_publish_period_copy_sec > 0.0)) {
    throw std::invalid_argument("odometry_publish_period_copy_sec must be finite and positive");
  }
}

// Y5 (2026-08-23): recover_climb_altitude_m_ is the Recover(CLIMB) goal's
// target altitude -- it must never exceed the SAME indoor ceiling every
// self-issued altitude in this node is held to (max_alt_copy_m), or
// "climb to a safe altitude" flies the drone through the ceiling instead.
void validateRecoverAltitude(double recover_climb_altitude_m, double max_alt_copy_m)
{
  if (recover_climb_altitude_m > max_alt_copy_m) {
    throw std::invalid_argument(
      "recover_climb_altitude_m must be <= max_alt_copy_m (Recover(CLIMB) must not fly through "
      "the indoor ceiling)");
  }
}

// Y4 (2026-08-23, R27-5): self-issued waypoints (follow_target's search_z
// = takeoff_altitude_m_, search legs = home +/- search_offset_m_) must
// clear the SAME envelope check mission_registry.cpp already applies to
// every CALLER-supplied waypoint -- fail fast at startup rather than
// silently dispatching an out-of-envelope GotoPose at runtime.
// inspect_point's search_z instead copies approach_z, already validated
// by mission_registry.load() (a caller-supplied field), so it needs no
// separate check here.
void validateSelfIssuedWaypointEnvelope(
  double takeoff_altitude_m, double search_offset_m, double min_alt_copy_m, double max_alt_copy_m,
  double goto_reach_copy_m)
{
  if (takeoff_altitude_m < min_alt_copy_m || takeoff_altitude_m > max_alt_copy_m) {
    throw std::invalid_argument(
      "takeoff_altitude_m must be within [min_alt_copy_m, max_alt_copy_m] -- it is also "
      "follow_target's self-issued search_z");
  }
  if (search_offset_m <= 0.0) {
    throw std::invalid_argument("search_offset_m must be > 0");
  }
  // The longer of the two self-issued search legs: search_point_1 <->
  // search_point_2 spans 2 x search_offset_m (home - offset to home +
  // offset); home -> search_point_1/2 individually only spans
  // search_offset_m, strictly shorter.
  if (2.0 * search_offset_m > goto_reach_copy_m) {
    throw std::invalid_argument(
      "2 x search_offset_m must be <= goto_reach_copy_m (the search_point_1 <-> search_point_2 "
      "leg must clear the same per-leg reach cap caller waypoints do)");
  }
}

}  // namespace

MissionExecutorNode::MissionExecutorNode(const rclcpp::NodeOptions & options)
try
: Node("mission_executor_node", options),
  uav_id_(declare_parameter<std::string>("uav_id", "uav0")),
  prefix_("/uav/" + uav_id_),
  odom_frame_(declare_parameter<std::string>("odom_frame", "odom")),
  tick_hz_(declare_parameter<double>("tick_hz", 10.0)),
  takeoff_altitude_m_(declare_parameter<double>("takeoff_altitude_m", 1.5)),
  finish_goto_speed_mps_(declare_parameter<double>("finish_goto_speed_mps", 0.55)),
  finish_acceptance_radius_m_(declare_parameter<double>("finish_acceptance_radius_m", 0.3)),
  marker_seen_freshness_sec_(declare_parameter<double>("marker_seen_freshness_sec", 1.0)),
  target_seen_freshness_sec_(declare_parameter<double>("target_seen_freshness_sec", 1.0)),
  search_offset_m_(declare_parameter<double>("search_offset_m", 2.0)),
  search_attempts_max_(declare_parameter<int>("search_attempts_max", 2)),
  track_progress_reset_sec_(declare_parameter<double>("track_progress_reset_sec", 10.0)),
  recover_climb_altitude_m_(declare_parameter<double>("recover_climb_altitude_m", 7.5)),
  world_model_publish_period_copy_sec_(
    declare_parameter<double>("world_model_publish_period_copy_sec", 0.1)),
  odometry_publish_period_copy_sec_(
    declare_parameter<double>("odometry_publish_period_copy_sec", 0.1)),
  policy_params_(declarePolicyParams()),
  registry_params_(declareRegistryParams()),
  policy_(policy_params_),
  registry_(
    registry_params_,
    declare_parameter<std::string>(
      "missions_dir",
      ament_index_cpp::get_package_share_directory("uav_mission") + "/config/missions"))
{
  validateFollowSearchParams(search_attempts_max_, track_progress_reset_sec_, tick_hz_);
  validateFreshnessPeriods(world_model_publish_period_copy_sec_, odometry_publish_period_copy_sec_);
  validateRecoverAltitude(recover_climb_altitude_m_, registry_params_.max_alt_copy_m);
  validateSelfIssuedWaypointEnvelope(
    takeoff_altitude_m_, search_offset_m_, registry_params_.min_alt_copy_m,
    registry_params_.max_alt_copy_m, registry_params_.goto_reach_copy_m);

  // R24 point 1: tick_group_ is the ONLY member of a MutuallyExclusive
  // group (just the 10 Hz timer) -- io_group_ is Reentrant and holds
  // everything else. See header point 2 for the state_mutex_ discipline
  // this requires.
  tick_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  io_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = io_group_;

  authority_subscription_ = create_subscription<uav_interfaces::msg::ControlAuthority>(
    prefix_ + "/control/authority", rclcpp::QoS(1).reliable().transient_local(),
    [this](uav_interfaces::msg::ControlAuthority::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_authority_ = *msg;
      authority_received_ = true;
      authority_arrival_ = now();
    }, sub_options);

  health_subscription_ = create_subscription<uav_interfaces::msg::VehicleHealth>(
    prefix_ + "/state/health_px4", rclcpp::QoS(10),
    [this](uav_interfaces::msg::VehicleHealth::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_health_ = *msg;
      health_received_ = true;
      health_arrival_ = now();
    }, sub_options);

  localization_subscription_ = create_subscription<uav_interfaces::msg::LocalizationStatus>(
    prefix_ + "/state/localization_status", rclcpp::QoS(10),
    [this](uav_interfaces::msg::LocalizationStatus::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_localization_ = *msg;
      localization_received_ = true;
      localization_arrival_ = now();
    }, sub_options);

  odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    prefix_ + "/state/odometry_fused", rclcpp::QoS(10),
    [this](nav_msgs::msg::Odometry::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_odometry_ = *msg;
      odometry_received_ = true;
      odometry_arrival_ = now();   // B2/Y11: R32 arrival stamp, see odometryFresh()
    }, sub_options);

  landmarks_subscription_ = create_subscription<uav_interfaces::msg::SemanticLandmarkArray>(
    prefix_ + "/world/semantic_landmarks", rclcpp::QoS(10),
    [this](uav_interfaces::msg::SemanticLandmarkArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_landmarks_ = *msg;
      landmarks_received_ = true;
      landmarks_arrival_ = now();   // B2: R32 arrival stamp, see latestLandmark()
    }, sub_options);

  target_subscription_ = create_subscription<uav_interfaces::msg::TargetState>(
    prefix_ + "/world/target_state", rclcpp::QoS(10),
    [this](uav_interfaces::msg::TargetState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_target_ = *msg;
      target_received_ = true;
      target_arrival_ = now();   // B2: R32 arrival stamp, see latestTarget()
    }, sub_options);

  // Contract §2.19: TransientLocal so a late-joining subscriber sees the
  // current state immediately, not just the next on-change/2 Hz publish.
  status_publisher_ = create_publisher<uav_interfaces::msg::MissionStatus>(
    prefix_ + "/mission/status", rclcpp::QoS(1).reliable().transient_local());
  // Contract §2.19: Volatile -- an edge-history stream, not current state.
  events_publisher_ = create_publisher<uav_interfaces::msg::MissionEvent>(
    prefix_ + "/mission/events", rclcpp::QoS(50).reliable());

  load_service_ = create_service<uav_interfaces::srv::LoadMission>(
    prefix_ + "/mission/load",
    [this](
      const std::shared_ptr<uav_interfaces::srv::LoadMission::Request> request,
      std::shared_ptr<uav_interfaces::srv::LoadMission::Response> response) {
      handleLoadMission(request, response);
    }, rmw_qos_profile_services_default, io_group_);

  pause_service_ = create_service<uav_interfaces::srv::PauseMission>(
    prefix_ + "/mission/pause",
    [this](
      const std::shared_ptr<uav_interfaces::srv::PauseMission::Request> request,
      std::shared_ptr<uav_interfaces::srv::PauseMission::Response> response) {
      handlePauseMission(request, response);
    }, rmw_qos_profile_services_default, io_group_);

  resume_service_ = create_service<uav_interfaces::srv::ResumeMission>(
    prefix_ + "/mission/resume",
    [this](
      const std::shared_ptr<uav_interfaces::srv::ResumeMission::Request> request,
      std::shared_ptr<uav_interfaces::srv::ResumeMission::Response> response) {
      handleResumeMission(request, response);
    }, rmw_qos_profile_services_default, io_group_);

  abort_service_ = create_service<uav_interfaces::srv::AbortMission>(
    prefix_ + "/mission/abort",
    [this](
      const std::shared_ptr<uav_interfaces::srv::AbortMission::Request> request,
      std::shared_ptr<uav_interfaces::srv::AbortMission::Response> response) {
      handleAbortMission(request, response);
    }, rmw_qos_profile_services_default, io_group_);

  execute_mission_server_ = rclcpp_action::create_server<ExecuteMission>(
    this, prefix_ + "/mission/execute_mission",
    [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteMission::Goal> goal) {
      return handleGoal(uuid, goal);
    },
    [this](const std::shared_ptr<GoalHandleExecuteMission> goal_handle) {
      return handleCancel(goal_handle);
    },
    [this](const std::shared_ptr<GoalHandleExecuteMission> goal_handle) {
      handleAccepted(goal_handle);
    },
    rcl_action_server_get_default_options(), io_group_);

  takeoff_client_ = rclcpp_action::create_client<uav_interfaces::action::Takeoff>(
    this, prefix_ + "/planning/takeoff", io_group_);
  goto_pose_client_ = rclcpp_action::create_client<uav_interfaces::action::GotoPose>(
    this, prefix_ + "/planning/goto_pose", io_group_);
  hold_position_client_ = rclcpp_action::create_client<uav_interfaces::action::HoldPosition>(
    this, prefix_ + "/planning/hold_position", io_group_);
  follow_path_client_ = rclcpp_action::create_client<uav_interfaces::action::FollowPath>(
    this, prefix_ + "/planning/follow_path", io_group_);
  track_target_client_ = rclcpp_action::create_client<uav_interfaces::action::TrackTarget>(
    this, prefix_ + "/planning/track_target", io_group_);
  recover_client_ = rclcpp_action::create_client<uav_interfaces::action::Recover>(
    this, prefix_ + "/planning/recover", io_group_);
  land_client_ = rclcpp_action::create_client<uav_interfaces::action::Land>(
    this, prefix_ + "/planning/land", io_group_);

  registerMissionBtNodes(bt_factory_, this);

  publishStatus(true);   // initial IDLE, TransientLocal so a late joiner sees it immediately

  tick_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / tick_hz_),
    [this]() {onTick();}, tick_group_);

  RCLCPP_INFO(get_logger(), "mission_executor_node ready for %s", uav_id_.c_str());
}
catch (const std::invalid_argument & error)
{
  RCLCPP_FATAL(
    rclcpp::get_logger("mission_executor_node"), "refusing to start: %s", error.what());
  throw;
}

MissionPolicyParams MissionExecutorNode::declarePolicyParams()
{
  MissionPolicyParams params;
  params.authority_heartbeat_copy_sec = declare_parameter<double>(
    "authority_heartbeat_copy_sec", params.authority_heartbeat_copy_sec);
  params.authority_loss_grace_sec = declare_parameter<double>(
    "authority_loss_grace_sec", params.authority_loss_grace_sec);
  params.battery_warn_threshold =
    declare_parameter<double>("battery_warn_threshold", params.battery_warn_threshold);
  params.battery_min_start_threshold = declare_parameter<double>(
    "battery_min_start_threshold", params.battery_min_start_threshold);
  params.battery_warn_dwell_sec =
    declare_parameter<double>("battery_warn_dwell_sec", params.battery_warn_dwell_sec);
  params.battery_unknown_timeout_sec =
    declare_parameter<double>("battery_unknown_timeout_sec", params.battery_unknown_timeout_sec);
  params.battery_health_period_copy_sec = declare_parameter<double>(
    "battery_health_period_copy_sec", params.battery_health_period_copy_sec);
  params.paused_timeout_sec =
    declare_parameter<double>("paused_timeout_sec", params.paused_timeout_sec);
  params.localization_status_timeout_copy_sec = declare_parameter<double>(
    "localization_status_timeout_copy_sec", params.localization_status_timeout_copy_sec);
  params.max_step_retries = static_cast<uint32_t>(
    declare_parameter<int>("max_step_retries", static_cast<int>(params.max_step_retries)));
  params.retry_backoff_sec =
    declare_parameter<double>("retry_backoff_sec", params.retry_backoff_sec);
  // Shared with MissionRegistryParams (mission_params.yaml keeps ONE copy
  // of this key) -- declared exactly once, here; declareRegistryParams()
  // reads it back from policy_params_ instead of re-declaring it (a second
  // declare_parameter() with the same name throws). This relies on
  // policy_params_ being constructed before registry_params_, which is
  // guaranteed by their DECLARATION order in mission_executor_node.hpp,
  // not by the constructor's initializer-list order.
  params.navigator_goto_timeout_copy_sec = declare_parameter<double>(
    "navigator_goto_timeout_copy_sec", params.navigator_goto_timeout_copy_sec);
  params.step_timeout_sec = declare_parameter<double>("step_timeout_sec", params.step_timeout_sec);
  return params;
}

MissionRegistryParams MissionExecutorNode::declareRegistryParams()
{
  MissionRegistryParams params;
  params.min_alt_copy_m = declare_parameter<double>("min_alt_copy_m", params.min_alt_copy_m);
  params.max_alt_copy_m = declare_parameter<double>("max_alt_copy_m", params.max_alt_copy_m);
  params.goto_reach_copy_m =
    declare_parameter<double>("goto_reach_copy_m", params.goto_reach_copy_m);
  params.navigator_max_speed_copy_mps = declare_parameter<double>(
    "navigator_max_speed_copy_mps", params.navigator_max_speed_copy_mps);
  // See declarePolicyParams() comment: declared there, read back here.
  params.navigator_goto_timeout_copy_sec = policy_params_.navigator_goto_timeout_copy_sec;
  params.step_timeout_copy_sec = policy_params_.step_timeout_sec;   // 🟡-3: same shared-key pattern
  params.target_speed_max_mps =
    declare_parameter<double>("target_speed_max_mps", params.target_speed_max_mps);
  params.max_loops = declare_parameter<int>("max_loops", params.max_loops);
  return params;
}

// ---------------------------------------------------------- action server

rclcpp_action::GoalResponse MissionExecutorNode::handleGoal(
  const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const ExecuteMission::Goal> goal)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  // Y2 (2026-08-23, review round 1): phase_ alone is NOT enough -- it only
  // transitions away from kIdle inside handleAccepted(), a SEPARATE
  // io_group_ callback invoked LATER (io_group_ is Reentrant, so it can
  // run concurrently with ANOTHER handleGoal() call). Two goals sent back
  // to back can BOTH see phase_ == kIdle in their own handleGoal() calls,
  // in the WINDOW between the first goal's handleGoal() returning ACCEPT
  // and its handleAccepted() actually running (state_mutex_ is fully
  // released in between) -- a genuine TOCTOU, not a hypothetical one.
  // accepting_ closes it: set atomically in the SAME critical section as
  // this check, the instant a goal is accepted, and only cleared by
  // handleAccepted() (see its first line) once that reservation is
  // consumed one way or another (a running mission, or an early abort).
  if (phase_ != ExecPhase::kIdle || accepting_) {
    RCLCPP_WARN(get_logger(), "ExecuteMission REJECT: a mission goal is already running");
    return rclcpp_action::GoalResponse::REJECT;
  }
  bool known = false;
  for (const MissionDescriptor & descriptor : registry_.list()) {
    if (descriptor.mission_id == goal->mission_id) {
      known = true;
      break;
    }
  }
  if (!known) {
    RCLCPP_WARN(
      get_logger(), "ExecuteMission REJECT: unknown mission_id '%s'", goal->mission_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  accepting_ = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MissionExecutorNode::handleCancel(
  const std::shared_ptr<GoalHandleExecuteMission> /*goal_handle*/)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  operator_abort_requested_ = true;
  operator_abort_reason_ = "ExecuteMission goal canceled by client";
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MissionExecutorNode::handleAccepted(const std::shared_ptr<GoalHandleExecuteMission> goal_handle)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  // Y2: the reservation handleGoal() made is consumed HERE, unconditionally
  // and first -- regardless of which exit path this function takes below
  // (a running mission, or an early abort that leaves phase_ at kIdle).
  // Safe against a NEW handleGoal() racing in: both functions serialize on
  // the SAME state_mutex_, so no handleGoal() call can observe accepting_
  // between this line and whatever phase_ ends up as when this function
  // returns.
  accepting_ = false;
  const auto goal = goal_handle->get_goal();

  const double battery = freshBatteryOrNan(now().seconds());
  if (!isBatteryOkToStart(battery, policy_params_)) {
    auto result = std::make_shared<ExecuteMission::Result>();
    result->success = false;
    result->result_code = kResultAbortedLowBattery;
    result->message = "battery below battery_min_start_threshold (or unmeasured) at goal acceptance";
    goal_handle->abort(result);
    return;
  }

  // Y3 (2026-08-23, R30): refuse to start from an UNKNOWN home position --
  // without this, mission_home_ silently falls back to (0,0,0), which
  // reads as a perfectly plausible real position, not "not measured".
  // Better an honest goal rejection than a Finish leg that flies the drone
  // toward the origin because odometry never actually arrived.
  if (!odometryFresh()) {
    auto result = std::make_shared<ExecuteMission::Result>();
    result->success = false;
    result->result_code = kResultAbortedLostLocalization;
    result->message = "no fresh /state/odometry_fused at goal acceptance -- refusing to start "
      "from an unknown home position (R30: never silently substitute (0,0,0))";
    goal_handle->abort(result);
    return;
  }

  MissionPlan plan;
  try {
    plan = registry_.load(goal->mission_id, goal->mission_params);
  } catch (const std::exception & e) {
    auto result = std::make_shared<ExecuteMission::Result>();
    result->success = false;
    result->result_code = kResultAbortedInvalidGoal;
    result->message = std::string("mission_params rejected: ") + e.what();
    goal_handle->abort(result);
    return;
  }

  std::unique_ptr<BT::Tree> tree;
  try {
    tree = std::make_unique<BT::Tree>(bt_factory_.createTreeFromFile(plan.xml_path));
  } catch (const std::exception & e) {
    auto result = std::make_shared<ExecuteMission::Result>();
    result->success = false;
    result->result_code = kResultAbortedInvalidGoal;
    result->message = std::string("failed to build mission tree: ") + e.what();
    goal_handle->abort(result);
    return;
  }

  try {
    populateBlackboard(*tree, plan);
  } catch (const std::exception & e) {
    // R0: a mission XML/params bug must never take the whole node down --
    // a stack that can only survive well-formed missions cannot run ANY
    // other mission afterward either. BT::Blackboard::set() throws
    // BT::LogicError (a std::exception) when a value's type disagrees with
    // a port's type as already declared by the XML -- previously uncaught
    // here, escaping handleAccepted() into std::terminate() (G-M3 gate,
    // 2026-08-22, follow_target's Timeout::msec fix above).
    auto result = std::make_shared<ExecuteMission::Result>();
    result->success = false;
    result->result_code = kResultAbortedInternalError;
    result->message = std::string("failed to populate mission blackboard: ") + e.what();
    goal_handle->abort(result);
    return;
  }

  goal_handle_ = goal_handle;
  settling_goal_id_ = uuidToHex(goal_handle->get_goal_id());
  current_plan_ = plan;
  tree_ = std::move(tree);
  loop_mission_ = goal->loop;
  current_mission_id_ = goal->mission_id;
  current_step_name_.clear();
  current_step_index_ = 0;
  total_steps_ = plan.total_steps;
  last_result_code_ = kResultUnknown;
  last_reason_.clear();
  mission_start_time_ = now();
  resetStepRetryState();
  follow_search_attempts_ = 0;   // G-M4.4b: fresh loss-episode budget per mission run
  operator_abort_requested_ = false;
  operator_pause_requested_ = false;
  authority_seize_tracker_.reset();
  battery_warn_tracker_.reset();
  battery_unknown_tracker_.reset();
  paused_tracker_.reset();

  // odometryFresh() is guaranteed true here (Y3 gate above already refused
  // the goal otherwise) -- kept as the read path anyway, not
  // odometry_received_ directly, so this stays correct if that gate is
  // ever bypassed/refactored.
  mission_home_.x = odometryFresh() ? last_odometry_.pose.pose.position.x : 0.0;
  mission_home_.y = odometryFresh() ? last_odometry_.pose.pose.position.y : 0.0;
  mission_home_.z = odometryFresh() ? last_odometry_.pose.pose.position.z : 0.0;

  mission_state_ = uav_interfaces::msg::MissionStatus::STATE_RUNNING;
  publishEvent(uav_interfaces::msg::MissionEvent::EVENT_STARTED, "", "", kResultUnknown);

  phase_ = ExecPhase::kTakeoffSending;
  phase_goal_dispatched_ = false;
  publishStatus(true);
}

void MissionExecutorNode::populateBlackboard(BT::Tree & tree, const MissionPlan & plan)
{
  auto bb = tree.rootBlackboard();
  const double home_x = odometryFresh() ? last_odometry_.pose.pose.position.x : 0.0;
  const double home_y = odometryFresh() ? last_odometry_.pose.pose.position.y : 0.0;

  switch (plan.kind) {
    case MissionKind::kIndoorPatrol:
      bb->set("waypoints", plan.indoor_patrol.waypoints);
      bb->set("loops", static_cast<int>(plan.indoor_patrol.loops));
      bb->set("dwell_sec", plan.indoor_patrol.dwell_sec);
      break;
    case MissionKind::kFollowTarget:
      // BT.CPP's built-in Timeout decorator declares msec as `unsigned`
      // (decorators/timeout_node.h) -- an `int` here throws BT::LogicError
      // ("type shall not change") the first time this ever runs (G-M3 gate,
      // 2026-08-22: uncaught, std::terminate() killed the whole node).
      bb->set("timeout_msec", static_cast<unsigned int>(plan.follow_target.timeout_sec * 1000.0));
      // No mission_registry field for target_id (P9.2/P9.3 froze that
      // schema without it) -- node-level default: track whichever target
      // is first seen. See README.md "P9.4 lech plan".
      bb->set("target_id", -1);
      bb->set("standoff_m", plan.follow_target.standoff_m);
      bb->set("search_x1", home_x + search_offset_m_);
      bb->set("search_y1", home_y);
      bb->set("search_x2", home_x - search_offset_m_);
      bb->set("search_y2", home_y);
      bb->set("search_z", takeoff_altitude_m_);
      break;
    case MissionKind::kInspectPoint:
      bb->set("approach_x", plan.inspect_point.approach.x);
      bb->set("approach_y", plan.inspect_point.approach.y);
      bb->set("approach_z", plan.inspect_point.approach.z);
      bb->set("marker_id", plan.inspect_point.marker_id);
      bb->set("search_x1", plan.inspect_point.approach.x + search_offset_m_);
      bb->set("search_y1", plan.inspect_point.approach.y);
      bb->set("search_x2", plan.inspect_point.approach.x - search_offset_m_);
      bb->set("search_y2", plan.inspect_point.approach.y);
      bb->set("dwell_sec", plan.inspect_point.dwell_sec);
      break;
  }
}

void MissionExecutorNode::settleGoalHandle(
  bool success, uint8_t result_code, const std::string & message)
{
  if (!goal_handle_) {
    return;
  }
  auto result = std::make_shared<ExecuteMission::Result>();
  result->success = success;
  result->result_code = result_code;
  result->message = message;
  result->steps_completed = current_step_index_;
  result->duration_seconds = static_cast<float>((now() - mission_start_time_).seconds());
  if (success) {
    goal_handle_->succeed(result);
  } else if (goal_handle_->is_canceling()) {
    goal_handle_->canceled(result);
  } else {
    goal_handle_->abort(result);
  }
  goal_handle_.reset();
}

// ---------------------------------------------------------------- services

bool MissionExecutorNode::targetsRunningMission(
  const std::string & mission_id, std::string * out_message) const
{
  if (phase_ == ExecPhase::kIdle) {
    *out_message = "no mission is running";
    return false;
  }
  if (!mission_id.empty() && mission_id != current_mission_id_) {
    *out_message = "mission_id '" + mission_id + "' does not match the running mission '" +
      current_mission_id_ + "'";
    return false;
  }
  return true;
}

void MissionExecutorNode::handleLoadMission(
  const std::shared_ptr<uav_interfaces::srv::LoadMission::Request> request,
  std::shared_ptr<uav_interfaces::srv::LoadMission::Response> response)
{
  try {
    const MissionPlan plan = registry_.load(request->mission_id, request->mission_params);
    response->success = true;
    response->result_code = kResultSucceeded;
    response->message = "loaded (preview only, mission not started)";
    response->total_steps = plan.total_steps;
  } catch (const std::exception & e) {
    response->success = false;
    response->result_code = kResultAbortedInvalidGoal;
    response->message = e.what();
    response->total_steps = 0;
  }
}

void MissionExecutorNode::handlePauseMission(
  const std::shared_ptr<uav_interfaces::srv::PauseMission::Request> request,
  std::shared_ptr<uav_interfaces::srv::PauseMission::Response> response)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  std::string message;
  if (!targetsRunningMission(request->mission_id, &message)) {
    response->success = false;
    response->result_code = kResultAbortedInvalidGoal;
    response->message = message;
    return;
  }
  if (mission_state_ == uav_interfaces::msg::MissionStatus::STATE_PAUSED) {
    response->success = true;
    response->result_code = kResultSucceeded;
    response->message = "already paused";
    return;
  }
  operator_pause_requested_ = true;
  response->success = true;
  response->result_code = kResultSucceeded;
  response->message = "pause requested, taking effect within one tick";
}

void MissionExecutorNode::handleResumeMission(
  const std::shared_ptr<uav_interfaces::srv::ResumeMission::Request> request,
  std::shared_ptr<uav_interfaces::srv::ResumeMission::Response> response)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  std::string message;
  if (!targetsRunningMission(request->mission_id, &message)) {
    response->success = false;
    response->result_code = kResultAbortedInvalidGoal;
    response->message = message;
    return;
  }
  if (mission_state_ != uav_interfaces::msg::MissionStatus::STATE_PAUSED) {
    response->success = false;
    response->result_code = kResultAbortedInvalidGoal;
    response->message = "mission is not paused";
    return;
  }
  // current_step_index_/tree_ are untouched by pause -- the tree simply was
  // not ticked (plan P9 S:5 gate criterion (d)): resuming continues the
  // SAME leaf's RUNNING state, no reset.
  phase_ = paused_from_phase_;
  phase_goal_dispatched_ = false;
  mission_state_ = uav_interfaces::msg::MissionStatus::STATE_RUNNING;
  paused_tracker_.reset();
  battery_warn_notified_while_paused_ = false;   // Y6: fresh latch if paused again later
  publishEvent(
    uav_interfaces::msg::MissionEvent::EVENT_RESUMED, current_step_name_, "", kResultUnknown);
  response->success = true;
  response->result_code = kResultSucceeded;
  response->message = "resumed";
}

void MissionExecutorNode::handleAbortMission(
  const std::shared_ptr<uav_interfaces::srv::AbortMission::Request> request,
  std::shared_ptr<uav_interfaces::srv::AbortMission::Response> response)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  std::string message;
  if (!targetsRunningMission(request->mission_id, &message)) {
    response->success = false;
    response->result_code = kResultAbortedInvalidGoal;
    response->message = message;
    return;
  }
  operator_abort_requested_ = true;
  operator_abort_reason_ = request->reason.empty() ? "AbortMission service call" : request->reason;
  response->success = true;
  response->result_code = kResultSucceeded;
  response->message = "abort requested, taking effect within one tick";
}

// ------------------------------------------------------------------ tick

void MissionExecutorNode::onTick()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  const double now_sec = now().seconds();

  if (phase_ == ExecPhase::kIdle) {
    authority_seize_tracker_.reset();
    battery_warn_tracker_.reset();
    battery_unknown_tracker_.reset();
    paused_tracker_.reset();
    battery_warn_notified_while_paused_ = false;   // Y6: fresh latch for the next mission
    authority_seize_warned_this_episode_ = false;
    return;
  }

  if (phase_ == ExecPhase::kCanceling) {
    tickCanceling(now_sec);
    publishStatus(false);
    return;
  }

  if (phase_ == ExecPhase::kPaused) {
    const MissionWorldView view = buildTopLevelView(now_sec);
    const GuardVerdict verdict = policy_.evaluate(view);
    if (verdict.action == GuardAction::kAbortHold) {
      finishAsAborted(verdict.result_code, verdict.reason, now_sec);
      publishStatus(false);
      return;
    }
    // Y6 (2026-08-23, review round 1, coordinator decision): PAUSED means
    // "an operator/guard already ordered a stop" -- this node still must
    // NOT auto-fly out of it for battery reasons (CRITICAL-level battery
    // failsafes remain PX4's own job, untouched here); logic is
    // deliberately UNCHANGED. What was missing is OBSERVABILITY: a
    // battery-WARN (or -unknown) condition arising WHILE already paused
    // was previously silent -- checked raw view fields directly (not
    // verdict.action) so a HIGHER-priority authority-seize masking the
    // returned verdict can never also mask this notification. Edge-
    // latched (battery_warn_notified_while_paused_) so it fires ONCE per
    // continuous WARN episode, not every 10 Hz tick. No dedicated
    // MissionEvent type exists for "still paused, now also a warning"
    // (uav_interfaces is a different package, out of scope here) --
    // reuses EVENT_PAUSED with a distinguishing description, see
    // README.md/contract §2.19.
    const bool battery_warn_now =
      view.battery_warn_elapsed_sec > 0.0 || view.battery_unknown_elapsed_sec > 0.0;
    if (battery_warn_now && !battery_warn_notified_while_paused_) {
      battery_warn_notified_while_paused_ = true;
      publishEvent(
        uav_interfaces::msg::MissionEvent::EVENT_PAUSED, current_step_name_,
        "battery WARN/unknown while mission PAUSED -- staying paused, NOT auto-flying "
        "(PX4/CRITICAL failsafes still apply); awaiting operator ResumeMission/AbortMission",
        kResultUnknown);
    } else if (!battery_warn_now) {
      battery_warn_notified_while_paused_ = false;
    }
    publishStatus(false);
    return;
  }

  // Finish/Recovering: full priority chain NOT re-evaluated every tick
  // (battery/pause livelock), operator_abort always honored,
  // authority-seize honored EXCEPT kFinishLanding (PX4-native-LAND owns
  // authority there). Full rationale -> README.md "kien truc that" (B1 +
  // G-M1 review round 1 entries), not repeated here (🟡-6).
  if (phase_ == ExecPhase::kFinishGotoHome || phase_ == ExecPhase::kFinishLanding ||
    phase_ == ExecPhase::kRecovering)
  {
    if (operator_abort_requested_) {
      cancelCurrentGoal();
      if (broker_.busy()) {
        publishStatus(false);
        return;
      }
      finishAsAborted(kResultCanceled, operator_abort_reason_, now_sec);
      publishStatus(true);
      return;
    }

    // kFinishLanding exempt (PX4-native-LAND legitimately owns authority
    // while landing, active_source=NONE there is expected, not a seize) --
    // root cause + rationale -> README.md "G-M1 review round 1" (🟡-6).
    if (phase_ != ExecPhase::kFinishLanding) {
      // Isolate PRIORITY 2 ONLY: every other field is forced structurally
      // inert (same technique evaluateStepFailure() already uses for
      // priorities 1/3/4), so a non-kContinue verdict here can ONLY be the
      // authority-seize branch -- battery/pause staying inert means this
      // call can never livelock the way the suppressed chain would have.
      const bool authority_stale =
        (authority_received_ ? (now_sec - authority_arrival_.seconds()) :
        std::numeric_limits<double>::infinity()) > 2.0 * policy_params_.authority_heartbeat_copy_sec;
      const bool authority_not_held =
        !authority_received_ || last_authority_.active_source != kSourceMission || authority_stale;
      MissionWorldView authority_view;
      authority_view.mission_in_flight = true;
      authority_view.operator_abort_requested = false;   // handled above, own dedicated branch
      authority_view.authority_active_source = authority_received_ ?
        last_authority_.active_source : kSourceNone;
      authority_view.authority_age_sec = authority_received_ ?
        (now_sec - authority_arrival_.seconds()) : std::numeric_limits<double>::infinity();
      authority_view.authority_seize_elapsed_sec =
        authority_seize_tracker_.update(authority_not_held, now_sec);
      authority_view.battery_remaining = freshBatteryOrNan(now_sec);
      authority_view.battery_warn_elapsed_sec = 0.0;      // forced inert -- priority 3 never fires here
      authority_view.battery_unknown_elapsed_sec = 0.0;   // forced inert -- priority 3 never fires here
      authority_view.operator_pause_requested = false;    // forced inert -- priority 4 never fires here
      authority_view.paused = false;                      // forced inert -- priority 4 never fires here
      authority_view.paused_elapsed_sec = 0.0;
      authority_view.localization_valid = localization_received_ && last_localization_.is_valid;
      authority_view.localization_age_sec = localization_received_ ?
        (now_sec - localization_arrival_.seconds()) : std::numeric_limits<double>::infinity();
      authority_view.last_nav_result_code = kResultUnknown;   // priority 5 never fires here either

      const GuardVerdict authority_verdict = policy_.evaluate(authority_view);
      if (authority_verdict.action != GuardAction::kContinue) {
        // Downgraded (2026-08-23): was WARN every tick while the verdict
        // kept firing (e.g. the whole CANCELING wait); now logs ONCE per
        // episode, right before this guard actually acts -- proved its
        // worth pinning the root cause above, kept for the next
        // hard-to-reproduce case, not as a steady-state hot-path log.
        if (!authority_seize_warned_this_episode_) {
          authority_seize_warned_this_episode_ = true;
          RCLCPP_WARN(
            get_logger(),
            "Finish/Recovering authority-seize verdict firing: phase=%d authority_received=%d "
            "active_source=%u age_sec=%.3f authority_not_held=%d seize_elapsed_sec=%.3f "
            "loss_grace_sec=%.3f now_sec=%.3f authority_arrival_sec=%.3f reason='%s'",
            static_cast<int>(phase_), authority_received_ ? 1 : 0,
            static_cast<unsigned>(authority_view.authority_active_source),
            authority_view.authority_age_sec, authority_not_held ? 1 : 0,
            authority_view.authority_seize_elapsed_sec, policy_params_.authority_loss_grace_sec,
            now_sec, authority_arrival_.seconds(), authority_verdict.reason.c_str());
        }
        cancelCurrentGoal();
        if (broker_.busy()) {
          publishStatus(false);
          return;
        }
        // paused_from_phase_/pending_termination_ handling rationale ->
        // README.md "kien truc that" (B1 entry) -- not repeated here (🟡-6).
        paused_from_phase_ = phase_;
        enterPaused(authority_verdict.reason, authority_verdict.result_code, now_sec);
        publishStatus(true);
        return;
      }
      authority_seize_warned_this_episode_ = false;   // predicate cleared -- fresh episode next time
    }

    if (phase_ == ExecPhase::kFinishGotoHome) {
      tickFinishGotoHome(now_sec);
    } else if (phase_ == ExecPhase::kFinishLanding) {
      tickFinishLanding(now_sec);
    } else {
      tickRecovering(now_sec);
    }
    publishStatus(false);
    return;
  }

  const MissionWorldView view = buildTopLevelView(now_sec);
  const GuardVerdict verdict = policy_.evaluate(view);
  if (verdict.action != GuardAction::kContinue) {
    beginCancelFor(verdict, now_sec);
    publishStatus(true);
    return;
  }

  switch (phase_) {
    case ExecPhase::kTakeoffSending:
      tickTakeoff(now_sec);
      break;
    case ExecPhase::kBody:
      tickBody(now_sec);
      break;
    default:
      break;
  }
  publishStatus(false);
}

double MissionExecutorNode::freshBatteryOrNan(double now_sec) const
{
  const bool health_fresh = health_received_ &&
    (now_sec - health_arrival_.seconds()) <= 2.0 * policy_params_.battery_health_period_copy_sec;
  return health_fresh ?
    static_cast<double>(last_health_.battery_remaining) : std::numeric_limits<double>::quiet_NaN();
}

MissionWorldView MissionExecutorNode::buildTopLevelView(double now_sec)
{
  MissionWorldView view;
  view.mission_in_flight = (phase_ != ExecPhase::kIdle);
  view.operator_abort_requested = operator_abort_requested_;

  view.authority_active_source = authority_received_ ? last_authority_.active_source : kSourceNone;
  view.authority_age_sec = authority_received_ ?
    (now_sec - authority_arrival_.seconds()) : std::numeric_limits<double>::infinity();
  const bool authority_stale =
    view.authority_age_sec > 2.0 * policy_params_.authority_heartbeat_copy_sec;
  const bool authority_not_held = (view.authority_active_source != kSourceMission) || authority_stale;
  view.authority_seize_elapsed_sec = authority_seize_tracker_.update(authority_not_held, now_sec);

  const double battery = freshBatteryOrNan(now_sec);
  view.battery_remaining = battery;
  const bool battery_finite = std::isfinite(battery);
  view.battery_warn_elapsed_sec = battery_warn_tracker_.update(
    battery_finite && battery < policy_params_.battery_warn_threshold, now_sec);
  view.battery_unknown_elapsed_sec = battery_unknown_tracker_.update(!battery_finite, now_sec);

  view.operator_pause_requested = operator_pause_requested_;
  view.paused = (mission_state_ == uav_interfaces::msg::MissionStatus::STATE_PAUSED);
  view.paused_elapsed_sec = paused_tracker_.update(view.paused, now_sec);

  view.localization_valid = localization_received_ && last_localization_.is_valid;
  view.localization_age_sec = localization_received_ ?
    (now_sec - localization_arrival_.seconds()) : std::numeric_limits<double>::infinity();
  // Priority 5 is evaluated inside NavAction (evaluateStepFailure()), never
  // here -- see header point 3. kResultUnknown never satisfies
  // isTerminalFailureCode(), so priority 5 can never fire from this view.
  view.last_nav_result_code = kResultUnknown;

  return view;
}

void MissionExecutorNode::beginCancelFor(const GuardVerdict & verdict, double /*now_sec*/)
{
  cancelCurrentGoal();
  paused_from_phase_ = phase_;
  pending_termination_.kind = (verdict.action == GuardAction::kPause) ?
    PendingTermination::Kind::kPause :
    (verdict.action == GuardAction::kAbortHold) ?
    PendingTermination::Kind::kAbortHold : PendingTermination::Kind::kFinish;
  pending_termination_.result_code = verdict.result_code;
  pending_termination_.reason = verdict.reason;
  phase_ = ExecPhase::kCanceling;
  phase_goal_dispatched_ = false;
}

void MissionExecutorNode::tickCanceling(double now_sec)
{
  if (broker_.busy()) {
    return;   // still waiting for the cancel to land, or the goal to end naturally
  }
  const PendingTermination pending = pending_termination_;
  pending_termination_ = PendingTermination{};
  switch (pending.kind) {
    case PendingTermination::Kind::kPause:
      enterPaused(pending.reason, pending.result_code, now_sec);
      break;
    case PendingTermination::Kind::kAbortHold:
      finishAsAborted(pending.result_code, pending.reason, now_sec);
      break;
    case PendingTermination::Kind::kFinish:
      beginFinish(pending.result_code, pending.reason, now_sec);
      break;
    default:
      phase_ = ExecPhase::kIdle;   // defensive: should be unreachable
      break;
  }
}

void MissionExecutorNode::enterPaused(
  const std::string & reason, uint8_t result_code, double /*now_sec*/)
{
  phase_ = ExecPhase::kPaused;
  mission_state_ = uav_interfaces::msg::MissionStatus::STATE_PAUSED;
  last_result_code_ = result_code;
  last_reason_ = reason;
  operator_pause_requested_ = false;
  publishEvent(uav_interfaces::msg::MissionEvent::EVENT_PAUSED, current_step_name_, reason, result_code);
}

void MissionExecutorNode::tickTakeoff(double now_sec)
{
  if (!phase_goal_dispatched_) {
    phase_goal_dispatched_ = dispatchTakeoff(takeoff_altitude_m_);
    return;
  }
  std::optional<GoalOutcome> outcome = broker_.takeOutcome();
  if (!outcome.has_value()) {
    return;
  }
  if (!outcome->succeeded) {
    finishAsAborted(
      outcome->canceled ? kResultCanceled : outcome->result_code,
      "takeoff failed: " + outcome->message, now_sec);
    return;
  }
  phase_ = ExecPhase::kBody;
  phase_goal_dispatched_ = false;
}

void MissionExecutorNode::tickBody(double now_sec)
{
  const BT::NodeStatus status = tree_->tickRoot();
  if (status == BT::NodeStatus::RUNNING) {
    return;
  }
  if (status == BT::NodeStatus::SUCCESS) {
    if (loop_mission_) {
      return;   // BT.CPP restarts a Sequence/Repeat from its first child on the next tick
    }
    // bug #12 (2026-08-23): follow_target's body has NO legitimate "done"
    // state -- track_when_seen's TrackTarget always runs duration_seconds
    // =0.0 ("track until told otherwise"), so the ONLY way tickRoot() ever
    // returns SUCCESS for this mission kind is search_when_lost's
    // reacquire-recheck succeeding. That must resume TRACKING on the next
    // tick, never end the mission -- conflating "navigation to a waypoint
    // succeeded" with "target found" was the root of bug #12; conflating
    // "target genuinely re-found" with "mission accomplished" would just
    // be the same mistake wearing a different hat. In THIS tree,
    // ReactiveFallback re-ticks track_when_seen from index 0 on EVERY
    // external tick, so in practice track_when_seen's OWN TargetSeen
    // always observes a reacquisition first and wins the race BEFORE
    // search_when_lost's recheck ever gets a chance to succeed (both read
    // the SAME state_mutex_-protected snapshot, so they can never
    // disagree within one tick) -- this branch is a defensive backstop,
    // not the path real reacquisition takes today, kept correct in case
    // that tree shape ever changes.
    if (current_plan_.kind == MissionKind::kFollowTarget) {
      return;   // tree_ restarts fresh next tick, same mechanism loop_mission_ uses above
    }
    beginFinish(kResultSucceeded, "mission body completed", now_sec);
    return;
  }

  // FAILURE.
  if (pending_terminal_verdict_.has_value()) {
    const GuardVerdict verdict = *pending_terminal_verdict_;
    pending_terminal_verdict_.reset();
    // G-M4.4b (2026-08-23): follow_target's search-attempts budget just ran
    // out THIS tick -- the ONLY way the whole tree can return FAILURE with
    // a stored ABORTED_LOST_TARGET verdict still intact is
    // SearchBudgetAvailable blocking search_when_lost before any NavAction
    // in that leg ever reaches onStart() (which would otherwise have wiped
    // pending_terminal_verdict_ via resetStepRetryState() -- see
    // mission_context.hpp). Route to Recover(CLIMB) instead of the normal
    // Finish/abort path; searchBudgetAvailable() is re-checked here (not
    // just trusted from the verdict) so a stale/unrelated LOST_TARGET
    // verdict from BEFORE the budget was exhausted can never mis-route.
    if (verdict.result_code == kResultAbortedLostTarget && !searchBudgetAvailable()) {
      beginRecovering(verdict.reason, now_sec);
    } else if (verdict.action == GuardAction::kAbortLand) {
      beginFinish(verdict.result_code, verdict.reason, now_sec);
    } else {
      finishAsAborted(verdict.result_code, verdict.reason, now_sec);
    }
    return;
  }

  // bug #12 (2026-08-23): search_when_lost's reacquire-recheck failed to
  // find the target THIS round, with NO terminal verdict recorded. That
  // can only happen when SearchBudgetAvailable did NOT block this round
  // (it ALWAYS records a verdict via noteSearchBudgetExhausted() when it
  // does, handled by the branch above) -- i.e. one search round genuinely
  // ran and found nothing, budget not yet exhausted. Not a mission-ending
  // condition -- retry next external tick, where SearchBudgetAvailable
  // will itself correctly decide (and record a verdict) once the budget
  // truly runs out.
  //
  // Distinguishing THIS from a genuine outer <Timeout> expiry (which ALSO
  // returns FAILURE with no verdict) via an INDEPENDENT, never-reset C++
  // clock (mission_start_time_) rather than any BT-internal signal: a
  // round trying-and-failing tick was FIRST attempted with a tag on the
  // tick SearchBudgetAvailable itself ran (search_round_evaluated_this_
  // tick_), but that is WRONG -- a search round routinely spans MULTIPLE
  // external ticks (SearchBudgetAvailable only ticks on the FIRST one),
  // so by the tick the recheck actually fails, that flag has long since
  // been reset, making every retry misroute to the catch-all (caught by
  // TDD test (p) below, genuinely red). mission_start_time_ never resets
  // on a tree restart -- unlike the XML <Timeout> node's OWN deadline,
  // which DOES reset (BT.CPP resets root state to IDLE after a terminal
  // status, so Timeout's onStart() reruns) -- trusting anything
  // BT-internal here would silently defeat the outer safety bound on
  // every retry.
  if (current_plan_.kind == MissionKind::kFollowTarget &&
    (now_sec - mission_start_time_.seconds()) < current_plan_.follow_target.timeout_sec)
  {
    return;
  }

  // A FAILURE with no recorded terminal verdict -- e.g. follow_target's
  // Timeout decorator expiring, or (for other mission kinds) a Fallback
  // exhausting without ever recording a verdict. Never a NavAction
  // retry-exhaustion (that path always records pending_terminal_verdict_
  // via evaluateStepFailure()). Apply the SAME Q-P9-1 gate directly rather
  // than assuming it is safe to fly home.
  const bool authority_ok = authority_received_ &&
    last_authority_.active_source == kSourceMission &&
    (now_sec - authority_arrival_.seconds()) <= 2.0 * policy_params_.authority_heartbeat_copy_sec;
  const bool localization_ok = localization_received_ && last_localization_.is_valid &&
    (now_sec - localization_arrival_.seconds()) <= policy_params_.localization_status_timeout_copy_sec;
  if (authority_ok && localization_ok) {
    beginFinish(kResultAbortedTimeout, "mission body ended without succeeding", now_sec);
  } else {
    finishAsAborted(
      kResultAbortedTimeout, "mission body ended without succeeding, unsafe to fly home", now_sec);
  }
}

void MissionExecutorNode::beginFinish(uint8_t result_code, const std::string & reason, double /*now_sec*/)
{
  pending_termination_.kind = PendingTermination::Kind::kFinish;
  pending_termination_.result_code = result_code;
  pending_termination_.reason = reason;
  phase_ = ExecPhase::kFinishGotoHome;
  phase_goal_dispatched_ = false;
}

void MissionExecutorNode::tickFinishGotoHome(double /*now_sec*/)
{
  if (!phase_goal_dispatched_) {
    const double z = odometryFresh() ? last_odometry_.pose.pose.position.z : mission_home_.z;
    phase_goal_dispatched_ = sendGotoPose(
      mission_home_.x, mission_home_.y, z, odom_frame_,
      finish_acceptance_radius_m_, finish_goto_speed_mps_);
    return;
  }
  std::optional<GoalOutcome> outcome = broker_.takeOutcome();
  if (!outcome.has_value()) {
    return;
  }
  if (!outcome->succeeded) {
    pending_termination_.reason += " [return-to-home leg failed: " + outcome->message + "]";
  }
  phase_ = ExecPhase::kFinishLanding;
  phase_goal_dispatched_ = false;
}

void MissionExecutorNode::tickFinishLanding(double now_sec)
{
  if (!phase_goal_dispatched_) {
    phase_goal_dispatched_ = dispatchLand();
    return;
  }
  std::optional<GoalOutcome> outcome = broker_.takeOutcome();
  if (!outcome.has_value()) {
    return;
  }
  const uint8_t code = pending_termination_.result_code;
  const std::string reason = pending_termination_.reason;
  pending_termination_ = PendingTermination{};

  if (!outcome->succeeded) {
    finishAsAborted(
      code == kResultSucceeded ? outcome->result_code : code,
      reason + " [landing leg failed: " + outcome->message + "]", now_sec);
    return;
  }
  if (code == kResultSucceeded) {
    finishAsCompleted(now_sec);
  } else {
    finishAsAborted(code, reason, now_sec);
  }
}

void MissionExecutorNode::finishAsAborted(
  uint8_t result_code, const std::string & reason, double /*now_sec*/)
{
  phase_ = ExecPhase::kIdle;
  phase_goal_dispatched_ = false;
  mission_state_ = uav_interfaces::msg::MissionStatus::STATE_ABORTED;
  last_result_code_ = result_code;
  last_reason_ = reason;
  publishEvent(
    uav_interfaces::msg::MissionEvent::EVENT_ABORTED, current_step_name_, reason, result_code);
  // Publish the terminal status BEFORE settling the goal (which resets
  // goal_handle_) -- contract 2.19, see settleGoalHandle() doc comment.
  publishStatus(true);
  settleGoalHandle(false, result_code, reason);
  operator_abort_requested_ = false;
  operator_pause_requested_ = false;
}

void MissionExecutorNode::finishAsCompleted(double /*now_sec*/)
{
  phase_ = ExecPhase::kIdle;
  phase_goal_dispatched_ = false;
  mission_state_ = uav_interfaces::msg::MissionStatus::STATE_COMPLETED;
  last_result_code_ = kResultSucceeded;
  last_reason_.clear();
  publishEvent(
    uav_interfaces::msg::MissionEvent::EVENT_COMPLETED, current_step_name_, "", kResultSucceeded);
  // Same ordering fix as finishAsAborted() -- see that comment.
  publishStatus(true);
  settleGoalHandle(true, kResultSucceeded, "mission completed");
}

void MissionExecutorNode::beginRecovering(const std::string & reason, double /*now_sec*/)
{
  recovering_reason_ = reason;
  phase_ = ExecPhase::kRecovering;
  phase_goal_dispatched_ = false;
}

void MissionExecutorNode::tickRecovering(double now_sec)
{
  if (!phase_goal_dispatched_) {
    phase_goal_dispatched_ = dispatchRecover();
    return;
  }
  std::optional<GoalOutcome> outcome = broker_.takeOutcome();
  if (!outcome.has_value()) {
    return;
  }
  // Plan P9 S:2.5: "Recover(CLIMB) -> abort" -- the mission's own terminal
  // result is ALWAYS ABORTED_LOST_TARGET (the reason the episode budget ran
  // out), never Recover's own result_code; Recover(CLIMB) is a best-effort
  // safety leg here, not itself the thing the mission reports as its
  // outcome. A failed Recover leg is folded into the reason text only.
  std::string reason = recovering_reason_;
  if (!outcome->succeeded) {
    reason += " [Recover(CLIMB) leg: " + outcome->message + "]";
  }
  finishAsAborted(kResultAbortedLostTarget, reason, now_sec);
}

// ------------------------------------------------------- navigator dispatch

template<typename ActionT>
bool MissionExecutorNode::sendTypedGoal(
  typename rclcpp_action::Client<ActionT>::SharedPtr client, const typename ActionT::Goal & goal)
{
  if (!client) {
    return false;
  }
  return broker_.request(
    [this, client, goal]() {
      typename rclcpp_action::Client<ActionT>::SendGoalOptions options;
      options.goal_response_callback =
        [this, client](typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr handle) {
          if (!handle) {
            broker_.onGoalRejected(kResultAbortedInvalidGoal, "navigator rejected the goal");
            return;
          }
          {
            std::lock_guard<std::mutex> lock(cancel_fn_mutex_);
            current_cancel_fn_ = [this, client, handle]() {
              try {
                client->async_cancel_goal(
                  handle,
                  [this](typename rclcpp_action::Client<ActionT>::CancelResponse::SharedPtr response) {
                    const bool accepted = response != nullptr &&
                      response->return_code ==
                      rclcpp_action::Client<ActionT>::CancelResponse::ERROR_NONE &&
                      !response->goals_canceling.empty();
                    if (accepted) {
                      broker_.onCancelAccepted();
                    } else {
                      broker_.onCancelRefused();
                    }
                  });
              } catch (const std::exception &) {
                // The goal already reached a terminal state, racing this
                // cancel -- onResult() still fires for it independently.
                broker_.onCancelRefused();
              }
            };
          }
          broker_.onGoalAccepted();
        };
      options.result_callback =
        [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & wrapped) {
          GoalOutcome outcome;
          switch (wrapped.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
            case rclcpp_action::ResultCode::ABORTED:
              // G-M4.4b gate bug (2026-08-23): navigator reports EVERY
              // domain-specific outcome (LOST_TARGET, INVALID_GOAL, SAFETY,
              // LOST_LOCALIZATION, NO_AUTHORITY, ...) through its OWN
              // Result.result_code, never through the ROS-level SUCCEEDED
              // vs ABORTED distinction alone -- a server that judges a goal
              // domain-unachievable calls abort() per normal ROS2 idiom
              // (navigator_action_server_node.cpp does, throughout) while
              // STILL filling in a meaningful result_code/message. Treating
              // ABORTED as "discard wrapped.result, use a generic code"
              // silently replaced EVERY real domain code with
              // ABORTED_INTERNAL_ERROR on the wire, defeating
              // mission_policy's entire per-code branching (never-retry
              // SAFETY/INVALID_GOAL, LOST_LOCALIZATION/NO_AUTHORITY special
              // cases) against the real navigator -- previously invisible
              // because isTerminalFailureCode() treats
              // ABORTED_INTERNAL_ERROR as terminal-ish too, so the GENERIC
              // retry-then-abort path still ran, just on the wrong code.
              if (wrapped.result) {
                outcome.succeeded = wrapped.result->success;
                outcome.result_code = wrapped.result->result_code;
                outcome.message = wrapped.result->message;
              } else {
                outcome.result_code = kResultAbortedInternalError;
                outcome.message = "navigator action client reported ABORTED with no result";
              }
              break;
            case rclcpp_action::ResultCode::CANCELED:
              outcome.canceled = true;
              outcome.result_code = kResultCanceled;
              outcome.message = "canceled";
              break;
            default:
              // A genuine transport-level UNKNOWN (no result at all) -- only
              // here is a synthesized code appropriate.
              outcome.result_code = kResultAbortedInternalError;
              outcome.message = "navigator action client reported UNKNOWN (no result)";
          }
          broker_.onResult(outcome);
        };
      client->async_send_goal(goal, options);
    });
}

geometry_msgs::msg::Quaternion MissionExecutorNode::currentHeadingOrientation()
{
  double yaw = 0.0;
  if (odometryFresh()) {
    const auto & q = last_odometry_.pose.pose.orientation;
    if (tryYawFromQuaternion(q.x, q.y, q.z, q.w, &yaw)) {
      return yawOnlyQuaternion(yaw);
    }
  }
  if (!yaw_fallback_warned_) {
    RCLCPP_WARN(
      get_logger(),
      "GotoPose yaw fallback to 0: odometry_fused unavailable/non-finite/stale (logged once)");
    yaw_fallback_warned_ = true;
  }
  return yawOnlyQuaternion(0.0);
}

bool MissionExecutorNode::sendGotoPose(
  double x, double y, double z, const std::string & frame_id, double acceptance_radius,
  double max_speed)
{
  uav_interfaces::action::GotoPose::Goal goal;
  goal.target_pose.position.x = x;
  goal.target_pose.position.y = y;
  goal.target_pose.position.z = z;
  goal.target_pose.orientation = currentHeadingOrientation();
  goal.frame_id = frame_id;
  goal.acceptance_radius = static_cast<float>(acceptance_radius);
  goal.max_speed = static_cast<float>(max_speed);
  return sendTypedGoal<uav_interfaces::action::GotoPose>(goto_pose_client_, goal);
}

bool MissionExecutorNode::sendTrackTarget(
  int32_t target_id, double standoff_distance, double duration_seconds, double target_lost_timeout)
{
  uav_interfaces::action::TrackTarget::Goal goal;
  goal.target_id = target_id;
  goal.standoff_distance = static_cast<float>(standoff_distance);
  goal.duration_seconds = static_cast<float>(duration_seconds);
  goal.target_lost_timeout = static_cast<float>(target_lost_timeout);
  return sendTypedGoal<uav_interfaces::action::TrackTarget>(track_target_client_, goal);
}

bool MissionExecutorNode::dispatchTakeoff(double target_altitude)
{
  uav_interfaces::action::Takeoff::Goal goal;
  goal.target_altitude = static_cast<float>(target_altitude);
  return sendTypedGoal<uav_interfaces::action::Takeoff>(takeoff_client_, goal);
}

bool MissionExecutorNode::dispatchLand()
{
  uav_interfaces::action::Land::Goal goal;
  goal.use_precision_landing = false;
  goal.marker_id = -1;
  return sendTypedGoal<uav_interfaces::action::Land>(land_client_, goal);
}

bool MissionExecutorNode::dispatchRecover()
{
  uav_interfaces::action::Recover::Goal goal;
  goal.recovery_type = uav_interfaces::action::Recover::Goal::TYPE_CLIMB_TO_SAFE_ALTITUDE;
  goal.safe_altitude = static_cast<float>(recover_climb_altitude_m_);
  goal.trigger_reason = "follow_target: search-attempts budget exhausted";
  return sendTypedGoal<uav_interfaces::action::Recover>(recover_client_, goal);
}

void MissionExecutorNode::cancelCurrentGoal()
{
  std::function<void()> fn;
  {
    std::lock_guard<std::mutex> lock(cancel_fn_mutex_);
    fn = current_cancel_fn_;
  }
  if (fn) {
    broker_.cancel(fn);
  }
}

// ------------------------------------------------------------- publishing

void MissionExecutorNode::publishStatus(bool force)
{
  const bool changed = (mission_state_ != last_published_state_);
  const bool due = (status_tick_count_ % kStatusEveryTicks == 0);
  ++status_tick_count_;
  if (!force && !changed && !due) {
    return;
  }

  uav_interfaces::msg::MissionStatus msg;
  msg.header.stamp = now();
  msg.uav_id = uav_id_;
  msg.state = mission_state_;
  msg.mission_id = current_mission_id_;
  msg.current_step_name = current_step_name_;
  msg.current_step_index = current_step_index_;
  msg.total_steps = total_steps_;
  msg.progress_percent = total_steps_ > 0 ?
    100.0f * static_cast<float>(current_step_index_) / static_cast<float>(total_steps_) : 0.0f;
  msg.elapsed_sec = (mission_state_ == uav_interfaces::msg::MissionStatus::STATE_IDLE) ?
    0.0f : static_cast<float>((now() - mission_start_time_).seconds());
  msg.last_result_code = last_result_code_;
  msg.last_reason = last_reason_;
  // Falls back to the remembered id, not to "": settleGoalHandle() has already
  // dropped goal_handle_ by the time the terminal sample goes out on three paths, and
  // that sample is the one that stays in the TransientLocal cache (contract 2.19).
  msg.goal_id = goal_handle_ ? uuidToHex(goal_handle_->get_goal_id()) : settling_goal_id_;

  status_publisher_->publish(msg);
  last_published_state_ = mission_state_;

  if (goal_handle_ &&
    (mission_state_ == uav_interfaces::msg::MissionStatus::STATE_RUNNING ||
    mission_state_ == uav_interfaces::msg::MissionStatus::STATE_PAUSED))
  {
    auto feedback = std::make_shared<ExecuteMission::Feedback>();
    feedback->current_step_name = current_step_name_;
    feedback->current_step_index = current_step_index_;
    feedback->total_steps = total_steps_;
    feedback->progress_percent = msg.progress_percent;
    goal_handle_->publish_feedback(feedback);
  }
}

void MissionExecutorNode::publishEvent(
  uint8_t event_type, const std::string & step_name, const std::string & description,
  uint8_t result_code)
{
  uav_interfaces::msg::MissionEvent msg;
  msg.header.stamp = now();
  msg.uav_id = uav_id_;
  msg.event_type = event_type;
  msg.mission_id = current_mission_id_;
  msg.step_name = step_name;
  msg.description = description;
  msg.result_code = result_code;
  events_publisher_->publish(msg);
}

// --------------------------------------------------------- MissionContext

double MissionExecutorNode::rosNowSeconds() const
{
  return now().seconds();
}

double MissionExecutorNode::retryBackoffSec() const
{
  return policy_params_.retry_backoff_sec;
}

double MissionExecutorNode::stepTimeoutSec() const
{
  return policy_params_.step_timeout_sec;
}

std::optional<GoalOutcome> MissionExecutorNode::pollNavActionOutcome()
{
  return broker_.takeOutcome();
}

void MissionExecutorNode::cancelNavActionGoal()
{
  cancelCurrentGoal();
}

void MissionExecutorNode::resetStepRetryState()
{
  step_retry_count_ = 0;
  have_last_failure_reason_ = false;
  last_failure_reason_.clear();
  pending_terminal_verdict_.reset();
}

GuardVerdict MissionExecutorNode::evaluateStepFailure(uint8_t result_code, const std::string & reason)
{
  const double now_sec = rosNowSeconds();
  const bool reason_repeated = have_last_failure_reason_ && reason == last_failure_reason_;

  // Priorities 1/3/4 forced structurally inert; priority 2's
  // authority_active_source/age are left REAL (only authority_seize_
  // elapsed_sec is pinned at 0.0) so Q-P9-1's still_holds_authority stays
  // truthful -- see header point 3.
  MissionWorldView view;
  view.mission_in_flight = true;
  view.operator_abort_requested = false;
  view.authority_active_source = authority_received_ ? last_authority_.active_source : kSourceNone;
  view.authority_age_sec = authority_received_ ?
    (now_sec - authority_arrival_.seconds()) : std::numeric_limits<double>::infinity();
  view.authority_seize_elapsed_sec = 0.0;
  view.battery_remaining = freshBatteryOrNan(now_sec);
  view.battery_warn_elapsed_sec = 0.0;
  view.battery_unknown_elapsed_sec = 0.0;
  view.operator_pause_requested = false;
  view.paused = false;
  view.paused_elapsed_sec = 0.0;
  view.localization_valid = localization_received_ && last_localization_.is_valid;
  view.localization_age_sec = localization_received_ ?
    (now_sec - localization_arrival_.seconds()) : std::numeric_limits<double>::infinity();
  view.last_nav_result_code = result_code;
  view.retry_count = step_retry_count_;
  view.last_failure_reason_repeated = reason_repeated;

  const GuardVerdict verdict = policy_.evaluate(view);

  last_failure_reason_ = reason;
  have_last_failure_reason_ = true;
  if (verdict.action == GuardAction::kContinue) {
    ++step_retry_count_;
  } else {
    pending_terminal_verdict_ = verdict;
  }
  return verdict;
}

void MissionExecutorNode::reportStepStarted(const std::string & step_name)
{
  current_step_name_ = step_name;
  publishEvent(
    uav_interfaces::msg::MissionEvent::EVENT_STEP_STARTED, step_name, "", kResultUnknown);
}

void MissionExecutorNode::reportStepCompleted(const std::string & step_name, bool succeeded)
{
  if (succeeded) {
    ++current_step_index_;
    publishEvent(
      uav_interfaces::msg::MissionEvent::EVENT_STEP_COMPLETED, step_name, "", kResultSucceeded);
  }
  // On failure the FAILED/ABORTED event, with a real description, is
  // published by whichever caller settles the terminal outcome
  // (finishAsAborted()/beginFinish()) once the retry budget is actually
  // exhausted -- a single retryable attempt failing is not yet a
  // mission-level event.
}

std::optional<SightedLandmark> MissionExecutorNode::latestLandmark(int32_t marker_id) const
{
  // B2 (2026-08-23, R32): the whole /world/semantic_landmarks MESSAGE must
  // itself be fresh (world_model still alive), on top of each landmark
  // entry's own time_since_seen_sec (world_model's own age-out of a stale
  // sighting) -- a dead world_model stream would otherwise keep replaying
  // its LAST message's time_since_seen_sec values forever without ever
  // aging them further, staying "fresh-looking" indefinitely.
  if (!landmarks_received_ ||
    (now().seconds() - landmarks_arrival_.seconds()) > 2.0 * world_model_publish_period_copy_sec_)
  {
    return std::nullopt;
  }
  // Green item (2026-08-23, review round 1): marker_id == -1 ("any") used
  // to return the FIRST array match, an unpublished/incidental ordering
  // dependency. Published criterion (contract §2.19): the MOST RECENTLY
  // seen match (smallest time_since_seen_sec) among all fresh matches.
  const uav_interfaces::msg::SemanticLandmark * best = nullptr;
  for (const auto & landmark : last_landmarks_.landmarks) {
    if (marker_id != -1 && landmark.marker_id != marker_id) {
      continue;
    }
    if (landmark.time_since_seen_sec > static_cast<float>(marker_seen_freshness_sec_)) {
      continue;
    }
    if (best == nullptr || landmark.time_since_seen_sec < best->time_since_seen_sec) {
      best = &landmark;
    }
  }
  if (best == nullptr) {
    return std::nullopt;
  }
  return SightedLandmark{best->pose.position.x, best->pose.position.y};
}

std::optional<SightedTarget> MissionExecutorNode::latestTarget(int32_t target_id) const
{
  // B2 (2026-08-23, R32): same "message itself must be fresh" gate as
  // latestLandmark() above, for /world/target_state.
  if (!target_received_ ||
    (now().seconds() - target_arrival_.seconds()) > 2.0 * world_model_publish_period_copy_sec_)
  {
    return std::nullopt;
  }
  if (target_id != -1 && last_target_.track_id != target_id) {
    return std::nullopt;
  }
  if (last_target_.time_since_seen_sec > static_cast<float>(target_seen_freshness_sec_)) {
    return std::nullopt;
  }
  return SightedTarget{
    last_target_.pose.position.x, last_target_.pose.position.y, last_target_.track_id};
}

bool MissionExecutorNode::odometryFresh() const
{
  return odometry_received_ &&
    (now().seconds() - odometry_arrival_.seconds()) <= 2.0 * odometry_publish_period_copy_sec_;
}

void MissionExecutorNode::noteTrackTargetProgress(double elapsed_this_attempt_sec)
{
  if (elapsed_this_attempt_sec >= track_progress_reset_sec_) {
    follow_search_attempts_ = 0;
  }
}

void MissionExecutorNode::noteSearchEpisodeBegins()
{
  // bug #12 (2026-08-23): moved here from NavAction's TrackTarget-terminal-
  // failure branch (G-M4.4b's original hook) -- see mission_context.hpp
  // doc comment for why. Fires EXACTLY once per fresh entry into
  // search_when_lost (SearchBudgetAvailable is its first child, and BT.CPP
  // Sequence only re-ticks an already-succeeded first child after the
  // WHOLE Sequence has returned a terminal status since).
  ++follow_search_attempts_;
}

bool MissionExecutorNode::searchBudgetAvailable() const
{
  return follow_search_attempts_ <= static_cast<uint32_t>(search_attempts_max_);
}

void MissionExecutorNode::noteSearchBudgetExhausted()
{
  pending_terminal_verdict_ = GuardVerdict{
    GuardAction::kAbortHold, kResultAbortedLostTarget,
    "search-attempts budget exhausted (recheck never confirmed reacquisition)"};
}

}  // namespace uav_mission
