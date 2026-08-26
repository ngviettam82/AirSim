#include "uav_mission/bt_nodes.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace uav_mission
{

namespace
{
constexpr const char * kNavTypeGotoPose = "GotoPose";
constexpr const char * kNavTypeTrackTarget = "TrackTarget";
}  // namespace

// ------------------------------------------------------------- NavAction

NavAction::NavAction(
  const std::string & name, const BT::NodeConfiguration & config, MissionContext * context)
: BT::StatefulActionNode(name, config), context_(context)
{
}

BT::PortsList NavAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("nav_type"),
    BT::InputPort<Waypoint>("target"),
    BT::InputPort<double>("x"),
    BT::InputPort<double>("y"),
    BT::InputPort<double>("z"),
    BT::InputPort<std::string>("frame_id"),
    BT::InputPort<double>("acceptance_radius"),
    BT::InputPort<double>("max_speed"),
    BT::InputPort<int>("target_id"),
    BT::InputPort<double>("standoff_distance"),
    BT::InputPort<double>("duration_seconds"),
    BT::InputPort<double>("target_lost_timeout"),
  };
}

bool NavAction::readPorts()
{
  if (!getInput("nav_type", nav_type_)) {
    return false;
  }

  if (nav_type_ == kNavTypeGotoPose) {
    Waypoint target;
    if (getInput("target", target)) {
      x_ = target.x;
      y_ = target.y;
      z_ = target.z;
    } else if (!(getInput("x", x_) && getInput("y", y_) && getInput("z", z_))) {
      return false;   // neither a {target} Waypoint nor all 3 of x/y/z given
    }
    getInput("frame_id", frame_id_);
    getInput("acceptance_radius", acceptance_radius_);
    getInput("max_speed", max_speed_);
    return true;
  }

  if (nav_type_ == kNavTypeTrackTarget) {
    int target_id_port = target_id_;
    getInput("target_id", target_id_port);
    target_id_ = target_id_port;
    getInput("standoff_distance", standoff_distance_);
    getInput("duration_seconds", duration_seconds_);
    getInput("target_lost_timeout", target_lost_timeout_);
    return true;
  }

  return false;   // unsupported nav_type -- P9.4 scope decision, see README
}

void NavAction::tryDispatch()
{
  const bool sent = (nav_type_ == kNavTypeGotoPose) ?
    context_->sendGotoPose(x_, y_, z_, frame_id_, acceptance_radius_, max_speed_) :
    context_->sendTrackTarget(target_id_, standoff_distance_, duration_seconds_, target_lost_timeout_);
  leg_state_ = sent ? LegState::kInFlight : LegState::kAwaitingDispatch;
  if (sent) {
    // G-M4.4b: ANY fresh send (first dispatch OR a backoff-retry re-send)
    // restarts the "sustained progress" clock -- see leg_start_sec_ doc
    // comment (bt_nodes.hpp). Y1: also the step-timeout clock, and its own
    // triggered-flag -- a FRESH dispatch (this attempt) starts a fresh
    // step_timeout_sec budget.
    leg_start_sec_ = context_->rosNowSeconds();
    step_timeout_triggered_ = false;
  }
}

BT::NodeStatus NavAction::onStart()
{
  context_->reportStepStarted(name());

  if (!readPorts()) {
    context_->resetStepRetryState();
    const GuardVerdict verdict = context_->evaluateStepFailure(
      kResultAbortedInvalidGoal, "NavAction: missing/unsupported ports for nav_type");
    // mission_policy.cpp forbids retrying ABORTED_INVALID_GOAL
    // unconditionally (retry_forbidden_by_code), so this is GUARANTEED
    // terminal regardless of retry_count. Green item (2026-08-23, review
    // round 1): a bare assert() here compiles to NOTHING in an
    // NDEBUG/Release build -- the ORIGINAL G-M2 bug (2026-08-22) was
    // exactly this: verdict came back kContinue, silently dropped via this
    // leaf's own hardcoded FAILURE return, pending_terminal_verdict_ stayed
    // empty. A real branch that survives Release: log loudly (the
    // guarantee is not trusted blindly at the one place a violation would
    // otherwise vanish silently) and STILL return FAILURE either way --
    // if pending_terminal_verdict_ genuinely was not recorded,
    // tickBody()'s catch-all still ends the mission safely instead of
    // hanging, just without the ERROR this branch now surfaces.
    if (verdict.action == GuardAction::kContinue) {
      RCLCPP_ERROR(
        rclcpp::get_logger("mission_bt_nodes"),
        "NavAction: evaluateStepFailure(ABORTED_INVALID_GOAL) returned kContinue -- "
        "mission_policy's retry_forbidden_by_code guarantee was violated (see mission_policy.cpp)");
    }
    context_->reportStepCompleted(name(), false);
    return BT::NodeStatus::FAILURE;
  }

  context_->resetStepRetryState();
  leg_state_ = LegState::kAwaitingDispatch;
  tryDispatch();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavAction::onRunning()
{
  const double now = context_->rosNowSeconds();

  if (leg_state_ == LegState::kBackoffWait) {
    if (now < backoff_until_sec_) {
      return BT::NodeStatus::RUNNING;
    }
    leg_state_ = LegState::kAwaitingDispatch;
  }

  if (leg_state_ == LegState::kAwaitingDispatch) {
    tryDispatch();
    return BT::NodeStatus::RUNNING;
  }

  // kInFlight.
  std::optional<GoalOutcome> outcome_opt = context_->pollNavActionOutcome();
  if (!outcome_opt.has_value()) {
    // G-M4.4b: still genuinely tracking, no outcome yet -- feed the
    // uninterrupted-progress clock so a real reacquisition can eventually
    // reset follow_target's search-attempts budget. GotoPose legs don't
    // participate in that budget; harmless to call unconditionally since
    // MissionContext only ever consults it for TrackTarget failures.
    if (nav_type_ == kNavTypeTrackTarget) {
      context_->noteTrackTargetProgress(now - leg_start_sec_);
    }
    // Y1 (2026-08-23, review round 1): step_timeout_sec was declared,
    // validated, and never enforced -- a goal stuck kInFlight forever
    // (navigator never replies) meant NavAction never left RUNNING, so
    // even the outer <Timeout>/battery/authority guards could only ever
    // cut it off from ABOVE, never this step's OWN budget. Cancel once
    // (step_timeout_triggered_ guards against re-issuing the cancel every
    // tick while waiting for it to land) and let the NEXT outcome --
    // whatever it turns out to be -- be treated as this step's own
    // ABORTED_TIMEOUT failure below, not the generic "canceled -> resend
    // forever" path.
    if (!step_timeout_triggered_ && (now - leg_start_sec_) >= context_->stepTimeoutSec()) {
      step_timeout_triggered_ = true;
      context_->cancelNavActionGoal();
    }
    return BT::NodeStatus::RUNNING;
  }
  const GoalOutcome & outcome = *outcome_opt;

  if (step_timeout_triggered_) {
    // Whatever this outcome turned out to be (canceled, or -- a refused-
    // cancel race -- succeeded/failed anyway) is superseded: the step
    // already blew its own step_timeout_sec budget, so route it through
    // the SAME evaluateStepFailure() path every other step failure uses
    // (retry/backoff, Q-P9-1, etc. all apply normally), with the mission-
    // policy code for THIS cause, not whatever the outcome carried.
    step_timeout_triggered_ = false;
    const GuardVerdict timeout_verdict = context_->evaluateStepFailure(
      kResultAbortedTimeout, "step exceeded step_timeout_sec (goal never resolved in time)");
    if (timeout_verdict.action == GuardAction::kContinue) {
      leg_state_ = LegState::kBackoffWait;
      backoff_until_sec_ = now + context_->retryBackoffSec();
      return BT::NodeStatus::RUNNING;
    }
    context_->reportStepCompleted(name(), false);
    return BT::NodeStatus::FAILURE;
  }

  if (outcome.canceled) {
    // A guard-initiated pause/abort ended this goal (S:2 "retry reset o
    // RANH GIOI BUOC" -- this is not this step's own fault, never counts
    // against retry_count_). The top-level tick loop stops ticking the
    // tree entirely while paused, so onRunning() only runs again once
    // ticking has resumed -- re-send the SAME goal, same target.
    leg_state_ = LegState::kAwaitingDispatch;
    return BT::NodeStatus::RUNNING;
  }

  if (outcome.succeeded) {
    context_->reportStepCompleted(name(), true);
    return BT::NodeStatus::SUCCESS;
  }

  const GuardVerdict verdict = context_->evaluateStepFailure(outcome.result_code, outcome.message);
  if (verdict.action == GuardAction::kContinue) {
    leg_state_ = LegState::kBackoffWait;
    backoff_until_sec_ = now + context_->retryBackoffSec();
    return BT::NodeStatus::RUNNING;
  }

  // kAbortHold/kAbortLand -- context_ already recorded the terminal verdict
  // for the tick loop; this leaf's own job is done. bug #12 (2026-08-23):
  // the search-attempts-budget increment used to live HERE (on a
  // TrackTarget leg's own LOST_TARGET retry-exhaustion) but moved to
  // SearchBudgetAvailable::tick() -- this branch can be reached WITHOUT
  // search_when_lost ever being entered at all (arena hoan toan sach,
  // TargetSeen never true even once, TrackTarget never dispatched), which
  // this hook could never observe.
  context_->reportStepCompleted(name(), false);
  return BT::NodeStatus::FAILURE;
}

void NavAction::onHalted()
{
  context_->cancelNavActionGoal();
  leg_state_ = LegState::kAwaitingDispatch;
  step_timeout_triggered_ = false;   // defensive -- onStart()'s tryDispatch() also clears it
}

// -------------------------------------------------------- WaypointCursor

WaypointCursor::WaypointCursor(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList WaypointCursor::providedPorts()
{
  return {
    BT::InputPort<std::vector<Waypoint>>("waypoints"),
    BT::OutputPort<Waypoint>("waypoint_out"),
  };
}

BT::NodeStatus WaypointCursor::tick()
{
  std::vector<Waypoint> waypoints;
  if (!getInput("waypoints", waypoints) || waypoints.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  const Waypoint & current = waypoints[index_ % waypoints.size()];
  setOutput("waypoint_out", current);
  ++index_;
  return BT::NodeStatus::SUCCESS;
}

// -------------------------------------------------------------- Dwell

Dwell::Dwell(const std::string & name, const BT::NodeConfiguration & config, MissionContext * context)
: BT::StatefulActionNode(name, config), context_(context)
{
}

BT::PortsList Dwell::providedPorts()
{
  return {BT::InputPort<double>("duration_sec")};
}

BT::NodeStatus Dwell::onStart()
{
  duration_sec_ = 0.0;
  getInput("duration_sec", duration_sec_);
  if (duration_sec_ < 0.0) {
    duration_sec_ = 0.0;
  }
  start_sec_ = context_->rosNowSeconds();
  return duration_sec_ <= 0.0 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

BT::NodeStatus Dwell::onRunning()
{
  return (context_->rosNowSeconds() - start_sec_ >= duration_sec_) ?
    BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

void Dwell::onHalted()
{
}

// ------------------------------------------------------------ MarkerSeen

MarkerSeen::MarkerSeen(
  const std::string & name, const BT::NodeConfiguration & config, MissionContext * context)
: BT::ConditionNode(name, config), context_(context)
{
}

BT::PortsList MarkerSeen::providedPorts()
{
  return {
    BT::InputPort<int>("marker_id"),
    BT::OutputPort<double>("marker_x"),
    BT::OutputPort<double>("marker_y"),
  };
}

BT::NodeStatus MarkerSeen::tick()
{
  int marker_id = -1;
  getInput("marker_id", marker_id);
  const std::optional<SightedLandmark> sighting = context_->latestLandmark(marker_id);
  if (!sighting.has_value()) {
    return BT::NodeStatus::FAILURE;
  }
  setOutput("marker_x", sighting->x);
  setOutput("marker_y", sighting->y);
  return BT::NodeStatus::SUCCESS;
}

// ------------------------------------------------------------ TargetSeen

TargetSeen::TargetSeen(
  const std::string & name, const BT::NodeConfiguration & config, MissionContext * context)
: BT::ConditionNode(name, config), context_(context)
{
}

BT::PortsList TargetSeen::providedPorts()
{
  return {
    BT::InputPort<int>("target_id"),
    BT::OutputPort<int>("track_id"),
  };
}

BT::NodeStatus TargetSeen::tick()
{
  int target_id = -1;
  getInput("target_id", target_id);
  const std::optional<SightedTarget> sighting = context_->latestTarget(target_id);
  if (!sighting.has_value()) {
    return BT::NodeStatus::FAILURE;
  }
  setOutput("track_id", sighting->track_id);
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------- SearchBudgetAvailable

SearchBudgetAvailable::SearchBudgetAvailable(
  const std::string & name, const BT::NodeConfiguration & config, MissionContext * context)
: BT::ConditionNode(name, config), context_(context)
{
}

BT::PortsList SearchBudgetAvailable::providedPorts()
{
  return {};
}

BT::NodeStatus SearchBudgetAvailable::tick()
{
  // bug #12 (2026-08-23): consumes one unit UNCONDITIONALLY on every fresh
  // entry (see MissionContext::noteSearchEpisodeBegins() doc comment for
  // why it moved here) -- checked AFTER incrementing, so search_attempts_
  // max=2 allows exactly 2 entries (1<=2, 2<=2) and blocks the 3rd (3<=2
  // false), matching plan P9 S:2.5 "2 luot".
  context_->noteSearchEpisodeBegins();
  if (context_->searchBudgetAvailable()) {
    return BT::NodeStatus::SUCCESS;
  }
  context_->noteSearchBudgetExhausted();
  return BT::NodeStatus::FAILURE;
}

// ------------------------------------------------------------ registration

void registerMissionBtNodes(BT::BehaviorTreeFactory & factory, MissionContext * context)
{
  factory.registerBuilder<NavAction>(
    "NavAction",
    [context](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<NavAction>(name, config, context);
    });
  factory.registerNodeType<WaypointCursor>("WaypointCursor");
  factory.registerBuilder<Dwell>(
    "Dwell",
    [context](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<Dwell>(name, config, context);
    });
  factory.registerBuilder<MarkerSeen>(
    "MarkerSeen",
    [context](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<MarkerSeen>(name, config, context);
    });
  factory.registerBuilder<TargetSeen>(
    "TargetSeen",
    [context](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<TargetSeen>(name, config, context);
    });
  factory.registerBuilder<SearchBudgetAvailable>(
    "SearchBudgetAvailable",
    [context](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<SearchBudgetAvailable>(name, config, context);
    });
}

}  // namespace uav_mission
