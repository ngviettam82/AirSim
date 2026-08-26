#include "uav_mission/mission_policy.hpp"

#include <cmath>
#include <stdexcept>

namespace uav_mission
{

namespace
{

void require(bool condition, const std::string & detail)
{
  if (!condition) {
    throw std::invalid_argument("mission policy parameters: " + detail);
  }
}

bool isTerminalFailureCode(uint8_t code)
{
  switch (code) {
    case kResultAbortedInvalidGoal:
    case kResultAbortedSafety:
    case kResultAbortedTimeout:
    case kResultAbortedPlannerFailed:
    case kResultAbortedLostTarget:
    case kResultAbortedInternalError:
    case kResultAbortedLowBattery:
      return true;
    default:
      return false;
  }
}

}  // namespace

const char * guardActionName(GuardAction action)
{
  switch (action) {
    case GuardAction::kContinue: return "CONTINUE";
    case GuardAction::kPause: return "PAUSE";
    case GuardAction::kEndEarly: return "END_EARLY";
    case GuardAction::kAbortHold: return "ABORT_HOLD";
    case GuardAction::kAbortLand: return "ABORT_LAND";
    default: return "UNKNOWN";
  }
}

MissionPolicy::MissionPolicy(const MissionPolicyParams & params)
: params_(params)
{
  validate();
}

void MissionPolicy::validate() const
{
  auto positive = [](double v) {return std::isfinite(v) && v > 0.0;};
  auto nonNegative = [](double v) {return std::isfinite(v) && v >= 0.0;};
  auto fraction = [](double v) {return std::isfinite(v) && v > 0.0 && v < 1.0;};

  require(
    positive(params_.authority_heartbeat_copy_sec),
    "authority_heartbeat_copy_sec must be finite and positive");
  require(
    positive(params_.authority_loss_grace_sec),
    "authority_loss_grace_sec must be finite and positive");
  require(
    params_.authority_loss_grace_sec >= 2.0 * params_.authority_heartbeat_copy_sec,
    "authority_loss_grace_sec must be >= 2 x authority_heartbeat_copy_sec (R29: a grace on a "
    "slow-sampled signal must outlive >= 2 sampling periods)");
  require(
    params_.authority_loss_grace_sec >= 1.5,
    "authority_loss_grace_sec must be >= 1.5 s (plan P9 S:2 contract floor, not a tunable)");

  require(fraction(params_.battery_warn_threshold), "battery_warn_threshold must be in (0, 1)");
  require(
    fraction(params_.battery_min_start_threshold), "battery_min_start_threshold must be in (0, 1)");
  require(
    params_.battery_min_start_threshold > params_.battery_warn_threshold,
    "battery_min_start_threshold must be > battery_warn_threshold (Q-P9-2: starting below the "
    "WARN margin defeats the margin's purpose)");
  require(
    positive(params_.battery_warn_dwell_sec), "battery_warn_dwell_sec must be finite and positive");
  require(
    positive(params_.battery_unknown_timeout_sec),
    "battery_unknown_timeout_sec must be finite and positive");
  require(
    positive(params_.battery_health_period_copy_sec),
    "battery_health_period_copy_sec must be finite and positive");
  require(
    params_.battery_warn_dwell_sec >= 2.0 * params_.battery_health_period_copy_sec,
    "battery_warn_dwell_sec must be >= 2 x battery_health_period_copy_sec (R29)");
  require(
    params_.battery_unknown_timeout_sec >= 2.0 * params_.battery_health_period_copy_sec,
    "battery_unknown_timeout_sec must be >= 2 x battery_health_period_copy_sec (R29)");

  require(positive(params_.paused_timeout_sec), "paused_timeout_sec must be finite and positive");

  require(
    positive(params_.localization_status_timeout_copy_sec),
    "localization_status_timeout_copy_sec must be finite and positive");
  require(
    params_.max_step_retries >= 1 && params_.max_step_retries <= 10,
    "max_step_retries must be in [1, 10] (plan P9 S:2 default is 2; a runaway cap is not a "
    "recovery policy)");
  require(positive(params_.retry_backoff_sec), "retry_backoff_sec must be finite and positive");

  require(
    positive(params_.navigator_goto_timeout_copy_sec),
    "navigator_goto_timeout_copy_sec must be finite and positive");
  require(positive(params_.step_timeout_sec), "step_timeout_sec must be finite and positive");
  require(
    params_.step_timeout_sec >= 1.2 * params_.navigator_goto_timeout_copy_sec,
    "step_timeout_sec must be >= 1.2 x navigator_goto_timeout_copy_sec (plan P9 S:3.4: a step "
    "must not time out before the single nav action inside it could legitimately still be running)");

  (void)nonNegative;   // reserved for future non-negative-only fields
}

GuardVerdict MissionPolicy::evaluate(const MissionWorldView & view) const
{
  GuardVerdict verdict;   // defaults: kContinue, kResultUnknown, ""

  if (!view.mission_in_flight) {
    return verdict;   // guard is inert when nothing is running
  }

  // Priority 1: operator abort. AbortMission.srv "does not land" -- HOLD,
  // never an automatic landing on an operator-initiated abort.
  if (view.operator_abort_requested) {
    verdict.action = GuardAction::kAbortHold;
    verdict.result_code = kResultCanceled;
    verdict.reason = "operator abort request -- holding position (AbortMission does not land)";
    return verdict;
  }

  // Priority 2: control authority seized away from MISSION, sustained.
  const bool authority_stale =
    view.authority_age_sec > 2.0 * params_.authority_heartbeat_copy_sec;
  const bool authority_not_mission = view.authority_active_source != kSourceMission;
  const bool authority_not_held = authority_not_mission || authority_stale;
  if (authority_not_held && view.authority_seize_elapsed_sec >= params_.authority_loss_grace_sec) {
    verdict.action = GuardAction::kPause;
    verdict.result_code = kResultAbortedNoAuthority;
    verdict.reason = (authority_stale && !authority_not_mission) ?
      "ControlAuthority message expired (R32) -- treated as not held, pausing" :
      "control authority held by a source other than MISSION -- pausing";
    return verdict;
  }

  // Priority: PAUSED past paused_timeout_sec with nobody calling
  // ResumeMission -- abort, still holding (not a fresh operator abort, but
  // the same "does not land" outcome).
  if (view.paused && view.paused_elapsed_sec >= params_.paused_timeout_sec) {
    verdict.action = GuardAction::kAbortHold;
    verdict.result_code = kResultAbortedTimeout;
    verdict.reason = "PAUSED past paused_timeout_sec with no ResumeMission -- aborting, holding";
    return verdict;
  }

  // Priority 3: battery. Checked AFTER authority on purpose (plan P9 S:2:
  // an end_early GotoPose(home)->Land sent while MISSION does not hold
  // authority would be a command into the void).
  if (!std::isfinite(view.battery_remaining)) {
    if (view.battery_unknown_elapsed_sec >= params_.battery_unknown_timeout_sec) {
      verdict.action = GuardAction::kEndEarly;
      verdict.result_code = kResultAbortedLowBattery;
      verdict.reason = "battery_remaining unmeasurable past unknown-timeout -- returning home to land";
      return verdict;
    }
  } else if (
    view.battery_remaining < params_.battery_warn_threshold &&
    view.battery_warn_elapsed_sec >= params_.battery_warn_dwell_sec)
  {
    verdict.action = GuardAction::kEndEarly;
    verdict.result_code = kResultAbortedLowBattery;
    verdict.reason = "battery below WARN threshold past dwell -- returning home to land";
    return verdict;
  }

  // Priority 4: explicit pause request, or already-paused steady state.
  if (view.operator_pause_requested) {
    verdict.action = GuardAction::kPause;
    verdict.reason = "operator pause request";
    return verdict;
  }
  if (view.paused) {
    verdict.action = GuardAction::kPause;
    verdict.reason = "awaiting ResumeMission";
    return verdict;
  }

  // Priority 5: step/body failure.
  if (view.last_nav_result_code == kResultAbortedLostLocalization) {
    // P8 policy: no retry, hold at once -- a frozen hold is the designed
    // recovery path when localization cannot be trusted, never a landing.
    verdict.action = GuardAction::kAbortHold;
    verdict.result_code = kResultAbortedLostLocalization;
    verdict.reason = "lost localization mid-step -- no retry, holding position";
    return verdict;
  }
  if (view.last_nav_result_code == kResultAbortedNoAuthority ||
    view.last_nav_result_code == kResultAbortedVehicleRejected)
  {
    // Navigator itself reported the loss directly (a backup path to
    // priority 2's own authority-message-driven detection) -- same
    // treatment: pause, do not abort.
    verdict.action = GuardAction::kPause;
    verdict.result_code = kResultAbortedNoAuthority;
    verdict.reason = "navigator reported NO_AUTHORITY/VEHICLE_REJECTED -- pausing, same as "
      "authority seize";
    return verdict;
  }

  if (isTerminalFailureCode(view.last_nav_result_code)) {
    // ABORTED_SAFETY is never retried blindly: the navigator itself judged
    // this a safety-relevant abort, retrying without understanding why is
    // not the conservative choice. ABORTED_INVALID_GOAL is likewise never
    // retried: it means THIS step's own ports/config are broken (a mission-
    // authoring error the tree cannot recover from by trying again) -- must
    // stay unconditional (not gated on retry_count) because
    // NavAction::onStart() (bt_nodes.cpp) calls this with retry_count freshly
    // reset to 0 for a step's first attempt, so "retries remain" would
    // otherwise always read true here (G-M2 bug, 2026-08-22).
    const bool retry_forbidden_by_code =
      view.last_nav_result_code == kResultAbortedSafety ||
      view.last_nav_result_code == kResultAbortedInvalidGoal;
    const bool reason_blocks_retry =
      view.last_failure_reason_repeated && view.retry_count >= 1;
    const bool retries_remaining =
      !retry_forbidden_by_code && view.retry_count < params_.max_step_retries;

    if (retries_remaining && !reason_blocks_retry) {
      verdict.action = GuardAction::kContinue;   // let the caller retry after its own backoff
      verdict.reason = "step failed, retry budget remains";
      return verdict;
    }

    // Q-P9-1: return-and-land only with ALL THREE conditions holding --
    // still MISSION authority, valid+fresh localization, and a failure
    // reason that is not itself localization-related (already excluded
    // above via the LOST_LOCALIZATION branch, so this is always true here,
    // kept explicit for readability/auditability).
    const bool still_holds_authority = !authority_not_held;
    const bool localization_ok = view.localization_valid &&
      view.localization_age_sec <= params_.localization_status_timeout_copy_sec;
    const bool reason_is_localization_related =
      view.last_nav_result_code == kResultAbortedLostLocalization;
    const bool land_allowed =
      still_holds_authority && localization_ok && !reason_is_localization_related;

    verdict.action = land_allowed ? GuardAction::kAbortLand : GuardAction::kAbortHold;
    verdict.result_code = view.last_nav_result_code;
    verdict.reason = land_allowed ?
      "step retries exhausted, still hold authority + valid localization -- returning home to land" :
      "step retries exhausted -- holding position (Q-P9-1 conditions not all met)";
    return verdict;
  }

  return verdict;   // healthy tick: CONTINUE
}

bool isBatteryOkToStart(double battery_remaining, const MissionPolicyParams & params)
{
  // R30: NaN (unmeasurable) never passes -- it is not "unknown but probably
  // fine", it is a reading this function must refuse to trust.
  if (!std::isfinite(battery_remaining)) {
    return false;
  }
  return battery_remaining >= params.battery_min_start_threshold;
}

}  // namespace uav_mission
