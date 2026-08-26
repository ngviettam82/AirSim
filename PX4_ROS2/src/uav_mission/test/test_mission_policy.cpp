// ROS-free (no domain, no rclcpp::init): the reactive priority table is
// arithmetic, so a test pins it exactly instead of trusting a flight --
// same rule uav_safety's failsafe_policy tests follow.
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "uav_mission/mission_policy.hpp"

namespace uav_mission
{
namespace
{

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

MissionPolicyParams defaultParams()
{
  return MissionPolicyParams{};
}

/// Nothing cutting: mission flying, MISSION holds fresh authority, full
/// battery, not paused, last step succeeded. Every test starts here and
/// perturbs one field (same convention as failsafe_policy's healthyMeasurements()).
MissionWorldView healthyView()
{
  MissionWorldView v;
  v.mission_in_flight = true;
  v.operator_abort_requested = false;

  v.authority_active_source = kSourceMission;
  v.authority_age_sec = 0.05;
  v.authority_seize_elapsed_sec = 0.0;

  v.battery_remaining = 0.80;
  v.battery_warn_elapsed_sec = 0.0;
  v.battery_unknown_elapsed_sec = 0.0;

  v.operator_pause_requested = false;
  v.paused = false;
  v.paused_elapsed_sec = 0.0;

  v.localization_valid = true;
  v.localization_age_sec = 0.05;
  v.last_nav_result_code = kResultSucceeded;
  v.retry_count = 0;
  v.last_failure_reason_repeated = false;
  return v;
}

}  // namespace

// ===========================================================================
// Parameter validation.
// ===========================================================================

TEST(ParamValidation, DefaultsAreAccepted)
{
  EXPECT_NO_THROW(MissionPolicy policy(defaultParams()));
}

TEST(ParamValidation, AuthorityGraceBelowTwoHeartbeatsIsRejected)
{
  MissionPolicyParams params = defaultParams();
  params.authority_heartbeat_copy_sec = 1.0;
  params.authority_loss_grace_sec = 1.5;   // < 2 x 1.0
  EXPECT_THROW(MissionPolicy policy(params), std::invalid_argument);
}

TEST(ParamValidation, AuthorityGraceBelowContractFloorIsRejected)
{
  MissionPolicyParams params = defaultParams();
  params.authority_loss_grace_sec = 1.49;   // < 1.5 s contract floor
  EXPECT_THROW(MissionPolicy policy(params), std::invalid_argument);
}

TEST(ParamValidation, AuthorityGraceAtContractFloorIsAccepted)
{
  MissionPolicyParams params = defaultParams();
  params.authority_loss_grace_sec = 1.5;
  EXPECT_NO_THROW(MissionPolicy policy(params));
}

TEST(ParamValidation, BatteryMinStartNotAboveWarnIsRejected)
{
  MissionPolicyParams params = defaultParams();
  params.battery_min_start_threshold = params.battery_warn_threshold;
  EXPECT_THROW(MissionPolicy policy(params), std::invalid_argument);
}

TEST(ParamValidation, BatteryWarnDwellBelowTwoHealthPeriodsIsRejected)
{
  MissionPolicyParams params = defaultParams();
  params.battery_warn_dwell_sec = 2.0 * params.battery_health_period_copy_sec - 0.001;
  EXPECT_THROW(MissionPolicy policy(params), std::invalid_argument);
}

TEST(ParamValidation, BatteryUnknownTimeoutBelowTwoHealthPeriodsIsRejected)
{
  MissionPolicyParams params = defaultParams();
  params.battery_unknown_timeout_sec = 2.0 * params.battery_health_period_copy_sec - 0.001;
  EXPECT_THROW(MissionPolicy policy(params), std::invalid_argument);
}

TEST(ParamValidation, StepTimeoutBelowOnePointTwoGotoTimeoutIsRejected)
{
  MissionPolicyParams params = defaultParams();
  params.step_timeout_sec = 1.2 * params.navigator_goto_timeout_copy_sec - 0.01;
  EXPECT_THROW(MissionPolicy policy(params), std::invalid_argument);
}

TEST(ParamValidation, StepTimeoutAtExactlyOnePointTwoGotoTimeoutIsAccepted)
{
  MissionPolicyParams params = defaultParams();
  params.step_timeout_sec = 1.2 * params.navigator_goto_timeout_copy_sec;
  EXPECT_NO_THROW(MissionPolicy policy(params));
}

TEST(ParamValidation, MaxStepRetriesZeroIsRejected)
{
  MissionPolicyParams params = defaultParams();
  params.max_step_retries = 0;
  EXPECT_THROW(MissionPolicy policy(params), std::invalid_argument);
}

TEST(ParamValidation, MaxStepRetriesAboveTenIsRejected)
{
  MissionPolicyParams params = defaultParams();
  params.max_step_retries = 11;
  EXPECT_THROW(MissionPolicy policy(params), std::invalid_argument);
}

TEST(ParamValidation, NonFiniteParamIsRejected)
{
  MissionPolicyParams params = defaultParams();
  params.paused_timeout_sec = kNan;
  EXPECT_THROW(MissionPolicy policy(params), std::invalid_argument);
}

TEST(ParamValidation, BatteryWarnThresholdOutOfRangeIsRejected)
{
  MissionPolicyParams params = defaultParams();
  params.battery_warn_threshold = 1.0;
  EXPECT_THROW(MissionPolicy policy(params), std::invalid_argument);
}

// ===========================================================================
// Baseline / idle no-op.
// ===========================================================================

TEST(Baseline, HealthyFlyingTickIsContinue)
{
  MissionPolicy policy(defaultParams());
  const GuardVerdict verdict = policy.evaluate(healthyView());
  EXPECT_EQ(verdict.action, GuardAction::kContinue);
}

TEST(Baseline, GuardIsNoOpWhenMissionNotInFlightEvenWithBadFields)
{
  // Every field below would cut on its own if mission_in_flight were true --
  // this pins that mission_in_flight is checked FIRST, unconditionally.
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.mission_in_flight = false;
  v.operator_abort_requested = true;
  v.authority_active_source = kSourceSafety;
  v.authority_seize_elapsed_sec = 999.0;
  v.battery_remaining = kNan;
  v.battery_unknown_elapsed_sec = 999.0;
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_EQ(verdict.action, GuardAction::kContinue);
}

// ===========================================================================
// Priority 1: operator abort.
// ===========================================================================

TEST(OperatorAbort, RequestedAbortsHoldNeverLands)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.operator_abort_requested = true;
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_EQ(verdict.action, GuardAction::kAbortHold);
  EXPECT_EQ(verdict.result_code, kResultCanceled);
}

// ===========================================================================
// Priority 2: control authority seized away from MISSION.
// ===========================================================================

TEST(AuthoritySeize, OtherSourceSustainedPastGracePauses)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.authority_active_source = kSourceSafety;
  v.authority_seize_elapsed_sec = defaultParams().authority_loss_grace_sec;
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_EQ(verdict.action, GuardAction::kPause);
  EXPECT_EQ(verdict.result_code, kResultAbortedNoAuthority);
}

TEST(AuthoritySeize, OtherSourceNotYetSustainedContinues)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.authority_active_source = kSourceSafety;
  v.authority_seize_elapsed_sec = defaultParams().authority_loss_grace_sec - 0.01;
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_EQ(verdict.action, GuardAction::kContinue);
}

TEST(AuthoritySeize, ExpiredAuthorityMessageR32SustainedPauses)
{
  // active_source still reads MISSION (stale cached value), but the message
  // itself is older than 2 x heartbeat -- R32: expired sample-and-hold is
  // treated as "not held", never as the last good value.
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.authority_active_source = kSourceMission;
  v.authority_age_sec = 2.0 * defaultParams().authority_heartbeat_copy_sec + 0.01;
  v.authority_seize_elapsed_sec = defaultParams().authority_loss_grace_sec;
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_EQ(verdict.action, GuardAction::kPause);
  EXPECT_EQ(verdict.result_code, kResultAbortedNoAuthority);
}

TEST(AuthoritySeize, FreshMissionAuthorityNeverPauses)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.authority_age_sec = 2.0 * defaultParams().authority_heartbeat_copy_sec;   // == boundary, not >
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_EQ(verdict.action, GuardAction::kContinue);
}

// ===========================================================================
// Priority: PAUSED timeout (no ResumeMission).
// ===========================================================================

TEST(PausedTimeout, PastTimeoutAbortsHold)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.paused = true;
  v.paused_elapsed_sec = defaultParams().paused_timeout_sec;
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_EQ(verdict.action, GuardAction::kAbortHold);
  EXPECT_EQ(verdict.result_code, kResultAbortedTimeout);
}

TEST(PausedTimeout, JustBeforeTimeoutStaysPaused)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.paused = true;
  v.paused_elapsed_sec = defaultParams().paused_timeout_sec - 0.01;
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_EQ(verdict.action, GuardAction::kPause);
}

// ===========================================================================
// Priority 3: battery.
// ===========================================================================

TEST(Battery, UnknownPastTimeoutEndsEarly)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.battery_remaining = kNan;
  v.battery_unknown_elapsed_sec = defaultParams().battery_unknown_timeout_sec;
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_EQ(verdict.action, GuardAction::kEndEarly);
  EXPECT_EQ(verdict.result_code, kResultAbortedLowBattery);
}

TEST(Battery, UnknownNotYetTimeoutContinues)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.battery_remaining = kNan;
  v.battery_unknown_elapsed_sec = defaultParams().battery_unknown_timeout_sec - 0.01;
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_EQ(verdict.action, GuardAction::kContinue);
}

TEST(Battery, WarnPastDwellEndsEarly)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.battery_remaining = defaultParams().battery_warn_threshold - 0.01;
  v.battery_warn_elapsed_sec = defaultParams().battery_warn_dwell_sec;
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_EQ(verdict.action, GuardAction::kEndEarly);
  EXPECT_EQ(verdict.result_code, kResultAbortedLowBattery);
}

TEST(Battery, WarnNotYetDwellContinues)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.battery_remaining = defaultParams().battery_warn_threshold - 0.01;
  v.battery_warn_elapsed_sec = defaultParams().battery_warn_dwell_sec - 0.01;
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_EQ(verdict.action, GuardAction::kContinue);
}

TEST(Battery, JustAboveWarnThresholdNeverEndsEarly)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.battery_remaining = defaultParams().battery_warn_threshold + 0.01;
  v.battery_warn_elapsed_sec = defaultParams().battery_warn_dwell_sec + 5.0;
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_EQ(verdict.action, GuardAction::kContinue);
}

// Priority ordering (R27 #3-class case): safety-seize must win over a
// simultaneous battery WARN -- the guard must stop at the FIRST cutting
// condition, never let a lower-priority row override a higher one.
TEST(PriorityOrder, AuthoritySeizeWinsOverSimultaneousBatteryWarn)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.authority_active_source = kSourceSafety;
  v.authority_seize_elapsed_sec = defaultParams().authority_loss_grace_sec;
  v.battery_remaining = defaultParams().battery_warn_threshold - 0.01;
  v.battery_warn_elapsed_sec = defaultParams().battery_warn_dwell_sec + 5.0;
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_EQ(verdict.action, GuardAction::kPause) << "authority seize (priority 2) must cut before "
    "battery (priority 3) is even consulted";
  EXPECT_EQ(verdict.result_code, kResultAbortedNoAuthority);
}

// ===========================================================================
// Priority 4: pause.
// ===========================================================================

TEST(Pause, OperatorPauseRequestPauses)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.operator_pause_requested = true;
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_EQ(verdict.action, GuardAction::kPause);
}

TEST(Pause, AlreadyPausedSteadyStateStaysPaused)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.paused = true;
  v.paused_elapsed_sec = 1.0;
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_EQ(verdict.action, GuardAction::kPause);
}

// ===========================================================================
// Priority 5: step/body failure.
// ===========================================================================

TEST(StepFailure, LostLocalizationNeverRetriesAlwaysHolds)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.last_nav_result_code = kResultAbortedLostLocalization;
  v.retry_count = 0;   // even on the very first occurrence
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_EQ(verdict.action, GuardAction::kAbortHold);
  EXPECT_EQ(verdict.result_code, kResultAbortedLostLocalization);
}

TEST(StepFailure, NavigatorReportedNoAuthorityPauses)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.last_nav_result_code = kResultAbortedNoAuthority;
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_EQ(verdict.action, GuardAction::kPause);
  EXPECT_EQ(verdict.result_code, kResultAbortedNoAuthority);
}

TEST(StepFailure, NavigatorReportedVehicleRejectedPauses)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.last_nav_result_code = kResultAbortedVehicleRejected;
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_EQ(verdict.action, GuardAction::kPause);
  EXPECT_EQ(verdict.result_code, kResultAbortedNoAuthority);
}

TEST(StepFailure, TerminalFailureWithRetryBudgetContinues)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.last_nav_result_code = kResultAbortedPlannerFailed;
  v.retry_count = 0;
  v.last_failure_reason_repeated = false;
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_EQ(verdict.action, GuardAction::kContinue);
}

TEST(StepFailure, TerminalFailureReasonRepeatedBlocksRetryEvenWithBudget)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.last_nav_result_code = kResultAbortedPlannerFailed;
  v.retry_count = 1;   // < max_step_retries (2)
  v.last_failure_reason_repeated = true;
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_NE(verdict.action, GuardAction::kContinue);
}

TEST(StepFailure, TerminalFailureRetriesExhaustedAborts)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.last_nav_result_code = kResultAbortedTimeout;
  v.retry_count = defaultParams().max_step_retries;
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_NE(verdict.action, GuardAction::kContinue);
  EXPECT_EQ(verdict.result_code, kResultAbortedTimeout);
}

TEST(StepFailure, AbortedSafetyNeverRetriesEvenOnFirstOccurrence)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.last_nav_result_code = kResultAbortedSafety;
  v.retry_count = 0;
  v.last_failure_reason_repeated = false;
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_NE(verdict.action, GuardAction::kContinue);
}

// G-M2 gate bug (2026-08-22): NavAction::onStart()'s readPorts()-fail branch
// (bt_nodes.cpp) always calls evaluateStepFailure() with retry_count freshly
// reset to 0 for THIS step's first (and only ever) attempt -- a step whose
// own ports/config are broken can never retry its way to success, so this
// must read terminal on retry_count==0, exactly like AbortedSafety above.
// Before the fix this returned kContinue, which NavAction discards (always
// returns FAILURE regardless) while pending_terminal_verdict_ stayed EMPTY --
// tickBody()'s catch-all then fired instead, landing on a misleading
// ABORTED_TIMEOUT/"mission body ended without succeeding" after an
// unnecessary full Finish (fly home + land) detour.
TEST(StepFailure, AbortedInvalidGoalNeverRetriesEvenOnFirstOccurrence)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.last_nav_result_code = kResultAbortedInvalidGoal;
  v.retry_count = 0;
  v.last_failure_reason_repeated = false;
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_NE(verdict.action, GuardAction::kContinue);
  EXPECT_EQ(verdict.result_code, kResultAbortedInvalidGoal);
}

// G-M4.4 (2026-08-23): ABORTED_LOST_TARGET goes through the GENERIC
// terminal-failure retry-then-abort path (no special-casing, unlike
// ABORTED_LOST_LOCALIZATION/ABORTED_SAFETY above) -- pinned explicitly here
// because test_mission_executor_node.cpp's identity-anchor integration test
// depends on this EXACT convergence: an anchored TrackTarget goal that
// keeps missing its own id retries the SAME anchored id up to
// max_step_retries times (NavAction's retry path re-dispatches from its
// own member state, never re-reading the blackboard) before the leaf
// itself reports terminal FAILURE and the tree falls through to search.
TEST(StepFailure, LostTargetRetriesThenAbortsAtBudget)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView below_budget = healthyView();
  below_budget.last_nav_result_code = kResultAbortedLostTarget;
  below_budget.retry_count = defaultParams().max_step_retries - 1;
  EXPECT_EQ(policy.evaluate(below_budget).action, GuardAction::kContinue)
    << "below the retry budget, a lost target must still retry the SAME anchored id, not abort";

  MissionWorldView at_budget = healthyView();
  at_budget.last_nav_result_code = kResultAbortedLostTarget;
  at_budget.retry_count = defaultParams().max_step_retries;
  const GuardVerdict verdict = policy.evaluate(at_budget);
  EXPECT_NE(verdict.action, GuardAction::kContinue);
  EXPECT_EQ(verdict.result_code, kResultAbortedLostTarget);
}

// --- Q-P9-1: land-vs-hold on retries-exhausted, 3-condition gate. ---

TEST(QP91, AllThreeConditionsMetAllowsLand)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.last_nav_result_code = kResultAbortedPlannerFailed;
  v.retry_count = defaultParams().max_step_retries;
  // still holds authority, localization valid + fresh (healthyView() defaults).
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_EQ(verdict.action, GuardAction::kAbortLand);
}

TEST(QP91, NotHoldingAuthorityForcesHoldNotLand)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.last_nav_result_code = kResultAbortedPlannerFailed;
  v.retry_count = defaultParams().max_step_retries;
  // Authority currently not with MISSION, but the seize dwell has not yet
  // reached grace -- priority 2 never fired, but Q-P9-1 must still refuse land.
  v.authority_active_source = kSourceSafety;
  v.authority_seize_elapsed_sec = 0.0;
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_EQ(verdict.action, GuardAction::kAbortHold);
}

TEST(QP91, StaleLocalizationForcesHoldNotLand)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.last_nav_result_code = kResultAbortedPlannerFailed;
  v.retry_count = defaultParams().max_step_retries;
  v.localization_age_sec = defaultParams().localization_status_timeout_copy_sec + 0.01;
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_EQ(verdict.action, GuardAction::kAbortHold);
}

TEST(QP91, InvalidLocalizationForcesHoldNotLand)
{
  MissionPolicy policy(defaultParams());
  MissionWorldView v = healthyView();
  v.last_nav_result_code = kResultAbortedPlannerFailed;
  v.retry_count = defaultParams().max_step_retries;
  v.localization_valid = false;
  const GuardVerdict verdict = policy.evaluate(v);
  EXPECT_EQ(verdict.action, GuardAction::kAbortHold);
}

// ===========================================================================
// isBatteryOkToStart (Q-P9-2 min-start gate).
// ===========================================================================

TEST(BatteryOkToStart, AboveMinStartPasses)
{
  const MissionPolicyParams params = defaultParams();
  EXPECT_TRUE(isBatteryOkToStart(params.battery_min_start_threshold + 0.01, params));
}

TEST(BatteryOkToStart, BelowMinStartFails)
{
  const MissionPolicyParams params = defaultParams();
  EXPECT_FALSE(isBatteryOkToStart(params.battery_min_start_threshold - 0.01, params));
}

TEST(BatteryOkToStart, NanNeverPasses)
{
  const MissionPolicyParams params = defaultParams();
  EXPECT_FALSE(isBatteryOkToStart(kNan, params));
}

}  // namespace uav_mission
