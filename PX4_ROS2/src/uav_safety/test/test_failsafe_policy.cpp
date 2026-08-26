// ROS-free (no domain, no rclcpp::init): the 12-code detection table, the
// FSM, and ClearFault are arithmetic, so a test pins them exactly instead of
// trusting a flight -- same rule uav_control_authority's authority_arbiter
// tests follow.
#include <cmath>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "uav_safety/failsafe_policy.hpp"

namespace uav_safety
{
namespace
{

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

FailsafePolicyParams defaultParams()
{
  return FailsafePolicyParams{};
}

/// Nothing violating: fresh localization, full battery, distant obstacle,
/// healthy offboard link, all estimator inputs ok, camera ok, frame ok, no
/// command in flight. Every test starts here and perturbs one field.
Measurements healthyMeasurements()
{
  Measurements m;
  m.localization_status_received = true;
  m.localization_status_age_sec = 0.05;
  m.localization_is_valid = true;
  m.fused_odometry_age_sec = 0.02;
  m.localization_jump_count = 0.0;

  m.command_setpoint_displacement_m = 0.0;
  m.command_setpoint_window_full = true;
  m.command_selected_source = kSourceMission;

  m.battery_remaining = 0.80;
  m.vehicle_state_received = true;
  m.failsafe_active = false;
  m.flight_mode = kFlightModeOffboard;
  // B5: "healthy" means actively flying under this system's control --
  // armed + OFFBOARD + a fresh command_selected is exactly the condition
  // B5's action gate requires for the 4 LATCHING codes to actually latch.
  m.armed = true;
  m.command_fresh = true;

  m.obstacle_min_distance_m = 5.0;
  m.obstacle_map_age_sec = 0.05;
  m.localization_good_for_obstacles = true;

  m.offboard_status_received = true;
  m.offboard_state = kOffboardStateActive;
  m.offboard_setpoint_rate_hz = 20.0;

  m.vehicle_health_received = true;
  m.estimator_flags_trustworthy = true;
  m.imu_ok = true;
  m.gps_ok = true;
  m.barometer_ok = true;
  m.magnetometer_ok = true;

  m.camera_health_received = true;
  m.camera_stream_ok = true;

  m.active_channel_dropping_wrong_frame = false;
  m.fused_frame_mismatch = false;

  m.cmd_mission_live = false;
  m.cmd_mission_setpoint_distance_from_fused_m = 0.0;
  return m;
}

ViolationEntry entryFor(const EvaluationResult & result, ViolationCode code)
{
  return result.violations[static_cast<size_t>(code)];
}

}  // namespace

// ===========================================================================
// Parameter validation (S:9d pairing rules + general sanity). Refuse to
// start on an unusable config, same discipline as AuthorityArbiter.
// ===========================================================================

TEST(ParamValidation, DefaultsAreAccepted)
{
  EXPECT_NO_THROW(FailsafePolicy policy(defaultParams()));
}

TEST(ParamValidation, V1_BlindGraceNotExceedingOdometryTimeoutPlusTwoTicksIsRejected)
{
  FailsafePolicyParams params = defaultParams();
  params.blind_grace_sec = 1.2;   // == 1.0 + 2/10 exactly -- must be strictly greater
  EXPECT_THROW(FailsafePolicy policy(params), std::invalid_argument);
}

TEST(ParamValidation, V2_FusedTimeoutNotBelowOdometryTimeoutCopyIsRejected)
{
  FailsafePolicyParams params = defaultParams();
  params.fused_timeout_sec = 1.0;   // == odometry_timeout_copy_sec, must be strictly below
  EXPECT_THROW(FailsafePolicy policy(params), std::invalid_argument);
}

TEST(ParamValidation, V3_HoldStreamHzBelowThreeOverArbiterTimeoutIsRejected)
{
  FailsafePolicyParams params = defaultParams();
  params.hold_stream_hz = 10.0;   // < 3/0.20 = 15
  EXPECT_THROW(FailsafePolicy policy(params), std::invalid_argument);
}

TEST(ParamValidation, V4_PoseMaxAgeAboveContractCeilingIsRejected)
{
  FailsafePolicyParams params = defaultParams();
  params.pose_max_age_sec = 0.26;
  EXPECT_THROW(FailsafePolicy policy(params), std::invalid_argument);
}

TEST(ParamValidation, V5_HandoverJumpLimitAboveMaxLeadCopyIsRejected)
{
  FailsafePolicyParams params = defaultParams();
  params.handover_jump_limit_m = 0.81;   // > max_lead_copy_m (0.80)
  EXPECT_THROW(FailsafePolicy policy(params), std::invalid_argument);
}

TEST(ParamValidation, V6_EmptyOdomFrameIsRejected)
{
  FailsafePolicyParams params = defaultParams();
  params.odom_frame.clear();
  EXPECT_THROW(FailsafePolicy policy(params), std::invalid_argument);
}

TEST(ParamValidation, V7_SelectedStaleBelowDownstreamTimeoutCopyIsRejected)
{
  FailsafePolicyParams params = defaultParams();
  params.selected_stale_sec = params.downstream_command_timeout_copy_sec - 0.01;
  EXPECT_THROW(FailsafePolicy policy(params), std::invalid_argument);
}

TEST(ParamValidation, V7_SelectedStaleEqualToDownstreamTimeoutCopyIsAccepted)
{
  FailsafePolicyParams params = defaultParams();
  params.selected_stale_sec = params.downstream_command_timeout_copy_sec;   // exactly the boundary
  EXPECT_NO_THROW(FailsafePolicy policy(params));
}

TEST(ParamValidation, BatteryCriticalThresholdMustBeBelowWarnThreshold)
{
  FailsafePolicyParams params = defaultParams();
  params.battery_critical_threshold = params.battery_warn_threshold;
  EXPECT_THROW(FailsafePolicy policy(params), std::invalid_argument);
}

// B3: real PX4 battery threshold copies must be internally ordered.
TEST(ParamValidation, B3_BatteryThresholdCopyOrderingIsEnforced)
{
  FailsafePolicyParams params = defaultParams();
  params.bat_crit_thr_copy = params.bat_low_thr_copy;   // violates emergen < crit < low
  EXPECT_THROW(FailsafePolicy policy(params), std::invalid_argument);
}

TEST(ParamValidation, B1_FrameMismatchGraceBelowTwoDiagnosticsPeriodsIsRejected)
{
  FailsafePolicyParams params = defaultParams();
  params.frame_mismatch_grace_sec = 2.0 * params.arbiter_diagnostics_period_copy_sec - 0.01;
  EXPECT_THROW(FailsafePolicy policy(params), std::invalid_argument);
}

TEST(ParamValidation, Y2_ObstacleMapTimeoutBelowTwoWorldModelPeriodsIsRejected)
{
  FailsafePolicyParams params = defaultParams();
  params.obstacle_map_timeout_copy_sec = 2.0 / params.world_model_publish_hz_copy - 0.001;
  EXPECT_THROW(FailsafePolicy policy(params), std::invalid_argument);
}

TEST(ParamValidation, Y8_PoseMaxAgeBelowTwoMuxPeriodsIsRejected)
{
  FailsafePolicyParams params = defaultParams();
  params.pose_max_age_sec = 2.0 * params.mux_publish_period_copy_sec - 0.001;
  EXPECT_THROW(FailsafePolicy policy(params), std::invalid_argument);
}

TEST(ParamValidation, NegativeGraceIsRejected)
{
  FailsafePolicyParams params = defaultParams();
  params.localization_invalid_grace_sec = -0.01;
  EXPECT_THROW(FailsafePolicy policy(params), std::invalid_argument);
}

TEST(ParamValidation, NonFiniteParamIsRejected)
{
  FailsafePolicyParams params = defaultParams();
  params.min_obstacle_distance_copy_m = kNan;
  EXPECT_THROW(FailsafePolicy policy(params), std::invalid_argument);
}

TEST(ParamValidation, ZeroBlindGraceIsRejected)
{
  // Also exercises V1 (0 fails the pairing rule long before reaching a bare
  // positivity check) -- kept as its own test because it is the literal
  // "grace set to 0" mutation the P8.1 handoff checks for.
  FailsafePolicyParams params = defaultParams();
  params.blind_grace_sec = 0.0;
  EXPECT_THROW(FailsafePolicy policy(params), std::invalid_argument);
}

TEST(ParamValidation, ZeroFrozenSetpointEpsilonIsRejected)
{
  FailsafePolicyParams params = defaultParams();
  params.frozen_setpoint_epsilon_copy_m = 0.0;
  EXPECT_THROW(FailsafePolicy policy(params), std::invalid_argument);
}

// ===========================================================================
// Baseline: a fully healthy tick reports OK on all 12 codes, target NONE,
// state stays NORMAL.
// ===========================================================================

TEST(Baseline, HealthyMeasurementsProduceNoViolationsAndStayNormal)
{
  FailsafePolicy policy(defaultParams());
  const EvaluationResult result = policy.evaluate(healthyMeasurements(), 0.0);
  for (size_t i = 0; i < kViolationCodeCount; ++i) {
    EXPECT_EQ(result.violations[i].status, ViolationStatus::kOk)
      << violationCodeName(static_cast<ViolationCode>(i));
  }
  EXPECT_EQ(result.overall_level, kLevelOk);
  EXPECT_EQ(result.target_action, FailsafeAction::kNone);
  EXPECT_EQ(result.state, SupervisorState::kNormal);
}

// ===========================================================================
// LOCALIZATION_INVALID (S:3 row 1): is_valid=false AND status message fresh.
// ===========================================================================

TEST(LocalizationInvalid, InvalidWithFreshStatusViolates)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.localization_is_valid = false;
  const EvaluationResult result = policy.evaluate(m, 0.0);
  EXPECT_EQ(entryFor(result, ViolationCode::kLocalizationInvalid).status, ViolationStatus::kViolating);
}

TEST(LocalizationInvalid, InvalidButStatusStaleDoesNotViolateThisCode)
{
  // The message itself is not fresh -- LOCALIZATION_STALE is the code that
  // owns this, not INVALID (S:3 row 1's own freshness gate).
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.localization_is_valid = false;
  m.localization_status_age_sec = defaultParams().status_timeout_sec + 0.01;
  const EvaluationResult result = policy.evaluate(m, 0.0);
  EXPECT_EQ(entryFor(result, ViolationCode::kLocalizationInvalid).status, ViolationStatus::kOk);
}

TEST(LocalizationInvalid, NeverReceivedDoesNotViolateThisCode)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.localization_status_received = false;
  const EvaluationResult result = policy.evaluate(m, 0.0);
  EXPECT_EQ(entryFor(result, ViolationCode::kLocalizationInvalid).status, ViolationStatus::kOk);
}

// ===========================================================================
// LOCALIZATION_STALE (S:3 row 2): odometry_fused age boundary.
// ===========================================================================

TEST(LocalizationStale, ExactlyAtTimeoutIsStillOk)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.fused_odometry_age_sec = defaultParams().fused_timeout_sec;
  const EvaluationResult result = policy.evaluate(m, 0.0);
  EXPECT_EQ(entryFor(result, ViolationCode::kLocalizationStale).status, ViolationStatus::kOk);
}

TEST(LocalizationStale, JustPastTimeoutViolates)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.fused_odometry_age_sec = defaultParams().fused_timeout_sec + 0.001;
  const EvaluationResult result = policy.evaluate(m, 0.0);
  EXPECT_EQ(entryFor(result, ViolationCode::kLocalizationStale).status, ViolationStatus::kViolating);
}

TEST(LocalizationStale, NeverReceivedIsInfiniteAgeAndViolates)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.fused_odometry_age_sec = kInf;
  const EvaluationResult result = policy.evaluate(m, 0.0);
  EXPECT_EQ(entryFor(result, ViolationCode::kLocalizationStale).status, ViolationStatus::kViolating);
}

// ===========================================================================
// LOCALIZATION_JUMP (S:3 row 3): NaN cannot-measure, otherwise jumps > 0.
// ===========================================================================

TEST(LocalizationJump, ZeroJumpsIsOk)
{
  FailsafePolicy policy(defaultParams());
  const EvaluationResult result = policy.evaluate(healthyMeasurements(), 0.0);
  EXPECT_EQ(entryFor(result, ViolationCode::kLocalizationJump).status, ViolationStatus::kOk);
}

TEST(LocalizationJump, OneJumpViolates)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.localization_jump_count = 1.0;
  const EvaluationResult result = policy.evaluate(m, 0.0);
  EXPECT_EQ(entryFor(result, ViolationCode::kLocalizationJump).status, ViolationStatus::kViolating);
}

TEST(LocalizationJump, NanJumpCountCannotMeasureAndTakesNoAction)
{
  // R27-2: NaN must never silently compare as "0 jumps" (false) or "always
  // jumping" (true) -- it must read as its own third state.
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.localization_jump_count = kNan;
  const EvaluationResult result = policy.evaluate(m, 0.0);
  EXPECT_EQ(entryFor(result, ViolationCode::kLocalizationJump).status, ViolationStatus::kCannotMeasure);
  EXPECT_EQ(result.target_action, FailsafeAction::kNone);
}

// ===========================================================================
// BLIND_COMMAND (S:0b decision R1, S:3 row 4): the central R1 pin --
// stationary hold on bad localization must NOT cut, moving must, and only
// MISSION/TEST count.
// ===========================================================================

TEST(BlindCommand, StationarySetpointOnBadLocalizationDoesNotTrigger)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.localization_is_valid = false;
  m.command_setpoint_displacement_m = 0.0;   // frozen hold, exactly the designed recovery path
  double now = 0.0;
  EvaluationResult result;
  for (; now <= defaultParams().blind_grace_sec + 1.0; now += 0.05) {
    result = policy.evaluate(m, now);
  }
  EXPECT_EQ(entryFor(result, ViolationCode::kBlindCommand).status, ViolationStatus::kOk);
  EXPECT_NE(result.state, SupervisorState::kEngagingInhibit);
  EXPECT_NE(result.state, SupervisorState::kInhibited);
}

TEST(BlindCommand, MovingSetpointPastGraceOnMissionTriggers)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.localization_is_valid = false;
  m.command_setpoint_displacement_m = defaultParams().frozen_setpoint_epsilon_copy_m + 0.01;
  m.command_selected_source = kSourceMission;
  double now = 0.0;
  EvaluationResult result;
  for (; now <= defaultParams().blind_grace_sec + 0.5; now += 0.05) {
    result = policy.evaluate(m, now);
  }
  EXPECT_EQ(entryFor(result, ViolationCode::kBlindCommand).status, ViolationStatus::kViolating);
  EXPECT_EQ(result.target_action, FailsafeAction::kInhibit);
}

TEST(BlindCommand, MovingSetpointOnOperatorSourceDoesNotTrigger)
{
  // S:3 row 4: only MISSION/TEST -- OPERATOR/SAFETY/NONE are exempt.
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.localization_is_valid = false;
  m.command_setpoint_displacement_m = defaultParams().frozen_setpoint_epsilon_copy_m + 0.01;
  m.command_selected_source = kSourceOperator;
  double now = 0.0;
  EvaluationResult result;
  for (; now <= defaultParams().blind_grace_sec + 0.5; now += 0.05) {
    result = policy.evaluate(m, now);
  }
  EXPECT_EQ(entryFor(result, ViolationCode::kBlindCommand).status, ViolationStatus::kOk);
}

TEST(BlindCommand, JustBeforeGraceDoesNotYetTrigger)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.localization_is_valid = false;
  m.command_setpoint_displacement_m = defaultParams().frozen_setpoint_epsilon_copy_m + 0.01;
  m.command_selected_source = kSourceMission;
  // Localization goes bad at t=0; check just under the grace boundary.
  policy.evaluate(m, 0.0);
  const EvaluationResult result = policy.evaluate(m, defaultParams().blind_grace_sec - 0.01);
  EXPECT_EQ(entryFor(result, ViolationCode::kBlindCommand).status, ViolationStatus::kOk);
}

TEST(BlindCommand, ExactlyAtGraceTriggers)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.localization_is_valid = false;
  m.command_setpoint_displacement_m = defaultParams().frozen_setpoint_epsilon_copy_m + 0.01;
  m.command_selected_source = kSourceMission;
  policy.evaluate(m, 0.0);
  const EvaluationResult result = policy.evaluate(m, defaultParams().blind_grace_sec);
  EXPECT_EQ(entryFor(result, ViolationCode::kBlindCommand).status, ViolationStatus::kViolating);
}

TEST(BlindCommand, DisplacementNotYetMeasurableCannotMeasureAndTakesNoAction)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.localization_is_valid = false;
  m.command_setpoint_window_full = false;   // not enough samples yet
  double now = 0.0;
  EvaluationResult result;
  for (; now <= defaultParams().blind_grace_sec + 0.5; now += 0.05) {
    result = policy.evaluate(m, now);
  }
  EXPECT_EQ(entryFor(result, ViolationCode::kBlindCommand).status, ViolationStatus::kCannotMeasure);
  EXPECT_EQ(result.target_action, FailsafeAction::kNone);
}

TEST(BlindCommand, ExactlyAtEpsilonDoesNotCountAsMoving)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.localization_is_valid = false;
  m.command_setpoint_displacement_m = defaultParams().frozen_setpoint_epsilon_copy_m;   // == epsilon, not >
  m.command_selected_source = kSourceMission;
  double now = 0.0;
  EvaluationResult result;
  for (; now <= defaultParams().blind_grace_sec + 0.5; now += 0.05) {
    result = policy.evaluate(m, now);
  }
  EXPECT_EQ(entryFor(result, ViolationCode::kBlindCommand).status, ViolationStatus::kOk);
}

// ===========================================================================
// BATTERY_WARN / BATTERY_CRITICAL (S:3 rows 5-6).
// ===========================================================================

TEST(Battery, JustAboveWarnThresholdIsOk)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.battery_remaining = defaultParams().battery_warn_threshold + 0.001;
  const EvaluationResult result = policy.evaluate(m, 0.0);
  EXPECT_EQ(entryFor(result, ViolationCode::kBatteryWarn).status, ViolationStatus::kOk);
}

TEST(Battery, JustBelowWarnThresholdViolatesWarnOnly)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.battery_remaining = defaultParams().battery_warn_threshold - 0.001;
  const EvaluationResult result = policy.evaluate(m, 0.0);
  EXPECT_EQ(entryFor(result, ViolationCode::kBatteryWarn).status, ViolationStatus::kViolating);
  EXPECT_EQ(entryFor(result, ViolationCode::kBatteryCritical).status, ViolationStatus::kOk);
}

TEST(Battery, JustBelowCriticalThresholdViolatesBoth)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.battery_remaining = defaultParams().battery_critical_threshold - 0.001;
  const EvaluationResult result = policy.evaluate(m, 0.0);
  EXPECT_EQ(entryFor(result, ViolationCode::kBatteryWarn).status, ViolationStatus::kViolating);
  EXPECT_EQ(entryFor(result, ViolationCode::kBatteryCritical).status, ViolationStatus::kViolating);
}

TEST(Battery, NanBatteryCannotMeasureAndTakesNoAction)
{
  // G-S1 #12: NaN battery must never be read as healthy OR as critical.
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.battery_remaining = kNan;
  const EvaluationResult result = policy.evaluate(m, 0.0);
  EXPECT_EQ(entryFor(result, ViolationCode::kBatteryWarn).status, ViolationStatus::kCannotMeasure);
  EXPECT_EQ(entryFor(result, ViolationCode::kBatteryCritical).status, ViolationStatus::kCannotMeasure);
  EXPECT_EQ(entryFor(result, ViolationCode::kBatteryPx4NoAction).status, ViolationStatus::kCannotMeasure);
  EXPECT_EQ(result.target_action, FailsafeAction::kNone);
  EXPECT_EQ(result.state, SupervisorState::kNormal);
}

// Golden-value pin (P8 checkpoint T3, R1: cannot call px4::toBatteryHealth()
// here): mirrors uav_px4_backend's BatteryHealth.
// GoldenSentinelChainedToSafetysNanContract (voltage=0, remaining=-1 -> NaN).
TEST(Battery, SentinelNanChainedFromBackendNeverReadsAsCritical)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.battery_remaining = kNan;   // == px4::toBatteryHealth({voltage=0, remaining=-1}).remaining
  const EvaluationResult result = policy.evaluate(m, 0.0);
  EXPECT_EQ(entryFor(result, ViolationCode::kBatteryWarn).status, ViolationStatus::kCannotMeasure);
  EXPECT_EQ(entryFor(result, ViolationCode::kBatteryCritical).status, ViolationStatus::kCannotMeasure);
  EXPECT_EQ(entryFor(result, ViolationCode::kBatteryPx4NoAction).status, ViolationStatus::kCannotMeasure);
  EXPECT_EQ(result.target_action, FailsafeAction::kNone);
}

// ===========================================================================
// BATTERY_PX4_NO_ACTION (S:3 row 7): critical past grace AND PX4 not acting.
// B3 (review 2026-08-21): gates on bat_crit_thr_copy (PX4's REAL threshold,
// 0.07, read via MAVLink), not battery_critical_threshold (this package's
// own, more conservative 0.10) -- these tests updated accordingly.
// ===========================================================================

TEST(BatteryPx4NoAction, CriticalPastGraceWithNoPx4ActionTriggers)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.battery_remaining = defaultParams().bat_crit_thr_copy - 0.01;
  double now = 0.0;
  EvaluationResult result;
  for (; now <= defaultParams().px4_action_grace_sec + 0.5; now += 0.1) {
    result = policy.evaluate(m, now);
  }
  EXPECT_EQ(entryFor(result, ViolationCode::kBatteryPx4NoAction).status, ViolationStatus::kViolating);
  EXPECT_EQ(result.target_action, FailsafeAction::kInhibit);
}

TEST(BatteryPx4NoAction, JustBeforeGraceDoesNotYetTrigger)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.battery_remaining = defaultParams().bat_crit_thr_copy - 0.01;
  policy.evaluate(m, 0.0);
  const EvaluationResult result = policy.evaluate(m, defaultParams().px4_action_grace_sec - 0.01);
  EXPECT_EQ(entryFor(result, ViolationCode::kBatteryPx4NoAction).status, ViolationStatus::kOk);
}

TEST(BatteryPx4NoAction, FailsafeActiveExemptsEvenPastGrace)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.battery_remaining = defaultParams().bat_crit_thr_copy - 0.01;
  m.failsafe_active = true;
  double now = 0.0;
  EvaluationResult result;
  for (; now <= defaultParams().px4_action_grace_sec + 0.5; now += 0.1) {
    result = policy.evaluate(m, now);
  }
  EXPECT_EQ(entryFor(result, ViolationCode::kBatteryPx4NoAction).status, ViolationStatus::kOk);
}

TEST(BatteryPx4NoAction, ReturnModeExemptsEvenPastGrace)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.battery_remaining = defaultParams().bat_crit_thr_copy - 0.01;
  m.flight_mode = kFlightModeReturn;
  double now = 0.0;
  EvaluationResult result;
  for (; now <= defaultParams().px4_action_grace_sec + 0.5; now += 0.1) {
    result = policy.evaluate(m, now);
  }
  EXPECT_EQ(entryFor(result, ViolationCode::kBatteryPx4NoAction).status, ViolationStatus::kOk);
}

// B3: the gap between this package's own battery_critical_threshold (0.10)
// and PX4's REAL bat_crit_thr_copy (0.07) must NEVER trigger -- PX4 has had
// no real chance to act yet in that band, that is not a PX4 failure.
TEST(BatteryPx4NoAction, BetweenOurThresholdAndPx4sRealThresholdNeverTriggers)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  ASSERT_LT(defaultParams().bat_crit_thr_copy, defaultParams().battery_critical_threshold)
    << "test assumes PX4's real threshold sits below this package's own (measured 2026-08-21)";
  m.battery_remaining =
    (defaultParams().bat_crit_thr_copy + defaultParams().battery_critical_threshold) / 2.0;
  double now = 0.0;
  EvaluationResult result;
  for (; now <= defaultParams().px4_action_grace_sec + 5.0; now += 0.1) {
    result = policy.evaluate(m, now);
  }
  EXPECT_EQ(entryFor(result, ViolationCode::kBatteryPx4NoAction).status, ViolationStatus::kOk)
    << "battery=" << m.battery_remaining << " is below OUR threshold but ABOVE PX4's real one";
}

TEST(BatteryPx4NoAction, LandModeExemptsEvenPastGrace)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.battery_remaining = defaultParams().bat_crit_thr_copy - 0.01;
  m.flight_mode = kFlightModeLand;
  double now = 0.0;
  EvaluationResult result;
  for (; now <= defaultParams().px4_action_grace_sec + 0.5; now += 0.1) {
    result = policy.evaluate(m, now);
  }
  EXPECT_EQ(entryFor(result, ViolationCode::kBatteryPx4NoAction).status, ViolationStatus::kOk);
}

// ===========================================================================
// OBSTACLE_TOO_CLOSE (S:3 row 8): distance, map freshness, localization gate.
// ===========================================================================

TEST(ObstacleTooClose, CloseWithFreshMapAndGoodLocalizationTriggersPastGrace)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m - 0.01;
  double now = 0.0;
  EvaluationResult result;
  for (; now <= defaultParams().obstacle_grace_sec + 0.3; now += 0.05) {
    result = policy.evaluate(m, now);
  }
  EXPECT_EQ(entryFor(result, ViolationCode::kObstacleTooClose).status, ViolationStatus::kViolating);
  EXPECT_EQ(result.target_action, FailsafeAction::kHold);
}

TEST(ObstacleTooClose, JustBeforeGraceDoesNotYetTrigger)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m - 0.01;
  policy.evaluate(m, 0.0);
  const EvaluationResult result = policy.evaluate(m, defaultParams().obstacle_grace_sec - 0.01);
  EXPECT_EQ(entryFor(result, ViolationCode::kObstacleTooClose).status, ViolationStatus::kOk);
}

TEST(ObstacleTooClose, CloseButMapStaleDoesNotTrigger)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m - 0.01;
  m.obstacle_map_age_sec = defaultParams().obstacle_map_timeout_copy_sec + 0.01;
  double now = 0.0;
  EvaluationResult result;
  for (; now <= defaultParams().obstacle_grace_sec + 0.3; now += 0.05) {
    result = policy.evaluate(m, now);
  }
  EXPECT_EQ(entryFor(result, ViolationCode::kObstacleTooClose).status, ViolationStatus::kOk);
}

TEST(ObstacleTooClose, CloseButLocalizationNotTrustedDoesNotTrigger)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m - 0.01;
  m.localization_good_for_obstacles = false;
  double now = 0.0;
  EvaluationResult result;
  for (; now <= defaultParams().obstacle_grace_sec + 0.3; now += 0.05) {
    result = policy.evaluate(m, now);
  }
  EXPECT_EQ(entryFor(result, ViolationCode::kObstacleTooClose).status, ViolationStatus::kOk);
}

TEST(ObstacleTooClose, NanDistanceCannotMeasureAndTakesNoAction)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.obstacle_min_distance_m = kNan;
  const EvaluationResult result = policy.evaluate(m, 0.0);
  EXPECT_EQ(entryFor(result, ViolationCode::kObstacleTooClose).status, ViolationStatus::kCannotMeasure);
  EXPECT_EQ(result.target_action, FailsafeAction::kNone);
}

// P8.5 bug fix (found via G-S1 item 8): +inf (buildMeasurements()' "empty
// obstacle map = nothing detected" convention) must read as a genuinely
// MEASURED, clear reading -- OK, never CANNOT_MEASURE. A !isfinite() guard
// used to swallow +inf the same as NaN, which then made clearFault() reject
// OBSTACLE_TOO_CLOSE forever whenever the map was truly, permanently empty.
TEST(ObstacleTooClose, InfiniteDistanceIsOkNotCannotMeasure)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.obstacle_min_distance_m = kInf;
  const EvaluationResult result = policy.evaluate(m, 0.0);
  EXPECT_EQ(entryFor(result, ViolationCode::kObstacleTooClose).status, ViolationStatus::kOk);
  EXPECT_EQ(result.target_action, FailsafeAction::kNone);
}

// ===========================================================================
// OFFBOARD_UNHEALTHY (S:3 row 9): STATE_FAULT or rate floor, REPORT only.
// ===========================================================================

TEST(OffboardUnhealthy, StateFaultTriggersPastGraceButNeverCuts)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.offboard_state = kOffboardStateFault;
  double now = 0.0;
  EvaluationResult result;
  for (; now <= defaultParams().offboard_grace_sec + 0.3; now += 0.05) {
    result = policy.evaluate(m, now);
  }
  EXPECT_EQ(entryFor(result, ViolationCode::kOffboardUnhealthy).status, ViolationStatus::kViolating);
  EXPECT_EQ(result.target_action, FailsafeAction::kNone) << "REPORT-only, must never cut";
  EXPECT_EQ(result.state, SupervisorState::kNormal);
}

TEST(OffboardUnhealthy, RateJustBelowFloorTriggersPastGrace)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.offboard_setpoint_rate_hz = defaultParams().offboard_min_rate_hz - 0.1;
  double now = 0.0;
  EvaluationResult result;
  for (; now <= defaultParams().offboard_grace_sec + 0.3; now += 0.05) {
    result = policy.evaluate(m, now);
  }
  EXPECT_EQ(entryFor(result, ViolationCode::kOffboardUnhealthy).status, ViolationStatus::kViolating);
}

TEST(OffboardUnhealthy, RateAtOrAboveFloorIsOk)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.offboard_setpoint_rate_hz = defaultParams().offboard_min_rate_hz;
  const EvaluationResult result = policy.evaluate(m, 0.0);
  EXPECT_EQ(entryFor(result, ViolationCode::kOffboardUnhealthy).status, ViolationStatus::kOk);
}

TEST(OffboardUnhealthy, NotReceivedCannotMeasure)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.offboard_status_received = false;
  const EvaluationResult result = policy.evaluate(m, 0.0);
  EXPECT_EQ(entryFor(result, ViolationCode::kOffboardUnhealthy).status, ViolationStatus::kCannotMeasure);
}

// ===========================================================================
// P8.4b: command_selected_age_sec is a second, faster detection path --
// real-flight finding, the original STATE_FAULT/rate path measured +2.66 s
// end-to-end, 1.07 s slower than PX4's own failsafe (G-S3 B1).
// ===========================================================================

TEST(OffboardUnhealthy, SelectedStaleTriggersPastGraceEvenWithHealthyRateAndState)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  // offboard_state/rate stay healthy throughout -- only the new path fires.
  m.command_selected_age_sec = defaultParams().selected_stale_sec + 0.01;
  double now = 0.0;
  EvaluationResult result;
  for (; now <= defaultParams().offboard_grace_sec + 0.3; now += 0.05) {
    result = policy.evaluate(m, now);
  }
  EXPECT_EQ(entryFor(result, ViolationCode::kOffboardUnhealthy).status, ViolationStatus::kViolating);
  EXPECT_EQ(result.target_action, FailsafeAction::kNone) << "still REPORT-only, must never cut";
}

TEST(OffboardUnhealthy, SelectedStaleJustBelowThresholdDoesNotTrigger)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.command_selected_age_sec = defaultParams().selected_stale_sec;   // == threshold, not >
  double now = 0.0;
  EvaluationResult result;
  for (; now <= defaultParams().offboard_grace_sec + 0.3; now += 0.05) {
    result = policy.evaluate(m, now);
  }
  EXPECT_EQ(entryFor(result, ViolationCode::kOffboardUnhealthy).status, ViolationStatus::kOk);
}

TEST(OffboardUnhealthy, SelectedStaleJustBeforeGraceDoesNotYetTrigger)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.command_selected_age_sec = defaultParams().selected_stale_sec + 0.01;
  policy.evaluate(m, 0.0);
  const EvaluationResult result = policy.evaluate(m, defaultParams().offboard_grace_sec - 0.01);
  EXPECT_EQ(entryFor(result, ViolationCode::kOffboardUnhealthy).status, ViolationStatus::kOk);
}

// ===========================================================================
// ESTIMATOR_INPUT_INVALID / CAMERA_STREAM_UNHEALTHY (S:3 row 10): REPORT only.
// ===========================================================================

TEST(EstimatorInputInvalid, AnyFlagFalseViolates)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.gps_ok = false;
  const EvaluationResult result = policy.evaluate(m, 0.0);
  EXPECT_EQ(entryFor(result, ViolationCode::kEstimatorInputInvalid).status, ViolationStatus::kViolating);
  EXPECT_EQ(result.target_action, FailsafeAction::kNone) << "REPORT-only, must never cut";
}

TEST(EstimatorInputInvalid, NotReceivedCannotMeasure)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.vehicle_health_received = false;
  // The node computes estimator_flags_trustworthy FROM vehicle_health_
  // received/fresh (buildMeasurements()) -- a lib-level test sets both
  // directly to pin the same relationship without the node's plumbing.
  m.estimator_flags_trustworthy = false;
  const EvaluationResult result = policy.evaluate(m, 0.0);
  EXPECT_EQ(entryFor(result, ViolationCode::kEstimatorInputInvalid).status, ViolationStatus::kCannotMeasure);
}

// Coordinator addition (post-B2 backend fix, review 2026-08-21): received
// AND fresh, but overall_level==LEVEL_UNKNOWN (backend has no real
// failsafe_flags yet) -- imu_ok/gps_ok/barometer_ok/magnetometer_ok default
// fabricated-true in that case and must NEVER be trusted alone.
TEST(EstimatorInputInvalid, ReceivedButUntrustworthyIsCannotMeasureNotOk)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.vehicle_health_received = true;
  m.estimator_flags_trustworthy = false;   // LEVEL_UNKNOWN, per node's buildMeasurements()
  // Fabricated defaults: all four still read "true" in the message struct.
  m.imu_ok = true;
  m.gps_ok = true;
  m.barometer_ok = true;
  m.magnetometer_ok = true;
  const EvaluationResult result = policy.evaluate(m, 0.0);
  EXPECT_EQ(entryFor(result, ViolationCode::kEstimatorInputInvalid).status, ViolationStatus::kCannotMeasure)
    << "fabricated-true flags must never be silently accepted as a healthy reading";
}

TEST(CameraStreamUnhealthy, NotOkViolates)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.camera_stream_ok = false;
  const EvaluationResult result = policy.evaluate(m, 0.0);
  EXPECT_EQ(entryFor(result, ViolationCode::kCameraStreamUnhealthy).status, ViolationStatus::kViolating);
  EXPECT_EQ(result.target_action, FailsafeAction::kNone);
}

TEST(CameraStreamUnhealthy, NotReceivedCannotMeasure)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.camera_health_received = false;
  const EvaluationResult result = policy.evaluate(m, 0.0);
  EXPECT_EQ(entryFor(result, ViolationCode::kCameraStreamUnhealthy).status, ViolationStatus::kCannotMeasure);
}

// ===========================================================================
// FRAME_MISMATCH (S:3 row 11): sustained drops (graced) OR fused mismatch
// (instantaneous).
// ===========================================================================

TEST(FrameMismatch, FusedFrameMismatchIsInstantaneous)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.fused_frame_mismatch = true;
  const EvaluationResult result = policy.evaluate(m, 0.0);
  EXPECT_EQ(entryFor(result, ViolationCode::kFrameMismatch).status, ViolationStatus::kViolating);
  EXPECT_EQ(result.target_action, FailsafeAction::kInhibit);
}

TEST(FrameMismatch, SustainedDropsTriggerPastGrace)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.active_channel_dropping_wrong_frame = true;
  double now = 0.0;
  EvaluationResult result;
  for (; now <= defaultParams().frame_mismatch_grace_sec + 0.3; now += 0.05) {
    result = policy.evaluate(m, now);
  }
  EXPECT_EQ(entryFor(result, ViolationCode::kFrameMismatch).status, ViolationStatus::kViolating);
}

TEST(FrameMismatch, JustBeforeGraceDoesNotYetTrigger)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.active_channel_dropping_wrong_frame = true;
  policy.evaluate(m, 0.0);
  const EvaluationResult result = policy.evaluate(m, defaultParams().frame_mismatch_grace_sec - 0.01);
  EXPECT_EQ(entryFor(result, ViolationCode::kFrameMismatch).status, ViolationStatus::kOk);
}

// ===========================================================================
// B5 (plan S:2, review 2026-08-21): armed && OFFBOARD && command_fresh gates
// the 4 LATCHING codes only. OBSTACLE_TOO_CLOSE (HOLD) is the representative
// code here -- the gate lives in evaluate() itself, identical for all 4.
// ===========================================================================

TEST(ActionGate, DisarmedNeverLatchesEvenWhenRawConditionIsTrue)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.armed = false;   // POSITIVELY confirmed disarmed (vehicle_state_received stays true)
  m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m - 0.01;
  double now = 0.0;
  EvaluationResult result;
  for (; now <= defaultParams().obstacle_grace_sec + 0.3; now += 0.05) {
    result = policy.evaluate(m, now);
  }
  EXPECT_EQ(entryFor(result, ViolationCode::kObstacleTooClose).status, ViolationStatus::kViolating)
    << "the MEASUREMENT must still be reported true -- only the ACTION is gated";
  EXPECT_EQ(result.state, SupervisorState::kNormal) << "disarmed must never let this latch/seize control";
  EXPECT_FALSE(policy.isLatched(ViolationCode::kObstacleTooClose));
}

TEST(ActionGate, NonOffboardFlightModeNeverLatches)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.flight_mode = kFlightModeUnknown;   // armed, but not OFFBOARD (e.g. POSITION mode)
  m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m - 0.01;
  double now = 0.0;
  EvaluationResult result;
  for (; now <= defaultParams().obstacle_grace_sec + 0.3; now += 0.05) {
    result = policy.evaluate(m, now);
  }
  EXPECT_EQ(result.state, SupervisorState::kNormal);
  EXPECT_FALSE(policy.isLatched(ViolationCode::kObstacleTooClose));
}

TEST(ActionGate, StaleCommandSelectedNeverLatches)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.command_fresh = false;   // armed + OFFBOARD, but command_selected itself stale
  m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m - 0.01;
  double now = 0.0;
  EvaluationResult result;
  for (; now <= defaultParams().obstacle_grace_sec + 0.3; now += 0.05) {
    result = policy.evaluate(m, now);
  }
  EXPECT_EQ(result.state, SupervisorState::kNormal);
  EXPECT_FALSE(policy.isLatched(ViolationCode::kObstacleTooClose));
}

TEST(ActionGate, ArmedOffboardFreshCommandLatchesNormally)
{
  // Positive counter-check (R21): the healthy baseline (armed+OFFBOARD+
  // fresh) must still latch exactly as before B5 -- the gate adds a NEW way
  // to suppress the action, it must never suppress the NORMAL flying case.
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m - 0.01;
  double now = 0.0;
  EvaluationResult result;
  for (; now <= defaultParams().obstacle_grace_sec + 0.3; now += 0.05) {
    result = policy.evaluate(m, now);
  }
  EXPECT_EQ(result.state, SupervisorState::kEngagingHold);
  EXPECT_TRUE(policy.isLatched(ViolationCode::kObstacleTooClose));
}

TEST(ActionGate, FailOpenWhenVehicleStateNeverReceivedStillLatches)
{
  // Fail-OPEN (R0): an UNMEASURABLE armed/flight_mode must never be read as
  // "confirmed grounded" and used to suppress protection -- only a
  // POSITIVELY CONFIRMED disarmed/non-OFFBOARD state may gate the latch off.
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.vehicle_state_received = false;   // e.g. VehicleState went stale (B2)
  m.armed = false;                    // stale cached value, irrelevant once unmeasurable
  m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m - 0.01;
  double now = 0.0;
  EvaluationResult result;
  for (; now <= defaultParams().obstacle_grace_sec + 0.3; now += 0.05) {
    result = policy.evaluate(m, now);
  }
  EXPECT_EQ(result.state, SupervisorState::kEngagingHold)
    << "an unmeasurable vehicle_state must never suppress protection";
  EXPECT_TRUE(policy.isLatched(ViolationCode::kObstacleTooClose));
}

TEST(ActionGate, ReportOnlyCodesAreNeverGated)
{
  // BATTERY_WARN is REPORT-only -- must violate regardless of armed state.
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.armed = false;
  m.battery_remaining = defaultParams().battery_warn_threshold - 0.01;
  const EvaluationResult result = policy.evaluate(m, 0.0);
  EXPECT_EQ(entryFor(result, ViolationCode::kBatteryWarn).status, ViolationStatus::kViolating)
    << "REPORT-only codes must stay visible even while disarmed/parked";
}

// ===========================================================================
// FSM (S:4.3): NORMAL -> ENGAGING_HOLD -> HOLDING,
// NORMAL -> ENGAGING_INHIBIT -> INHIBITED, HOLDING -> INHIBITED one-way,
// INHIBITED -> HOLDING forbidden, only ClearFault returns to NORMAL.
// ===========================================================================

TEST(Fsm, ObstacleClosePastGraceEntersEngagingHold)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m - 0.01;
  double now = 0.0;
  EvaluationResult result;
  for (; now <= defaultParams().obstacle_grace_sec + 0.3; now += 0.05) {
    result = policy.evaluate(m, now);
  }
  EXPECT_EQ(result.state, SupervisorState::kEngagingHold);
}

TEST(Fsm, ConfirmHoldEngagedWithFreshPoseReachesHolding)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m - 0.01;
  double now = 0.0;
  for (; now <= defaultParams().obstacle_grace_sec + 0.3; now += 0.05) {
    policy.evaluate(m, now);
  }
  ASSERT_EQ(policy.state(), SupervisorState::kEngagingHold);
  EXPECT_TRUE(policy.confirmHoldEngaged(0.05, now));
  EXPECT_EQ(policy.state(), SupervisorState::kHolding);
}

TEST(Fsm, ConfirmHoldEngagedWithStalePoseEscalatesStraightToEngagingInhibit)
{
  // S:4: "khong du => di thang INHIBIT" -- HOLD must never freeze an
  // untrustworthy pose.
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m - 0.01;
  double now = 0.0;
  for (; now <= defaultParams().obstacle_grace_sec + 0.3; now += 0.05) {
    policy.evaluate(m, now);
  }
  ASSERT_EQ(policy.state(), SupervisorState::kEngagingHold);
  EXPECT_TRUE(policy.confirmHoldEngaged(defaultParams().pose_max_age_sec + 0.01, now));
  EXPECT_EQ(policy.state(), SupervisorState::kEngagingInhibit);
  EXPECT_NE(policy.state(), SupervisorState::kHolding);
}

TEST(Fsm, ConfirmHoldEngagedIsANoOpFromAnyOtherState)
{
  FailsafePolicy policy(defaultParams());
  ASSERT_EQ(policy.state(), SupervisorState::kNormal);
  EXPECT_FALSE(policy.confirmHoldEngaged(0.0, 0.0));
  EXPECT_EQ(policy.state(), SupervisorState::kNormal);
}

TEST(Fsm, BlindCommandPastGraceEntersEngagingInhibitThenConfirmReachesInhibited)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.localization_is_valid = false;
  m.command_setpoint_displacement_m = defaultParams().frozen_setpoint_epsilon_copy_m + 0.01;
  m.command_selected_source = kSourceMission;
  double now = 0.0;
  for (; now <= defaultParams().blind_grace_sec + 0.3; now += 0.05) {
    policy.evaluate(m, now);
  }
  ASSERT_EQ(policy.state(), SupervisorState::kEngagingInhibit);
  EXPECT_TRUE(policy.confirmInhibitEngaged());
  EXPECT_EQ(policy.state(), SupervisorState::kInhibited);
}

TEST(Fsm, ConfirmInhibitEngagedIsANoOpFromAnyOtherState)
{
  FailsafePolicy policy(defaultParams());
  EXPECT_FALSE(policy.confirmInhibitEngaged());
  EXPECT_EQ(policy.state(), SupervisorState::kNormal);
}

TEST(Fsm, HoldingEscalatesToEngagingInhibitOneWayWhenANewInhibitCauseAppears)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m - 0.01;
  double now = 0.0;
  for (; now <= defaultParams().obstacle_grace_sec + 0.3; now += 0.05) {
    policy.evaluate(m, now);
  }
  ASSERT_TRUE(policy.confirmHoldEngaged(0.05, now));
  ASSERT_EQ(policy.state(), SupervisorState::kHolding);

  // A new INHIBIT-class cause appears while HOLDING.
  m.fused_frame_mismatch = true;
  const EvaluationResult result = policy.evaluate(m, now + 0.05);
  EXPECT_EQ(result.state, SupervisorState::kEngagingInhibit);
}

// ===========================================================================
// P8.5: HOLDING -> INHIBITED one-way when the fused pose itself is lost
// mid-hold -- independent of the "new INHIBIT-class cause" path above.
// ===========================================================================

TEST(Fsm, HoldingEscalatesOneWayWhenPoseIsLostMidHold)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m - 0.01;
  double now = 0.0;
  for (; now <= defaultParams().obstacle_grace_sec + 0.3; now += 0.05) {
    policy.evaluate(m, now);
  }
  ASSERT_TRUE(policy.confirmHoldEngaged(0.05, now));
  ASSERT_EQ(policy.state(), SupervisorState::kHolding);

  // Obstacle cause clears -- the ONLY thing left driving this transition is
  // pose staleness, isolating the new path from the already-pinned "new
  // INHIBIT-class cause" test above.
  m.obstacle_min_distance_m = 10.0;
  m.fused_odometry_age_sec = defaultParams().pose_max_age_sec + 0.01;
  const EvaluationResult result = policy.evaluate(m, now + 0.05);
  EXPECT_EQ(result.state, SupervisorState::kEngagingInhibit);
  EXPECT_TRUE(policy.isLatched(ViolationCode::kObstacleTooClose))
    << "pose loss moves the ceiling, it must not clear the original latch";
}

TEST(Fsm, HoldingEscalatesWhenPoseNeverArrivesAgain)
{
  // +inf age (R27-2 "never received" convention) must escalate exactly like
  // a finite stale age -- an isfinite() guard must not read silence as fine.
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m - 0.01;
  double now = 0.0;
  for (; now <= defaultParams().obstacle_grace_sec + 0.3; now += 0.05) {
    policy.evaluate(m, now);
  }
  ASSERT_TRUE(policy.confirmHoldEngaged(0.05, now));
  m.fused_odometry_age_sec = kInf;
  const EvaluationResult result = policy.evaluate(m, now + 0.05);
  EXPECT_EQ(result.state, SupervisorState::kEngagingInhibit);
}

TEST(Fsm, HoldingStaysHoldingWhenPoseAgeIsExactlyAtTheCeiling)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m - 0.01;
  double now = 0.0;
  for (; now <= defaultParams().obstacle_grace_sec + 0.3; now += 0.05) {
    policy.evaluate(m, now);
  }
  ASSERT_TRUE(policy.confirmHoldEngaged(0.05, now));
  m.fused_odometry_age_sec = defaultParams().pose_max_age_sec;   // == ceiling, not past it
  const EvaluationResult result = policy.evaluate(m, now + 0.05);
  EXPECT_EQ(result.state, SupervisorState::kHolding);
}

TEST(Fsm, LatchedHoldDoesNotAutoReturnToNormalWhenTheCauseClearsWithoutClearFault)
{
  // S:4.3: "chi ClearFault -> NORMAL" -- the underlying condition clearing
  // on its own must never silently drop the latch.
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m - 0.01;
  double now = 0.0;
  for (; now <= defaultParams().obstacle_grace_sec + 0.3; now += 0.05) {
    policy.evaluate(m, now);
  }
  ASSERT_TRUE(policy.confirmHoldEngaged(0.05, now));
  ASSERT_EQ(policy.state(), SupervisorState::kHolding);

  m.obstacle_min_distance_m = 10.0;   // cause fully gone
  const EvaluationResult result = policy.evaluate(m, now + 1.0);
  EXPECT_EQ(result.state, SupervisorState::kHolding);
  EXPECT_TRUE(policy.isLatched(ViolationCode::kObstacleTooClose));
}

// ===========================================================================
// ClearFault (S:3.1). The 3 rejection paths, the accept path, and the
// forbidden-transition mutation guard (INHIBITED must never step to HOLDING,
// even via a partial clear).
// ===========================================================================

class ClearFaultFixture : public ::testing::Test
{
protected:
  FailsafePolicy policy_{defaultParams()};

  double driveObstacleIntoHolding()
  {
    Measurements m = healthyMeasurements();
    m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m - 0.01;
    double now = 0.0;
    for (; now <= defaultParams().obstacle_grace_sec + 0.3; now += 0.05) {
      policy_.evaluate(m, now);
    }
    policy_.confirmHoldEngaged(0.05, now);
    return now;
  }

  double driveFrameMismatchIntoInhibitedOnTopOfHolding(double now)
  {
    Measurements m = healthyMeasurements();
    m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m - 0.01;   // stays latched
    m.fused_frame_mismatch = true;
    now += 0.05;
    policy_.evaluate(m, now);
    policy_.confirmInhibitEngaged();
    return now;
  }
};

TEST_F(ClearFaultFixture, ClearFaultWithNothingLatchedIsATrivialSuccess)
{
  const ClearFaultResult result = policy_.clearFault("", healthyMeasurements(), 0.0);
  EXPECT_TRUE(result.success);
  EXPECT_EQ(policy_.state(), SupervisorState::kNormal);
}

TEST_F(ClearFaultFixture, RejectsWhenCauseCannotBeMeasured)
{
  double now = driveObstacleIntoHolding();
  ASSERT_EQ(policy_.state(), SupervisorState::kHolding);

  Measurements m = healthyMeasurements();
  m.obstacle_min_distance_m = kNan;   // cannot confirm the cause is gone
  const ClearFaultResult result = policy_.clearFault("", m, now + 0.1);
  EXPECT_FALSE(result.success);
  ASSERT_EQ(result.remaining_faults.size(), 1U);
  EXPECT_NE(result.remaining_faults[0].find("cannot measure"), std::string::npos);
  EXPECT_EQ(policy_.state(), SupervisorState::kHolding);
}

TEST_F(ClearFaultFixture, RejectsWhenCauseIsStillActive)
{
  double now = driveObstacleIntoHolding();
  Measurements m = healthyMeasurements();
  m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m - 0.01;   // still close
  const ClearFaultResult result = policy_.clearFault("", m, now + 0.1);
  EXPECT_FALSE(result.success);
  ASSERT_EQ(result.remaining_faults.size(), 1U);
  EXPECT_NE(result.remaining_faults[0].find("measured="), std::string::npos);
  EXPECT_NE(result.remaining_faults[0].find("threshold="), std::string::npos);
  EXPECT_EQ(policy_.state(), SupervisorState::kHolding);
}

TEST_F(ClearFaultFixture, RejectsWhenNotYetStableForClearStabilitySeconds)
{
  double now = driveObstacleIntoHolding();
  Measurements m = healthyMeasurements();
  m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m +
    defaultParams().obstacle_clear_margin_m + 0.5;   // clearly clear, but only this one call so far
  const ClearFaultResult first = policy_.clearFault("", m, now + 0.1);
  EXPECT_FALSE(first.success) << "stable_for is 0 on the first measured-clear call";
  EXPECT_EQ(policy_.state(), SupervisorState::kHolding);
}

TEST_F(ClearFaultFixture, AcceptsAfterPollingClearForClearStabilitySeconds)
{
  double now = driveObstacleIntoHolding();
  Measurements m = healthyMeasurements();
  m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m +
    defaultParams().obstacle_clear_margin_m + 0.5;

  ClearFaultResult result;
  double t = now + 0.1;
  for (; t <= now + 0.1 + defaultParams().clear_stability_sec + 0.2; t += 0.5) {
    result = policy_.clearFault("", m, t);
    if (result.success) {
      break;
    }
  }
  EXPECT_TRUE(result.success);
  EXPECT_EQ(policy_.state(), SupervisorState::kNormal);
}

// P8.5 bug fix pin (found via G-S1 item 8): a TRULY empty obstacle map
// (+inf, not just "far away but finite") must be accepted by clearFault() --
// the old !isfinite()-based gate rejected +inf as CANNOT_MEASURE forever,
// which the finite-distance test above (min_obstacle_distance_copy_m + margin +
// 0.5) never exercised.
TEST_F(ClearFaultFixture, AcceptsWhenObstacleMapIsGenuinelyEmptyAfterPollingClear)
{
  double now = driveObstacleIntoHolding();
  Measurements m = healthyMeasurements();
  m.obstacle_min_distance_m = kInf;

  ClearFaultResult result;
  double t = now + 0.1;
  for (; t <= now + 0.1 + defaultParams().clear_stability_sec + 0.2; t += 0.5) {
    result = policy_.clearFault("", m, t);
    if (result.success) {
      break;
    }
  }
  EXPECT_TRUE(result.success);
  EXPECT_EQ(policy_.state(), SupervisorState::kNormal);
}

TEST_F(ClearFaultFixture, RejectsOnHandoverJumpEvenWhenCauseIsClear)
{
  double now = driveObstacleIntoHolding();
  Measurements m = healthyMeasurements();
  m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m +
    defaultParams().obstacle_clear_margin_m + 0.5;
  m.cmd_mission_live = true;
  m.cmd_mission_setpoint_distance_from_fused_m = defaultParams().handover_jump_limit_m + 0.5;

  ClearFaultResult result;
  double t = now + 0.1;
  for (; t <= now + 0.1 + defaultParams().clear_stability_sec + 0.2; t += 0.5) {
    result = policy_.clearFault("", m, t);
  }
  EXPECT_FALSE(result.success);
  bool found_jump_reason = false;
  for (const std::string & detail : result.remaining_faults) {
    if (detail.find("HANDOVER_JUMP") != std::string::npos) {
      found_jump_reason = true;
    }
  }
  EXPECT_TRUE(found_jump_reason);
  EXPECT_EQ(policy_.state(), SupervisorState::kHolding);
}

TEST_F(ClearFaultFixture, InhibitedNeverStepsDownToHoldingOnAPartialClear)
{
  // The core forbidden-transition pin (S:4.3), and the exact trap the
  // "recompute ceiling from surviving latch set" bug (fixed during P8.1
  // review) would have fallen into: FRAME_MISMATCH clears while
  // OBSTACLE_TOO_CLOSE (a HOLD-class code) is still latched underneath.
  double now = driveObstacleIntoHolding();
  now = driveFrameMismatchIntoInhibitedOnTopOfHolding(now);
  ASSERT_EQ(policy_.state(), SupervisorState::kInhibited);
  ASSERT_TRUE(policy_.isLatched(ViolationCode::kObstacleTooClose));
  ASSERT_TRUE(policy_.isLatched(ViolationCode::kFrameMismatch));

  Measurements m = healthyMeasurements();
  m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m - 0.01;   // still latched, on purpose
  m.fused_frame_mismatch = false;
  m.active_channel_dropping_wrong_frame = false;

  ClearFaultResult result;
  double t = now + 0.1;
  for (; t <= now + 0.1 + defaultParams().clear_stability_sec + 0.2; t += 0.5) {
    result = policy_.clearFault("FRAME_MISMATCH", m, t);
    ASSERT_NE(policy_.state(), SupervisorState::kHolding)
      << "INHIBITED must never step down to HOLDING, even mid-poll";
  }
  EXPECT_TRUE(result.success);
  EXPECT_EQ(policy_.state(), SupervisorState::kInhibited)
    << "FRAME_MISMATCH cleared but OBSTACLE_TOO_CLOSE is still latched";
  EXPECT_FALSE(policy_.isLatched(ViolationCode::kFrameMismatch));
  EXPECT_TRUE(policy_.isLatched(ViolationCode::kObstacleTooClose));
}

TEST_F(ClearFaultFixture, FullClearOfEveryLatchedCodeReturnsToNormal)
{
  double now = driveObstacleIntoHolding();
  now = driveFrameMismatchIntoInhibitedOnTopOfHolding(now);
  ASSERT_EQ(policy_.state(), SupervisorState::kInhibited);

  Measurements m = healthyMeasurements();
  m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m +
    defaultParams().obstacle_clear_margin_m + 0.5;
  m.fused_frame_mismatch = false;
  m.active_channel_dropping_wrong_frame = false;

  ClearFaultResult result;
  double t = now + 0.1;
  for (; t <= now + 0.1 + defaultParams().clear_stability_sec + 0.2; t += 0.5) {
    result = policy_.clearFault("", m, t);
    if (result.success) {
      break;
    }
  }
  EXPECT_TRUE(result.success);
  EXPECT_EQ(policy_.state(), SupervisorState::kNormal);
  EXPECT_FALSE(policy_.isLatched(ViolationCode::kObstacleTooClose));
  EXPECT_FALSE(policy_.isLatched(ViolationCode::kFrameMismatch));
}

TEST_F(ClearFaultFixture, RequestingAFaultCodeNotCurrentlyLatchedIsATrivialSuccess)
{
  double now = driveObstacleIntoHolding();
  ASSERT_EQ(policy_.state(), SupervisorState::kHolding);
  const ClearFaultResult result = policy_.clearFault("FRAME_MISMATCH", healthyMeasurements(), now + 0.1);
  EXPECT_TRUE(result.success);
  EXPECT_EQ(policy_.state(), SupervisorState::kHolding)
    << "nothing was latched under that name, but the real latch must be untouched";
  EXPECT_TRUE(policy_.isLatched(ViolationCode::kObstacleTooClose));
}

// ===========================================================================
// S3 coverage, 2026-08-26. Three arms of this policy had never been executed by
// any test: the combined offboard detail, the HOLD-to-INHIBIT escalation that
// happens WHILE still engaging, and the battery clear condition. All three sit
// on the cut chain, and all three are reachable from the ordinary API.
// ===========================================================================

// Two independent detections of the same fault arriving together must not lose
// one of them: whoever reads the detail has to see BOTH, or a post-flight
// diagnosis blames the slower path alone.
TEST(OffboardUnhealthy, BothDetectionPathsAtOnceAreNamedTogether)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.offboard_state = kOffboardStateFault;
  m.command_selected_age_sec = defaultParams().selected_stale_sec + 0.01;

  double now = 0.0;
  EvaluationResult result;
  for (; now <= defaultParams().offboard_grace_sec + 0.3; now += 0.05) {
    result = policy.evaluate(m, now);
  }

  const ViolationEntry entry = entryFor(result, ViolationCode::kOffboardUnhealthy);
  ASSERT_EQ(entry.status, ViolationStatus::kViolating);
  EXPECT_NE(entry.detail.find("STATE_FAULT"), std::string::npos) << entry.detail;
  EXPECT_NE(entry.detail.find("command_selected stale"), std::string::npos) << entry.detail;
}

// S:4 escalation, on the path where the aircraft has NOT finished engaging HOLD
// yet. Going the other way -- INHIBIT relaxing into HOLD -- must never happen.
TEST(Fsm, AnInhibitArrivingWhileHoldIsStillEngagingEscalatesImmediately)
{
  FailsafePolicy policy(defaultParams());
  Measurements m = healthyMeasurements();
  m.obstacle_min_distance_m = defaultParams().min_obstacle_distance_copy_m - 0.01;

  double now = 0.0;
  for (; now <= defaultParams().obstacle_grace_sec + 0.3; now += 0.05) {
    policy.evaluate(m, now);
  }
  ASSERT_EQ(policy.state(), SupervisorState::kEngagingHold);

  // Battery goes critical with PX4 not acting: an INHIBIT-class fault (S:3 row 7),
  // higher than the obstacle HOLD already engaging.
  m.battery_remaining = defaultParams().bat_crit_thr_copy - 0.01;
  EvaluationResult result;
  for (double end = now + defaultParams().px4_action_grace_sec + 0.5; now <= end; now += 0.1) {
    result = policy.evaluate(m, now);
  }

  EXPECT_EQ(result.target_action, FailsafeAction::kInhibit);
  EXPECT_EQ(policy.state(), SupervisorState::kEngagingInhibit)
    << "an INHIBIT waited behind a HOLD that had not even engaged yet";
}

// The clear side of BATTERY_PX4_NO_ACTION: it must need the margin, not merely
// the threshold, or the violation chatters at the boundary.
TEST(BatteryPx4NoAction, ClearsOnlyOnceTheChargeIsBackAboveThresholdPlusMargin)
{
  const FailsafePolicyParams params = defaultParams();
  FailsafePolicy policy(params);
  Measurements m = healthyMeasurements();
  m.battery_remaining = params.bat_crit_thr_copy - 0.01;

  double now = 0.0;
  for (; now <= params.px4_action_grace_sec + 0.3; now += 0.05) {
    policy.evaluate(m, now);
  }
  ASSERT_TRUE(policy.isLatched(ViolationCode::kBatteryPx4NoAction))
    << "FAILED TO MEASURE: the violation never latched, so its clear path is untested";

  // clearFault is a POLL, not a single question: it only succeeds once the clear
  // condition has HELD for clear_stability_sec (CLAUDE.md: "ClearFault phai POLL").
  const auto pollClear = [&](double from) {
      ClearFaultResult out;
      for (double t = from; t <= from + params.clear_stability_sec + 0.5; t += 0.25) {
        out = policy.clearFault("BATTERY_PX4_NO_ACTION", m, t);
        if (out.success) {
          break;
        }
      }
      return out;
    };

  // Just above the bare threshold is NOT enough, however long it is polled.
  m.battery_remaining = params.battery_critical_threshold + 0.5 * params.battery_clear_margin;
  const ClearFaultResult refused = pollClear(now + 0.1);
  EXPECT_FALSE(refused.success) << "cleared inside the margin: " << refused.message;

  // Above threshold + margin, and held, it clears.
  m.battery_remaining = params.battery_critical_threshold + params.battery_clear_margin + 0.01;
  const ClearFaultResult accepted = pollClear(now + 10.0);
  EXPECT_TRUE(accepted.success) << accepted.message;
  EXPECT_FALSE(policy.isLatched(ViolationCode::kBatteryPx4NoAction));
}

}  // namespace uav_safety
