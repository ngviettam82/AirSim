#include "uav_safety/failsafe_policy.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace uav_safety
{

namespace
{

void require(bool condition, const std::string & detail)
{
  if (!condition) {
    throw std::invalid_argument("failsafe policy parameters: " + detail);
  }
}

bool isLatchingCode(ViolationCode code)
{
  const FailsafeAction action = violationCodeAction(code);
  return action == FailsafeAction::kHold || action == FailsafeAction::kInhibit;
}

}  // namespace

const char * violationCodeName(ViolationCode code)
{
  switch (code) {
    case ViolationCode::kLocalizationInvalid: return "LOCALIZATION_INVALID";
    case ViolationCode::kLocalizationStale: return "LOCALIZATION_STALE";
    case ViolationCode::kLocalizationJump: return "LOCALIZATION_JUMP";
    case ViolationCode::kBlindCommand: return "BLIND_COMMAND";
    case ViolationCode::kBatteryWarn: return "BATTERY_WARN";
    case ViolationCode::kBatteryCritical: return "BATTERY_CRITICAL";
    case ViolationCode::kBatteryPx4NoAction: return "BATTERY_PX4_NO_ACTION";
    case ViolationCode::kObstacleTooClose: return "OBSTACLE_TOO_CLOSE";
    case ViolationCode::kOffboardUnhealthy: return "OFFBOARD_UNHEALTHY";
    case ViolationCode::kEstimatorInputInvalid: return "ESTIMATOR_INPUT_INVALID";
    case ViolationCode::kCameraStreamUnhealthy: return "CAMERA_STREAM_UNHEALTHY";
    case ViolationCode::kFrameMismatch: return "FRAME_MISMATCH";
    default: return "UNKNOWN";
  }
}

FailsafeAction violationCodeAction(ViolationCode code)
{
  switch (code) {
    case ViolationCode::kBlindCommand: return FailsafeAction::kInhibit;
    case ViolationCode::kBatteryPx4NoAction: return FailsafeAction::kInhibit;
    case ViolationCode::kFrameMismatch: return FailsafeAction::kInhibit;
    case ViolationCode::kObstacleTooClose: return FailsafeAction::kHold;
    case ViolationCode::kLocalizationInvalid:
    case ViolationCode::kLocalizationStale:
    case ViolationCode::kLocalizationJump:
    case ViolationCode::kBatteryWarn:
    case ViolationCode::kBatteryCritical:
    case ViolationCode::kOffboardUnhealthy:
    case ViolationCode::kEstimatorInputInvalid:
    case ViolationCode::kCameraStreamUnhealthy:
      return FailsafeAction::kReport;
    default: return FailsafeAction::kNone;
  }
}

uint8_t violationCodeSeverity(ViolationCode code)
{
  switch (code) {
    // S:3 states the level explicitly for these five.
    case ViolationCode::kLocalizationInvalid: return kLevelEmergency;
    case ViolationCode::kLocalizationStale: return kLevelEmergency;
    case ViolationCode::kLocalizationJump: return kLevelError;
    case ViolationCode::kBatteryCritical: return kLevelEmergency;
    case ViolationCode::kOffboardUnhealthy: return kLevelError;
    // Every code that seizes or blocks control is treated as EMERGENCY --
    // this library's own choice, S:3 gives the action but not a level here.
    case ViolationCode::kBlindCommand: return kLevelEmergency;
    case ViolationCode::kBatteryPx4NoAction: return kLevelEmergency;
    case ViolationCode::kObstacleTooClose: return kLevelEmergency;
    case ViolationCode::kFrameMismatch: return kLevelEmergency;
    // Not leveled by S:3 -- this library's own reasonable default.
    case ViolationCode::kBatteryWarn: return kLevelWarning;
    case ViolationCode::kEstimatorInputInvalid: return kLevelError;
    case ViolationCode::kCameraStreamUnhealthy: return kLevelWarning;
    default: return kLevelUnknown;
  }
}

void FailsafePolicy::ContinuityTracker::update(bool condition_now, double now_sec)
{
  if (condition_now) {
    if (true_since_sec == kNotTrue) {
      true_since_sec = now_sec;
    }
  } else {
    true_since_sec = kNotTrue;
  }
}

double FailsafePolicy::ContinuityTracker::durationSec(double now_sec) const
{
  return isTrueNow() ? (now_sec - true_since_sec) : 0.0;
}

FailsafePolicy::FailsafePolicy(const FailsafePolicyParams & params)
: params_(params)
{
  validate();
}

void FailsafePolicy::validate() const
{
  auto positive = [](double v) {return std::isfinite(v) && v > 0.0;};
  auto nonNegative = [](double v) {return std::isfinite(v) && v >= 0.0;};

  require(positive(params_.status_timeout_sec), "status_timeout_sec must be finite and positive");
  require(positive(params_.fused_timeout_sec), "fused_timeout_sec must be finite and positive");
  require(
    nonNegative(params_.localization_invalid_grace_sec),
    "localization_invalid_grace_sec must be finite and non-negative");
  require(
    nonNegative(params_.localization_stale_grace_sec),
    "localization_stale_grace_sec must be finite and non-negative");
  require(
    nonNegative(params_.localization_jump_grace_sec),
    "localization_jump_grace_sec must be finite and non-negative");

  require(positive(params_.blind_grace_sec), "blind_grace_sec must be finite and positive");
  require(
    positive(params_.frozen_setpoint_epsilon_copy_m),
    "frozen_setpoint_epsilon_copy_m must be finite and positive");

  require(
    std::isfinite(params_.battery_warn_threshold) && params_.battery_warn_threshold > 0.0 &&
      params_.battery_warn_threshold < 1.0,
    "battery_warn_threshold must be in (0, 1)");
  require(
    std::isfinite(params_.battery_critical_threshold) &&
      params_.battery_critical_threshold > 0.0 && params_.battery_critical_threshold < 1.0,
    "battery_critical_threshold must be in (0, 1)");
  require(
    params_.battery_critical_threshold < params_.battery_warn_threshold,
    "battery_critical_threshold must be < battery_warn_threshold");
  require(
    nonNegative(params_.battery_warn_grace_sec), "battery_warn_grace_sec must be non-negative");
  require(
    nonNegative(params_.battery_critical_grace_sec),
    "battery_critical_grace_sec must be non-negative");

  require(positive(params_.px4_action_grace_sec), "px4_action_grace_sec must be finite and positive");

  require(
    positive(params_.min_obstacle_distance_copy_m), "min_obstacle_distance_copy_m must be finite and positive");
  require(positive(params_.obstacle_grace_sec), "obstacle_grace_sec must be finite and positive");
  require(
    nonNegative(params_.obstacle_clear_margin_m), "obstacle_clear_margin_m must be non-negative");
  require(
    positive(params_.world_model_publish_hz_copy), "world_model_publish_hz_copy must be finite and positive");
  require(
    positive(params_.obstacle_map_timeout_copy_sec), "obstacle_map_timeout_copy_sec must be finite and positive");

  require(
    positive(params_.offboard_min_rate_hz), "offboard_min_rate_hz must be finite and positive");
  require(positive(params_.offboard_grace_sec), "offboard_grace_sec must be finite and positive");
  require(positive(params_.selected_stale_sec), "selected_stale_sec must be finite and positive");
  require(
    positive(params_.downstream_command_timeout_copy_sec),
    "downstream_command_timeout_copy_sec must be finite and positive");

  require(nonNegative(params_.estimator_grace_sec), "estimator_grace_sec must be non-negative");
  require(nonNegative(params_.camera_grace_sec), "camera_grace_sec must be non-negative");

  // B2/Y1: freshness ceilings for the 4 cached-indefinitely subscriptions.
  require(
    positive(params_.vehicle_health_timeout_sec), "vehicle_health_timeout_sec must be finite and positive");
  require(
    positive(params_.vehicle_state_timeout_sec), "vehicle_state_timeout_sec must be finite and positive");
  require(
    positive(params_.offboard_status_timeout_sec),
    "offboard_status_timeout_sec must be finite and positive");
  require(
    positive(params_.camera_health_timeout_sec), "camera_health_timeout_sec must be finite and positive");

  // B3: real PX4 battery threshold copies, each a fraction in (0, 1), and
  // internally consistent with each other (catches a transcription error --
  // this package cannot re-verify them against PX4 at runtime).
  auto fraction = [](double v) {return std::isfinite(v) && v > 0.0 && v < 1.0;};
  require(fraction(params_.bat_low_thr_copy), "bat_low_thr_copy must be in (0, 1)");
  require(fraction(params_.bat_crit_thr_copy), "bat_crit_thr_copy must be in (0, 1)");
  require(fraction(params_.bat_emergen_thr_copy), "bat_emergen_thr_copy must be in (0, 1)");
  require(
    params_.bat_emergen_thr_copy < params_.bat_crit_thr_copy &&
      params_.bat_crit_thr_copy < params_.bat_low_thr_copy,
    "bat_emergen_thr_copy < bat_crit_thr_copy < bat_low_thr_copy must hold "
    "(PX4's own battery threshold ordering)");

  require(
    positive(params_.arbiter_diagnostics_period_copy_sec),
    "arbiter_diagnostics_period_copy_sec must be finite and positive");
  require(
    positive(params_.localization_diag_period_copy_sec),
    "localization_diag_period_copy_sec must be finite and positive");
  require(
    positive(params_.frame_mismatch_grace_sec), "frame_mismatch_grace_sec must be finite and positive");

  require(positive(params_.clear_stability_sec), "clear_stability_sec must be finite and positive");
  require(nonNegative(params_.battery_clear_margin), "battery_clear_margin must be non-negative");
  require(
    nonNegative(params_.localization_clear_margin_sec),
    "localization_clear_margin_sec must be non-negative");
  require(
    positive(params_.handover_jump_limit_m), "handover_jump_limit_m must be finite and positive");

  require(positive(params_.pose_max_age_sec), "pose_max_age_sec must be finite and positive");
  require(positive(params_.hold_stream_hz), "hold_stream_hz must be finite and positive");
  require(!params_.odom_frame.empty(), "odom_frame must not be empty");

  require(
    positive(params_.odometry_timeout_copy_sec), "odometry_timeout_copy_sec must be finite and positive");
  require(positive(params_.progress_hz_copy), "progress_hz_copy must be finite and positive");
  require(
    positive(params_.arbiter_source_timeout_copy_sec),
    "arbiter_source_timeout_copy_sec must be finite and positive");
  require(positive(params_.max_lead_copy_m), "max_lead_copy_m must be finite and positive");
  require(
    positive(params_.mux_publish_period_copy_sec),
    "mux_publish_period_copy_sec must be finite and positive");

  // S:9d pairing rules -- refuse to start on an unusable cross-package config,
  // same discipline as AuthorityArbiter::validate().
  require(
    params_.blind_grace_sec > params_.odometry_timeout_copy_sec + 2.0 / params_.progress_hz_copy,
    "blind_grace_sec must exceed odometry_timeout_copy_sec + 2/progress_hz_copy "
    "(must not race navigator's own layer-1 pose-staleness reaction)");
  require(
    params_.fused_timeout_sec < params_.odometry_timeout_copy_sec,
    "fused_timeout_sec must be < odometry_timeout_copy_sec (safety must know before navigator does)");
  require(
    params_.hold_stream_hz >= 3.0 / params_.arbiter_source_timeout_copy_sec,
    "hold_stream_hz must be >= 3/arbiter_source_timeout_copy_sec (else the arbiter "
    "would see HOLD's own stream as not-live)");
  require(
    params_.pose_max_age_sec <= 0.25,
    "pose_max_age_sec must be <= 0.25 s (S:4 contract ceiling, not a tunable)");
  require(
    params_.handover_jump_limit_m <= params_.max_lead_copy_m,
    "handover_jump_limit_m must be <= max_lead_copy_m (else ClearFault could hand back "
    "control to a jump navigator's own leash would never have allowed)");
  require(
    params_.selected_stale_sec >= params_.downstream_command_timeout_copy_sec,
    "selected_stale_sec must be >= downstream_command_timeout_copy_sec (P8.4b: else this would "
    "false-flag while the gateway is still legitimately buffering a not-yet-cut command)");
  require(
    params_.frame_mismatch_grace_sec >= 2.0 * params_.arbiter_diagnostics_period_copy_sec,
    "frame_mismatch_grace_sec must be >= 2 x arbiter_diagnostics_period_copy_sec (B1: the "
    "per-channel drop count only refreshes once per diagnostics cycle -- grace must span "
    ">= 2 cycles or a stale-but-unchanged reading between cycles can look like still rising)");
  require(
    params_.obstacle_map_timeout_copy_sec >= 2.0 / params_.world_model_publish_hz_copy,
    "obstacle_map_timeout_copy_sec must be >= 2/world_model_publish_hz_copy (same B1-class "
    "discipline: a freshness grace must outlive >= 2 sampling periods of the signal it grades)");
  require(
    params_.pose_max_age_sec >= 2.0 * params_.mux_publish_period_copy_sec,
    "pose_max_age_sec must be >= 2 x mux_publish_period_copy_sec (Y8: HOLD must never freeze "
    "a pose fresh enough to look trustworthy after only a single missed mux publish)");
}

bool FailsafePolicy::isLatched(ViolationCode code) const
{
  return latched_[static_cast<size_t>(code)];
}

bool FailsafePolicy::rawLocalizationBad(const Measurements & m) const
{
  const bool invalid_raw = m.localization_status_received &&
    (m.localization_status_age_sec <= params_.status_timeout_sec) && !m.localization_is_valid;
  const bool stale_raw = m.fused_odometry_age_sec > params_.fused_timeout_sec;
  return invalid_raw || stale_raw;
}

ViolationEntry FailsafePolicy::evalLocalizationInvalid(const Measurements & m, double now_sec)
{
  const size_t idx = static_cast<size_t>(ViolationCode::kLocalizationInvalid);
  ViolationEntry entry;
  entry.threshold = params_.status_timeout_sec;
  entry.measured = m.localization_status_age_sec;

  const bool raw = m.localization_status_received &&
    (m.localization_status_age_sec <= params_.status_timeout_sec) && !m.localization_is_valid;
  rising_[idx].update(raw, now_sec);
  const bool violating = raw && rising_[idx].durationSec(now_sec) >= params_.localization_invalid_grace_sec;
  entry.status = violating ? ViolationStatus::kViolating : ViolationStatus::kOk;
  entry.detail = violating ? "LocalizationStatus.is_valid is false" : "";
  return entry;
}

ViolationEntry FailsafePolicy::evalLocalizationStale(const Measurements & m, double now_sec)
{
  const size_t idx = static_cast<size_t>(ViolationCode::kLocalizationStale);
  ViolationEntry entry;
  entry.threshold = params_.fused_timeout_sec;
  entry.measured = m.fused_odometry_age_sec;

  const bool raw = m.fused_odometry_age_sec > params_.fused_timeout_sec;
  rising_[idx].update(raw, now_sec);
  const bool violating = raw && rising_[idx].durationSec(now_sec) >= params_.localization_stale_grace_sec;
  entry.status = violating ? ViolationStatus::kViolating : ViolationStatus::kOk;
  entry.detail = violating ? "odometry_fused older than fused_timeout_sec" : "";
  return entry;
}

ViolationEntry FailsafePolicy::evalLocalizationJump(const Measurements & m, double now_sec)
{
  const size_t idx = static_cast<size_t>(ViolationCode::kLocalizationJump);
  ViolationEntry entry;
  entry.threshold = 0.0;

  if (!std::isfinite(m.localization_jump_count)) {
    entry.status = ViolationStatus::kCannotMeasure;
    entry.detail = "jumps diagnostic unavailable";
    return entry;
  }
  entry.measured = m.localization_jump_count;

  const bool raw = m.localization_jump_count > 0.0;
  rising_[idx].update(raw, now_sec);
  const bool violating = raw && rising_[idx].durationSec(now_sec) >= params_.localization_jump_grace_sec;
  entry.status = violating ? ViolationStatus::kViolating : ViolationStatus::kOk;
  entry.detail = violating ? "fused pose jumped" : "";
  return entry;
}

ViolationEntry FailsafePolicy::evalBlindCommand(const Measurements & m, double now_sec)
{
  const size_t idx = static_cast<size_t>(ViolationCode::kBlindCommand);
  ViolationEntry entry;
  entry.threshold = params_.frozen_setpoint_epsilon_copy_m;

  // Localization's own continuity is always computable (ages default to
  // +inf, never NaN) -- only the caller-supplied window displacement can be
  // genuinely unmeasurable, so the tracker still advances even on a
  // CANNOT_MEASURE tick below (R27-2 applies to the ACTION, not this timer).
  const bool bad_localization = rawLocalizationBad(m);
  rising_[idx].update(bad_localization, now_sec);
  const double bad_duration = rising_[idx].durationSec(now_sec);

  const bool displacement_measurable =
    m.command_setpoint_window_full && std::isfinite(m.command_setpoint_displacement_m);
  if (!displacement_measurable) {
    entry.status = ViolationStatus::kCannotMeasure;
    entry.detail = "setpoint displacement window not yet measurable";
    return entry;
  }
  entry.measured = m.command_setpoint_displacement_m;

  const bool moved = m.command_setpoint_displacement_m > params_.frozen_setpoint_epsilon_copy_m;
  const bool blind_source =
    m.command_selected_source == kSourceMission || m.command_selected_source == kSourceTest;
  const bool triggered = bad_duration >= params_.blind_grace_sec && moved && blind_source;

  if (triggered) {
    entry.status = ViolationStatus::kViolating;
    entry.detail = "blind command: pose bad past grace, setpoint moving, MISSION/TEST source";
  } else if (bad_localization) {
    // S:0b R1: a frozen hold on bad localization is the DESIGNED recovery
    // path (Recover TYPE_HOLD) -- BLIND_COMMAND must not cut it, only
    // LOCALIZATION_INVALID/STALE report the underlying cause.
    entry.status = ViolationStatus::kOk;
    entry.detail = "pose bad but setpoint stationary -- valid frozen hold, not cut";
  } else {
    entry.status = ViolationStatus::kOk;
  }
  return entry;
}

ViolationEntry FailsafePolicy::evalBatteryWarn(const Measurements & m, double now_sec)
{
  const size_t idx = static_cast<size_t>(ViolationCode::kBatteryWarn);
  ViolationEntry entry;
  entry.threshold = params_.battery_warn_threshold;

  if (!std::isfinite(m.battery_remaining)) {
    entry.status = ViolationStatus::kCannotMeasure;
    entry.detail = "battery_remaining not finite";
    return entry;
  }
  entry.measured = m.battery_remaining;

  const bool raw = m.battery_remaining < params_.battery_warn_threshold;
  rising_[idx].update(raw, now_sec);
  const bool violating = raw && rising_[idx].durationSec(now_sec) >= params_.battery_warn_grace_sec;
  entry.status = violating ? ViolationStatus::kViolating : ViolationStatus::kOk;
  entry.detail = violating ? "battery_remaining below warn threshold" : "";
  return entry;
}

ViolationEntry FailsafePolicy::evalBatteryCritical(const Measurements & m, double now_sec)
{
  const size_t idx = static_cast<size_t>(ViolationCode::kBatteryCritical);
  ViolationEntry entry;
  entry.threshold = params_.battery_critical_threshold;

  if (!std::isfinite(m.battery_remaining)) {
    entry.status = ViolationStatus::kCannotMeasure;
    entry.detail = "battery_remaining not finite";
    return entry;
  }
  entry.measured = m.battery_remaining;

  const bool raw = m.battery_remaining < params_.battery_critical_threshold;
  rising_[idx].update(raw, now_sec);
  const bool violating = raw && rising_[idx].durationSec(now_sec) >= params_.battery_critical_grace_sec;
  entry.status = violating ? ViolationStatus::kViolating : ViolationStatus::kOk;
  entry.detail = violating ? "battery_remaining below critical threshold" : "";
  return entry;
}

ViolationEntry FailsafePolicy::evalBatteryPx4NoAction(const Measurements & m, double now_sec)
{
  const size_t idx = static_cast<size_t>(ViolationCode::kBatteryPx4NoAction);
  ViolationEntry entry;
  entry.threshold = params_.px4_action_grace_sec;

  if (!std::isfinite(m.battery_remaining) || !m.vehicle_state_received) {
    entry.status = ViolationStatus::kCannotMeasure;
    entry.detail = "battery_remaining or vehicle_state not measurable";
    return entry;
  }
  entry.measured = m.battery_remaining;

  // B3 (real PX4 params read via MAVLink 2026-08-21): gate on PX4's OWN
  // critical threshold (bat_crit_thr_copy=0.07), not this package's more
  // conservative battery_critical_threshold (0.10) -- PX4 has had no real
  // chance to act while battery sits between the two, that gap is not a
  // PX4 failure. See FailsafePolicyParams' comment for the SITL caveat.
  const bool critical = m.battery_remaining < params_.bat_crit_thr_copy;
  rising_[idx].update(critical, now_sec);
  const double critical_duration = rising_[idx].durationSec(now_sec);

  const bool px4_acting =
    m.failsafe_active || m.flight_mode == kFlightModeReturn || m.flight_mode == kFlightModeLand;
  const bool triggered = critical_duration >= params_.px4_action_grace_sec && !px4_acting;

  entry.status = triggered ? ViolationStatus::kViolating : ViolationStatus::kOk;
  entry.detail = triggered ? "battery critical past grace, PX4 has taken no action" : "";
  return entry;
}

ViolationEntry FailsafePolicy::evalObstacleTooClose(const Measurements & m, double now_sec)
{
  const size_t idx = static_cast<size_t>(ViolationCode::kObstacleTooClose);
  ViolationEntry entry;
  entry.threshold = params_.min_obstacle_distance_copy_m;

  // R27-2: NaN means "cannot measure"; +inf is a genuinely MEASURED value
  // (buildMeasurements() -- an empty obstacle map is evidence of "nothing
  // detected", not a missing reading). isnan(), not !isfinite(), is the
  // correct gate here -- the only field in this library where the two
  // diverge (P8.5 bug fix: the old !isfinite() guard silently swallowed the
  // common "no obstacles nearby" case into CANNOT_MEASURE, which then made
  // clearFault() reject OBSTACLE_TOO_CLOSE forever whenever the map was
  // truly empty).
  if (std::isnan(m.obstacle_min_distance_m)) {
    entry.status = ViolationStatus::kCannotMeasure;
    entry.detail = "obstacle_min_distance_m unavailable";
    return entry;
  }
  entry.measured = m.obstacle_min_distance_m;

  const bool map_fresh = m.obstacle_map_age_sec <= params_.obstacle_map_timeout_copy_sec;
  const bool too_close = m.obstacle_min_distance_m < params_.min_obstacle_distance_copy_m;
  const bool raw = too_close && map_fresh && m.localization_good_for_obstacles;
  rising_[idx].update(raw, now_sec);
  const bool violating = raw && rising_[idx].durationSec(now_sec) >= params_.obstacle_grace_sec;

  entry.status = violating ? ViolationStatus::kViolating : ViolationStatus::kOk;
  if (violating) {
    entry.detail = "obstacle inside min distance, map fresh, localization trusted";
  } else if (too_close && !map_fresh) {
    entry.detail = "distance under threshold but obstacle map stale -- not trusted";
  } else if (too_close && !m.localization_good_for_obstacles) {
    entry.detail = "distance under threshold but localization not trusted for registration";
  }
  return entry;
}

ViolationEntry FailsafePolicy::evalOffboardUnhealthy(const Measurements & m, double now_sec)
{
  const size_t idx = static_cast<size_t>(ViolationCode::kOffboardUnhealthy);
  ViolationEntry entry;
  entry.threshold = params_.offboard_min_rate_hz;

  if (!m.offboard_status_received || !std::isfinite(m.offboard_setpoint_rate_hz)) {
    entry.status = ViolationStatus::kCannotMeasure;
    entry.detail = "OffboardStatus not measurable";
    return entry;
  }
  entry.measured = m.offboard_setpoint_rate_hz;

  const bool state_or_rate_bad = m.offboard_state == kOffboardStateFault ||
    m.offboard_setpoint_rate_hz < params_.offboard_min_rate_hz;
  // P8.4b (coordinator decision, real-flight finding): command_selected's
  // own raw age is a second, more direct signal -- unaffected by the
  // arbiter's release-dwell, the gateway's own command_timeout_sec, or
  // offboard_status's own rate-window smoothing, all of which stack into the
  // ~2.66 s end-to-end delay the ORIGINAL detection measured in real flight
  // (1.07 s slower than PX4's own failsafe). Still gated on offboard_status_
  // received above -- see header comment for why that gate stays.
  const bool selected_stale = m.command_selected_age_sec > params_.selected_stale_sec;
  const bool raw = state_or_rate_bad || selected_stale;
  rising_[idx].update(raw, now_sec);
  const bool violating = raw && rising_[idx].durationSec(now_sec) >= params_.offboard_grace_sec;
  entry.status = violating ? ViolationStatus::kViolating : ViolationStatus::kOk;
  if (violating) {
    if (state_or_rate_bad && selected_stale) {
      entry.detail = "offboard STATE_FAULT/rate low AND command_selected stale";
    } else if (selected_stale) {
      entry.detail = "command_selected stale past selected_stale_sec (P8.4b faster path)";
    } else {
      entry.detail = "offboard STATE_FAULT or setpoint rate below floor";
    }
  }
  return entry;
}

ViolationEntry FailsafePolicy::evalEstimatorInputInvalid(const Measurements & m, double now_sec)
{
  const size_t idx = static_cast<size_t>(ViolationCode::kEstimatorInputInvalid);
  ViolationEntry entry;
  entry.threshold = 1.0;   // "all four estimator inputs OK" -- 1.0=healthy, 0.0=degraded

  // estimator_flags_trustworthy is false both when VehicleHealth was never/
  // not-freshly received AND when overall_level==LEVEL_UNKNOWN (backend has
  // no real failsafe_flags yet) -- imu_ok/gps_ok/barometer_ok/magnetometer_ok
  // are plain bools that default fabricated-true, never to be trusted alone.
  if (!m.estimator_flags_trustworthy) {
    entry.status = ViolationStatus::kCannotMeasure;
    entry.detail = "VehicleHealth not received or overall_level==UNKNOWN (no real failsafe_flags yet)";
    return entry;
  }
  const bool raw = !(m.imu_ok && m.gps_ok && m.barometer_ok && m.magnetometer_ok);
  entry.measured = raw ? 0.0 : 1.0;
  rising_[idx].update(raw, now_sec);
  const bool violating = raw && rising_[idx].durationSec(now_sec) >= params_.estimator_grace_sec;
  entry.status = violating ? ViolationStatus::kViolating : ViolationStatus::kOk;
  entry.detail = violating ? "one or more estimator inputs report not-ok" : "";
  return entry;
}

ViolationEntry FailsafePolicy::evalCameraStreamUnhealthy(const Measurements & m, double now_sec)
{
  const size_t idx = static_cast<size_t>(ViolationCode::kCameraStreamUnhealthy);
  ViolationEntry entry;
  entry.threshold = 1.0;

  if (!m.camera_health_received) {
    entry.status = ViolationStatus::kCannotMeasure;
    entry.detail = "camera health not received";
    return entry;
  }
  const bool raw = !m.camera_stream_ok;
  entry.measured = raw ? 0.0 : 1.0;
  rising_[idx].update(raw, now_sec);
  const bool violating = raw && rising_[idx].durationSec(now_sec) >= params_.camera_grace_sec;
  entry.status = violating ? ViolationStatus::kViolating : ViolationStatus::kOk;
  entry.detail = violating ? "camera stream reported unhealthy" : "";
  return entry;
}

ViolationEntry FailsafePolicy::evalFrameMismatch(const Measurements & m, double now_sec)
{
  const size_t idx = static_cast<size_t>(ViolationCode::kFrameMismatch);
  ViolationEntry entry;
  entry.threshold = params_.frame_mismatch_grace_sec;
  entry.measured = m.active_channel_dropping_wrong_frame ? 1.0 : 0.0;

  // Two independent triggers (S:3): a sustained run of wrong-frame drops on
  // the channel currently holding authority (graced, Y15), OR fused itself
  // reporting the wrong frame_id (instantaneous -- a static string is either
  // right or wrong, no reason to wait on it).
  rising_[idx].update(m.active_channel_dropping_wrong_frame, now_sec);
  const bool drops_sustained =
    m.active_channel_dropping_wrong_frame &&
    rising_[idx].durationSec(now_sec) >= params_.frame_mismatch_grace_sec;

  const bool triggered = drops_sustained || m.fused_frame_mismatch;
  entry.status = triggered ? ViolationStatus::kViolating : ViolationStatus::kOk;
  if (triggered) {
    entry.detail = m.fused_frame_mismatch ?
      "odometry_fused reports the wrong frame_id" :
      "held channel dropping wrong-frame commands past grace";
  }
  return entry;
}

ViolationEntry FailsafePolicy::evaluateOne(ViolationCode code, const Measurements & m, double now_sec)
{
  switch (code) {
    case ViolationCode::kLocalizationInvalid: return evalLocalizationInvalid(m, now_sec);
    case ViolationCode::kLocalizationStale: return evalLocalizationStale(m, now_sec);
    case ViolationCode::kLocalizationJump: return evalLocalizationJump(m, now_sec);
    case ViolationCode::kBlindCommand: return evalBlindCommand(m, now_sec);
    case ViolationCode::kBatteryWarn: return evalBatteryWarn(m, now_sec);
    case ViolationCode::kBatteryCritical: return evalBatteryCritical(m, now_sec);
    case ViolationCode::kBatteryPx4NoAction: return evalBatteryPx4NoAction(m, now_sec);
    case ViolationCode::kObstacleTooClose: return evalObstacleTooClose(m, now_sec);
    case ViolationCode::kOffboardUnhealthy: return evalOffboardUnhealthy(m, now_sec);
    case ViolationCode::kEstimatorInputInvalid: return evalEstimatorInputInvalid(m, now_sec);
    case ViolationCode::kCameraStreamUnhealthy: return evalCameraStreamUnhealthy(m, now_sec);
    case ViolationCode::kFrameMismatch: return evalFrameMismatch(m, now_sec);
    default: return ViolationEntry{};
  }
}

void FailsafePolicy::applyFsm(FailsafeAction target_action)
{
  switch (state_) {
    case SupervisorState::kNormal:
      if (target_action == FailsafeAction::kInhibit) {
        state_ = SupervisorState::kEngagingInhibit;
      } else if (target_action == FailsafeAction::kHold) {
        state_ = SupervisorState::kEngagingHold;
      }
      break;
    case SupervisorState::kEngagingHold:
      if (target_action == FailsafeAction::kInhibit) {
        state_ = SupervisorState::kEngagingInhibit;
      }
      break;
    case SupervisorState::kHolding:
      if (target_action == FailsafeAction::kInhibit) {
        // One-way escalation (S:4.3); never auto-downgrades back to NORMAL
        // even if target_action drops to kNone -- only clearFault() may.
        state_ = SupervisorState::kEngagingInhibit;
      }
      break;
    case SupervisorState::kEngagingInhibit:
    case SupervisorState::kInhibited:
    case SupervisorState::kClearing:
      // Already at/entering the ceiling, or mid-ClearFault: nothing to do.
      break;
  }
}

EvaluationResult FailsafePolicy::evaluate(const Measurements & measurements, double now_sec)
{
  EvaluationResult result;
  uint8_t overall_level = kLevelOk;

  // B5 (plan S:2): armed && OFFBOARD && command_fresh gates the 4 LATCHING
  // codes ONLY -- the 8 REPORT-only codes stay ungated (always visible,
  // never seize control anyway). Fail-OPEN on armed/flight_mode: a stale/
  // unmeasurable vehicle_state must NEVER be read as "confirmed grounded"
  // and used as an excuse to suppress protection -- only a POSITIVELY
  // CONFIRMED disarmed-or-non-OFFBOARD state may gate a latch off.
  // command_fresh itself needs no such fail-open: its age never produces
  // NaN (R27-2 convention), "never received" is a genuine not-fresh reading.
  const bool confirmed_not_flying = measurements.vehicle_state_received &&
    (!measurements.armed || measurements.flight_mode != kFlightModeOffboard);
  const bool action_gate = !confirmed_not_flying && measurements.command_fresh;

  for (size_t i = 0; i < kViolationCodeCount; ++i) {
    const ViolationCode code = static_cast<ViolationCode>(i);
    ViolationEntry entry = evaluateOne(code, measurements, now_sec);

    if (isLatchingCode(code)) {
      if (entry.status == ViolationStatus::kViolating && action_gate) {
        latched_[i] = true;
      }
      if (latched_[i]) {
        // Sticky: keeps reporting while latched regardless of this tick's
        // raw re-check -- only clearFault() ever un-latches (S:0 "kèm latch
        // tới ClearFault").
        entry.status = ViolationStatus::kViolating;
        if (entry.detail.empty()) {
          entry.detail = "latched, awaiting ClearFault";
        }
      }
    }

    if (entry.status == ViolationStatus::kViolating) {
      overall_level = std::max(overall_level, violationCodeSeverity(code));
    }
    result.violations[i] = entry;
  }

  FailsafeAction target_action = FailsafeAction::kNone;
  for (size_t i = 0; i < kViolationCodeCount; ++i) {
    if (!latched_[i]) {
      continue;
    }
    const FailsafeAction action = violationCodeAction(static_cast<ViolationCode>(i));
    if (action == FailsafeAction::kInhibit) {
      target_action = FailsafeAction::kInhibit;
    } else if (action == FailsafeAction::kHold && target_action != FailsafeAction::kInhibit) {
      target_action = FailsafeAction::kHold;
    }
  }

  // P8.5 (S:4): HOLDING must never keep streaming an untrustworthy frozen
  // pose -- escalate one-way to INHIBIT the instant fused pose goes stale,
  // even with no INHIBIT-class code latched (no grace, same "khong du => di
  // thang INHIBIT" rule confirmHoldEngaged() applies at entry). The ORIGINAL
  // HOLD-class latch is untouched by this -- only the ceiling moves.
  if (state_ == SupervisorState::kHolding &&
    (!std::isfinite(measurements.fused_odometry_age_sec) ||
      measurements.fused_odometry_age_sec > params_.pose_max_age_sec))
  {
    target_action = FailsafeAction::kInhibit;
  }

  applyFsm(target_action);

  result.overall_level = overall_level;
  result.target_action = target_action;
  result.state = state_;
  return result;
}

bool FailsafePolicy::confirmHoldEngaged(double pose_age_sec, double now_sec)
{
  (void)now_sec;
  if (state_ != SupervisorState::kEngagingHold) {
    return false;
  }
  if (!std::isfinite(pose_age_sec) || pose_age_sec > params_.pose_max_age_sec) {
    // S:4: an untrustworthy pose must never be frozen -- escalate instead.
    state_ = SupervisorState::kEngagingInhibit;
  } else {
    state_ = SupervisorState::kHolding;
  }
  return true;
}

bool FailsafePolicy::confirmInhibitEngaged()
{
  if (state_ != SupervisorState::kEngagingInhibit) {
    return false;
  }
  state_ = SupervisorState::kInhibited;
  return true;
}

ClearFaultResult FailsafePolicy::clearFault(
  const std::string & fault_code, const Measurements & measurements, double now_sec)
{
  ClearFaultResult result;

  // Gate on the LATCH SET, not the FSM ceiling: a code can be latched while
  // state_ still reads ENGAGING_HOLD/ENGAGING_INHIBIT (confirm not called
  // yet) -- ClearFault must still see it, or a caller that clears mid-engage
  // would get a false "nothing to clear".
  const bool anything_latched = std::any_of(latched_.begin(), latched_.end(), [](bool v) {return v;});
  if (!anything_latched) {
    result.success = true;
    result.result_code = kResultSucceeded;
    result.message = "no fault latched";
    return result;
  }

  // S:4.3: INHIBITED -> HOLDING is forbidden outright, so the ceiling this
  // call started at is the ONLY thing a partial/rejected clear may revert
  // to -- never recomputed from which class of code happens to remain
  // latched, which is exactly the trap that would let a stray HOLD-class
  // survivor drag an INHIBITED supervisor down to HOLDING.
  const SupervisorState ceiling_before = state_;
  state_ = SupervisorState::kClearing;

  std::vector<ViolationCode> targets;
  for (size_t i = 0; i < kViolationCodeCount; ++i) {
    if (!latched_[i]) {
      continue;
    }
    const ViolationCode code = static_cast<ViolationCode>(i);
    if (fault_code.empty() || fault_code == violationCodeName(code)) {
      targets.push_back(code);
    }
  }

  if (targets.empty()) {
    // fault_code named something not currently latched -- nothing blocks
    // it, but nothing to clear either; the ceiling is untouched.
    state_ = ceiling_before;
    result.success = true;
    result.result_code = kResultSucceeded;
    result.message = "no such fault currently latched";
    return result;
  }

  // (1)+(2) per target, in one pass so a code that fails the cannot-measure
  // gate never skips updating a SIBLING target's clear-stability tracker --
  // each code's clearing_ clock must keep advancing on every clearFault()
  // call it is eligible for, independent of any other target's outcome.
  // clear_stability_sec is measured across CALLS to this method (ClearFault
  // is a service; a caller wanting timely confirmation polls it, same
  // pattern as any ROS2 service-based retry loop) -- not via evaluate().
  bool any_rejected = false;
  for (const ViolationCode code : targets) {
    const size_t idx = static_cast<size_t>(code);
    // N1 (real bug, cheap fix): evaluateOne() mutates rising_[idx] as a side
    // effect of its own grace-timing -- a clearFault() PROBE (called from a
    // service, at arbitrary times off the 20 Hz evaluate() cadence) must
    // never perturb the SAME tracker evaluate() relies on. This probe only
    // ever reads probe.status; snapshot/restore keeps it read-only in effect.
    const ContinuityTracker rising_snapshot = rising_[idx];
    const ViolationEntry probe = evaluateOne(code, measurements, now_sec);
    rising_[idx] = rising_snapshot;
    if (probe.status == ViolationStatus::kCannotMeasure) {
      any_rejected = true;
      result.remaining_faults.push_back(
        std::string(violationCodeName(code)) + ": cannot measure cause, refusing to clear");
      continue;
    }

    bool clear_now = false;
    double measured = 0.0;
    double threshold = 0.0;

    switch (code) {
      case ViolationCode::kBlindCommand: {
        const double margin = params_.localization_clear_margin_sec;
        const bool status_ok = measurements.localization_status_received &&
          measurements.localization_status_age_sec <= (params_.status_timeout_sec - margin) &&
          measurements.localization_is_valid;
        const bool fused_ok =
          measurements.fused_odometry_age_sec <= (params_.fused_timeout_sec - margin);
        clear_now = status_ok && fused_ok;
        measured = measurements.fused_odometry_age_sec;
        threshold = params_.fused_timeout_sec - margin;
        break;
      }
      case ViolationCode::kBatteryPx4NoAction: {
        if (std::isfinite(measurements.battery_remaining)) {
          clear_now = measurements.battery_remaining >=
            (params_.battery_critical_threshold + params_.battery_clear_margin);
        }
        measured = measurements.battery_remaining;
        threshold = params_.battery_critical_threshold + params_.battery_clear_margin;
        break;
      }
      case ViolationCode::kObstacleTooClose: {
        // Same R27-2 fix as evalObstacleTooClose(): +inf (empty map) must
        // clear, not be silently blocked -- isnan(), not !isfinite().
        if (!std::isnan(measurements.obstacle_min_distance_m)) {
          clear_now = measurements.obstacle_min_distance_m >=
            (params_.min_obstacle_distance_copy_m + params_.obstacle_clear_margin_m);
        }
        measured = measurements.obstacle_min_distance_m;
        threshold = params_.min_obstacle_distance_copy_m + params_.obstacle_clear_margin_m;
        break;
      }
      case ViolationCode::kFrameMismatch: {
        clear_now =
          !measurements.active_channel_dropping_wrong_frame && !measurements.fused_frame_mismatch;
        measured = clear_now ? 0.0 : 1.0;
        threshold = 0.0;
        break;
      }
      default:
        clear_now = true;
        break;
    }

    clearing_[idx].update(clear_now, now_sec);
    const double stable_for = clearing_[idx].durationSec(now_sec);
    if (!clear_now || stable_for < params_.clear_stability_sec) {
      any_rejected = true;
      result.remaining_faults.push_back(
        std::string(violationCodeName(code)) + ": cause not clear (measured=" +
        std::to_string(measured) + ", threshold=" + std::to_string(threshold) +
        ", stable_for=" + std::to_string(std::max(stable_for, 0.0)) + "s of " +
        std::to_string(params_.clear_stability_sec) + "s required)");
    }
  }

  // (3) Handover-jump gate: only relevant when clearing would hand control
  // back to a live MISSION -- checked last, and it blocks the WHOLE attempt
  // even if every individual cause above is otherwise resolved.
  if (!any_rejected && measurements.cmd_mission_live) {
    const double jump = measurements.cmd_mission_setpoint_distance_from_fused_m;
    if (!std::isfinite(jump) || jump > params_.handover_jump_limit_m) {
      any_rejected = true;
      result.remaining_faults.push_back(
        "HANDOVER_JUMP: cmd_mission setpoint too far from fused pose to resume safely");
    }
  }

  if (any_rejected) {
    // S:4.3: revert to the SAME ceiling this call started at -- never
    // recomputed from the surviving latch set (see comment above).
    state_ = ceiling_before;
    result.success = false;
    result.result_code = kResultUnknown;
    result.message = "one or more faults could not be cleared";
    return result;
  }

  for (const ViolationCode code : targets) {
    latched_[static_cast<size_t>(code)] = false;
  }

  const bool anything_still_latched = std::any_of(
    latched_.begin(), latched_.end(), [](bool v) {return v;});
  // Same rule: a PARTIAL clear (something still latched) never steps the
  // ceiling down, even from INHIBITED to HOLDING -- only a FULL clear ever
  // reaches NORMAL (S:4.3, "chi ClearFault -> NORMAL", not "-> a lower ceiling").
  state_ = anything_still_latched ? ceiling_before : SupervisorState::kNormal;

  result.success = true;
  result.result_code = kResultSucceeded;
  result.message = anything_still_latched ? "requested fault(s) cleared, others remain" : "all faults cleared";
  return result;
}

}  // namespace uav_safety
