#ifndef UAV_SAFETY__FAILSAFE_POLICY_HPP_
#define UAV_SAFETY__FAILSAFE_POLICY_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace uav_safety
{

// Mirrored message constants (ROS-free, same technique as
// uav_control_authority's authority_arbiter.hpp -- this library never
// includes a generated message header, only the numeric contract).

// Mirrors uav_interfaces/msg/ControlAuthority.msg SOURCE_*.
constexpr uint8_t kSourceNone = 0;
constexpr uint8_t kSourceTest = 1;
constexpr uint8_t kSourceMission = 2;
constexpr uint8_t kSourceOperator = 3;
constexpr uint8_t kSourceSafety = 4;

// Mirrors uav_interfaces/msg/VehicleState.msg FLIGHT_MODE_* -- the values
// BATTERY_PX4_NO_ACTION's "PX4 already acting" check and B5's action gate read.
constexpr uint8_t kFlightModeUnknown = 0;
constexpr uint8_t kFlightModeOffboard = 4;
constexpr uint8_t kFlightModeReturn = 6;
constexpr uint8_t kFlightModeLand = 8;

// Mirrors uav_interfaces/msg/OffboardStatus.msg STATE_*.
constexpr uint8_t kOffboardStateIdle = 0;
constexpr uint8_t kOffboardStateStreaming = 1;
constexpr uint8_t kOffboardStateEngaging = 2;
constexpr uint8_t kOffboardStateActive = 3;
constexpr uint8_t kOffboardStateFault = 4;

// Mirrors uav_interfaces/msg/SafetyState.msg LEVEL_*.
constexpr uint8_t kLevelUnknown = 0;
constexpr uint8_t kLevelOk = 1;
constexpr uint8_t kLevelWarning = 2;
constexpr uint8_t kLevelError = 3;
constexpr uint8_t kLevelEmergency = 4;

// Mirrors uav_interfaces/msg/ResultCode.msg -- only the two values
// ClearFaultResult ever reports (this is a status query, never an action).
constexpr uint8_t kResultUnknown = 0;
constexpr uint8_t kResultSucceeded = 1;

// The 12 violation codes of plan P8-safety.md S:3. PLAN_FAILED is
// deliberately NOT here: S:1.6 keeps safety from deciding anything off
// Trajectory3D, so it never becomes a graded/latched violation in this
// library -- it is diagnostics wiring done entirely at the node layer.

enum class ViolationCode : uint8_t
{
  kLocalizationInvalid = 0,
  kLocalizationStale,
  kLocalizationJump,
  kBlindCommand,
  kBatteryWarn,
  kBatteryCritical,
  kBatteryPx4NoAction,
  kObstacleTooClose,
  kOffboardUnhealthy,
  kEstimatorInputInvalid,
  kCameraStreamUnhealthy,
  kFrameMismatch,
  kCount   // sentinel -- array size, never a real code
};

constexpr size_t kViolationCodeCount = static_cast<size_t>(ViolationCode::kCount);

/// String form for SafetyState.violation_codes -- matches S:3's names exactly.
const char * violationCodeName(ViolationCode code);

/// Only 4 codes ever latch (BLIND_COMMAND, BATTERY_PX4_NO_ACTION,
/// OBSTACLE_TOO_CLOSE, FRAME_MISMATCH); the other 8 are REPORT-only and
/// never touch the FSM (S:3 action column).
enum class FailsafeAction : uint8_t
{
  kNone,
  kReport,
  kHold,
  kInhibit,
};

FailsafeAction violationCodeAction(ViolationCode code);

/// SafetyState.level this code contributes when VIOLATING. S:3 states the
/// level explicitly for 5 codes (INVALID/STALE/CRITICAL=EMERGENCY,
/// JUMP/OFFBOARD_UNHEALTHY=ERROR); the rest (BATTERY_WARN, the two HOLD/
/// INHIBIT codes without an explicit level, ESTIMATOR_INPUT_INVALID,
/// CAMERA_STREAM_UNHEALTHY) are this library's own reasonable choice,
/// flagged in the P8.1 handoff report for reviewer sign-off.
uint8_t violationCodeSeverity(ViolationCode code);

// FSM (S:4.3): NORMAL -> ENGAGING_HOLD -> HOLDING,
// NORMAL -> ENGAGING_INHIBIT -> INHIBITED, HOLDING -> INHIBITED one-way,
// INHIBITED -> HOLDING forbidden, only ClearFault ever returns to NORMAL.

enum class SupervisorState : uint8_t
{
  kNormal,
  kEngagingHold,
  kHolding,
  kEngagingInhibit,
  kInhibited,
  kClearing,
};

// Parameters. Every "*_copy" field is a documented COPY of a threshold that
// truly lives in another package's parameter server (same reason
// uav_control_authority carries downstream_command_timeout_sec) -- this
// library cannot see the real value, only compare against a hand-kept
// mirror. validate() enforces the S:9d pairing rules; it never fixes a bad
// value, it refuses to start (same discipline as AuthorityArbiter).

struct FailsafePolicyParams
{
  // --- Localization (S:3 rows 1-3) ---
  double status_timeout_sec = 1.0;          // LocalizationStatus freshness
  double fused_timeout_sec = 0.5;           // odometry_fused freshness
  double localization_invalid_grace_sec = 0.0;
  double localization_stale_grace_sec = 0.0;
  double localization_jump_grace_sec = 0.0;

  // --- BLIND_COMMAND (S:0b R1, S:3 row 4) ---
  double blind_grace_sec = 1.5;
  double frozen_setpoint_epsilon_copy_m = 0.05;  // kFrozenSetpointDrift copy, navigator :2852

  // --- Battery (S:3 rows 5-6) ---
  double battery_warn_threshold = 0.20;
  double battery_critical_threshold = 0.10;
  double battery_warn_grace_sec = 0.0;
  double battery_critical_grace_sec = 0.0;

  // --- BATTERY_PX4_NO_ACTION (S:3 row 7) ---
  double px4_action_grace_sec = 5.0;

  // --- OBSTACLE_TOO_CLOSE (S:3 row 8) ---
  // Y2: copy of uav_navigation's local_planner costmap.drone_radius_m --
  // this package cannot see that parameter directly (same limitation as
  // frozen_setpoint_epsilon_copy_m above); no cross-package validate is
  // possible, manual checklist item only.
  double min_obstacle_distance_copy_m = 0.35;
  double obstacle_grace_sec = 0.5;
  double obstacle_clear_margin_m = 0.10;    // hysteresis, ClearFault only
  // Y2: copy of world_model_node's obstacle map publish rate (CLAUDE.md
  // S:4.4) -- validate() requires obstacle_map_timeout_copy_sec >=
  // 2/world_model_publish_hz_copy (same B1-class discipline: a freshness
  // grace must outlive >= 2 sampling periods of the signal it grades, or a
  // single stale-but-unchanged reading between cycles can pass as "fresh").
  double world_model_publish_hz_copy = 10.0;
  double obstacle_map_timeout_copy_sec = 0.3;

  // --- OFFBOARD_UNHEALTHY (S:3 row 9) ---
  double offboard_min_rate_hz = 5.0;
  double offboard_grace_sec = 0.5;
  // P8.4b (coordinator decision, real-flight finding): the original STATE_
  // FAULT/rate detection sits behind the arbiter's release-dwell + gateway's
  // own command_timeout_sec + offboard_status's own rate-window smoothing --
  // measured end-to-end at +2.66 s after a real stream loss, 1.07 s SLOWER
  // than PX4's own failsafe (contradicts S:8's original "sooner than PX4"
  // expectation; overviewPlan §P8 already flagged this as a known risk).
  // command_selected_age_sec is a second, more direct signal: it is the raw
  // age of the last message on command_selected itself, unaffected by any
  // of those downstream hops. 0.5 s is chosen to sit strictly above the
  // arbiter's OWN validated worst-case legitimate handover silence
  // (release_dwell_sec + 2*monitor_period + 1/stream_hz <= 0.4 s by the
  // arbiter's own Y1 pairing rule) -- below that, a normal MISSION<->
  // OPERATOR handover would falsely read as unhealthy. Still REPORT-only,
  // never an action change.
  double selected_stale_sec = 0.5;
  // Hand-kept copy of px4_command_gateway_node's command_timeout_sec (also
  // copied by authority_arbiter as downstream_command_timeout_sec) -- this
  // package cannot see the gateway's own parameter. validate() requires
  // selected_stale_sec >= this copy so the new path can never fire while the
  // gateway is still legitimately buffering a stale-but-not-yet-cut command.
  double downstream_command_timeout_copy_sec = 0.5;

  // --- ESTIMATOR_INPUT_INVALID / CAMERA_STREAM_UNHEALTHY (S:3 row 10) ---
  // S:3 groups these as "as blueprint" without a numeric threshold this
  // package has access to; both stay REPORT-only (never latch), so an
  // immediate (0 s) debounce is the conservative choice -- flagged in the
  // P8.1 handoff report, not invented safety margins.
  double estimator_grace_sec = 0.0;
  double camera_grace_sec = 0.0;

  // --- B2/Y1 (review 2026-08-21): freshness for 4 subscriptions the node
  // caches indefinitely -- without this, a dead publisher leaves the last
  // GOOD reading frozen forever, read as still-trustworthy. Each is 3x the
  // real publisher's own period (same "3 missed cycles" convention as
  // obstacle_map_timeout_copy_sec above); node buildMeasurements() applies
  // them, gating vehicle_health_received/vehicle_state_received/
  // offboard_status_received/camera_health_received on age, not just
  // "ever arrived" -- every evaluator that already branches on those
  // booleans into CANNOT_MEASURE gets this for free, no evaluator changes.
  double vehicle_health_timeout_sec = 0.3;    // 3 x 10 Hz, px4_state_adapter_node publish_rate_hz
  double vehicle_state_timeout_sec = 0.3;     // 3 x 10 Hz, px4_state_adapter_node publish_rate_hz
  double offboard_status_timeout_sec = 0.6;   // 3 x 200 ms, offboard_session_manager_node kSupervisionInterval
  double camera_health_timeout_sec = 3.0;     // 3 x 1 Hz, camera_health_node report_period_sec default

  // --- B3 (review 2026-08-21): real PX4 battery thresholds, read live via
  // MAVLink param_request_read against PX4 v1.15.4 SITL default params
  // (2026-08-21) -- BAT_LOW_THR/BAT_CRIT_THR/BAT_EMERGEN_THR. PX4's own
  // CRIT (0.07) sits BELOW this package's battery_critical_threshold
  // (0.10): BATTERY_PX4_NO_ACTION must judge "has PX4 had a fair chance to
  // act" against PX4's OWN threshold, not ours -- between 0.10 and 0.07
  // PX4 has no reason to have acted yet, that gap is not PX4's fault.
  // CAVEAT: the "PX4 reacts within ~0.58 s of CRITICAL" finding (G-S3 B2,
  // P8.4/P8.4b) was measured under an ARTIFICIALLY FORCED drain rate in
  // SITL -- it is a lower bound on PX4's reaction latency under that one
  // condition, not a general guarantee for a real, slow, natural discharge.
  double bat_low_thr_copy = 0.15;
  double bat_crit_thr_copy = 0.07;
  double bat_emergen_thr_copy = 0.05;

  // --- FRAME_MISMATCH (S:3 row 11) ---
  // B1 (real bug, review 2026-08-21): the per-channel drop count this code
  // reads only refreshes once per arbiter diagnostics cycle (1 Hz) -- grace
  // must span >= 2 cycles, or a stale-but-unchanged reading between cycles
  // can look like "still rising" (same B1-class discipline as
  // obstacle_map_timeout_copy_sec above). Default bumped 1.0 -> 2.0 s.
  double arbiter_diagnostics_period_copy_sec = 1.0;   // authority_arbiter's diagnostics_every_ticks_ cadence
  double frame_mismatch_grace_sec = 2.0;
  // R32 (Y1/N3, 2026-08-22): every sample-and-hold read from another node's
  // stream must carry an age -- the node expires each held reading once it
  // is older than 2 sampling periods of its source.
  double localization_diag_period_copy_sec = 1.0;   // localization_health_node report_timer_ cadence

  // --- ClearFault (S:3.1) ---
  double clear_stability_sec = 3.0;
  double battery_clear_margin = 0.02;       // hysteresis, ClearFault only
  double localization_clear_margin_sec = 0.1;  // hysteresis, ClearFault only
  double handover_jump_limit_m = 0.80;

  // --- HOLD engagement (S:4) ---
  double pose_max_age_sec = 0.25;
  double hold_stream_hz = 20.0;

  std::string odom_frame = "odom";

  // --- Copies of thresholds this package cannot see directly (S:9d) ---
  double odometry_timeout_copy_sec = 1.0;      // navigator's own pose-staleness gate
  double progress_hz_copy = 10.0;              // navigator's tick rate
  double arbiter_source_timeout_copy_sec = 0.20;   // authority_arbiter's source_timeout_sec
  double max_lead_copy_m = 0.80;               // navigator's max_lead_horizontal_m
  // Y8: copy of localization_mux_node's publish PERIOD (not rate) -- 10 Hz.
  // validate() requires pose_max_age_sec >= 2 x mux_publish_period_copy_sec
  // (HOLD must never freeze a pose fresh enough to look trustworthy after
  // only a single missed mux publish).
  double mux_publish_period_copy_sec = 0.1;    // localization_mux_node's 10 Hz publish period
};

// Raw measurements. Every field is either directly readable off a real
// message/diagnostic or -- for the two geometry comparisons this library
// deliberately never buffers or computes itself (S:1: it forwards
// judgements, not geometry) -- pre-computed by the caller from real wire
// samples: command_setpoint_displacement_m (window W) and
// cmd_mission_setpoint_distance_from_fused_m (ClearFault's handover gate).
//
// "Never received" ages default to +infinity (a genuinely stale reading,
// same convention as AuthorityArbiter::rawArrivalAgeSec) -- content fields
// that could themselves be corrupt (battery_remaining, distances, the
// window displacement) default to NaN, meaning "cannot measure", handled by
// isMeasured() below, never by relying on a NaN comparison alone (R27-2).

struct Measurements
{
  // Localization.
  bool localization_status_received = false;
  double localization_status_age_sec = 0.0;   // meaningful iff received; caller sets +inf otherwise
  bool localization_is_valid = false;         // meaningful iff localization_status_received
  double fused_odometry_age_sec = 0.0;        // caller sets +inf if never received
  double localization_jump_count = 0.0;       // NaN if the "jumps" diagnostic is unavailable

  // Active command_selected setpoint (for BLIND_COMMAND, S:0b R1).
  double command_setpoint_displacement_m = 0.0;   // NaN if not yet measurable
  bool command_setpoint_window_full = false;      // caller has >= W samples
  uint8_t command_selected_source = kSourceNone;

  // Battery / vehicle state.
  double battery_remaining = 0.0;             // NaN if unavailable/corrupt, 0..1 otherwise
  bool vehicle_state_received = false;
  bool failsafe_active = false;               // meaningful iff vehicle_state_received
  uint8_t flight_mode = kFlightModeUnknown;   // meaningful iff vehicle_state_received
  // B5: meaningful iff vehicle_state_received. Fail-OPEN, never fail-closed,
  // on staleness (see evaluate()): a stale vehicle_state must never look
  // like "confirmed disarmed" and suppress a latching code's protection.
  bool armed = false;

  // Obstacle map.
  double obstacle_min_distance_m = 0.0;       // NaN if unavailable
  double obstacle_map_age_sec = 0.0;          // caller sets +inf if never received
  bool localization_good_for_obstacles = false;

  // Offboard link.
  bool offboard_status_received = false;
  uint8_t offboard_state = kOffboardStateIdle;   // meaningful iff offboard_status_received
  double offboard_setpoint_rate_hz = 0.0;        // NaN if unavailable
  // P8.4b: raw age of the last message on command_selected, arrival-time
  // based (never header.stamp). +inf if never received -- a genuinely stale
  // reading, same convention as every other age field here, not "cannot
  // measure" (silence itself IS the measurement for this one).
  double command_selected_age_sec = 0.0;
  // Y4: REAL freshness of command_selected (command_selected_age_sec <=
  // selected_stale_sec) -- feeds both SafetyState.command_fresh (no longer
  // a !BLIND_COMMAND proxy) and B5's action gate. Never NaN (age defaults
  // to +inf, a genuine "not fresh" reading, R27-2 convention).
  bool command_fresh = false;

  // Estimator input health.
  bool vehicle_health_received = false;
  // False when VehicleHealth.overall_level==LEVEL_UNKNOWN (backend has no
  // real failsafe_flags data yet) -- imu_ok/gps_ok/barometer_ok/
  // magnetometer_ok are plain bools that cannot themselves encode
  // "unknown", so this is the ONLY thing ESTIMATOR_INPUT_INVALID may trust
  // to tell a genuine reading apart from the struct's fabricated default-true.
  bool estimator_flags_trustworthy = false;
  bool imu_ok = false;
  bool gps_ok = false;
  bool barometer_ok = false;
  bool magnetometer_ok = false;

  // Camera stream health.
  bool camera_health_received = false;
  bool camera_stream_ok = false;

  // Frame integrity.
  bool active_channel_dropping_wrong_frame = false;   // rising edge, this tick
  bool fused_frame_mismatch = false;

  // ClearFault's handover-jump gate.
  bool cmd_mission_live = false;
  double cmd_mission_setpoint_distance_from_fused_m = 0.0;   // NaN if unavailable
};

enum class ViolationStatus : uint8_t
{
  kOk,
  kViolating,
  kCannotMeasure,
};

struct ViolationEntry
{
  ViolationStatus status = ViolationStatus::kOk;
  double measured = 0.0;
  double threshold = 0.0;
  std::string detail;
};

struct EvaluationResult
{
  std::array<ViolationEntry, kViolationCodeCount> violations;
  uint8_t overall_level = kLevelOk;
  FailsafeAction target_action = FailsafeAction::kNone;
  SupervisorState state = SupervisorState::kNormal;   // state AFTER this evaluate()
};

struct ClearFaultResult
{
  bool success = false;
  uint8_t result_code = kResultUnknown;
  std::string message;
  std::vector<std::string> remaining_faults;
};

/// Pure policy engine: one evaluate() per safety tick, plus the two explicit
/// engagement confirmations and ClearFault. No clock read anywhere -- every
/// "now" is a caller-supplied second count (R21), and every entry point must
/// be called from a single, non-concurrent context (R24, same rule as
/// AuthorityArbiter -- the future node's ONE MutuallyExclusive group is what
/// guarantees that, not this class).
class FailsafePolicy
{
public:
  /// Throws std::invalid_argument if params are unusable -- refuse to start,
  /// same rule as AuthorityArbiter and navigator's rejectUnusableTrajectoryLimits.
  explicit FailsafePolicy(const FailsafePolicyParams & params);

  /// Runs all 12 detectors, advances every grace/clear-stability tracker,
  /// and -- only for the 4 latching codes -- may auto-advance
  /// NORMAL -> ENGAGING_HOLD / ENGAGING_INHIBIT the instant a new latch is
  /// set. Never resolves an ENGAGING_* state itself: see
  /// confirmHoldEngaged()/confirmInhibitEngaged() ("0 hop tre": a caller
  /// that wants zero added latency simply calls the confirm right after,
  /// same tick; a caller doing real async work may wait). Also (P8.5, S:4):
  /// while HOLDING, an independent one-way escalation to ENGAGING_INHIBIT
  /// fires the instant measurements.fused_odometry_age_sec exceeds
  /// pose_max_age_sec -- no grace, no latched code required. HOLDING must
  /// never keep streaming a frozen pose it can no longer trust is still
  /// grounded in a live source.
  EvaluationResult evaluate(const Measurements & measurements, double now_sec);

  /// ENGAGING_HOLD -> HOLDING, unless pose_age_sec exceeds pose_max_age_sec,
  /// in which case it escalates straight to ENGAGING_INHIBIT instead (S:4:
  /// "khong du => di thang INHIBIT" -- HOLD must never freeze a pose it
  /// cannot trust). No-op (returns false) when not in ENGAGING_HOLD.
  bool confirmHoldEngaged(double pose_age_sec, double now_sec);

  /// ENGAGING_INHIBIT -> INHIBITED. No-op (returns false) otherwise.
  bool confirmInhibitEngaged();

  /// fault_code == "" attempts every currently latched code. Order per
  /// S:3.1: (1) any requested code whose cause cannot currently be measured
  /// -> reject; (2) any requested code whose cause is not yet stably clear
  /// for clear_stability_sec -> reject with measured/threshold; (3) only if
  /// (1)+(2) all pass AND clearing would hand control back to a live
  /// MISSION, the handover-jump gate -> reject the WHOLE attempt if it
  /// fails. Only a full accept ever moves the FSM (HOLDING/INHIBITED -> NORMAL
  /// once every latched code is gone); a partial/rejected attempt never
  /// steps INHIBITED down to HOLDING (S:4.3 forbids that transition outright).
  /// clear_stability_sec is measured across CALLS to this method, not via
  /// evaluate() -- a caller wanting timely confirmation polls clearFault()
  /// (same retry pattern as any ROS2 service), each call's now_sec must be
  /// monotonically non-decreasing for the stability clock to mean anything.
  ClearFaultResult clearFault(
    const std::string & fault_code, const Measurements & measurements, double now_sec);

  SupervisorState state() const {return state_;}
  bool isLatched(ViolationCode code) const;

private:
  void validate() const;

  struct ContinuityTracker
  {
    // NaN = condition not currently (continuously) true.
    double true_since_sec = kNotTrue;
    static constexpr double kNotTrue = -1.0;

    void update(bool condition_now, double now_sec);
    /// Duration the condition has held true, ending at now_sec; 0 when not
    /// currently true. A tick where update() is deliberately never called
    /// (measurement unavailable, R27-2) leaves true_since_sec untouched --
    /// the CANNOT_MEASURE tick itself yields no action, but a duration
    /// already accumulating before the gap keeps counting from where it
    /// started once a measurable tick confirms the condition is still true.
    double durationSec(double now_sec) const;
    bool isTrueNow() const {return true_since_sec != kNotTrue;}
  };

  ViolationEntry evaluateOne(
    ViolationCode code, const Measurements & m, double now_sec);

  // One evaluator per code -- keeps S:3's table literally one function per
  // row, so a mutation on a single grace/threshold only ever touches one
  // function and one test group.
  ViolationEntry evalLocalizationInvalid(const Measurements & m, double now_sec);
  ViolationEntry evalLocalizationStale(const Measurements & m, double now_sec);
  ViolationEntry evalLocalizationJump(const Measurements & m, double now_sec);
  ViolationEntry evalBlindCommand(const Measurements & m, double now_sec);
  ViolationEntry evalBatteryWarn(const Measurements & m, double now_sec);
  ViolationEntry evalBatteryCritical(const Measurements & m, double now_sec);
  ViolationEntry evalBatteryPx4NoAction(const Measurements & m, double now_sec);
  ViolationEntry evalObstacleTooClose(const Measurements & m, double now_sec);
  ViolationEntry evalOffboardUnhealthy(const Measurements & m, double now_sec);
  ViolationEntry evalEstimatorInputInvalid(const Measurements & m, double now_sec);
  ViolationEntry evalCameraStreamUnhealthy(const Measurements & m, double now_sec);
  ViolationEntry evalFrameMismatch(const Measurements & m, double now_sec);

  /// The raw (ungraced) boolean each evaluator above tests -- reused by
  /// clearFault() to check "is the cause still alive right now" (S:3.1
  /// rejection 2) without duplicating the condition in two places.
  bool rawLocalizationBad(const Measurements & m) const;

  void applyFsm(FailsafeAction target_action);

  FailsafePolicyParams params_;
  SupervisorState state_ = SupervisorState::kNormal;

  std::array<ContinuityTracker, kViolationCodeCount> rising_;
  // Only the 4 latching codes use these; harmless unused slots otherwise.
  std::array<ContinuityTracker, kViolationCodeCount> clearing_;
  std::array<bool, kViolationCodeCount> latched_{};
};

}  // namespace uav_safety

#endif  // UAV_SAFETY__FAILSAFE_POLICY_HPP_
