#ifndef UAV_MISSION__MISSION_POLICY_HPP_
#define UAV_MISSION__MISSION_POLICY_HPP_

#include <cstdint>
#include <string>

namespace uav_mission
{

// Mirrored message constants (ROS-free, same technique as uav_safety's
// failsafe_policy.hpp -- this library never includes a generated message
// header, only the numeric contract).

// Mirrors uav_interfaces/msg/ControlAuthority.msg SOURCE_*.
constexpr uint8_t kSourceNone = 0;
constexpr uint8_t kSourceTest = 1;
constexpr uint8_t kSourceMission = 2;
constexpr uint8_t kSourceOperator = 3;
constexpr uint8_t kSourceSafety = 4;

// Mirrors uav_interfaces/msg/ResultCode.msg (shared by all 9 nav actions and
// by ExecuteMission/MissionStatus.last_result_code).
constexpr uint8_t kResultUnknown = 0;
constexpr uint8_t kResultSucceeded = 1;
constexpr uint8_t kResultCanceled = 2;
constexpr uint8_t kResultAbortedInvalidGoal = 3;
constexpr uint8_t kResultAbortedNoAuthority = 4;
constexpr uint8_t kResultAbortedSafety = 5;
constexpr uint8_t kResultAbortedTimeout = 6;
constexpr uint8_t kResultAbortedPlannerFailed = 7;
constexpr uint8_t kResultAbortedLostLocalization = 8;
constexpr uint8_t kResultAbortedLostTarget = 9;
constexpr uint8_t kResultAbortedVehicleRejected = 10;
constexpr uint8_t kResultAbortedInternalError = 11;
constexpr uint8_t kResultAbortedLowBattery = 12;

/// The 5 outcomes MissionGuard (P9.4's BT decorator) may act on. Never a
/// 6th "RETRY" action: a retry that still has budget is reported as
/// kContinue (S:2 "retry backoff 3s x2" is the CALLER's scheduling, not a
/// distinct guard verdict) -- see evaluate()'s step-failure branch.
enum class GuardAction : uint8_t
{
  kContinue,
  kPause,
  kEndEarly,
  kAbortHold,
  kAbortLand,
};

const char * guardActionName(GuardAction action);

struct GuardVerdict
{
  GuardAction action = GuardAction::kContinue;
  uint8_t result_code = kResultUnknown;   // meaningful iff action != kContinue
  std::string reason;                     // MissionStatus.last_reason / MissionEvent.description
};

// Flat, caller-computed snapshot of the world MissionGuard ticks against.
// Every "*_elapsed_sec" field is the duration a CONDITION has held
// continuously true, tracked by the CALLER (mission_executor_node, P9.4) --
// this library never reads a clock (R21) and holds no state of its own
// (evaluate() is a pure function of one MissionWorldView). This is a
// DELIBERATE simplification from failsafe_policy's internal ContinuityTracker
// style: P9.4's broker already needs its own per-cause timers for retry
// backoff and search sub-behaviours, so consolidating dwell bookkeeping one
// layer up keeps this "brain" a plain decision table, table-tested with no
// time-stepping loops (R27-3).
struct MissionWorldView
{
  // Whether ExecuteMission currently has an active goal (RUNNING or PAUSED,
  // i.e. not IDLE/terminal). The guard is a no-op (kContinue) when false.
  bool mission_in_flight = false;

  // --- Priority 1: operator abort (AbortMission.srv "does not land"). ---
  bool operator_abort_requested = false;

  // --- Priority 2: control authority (P7 arbiter, ControlAuthority.msg,
  // TransientLocal 2 Hz contract). ---
  uint8_t authority_active_source = kSourceNone;
  // Age of the LAST received ControlAuthority message (R32 sample-and-hold);
  // used only to feed the "not held" predicate below and to word the reason.
  double authority_age_sec = 0.0;
  // Elapsed time the predicate (active_source != MISSION ||
  // authority_age_sec > 2 x authority_heartbeat_copy_sec) has held TRUE
  // continuously; 0 whenever mission currently, freshly holds authority.
  double authority_seize_elapsed_sec = 0.0;

  // --- Priority 3: battery (VehicleHealth.battery_remaining; R30: NaN means
  // "not measured", never 0.0). Checked AFTER authority on purpose (plan P9
  // S:2: "an end_early command sent without authority is a command into the
  // void"). ---
  double battery_remaining = 0.0;             // NaN if unavailable, 0..1 otherwise
  double battery_warn_elapsed_sec = 0.0;      // elapsed time continuously < warn threshold
  double battery_unknown_elapsed_sec = 0.0;   // elapsed time continuously unmeasurable while flying

  // --- Priority 4: pause (explicit PauseMission service, or already
  // PAUSED awaiting ResumeMission). ---
  bool operator_pause_requested = false;
  bool paused = false;                 // MissionStatus.state == STATE_PAUSED right now
  double paused_elapsed_sec = 0.0;     // meaningful iff paused

  // --- Priority 5: step/body failure (outcome of the last finished nav
  // action). Reset at the STEP boundary by the caller, not by this library
  // ("Retry reset o RANH GIOI BUOC", plan P9 S:2). ---
  bool localization_valid = false;
  double localization_age_sec = 0.0;
  uint8_t last_nav_result_code = kResultUnknown;
  uint32_t retry_count = 0;                   // retries already spent on the CURRENT step
  bool last_failure_reason_repeated = false;  // this attempt's reason == the previous attempt's
};

// Parameters. Every "*_copy" field is a documented COPY of a threshold that
// truly lives in another package's parameter server (same convention as
// uav_safety's FailsafePolicyParams) -- validate() enforces the pairing
// rules; it never fixes a bad value, it refuses to start.
struct MissionPolicyParams
{
  // --- Priority 2: authority ---
  double authority_heartbeat_copy_sec = 0.5;   // COPY: ControlAuthority publish period (2 Hz, P7)
  double authority_loss_grace_sec = 1.5;       // plan P9 S:2 dwell; >= 2 x heartbeat_copy AND >= 1.5

  // --- Priority 3: battery (Q-P9-2, chu du an ky 2026-08-22) ---
  double battery_warn_threshold = 0.35;
  double battery_min_start_threshold = 0.40;   // isBatteryOkToStart() gate, not evaluate()
  double battery_warn_dwell_sec = 3.0;         // R29 grace
  double battery_unknown_timeout_sec = 30.0;
  double battery_health_period_copy_sec = 0.1;  // COPY: px4_state_adapter_node publish_rate_hz (10 Hz)

  // --- Priority 4: pause ---
  double paused_timeout_sec = 180.0;           // plan P9 S:2

  // --- Priority 5: step failure / Q-P9-1 land-vs-hold ---
  double localization_status_timeout_copy_sec = 1.0;   // COPY: failsafe_policy's status_timeout_sec
  uint32_t max_step_retries = 2;               // plan P9 S:2 "retry backoff 3s x2"
  double retry_backoff_sec = 3.0;              // documented for the caller's own scheduler

  // --- Cross-package step timeout budget (plan P9 S:3.4) ---
  double navigator_goto_timeout_copy_sec = 240.0;   // COPY: navigator_action_server_node goto_timeout_sec
  double step_timeout_sec = 300.0;             // >= 1.2 x navigator_goto_timeout_copy_sec
};

/// Pure policy engine: one evaluate() per MissionGuard tick (10 Hz, plan P9
/// S:1 "tick theo dong ho, khong event-driven"). No clock read anywhere --
/// every dwell is a caller-supplied elapsed-seconds count (R21), and every
/// entry point is meant for a single, non-concurrent context (same
/// discipline as uav_safety's FailsafePolicy / uav_control_authority's
/// AuthorityArbiter).
class MissionPolicy
{
public:
  /// Throws std::invalid_argument if params are unusable -- refuse to
  /// start, same rule as FailsafePolicy and AuthorityArbiter.
  explicit MissionPolicy(const MissionPolicyParams & params);

  /// Evaluates the priority table (plan P9 S:2), stopping at the first
  /// condition that cuts: 1 operator abort -> 2 authority seize ->
  /// 3 battery -> 4 pause -> 5 step failure -> else CONTINUE.
  GuardVerdict evaluate(const MissionWorldView & view) const;

  const MissionPolicyParams & params() const {return params_;}

private:
  void validate() const;

  MissionPolicyParams params_;
};

/// Q-P9-2 min-start gate: whether a NEW mission is allowed to begin given
/// the battery at goal-acceptance time. Separate from evaluate() -- this is
/// a one-shot check at ExecuteMission goal acceptance (P9.4), not a
/// per-tick reactive condition. NaN (unmeasurable) never passes (R30).
bool isBatteryOkToStart(double battery_remaining, const MissionPolicyParams & params);

}  // namespace uav_mission

#endif  // UAV_MISSION__MISSION_POLICY_HPP_
