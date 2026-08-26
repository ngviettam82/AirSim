#ifndef UAV_CONTROL_AUTHORITY__AUTHORITY_ARBITER_HPP_
#define UAV_CONTROL_AUTHORITY__AUTHORITY_ARBITER_HPP_

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace uav_control_authority
{

// Mirrors uav_interfaces/msg/ControlAuthority.msg SOURCE_*. Kept as plain
// constants (not the message type) so this library stays ROS-free, same rule
// uav_navigation's goal_admission/carrot follow.
constexpr uint8_t kSourceNone = 0;
constexpr uint8_t kSourceTest = 1;
constexpr uint8_t kSourceMission = 2;
constexpr uint8_t kSourceOperator = 3;
constexpr uint8_t kSourceSafety = 4;

// Mirrors uav_interfaces/msg/ControlCommand.msg MODE_* -- only the three the
// gateway actually streams (px4_command_gateway_node.cpp:150-160).
constexpr uint8_t kModePosition = 1;
constexpr uint8_t kModeVelocity = 2;
constexpr uint8_t kModeAcceleration = 3;

/// Every shipped source publishes at this rate (gateway + navigator config).
/// Named so validate() and any caller reason about the same number (G6).
constexpr double kAssumedSourceStreamHz = 20.0;

/// The monitor timer's own period. Shared with control_authority_manager_node
/// so the timer and the validation math that reasons about it never drift
/// apart into two constants encoding one number (G6, 4 prior recurrences).
constexpr double kMonitorPeriodSec = 0.05;

bool isSourceChannel(uint8_t source);
const char * sourceName(uint8_t source);

struct ArbiterParams
{
  double source_timeout_sec = 0.20;
  double downstream_command_timeout_sec = 0.50;
  double release_dwell_sec = 0.20;
  double latch_grace_sec = 2.0;
  double latch_timeout_sec = 5.0;
  bool check_command_age = true;
  double max_command_age_sec = 0.5;
  bool enable_test_source = true;
  bool enable_operator_source = true;
  std::string odom_frame = "odom";

  // Y5 used to be a separate demote_after_bad_ticks counter here. B2 (review
  // 2026-08-20) removed it: once liveness/latch/dwell all key off VALID
  // arrivals only (see last_valid_arrival_ below), a channel spewing garbage
  // was never live in the first place, so it can never win or hold a latch
  // in the first place -- the streak counter had become dead code protecting
  // against a scenario the structural fix already makes unreachable, and
  // dead safety code is worse than no code (it cannot be exercised by test).
};

/// The fields the arbiter actually inspects, converted from ControlCommand by
/// the ROS layer. Never carries geometry the core would need to interpret --
/// it only ever forwards or blocks, per S:1 of the plan.
struct CommandSnapshot
{
  double stamp_sec = 0.0;
  std::string frame_id;
  uint8_t control_mode = 0;
  uint8_t source = 0;
  double position[3] = {0.0, 0.0, 0.0};
  double velocity[3] = {0.0, 0.0, 0.0};
  double acceleration[3] = {0.0, 0.0, 0.0};
  double yaw = 0.0;
  double yaw_rate = 0.0;
};

enum class DropReason
{
  NONE,
  NOT_WINNER,
  NOT_FINITE,
  WRONG_FRAME,
  UNSUPPORTED_MODE,
  STALE,
};

const char * toString(DropReason reason);

enum class PublishAction
{
  DROP,
  PUBLISH,
  PUBLISH_RESTAMPED,
};

struct ArrivalResult
{
  PublishAction action = PublishAction::DROP;
  DropReason drop_reason = DropReason::NOT_WINNER;
  bool authority_changed = false;
  uint8_t active_source = kSourceNone;
  uint8_t previous_source = kSourceNone;
};

enum class LatchRevocation
{
  NONE,
  GRACE_EXPIRED,
  TIMED_OUT,
};

struct TickResult
{
  bool authority_changed = false;
  uint8_t active_source = kSourceNone;
  uint8_t previous_source = kSourceNone;
  LatchRevocation latch_revocation = LatchRevocation::NONE;
};

struct LatchOutcome
{
  bool granted = false;
  uint8_t granted_source = kSourceNone;
  std::string message;
};

/// P8 R2: outcome of clearSafetyLatch(), the ONLY way to release a SAFETY
/// latch. Always "cleared" -- this call never judges whether clearing is
/// warranted (that belongs to uav_safety's own ClearFault), it only executes.
struct ClearSafetyLatchOutcome
{
  bool cleared = false;
  std::string message;
};

/// Priority mux for /control/command_selected: one topic per priority level,
/// asymmetric hysteresis, and a SetControlAuthority latch that raises the
/// floor. Pure relay -- never synthesizes a setpoint, never reads a clock
/// (every "now" is a caller-supplied second count, R21 discipline).
///
/// R24: this object is NOT internally synchronised. Every entry point
/// (onMessageArrived/onTick/requestLatch) must be called from callbacks that
/// cannot run concurrently with each other -- control_authority_manager_node
/// puts all three in the SAME MutuallyExclusive callback group for exactly
/// this reason. Do not call this class from more than one callback group.
class AuthorityArbiter
{
public:
  /// Throws std::invalid_argument if params are unusable -- refuse to start,
  /// same rule as navigator_action_server_node::rejectUnusableTrajectoryLimits.
  explicit AuthorityArbiter(const ArbiterParams & params);

  /// A ControlCommand physically arrived on channel's topic. channel must be
  /// one of kSourceTest..kSourceSafety.
  ArrivalResult onMessageArrived(uint8_t channel, const CommandSnapshot & command, double now_sec);

  /// Monitor tick (period kMonitorPeriodSec): release-dwell expiry, latch
  /// grace/timeout, and recovering active_source after a latch floor moves.
  /// Silence never triggers a publish; it only ever changes who WOULD win
  /// the next message.
  TickResult onTick(double now_sec);

  /// SetControlAuthority. requested_source == kSourceNone releases whatever
  /// latch is held; any other value asks for a floor at that level.
  LatchOutcome requestLatch(uint8_t requested_source, double now_sec);

  uint8_t activeSource() const {return active_source_;}
  bool latchActive() const {return latch_active_;}
  uint8_t latchLevel() const {return latch_active_ ? latch_level_ : kSourceNone;}
  /// Y4: the latch only EXCLUDES lower channels once its holder has proven
  /// itself alive at least once. latchActive() can be true while this is
  /// false -- that is the whole point (a latch that never publishes must
  /// not be able to silence a mission that is actually flying).
  /// P8 R2 exception: a SAFETY latch is effective immediately, proof or not
  /// -- policy-1 (P8) forbids SAFETY from ever publishing a setpoint on a
  /// broken pose, so it can never "prove it's driving" the way the other
  /// three levels can; requiring proof from it would make INHIBIT impossible.
  bool latchEffective() const
  {
    return latch_active_ && (latch_ever_alive_ || latch_level_ == kSourceSafety);
  }

  /// Y15: raw liveness flag of the CURRENT grant, regardless of level --
  /// diagnostics only (distinguishes "SAFETY latch effective because it's
  /// SAFETY" from "SAFETY latch effective AND already proven alive").
  bool latchEverAlive() const {return latch_ever_alive_;}

  /// Y15: seconds since the current latch was granted; 0 if none is held.
  /// C4 (P10.8b review finding): routes through ageSecOrInf() like every
  /// other age in this class -- a clock rewind past latch_granted_at_sec_
  /// used to read as a raw NEGATIVE number here, the opposite of what the
  /// "latch_age_sec growing unbounded means a dead SAFETY node" monitor
  /// (README/yaml) needs. NaN ("cannot be measured", R30) replaces +inf:
  /// this value is a diagnostics DISPLAY number, never compared against a
  /// threshold, so NaN reads unambiguously as "not measurable" instead of
  /// silently sorting as the largest possible age.
  double latchAgeSec(double now_sec) const
  {
    if (!latch_active_) {
      return 0.0;
    }
    const double age = ageSecOrInf(now_sec, latch_granted_at_sec_);
    return std::isinf(age) ? std::numeric_limits<double>::quiet_NaN() : age;
  }

  /// Y15/R26: the deliberate INHIBIT state -- a SAFETY latch effective with
  /// zero proof of life, i.e. the holder has published nothing and never
  /// will (policy-1). Distinct from latchEffective() alone: MISSION/OPERATOR/
  /// TEST latches are never "inhibit", they either haven't proven themselves
  /// yet (not effective) or have (effective AND alive).
  bool inhibitActive() const
  {
    return latch_active_ && latch_level_ == kSourceSafety && !latch_ever_alive_;
  }

  /// P8 R2: the ONLY way to release a SAFETY latch -- requestLatch(kSourceNone,
  /// ...) still unconditionally refuses it (Y6). A no-op success if no SAFETY
  /// latch is currently held: this call never judges whether clearing is
  /// warranted, it only executes what the caller (uav_safety, via its own
  /// accepted ClearFault) has already decided.
  ClearSafetyLatchOutcome clearSafetyLatch(double now_sec);

  unsigned droppedNotFiniteCount() const {return dropped_not_finite_;}
  unsigned droppedWrongFrameCount() const {return dropped_wrong_frame_;}
  /// B1: PER-CHANNEL wrong_frame count -- the aggregate above mixes in
  /// errors from channels that never won authority, which let one stray
  /// wrong-frame message on an UNRELATED channel latch FRAME_MISMATCH
  /// forever downstream (uav_safety). 0 for any non-source channel.
  unsigned droppedWrongFrameCount(uint8_t channel) const;
  unsigned droppedModeCount() const {return dropped_mode_;}
  unsigned droppedStaleCount() const {return dropped_stale_;}
  unsigned restampedCount() const {return restamped_;}
  unsigned releaseAttemptCount() const {return release_attempts_;}
  unsigned releaseRejectedCount() const {return release_rejected_;}
  unsigned clearSafetyLatchAttemptCount() const {return clear_safety_latch_attempts_;}
  unsigned clearSafetyLatchSuccessCount() const {return clear_safety_latch_success_;}

  /// B2-d2: age of the last RAW message (valid or not) on a channel --
  /// diagnostics only, never read by any arbitration decision. This is what
  /// tells "channel dead" (age keeps growing without bound) apart from
  /// "channel alive but 100% garbage" (age stays small while the channel
  /// still never wins) -- two very different failure modes to debug.
  /// +inf if the channel has never sent anything at all.
  double rawArrivalAgeSec(uint8_t source, double now_sec) const;

  /// N-c (P10.8, Q-P10-7): number of times any age computation below observed
  /// now_sec < stamp -- a sim clock rewind (e.g. bag replay/loop). Diagnostics
  /// only, never read by any arbitration decision; a nonzero value means at
  /// least one channel read as "not LIVE"/"latch expired" purely because the
  /// clock moved backward, not because anything actually went quiet.
  unsigned clockRegressionsCount() const {return clock_regressions_;}

private:
  void validate() const;
  bool channelEnabled(uint8_t source) const;
  /// B2 (review 2026-08-20): liveness is judged on VALID arrivals only
  /// (last_valid_arrival_), never on raw arrivals. A channel spewing NaN/
  /// wrong-frame/stale garbage at 20 Hz must never read as "alive" -- alive
  /// is a claim about USABLE evidence, and garbage is the opposite of that.
  bool isLive(uint8_t source, double now_sec) const;
  /// Y4: a latch with a holder that has never published a VALID message has
  /// NO effect (B2: "published" means validated, not merely arrived).
  uint8_t currentFloor() const {return latchEffective() ? latch_level_ : kSourceNone;}
  /// exclude lets a caller ask "who else is live", used only when the channel
  /// being excluded is the one THIS call is in the middle of releasing --
  /// without it, a channel released for exceeding release_dwell_sec could be
  /// handed control right back in the same call, since source_timeout_sec
  /// (a different, looser threshold) may still call it live.
  uint8_t computeWinner(double now_sec, uint8_t floor, uint8_t exclude = kSourceNone) const;
  /// N-c (P10.8) + C4 (P10.8b): the ONLY place any "now - stamp" subtraction
  /// may happen in this class -- EVERY caller routes through here, including
  /// validateContent()'s content-age (STALE) guard and latchAgeSec() (C4:
  /// both used to do their own raw subtraction, missed by N-c's first pass).
  /// now_sec < stamp_sec (clock rewind) reads as +inf -- the same value
  /// already used for "never arrived" -- and counts into clock_regressions_,
  /// so a rewind can never silently make a dead channel read as LIVE/fresh
  /// forever (the bug this fixes) nor pass unnoticed (Q-P10-7: "not
  /// measurable" must never look like "OK"). const + the counter is mutable:
  /// isLive()/rawArrivalAgeSec() are const call sites.
  double ageSecOrInf(double now_sec, double stamp_sec) const;
  DropReason validateContent(const CommandSnapshot & command, double now_sec) const;
  void countDrop(DropReason reason, uint8_t channel);
  LatchRevocation processLatchExpiry(double now_sec);
  void enforceFloorNow(double now_sec);
  void setActiveSource(uint8_t candidate);

  ArbiterParams params_;

  uint8_t active_source_ = kSourceNone;
  // B2: raw arrival, EVERY message regardless of content -- kept only as a
  // diagnostics signal (age of the last thing heard at all, valid or not),
  // never read by any arbitration decision below.
  bool has_arrived_[5] = {false, false, false, false, false};
  double last_arrival_[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  // B2: the ONLY clock arbitration decisions may read -- updated exclusively
  // when validateContent() passes. isLive(), latch_ever_alive_, the release
  // dwell check, and processLatchExpiry() all use this, never last_arrival_.
  bool has_valid_arrived_[5] = {false, false, false, false, false};
  double last_valid_arrival_[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

  bool latch_active_ = false;
  uint8_t latch_level_ = kSourceNone;
  double latch_granted_at_sec_ = 0.0;
  bool latch_ever_alive_ = false;

  unsigned dropped_not_finite_ = 0;
  unsigned dropped_wrong_frame_ = 0;
  // B1: indexed by source (kSourceTest..kSourceSafety, 1..4); index 0 unused.
  unsigned dropped_wrong_frame_per_channel_[5] = {0, 0, 0, 0, 0};
  unsigned dropped_mode_ = 0;
  unsigned dropped_stale_ = 0;
  unsigned restamped_ = 0;
  unsigned release_attempts_ = 0;
  unsigned release_rejected_ = 0;
  unsigned clear_safety_latch_attempts_ = 0;
  unsigned clear_safety_latch_success_ = 0;
  // N-c (P10.8): mutable -- incremented from ageSecOrInf(), called by the
  // const methods isLive()/rawArrivalAgeSec()/computeWinner(). Diagnostics
  // only, same rule as every other counter in this class (B1/B2-d2 above).
  mutable unsigned clock_regressions_ = 0;
};

}  // namespace uav_control_authority

#endif  // UAV_CONTROL_AUTHORITY__AUTHORITY_ARBITER_HPP_
