#include "uav_control_authority/authority_arbiter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace uav_control_authority
{

namespace
{

void require(bool condition, const std::string & detail)
{
  if (!condition) {
    throw std::invalid_argument("control authority arbiter parameters: " + detail);
  }
}

bool allFinite3(const double values[3])
{
  return std::isfinite(values[0]) && std::isfinite(values[1]) && std::isfinite(values[2]);
}

/// The mode picks which fields the gateway reads (publishTrajectorySetpoint()
/// mirrors this exact switch); a field the mode never uses may safely be NaN.
bool contentFinite(const CommandSnapshot & command)
{
  switch (command.control_mode) {
    case kModePosition:
      return allFinite3(command.position) && std::isfinite(command.yaw);
    case kModeVelocity:
      return allFinite3(command.velocity) && std::isfinite(command.yaw_rate);
    case kModeAcceleration:
      return allFinite3(command.acceleration) && std::isfinite(command.yaw);
    default:
      return false;    // unreachable: mode is filtered before this runs
  }
}

}  // namespace

bool isSourceChannel(uint8_t source)
{
  return source == kSourceTest || source == kSourceMission ||
    source == kSourceOperator || source == kSourceSafety;
}

const char * sourceName(uint8_t source)
{
  switch (source) {
    case kSourceNone: return "NONE";
    case kSourceTest: return "TEST";
    case kSourceMission: return "MISSION";
    case kSourceOperator: return "OPERATOR";
    case kSourceSafety: return "SAFETY";
    default: return "UNKNOWN";
  }
}

const char * toString(DropReason reason)
{
  switch (reason) {
    case DropReason::NONE: return "none";
    case DropReason::NOT_WINNER: return "not_winner";
    case DropReason::NOT_FINITE: return "not_finite";
    case DropReason::WRONG_FRAME: return "wrong_frame";
    case DropReason::UNSUPPORTED_MODE: return "unsupported_mode";
    case DropReason::STALE: return "stale";
    default: return "unknown";
  }
}

AuthorityArbiter::AuthorityArbiter(const ArbiterParams & params)
: params_(params)
{
  validate();
}

void AuthorityArbiter::validate() const
{
  const double min_source_timeout = 3.0 / kAssumedSourceStreamHz;

  require(
    std::isfinite(params_.source_timeout_sec) && params_.source_timeout_sec > min_source_timeout,
    "source_timeout_sec must be finite and exceed 3 missed ticks at the shipped stream rate");
  require(
    std::isfinite(params_.downstream_command_timeout_sec) &&
      params_.downstream_command_timeout_sec > 0.0,
    "downstream_command_timeout_sec must be finite and positive");
  require(
    params_.source_timeout_sec <= 0.6 * params_.downstream_command_timeout_sec,
    "source_timeout_sec must be <= 0.6 x downstream_command_timeout_sec (gateway pairing rule)");
  require(
    std::isfinite(params_.release_dwell_sec) && params_.release_dwell_sec > 0.0,
    "release_dwell_sec must be finite and positive");
  require(
    params_.release_dwell_sec >= params_.source_timeout_sec,
    "release_dwell_sec must be >= source_timeout_sec (Y3: otherwise NONE and the "
    "just-released channel can chatter against each other at the shared boundary)");
  require(
    params_.release_dwell_sec + 2.0 * kMonitorPeriodSec + 1.0 / kAssumedSourceStreamHz <=
      0.8 * params_.downstream_command_timeout_sec,
    "release_dwell_sec + 2x monitor period + 1/stream_hz must be <= 0.8 x "
    "downstream_command_timeout_sec (Y1: worst-case handover must not starve the gateway)");
  require(
    std::isfinite(params_.latch_grace_sec) && params_.latch_grace_sec > 0.0,
    "latch_grace_sec must be finite and positive");
  require(
    std::isfinite(params_.latch_timeout_sec) && params_.latch_timeout_sec > params_.latch_grace_sec,
    "latch_timeout_sec must be finite and exceed latch_grace_sec");
  require(
    std::isfinite(params_.max_command_age_sec) && params_.max_command_age_sec > 0.0,
    "max_command_age_sec must be finite and positive even when check_command_age is false");
  require(!params_.odom_frame.empty(), "odom_frame must not be empty");
}

bool AuthorityArbiter::channelEnabled(uint8_t source) const
{
  switch (source) {
    case kSourceTest: return params_.enable_test_source;
    case kSourceOperator: return params_.enable_operator_source;
    case kSourceMission: return true;
    case kSourceSafety: return true;
    default: return false;
  }
}

double AuthorityArbiter::ageSecOrInf(double now_sec, double stamp_sec) const
{
  // N-c (P10.8, Q-P10-7): a rewound clock (now < stamp) is "cannot be
  // measured", not a negative age -- +inf reads the same way "never arrived"
  // already does everywhere below, so no caller needs a second special case.
  if (now_sec < stamp_sec) {
    ++clock_regressions_;
    return std::numeric_limits<double>::infinity();
  }
  return now_sec - stamp_sec;
}

bool AuthorityArbiter::isLive(uint8_t source, double now_sec) const
{
  if (!isSourceChannel(source) || !has_valid_arrived_[source]) {
    return false;
  }
  return ageSecOrInf(now_sec, last_valid_arrival_[source]) <= params_.source_timeout_sec;
}

double AuthorityArbiter::rawArrivalAgeSec(uint8_t source, double now_sec) const
{
  if (!isSourceChannel(source) || !has_arrived_[source]) {
    return std::numeric_limits<double>::infinity();
  }
  return ageSecOrInf(now_sec, last_arrival_[source]);
}

uint8_t AuthorityArbiter::computeWinner(double now_sec, uint8_t floor, uint8_t exclude) const
{
  const int lowest = std::max<int>(floor, kSourceTest);
  for (int source = kSourceSafety; source >= lowest; --source) {
    const uint8_t candidate = static_cast<uint8_t>(source);
    if (candidate != exclude && channelEnabled(candidate) && isLive(candidate, now_sec)) {
      return candidate;
    }
  }
  return kSourceNone;
}

DropReason AuthorityArbiter::validateContent(const CommandSnapshot & command, double now_sec) const
{
  // Mode first: it decides which fields below are even meaningful to read.
  if (command.control_mode != kModePosition && command.control_mode != kModeVelocity &&
    command.control_mode != kModeAcceleration)
  {
    return DropReason::UNSUPPORTED_MODE;
  }
  if (!contentFinite(command)) {
    return DropReason::NOT_FINITE;
  }
  if (command.frame_id != params_.odom_frame) {
    return DropReason::WRONG_FRAME;
  }
  // C4 (P10.8b review finding): must go through ageSecOrInf() like every
  // other age check in this file -- a raw subtraction here went NEGATIVE on
  // a clock rewind (bag replay/loop, or a publisher that forgot
  // use_sim_time), which is never > max_command_age_sec, so the ONE guard
  // that reads command CONTENT age (liveness below reads ARRIVAL time,
  // never this) silently let unmeasurable-age commands through as if fresh.
  if (params_.check_command_age &&
    ageSecOrInf(now_sec, command.stamp_sec) > params_.max_command_age_sec)
  {
    return DropReason::STALE;
  }
  return DropReason::NONE;
}

void AuthorityArbiter::countDrop(DropReason reason, uint8_t channel)
{
  switch (reason) {
    case DropReason::NOT_FINITE: ++dropped_not_finite_; break;
    case DropReason::WRONG_FRAME:
      ++dropped_wrong_frame_;
      // B1: per-channel breakdown alongside the aggregate.
      if (isSourceChannel(channel)) {
        ++dropped_wrong_frame_per_channel_[channel];
      }
      break;
    case DropReason::UNSUPPORTED_MODE: ++dropped_mode_; break;
    case DropReason::STALE: ++dropped_stale_; break;
    default: break;
  }
}

unsigned AuthorityArbiter::droppedWrongFrameCount(uint8_t channel) const
{
  return isSourceChannel(channel) ? dropped_wrong_frame_per_channel_[channel] : 0U;
}

void AuthorityArbiter::setActiveSource(uint8_t candidate)
{
  active_source_ = candidate;
}

ArrivalResult AuthorityArbiter::onMessageArrived(
  uint8_t channel, const CommandSnapshot & command, double now_sec)
{
  ArrivalResult result;
  result.previous_source = active_source_;
  result.active_source = active_source_;

  if (!isSourceChannel(channel) || !channelEnabled(channel)) {
    result.drop_reason = DropReason::NOT_WINNER;
    return result;
  }

  // B2: raw arrival is diagnostics-only from here -- recorded, but nothing
  // below (liveness, latch, winner selection) is allowed to read it.
  has_arrived_[channel] = true;
  last_arrival_[channel] = now_sec;

  // B2: content is judged BEFORE it can count as evidence of anything. A
  // malformed message is not "the channel is alive" -- it is proof this
  // channel cannot currently be trusted with authority, the opposite claim.
  const DropReason content_issue = validateContent(command, now_sec);
  if (content_issue != DropReason::NONE) {
    countDrop(content_issue, channel);
    result.drop_reason = content_issue;
    return result;
  }

  // Only a VALID message is liveness evidence from here on.
  has_valid_arrived_[channel] = true;
  last_valid_arrival_[channel] = now_sec;
  if (latch_active_ && channel == latch_level_) {
    latch_ever_alive_ = true;
  }

  // Up only: an arriving VALID message may promote a higher-priority channel
  // to active immediately; silence -- never a message -- starts the
  // release-dwell clock that lets active_source step DOWN (S:3b).
  const uint8_t candidate = computeWinner(now_sec, currentFloor());
  if (candidate != kSourceNone && candidate > active_source_) {
    setActiveSource(candidate);
  }

  result.active_source = active_source_;
  result.authority_changed = (active_source_ != result.previous_source);

  if (channel != active_source_) {
    result.drop_reason = DropReason::NOT_WINNER;
    return result;
  }

  result.drop_reason = DropReason::NONE;
  if (command.source != channel) {
    ++restamped_;
    result.action = PublishAction::PUBLISH_RESTAMPED;
  } else {
    result.action = PublishAction::PUBLISH;
  }
  return result;
}

LatchRevocation AuthorityArbiter::processLatchExpiry(double now_sec)
{
  if (!latch_active_) {
    return LatchRevocation::NONE;
  }

  if (!latch_ever_alive_) {
    // P8 R2: a SAFETY latch that has never proven itself alive is the
    // deliberate INHIBIT state (policy-1: SAFETY may never publish a
    // setpoint on a broken pose, so it can never "prove it's driving") --
    // it must NOT grace-expire. Every other level keeps the old Y4 rule,
    // unchanged: this early return is the only new branch here.
    if (latch_level_ == kSourceSafety) {
      return LatchRevocation::NONE;
    }
    if (ageSecOrInf(now_sec, latch_granted_at_sec_) > params_.latch_grace_sec) {
      latch_active_ = false;
      return LatchRevocation::GRACE_EXPIRED;
    }
    return LatchRevocation::NONE;
  }

  // B2: garbage from the holder must never feed the latch's own clock --
  // last_valid_arrival_ only advances on content that passed validateContent().
  // latch_ever_alive_ implies has_valid_arrived_[latch_level_] (both are set
  // together, in onMessageArrived, at the exact same valid arrival).
  // R5: SAFETY latch never times out once alive either.
  if (latch_level_ == kSourceSafety) {
    return LatchRevocation::NONE;
  }
  if (ageSecOrInf(now_sec, last_valid_arrival_[latch_level_]) > params_.latch_timeout_sec) {
    latch_active_ = false;
    return LatchRevocation::TIMED_OUT;
  }
  return LatchRevocation::NONE;
}

TickResult AuthorityArbiter::onTick(double now_sec)
{
  TickResult result;
  result.previous_source = active_source_;

  result.latch_revocation = processLatchExpiry(now_sec);

  // Deliberately NOT gated on isLive()/source_timeout_sec here: source_timeout_sec
  // decides which OTHER channels are eligible replacements (computeWinner), but
  // whether the active channel itself has earned a downgrade is release_dwell_sec's
  // question alone -- that separation is what keeps a single missed tick from
  // flapping active_source even when the two constants share a default value.
  // B2: silence is measured on last_valid_arrival_ -- a channel that holds
  // authority by spewing garbage still falls out after release_dwell_sec,
  // because garbage never refreshes this clock at all.
  const uint8_t floor = currentFloor();
  if (active_source_ != kSourceNone) {
    const bool below_floor = active_source_ < floor;
    const bool silent_past_dwell =
      ageSecOrInf(now_sec, last_valid_arrival_[active_source_]) >= params_.release_dwell_sec;
    if (below_floor || silent_past_dwell) {
      // Exclude the channel being released: source_timeout_sec may still call
      // it "live" at this exact instant (validated release_dwell_sec >= it),
      // which would otherwise hand control right back in this same call.
      setActiveSource(computeWinner(now_sec, floor, active_source_));
    }
  } else {
    setActiveSource(computeWinner(now_sec, floor));
  }

  result.active_source = active_source_;
  result.authority_changed = (result.active_source != result.previous_source);
  return result;
}

void AuthorityArbiter::enforceFloorNow(double now_sec)
{
  const uint8_t floor = currentFloor();
  if (active_source_ == kSourceNone || active_source_ < floor) {
    setActiveSource(computeWinner(now_sec, floor));
  }
}

LatchOutcome AuthorityArbiter::requestLatch(uint8_t requested_source, double now_sec)
{
  LatchOutcome outcome;

  if (requested_source == kSourceNone) {
    ++release_attempts_;
    if (latch_active_ && latch_level_ == kSourceSafety) {
      // Y6: a SAFETY latch is cleared only by P8's ClearFault, never by a
      // plain release request -- whatever raised it stays authoritative
      // until the fault it responded to is explicitly acknowledged.
      ++release_rejected_;
      outcome.granted = false;
      outcome.granted_source = latch_level_;
      outcome.message = "rejected: a SAFETY latch can only be cleared by ClearFault (P8)";
    } else {
      latch_active_ = false;
      latch_level_ = kSourceNone;
      latch_ever_alive_ = false;
      outcome.granted = true;
      outcome.granted_source = kSourceNone;
      outcome.message = "latch released";
    }
  } else if (!isSourceChannel(requested_source)) {
    outcome.granted = false;
    outcome.granted_source = latchLevel();
    outcome.message = "requested_source is not a valid channel";
  } else if (latch_active_ && latch_level_ > requested_source) {
    outcome.granted = false;
    outcome.granted_source = latch_level_;
    outcome.message = "rejected: current holder has higher priority";
  } else {
    latch_active_ = true;
    latch_level_ = requested_source;
    latch_granted_at_sec_ = now_sec;
    latch_ever_alive_ = false;
    outcome.granted = true;
    outcome.granted_source = requested_source;
    outcome.message = "latch granted";
  }

  // The floor may have just moved; enforce it immediately, not at the next
  // monitor tick -- a latch is a deliberate authority change, not organic decay.
  enforceFloorNow(now_sec);
  return outcome;
}

ClearSafetyLatchOutcome AuthorityArbiter::clearSafetyLatch(double now_sec)
{
  ++clear_safety_latch_attempts_;
  ClearSafetyLatchOutcome outcome;
  if (!latch_active_ || latch_level_ != kSourceSafety) {
    // Idempotent: nothing to clear is still a successful outcome, not an
    // error -- the arbiter never judges whether a call was WARRANTED.
    outcome.cleared = true;
    outcome.message = "no SAFETY latch held";
  } else {
    latch_active_ = false;
    latch_level_ = kSourceNone;
    latch_ever_alive_ = false;
    outcome.cleared = true;
    outcome.message = "SAFETY latch cleared";
  }
  ++clear_safety_latch_success_;

  // A deliberate authority change (or a floor that needs re-syncing even on
  // the no-op path) -- enforce now, same reasoning as requestLatch() above.
  enforceFloorNow(now_sec);
  return outcome;
}

}  // namespace uav_control_authority
