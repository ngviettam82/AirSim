#include "uav_navigation/goal_admission.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>

namespace uav_navigation
{

namespace
{

Admission reject(RejectReason reason, std::string detail)
{
  return Admission{false, reason, std::move(detail)};
}

Admission accept()
{
  return Admission{true, RejectReason::NONE, std::string()};
}

Admission checkAvailability(NavigatorState state, bool needs_flight)
{
  if (!acceptsNewGoal(state)) {
    return reject(
      RejectReason::BUSY,
      std::string("navigator is busy in ") + toString(state));
  }
  if (needs_flight && state != NavigatorState::HOLDING) {
    return reject(RejectReason::NOT_AIRBORNE, "aircraft is not hovering under navigator control");
  }
  return accept();
}

}  // namespace

const char * toString(NavigatorState state)
{
  switch (state) {
    case NavigatorState::IDLE:
      return "IDLE";
    case NavigatorState::TAKING_OFF:
      return "TAKING_OFF";
    case NavigatorState::GOING_TO_POSE:
      return "GOING_TO_POSE";
    case NavigatorState::HOLD_TASK:
      return "HOLD_TASK";
    case NavigatorState::HOLDING:
      return "HOLDING";
    case NavigatorState::FOLLOWING_PATH:
      return "FOLLOWING_PATH";
    case NavigatorState::TRACKING_TARGET:
      return "TRACKING_TARGET";
    case NavigatorState::RECOVERING:
      return "RECOVERING";
    case NavigatorState::LANDING:
      return "LANDING";
  }
  return "UNKNOWN";
}

const char * toString(TaskType task)
{
  switch (task) {
    case TaskType::TAKEOFF:
      return "takeoff";
    case TaskType::GOTO_POSE:
      return "goto_pose";
    case TaskType::HOLD_POSITION:
      return "hold_position";
    case TaskType::LAND:
      return "land";
    case TaskType::FOLLOW_PATH:
      return "follow_path";
    case TaskType::TRACK_TARGET:
      return "track_target";
    case TaskType::RECOVER:
      return "recover";
  }
  return "unknown";
}

const char * toString(RejectReason reason)
{
  switch (reason) {
    case RejectReason::NONE:
      return "accepted";
    case RejectReason::BUSY:
      return "busy";
    case RejectReason::NOT_AIRBORNE:
      return "not airborne";
    case RejectReason::ALREADY_AIRBORNE:
      return "already airborne";
    case RejectReason::INVALID_FRAME:
      return "invalid frame";
    case RejectReason::NOT_FINITE:
      return "not finite";
    case RejectReason::OUT_OF_RANGE:
      return "out of range";
    case RejectReason::UNSUPPORTED:
      return "unsupported";
  }
  return "unknown";
}

bool acceptsNewGoal(NavigatorState state)
{
  return state == NavigatorState::IDLE || state == NavigatorState::HOLDING;
}

Admission admitTakeoff(
  NavigatorState state, const TakeoffRequest & request, const MotionLimits & limits)
{
  const Admission availability = checkAvailability(state, false);
  if (!availability.accepted) {
    return availability;
  }
  if (state != NavigatorState::IDLE) {
    return reject(RejectReason::ALREADY_AIRBORNE, "already flying; takeoff starts from the ground");
  }
  if (!std::isfinite(request.target_altitude)) {
    return reject(RejectReason::NOT_FINITE, "target_altitude is not finite");
  }
  if (request.target_altitude <= 0.0) {
    return reject(RejectReason::OUT_OF_RANGE, "target_altitude must be greater than zero");
  }
  if (request.target_altitude > limits.max_takeoff_altitude) {
    return reject(RejectReason::OUT_OF_RANGE, "target_altitude above max_takeoff_altitude_m");
  }
  return accept();
}

Admission admitGoto(
  NavigatorState state, const GotoRequest & request, const MotionLimits & limits,
  const std::string & odom_frame)
{
  const Admission availability = checkAvailability(state, true);
  if (!availability.accepted) {
    return availability;
  }
  if (request.frame_id != odom_frame) {
    return reject(
      RejectReason::INVALID_FRAME,
      "frame_id \"" + request.frame_id + "\" is not \"" + odom_frame + "\"");
  }
  if (!std::isfinite(request.x) || !std::isfinite(request.y) || !std::isfinite(request.z)) {
    return reject(RejectReason::NOT_FINITE, "target_pose position is not finite");
  }
  if (!std::isfinite(request.acceptance_radius) || !std::isfinite(request.max_speed)) {
    return reject(RejectReason::NOT_FINITE, "acceptance_radius or max_speed is not finite");
  }
  if (request.z < limits.min_altitude) {
    return reject(RejectReason::OUT_OF_RANGE, "target below min_altitude_m");
  }
  if (request.z > limits.max_altitude) {
    return reject(RejectReason::OUT_OF_RANGE, "target above max_altitude_m");
  }
  return accept();
}

Admission admitHold(NavigatorState state, const HoldRequest & request)
{
  const Admission availability = checkAvailability(state, true);
  if (!availability.accepted) {
    return availability;
  }
  if (!std::isfinite(request.duration_seconds)) {
    return reject(RejectReason::NOT_FINITE, "duration_seconds is not finite");
  }
  if (request.duration_seconds < 0.0) {
    return reject(RejectReason::OUT_OF_RANGE, "duration_seconds must not be negative");
  }
  return accept();
}

Admission admitLand(NavigatorState state, const LandRequest & request)
{
  const Admission availability = checkAvailability(state, true);
  if (!availability.accepted) {
    return availability;
  }
  if (request.use_precision_landing) {
    return reject(RejectReason::UNSUPPORTED, "precision landing is not implemented yet");
  }
  return accept();
}

Admission admitFollowPath(
  NavigatorState state, const FollowPathRequest & request, const MotionLimits & limits,
  const std::string & odom_frame)
{
  const Admission availability = checkAvailability(state, true);
  if (!availability.accepted) {
    return availability;
  }
  if (request.frame_id != odom_frame) {
    return reject(
      RejectReason::INVALID_FRAME,
      "path frame_id \"" + request.frame_id + "\" is not \"" + odom_frame + "\"");
  }
  if (!request.path_is_valid) {
    return reject(RejectReason::UNSUPPORTED, "path is not marked valid");
  }
  if (request.waypoints.empty()) {
    return reject(RejectReason::OUT_OF_RANGE, "path has no waypoints");
  }
  if (request.waypoints.size() > kMaxPathWaypoints) {
    return reject(RejectReason::OUT_OF_RANGE, "path has more waypoints than the core will plan");
  }
  if (!std::isfinite(request.max_speed) || !std::isfinite(request.acceptance_radius)) {
    return reject(RejectReason::NOT_FINITE, "max_speed or waypoint_acceptance_radius is not finite");
  }
  for (std::size_t index = 0; index < request.waypoints.size(); ++index) {
    const Vec3 & waypoint = request.waypoints[index];
    char buffer[96];
    if (!isFinite(waypoint)) {
      std::snprintf(buffer, sizeof(buffer), "waypoint %zu is not finite", index);
      return reject(RejectReason::NOT_FINITE, buffer);
    }
    if (waypoint.z < limits.min_altitude) {
      std::snprintf(buffer, sizeof(buffer), "waypoint %zu is below min_altitude_m", index);
      return reject(RejectReason::OUT_OF_RANGE, buffer);
    }
    if (waypoint.z > limits.max_altitude) {
      std::snprintf(buffer, sizeof(buffer), "waypoint %zu is above max_altitude_m", index);
      return reject(RejectReason::OUT_OF_RANGE, buffer);
    }
  }
  return accept();
}

Admission admitTrackTarget(
  NavigatorState state, const TrackRequest & request, const MotionLimits & limits)
{
  (void)limits;
  const Admission availability = checkAvailability(state, true);
  if (!availability.accepted) {
    return availability;
  }
  if (!std::isfinite(request.standoff_distance) || !std::isfinite(request.duration_seconds) ||
    !std::isfinite(request.target_lost_timeout))
  {
    return reject(RejectReason::NOT_FINITE, "standoff, duration or lost timeout is not finite");
  }
  if (request.standoff_distance <= 0.0) {
    return reject(RejectReason::OUT_OF_RANGE, "standoff_distance must be greater than zero");
  }
  if (request.duration_seconds < 0.0) {
    return reject(RejectReason::OUT_OF_RANGE, "duration_seconds must not be negative");
  }
  if (request.target_lost_timeout <= 0.0) {
    return reject(RejectReason::OUT_OF_RANGE, "target_lost_timeout must be greater than zero");
  }
  return accept();
}

Admission admitRecover(NavigatorState state, const RecoverRequest & request)
{
  // Type FIRST, before the state: a type this node will never fly is refused for
  // that reason in every state. Answering "not airborne" to a Recover(LAND) sent on
  // the ground hides the permanent reason behind a temporary one.
  if (!request.type_executable) {
    return reject(RejectReason::UNSUPPORTED, request.refusal);
  }
  if (request.reads_altitude && !std::isfinite(request.safe_altitude)) {
    return reject(RejectReason::NOT_FINITE, "safe_altitude is not finite");
  }
  // No checkAvailability on purpose: this goal exists to interrupt whatever is flying.
  if (state == NavigatorState::IDLE) {
    return reject(RejectReason::NOT_AIRBORNE, "aircraft is not flying under navigator control");
  }
  if (state == NavigatorState::LANDING) {
    return reject(RejectReason::BUSY, "the autopilot is flying the landing");
  }
  if (state == NavigatorState::RECOVERING) {
    return reject(RejectReason::BUSY, "a recovery is already running");
  }
  return accept();
}

GotoLimits resolveGotoLimits(const GotoRequest & request, const MotionLimits & limits)
{
  GotoLimits resolved;
  resolved.acceptance_radius =
    (std::isfinite(request.acceptance_radius) && request.acceptance_radius > 0.0)
    ? request.acceptance_radius
    : limits.acceptance_radius;
  resolved.max_speed = (std::isfinite(request.max_speed) && request.max_speed > 0.0)
    ? std::min(request.max_speed, limits.max_speed)
    : limits.max_speed;
  return resolved;
}

}  // namespace uav_navigation
