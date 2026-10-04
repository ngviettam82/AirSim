#include "uav_navigation/recovery_planner.hpp"

#include <cmath>
#include <cstdio>
#include <string>

namespace uav_navigation
{
namespace
{

bool positiveFinite(double value)
{
  return std::isfinite(value) && value > 0.0;
}

std::string metres(double value)
{
  char text[32];
  std::snprintf(text, sizeof(text), "%.2f m", value);
  return std::string(text);
}

}  // namespace

const char * recoveryTypeName(RecoveryType type)
{
  switch (type) {
    case RecoveryType::Hover:
      return "hover";
    case RecoveryType::ClimbToSafe:
      return "climb to safe altitude";
    case RecoveryType::ReturnHome:
      return "return home";
    case RecoveryType::Unknown:
      break;
  }
  return "unknown";
}

RecoveryPlanner RecoveryPlanner::build(const RecoveryLimits & limits, std::string & reason)
{
  RecoveryPlanner planner;
  reason.clear();

  // Validated, not clamped: a swallowed zero disables the guard.
  if (!positiveFinite(limits.min_travel_m)) {
    reason = "min_travel_m must be finite and positive";
    return planner;
  }
  if (!positiveFinite(limits.max_climb_m)) {
    reason = "max_climb_m must be finite and positive";
    return planner;
  }
  if (!positiveFinite(limits.max_safe_altitude_m)) {
    reason = "max_safe_altitude_m must be finite and positive";
    return planner;
  }
  if (!positiveFinite(limits.min_home_clearance_m)) {
    reason = "min_home_clearance_m must be finite and positive";
    return planner;
  }
  if (!positiveFinite(limits.max_home_distance_m)) {
    reason = "max_home_distance_m must be finite and positive";
    return planner;
  }

  planner.limits_ = limits;
  planner.valid_ = true;
  return planner;
}

RecoveryPlan RecoveryPlanner::plan(
  const RecoveryRequest & request, const RecoveryState & state) const
{
  RecoveryPlan result;
  result.type = request.type;

  if (!valid_) {
    result.reason = "recovery planner was never configured";
    return result;
  }
  if (!isFinite(state.position)) {
    result.reason = "aircraft position is not finite";
    return result;
  }

  switch (request.type) {
    case RecoveryType::Hover:
      return planHover(state);
    case RecoveryType::ClimbToSafe:
      return planClimb(request, state);
    case RecoveryType::ReturnHome:
      return planReturnHome(request, state);
    case RecoveryType::Unknown:
      break;
  }

  result.reason = "unsupported recovery type; only hover, climb and return home exist";
  return result;
}

RecoveryPlan RecoveryPlanner::planHover(const RecoveryState & state) const
{
  RecoveryPlan result;
  result.type = RecoveryType::Hover;
  result.accepted = true;
  result.hold_point = state.position;
  // No waypoints on purpose: holding adds no motion.
  result.reason = "holding at the current position";
  return result;
}

RecoveryPlan RecoveryPlanner::planClimb(
  const RecoveryRequest & request, const RecoveryState & state) const
{
  RecoveryPlan result;
  result.type = RecoveryType::ClimbToSafe;

  const double apex = request.safe_altitude_m;
  if (!std::isfinite(apex)) {
    result.reason = "safe altitude is not finite";
    return result;
  }

  const double climb = apex - state.position.z;
  result.climb_m = climb;

  if (!(climb >= limits_.min_travel_m)) {
    result.reason = "safe altitude " + metres(apex) + " is not " +
      metres(limits_.min_travel_m) + " above the aircraft at " + metres(state.position.z);
    return result;
  }
  if (apex > limits_.max_safe_altitude_m) {
    result.reason = "safe altitude " + metres(apex) + " is above the " +
      metres(limits_.max_safe_altitude_m) + " ceiling";
    return result;
  }
  if (climb > limits_.max_climb_m) {
    result.reason = "climb of " + metres(climb) + " exceeds the " +
      metres(limits_.max_climb_m) + " single-step cap";
    return result;
  }

  const Vec3 top{state.position.x, state.position.y, apex};
  result.waypoints.push_back(state.position);
  result.waypoints.push_back(top);
  result.hold_point = top;
  result.accepted = true;
  result.reason = "climbing " + metres(climb) + " to " + metres(apex);
  return result;
}

RecoveryPlan RecoveryPlanner::planReturnHome(
  const RecoveryRequest & request, const RecoveryState & state) const
{
  RecoveryPlan result;
  result.type = RecoveryType::ReturnHome;

  if (!state.home_known) {
    result.reason = "home has never been received; refusing to assume the origin";
    return result;
  }
  if (!isFinite(state.home)) {
    result.reason = "home is not finite";
    return result;
  }

  const double cruise = request.safe_altitude_m;
  if (!std::isfinite(cruise)) {
    result.reason = "return altitude is not finite";
    return result;
  }

  const double climb = cruise - state.position.z;
  const double reach = horizontalDistance(state.position, state.home);
  result.climb_m = climb;
  result.home_distance_m = reach;

  if (cruise < state.position.z) {
    result.reason = "return altitude " + metres(cruise) + " is below the aircraft at " +
      metres(state.position.z) + "; recovery never descends";
    return result;
  }
  if (cruise - state.home.z < limits_.min_home_clearance_m) {
    result.reason = "return altitude " + metres(cruise) + " leaves less than " +
      metres(limits_.min_home_clearance_m) + " above home at " + metres(state.home.z);
    return result;
  }
  if (cruise > limits_.max_safe_altitude_m) {
    result.reason = "return altitude " + metres(cruise) + " is above the " +
      metres(limits_.max_safe_altitude_m) + " ceiling";
    return result;
  }
  if (climb > limits_.max_climb_m) {
    result.reason = "climb of " + metres(climb) + " exceeds the " +
      metres(limits_.max_climb_m) + " single-step cap";
    return result;
  }
  if (reach > limits_.max_home_distance_m) {
    result.reason = "home is " + metres(reach) + " away, beyond the " +
      metres(limits_.max_home_distance_m) + " recovery range";
    return result;
  }
  if (climb + reach < limits_.min_travel_m) {
    result.reason = "aircraft is already over home, within " +
      metres(limits_.min_travel_m);
    return result;
  }

  result.waypoints.push_back(state.position);
  if (climb >= limits_.min_travel_m) {
    result.waypoints.push_back(Vec3{state.position.x, state.position.y, cruise});
  }
  // Ends above home, never at it: descent is separate.
  const Vec3 over_home{state.home.x, state.home.y, cruise};
  result.waypoints.push_back(over_home);
  result.hold_point = over_home;
  result.accepted = true;
  result.reason = "returning to home " + metres(reach) + " away, holding at " + metres(cruise);
  return result;
}

}  // namespace uav_navigation
