#include "uav_navigation/local_avoidance.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace uav_navigation
{
namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

bool positiveFinite(double value)
{
  return std::isfinite(value) && value > 0.0;
}

Vec3 lerp(const Vec3 & from, const Vec3 & to, double ratio)
{
  return Vec3{
    from.x + (to.x - from.x) * ratio,
    from.y + (to.y - from.y) * ratio,
    from.z + (to.z - from.z) * ratio};
}

std::string metres(double value)
{
  char text[32];
  std::snprintf(text, sizeof(text), "%.2f m", value);
  return std::string(text);
}

std::string seconds(double value)
{
  char text[32];
  std::snprintf(text, sizeof(text), "%.2f s", value);
  return std::string(text);
}

}  // namespace

LocalAvoidance LocalAvoidance::build(const AvoidanceLimits & limits, std::string & reason)
{
  LocalAvoidance planner;
  reason.clear();

  // Validated, not clamped: a swallowed zero here becomes a silent no-op advisor.
  if (!positiveFinite(limits.horizon_m)) {
    reason = "horizon_m must be finite and positive";
    return planner;
  }
  if (!positiveFinite(limits.sample_step_m)) {
    reason = "sample_step_m must be finite and positive";
    return planner;
  }
  if (!positiveFinite(limits.clearance_scan_m)) {
    reason = "clearance_scan_m must be finite and positive";
    return planner;
  }
  if (!positiveFinite(limits.spiral_growth_m)) {
    reason = "spiral_growth_m must be finite and positive";
    return planner;
  }
  if (!positiveFinite(limits.max_escape_climb_m)) {
    reason = "max_escape_climb_m must be finite and positive";
    return planner;
  }
  if (!positiveFinite(limits.map_timeout_sec)) {
    reason = "map_timeout_sec must be finite and positive";
    return planner;
  }
  if (limits.max_spiral_steps < 1) {
    reason = "max_spiral_steps must be at least one";
    return planner;
  }

  planner.limits_ = limits;
  planner.valid_ = true;
  return planner;
}

bool LocalAvoidance::blocked(const Costmap & map, const Vec3 & point) const
{
  const uint8_t cost = map.costAt(point);
  // 255 is beyond the window, not a wall.
  return cost >= kCostInscribed && cost != kCostUnknown;
}

double LocalAvoidance::clearanceAt(const Costmap & map, const Vec3 & point) const
{
  int cell_x = 0;
  int cell_y = 0;
  if (!map.worldToCell(point, cell_x, cell_y)) {
    return limits_.clearance_scan_m;
  }

  const int reach = static_cast<int>(std::ceil(limits_.clearance_scan_m / map.resolution()));
  double nearest = limits_.clearance_scan_m;

  for (int dy = -reach; dy <= reach; ++dy) {
    for (int dx = -reach; dx <= reach; ++dx) {
      const uint8_t cost = map.cost(cell_x + dx, cell_y + dy);
      if (cost < kCostInscribed || cost == kCostUnknown) {
        continue;
      }
      const Vec3 cell = map.cellToWorld(cell_x + dx, cell_y + dy);
      nearest = std::min(nearest, horizontalDistance(point, cell));
    }
  }
  return nearest;
}

bool LocalAvoidance::segmentClear(const Costmap & map, const Vec3 & from, const Vec3 & to) const
{
  const double length = distance3(from, to);
  if (!std::isfinite(length)) {
    return false;
  }
  if (length <= 0.0) {
    return !blocked(map, from);
  }

  const int steps = static_cast<int>(std::ceil(length / limits_.sample_step_m));
  for (int step = 0; step <= steps; ++step) {
    const Vec3 sample = lerp(from, to, static_cast<double>(step) / steps);
    if (map.costAt(sample) == kCostUnknown) {
      return true;                    // clear as far as the map can see
    }
    if (blocked(map, sample)) {
      return false;
    }
  }
  return true;
}

bool LocalAvoidance::findEscape(
  const Costmap & map, const Vec3 & obstacle, const Vec3 & position,
  const Vec3 & goal, Vec3 & escape, std::string & reason) const
{
  const double span_x = goal.x - position.x;
  const double span_y = goal.y - position.y;
  const double span = std::hypot(span_x, span_y);
  if (!(span > 0.0)) {
    reason = "goal is directly overhead, no lateral escape plane";
    return false;
  }
  const double heading_x = span_x / span;
  const double heading_y = span_y / span;

  // The cells are one flattened slice, so a climb past the band is unverified.
  const double climb_cap = std::min(limits_.max_escape_climb_m, map.flightBand());

  int refused_climb = 0;
  int refused_descent = 0;
  int refused_blocked = 0;

  double theta = 1.0;
  for (int step = 0; step < limits_.max_spiral_steps; ++step) {
    const double reach = limits_.spiral_growth_m * theta;
    const double lateral = reach * std::cos(theta);

    Vec3 candidate;
    candidate.x = obstacle.x + lateral * heading_y;
    candidate.y = obstacle.y - lateral * heading_x;
    candidate.z = obstacle.z + reach * std::sin(theta);

    theta = 2.0 * std::sqrt(theta * theta / 4.0 + 1.0);

    if (!isFinite(candidate)) {
      continue;
    }
    if (std::abs(candidate.z - obstacle.z) > climb_cap) {
      ++refused_climb;
      continue;
    }
    // The climb cap is measured from the OBSTACLE, but the cells are one slice
    // around the AIRCRAFT. A route that climbs pulls the two apart, and the gap is
    // exactly how far an allowed escape could sit outside the map that cleared it.
    if (std::abs(candidate.z - map.origin().z) > map.flightBand()) {
      ++refused_climb;
      continue;
    }
    if (!limits_.allow_descent && candidate.z < obstacle.z) {
      ++refused_descent;
      continue;
    }
    if (!segmentClear(map, position, candidate) || !segmentClear(map, candidate, goal)) {
      ++refused_blocked;
      continue;
    }

    escape = candidate;
    return true;
  }

  reason = "no escape in " + std::to_string(limits_.max_spiral_steps) + " turns (" +
    std::to_string(refused_climb) + " too high, " +
    std::to_string(refused_descent) + " would descend, " +
    std::to_string(refused_blocked) + " still blocked)";
  return false;
}

AvoidanceResult LocalAvoidance::advise(
  const Costmap & map, const std::vector<Vec3> & route_ahead, const Vec3 & position) const
{
  AvoidanceResult result;
  result.map_age_sec = map.lastInputAge();
  result.map_fresh = std::isfinite(result.map_age_sec) &&
    result.map_age_sec <= limits_.map_timeout_sec;

  if (!valid_) {
    result.reason = "advisor was never configured";
    return result;
  }
  if (!map.valid()) {
    result.reason = "costmap is not valid";
    return result;
  }
  if (!isFinite(position)) {
    result.reason = "aircraft position is not finite";
    return result;
  }
  if (route_ahead.size() < 2) {
    result.reason = "route ahead has fewer than two points";
    return result;
  }
  for (const Vec3 & point : route_ahead) {
    if (!isFinite(point)) {
      result.reason = "route ahead contains a non-finite point";
      return result;
    }
  }
  if (!result.map_fresh) {
    result.reason = std::isfinite(result.map_age_sec) ?
      "obstacle map is stale at " + seconds(result.map_age_sec) :
      "no obstacle input has ever arrived";
    return result;
  }

  double travelled = 0.0;
  double clearance = limits_.clearance_scan_m;
  bool hit = false;
  Vec3 obstacle;

  for (std::size_t leg = 1; leg < route_ahead.size() && !hit; ++leg) {
    const Vec3 & from = route_ahead[leg - 1];
    const Vec3 & to = route_ahead[leg];
    const double length = distance3(from, to);
    if (!(length > 0.0)) {
      continue;
    }
    const int steps = static_cast<int>(std::ceil(length / limits_.sample_step_m));

    for (int step = 1; step <= steps; ++step) {
      const double along = std::min(
        length, static_cast<double>(step) * limits_.sample_step_m);
      const Vec3 sample = lerp(from, to, along / length);

      if (travelled + along > limits_.horizon_m) {
        result.checked_horizon_m = limits_.horizon_m;
        result.clearance_m = clearance;
        result.advice = Advice::Clear;
        result.reason = "clear to the " + metres(limits_.horizon_m) + " horizon";
        return result;
      }
      if (map.costAt(sample) == kCostUnknown) {
        result.checked_horizon_m = travelled + along;
        result.clearance_m = clearance;
        result.advice = Advice::Clear;
        result.reason = "clear to the map edge at " + metres(result.checked_horizon_m);
        return result;
      }

      clearance = std::min(clearance, clearanceAt(map, sample));
      if (blocked(map, sample)) {
        hit = true;
        obstacle = sample;
        result.checked_horizon_m = travelled + along;
        break;
      }
    }
    travelled += length;
  }

  result.clearance_m = clearance;

  if (!hit) {
    result.checked_horizon_m = std::min(travelled, limits_.horizon_m);
    result.advice = Advice::Clear;
    result.reason = "whole route ahead is clear over " + metres(result.checked_horizon_m);
    return result;
  }

  Vec3 escape;
  std::string escape_reason;
  if (findEscape(map, obstacle, position, route_ahead.back(), escape, escape_reason)) {
    result.advice = Advice::Escape;
    result.escape_point = escape;
    result.reason = "obstacle at " + metres(result.checked_horizon_m) + " ahead, escaping";
    return result;
  }

  result.advice = Advice::Hold;
  result.escape_point = Vec3{kNaN, kNaN, kNaN};
  result.reason = "blocked at " + metres(result.checked_horizon_m) + ": " + escape_reason;
  return result;
}

}  // namespace uav_navigation
