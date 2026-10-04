#include "uav_navigation/route_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace uav_navigation
{

namespace
{

/// Nav2 cost maps onto the 0..100 occupancy scale the penalty curve was tuned on.
constexpr double kOccupancyScale = 100.0 / 254.0;
constexpr double kPenaltyDivisor = 40.0;

/// Fan of headings tried when the direct ray to the goal leaves no room.
constexpr double kProjectionFanDegrees[] = {0.0, 15.0, -15.0, 30.0, -30.0, 45.0, -45.0};

struct Node
{
  double f = 0.0;
  unsigned long order = 0;
  int index = 0;
};

struct NodeGreater
{
  bool operator()(const Node & left, const Node & right) const
  {
    // Order breaks ties so the same map always yields the same route.
    if (left.f != right.f) {
      return left.f > right.f;
    }
    return left.order > right.order;
  }
};

/// Unknown sits above inscribed, so a segment leaving the window is blocked too.
bool blocked(uint8_t cost)
{
  return cost >= kCostInscribed;
}

/// Worst cell the straight segment crosses; kCostUnknown if it leaves the window.
uint8_t worstAlongSegment(
  const Costmap & map, const Vec3 & from, const Vec3 & to, double sample_step)
{
  const double span = horizontalDistance(from, to);
  const int steps = span > 0.0 ? static_cast<int>(std::ceil(span / sample_step)) : 0;
  uint8_t worst = map.costAt(from);
  for (int index = 1; index <= steps; ++index) {
    const double ratio = static_cast<double>(index) / static_cast<double>(steps);
    const Vec3 point{
      from.x + (to.x - from.x) * ratio,
      from.y + (to.y - from.y) * ratio,
      from.z};
    worst = std::max(worst, map.costAt(point));
  }
  return worst;
}

/// Greedy line-of-sight reduction that may not spend the clearance A* just bought.
///
/// A shortcut is taken only when the straight segment is no worse than the grid
/// path it replaces. Checking merely "is it blocked" would straighten the bow the
/// soft cost put around an obstacle, quietly handing back the corridor width.
std::vector<Vec3> shortcutPath(
  const Costmap & map, const std::vector<Vec3> & raw, double sample_step)
{
  std::vector<Vec3> kept;
  if (raw.empty()) {
    return kept;
  }
  kept.push_back(raw.front());

  std::size_t anchor = 0;
  while (anchor + 1 < raw.size()) {
    std::size_t furthest = anchor + 1;
    for (std::size_t probe = raw.size() - 1; probe > anchor; --probe) {
      uint8_t detour_worst = 0;
      for (std::size_t step = anchor; step < probe; ++step) {
        detour_worst = std::max(
          detour_worst, worstAlongSegment(map, raw[step], raw[step + 1], sample_step));
      }
      const uint8_t straight_worst = worstAlongSegment(map, raw[anchor], raw[probe], sample_step);
      if (!blocked(straight_worst) && straight_worst <= detour_worst) {
        furthest = probe;
        break;
      }
    }
    kept.push_back(raw[furthest]);
    anchor = furthest;
  }
  return kept;
}

}  // namespace

uint8_t worstCostAlongTrajectory(const Costmap & map, const Trajectory & trajectory)
{
  uint8_t worst = kCostFree;
  for (const TrajectorySample & sample : trajectory.grid()) {
    worst = std::max(worst, map.costAt(sample.position));
  }
  return worst;
}

std::vector<Vec3> tightenForSpline(
  const Costmap & map, const std::vector<Vec3> & waypoints,
  const TrajectoryLimits & limits, unsigned max_rounds, std::string & reason)
{
  reason.clear();
  std::vector<Vec3> current = waypoints;
  if (current.size() < 2) {
    reason = "a route of fewer than two points has no corner to cut";
    return current;
  }

  for (unsigned round = 0; round <= max_rounds; ++round) {
    std::string build_reason;
    const Trajectory candidate = Trajectory::build(current, 0.0, limits, build_reason);
    if (!candidate.valid()) {
      reason = "trajectory refused the route: " + build_reason;
      return current;
    }
    if (!blocked(worstCostAlongTrajectory(map, candidate))) {
      return current;
    }
    if (round == max_rounds) {
      reason = "the spline still breaches the map after max_rounds subdivisions";
      return current;
    }

    std::vector<Vec3> denser;
    denser.reserve(current.size() * 2);
    for (std::size_t leg = 0; leg + 1 < current.size(); ++leg) {
      denser.push_back(current[leg]);
      denser.push_back(
        Vec3{
          0.5 * (current[leg].x + current[leg + 1].x),
          0.5 * (current[leg].y + current[leg + 1].y),
          0.5 * (current[leg].z + current[leg + 1].z)});
    }
    denser.push_back(current.back());
    current = denser;
  }
  return current;
}

double RoutePlanner::cellPenalty(uint8_t cost) const
{
  if (blocked(cost)) {
    return std::numeric_limits<double>::infinity();
  }
  const double occupancy = static_cast<double>(cost) * kOccupancyScale;
  return limits_.soft_cost_gain * (std::exp(occupancy / kPenaltyDivisor) - 1.0);
}

RouteResult RoutePlanner::noteFailure(
  RouteStatus status, const std::string & reason, double now_seconds)
{
  if (!failing_) {
    failing_ = true;
    failure_start_ = now_seconds;
  }
  failure_age_ = now_seconds - failure_start_;
  holding_ = failure_age_ >= limits_.projection_timeout_sec;

  RouteResult result;
  result.status = holding_ ? RouteStatus::Hold : status;
  result.reason = holding_ ? "routeless past projection_timeout_sec: " + reason : reason;
  return result;
}

void RoutePlanner::noteSuccess()
{
  failing_ = false;
  failure_age_ = 0.0;
  holding_ = false;
}

RouteResult RoutePlanner::plan(
  const Costmap & map, const Vec3 & start, const Vec3 & goal, double now_seconds)
{
  RouteResult result;
  result.projected_goal = goal;

  if (!map.valid()) {
    result.status = RouteStatus::NoMap;
    result.reason = "costmap was never built";
    return result;
  }
  if (!isFinite(start) || !isFinite(goal) || !std::isfinite(now_seconds)) {
    result.status = RouteStatus::NoMap;
    result.reason = "plan needs a finite start, goal and time";
    return result;
  }

  const double resolution = map.resolution();
  const int side = static_cast<int>(map.cellsPerSide());
  const double band_z = map.origin().z;

  int start_x = 0;
  int start_y = 0;
  if (!map.worldToCell(start, start_x, start_y)) {
    return noteFailure(
      RouteStatus::StartBlocked, "the aircraft is outside its own window", now_seconds);
  }
  if (blocked(map.cost(start_x, start_y))) {
    return noteFailure(
      RouteStatus::StartBlocked, "the aircraft already sits inside the keep-out ring",
      now_seconds);
  }

  if (horizontalDistance(start, goal) <= limits_.goal_tolerance_m) {
    result.status = RouteStatus::AlreadyThere;
    result.waypoints.push_back(Vec3{goal.x, goal.y, band_z});
    noteSuccess();
    return result;
  }

  // --- receding horizon: pull a far goal back into the window -----------------
  const Vec3 window_centre = map.cellToWorld(side / 2, side / 2);
  const double reach = 0.5 * static_cast<double>(side) * resolution - limits_.window_margin_m;

  Vec3 target{goal.x, goal.y, band_z};
  int goal_x = 0;
  int goal_y = 0;
  bool have_target = map.worldToCell(target, goal_x, goal_y) &&
    !blocked(map.cost(goal_x, goal_y)) &&
    horizontalDistance(window_centre, target) <= reach;

  if (!have_target) {
    const double heading = std::atan2(goal.y - start.y, goal.x - start.x);
    const double span = horizontalDistance(start, goal);
    double best_gap = std::numeric_limits<double>::infinity();

    for (const double degrees : kProjectionFanDegrees) {
      const double angle = heading + degrees * M_PI / 180.0;
      const double cos_a = std::cos(angle);
      const double sin_a = std::sin(angle);

      for (double range = std::min(span, 2.0 * reach); range > limits_.goal_tolerance_m;
        range -= resolution)
      {
        const Vec3 probe{start.x + cos_a * range, start.y + sin_a * range, band_z};
        if (horizontalDistance(window_centre, probe) > reach) {
          continue;
        }
        int probe_x = 0;
        int probe_y = 0;
        if (!map.worldToCell(probe, probe_x, probe_y) || blocked(map.cost(probe_x, probe_y))) {
          continue;
        }
        const double gap = horizontalDistance(probe, goal);
        if (gap < best_gap) {
          best_gap = gap;
          target = probe;
          goal_x = probe_x;
          goal_y = probe_y;
          have_target = true;
        }
        break;  // farthest reachable point on this ray is the best it offers
      }
    }
    result.goal_was_projected = have_target;
  }

  if (!have_target) {
    return noteFailure(
      RouteStatus::ProjectionFailed, "no reachable sub-goal toward the goal in this window",
      now_seconds);
  }
  result.projected_goal = target;

  // --- A* over the cost field -------------------------------------------------
  const std::size_t cells = static_cast<std::size_t>(side) * static_cast<std::size_t>(side);
  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> travelled(cells, infinity);
  std::vector<int> came_from(cells, -1);
  std::vector<bool> settled(cells, false);

  auto flatten = [side](int x, int y) {return static_cast<std::size_t>(y) * side + x;};
  auto heuristic = [&](int x, int y) {
      const double dx = static_cast<double>(x - goal_x);
      const double dy = static_cast<double>(y - goal_y);
      return std::hypot(dx, dy) * resolution;
    };

  std::priority_queue<Node, std::vector<Node>, NodeGreater> frontier;
  unsigned long order = 0;
  travelled[flatten(start_x, start_y)] = 0.0;
  frontier.push(Node{heuristic(start_x, start_y), order++,
      static_cast<int>(flatten(start_x, start_y))});

  bool reached = false;
  while (!frontier.empty()) {
    const Node node = frontier.top();
    frontier.pop();
    const std::size_t here = static_cast<std::size_t>(node.index);
    if (settled[here]) {
      continue;
    }
    settled[here] = true;
    ++result.expansions;

    const int here_x = static_cast<int>(here % static_cast<std::size_t>(side));
    const int here_y = static_cast<int>(here / static_cast<std::size_t>(side));
    if (here_x == goal_x && here_y == goal_y) {
      reached = true;
      break;
    }
    if (result.expansions >= limits_.max_expansions) {
      break;
    }

    for (int step_y = -1; step_y <= 1; ++step_y) {
      for (int step_x = -1; step_x <= 1; ++step_x) {
        if (step_x == 0 && step_y == 0) {
          continue;
        }
        const int next_x = here_x + step_x;
        const int next_y = here_y + step_y;
        if (!map.inside(next_x, next_y)) {
          continue;
        }
        const uint8_t cost = map.cost(next_x, next_y);
        if (blocked(cost)) {
          continue;
        }
        // A diagonal may not squeeze between two blocked cells: the airframe is
        // round, and the gap the grid seems to offer does not exist in the air.
        if (step_x != 0 && step_y != 0) {
          if (blocked(map.cost(here_x + step_x, here_y)) ||
            blocked(map.cost(here_x, here_y + step_y)))
          {
            continue;
          }
        }

        const double stride = (step_x != 0 && step_y != 0) ? std::sqrt(2.0) : 1.0;
        const double reach_cost = travelled[here] +
          stride * resolution + cellPenalty(cost) * resolution;
        const std::size_t next = flatten(next_x, next_y);
        if (reach_cost < travelled[next]) {
          travelled[next] = reach_cost;
          came_from[next] = static_cast<int>(here);
          frontier.push(Node{reach_cost + heuristic(next_x, next_y), order++,
              static_cast<int>(next)});
        }
      }
    }
  }

  if (!reached) {
    return noteFailure(
      RouteStatus::GoalUnreachable,
      result.expansions >= limits_.max_expansions ?
      "A* hit its expansion budget" : "no free corridor to the sub-goal",
      now_seconds);
  }

  // --- unwind, then shortcut --------------------------------------------------
  std::vector<Vec3> raw;
  for (int index = static_cast<int>(flatten(goal_x, goal_y)); index >= 0;
    index = came_from[static_cast<std::size_t>(index)])
  {
    const int x = index % side;
    const int y = index / side;
    raw.push_back(map.cellToWorld(x, y));
    if (x == start_x && y == start_y) {
      break;
    }
  }
  std::reverse(raw.begin(), raw.end());

  result.waypoints = shortcutPath(map, raw, resolution * limits_.shortcut_sample_ratio);
  result.status = RouteStatus::Ok;
  noteSuccess();
  return result;
}

}  // namespace uav_navigation
