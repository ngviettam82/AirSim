#ifndef UAV_NAVIGATION__ROUTE_PLANNER_HPP_
#define UAV_NAVIGATION__ROUTE_PLANNER_HPP_

#include <string>
#include <vector>

#include "uav_navigation/costmap.hpp"
#include "uav_navigation/trajectory.hpp"

namespace uav_navigation
{

struct RouteLimits
{
  /// Multiplies exp(occupancy / 40) - 1, the soft penalty that buys wide corridors.
  double soft_cost_gain = 1.0;

  /// A goal nearer than this is already reached, so no route is planned.
  double goal_tolerance_m = 0.3;

  /// A sub-goal is never placed nearer than this to the window edge.
  double window_margin_m = 0.5;

  /// A* gives up past this many expansions instead of stalling the stream tick.
  unsigned max_expansions = 120000;

  /// How long the planner may keep failing to produce a route before the caller
  /// must hold. Covers every failure, not just projection: a blocked corridor and
  /// an unprojectable goal leave the aircraft equally routeless.
  double projection_timeout_sec = 2.0;

  /// Straight-line shortcut sampling; half a cell never steps over a lethal cell.
  double shortcut_sample_ratio = 0.5;
};

enum class RouteStatus
{
  Ok,
  NoMap,
  AlreadyThere,
  StartBlocked,
  GoalUnreachable,
  ProjectionFailed,
  Hold,
};

struct RouteResult
{
  RouteStatus status = RouteStatus::NoMap;

  /// World waypoints from start to sub-goal. All carry the band-centre altitude:
  /// obstacles were only ever checked in that slice, so the vertical profile is
  /// the caller's to own (plan P6 decision 3, 2.5D).
  std::vector<Vec3> waypoints;

  Vec3 projected_goal;
  bool goal_was_projected = false;
  unsigned expansions = 0;
  std::string reason;

  bool usable() const {return status == RouteStatus::Ok;}
};

/// Worst cell the SHIPPED trajectory crosses, sampled on the grid that is actually
/// streamed. The route is planned on waypoints, but the aircraft flies the spline
/// between them -- clearance proven on the waypoints is not clearance flown.
uint8_t worstCostAlongTrajectory(const Costmap & map, const Trajectory & trajectory);

/// Subdivides the route until the spline through it clears the map, or gives up.
///
/// Measured 2026-08-18: the spline cuts 0.44-0.59 m off an A* corner. In open ground
/// the soft cost leaves a 2 m cushion and that cut is harmless, but a forced corridor
/// has no cushion, and paying a fixed budget into inflation would instead close every
/// gap under ~1.9 m. Halving the segments halves the cut, so this buys the guarantee
/// without spending corridor width. reason names why it stopped.
std::vector<Vec3> tightenForSpline(
  const Costmap & map, const std::vector<Vec3> & waypoints,
  const TrajectoryLimits & limits, unsigned max_rounds, std::string & reason);

/// Global route across the rolling window: A* on the cost field, then a straight-line
/// shortcut pass, then a receding-horizon projection for goals beyond the window.
///
/// Two rules carry the safety weight:
///   1. A cell the airframe cannot fit through is NOT a cell with a high price. It is
///      refused outright, and so is a diagonal that squeezes between two of them.
///   2. Projection that keeps failing is not retried forever. Past projection_timeout
///      the answer becomes Hold, because a planner with no reachable sub-goal that
///      keeps flying the last one is flying blind.
class RoutePlanner
{
public:
  RoutePlanner() = default;
  explicit RoutePlanner(const RouteLimits & limits)
  : limits_(limits) {}

  RouteResult plan(
    const Costmap & map, const Vec3 & start, const Vec3 & goal, double now_seconds);

  /// True once the planner has been failing longer than the timeout.
  bool holding() const {return holding_;}

  /// Seconds the planner has been routeless; zero while it succeeds.
  double routeFailureAge() const {return failure_age_;}

  const RouteLimits & limits() const {return limits_;}

  /// Cost a planner pays to enter this cell, on top of the metres travelled.
  /// Infinite when the airframe cannot fit, which is not a price but a refusal.
  double cellPenalty(uint8_t cost) const;

private:
  /// Starts or extends the routeless clock and escalates to Hold past the timeout.
  RouteResult noteFailure(RouteStatus status, const std::string & reason, double now_seconds);
  void noteSuccess();

  RouteLimits limits_;
  bool holding_ = false;
  bool failing_ = false;
  double failure_start_ = 0.0;
  double failure_age_ = 0.0;
};

}  // namespace uav_navigation

#endif  // UAV_NAVIGATION__ROUTE_PLANNER_HPP_
