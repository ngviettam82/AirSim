#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "uav_navigation/route_planner.hpp"

using uav_navigation::Costmap;
using uav_navigation::CostmapLimits;
using uav_navigation::CostmapObstacle;
using uav_navigation::horizontalDistance;
using uav_navigation::kCostInscribed;
using uav_navigation::RouteLimits;
using uav_navigation::RoutePlanner;
using uav_navigation::RouteResult;
using uav_navigation::RouteStatus;
using uav_navigation::Vec3;

namespace
{

CostmapObstacle box(double x, double y, double size_x, double size_y, double stamp = 100.0)
{
  CostmapObstacle obstacle;
  obstacle.center = Vec3{x, y, 0.0};
  obstacle.size = Vec3{size_x, size_y, 2.0};
  obstacle.position_uncertainty = 0.0;
  obstacle.stamp_seconds = stamp;
  return obstacle;
}

Costmap mapWith(
  const std::vector<CostmapObstacle> & obstacles, const CostmapLimits & limits = CostmapLimits())
{
  std::string reason;
  Costmap map = Costmap::build(limits, reason);
  EXPECT_TRUE(map.valid()) << reason;
  map.update(obstacles, Vec3{0.0, 0.0, 0.0}, 100.0, 100.0, reason);
  return map;
}

/// Walks the polyline the aircraft would actually fly, not just its corners.
double worstCostAlong(const Costmap & map, const std::vector<Vec3> & waypoints)
{
  double worst = 0.0;
  for (std::size_t leg = 0; leg + 1 < waypoints.size(); ++leg) {
    const Vec3 & from = waypoints[leg];
    const Vec3 & to = waypoints[leg + 1];
    const double span = horizontalDistance(from, to);
    const int steps = std::max(1, static_cast<int>(std::ceil(span / 0.02)));
    for (int index = 0; index <= steps; ++index) {
      const double ratio = static_cast<double>(index) / static_cast<double>(steps);
      const Vec3 point{
        from.x + (to.x - from.x) * ratio, from.y + (to.y - from.y) * ratio, from.z};
      worst = std::max(worst, static_cast<double>(map.costAt(point)));
    }
  }
  return worst;
}

double pathLength(const std::vector<Vec3> & waypoints)
{
  double total = 0.0;
  for (std::size_t leg = 0; leg + 1 < waypoints.size(); ++leg) {
    total += horizontalDistance(waypoints[leg], waypoints[leg + 1]);
  }
  return total;
}

double closestApproach(const std::vector<Vec3> & waypoints, const Vec3 & point)
{
  double nearest = std::numeric_limits<double>::infinity();
  for (std::size_t leg = 0; leg + 1 < waypoints.size(); ++leg) {
    const Vec3 & from = waypoints[leg];
    const Vec3 & to = waypoints[leg + 1];
    const double span = horizontalDistance(from, to);
    const int steps = std::max(1, static_cast<int>(std::ceil(span / 0.02)));
    for (int index = 0; index <= steps; ++index) {
      const double ratio = static_cast<double>(index) / static_cast<double>(steps);
      const Vec3 sample{
        from.x + (to.x - from.x) * ratio, from.y + (to.y - from.y) * ratio, from.z};
      nearest = std::min(nearest, horizontalDistance(sample, point));
    }
  }
  return nearest;
}

/// One cell wide at the given centre, so a wall can be built cell by cell.
CostmapObstacle cellBlock(double x, double y)
{
  return box(x, y, 0.2, 0.2);
}

CostmapLimits sharpLimits()
{
  CostmapLimits limits;
  limits.width_m = 10.0;
  limits.drone_radius_m = 0.0;   // one obstacle, one cell: the grid rule alone is on trial
  limits.corner_cut_m = 0.0;     // ditto: no budget may widen the cell under test
  limits.cost_scaling = 50.0;    // soft ring narrower than half a cell
  return limits;
}

}  // namespace

TEST(RoutePlanner, AClearWindowGivesTheStraightLine)
{
  const Costmap map = mapWith({});
  RoutePlanner planner;
  const RouteResult route = planner.plan(map, Vec3{-5.0, 0.0, 0.0}, Vec3{5.0, 0.0, 0.0}, 100.0);

  ASSERT_TRUE(route.usable()) << route.reason;
  EXPECT_EQ(route.waypoints.size(), 2u) << "the shortcut pass should collapse a clear run";
  EXPECT_NEAR(pathLength(route.waypoints), 10.0, 0.5);
  EXPECT_FALSE(route.goal_was_projected);
}

TEST(RoutePlanner, TheRouteNeverEntersACellTheAirframeCannotFit)
{
  const Costmap map = mapWith({box(0.0, 0.0, 2.0, 2.0)});
  RoutePlanner planner;
  const RouteResult route = planner.plan(map, Vec3{-5.0, 0.0, 0.0}, Vec3{5.0, 0.0, 0.0}, 100.0);

  ASSERT_TRUE(route.usable()) << route.reason;
  // G-N3: measured on the flown polyline, not on the corners the planner emitted.
  EXPECT_LT(worstCostAlong(map, route.waypoints), static_cast<double>(kCostInscribed));

  // Positive control: the straight line the planner refused does breach it.
  const std::vector<Vec3> naive{Vec3{-5.0, 0.0, 0.0}, Vec3{5.0, 0.0, 0.0}};
  EXPECT_GE(worstCostAlong(map, naive), static_cast<double>(kCostInscribed));
}

TEST(RoutePlanner, TheRouteKeepsTheAirframeRadiusClearOfTheObstacleSurface)
{
  const Costmap map = mapWith({box(0.0, 0.0, 2.0, 2.0)});
  RoutePlanner planner;
  const RouteResult route = planner.plan(map, Vec3{-5.0, 0.0, 0.0}, Vec3{5.0, 0.0, 0.0}, 100.0);
  ASSERT_TRUE(route.usable()) << route.reason;

  // Nearest point of a 2 m box to any path sample is at least half-width + radius,
  // minus one cell of quantisation.
  const double nearest = closestApproach(route.waypoints, Vec3{0.0, 0.0, 0.0});
  EXPECT_GT(nearest, 1.0 + 0.35 - map.resolution());
}

TEST(RoutePlanner, ARouteAroundAnObstacleCostsMoreThanTheBlockedStraightLine)
{
  const Costmap map = mapWith({box(0.0, 0.0, 2.0, 2.0)});
  RoutePlanner planner;
  const RouteResult route = planner.plan(map, Vec3{-5.0, 0.0, 0.0}, Vec3{5.0, 0.0, 0.0}, 100.0);

  ASSERT_TRUE(route.usable()) << route.reason;
  EXPECT_GT(pathLength(route.waypoints), 10.0) << "a detour that costs nothing is not a detour";
  EXPECT_LT(pathLength(route.waypoints), 20.0) << "and it should not wander";
  EXPECT_GE(route.waypoints.size(), 3u);
}

TEST(RoutePlanner, TheSoftCostBuysDistanceFromTheObstacle)
{
  const Costmap map = mapWith({box(0.0, 2.0, 2.0, 2.0)});

  RouteLimits hugging;
  hugging.soft_cost_gain = 0.0;
  RoutePlanner hugger(hugging);
  const RouteResult tight =
    hugger.plan(map, Vec3{-5.0, 0.0, 0.0}, Vec3{5.0, 0.0, 0.0}, 100.0);

  RoutePlanner cautious;  // shipped gain 1.0
  const RouteResult wide =
    cautious.plan(map, Vec3{-5.0, 0.0, 0.0}, Vec3{5.0, 0.0, 0.0}, 100.0);

  ASSERT_TRUE(tight.usable()) << tight.reason;
  ASSERT_TRUE(wide.usable()) << wide.reason;

  const Vec3 obstacle{0.0, 2.0, 0.0};
  EXPECT_GT(closestApproach(wide.waypoints, obstacle), closestApproach(tight.waypoints, obstacle))
    << "the exponential penalty is what buys a wide corridor";
}

TEST(RoutePlanner, ADiagonalMayNotSqueezeBetweenTwoBlockedCells)
{
  const CostmapLimits limits = sharpLimits();
  std::string reason;
  Costmap ruler = Costmap::build(limits, reason);
  ASSERT_TRUE(ruler.valid()) << reason;
  ruler.update({}, Vec3{0.0, 0.0, 0.0}, 100.0, 100.0, reason);
  const int side = static_cast<int>(ruler.cellsPerSide());

  // Two walls one cell apart, each spanning the whole window so there is no way round.
  const int wall_a = side / 2;
  const int wall_b = wall_a + 1;
  const int gap_row = side / 2;

  std::vector<CostmapObstacle> staggered;
  std::vector<CostmapObstacle> aligned;
  for (int row = 0; row < side; ++row) {
    const Vec3 at_a = ruler.cellToWorld(wall_a, row);
    const Vec3 at_b = ruler.cellToWorld(wall_b, row);
    if (row != gap_row) {
      staggered.push_back(cellBlock(at_a.x, at_a.y));
      aligned.push_back(cellBlock(at_a.x, at_a.y));
      aligned.push_back(cellBlock(at_b.x, at_b.y));
    }
    if (row != gap_row + 1) {
      staggered.push_back(cellBlock(at_b.x, at_b.y));
    }
  }

  const Vec3 start = ruler.cellToWorld(wall_a - 6, gap_row);
  const Vec3 goal = ruler.cellToWorld(wall_b + 6, gap_row);

  RoutePlanner planner;
  const Costmap blocked_map = mapWith(staggered, limits);
  const RouteResult squeezed = planner.plan(blocked_map, start, goal, 100.0);
  EXPECT_FALSE(squeezed.usable())
    << "the only opening is a diagonal between two blocked cells";

  // Positive control: line the two gaps up and the same wall becomes passable.
  RoutePlanner second;
  const Costmap open_map = mapWith(aligned, limits);
  const RouteResult through = second.plan(open_map, start, goal, 100.0);
  EXPECT_TRUE(through.usable()) << through.reason;
}

TEST(RoutePlanner, AGoalBeyondTheWindowIsPulledBackIntoIt)
{
  const Costmap map = mapWith({});
  RoutePlanner planner;
  const RouteResult route = planner.plan(map, Vec3{0.0, 0.0, 0.0}, Vec3{80.0, 0.0, 0.0}, 100.0);

  ASSERT_TRUE(route.usable()) << route.reason;
  EXPECT_TRUE(route.goal_was_projected);

  int cell_x = 0;
  int cell_y = 0;
  EXPECT_TRUE(map.worldToCell(route.projected_goal, cell_x, cell_y))
    << "the sub-goal must sit inside the window it was planned in";
  EXPECT_GT(route.projected_goal.x, 0.0) << "and it must lead toward the goal";
  EXPECT_LT(route.projected_goal.x, 80.0);
}

TEST(RoutePlanner, AWalledOffGoalIsUnreachableAndThenBecomesHold)
{
  // A wall across the whole window: no corridor exists at any heading.
  const Costmap map = mapWith({box(2.0, 0.0, 0.5, 40.0)});
  RouteLimits limits;
  limits.projection_timeout_sec = 2.0;
  RoutePlanner planner;
  RoutePlanner timed(limits);

  const RouteResult first = timed.plan(map, Vec3{0.0, 0.0, 0.0}, Vec3{8.0, 0.0, 0.0}, 100.0);
  EXPECT_EQ(first.status, RouteStatus::GoalUnreachable) << first.reason;
  EXPECT_FALSE(timed.holding());

  const RouteResult soon = timed.plan(map, Vec3{0.0, 0.0, 0.0}, Vec3{8.0, 0.0, 0.0}, 101.0);
  EXPECT_EQ(soon.status, RouteStatus::GoalUnreachable);
  EXPECT_FALSE(timed.holding()) << "one second is inside the 2 s budget";

  const RouteResult late = timed.plan(map, Vec3{0.0, 0.0, 0.0}, Vec3{8.0, 0.0, 0.0}, 102.5);
  EXPECT_EQ(late.status, RouteStatus::Hold);
  EXPECT_TRUE(timed.holding());
  EXPECT_NEAR(timed.routeFailureAge(), 2.5, 1e-9);
  EXPECT_NE(late.reason.find("routeless"), std::string::npos);
}

TEST(RoutePlanner, OneGoodRouteClearsTheRoutelessClock)
{
  const Costmap walled = mapWith({box(2.0, 0.0, 0.5, 40.0)});
  const Costmap clear = mapWith({});
  RoutePlanner planner;

  planner.plan(walled, Vec3{0.0, 0.0, 0.0}, Vec3{8.0, 0.0, 0.0}, 100.0);
  planner.plan(walled, Vec3{0.0, 0.0, 0.0}, Vec3{8.0, 0.0, 0.0}, 103.0);
  ASSERT_TRUE(planner.holding());

  const RouteResult recovered =
    planner.plan(clear, Vec3{0.0, 0.0, 0.0}, Vec3{8.0, 0.0, 0.0}, 103.5);
  ASSERT_TRUE(recovered.usable()) << recovered.reason;
  EXPECT_FALSE(planner.holding());
  EXPECT_DOUBLE_EQ(planner.routeFailureAge(), 0.0);

  // Positive control: the clock restarts from the next failure, not from the old one.
  const RouteResult again = planner.plan(walled, Vec3{0.0, 0.0, 0.0}, Vec3{8.0, 0.0, 0.0}, 104.0);
  EXPECT_EQ(again.status, RouteStatus::GoalUnreachable);
  EXPECT_FALSE(planner.holding());
}

TEST(RoutePlanner, AGoalAlreadyUnderTheAircraftIsNotPlanned)
{
  const Costmap map = mapWith({});
  RoutePlanner planner;
  const RouteResult route = planner.plan(map, Vec3{1.0, 1.0, 0.0}, Vec3{1.1, 1.1, 0.0}, 100.0);

  EXPECT_EQ(route.status, RouteStatus::AlreadyThere);
  EXPECT_EQ(route.expansions, 0u);
  EXPECT_FALSE(planner.holding());
}

TEST(RoutePlanner, AnAircraftInsideTheKeepOutRingSaysSoInsteadOfPlanning)
{
  const Costmap map = mapWith({box(0.0, 0.0, 2.0, 2.0)});
  RoutePlanner planner;
  const RouteResult route = planner.plan(map, Vec3{0.0, 0.0, 0.0}, Vec3{5.0, 0.0, 0.0}, 100.0);

  EXPECT_EQ(route.status, RouteStatus::StartBlocked);
  EXPECT_TRUE(route.waypoints.empty());
}

TEST(RoutePlanner, AnUnbuiltMapOrANonFiniteRequestPlansNothing)
{
  const Costmap nothing;
  RoutePlanner planner;
  EXPECT_EQ(
    planner.plan(nothing, Vec3{0.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}, 100.0).status,
    RouteStatus::NoMap);

  const Costmap map = mapWith({});
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    planner.plan(map, Vec3{nan, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}, 100.0).status,
    RouteStatus::NoMap);
  EXPECT_EQ(
    planner.plan(map, Vec3{0.0, 0.0, 0.0}, Vec3{nan, 0.0, 0.0}, 100.0).status,
    RouteStatus::NoMap);
}

TEST(RoutePlanner, TheSamePictureAlwaysGivesTheSameRoute)
{
  const Costmap map = mapWith({box(0.0, 0.0, 2.0, 2.0), box(1.0, -3.0, 1.0, 1.0)});
  RoutePlanner first;
  RoutePlanner second;
  const RouteResult left = first.plan(map, Vec3{-5.0, 0.0, 0.0}, Vec3{5.0, 0.0, 0.0}, 100.0);
  const RouteResult right = second.plan(map, Vec3{-5.0, 0.0, 0.0}, Vec3{5.0, 0.0, 0.0}, 100.0);

  ASSERT_TRUE(left.usable());
  ASSERT_EQ(left.waypoints.size(), right.waypoints.size());
  for (std::size_t index = 0; index < left.waypoints.size(); ++index) {
    EXPECT_DOUBLE_EQ(left.waypoints[index].x, right.waypoints[index].x);
    EXPECT_DOUBLE_EQ(left.waypoints[index].y, right.waypoints[index].y);
  }
}

TEST(RoutePlanner, EveryWaypointCarriesTheBandAltitudeTheObstaclesWereCheckedIn)
{
  std::string reason;
  Costmap map = Costmap::build(CostmapLimits(), reason);
  map.update({box(0.0, 0.0, 2.0, 2.0)}, Vec3{0.0, 0.0, 4.0}, 100.0, 100.0, reason);

  RoutePlanner planner;
  const RouteResult route = planner.plan(map, Vec3{-5.0, 0.0, 4.0}, Vec3{5.0, 0.0, 9.0}, 100.0);
  ASSERT_TRUE(route.usable()) << route.reason;
  for (const Vec3 & waypoint : route.waypoints) {
    EXPECT_DOUBLE_EQ(waypoint.z, 4.0) << "the vertical profile is the caller's, not the map's";
  }
}

TEST(RoutePlanner, CellPenaltyRefusesWhatItCannotPrice)
{
  RoutePlanner planner;
  EXPECT_DOUBLE_EQ(planner.cellPenalty(0), 0.0);
  EXPECT_TRUE(std::isinf(planner.cellPenalty(kCostInscribed)));
  EXPECT_TRUE(std::isinf(planner.cellPenalty(254)));
  EXPECT_TRUE(std::isinf(planner.cellPenalty(255)));
  EXPECT_GT(planner.cellPenalty(200), planner.cellPenalty(100));
  EXPECT_GT(planner.cellPenalty(100), planner.cellPenalty(10));
}

namespace
{
/// A 1.4 m gap with a dog-leg: the soft cost has no room left to leave a cushion,
/// which is the only geometry where the corner the spline cuts actually matters.
std::vector<CostmapObstacle> narrowDogLeg()
{
  return {box(0.0, 3.7, 0.5, 6.0), box(0.0, -3.7, 0.5, 6.0)};
}

uav_navigation::Trajectory splineThrough(const std::vector<Vec3> & waypoints)
{
  std::string reason;
  return uav_navigation::Trajectory::build(
    waypoints, 0.0, uav_navigation::TrajectoryLimits(), reason);
}
}  // namespace

TEST(RoutePlanner, InOpenGroundTheFlownSplineNeverReachesTheRing)
{
  // Four layouts, because one corner proves nothing about the next.
  const std::vector<std::vector<CostmapObstacle>> layouts{
    {box(0.0, 0.0, 2.0, 2.0)},
    {box(0.0, 0.0, 2.0, 2.0), box(2.0, 2.0, 2.0, 2.0)},
    {box(0.0, 0.0, 2.0, 2.0), box(-2.0, 2.5, 3.0, 1.0)},
    {box(0.0, 1.5, 4.0, 1.0)},
  };
  const std::vector<std::pair<Vec3, Vec3>> trips{
    {Vec3{-5.0, 0.0, 0.0}, Vec3{5.0, 0.0, 0.0}},
    {Vec3{-5.0, -1.0, 0.0}, Vec3{5.0, 3.0, 0.0}},
    {Vec3{-6.0, 0.0, 0.0}, Vec3{6.0, 1.0, 0.0}},
    {Vec3{-5.0, 0.0, 0.0}, Vec3{5.0, 4.0, 0.0}},
  };

  for (std::size_t index = 0; index < layouts.size(); ++index) {
    const Costmap map = mapWith(layouts[index]);
    RoutePlanner planner;
    const RouteResult route =
      planner.plan(map, trips[index].first, trips[index].second, 100.0);
    ASSERT_TRUE(route.usable()) << index << ": " << route.reason;

    const uav_navigation::Trajectory flown = splineThrough(route.waypoints);
    ASSERT_TRUE(flown.valid()) << index;
    EXPECT_LT(
      uav_navigation::worstCostAlongTrajectory(map, flown),
      static_cast<int>(kCostInscribed)) << "layout " << index;
  }
}

TEST(RoutePlanner, AForcedCorridorIsWhereTheCornerCutActuallyBites)
{
  // Positive control for the tightening below, and the reason it exists at all:
  // clearance proven on the waypoints is NOT the clearance flown.
  const Costmap map = mapWith(narrowDogLeg());
  RoutePlanner planner;
  const RouteResult route = planner.plan(map, Vec3{-4.0, 3.0, 0.0}, Vec3{4.0, -3.0, 0.0}, 100.0);
  ASSERT_TRUE(route.usable()) << route.reason;

  EXPECT_LT(worstCostAlong(map, route.waypoints), static_cast<double>(kCostInscribed))
    << "the waypoints themselves are clear, which is exactly what makes this trap quiet";

  const uav_navigation::Trajectory raw = splineThrough(route.waypoints);
  ASSERT_TRUE(raw.valid());
  EXPECT_GE(
    uav_navigation::worstCostAlongTrajectory(map, raw), static_cast<int>(kCostInscribed))
    << "if this passes, the geometry no longer exercises the corner cut";
}

TEST(RoutePlanner, TighteningPullsTheSplineBackOutOfTheRing)
{
  const Costmap map = mapWith(narrowDogLeg());
  RoutePlanner planner;
  const RouteResult route = planner.plan(map, Vec3{-4.0, 3.0, 0.0}, Vec3{4.0, -3.0, 0.0}, 100.0);
  ASSERT_TRUE(route.usable()) << route.reason;

  std::string reason;
  const std::vector<Vec3> tightened = uav_navigation::tightenForSpline(
    map, route.waypoints, uav_navigation::TrajectoryLimits(), 4, reason);
  EXPECT_TRUE(reason.empty()) << reason;
  EXPECT_GT(tightened.size(), route.waypoints.size()) << "it had to subdivide to get there";

  const uav_navigation::Trajectory flown = splineThrough(tightened);
  ASSERT_TRUE(flown.valid());
  EXPECT_LT(
    uav_navigation::worstCostAlongTrajectory(map, flown), static_cast<int>(kCostInscribed));
}

TEST(RoutePlanner, TighteningLeavesAnAlreadyClearRouteAlone)
{
  const Costmap map = mapWith({box(0.0, 0.0, 2.0, 2.0)});
  RoutePlanner planner;
  const RouteResult route = planner.plan(map, Vec3{-5.0, 0.0, 0.0}, Vec3{5.0, 0.0, 0.0}, 100.0);
  ASSERT_TRUE(route.usable()) << route.reason;

  std::string reason;
  const std::vector<Vec3> tightened = uav_navigation::tightenForSpline(
    map, route.waypoints, uav_navigation::TrajectoryLimits(), 4, reason);
  EXPECT_TRUE(reason.empty()) << reason;
  EXPECT_EQ(tightened.size(), route.waypoints.size())
    << "subdividing a clear route only costs stream bandwidth";
}
