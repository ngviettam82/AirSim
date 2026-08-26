#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "uav_navigation/local_avoidance.hpp"

using uav_navigation::Advice;
using uav_navigation::AvoidanceLimits;
using uav_navigation::AvoidanceResult;
using uav_navigation::Costmap;
using uav_navigation::CostmapLimits;
using uav_navigation::CostmapObstacle;
using uav_navigation::LocalAvoidance;
using uav_navigation::Vec3;

namespace
{

constexpr double kNow = 100.0;

LocalAvoidance advisorWith(const AvoidanceLimits & limits = AvoidanceLimits())
{
  std::string reason;
  LocalAvoidance advisor = LocalAvoidance::build(limits, reason);
  EXPECT_TRUE(advisor.valid()) << reason;
  return advisor;
}

/// No obstacle report has ever arrived, as opposed to one that carried nothing.
constexpr double kNoReport = -std::numeric_limits<double>::infinity();

CostmapObstacle boxAt(double x, double y, double z, double side)
{
  CostmapObstacle obstacle;
  obstacle.center = Vec3{x, y, z};
  obstacle.size = Vec3{side, side, side};
  obstacle.position_uncertainty = 0.0;
  obstacle.stamp_seconds = kNow;
  return obstacle;
}

/// Map centred on the aircraft, holding whatever obstacles the case needs. The report
/// is always fresh, mapWith({}) included -- an empty report means the sensor looked
/// and saw clear, which is what starvedMap() below is NOT.
Costmap mapWith(
  const std::vector<CostmapObstacle> & obstacles, const Vec3 & center = Vec3{0.0, 0.0, 2.0})
{
  std::string reason;
  Costmap map = Costmap::build(CostmapLimits(), reason);
  EXPECT_TRUE(map.valid()) << reason;
  map.update(obstacles, center, kNow, kNow, reason);
  return map;
}

/// A map that has never been fed, so its input age is infinite.
Costmap starvedMap()
{
  std::string reason;
  Costmap map = Costmap::build(CostmapLimits(), reason);
  EXPECT_TRUE(map.valid()) << reason;
  return map;
}

std::vector<Vec3> straightRoute(double from_x, double to_x, double y, double z)
{
  return {Vec3{from_x, y, z}, Vec3{to_x, y, z}};
}

}  // namespace

// ===========================================================================
// S3 coverage, 2026-08-26: the advisor's REFUSAL arms. Every case below was
// reachable only by flying into the condition, so the reason strings the rest
// of the stack logs and gates on had never been checked against a known input.
// Each one must Hold -- an advisor that cannot see must never wave the aircraft
// through (same rule as AnUnconfiguredAdvisorHolds...).
// ===========================================================================

TEST(LocalAvoidance, AnInvalidCostmapHoldsAndSaysSo)
{
  const LocalAvoidance advisor = advisorWith();
  const Costmap unbuilt;                       // never built: valid() is false
  const AvoidanceResult result =
    advisor.advise(unbuilt, straightRoute(0.0, 5.0, 0.0, 2.0), Vec3{0.0, 0.0, 2.0});

  EXPECT_EQ(result.advice, Advice::Hold);
  EXPECT_NE(result.reason.find("costmap is not valid"), std::string::npos) << result.reason;
}

TEST(LocalAvoidance, ANonFinitePositionHoldsInsteadOfPlanningFromIt)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const LocalAvoidance advisor = advisorWith();
  const Costmap map = mapWith({});

  for (const Vec3 & broken :
    {Vec3{nan, 0.0, 2.0}, Vec3{0.0, nan, 2.0}, Vec3{0.0, 0.0, nan}})
  {
    const AvoidanceResult result =
      advisor.advise(map, straightRoute(0.0, 5.0, 0.0, 2.0), broken);
    EXPECT_EQ(result.advice, Advice::Hold);
    EXPECT_NE(result.reason.find("aircraft position is not finite"), std::string::npos)
      << result.reason;
  }
}

// Positive control for the two Holds above lives at
// AnEmptyButFreshMapIsClearAndTheReasonIsStillFilled: the same advisor, a fed map,
// and a Clear verdict. Without it these two would also pass on an advisor that
// Holds unconditionally (R21/R27-3).

TEST(LocalAvoidance, BuildRefusesEveryUnusableLimitAndNamesIt)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  struct Case
  {
    const char * label;
    AvoidanceLimits limits;
  };

  std::vector<Case> cases;
  AvoidanceLimits limits;

  limits = AvoidanceLimits(); limits.horizon_m = 0.0;
  cases.push_back({"zero horizon", limits});
  limits = AvoidanceLimits(); limits.horizon_m = nan;
  cases.push_back({"nan horizon", limits});
  limits = AvoidanceLimits(); limits.sample_step_m = -0.1;
  cases.push_back({"negative step", limits});
  limits = AvoidanceLimits(); limits.clearance_scan_m = 0.0;
  cases.push_back({"zero clearance scan", limits});
  limits = AvoidanceLimits(); limits.spiral_growth_m = nan;
  cases.push_back({"nan growth", limits});
  limits = AvoidanceLimits(); limits.max_escape_climb_m = 0.0;
  cases.push_back({"zero climb guard", limits});
  limits = AvoidanceLimits(); limits.map_timeout_sec = -1.0;
  cases.push_back({"negative timeout", limits});
  limits = AvoidanceLimits(); limits.max_spiral_steps = 0;
  cases.push_back({"no spiral steps", limits});

  for (const Case & bad : cases) {
    std::string reason;
    const LocalAvoidance advisor = LocalAvoidance::build(bad.limits, reason);
    EXPECT_FALSE(advisor.valid()) << bad.label;
    EXPECT_FALSE(reason.empty()) << bad.label;
  }
}

TEST(LocalAvoidance, AnUnconfiguredAdvisorHoldsRatherThanWavingTheAircraftThrough)
{
  const LocalAvoidance advisor;
  const Costmap map = mapWith({});

  const AvoidanceResult result = advisor.advise(map, straightRoute(0.0, 5.0, 0.0, 2.0), Vec3{});

  EXPECT_EQ(result.advice, Advice::Hold);
  EXPECT_FALSE(result.reason.empty());
}

TEST(LocalAvoidance, AnEmptyRouteIsHeldNotCalledClear)
{
  const LocalAvoidance advisor = advisorWith();
  const Costmap map = mapWith({});

  EXPECT_EQ(advisor.advise(map, {}, Vec3{}).advice, Advice::Hold);
  EXPECT_EQ(advisor.advise(map, {Vec3{}}, Vec3{}).advice, Advice::Hold);
}

TEST(LocalAvoidance, ARouteCarryingANonFinitePointIsHeld)
{
  const LocalAvoidance advisor = advisorWith();
  const Costmap map = mapWith({});
  const double nan = std::numeric_limits<double>::quiet_NaN();

  const AvoidanceResult result =
    advisor.advise(map, {Vec3{0.0, 0.0, 2.0}, Vec3{nan, 0.0, 2.0}}, Vec3{});

  EXPECT_EQ(result.advice, Advice::Hold);
}

// The heart of the freshness contract: an empty map and a dead map look identical
// if you only read cells, so the advisor must ask the age instead.
TEST(LocalAvoidance, AMapThatWasNeverFedIsHeldAndSaysSo)
{
  const LocalAvoidance advisor = advisorWith();
  const Costmap map = starvedMap();

  const AvoidanceResult result =
    advisor.advise(map, straightRoute(0.0, 5.0, 0.0, 2.0), Vec3{0.0, 0.0, 2.0});

  EXPECT_EQ(result.advice, Advice::Hold);
  EXPECT_FALSE(result.map_fresh);
  EXPECT_FALSE(std::isfinite(result.map_age_sec));
  EXPECT_NE(result.reason.find("ever arrived"), std::string::npos);
}

TEST(LocalAvoidance, AnEmptyButFreshMapIsClearAndTheReasonIsStillFilled)
{
  const LocalAvoidance advisor = advisorWith();
  const Costmap map = mapWith({boxAt(0.0, 8.0, 2.0, 0.4)});

  const AvoidanceResult result =
    advisor.advise(map, straightRoute(0.0, 5.0, 0.0, 2.0), Vec3{0.0, 0.0, 2.0});

  EXPECT_EQ(result.advice, Advice::Clear);
  EXPECT_TRUE(result.map_fresh);
  EXPECT_FALSE(result.reason.empty()) << "reason is filled even on Clear";
  EXPECT_GT(result.checked_horizon_m, 0.0);
}

TEST(LocalAvoidance, AnEscapePointIsNaNWheneverTheAdviceIsNotEscape)
{
  const LocalAvoidance advisor = advisorWith();
  const Costmap map = mapWith({boxAt(0.0, 8.0, 2.0, 0.4)});

  const AvoidanceResult result =
    advisor.advise(map, straightRoute(0.0, 5.0, 0.0, 2.0), Vec3{0.0, 0.0, 2.0});

  ASSERT_EQ(result.advice, Advice::Clear);
  EXPECT_TRUE(std::isnan(result.escape_point.x)) << "(0,0,0) in odom is the launch point";
  EXPECT_TRUE(std::isnan(result.escape_point.y));
  EXPECT_TRUE(std::isnan(result.escape_point.z));
}

// 255 means "past the rolling window". Reading it as a wall would wall off every
// route longer than the map.
TEST(LocalAvoidance, LeavingTheMapWindowEndsTheCheckInsteadOfCondemningTheRoute)
{
  const LocalAvoidance advisor = advisorWith();
  const Costmap map = mapWith({boxAt(0.0, 8.0, 2.0, 0.4)});

  const AvoidanceResult result =
    advisor.advise(map, straightRoute(0.0, 400.0, 0.0, 2.0), Vec3{0.0, 0.0, 2.0});

  EXPECT_EQ(result.advice, Advice::Clear);
  EXPECT_LT(result.checked_horizon_m, 400.0) << "it must admit how far it actually saw";
}

TEST(LocalAvoidance, TheCheckStopsAtTheHorizonAndReportsThatDistance)
{
  AvoidanceLimits limits;
  limits.horizon_m = 3.0;
  const LocalAvoidance advisor = advisorWith(limits);
  const Costmap map = mapWith({boxAt(0.0, 8.0, 2.0, 0.4)});

  const AvoidanceResult result =
    advisor.advise(map, straightRoute(0.0, 10.0, 0.0, 2.0), Vec3{0.0, 0.0, 2.0});

  EXPECT_EQ(result.advice, Advice::Clear);
  EXPECT_NEAR(result.checked_horizon_m, 3.0, 0.2);
}

// The horizon has to actually stop the walk, not just clamp the number printed
// afterwards: an obstacle past it must stay unseen.
TEST(LocalAvoidance, AnObstacleBeyondTheHorizonIsNotReported)
{
  AvoidanceLimits limits;
  limits.horizon_m = 2.0;
  const LocalAvoidance advisor = advisorWith(limits);
  const Costmap map = mapWith({boxAt(5.0, 0.0, 2.0, 1.0)});

  const AvoidanceResult result =
    advisor.advise(map, straightRoute(0.0, 8.0, 0.0, 2.0), Vec3{0.0, 0.0, 2.0});

  EXPECT_EQ(result.advice, Advice::Clear) << result.reason;
  EXPECT_LE(result.checked_horizon_m, 2.0 + 0.2);
}

TEST(LocalAvoidance, AnObstacleSittingOnTheRouteEarnsAnEscapePoint)
{
  const LocalAvoidance advisor = advisorWith();
  const Costmap map = mapWith({boxAt(3.0, 0.0, 2.0, 1.0)});

  const AvoidanceResult result =
    advisor.advise(map, straightRoute(0.0, 6.0, 0.0, 2.0), Vec3{0.0, 0.0, 2.0});

  ASSERT_EQ(result.advice, Advice::Escape) << result.reason;
  EXPECT_TRUE(std::isfinite(result.escape_point.x));
  EXPECT_TRUE(std::isfinite(result.escape_point.y));
  EXPECT_TRUE(std::isfinite(result.escape_point.z));
}

// Decision 3 of the plan, and the rule the safety skill states outright. The pair
// below is the whole descent rule: with a wall ahead and another beside it, the
// spiral offers a LOWER turn before it offers a higher one, so the guard is the
// only thing choosing the higher.
TEST(LocalAvoidance, TheEscapeNeverDucksUnderTheObstacleEvenWhenTheLowerTurnComesFirst)
{
  const LocalAvoidance advisor = advisorWith();
  const Costmap map = mapWith({boxAt(3.0, 0.0, 2.0, 1.0), boxAt(3.0, 1.5, 2.0, 1.0)});

  const AvoidanceResult result =
    advisor.advise(map, straightRoute(0.0, 6.0, 0.0, 2.0), Vec3{0.0, 0.0, 2.0});

  ASSERT_EQ(result.advice, Advice::Escape) << result.reason;
  EXPECT_GE(result.escape_point.z, 2.0) << "a lower turn was offered first and refused";
}

TEST(LocalAvoidance, TheLowerTurnIsRealAndGetsTakenOnceDescendingIsAllowed)
{
  AvoidanceLimits limits;
  limits.allow_descent = true;
  const LocalAvoidance advisor = advisorWith(limits);
  const Costmap map = mapWith({boxAt(3.0, 0.0, 2.0, 1.0), boxAt(3.0, 1.5, 2.0, 1.0)});

  const AvoidanceResult result =
    advisor.advise(map, straightRoute(0.0, 6.0, 0.0, 2.0), Vec3{0.0, 0.0, 2.0});

  ASSERT_EQ(result.advice, Advice::Escape) << result.reason;
  EXPECT_LT(result.escape_point.z, 2.0)
    << "proves the refused option existed; without this the test above is vacuous";
}

// The cells are one flattened slice of flight_band_m, so a climb past that band is
// a climb into ground the map never described: the cap must be the tighter of the
// guard and the band. Measured on this geometry -- with the band in the cap the
// spiral settles 0.42 m up, without it the same call climbs 1.76 m.
TEST(LocalAvoidance, TheClimbIsCappedByTheSliceTheMapCanActuallyVouchFor)
{
  AvoidanceLimits limits;
  limits.spiral_growth_m = 1.0;
  limits.max_escape_climb_m = 2.0;        // deliberately looser than the band
  const LocalAvoidance advisor = advisorWith(limits);
  const Costmap map = mapWith({boxAt(3.0, 0.0, 2.0, 1.0)});
  ASSERT_LT(map.flightBand(), limits.max_escape_climb_m) << "precondition: band is tighter";

  const AvoidanceResult result =
    advisor.advise(map, straightRoute(0.0, 6.0, 0.0, 2.0), Vec3{0.0, 0.0, 2.0});

  ASSERT_EQ(result.advice, Advice::Escape) << result.reason;
  EXPECT_LE(std::abs(result.escape_point.z - 2.0), map.flightBand())
    << "escape z " << result.escape_point.z << " left the slice";
}

// Same rule seen from the other side: when every turn that stays inside the band is
// blocked, holding is the answer. Climbing out of the mapped slice is not.
TEST(LocalAvoidance, AnEscapeThatWouldLeaveTheMappedSliceIsRefusedOutright)
{
  AvoidanceLimits limits;
  limits.spiral_growth_m = 3.0;           // every early turn overshoots the band
  limits.max_escape_climb_m = 2.0;
  const LocalAvoidance advisor = advisorWith(limits);
  const Costmap map = mapWith({boxAt(3.0, 0.0, 2.0, 1.0)});

  const AvoidanceResult result =
    advisor.advise(map, straightRoute(0.0, 6.0, 0.0, 2.0), Vec3{0.0, 0.0, 2.0});

  EXPECT_EQ(result.advice, Advice::Hold) << result.reason;
  EXPECT_NE(result.reason.find("too high"), std::string::npos) << result.reason;
}

TEST(LocalAvoidance, ATighterVerticalGuardIsObeyed)
{
  AvoidanceLimits limits;
  limits.max_escape_climb_m = 0.5;
  const LocalAvoidance advisor = advisorWith(limits);
  const Costmap map = mapWith({boxAt(3.0, 0.0, 2.0, 1.0)});

  const AvoidanceResult result =
    advisor.advise(map, straightRoute(0.0, 6.0, 0.0, 2.0), Vec3{0.0, 0.0, 2.0});

  if (result.advice == Advice::Escape) {
    EXPECT_LE(std::abs(result.escape_point.z - 2.0), 0.5);
  } else {
    EXPECT_EQ(result.advice, Advice::Hold) << "refusing is allowed; sneaking past is not";
  }
}

TEST(LocalAvoidance, AWalledInAircraftIsHeldRatherThanSentSomewhereUnchecked)
{
  const LocalAvoidance advisor = advisorWith();
  std::vector<CostmapObstacle> wall;
  for (double y = -6.0; y <= 6.0; y += 0.5) {
    wall.push_back(boxAt(3.0, y, 2.0, 1.0));
  }
  const Costmap map = mapWith(wall);

  const AvoidanceResult result =
    advisor.advise(map, straightRoute(0.0, 6.0, 0.0, 2.0), Vec3{0.0, 0.0, 2.0});

  EXPECT_EQ(result.advice, Advice::Hold) << result.reason;
  EXPECT_TRUE(result.map_fresh) << "it must hold because of the wall, not a dark map";
  EXPECT_NE(result.reason.find("blocked"), std::string::npos) << result.reason;
  EXPECT_TRUE(std::isnan(result.escape_point.x));
}

TEST(LocalAvoidance, ClearanceFallsWhenTheRoutePassesCloserToAnObstacle)
{
  const LocalAvoidance advisor = advisorWith();
  const Costmap near_map = mapWith({boxAt(3.0, 1.2, 2.0, 0.4)});
  const Costmap far_map = mapWith({boxAt(3.0, 5.0, 2.0, 0.4)});

  const AvoidanceResult near_result =
    advisor.advise(near_map, straightRoute(0.0, 6.0, 0.0, 2.0), Vec3{0.0, 0.0, 2.0});
  const AvoidanceResult far_result =
    advisor.advise(far_map, straightRoute(0.0, 6.0, 0.0, 2.0), Vec3{0.0, 0.0, 2.0});

  ASSERT_EQ(near_result.advice, Advice::Clear) << near_result.reason;
  ASSERT_EQ(far_result.advice, Advice::Clear) << far_result.reason;
  EXPECT_LT(near_result.clearance_m, far_result.clearance_m);
}

TEST(LocalAvoidance, ClearanceSaturatesAtTheScanRadiusAndNeverExceedsIt)
{
  AvoidanceLimits limits;
  limits.clearance_scan_m = 1.5;
  const LocalAvoidance advisor = advisorWith(limits);
  const Costmap map = mapWith({boxAt(0.0, 9.0, 2.0, 0.4)});

  const AvoidanceResult result =
    advisor.advise(map, straightRoute(0.0, 5.0, 0.0, 2.0), Vec3{0.0, 0.0, 2.0});

  EXPECT_LE(result.clearance_m, 1.5) << "the reported clearance is a floor, not a measurement";
}

TEST(LocalAvoidance, AStaleMapIsHeldEvenThoughEveryCellReadsFree)
{
  AvoidanceLimits limits;
  limits.map_timeout_sec = 1.0;
  const LocalAvoidance advisor = advisorWith(limits);

  std::string reason;
  Costmap map = Costmap::build(CostmapLimits(), reason);
  ASSERT_TRUE(map.valid()) << reason;
  map.update({boxAt(0.0, 9.0, 2.0, 0.4)}, Vec3{0.0, 0.0, 2.0}, kNow, kNow, reason);
  // Same map, asked about later: the cells are unchanged, the age is not.
  map.update({}, Vec3{0.0, 0.0, 2.0}, kNow + 5.0, kNoReport, reason);

  const AvoidanceResult result =
    advisor.advise(map, straightRoute(0.0, 5.0, 0.0, 2.0), Vec3{0.0, 0.0, 2.0});

  EXPECT_EQ(result.advice, Advice::Hold);
  EXPECT_FALSE(result.map_fresh);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// The climb cap is measured from the OBSTACLE, but the cells are one slice around
// the AIRCRAFT. A route that climbs pulls the two apart, and an escape allowed by
// the cap can then sit outside the very cells that cleared it.
TEST(LocalAvoidance, AnEscapeStaysInsideTheSliceWhenTheObstacleSitsAboveTheAircraft)
{
  const LocalAvoidance advisor = advisorWith();
  // Aircraft at 2.0 so the map describes 1.0..3.0; the blocked route point is 2.4.
  const Costmap map = mapWith({boxAt(3.0, 0.0, 2.4, 0.2)}, Vec3{0.0, 0.0, 2.0});

  const AvoidanceResult result =
    advisor.advise(map, straightRoute(0.0, 6.0, 0.0, 2.4), Vec3{0.0, 0.0, 2.0});

  ASSERT_EQ(result.advice, Advice::Escape) << result.reason;
  EXPECT_LE(std::abs(result.escape_point.z - map.origin().z), map.flightBand())
    << "escape z " << result.escape_point.z << " left the slice centred on "
    << map.origin().z;
}

// The string goes on the wire in AvoidanceAdvice.reason and ends up in result.message.
TEST(LocalAvoidance, AStaleMapIsReportedInSecondsNotInMetres)
{
  const LocalAvoidance advisor = advisorWith();
  std::string reason;
  Costmap map = Costmap::build(CostmapLimits(), reason);
  ASSERT_TRUE(map.valid()) << reason;
  map.update({boxAt(3.0, 0.0, 2.0, 1.0)}, Vec3{0.0, 0.0, 2.0}, kNow, kNow, reason);
  ASSERT_TRUE(std::isfinite(map.lastInputAge())) << "the obstacle never reached the map";
  // Nothing new arrives for half a minute: the cells stay, the age must not.
  map.update(std::vector<CostmapObstacle>{}, Vec3{0.0, 0.0, 2.0}, kNow + 30.0, kNoReport, reason);

  const AvoidanceResult result =
    advisor.advise(map, straightRoute(0.0, 6.0, 0.0, 2.0), Vec3{0.0, 0.0, 2.0});

  ASSERT_EQ(result.advice, Advice::Hold) << result.reason;
  EXPECT_NE(result.reason.find("30.00 s"), std::string::npos)
    << "an age in seconds was printed as a distance: " << result.reason;
  EXPECT_EQ(result.reason.find("30.00 m"), std::string::npos)
    << "a duration is still wearing metres: " << result.reason;
}

// A stamp from the future is a broken clock; reading it as freshness is worse.
TEST(LocalAvoidance, AMapStampedInTheFutureIsNotFresh)
{
  const LocalAvoidance advisor = advisorWith();
  std::string reason;
  Costmap map = Costmap::build(CostmapLimits(), reason);
  ASSERT_TRUE(map.valid()) << reason;
  map.update({boxAt(3.0, 0.0, 2.0, 1.0)}, Vec3{0.0, 0.0, 2.0}, kNow - 30.0, kNow, reason);

  EXPECT_TRUE(std::isinf(map.lastInputAge())) << "age " << map.lastInputAge();

  const AvoidanceResult result =
    advisor.advise(map, straightRoute(0.0, 6.0, 0.0, 2.0), Vec3{0.0, 0.0, 2.0});
  EXPECT_FALSE(result.map_fresh) << result.reason;
  EXPECT_EQ(result.advice, Advice::Hold) << result.reason;
}
