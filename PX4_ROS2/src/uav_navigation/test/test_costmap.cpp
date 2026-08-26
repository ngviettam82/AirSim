#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "uav_navigation/costmap.hpp"

using uav_navigation::Costmap;
using uav_navigation::CostmapLimits;
using uav_navigation::CostmapObstacle;
using uav_navigation::kCostFree;
using uav_navigation::kCostInscribed;
using uav_navigation::kCostLethal;
using uav_navigation::kCostUnknown;
using uav_navigation::Vec3;

namespace
{

Costmap freshMap(const CostmapLimits & limits = CostmapLimits())
{
  std::string reason;
  Costmap map = Costmap::build(limits, reason);
  EXPECT_TRUE(map.valid()) << reason;
  return map;
}

/// No obstacle report has ever arrived. Distinct from a report that carried nothing:
/// the first is a dead feed, the second is a sensor saying it looked and saw clear.
constexpr double kNoReport = -std::numeric_limits<double>::infinity();

CostmapObstacle boxAt(double x, double y, double z, double side, double stamp)
{
  CostmapObstacle obstacle;
  obstacle.center = Vec3{x, y, z};
  obstacle.size = Vec3{side, side, side};
  obstacle.position_uncertainty = 0.0;
  obstacle.stamp_seconds = stamp;
  return obstacle;
}

}  // namespace

TEST(Costmap, ShippedDefaultsGiveTheWindowTheSizeThePlanAsks)
{
  const Costmap map = freshMap();
  EXPECT_EQ(map.cellsPerSide(), 100u);
  EXPECT_DOUBLE_EQ(map.resolution(), 0.25);
}

TEST(Costmap, BuildRejectsEveryUnusableLimitAndNamesIt)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  struct Case
  {
    const char * label;
    CostmapLimits limits;
  };

  std::vector<Case> cases;
  CostmapLimits bad;

  bad = CostmapLimits(); bad.resolution_m = 0.0;      cases.push_back({"zero resolution", bad});
  bad = CostmapLimits(); bad.resolution_m = -0.25;    cases.push_back({"negative resolution", bad});
  bad = CostmapLimits(); bad.resolution_m = nan;      cases.push_back({"nan resolution", bad});
  bad = CostmapLimits(); bad.width_m = 0.0;           cases.push_back({"zero width", bad});
  bad = CostmapLimits(); bad.width_m = nan;           cases.push_back({"nan width", bad});
  bad = CostmapLimits(); bad.cost_scaling = 0.0;      cases.push_back({"zero scaling", bad});
  bad = CostmapLimits(); bad.drone_radius_m = -0.1;   cases.push_back({"negative radius", bad});
  bad = CostmapLimits(); bad.corner_cut_m = nan;      cases.push_back({"nan corner cut", bad});
  bad = CostmapLimits(); bad.obstacle_timeout_sec = -1.0; cases.push_back({"negative timeout", bad});
  bad = CostmapLimits(); bad.flight_band_m = nan;     cases.push_back({"nan band", bad});
  bad = CostmapLimits(); bad.width_m = 1.0e6;         cases.push_back({"absurd cell count", bad});

  for (const Case & entry : cases) {
    std::string reason;
    const Costmap map = Costmap::build(entry.limits, reason);
    EXPECT_FALSE(map.valid()) << entry.label;
    EXPECT_FALSE(reason.empty()) << entry.label;
  }

  // Positive control: the same harness must accept the shipped limits.
  std::string reason;
  EXPECT_TRUE(Costmap::build(CostmapLimits(), reason).valid()) << reason;
  EXPECT_TRUE(reason.empty());
}

TEST(Costmap, AnUntouchedMapReportsInfiniteAgeInsteadOfFreeSpace)
{
  const Costmap map = freshMap();
  EXPECT_TRUE(std::isinf(map.lastInputAge()));
  // The cells do read FREE, which is exactly why age is the honest channel.
  EXPECT_EQ(map.cost(50, 50), kCostFree);
}

TEST(Costmap, SilenceAgesTheMapInsteadOfLookingLikeOpenAir)
{
  Costmap map = freshMap();
  std::string reason;

  std::vector<CostmapObstacle> obstacles{boxAt(2.0, 0.0, 0.0, 1.0, 100.0)};
  map.update(obstacles, Vec3{0.0, 0.0, 0.0}, 100.0, 100.0, reason);
  EXPECT_DOUBLE_EQ(map.lastInputAge(), 0.0);
  EXPECT_EQ(map.written(), 1u);

  map.update({}, Vec3{0.0, 0.0, 0.0}, 102.0, kNoReport, reason);
  EXPECT_DOUBLE_EQ(map.lastInputAge(), 2.0);
  EXPECT_EQ(map.cost(50, 50), kCostFree);

  map.update({}, Vec3{0.0, 0.0, 0.0}, 130.0, kNoReport, reason);
  EXPECT_DOUBLE_EQ(map.lastInputAge(), 30.0);

  // Positive control: fresh input pulls the age straight back down.
  obstacles[0].stamp_seconds = 130.0;
  map.update(obstacles, Vec3{0.0, 0.0, 0.0}, 130.0, 130.0, reason);
  EXPECT_DOUBLE_EQ(map.lastInputAge(), 0.0);
}

// Debt #19, measured 2026-08-26: freshness came from the newest obstacle WRITTEN, so a
// healthy perception reporting clear sky read exactly like a dead one -- age inf, "no
// obstacle input has ever arrived". The operational cost is real: fly out of a cluttered
// area into the open and the advisor Holds, and require_obstacle_feed is the real-side
// default, so the planner refuses to plan at all.
TEST(Costmap, AnEmptyReportIsFreshBecauseTheSensorDidLook)
{
  Costmap map = freshMap();
  std::string reason;

  map.update({}, Vec3{0.0, 0.0, 0.0}, 100.0, 100.0, reason);

  EXPECT_DOUBLE_EQ(map.lastInputAge(), 0.0) << "an empty report is a report";
  EXPECT_EQ(map.written(), 0u);
  EXPECT_EQ(map.rejected(), 0u);
}

TEST(Costmap, ObstaclesAllOutsideTheBandStillProveTheFeedIsAlive)
{
  Costmap map = freshMap();
  std::string reason;

  const std::vector<CostmapObstacle> above{
    boxAt(0.0, 3.0, 40.0, 1.0, 100.0),
    boxAt(0.0, -3.0, 40.0, 1.0, 100.0),
  };
  map.update(above, Vec3{0.0, 0.0, 0.0}, 100.0, 100.0, reason);

  EXPECT_EQ(map.written(), 0u);
  EXPECT_EQ(map.rejected(), 2u) << "the band filter did drop them";
  EXPECT_DOUBLE_EQ(map.lastInputAge(), 0.0)
    << "dropped for being irrelevant is the map working, not the feed dying";
}

// The other half, and the reason freshness is not simply "a report arrived": a report the
// map could not trust must not license planning over an empty map.
TEST(Costmap, ACorruptReportDoesNotCountAsFreshness)
{
  Costmap map = freshMap();
  std::string reason;

  CostmapObstacle poisoned = boxAt(2.0, 0.0, 0.0, 1.0, 100.0);
  poisoned.size.x = std::numeric_limits<double>::quiet_NaN();
  map.update({poisoned}, Vec3{0.0, 0.0, 0.0}, 100.0, 100.0, reason);
  EXPECT_TRUE(std::isinf(map.lastInputAge())) << "age " << map.lastInputAge();

  CostmapObstacle unmeasured = boxAt(2.0, 0.0, 0.0, 1.0, 100.0);
  unmeasured.position_uncertainty = -1.0;
  map.update({unmeasured}, Vec3{0.0, 0.0, 0.0}, 100.0, 100.0, reason);
  EXPECT_TRUE(std::isinf(map.lastInputAge()));

  const CostmapObstacle too_old = boxAt(2.0, 0.0, 0.0, 1.0, 100.0);
  map.update({too_old}, Vec3{0.0, 0.0, 0.0}, 104.0, 100.0, reason);
  EXPECT_TRUE(std::isinf(map.lastInputAge())) << "stale content is not a live feed";

  // Positive control: the same call with a sound report does move the age.
  map.update({}, Vec3{0.0, 0.0, 0.0}, 104.0, 104.0, reason);
  EXPECT_DOUBLE_EQ(map.lastInputAge(), 0.0);
}

TEST(Costmap, ADeadFeedIsStillDistinguishableFromAQuietOne)
{
  Costmap map = freshMap();
  std::string reason;

  map.update({}, Vec3{0.0, 0.0, 0.0}, 100.0, kNoReport, reason);

  EXPECT_TRUE(std::isinf(map.lastInputAge()))
    << "nothing has ever arrived -- that must not read as clear sky";
}

TEST(Costmap, EveryObstacleIsEitherWrittenOrRejected)
{
  Costmap map = freshMap();
  std::string reason;

  std::vector<CostmapObstacle> obstacles{
    boxAt(2.0, 0.0, 0.0, 1.0, 100.0),
    boxAt(-2.0, 0.0, 0.0, 1.0, 90.0),      // stale
    boxAt(0.0, 3.0, 40.0, 1.0, 100.0),     // out of band
    boxAt(3.0, 3.0, 0.0, 1.0, 100.0),
  };
  obstacles[3].position_uncertainty = -1.0;  // unmeasured

  map.update(obstacles, Vec3{0.0, 0.0, 0.0}, 100.0, 100.0, reason);
  EXPECT_EQ(map.written() + map.rejected(), obstacles.size());
  EXPECT_EQ(map.written(), 1u);
  EXPECT_EQ(map.rejected(), 3u);
  EXPECT_FALSE(reason.empty());
}

TEST(Costmap, ANonFiniteObstacleIsRejectedNotSwallowed)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  Costmap map = freshMap();
  std::string reason;

  CostmapObstacle poisoned = boxAt(2.0, 0.0, 0.0, 1.0, 100.0);
  poisoned.size.x = nan;
  map.update({poisoned}, Vec3{0.0, 0.0, 0.0}, 100.0, 100.0, reason);
  EXPECT_EQ(map.rejected(), 1u);
  EXPECT_EQ(map.written(), 0u);
  EXPECT_TRUE(std::isinf(map.lastInputAge()));

  // Positive control: the identical obstacle with a finite extent is written.
  const CostmapObstacle clean = boxAt(2.0, 0.0, 0.0, 1.0, 100.0);
  map.update({clean}, Vec3{0.0, 0.0, 0.0}, 100.0, 100.0, reason);
  EXPECT_EQ(map.written(), 1u);
  EXPECT_EQ(map.rejected(), 0u);
}

TEST(Costmap, AStaleObstacleIsRefusedAtTheTimeoutEdge)
{
  Costmap map = freshMap();
  std::string reason;
  const CostmapObstacle obstacle = boxAt(2.0, 0.0, 0.0, 1.0, 100.0);

  map.update({obstacle}, Vec3{0.0, 0.0, 0.0}, 103.0, 100.0, reason);
  EXPECT_EQ(map.written(), 1u) << "3.0 s is the timeout, not past it";

  map.update({obstacle}, Vec3{0.0, 0.0, 0.0}, 103.5, 100.0, reason);
  EXPECT_EQ(map.rejected(), 1u);
}

TEST(Costmap, TheFootprintIsLethalAndTheRingAroundItIsUnfittable)
{
  Costmap map = freshMap();
  std::string reason;
  map.update({boxAt(0.0, 0.0, 0.0, 1.0, 100.0)}, Vec3{0.0, 0.0, 0.0}, 100.0, 100.0, reason);

  EXPECT_EQ(map.costAt(Vec3{0.0, 0.0, 0.0}), kCostLethal);
  // 0.6 m out is 0.1 m of clearance, well inside the 0.35 m the airframe needs.
  EXPECT_GE(map.costAt(Vec3{0.6, 0.0, 0.0}), kCostInscribed);
  // 1.5 m out clears the airframe, so the cost must drop below the inscribed band.
  const uint8_t soft = map.costAt(Vec3{1.5, 0.0, 0.0});
  EXPECT_LT(soft, kCostInscribed);
  EXPECT_GT(soft, kCostFree);
}

TEST(Costmap, SoftCostDecaysMonotonicallyAwayFromTheObstacle)
{
  Costmap map = freshMap();
  std::string reason;
  map.update({boxAt(0.0, 0.0, 0.0, 1.0, 100.0)}, Vec3{0.0, 0.0, 0.0}, 100.0, 100.0, reason);

  uint8_t previous = kCostLethal;
  bool ever_dropped = false;
  for (double x = 1.0; x <= 8.0; x += 0.25) {
    const uint8_t here = map.costAt(Vec3{x, 0.0, 0.0});
    EXPECT_LE(here, previous) << "cost rose again at x=" << x;
    if (here < previous) {
      ever_dropped = true;
    }
    previous = here;
  }
  EXPECT_TRUE(ever_dropped) << "a flat field would pass a monotonic check vacuously";
  EXPECT_EQ(map.costAt(Vec3{9.0, 0.0, 0.0}), kCostFree);
}

TEST(Costmap, UncertaintyWidensTheKeepOutRing)
{
  std::string reason;
  const Vec3 probe{1.1, 0.0, 0.0};  // 0.6 m of clearance from a 1 m box

  Costmap certain = freshMap();
  certain.update({boxAt(0.0, 0.0, 0.0, 1.0, 100.0)}, Vec3{0.0, 0.0, 0.0}, 100.0, 100.0, reason);
  EXPECT_LT(certain.costAt(probe), kCostInscribed);

  CostmapObstacle doubtful = boxAt(0.0, 0.0, 0.0, 1.0, 100.0);
  doubtful.position_uncertainty = 0.5;
  Costmap uncertain = freshMap();
  uncertain.update({doubtful}, Vec3{0.0, 0.0, 0.0}, 100.0, 100.0, reason);
  EXPECT_GE(uncertain.costAt(probe), kCostInscribed)
    << "0.35 + 0.5 must swallow a 0.6 m gap";
}

TEST(Costmap, TheCornerTheSplineCutsIsPaidForInInflation)
{
  std::string reason;
  const Vec3 probe{1.1, 0.0, 0.0};
  const CostmapObstacle obstacle = boxAt(0.0, 0.0, 0.0, 1.0, 100.0);

  Costmap without = freshMap();
  without.update({obstacle}, Vec3{0.0, 0.0, 0.0}, 100.0, 100.0, reason);
  EXPECT_LT(without.costAt(probe), kCostInscribed);

  CostmapLimits paid;
  paid.corner_cut_m = 0.5;
  Costmap with = freshMap(paid);
  with.update({obstacle}, Vec3{0.0, 0.0, 0.0}, 100.0, 100.0, reason);
  EXPECT_GE(with.costAt(probe), kCostInscribed)
    << "clearance bought at the waypoint is not the clearance flown";
}

TEST(Costmap, OverlappingObstaclesKeepTheHigherCost)
{
  Costmap map = freshMap();
  std::string reason;
  const std::vector<CostmapObstacle> obstacles{
    boxAt(0.0, 0.0, 0.0, 1.0, 100.0),
    boxAt(4.0, 0.0, 0.0, 1.0, 100.0),
  };
  map.update(obstacles, Vec3{0.0, 0.0, 0.0}, 100.0, 100.0, reason);

  EXPECT_EQ(map.costAt(Vec3{0.0, 0.0, 0.0}), kCostLethal);
  EXPECT_EQ(map.costAt(Vec3{4.0, 0.0, 0.0}), kCostLethal);
  // Between them the nearer obstacle wins; neither erases the other.
  EXPECT_GT(map.costAt(Vec3{2.0, 0.0, 0.0}), kCostFree);
}

TEST(Costmap, AnObstacleOutsideTheFlightBandIsIgnoredButDoubtCanReachIntoIt)
{
  std::string reason;
  CostmapObstacle overhead = boxAt(0.0, 0.0, 2.5, 1.0, 100.0);

  Costmap ignored = freshMap();
  ignored.update({overhead}, Vec3{0.0, 0.0, 0.0}, 100.0, 100.0, reason);
  EXPECT_EQ(ignored.rejected(), 1u);
  EXPECT_EQ(ignored.costAt(Vec3{0.0, 0.0, 0.0}), kCostFree);

  // Positive control: 1 m of doubt drops its underside to the top of the band.
  overhead.position_uncertainty = 1.0;
  Costmap reached = freshMap();
  reached.update({overhead}, Vec3{0.0, 0.0, 0.0}, 100.0, 100.0, reason);
  EXPECT_EQ(reached.written(), 1u);
  EXPECT_EQ(reached.costAt(Vec3{0.0, 0.0, 0.0}), kCostLethal);
}

TEST(Costmap, TheOriginSnapsToTheGridSoTheWindowDoesNotResampleItself)
{
  Costmap map = freshMap();
  std::string reason;

  map.update({}, Vec3{0.0, 0.0, 0.0}, 100.0, kNoReport, reason);
  const Vec3 first = map.origin();
  EXPECT_DOUBLE_EQ(std::fmod(first.x, 0.25), 0.0);
  EXPECT_DOUBLE_EQ(std::fmod(first.y, 0.25), 0.0);

  // Sub-cell drift must not shift the lattice under the planner.
  map.update({}, Vec3{0.1, 0.1, 0.0}, 100.0, kNoReport, reason);
  EXPECT_DOUBLE_EQ(map.origin().x, first.x);
  EXPECT_DOUBLE_EQ(map.origin().y, first.y);

  // Positive control: a full cell of travel does move it, by exactly one cell.
  map.update({}, Vec3{0.25, 0.0, 0.0}, 100.0, kNoReport, reason);
  EXPECT_DOUBLE_EQ(map.origin().x, first.x + 0.25);
}

TEST(Costmap, WorldAndCellRoundTripInsideTheWindow)
{
  Costmap map = freshMap();
  std::string reason;
  map.update({}, Vec3{7.0, -3.0, 2.0}, 100.0, kNoReport, reason);

  for (int cell_y : {0, 17, 99}) {
    for (int cell_x : {0, 42, 99}) {
      const Vec3 point = map.cellToWorld(cell_x, cell_y);
      int back_x = -1;
      int back_y = -1;
      ASSERT_TRUE(map.worldToCell(point, back_x, back_y));
      EXPECT_EQ(back_x, cell_x);
      EXPECT_EQ(back_y, cell_y);
    }
  }
  EXPECT_DOUBLE_EQ(map.cellToWorld(0, 0).z, 2.0) << "the band centre travels with the window";
}

TEST(Costmap, OutsideTheWindowTheAnswerIsUnknownNotFree)
{
  Costmap map = freshMap();
  std::string reason;
  map.update({}, Vec3{0.0, 0.0, 0.0}, 100.0, kNoReport, reason);

  EXPECT_EQ(map.cost(-1, 0), kCostUnknown);
  EXPECT_EQ(map.cost(0, 100), kCostUnknown);
  EXPECT_EQ(map.costAt(Vec3{500.0, 0.0, 0.0}), kCostUnknown);
  EXPECT_EQ(map.costAt(Vec3{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}), kCostUnknown);
  EXPECT_EQ(map.cost(0, 0), kCostFree);
}

TEST(Costmap, AnUnbuiltMapRefusesToPretendItIsAMap)
{
  Costmap map;
  std::string reason;
  EXPECT_FALSE(map.valid());
  map.update({boxAt(0.0, 0.0, 0.0, 1.0, 100.0)}, Vec3{0.0, 0.0, 0.0}, 100.0, 100.0, reason);
  EXPECT_FALSE(reason.empty());
  EXPECT_EQ(map.cost(0, 0), kCostUnknown);
  EXPECT_EQ(map.costAt(Vec3{0.0, 0.0, 0.0}), kCostUnknown);
}

TEST(Costmap, ANonFiniteCentreDropsTheWholeBatchInsteadOfPlacingItWrong)
{
  Costmap map = freshMap();
  std::string reason;
  const std::vector<CostmapObstacle> obstacles{
    boxAt(0.0, 0.0, 0.0, 1.0, 100.0),
    boxAt(2.0, 0.0, 0.0, 1.0, 100.0),
  };
  const double nan = std::numeric_limits<double>::quiet_NaN();

  map.update(obstacles, Vec3{nan, 0.0, 0.0}, 100.0, 100.0, reason);
  EXPECT_EQ(map.rejected(), obstacles.size());
  EXPECT_EQ(map.written(), 0u);
  EXPECT_TRUE(std::isinf(map.lastInputAge()));
  EXPECT_FALSE(reason.empty());
}
