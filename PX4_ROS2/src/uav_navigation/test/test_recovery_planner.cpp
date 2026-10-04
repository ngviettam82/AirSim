#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "uav_navigation/recovery_planner.hpp"
#include "uav_navigation/trajectory.hpp"

using uav_navigation::RecoveryLimits;
using uav_navigation::RecoveryPlan;
using uav_navigation::RecoveryPlanner;
using uav_navigation::RecoveryRequest;
using uav_navigation::RecoveryState;
using uav_navigation::RecoveryType;
using uav_navigation::Trajectory;
using uav_navigation::TrajectoryLimits;
using uav_navigation::Vec3;
using uav_navigation::isFinite;
using uav_navigation::recoveryTypeName;

namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

RecoveryPlanner plannerWith(const RecoveryLimits & limits = RecoveryLimits())
{
  std::string reason;
  RecoveryPlanner planner = RecoveryPlanner::build(limits, reason);
  EXPECT_TRUE(planner.valid()) << reason;
  return planner;
}

RecoveryState stateAt(const Vec3 & position)
{
  RecoveryState state;
  state.position = position;
  return state;
}

RecoveryState stateWithHome(const Vec3 & position, const Vec3 & home)
{
  RecoveryState state;
  state.position = position;
  state.home = home;
  state.home_known = true;
  return state;
}

RecoveryRequest requestFor(RecoveryType type, double safe_altitude = kNaN)
{
  RecoveryRequest request;
  request.type = type;
  request.safe_altitude_m = safe_altitude;
  return request;
}

bool mentions(const std::string & reason, const std::string & needle)
{
  return reason.find(needle) != std::string::npos;
}

/// Every refusal must look the same: no waypoints, no hold point, a reason.
void expectRefused(const RecoveryPlan & plan)
{
  EXPECT_FALSE(plan.accepted) << plan.reason;
  EXPECT_TRUE(plan.waypoints.empty());
  EXPECT_FALSE(std::isfinite(plan.hold_point.x));
  EXPECT_FALSE(std::isfinite(plan.hold_point.y));
  EXPECT_FALSE(std::isfinite(plan.hold_point.z));
  EXPECT_FALSE(plan.reason.empty());
}

}  // namespace

TEST(RecoveryPlanner, BuildRefusesEveryUnusableLimitAndNamesIt)
{
  struct Case
  {
    const char * field;
    RecoveryLimits limits;
  };

  std::vector<Case> cases;
  for (double bad : {0.0, -1.0, kNaN, std::numeric_limits<double>::infinity()}) {
    RecoveryLimits limits;
    limits.min_travel_m = bad;
    cases.push_back({"min_travel_m", limits});

    limits = RecoveryLimits();
    limits.max_climb_m = bad;
    cases.push_back({"max_climb_m", limits});

    limits = RecoveryLimits();
    limits.max_safe_altitude_m = bad;
    cases.push_back({"max_safe_altitude_m", limits});

    limits = RecoveryLimits();
    limits.min_home_clearance_m = bad;
    cases.push_back({"min_home_clearance_m", limits});

    limits = RecoveryLimits();
    limits.max_home_distance_m = bad;
    cases.push_back({"max_home_distance_m", limits});
  }

  for (const Case & item : cases) {
    std::string reason;
    const RecoveryPlanner planner = RecoveryPlanner::build(item.limits, reason);
    EXPECT_FALSE(planner.valid()) << item.field;
    EXPECT_TRUE(mentions(reason, item.field)) << item.field << " got: " << reason;
  }
}

TEST(RecoveryPlanner, BuildAcceptsTheDefaultsAndKeepsThem)
{
  const RecoveryLimits limits;
  const RecoveryPlanner planner = plannerWith(limits);
  EXPECT_EQ(planner.limits().min_travel_m, limits.min_travel_m);
  EXPECT_EQ(planner.limits().max_home_distance_m, limits.max_home_distance_m);
}

TEST(RecoveryPlanner, UnconfiguredPlannerRefusesEvenTheSimplestRequest)
{
  const RecoveryPlanner planner;
  ASSERT_FALSE(planner.valid());
  const RecoveryPlan plan =
    planner.plan(requestFor(RecoveryType::Hover), stateAt(Vec3{1.0, 2.0, 3.0}));
  expectRefused(plan);
  EXPECT_TRUE(mentions(plan.reason, "never configured")) << plan.reason;
}

TEST(RecoveryPlanner, NonFinitePositionIsRefusedForEveryType)
{
  const RecoveryPlanner planner = plannerWith();
  const RecoveryLimits limits = planner.limits();
  const Vec3 broken{0.0, kNaN, 2.0};

  RecoveryState state = stateWithHome(broken, Vec3{0.0, 0.0, 0.0});
  for (RecoveryType type :
    {RecoveryType::Hover, RecoveryType::ClimbToSafe, RecoveryType::ReturnHome})
  {
    const RecoveryPlan plan =
      planner.plan(requestFor(type, limits.min_home_clearance_m + 1.0), state);
    expectRefused(plan);
    EXPECT_TRUE(mentions(plan.reason, "position is not finite")) << plan.reason;
  }
}

TEST(RecoveryPlanner, TypesOutsideTheThreeAreRefused)
{
  const RecoveryPlanner planner = plannerWith();
  const RecoveryState state = stateWithHome(Vec3{0.0, 0.0, 2.0}, Vec3{0.0, 0.0, 0.0});

  // 4 and 5 are the action's LAND and HANDOVER_TO_PILOT codes.
  for (uint8_t code : {uint8_t{0}, uint8_t{4}, uint8_t{5}, uint8_t{200}}) {
    RecoveryRequest request = requestFor(static_cast<RecoveryType>(code), 5.0);
    const RecoveryPlan plan = planner.plan(request, state);
    expectRefused(plan);
    EXPECT_TRUE(mentions(plan.reason, "unsupported recovery type"))
      << static_cast<int>(code) << " got: " << plan.reason;
  }
}

TEST(RecoveryPlanner, HoverProducesNoMotionAtAll)
{
  const RecoveryPlanner planner = plannerWith();
  const Vec3 position{3.5, -1.25, 4.75};

  // A safe altitude is offered and must be ignored.
  const RecoveryPlan plan = planner.plan(requestFor(RecoveryType::Hover, 20.0), stateAt(position));

  EXPECT_TRUE(plan.accepted) << plan.reason;
  EXPECT_EQ(plan.type, RecoveryType::Hover);
  EXPECT_TRUE(plan.waypoints.empty());
  EXPECT_EQ(plan.hold_point.x, position.x);
  EXPECT_EQ(plan.hold_point.y, position.y);
  EXPECT_EQ(plan.hold_point.z, position.z);
  EXPECT_EQ(plan.climb_m, 0.0);
}

TEST(RecoveryPlanner, HoverNeedsNoHomeAndNoAltitude)
{
  const RecoveryPlanner planner = plannerWith();
  const RecoveryPlan plan =
    planner.plan(requestFor(RecoveryType::Hover), stateAt(Vec3{0.0, 0.0, 1.0}));
  EXPECT_TRUE(plan.accepted) << plan.reason;
  EXPECT_TRUE(plan.waypoints.empty());
}

TEST(RecoveryPlanner, ClimbRefusesANonFiniteSafeAltitude)
{
  const RecoveryPlanner planner = plannerWith();
  for (double altitude : {kNaN, std::numeric_limits<double>::infinity()}) {
    const RecoveryPlan plan =
      planner.plan(requestFor(RecoveryType::ClimbToSafe, altitude), stateAt(Vec3{0.0, 0.0, 2.0}));
    expectRefused(plan);
    EXPECT_TRUE(mentions(plan.reason, "not finite")) << plan.reason;
  }
}

TEST(RecoveryPlanner, ClimbRefusesAnythingNotClearlyAboveTheAircraft)
{
  const RecoveryPlanner planner = plannerWith();
  const RecoveryLimits limits = planner.limits();
  const double here = 2.0;

  // Ceiling and cap both satisfied, so only this guard fires.
  for (double apex :
    {here - 1.0, here, here + limits.min_travel_m * 0.5})
  {
    const RecoveryPlan plan =
      planner.plan(requestFor(RecoveryType::ClimbToSafe, apex), stateAt(Vec3{0.0, 0.0, here}));
    expectRefused(plan);
    EXPECT_TRUE(mentions(plan.reason, "above the aircraft")) << apex << " got: " << plan.reason;
  }
}

TEST(RecoveryPlanner, ClimbRefusesAnApexAboveTheCeilingWhileTheStepIsLegal)
{
  RecoveryLimits limits;
  const RecoveryPlanner planner = plannerWith(limits);

  const double apex = limits.max_safe_altitude_m + 1.0;
  const double here = apex - limits.max_climb_m * 0.5;   // step stays inside the cap
  const RecoveryPlan plan =
    planner.plan(requestFor(RecoveryType::ClimbToSafe, apex), stateAt(Vec3{0.0, 0.0, here}));

  expectRefused(plan);
  EXPECT_TRUE(mentions(plan.reason, "ceiling")) << plan.reason;
}

TEST(RecoveryPlanner, ClimbRefusesAStepPastTheCapWhileTheApexIsLegal)
{
  RecoveryLimits limits;
  const RecoveryPlanner planner = plannerWith(limits);

  const double apex = limits.max_safe_altitude_m;         // ceiling is satisfied
  const double here = apex - limits.max_climb_m * 1.5;    // the step alone is too big
  const RecoveryPlan plan =
    planner.plan(requestFor(RecoveryType::ClimbToSafe, apex), stateAt(Vec3{0.0, 0.0, here}));

  expectRefused(plan);
  EXPECT_TRUE(mentions(plan.reason, "single-step cap")) << plan.reason;
}

TEST(RecoveryPlanner, ClimbGoesStraightUpFromWhereTheAircraftIs)
{
  const RecoveryPlanner planner = plannerWith();
  const RecoveryLimits limits = planner.limits();
  const Vec3 position{7.5, -3.25, 2.0};
  // Bracketed, not exact: (z+m)-z can land just under m.
  const double apex = position.z + limits.min_travel_m * 1.5;

  const RecoveryPlan plan =
    planner.plan(requestFor(RecoveryType::ClimbToSafe, apex), stateAt(position));

  ASSERT_TRUE(plan.accepted) << plan.reason;
  EXPECT_EQ(plan.type, RecoveryType::ClimbToSafe);
  ASSERT_EQ(plan.waypoints.size(), 2u);
  EXPECT_EQ(plan.waypoints.front().x, position.x);
  EXPECT_EQ(plan.waypoints.front().y, position.y);
  EXPECT_EQ(plan.waypoints.front().z, position.z);
  EXPECT_EQ(plan.waypoints.back().x, position.x);
  EXPECT_EQ(plan.waypoints.back().y, position.y);
  EXPECT_EQ(plan.waypoints.back().z, apex);
  EXPECT_EQ(plan.hold_point.z, apex);
  EXPECT_NEAR(plan.climb_m, apex - position.z, 1e-12);
}

TEST(RecoveryPlanner, ReturnRefusesWhenHomeWasNeverReceivedEvenThoughOriginLooksValid)
{
  const RecoveryPlanner planner = plannerWith();
  const RecoveryLimits limits = planner.limits();
  const Vec3 position{12.0, 5.0, 3.0};
  const double cruise = position.z + limits.min_home_clearance_m;

  RecoveryState state = stateAt(position);
  state.home = Vec3{0.0, 0.0, 0.0};     // the launch point, and still not a home
  ASSERT_FALSE(state.home_known);

  const RecoveryPlan plan = planner.plan(requestFor(RecoveryType::ReturnHome, cruise), state);
  expectRefused(plan);
  EXPECT_TRUE(mentions(plan.reason, "never been received")) << plan.reason;
}

TEST(RecoveryPlanner, TheSameReturnIsAcceptedOnceHomeIsKnown)
{
  const RecoveryPlanner planner = plannerWith();
  const RecoveryLimits limits = planner.limits();
  const Vec3 position{12.0, 5.0, 3.0};
  const Vec3 home{0.0, 0.0, 0.0};
  const double cruise = position.z + limits.min_home_clearance_m;

  const RecoveryPlan plan =
    planner.plan(requestFor(RecoveryType::ReturnHome, cruise), stateWithHome(position, home));

  ASSERT_TRUE(plan.accepted) << plan.reason;
  EXPECT_EQ(plan.waypoints.back().x, home.x);
  EXPECT_EQ(plan.waypoints.back().y, home.y);
}

TEST(RecoveryPlanner, ReturnRefusesANonFiniteHome)
{
  const RecoveryPlanner planner = plannerWith();
  const RecoveryLimits limits = planner.limits();

  const RecoveryPlan plan = planner.plan(
    requestFor(RecoveryType::ReturnHome, limits.min_home_clearance_m + 2.0),
    stateWithHome(Vec3{1.0, 1.0, 2.0}, Vec3{kNaN, 0.0, 0.0}));

  expectRefused(plan);
  EXPECT_TRUE(mentions(plan.reason, "home is not finite")) << plan.reason;
}

TEST(RecoveryPlanner, ReturnRefusesANonFiniteReturnAltitude)
{
  const RecoveryPlanner planner = plannerWith();
  const RecoveryPlan plan = planner.plan(
    requestFor(RecoveryType::ReturnHome, kNaN),
    stateWithHome(Vec3{5.0, 0.0, 2.0}, Vec3{0.0, 0.0, 0.0}));

  expectRefused(plan);
  EXPECT_TRUE(mentions(plan.reason, "return altitude is not finite")) << plan.reason;
}

TEST(RecoveryPlanner, ReturnRefusesToDescendOnTheWayHome)
{
  const RecoveryPlanner planner = plannerWith();
  const RecoveryLimits limits = planner.limits();

  const Vec3 home{10.0, 0.0, 0.0};
  const Vec3 position{0.0, 0.0, 10.0};
  // Clearance and ceiling satisfied, so only descent fires.
  const double cruise = 5.0;
  ASSERT_GT(cruise - home.z, limits.min_home_clearance_m);
  ASSERT_LT(cruise, limits.max_safe_altitude_m);

  const RecoveryPlan plan =
    planner.plan(requestFor(RecoveryType::ReturnHome, cruise), stateWithHome(position, home));

  expectRefused(plan);
  EXPECT_TRUE(mentions(plan.reason, "never descends")) << plan.reason;
}

TEST(RecoveryPlanner, ReturnRefusesToArriveTooLowOverHome)
{
  const RecoveryPlanner planner = plannerWith();
  const RecoveryLimits limits = planner.limits();

  // Home sits high, the aircraft just under the cruise.
  const Vec3 home{8.0, 0.0, 5.0};
  const double cruise = home.z + limits.min_home_clearance_m * 0.5;
  const Vec3 position{0.0, 0.0, cruise - limits.min_travel_m};

  const RecoveryPlan plan =
    planner.plan(requestFor(RecoveryType::ReturnHome, cruise), stateWithHome(position, home));

  expectRefused(plan);
  EXPECT_TRUE(mentions(plan.reason, "above home")) << plan.reason;
}

TEST(RecoveryPlanner, ReturnRefusesACruiseAboveTheCeilingWhileTheStepIsLegal)
{
  RecoveryLimits limits;
  const RecoveryPlanner planner = plannerWith(limits);

  const double cruise = limits.max_safe_altitude_m + 1.0;
  const Vec3 position{0.0, 0.0, cruise - limits.max_climb_m * 0.5};
  const Vec3 home{10.0, 0.0, 0.0};

  const RecoveryPlan plan =
    planner.plan(requestFor(RecoveryType::ReturnHome, cruise), stateWithHome(position, home));

  expectRefused(plan);
  EXPECT_TRUE(mentions(plan.reason, "ceiling")) << plan.reason;
}

TEST(RecoveryPlanner, ReturnRefusesAStepPastTheCapWhileTheCruiseIsLegal)
{
  RecoveryLimits limits;
  const RecoveryPlanner planner = plannerWith(limits);

  const double cruise = limits.max_safe_altitude_m;
  const Vec3 position{0.0, 0.0, cruise - limits.max_climb_m * 1.5};
  const Vec3 home{10.0, 0.0, 0.0};

  const RecoveryPlan plan =
    planner.plan(requestFor(RecoveryType::ReturnHome, cruise), stateWithHome(position, home));

  expectRefused(plan);
  EXPECT_TRUE(mentions(plan.reason, "single-step cap")) << plan.reason;
}

TEST(RecoveryPlanner, ReturnRefusesAHomeBeyondRecoveryRange)
{
  RecoveryLimits limits;
  const RecoveryPlanner planner = plannerWith(limits);

  const double cruise = limits.min_home_clearance_m + 4.0;
  const Vec3 position{0.0, 0.0, cruise};
  const Vec3 home{limits.max_home_distance_m + 1.0, 0.0, 0.0};

  const RecoveryPlan plan =
    planner.plan(requestFor(RecoveryType::ReturnHome, cruise), stateWithHome(position, home));

  expectRefused(plan);
  EXPECT_TRUE(mentions(plan.reason, "recovery range")) << plan.reason;
}

TEST(RecoveryPlanner, ReturnRefusesWhenThereIsNothingLeftToFly)
{
  RecoveryLimits limits;
  const RecoveryPlanner planner = plannerWith(limits);

  const double cruise = limits.min_home_clearance_m + 2.0;
  const Vec3 position{0.0, 0.0, cruise};
  const Vec3 home{limits.min_travel_m * 0.5, 0.0, 0.0};

  const RecoveryPlan plan =
    planner.plan(requestFor(RecoveryType::ReturnHome, cruise), stateWithHome(position, home));

  expectRefused(plan);
  EXPECT_TRUE(mentions(plan.reason, "already over home")) << plan.reason;
}

TEST(RecoveryPlanner, ReturnClimbsFirstThenCrossesAndHoldsAboveHome)
{
  const RecoveryPlanner planner = plannerWith();
  const RecoveryLimits limits = planner.limits();

  const Vec3 position{20.0, -10.0, 2.0};
  const Vec3 home{1.0, 2.0, 0.5};
  const double cruise = position.z + limits.min_travel_m * 4.0;

  const RecoveryPlan plan =
    planner.plan(requestFor(RecoveryType::ReturnHome, cruise), stateWithHome(position, home));

  ASSERT_TRUE(plan.accepted) << plan.reason;
  ASSERT_EQ(plan.waypoints.size(), 3u);

  EXPECT_EQ(plan.waypoints[0].x, position.x);
  EXPECT_EQ(plan.waypoints[0].y, position.y);
  EXPECT_EQ(plan.waypoints[0].z, position.z);

  EXPECT_EQ(plan.waypoints[1].x, position.x);
  EXPECT_EQ(plan.waypoints[1].y, position.y);
  EXPECT_EQ(plan.waypoints[1].z, cruise);

  EXPECT_EQ(plan.waypoints[2].x, home.x);
  EXPECT_EQ(plan.waypoints[2].y, home.y);
  EXPECT_EQ(plan.waypoints[2].z, cruise);

  // Above home, never at it: this planner does not land.
  EXPECT_GT(plan.waypoints.back().z, home.z);
  EXPECT_EQ(plan.hold_point.z, cruise);
  EXPECT_NEAR(plan.home_distance_m, std::hypot(position.x - home.x, position.y - home.y), 1e-9);
}

TEST(RecoveryPlanner, ReturnSkipsTheClimbLegWhenThereIsNoClimbToMake)
{
  const RecoveryPlanner planner = plannerWith();
  const RecoveryLimits limits = planner.limits();

  const Vec3 home{0.0, 0.0, 0.0};
  const double cruise = limits.min_home_clearance_m + 3.0;
  const Vec3 position{15.0, 0.0, cruise};

  const RecoveryPlan plan =
    planner.plan(requestFor(RecoveryType::ReturnHome, cruise), stateWithHome(position, home));

  ASSERT_TRUE(plan.accepted) << plan.reason;
  ASSERT_EQ(plan.waypoints.size(), 2u);
  EXPECT_EQ(plan.waypoints.front().z, position.z);
  EXPECT_EQ(plan.waypoints.back().z, cruise);
  EXPECT_EQ(plan.climb_m, 0.0);
}

TEST(RecoveryPlanner, EveryAcceptedPlanStartsWhereTheAircraftIsAndStaysFinite)
{
  const RecoveryPlanner planner = plannerWith();
  const RecoveryLimits limits = planner.limits();
  const Vec3 position{6.0, 4.0, 2.0};
  const Vec3 home{0.0, 0.0, 0.0};
  const double altitude = position.z + limits.min_travel_m * 5.0;

  for (RecoveryType type : {RecoveryType::ClimbToSafe, RecoveryType::ReturnHome}) {
    const RecoveryPlan plan =
      planner.plan(requestFor(type, altitude), stateWithHome(position, home));
    ASSERT_TRUE(plan.accepted) << plan.reason;
    ASSERT_FALSE(plan.waypoints.empty());
    EXPECT_EQ(plan.waypoints.front().x, position.x);
    EXPECT_EQ(plan.waypoints.front().y, position.y);
    EXPECT_EQ(plan.waypoints.front().z, position.z);
    for (const Vec3 & waypoint : plan.waypoints) {
      EXPECT_TRUE(isFinite(waypoint));
    }
    EXPECT_TRUE(isFinite(plan.hold_point));
  }
}

TEST(RecoveryPlanner, AcceptedPlansFeedTheOrdinaryTrajectoryCore)
{
  const RecoveryPlanner planner = plannerWith();
  const RecoveryLimits limits = planner.limits();
  const Vec3 position{20.0, -12.0, 2.0};
  const Vec3 home{0.0, 0.0, 0.0};
  const double altitude = position.z + limits.min_travel_m * 6.0;

  const TrajectoryLimits shape;
  for (RecoveryType type : {RecoveryType::ClimbToSafe, RecoveryType::ReturnHome}) {
    const RecoveryPlan plan =
      planner.plan(requestFor(type, altitude), stateWithHome(position, home));
    ASSERT_TRUE(plan.accepted) << plan.reason;

    std::string reason;
    const Trajectory flight = Trajectory::build(plan.waypoints, 0.0, shape, reason);
    ASSERT_TRUE(flight.valid()) << recoveryTypeName(type) << ": " << reason;
    EXPECT_GT(flight.duration(), 0.0);
    EXPECT_LE(flight.peakSpeed(), shape.max_speed * 1.02);

    const Vec3 landed = flight.finalPosition();
    EXPECT_NEAR(landed.x, plan.hold_point.x, 1e-6);
    EXPECT_NEAR(landed.y, plan.hold_point.y, 1e-6);
    EXPECT_NEAR(landed.z, plan.hold_point.z, 1e-6);
  }
}

TEST(RecoveryPlanner, TheWidestAcceptedReturnIsStillFlyableByTheCore)
{
  const RecoveryPlanner planner = plannerWith();
  const RecoveryLimits limits = planner.limits();

  const Vec3 home{0.0, 0.0, 0.0};
  const Vec3 position{limits.max_home_distance_m * 0.999, 0.0, 2.0};
  const double cruise = position.z + limits.min_travel_m * 4.0;

  const RecoveryPlan plan =
    planner.plan(requestFor(RecoveryType::ReturnHome, cruise), stateWithHome(position, home));
  ASSERT_TRUE(plan.accepted) << plan.reason;

  std::string reason;
  const TrajectoryLimits shape;
  const Trajectory flight = Trajectory::build(plan.waypoints, 0.0, shape, reason);
  ASSERT_TRUE(flight.valid()) << reason;
  EXPECT_LT(flight.duration(), shape.max_duration);
}

TEST(RecoveryPlanner, TypeNamesAreDistinct)
{
  EXPECT_STRNE(recoveryTypeName(RecoveryType::Hover), recoveryTypeName(RecoveryType::ClimbToSafe));
  EXPECT_STRNE(
    recoveryTypeName(RecoveryType::ClimbToSafe), recoveryTypeName(RecoveryType::ReturnHome));
  EXPECT_STREQ(recoveryTypeName(RecoveryType::Unknown), "unknown");
}
