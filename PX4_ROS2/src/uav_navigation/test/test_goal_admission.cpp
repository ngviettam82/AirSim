#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "uav_navigation/goal_admission.hpp"

using uav_navigation::Admission;
using uav_navigation::admitFollowPath;
using uav_navigation::admitGoto;
using uav_navigation::admitHold;
using uav_navigation::admitLand;
using uav_navigation::admitRecover;
using uav_navigation::admitTakeoff;
using uav_navigation::admitTrackTarget;
using uav_navigation::FollowPathRequest;
using uav_navigation::GotoRequest;
using uav_navigation::HoldRequest;
using uav_navigation::LandRequest;
using uav_navigation::MotionLimits;
using uav_navigation::NavigatorState;
using uav_navigation::RecoverRequest;
using uav_navigation::RejectReason;
using uav_navigation::resolveGotoLimits;
using uav_navigation::TakeoffRequest;
using uav_navigation::TrackRequest;
using uav_navigation::Vec3;

namespace
{

const char kOdom[] = "odom";

MotionLimits shippedLimits()
{
  return MotionLimits{0.55, 0.45, 0.3, 10.0, 0.5, 20.0};
}

GotoRequest validGoto()
{
  GotoRequest request;
  request.x = 1.5;
  request.y = 1.0;
  request.z = 2.5;
  request.frame_id = kOdom;
  request.acceptance_radius = 0.3;
  request.max_speed = 1.0;
  return request;
}

const std::vector<NavigatorState> kAllStates{
  NavigatorState::IDLE,
  NavigatorState::TAKING_OFF,
  NavigatorState::GOING_TO_POSE,
  NavigatorState::HOLD_TASK,
  NavigatorState::HOLDING,
  NavigatorState::LANDING,
};

}  // namespace

TEST(GoalAdmission, OnlyIdleAndHoldingEverAcceptAGoal)
{
  for (const NavigatorState state : kAllStates) {
    const bool settled =
      state == NavigatorState::IDLE || state == NavigatorState::HOLDING;
    EXPECT_EQ(uav_navigation::acceptsNewGoal(state), settled)
      << "state " << uav_navigation::toString(state);
  }
}

TEST(GoalAdmission, FullMatrixOfStateVersusTask)
{
  const MotionLimits limits = shippedLimits();
  for (const NavigatorState state : kAllStates) {
    const bool idle = state == NavigatorState::IDLE;
    const bool holding = state == NavigatorState::HOLDING;

    EXPECT_EQ(admitTakeoff(state, TakeoffRequest{2.5}, limits).accepted, idle)
      << "takeoff from " << uav_navigation::toString(state);
    EXPECT_EQ(admitGoto(state, validGoto(), limits, kOdom).accepted, holding)
      << "goto from " << uav_navigation::toString(state);
    EXPECT_EQ(admitHold(state, HoldRequest{5.0}).accepted, holding)
      << "hold from " << uav_navigation::toString(state);
    EXPECT_EQ(admitLand(state, LandRequest{false}).accepted, holding)
      << "land from " << uav_navigation::toString(state);
  }
}

TEST(GoalAdmission, BusyStatesReportBusyNotTheGoalContent)
{
  const MotionLimits limits = shippedLimits();
  GotoRequest request = validGoto();
  request.frame_id = "map";

  const Admission verdict =
    admitGoto(NavigatorState::GOING_TO_POSE, request, limits, kOdom);
  EXPECT_FALSE(verdict.accepted);
  EXPECT_EQ(verdict.reason, RejectReason::BUSY);
}

TEST(GoalAdmission, TakeoffFromHoldingIsRefusedAsAlreadyAirborne)
{
  const Admission verdict =
    admitTakeoff(NavigatorState::HOLDING, TakeoffRequest{2.5}, shippedLimits());
  EXPECT_FALSE(verdict.accepted);
  EXPECT_EQ(verdict.reason, RejectReason::ALREADY_AIRBORNE);
}

TEST(GoalAdmission, TakeoffRejectsNonFiniteAltitude)
{
  const MotionLimits limits = shippedLimits();
  EXPECT_EQ(
    admitTakeoff(
      NavigatorState::IDLE,
      TakeoffRequest{std::numeric_limits<double>::quiet_NaN()}, limits).reason,
    RejectReason::NOT_FINITE);
  EXPECT_EQ(
    admitTakeoff(
      NavigatorState::IDLE,
      TakeoffRequest{std::numeric_limits<double>::infinity()}, limits).reason,
    RejectReason::NOT_FINITE);
}

TEST(GoalAdmission, TakeoffRejectsZeroNegativeAndCeilingBreakingAltitude)
{
  const MotionLimits limits = shippedLimits();
  EXPECT_EQ(
    admitTakeoff(NavigatorState::IDLE, TakeoffRequest{0.0}, limits).reason,
    RejectReason::OUT_OF_RANGE);
  EXPECT_EQ(
    admitTakeoff(NavigatorState::IDLE, TakeoffRequest{-2.0}, limits).reason,
    RejectReason::OUT_OF_RANGE);
  EXPECT_EQ(
    admitTakeoff(NavigatorState::IDLE, TakeoffRequest{10.001}, limits).reason,
    RejectReason::OUT_OF_RANGE);
  EXPECT_TRUE(admitTakeoff(NavigatorState::IDLE, TakeoffRequest{10.0}, limits).accepted);
}

TEST(GoalAdmission, GotoBeforeTakeoffIsRefusedAsNotAirborne)
{
  const Admission verdict =
    admitGoto(NavigatorState::IDLE, validGoto(), shippedLimits(), kOdom);
  EXPECT_FALSE(verdict.accepted);
  EXPECT_EQ(verdict.reason, RejectReason::NOT_AIRBORNE);
}

TEST(GoalAdmission, GotoRejectsAnyFrameThatIsNotOdom)
{
  const MotionLimits limits = shippedLimits();
  for (const char * frame : {"map", "base_link", "", "Odom", "odom "}) {
    GotoRequest request = validGoto();
    request.frame_id = frame;
    const Admission verdict = admitGoto(NavigatorState::HOLDING, request, limits, kOdom);
    EXPECT_FALSE(verdict.accepted) << "frame " << frame;
    EXPECT_EQ(verdict.reason, RejectReason::INVALID_FRAME) << "frame " << frame;
  }
}

TEST(GoalAdmission, GotoRejectsNonFiniteTarget)
{
  const MotionLimits limits = shippedLimits();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  GotoRequest bad_x = validGoto();
  bad_x.x = nan;
  EXPECT_EQ(
    admitGoto(NavigatorState::HOLDING, bad_x, limits, kOdom).reason, RejectReason::NOT_FINITE);

  GotoRequest bad_y = validGoto();
  bad_y.y = inf;
  EXPECT_EQ(
    admitGoto(NavigatorState::HOLDING, bad_y, limits, kOdom).reason, RejectReason::NOT_FINITE);

  GotoRequest bad_z = validGoto();
  bad_z.z = nan;
  EXPECT_EQ(
    admitGoto(NavigatorState::HOLDING, bad_z, limits, kOdom).reason, RejectReason::NOT_FINITE);

  GotoRequest bad_radius = validGoto();
  bad_radius.acceptance_radius = nan;
  EXPECT_EQ(
    admitGoto(NavigatorState::HOLDING, bad_radius, limits, kOdom).reason,
    RejectReason::NOT_FINITE);

  GotoRequest bad_speed = validGoto();
  bad_speed.max_speed = inf;
  EXPECT_EQ(
    admitGoto(NavigatorState::HOLDING, bad_speed, limits, kOdom).reason,
    RejectReason::NOT_FINITE);
}

TEST(GoalAdmission, GotoRejectsTargetsOutsideTheAltitudeEnvelope)
{
  const MotionLimits limits = shippedLimits();

  GotoRequest low = validGoto();
  low.z = 0.4;
  EXPECT_EQ(
    admitGoto(NavigatorState::HOLDING, low, limits, kOdom).reason, RejectReason::OUT_OF_RANGE);

  GotoRequest underground = validGoto();
  underground.z = -3.0;
  EXPECT_EQ(
    admitGoto(NavigatorState::HOLDING, underground, limits, kOdom).reason,
    RejectReason::OUT_OF_RANGE);

  GotoRequest high = validGoto();
  high.z = 20.1;
  EXPECT_EQ(
    admitGoto(NavigatorState::HOLDING, high, limits, kOdom).reason, RejectReason::OUT_OF_RANGE);
}

TEST(GoalAdmission, GotoFromHoldingWithAValidGoalIsAccepted)
{
  const Admission verdict =
    admitGoto(NavigatorState::HOLDING, validGoto(), shippedLimits(), kOdom);
  EXPECT_TRUE(verdict.accepted);
  EXPECT_EQ(verdict.reason, RejectReason::NONE);
  EXPECT_TRUE(verdict.detail.empty());
}

TEST(GoalAdmission, UnsetAcceptanceRadiusAndSpeedFallBackToTheConfiguredDefaults)
{
  const MotionLimits limits = shippedLimits();
  GotoRequest request = validGoto();
  request.acceptance_radius = 0.0;
  request.max_speed = 0.0;

  const auto resolved = resolveGotoLimits(request, limits);
  EXPECT_DOUBLE_EQ(resolved.acceptance_radius, 0.3);
  EXPECT_DOUBLE_EQ(resolved.max_speed, 0.55);

  request.acceptance_radius = -1.0;
  request.max_speed = -2.0;
  const auto negative = resolveGotoLimits(request, limits);
  EXPECT_DOUBLE_EQ(negative.acceptance_radius, 0.3);
  EXPECT_DOUBLE_EQ(negative.max_speed, 0.55);
}

TEST(GoalAdmission, GoalSpeedNeverExceedsTheConfiguredCeiling)
{
  const MotionLimits limits = shippedLimits();
  GotoRequest request = validGoto();
  request.max_speed = 9.0;
  EXPECT_DOUBLE_EQ(resolveGotoLimits(request, limits).max_speed, 0.55);

  request.max_speed = 0.5;
  EXPECT_DOUBLE_EQ(resolveGotoLimits(request, limits).max_speed, 0.5);
}

TEST(GoalAdmission, HoldAcceptsZeroDurationAndRejectsNegative)
{
  EXPECT_TRUE(admitHold(NavigatorState::HOLDING, HoldRequest{0.0}).accepted);
  EXPECT_EQ(
    admitHold(NavigatorState::HOLDING, HoldRequest{-1.0}).reason, RejectReason::OUT_OF_RANGE);
  EXPECT_EQ(
    admitHold(
      NavigatorState::HOLDING,
      HoldRequest{std::numeric_limits<double>::quiet_NaN()}).reason,
    RejectReason::NOT_FINITE);
}

TEST(GoalAdmission, LandRefusesPrecisionLandingUntilItExists)
{
  const Admission verdict = admitLand(NavigatorState::HOLDING, LandRequest{true});
  EXPECT_FALSE(verdict.accepted);
  EXPECT_EQ(verdict.reason, RejectReason::UNSUPPORTED);
}

TEST(GoalAdmission, LandOnTheGroundIsRefused)
{
  const Admission verdict = admitLand(NavigatorState::IDLE, LandRequest{false});
  EXPECT_FALSE(verdict.accepted);
  EXPECT_EQ(verdict.reason, RejectReason::NOT_AIRBORNE);
}

TEST(GoalAdmission, EveryRejectionCarriesAReadableDetail)
{
  const MotionLimits limits = shippedLimits();
  GotoRequest request = validGoto();
  request.frame_id = "map";

  const Admission verdict = admitGoto(NavigatorState::HOLDING, request, limits, kOdom);
  EXPECT_FALSE(verdict.accepted);
  EXPECT_NE(verdict.detail.find("map"), std::string::npos);
  EXPECT_NE(verdict.detail.find("odom"), std::string::npos);
}


// ---------------------------------------------------------------- follow path

namespace
{

FollowPathRequest validPath()
{
  FollowPathRequest request;
  request.frame_id = kOdom;
  request.path_is_valid = true;
  request.max_speed = 0.5;
  request.acceptance_radius = 0.4;
  request.waypoints = {Vec3{1.0, 0.0, 2.0}, Vec3{2.0, 1.0, 2.0}, Vec3{3.0, 0.0, 2.0}};
  return request;
}

TrackRequest validTrack()
{
  TrackRequest request;
  request.standoff_distance = 2.0;
  request.duration_seconds = 10.0;
  request.target_lost_timeout = 2.0;
  return request;
}

/// A climb: the node has already resolved the type, which is where LAND and
/// HANDOVER_TO_PILOT are refused by name.
RecoverRequest validRecover()
{
  RecoverRequest request;
  request.type_executable = true;
  request.reads_altitude = true;
  request.safe_altitude = 5.0;
  return request;
}

}  // namespace

TEST(GoalAdmission, AWellFormedPathIsAcceptedWhileHovering)
{
  const Admission verdict =
    admitFollowPath(NavigatorState::HOLDING, validPath(), shippedLimits(), kOdom);
  EXPECT_TRUE(verdict.accepted) << verdict.detail;
}

TEST(GoalAdmission, EveryUnusablePathIsRefusedAndNamed)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  struct Case
  {
    const char * label;
    FollowPathRequest request;
    RejectReason reason;
  };

  std::vector<Case> cases;
  FollowPathRequest request = validPath();
  request.frame_id = "map";
  cases.push_back({"foreign frame", request, RejectReason::INVALID_FRAME});

  request = validPath();
  request.path_is_valid = false;
  cases.push_back({"path not marked valid", request, RejectReason::UNSUPPORTED});

  request = validPath();
  request.waypoints.clear();
  cases.push_back({"no waypoints", request, RejectReason::OUT_OF_RANGE});

  request = validPath();
  request.waypoints.assign(uav_navigation::kMaxPathWaypoints + 1, Vec3{1.0, 0.0, 2.0});
  cases.push_back({"more waypoints than the core plans", request, RejectReason::OUT_OF_RANGE});

  request = validPath();
  request.waypoints[1].y = nan;
  cases.push_back({"waypoint not finite", request, RejectReason::NOT_FINITE});

  request = validPath();
  request.max_speed = nan;
  cases.push_back({"max_speed not finite", request, RejectReason::NOT_FINITE});

  request = validPath();
  request.waypoints[1].z = 0.1;
  cases.push_back({"waypoint under the floor", request, RejectReason::OUT_OF_RANGE});

  request = validPath();
  request.waypoints[1].z = 50.0;
  cases.push_back({"waypoint over the ceiling", request, RejectReason::OUT_OF_RANGE});

  for (const Case & entry : cases) {
    const Admission verdict =
      admitFollowPath(NavigatorState::HOLDING, entry.request, shippedLimits(), kOdom);
    EXPECT_FALSE(verdict.accepted) << entry.label << " was accepted";
    EXPECT_EQ(verdict.reason, entry.reason) << entry.label;
    EXPECT_FALSE(verdict.detail.empty()) << entry.label << " was refused without a reason";
  }
}

TEST(GoalAdmission, APathIsRefusedOnTheGroundAndWhileAnotherTaskRuns)
{
  EXPECT_FALSE(
    admitFollowPath(NavigatorState::IDLE, validPath(), shippedLimits(), kOdom).accepted);
  EXPECT_FALSE(
    admitFollowPath(
      NavigatorState::GOING_TO_POSE, validPath(), shippedLimits(), kOdom).accepted);
}

// --------------------------------------------------------------- track target

TEST(GoalAdmission, AWellFormedTrackIsAcceptedWhileHovering)
{
  const Admission verdict =
    admitTrackTarget(NavigatorState::HOLDING, validTrack(), shippedLimits());
  EXPECT_TRUE(verdict.accepted) << verdict.detail;
}

TEST(GoalAdmission, EveryUnusableTrackIsRefusedAndNamed)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  struct Case
  {
    const char * label;
    TrackRequest request;
    RejectReason reason;
  };

  std::vector<Case> cases;
  TrackRequest request = validTrack();
  request.standoff_distance = 0.0;
  cases.push_back({"zero standoff", request, RejectReason::OUT_OF_RANGE});

  request = validTrack();
  request.standoff_distance = nan;
  cases.push_back({"standoff not finite", request, RejectReason::NOT_FINITE});

  request = validTrack();
  request.duration_seconds = -1.0;
  cases.push_back({"negative duration", request, RejectReason::OUT_OF_RANGE});

  request = validTrack();
  request.target_lost_timeout = 0.0;
  cases.push_back({"zero lost timeout", request, RejectReason::OUT_OF_RANGE});

  for (const Case & entry : cases) {
    const Admission verdict =
      admitTrackTarget(NavigatorState::HOLDING, entry.request, shippedLimits());
    EXPECT_FALSE(verdict.accepted) << entry.label << " was accepted";
    EXPECT_EQ(verdict.reason, entry.reason) << entry.label;
    EXPECT_FALSE(verdict.detail.empty()) << entry.label << " was refused without a reason";
  }
}

// ------------------------------------------------------------------- recover

// The one goal that interrupts: every other action is refused as busy in these
// states, which is exactly what a recovery must not be.
TEST(GoalAdmission, ARecoveryIsAcceptedOutOfEveryStateThatIsStillFlying)
{
  const std::vector<NavigatorState> flying{
    NavigatorState::TAKING_OFF, NavigatorState::GOING_TO_POSE, NavigatorState::HOLD_TASK,
    NavigatorState::FOLLOWING_PATH, NavigatorState::TRACKING_TARGET, NavigatorState::HOLDING};
  for (const NavigatorState state : flying) {
    const Admission verdict = admitRecover(state, validRecover());
    EXPECT_TRUE(verdict.accepted)
      << "a recovery was refused in " << uav_navigation::toString(state) << ": " << verdict.detail;
  }
}

TEST(GoalAdmission, ARecoveryIsRefusedOnTheGroundDuringLandingAndDuringAnotherRecovery)
{
  const Admission grounded = admitRecover(NavigatorState::IDLE, validRecover());
  EXPECT_FALSE(grounded.accepted);
  EXPECT_EQ(grounded.reason, RejectReason::NOT_AIRBORNE);

  // The autopilot already owns the aircraft; a recovery would be fighting it.
  const Admission landing = admitRecover(NavigatorState::LANDING, validRecover());
  EXPECT_FALSE(landing.accepted);
  EXPECT_EQ(landing.reason, RejectReason::BUSY);

  const Admission again = admitRecover(NavigatorState::RECOVERING, validRecover());
  EXPECT_FALSE(again.accepted);
  EXPECT_EQ(again.reason, RejectReason::BUSY);
}

TEST(GoalAdmission, ATypeThisNodeMayNotFlyIsRefusedWithTheReasonItWasGiven)
{
  RecoverRequest request = validRecover();
  request.type_executable = false;
  request.refusal = "landing belongs to the safety layer";
  const Admission verdict = admitRecover(NavigatorState::GOING_TO_POSE, request);
  EXPECT_FALSE(verdict.accepted);
  EXPECT_EQ(verdict.reason, RejectReason::UNSUPPORTED);
  EXPECT_EQ(verdict.detail, request.refusal) << "the refusal was replaced by a generic one";

  // Order matters: on the ground the state alone would also refuse, and answering
  // "not airborne" would hide a permanent refusal behind a temporary one - which is
  // exactly what makes a gate pass for the wrong reason.
  const Admission grounded = admitRecover(NavigatorState::IDLE, request);
  EXPECT_EQ(grounded.reason, RejectReason::UNSUPPORTED) << grounded.detail;
  EXPECT_EQ(grounded.detail, request.refusal);
}

TEST(GoalAdmission, AnAltitudeIsDemandedOnlyByTheTypesThatReadIt)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  RecoverRequest climbing = validRecover();
  climbing.safe_altitude = nan;
  const Admission refused = admitRecover(NavigatorState::HOLDING, climbing);
  EXPECT_FALSE(refused.accepted);
  EXPECT_EQ(refused.reason, RejectReason::NOT_FINITE);

  // A hover ignores the altitude, so an unset one must not stand in its way: it is
  // the recovery that still works when nothing else does.
  RecoverRequest hovering = validRecover();
  hovering.reads_altitude = false;
  hovering.safe_altitude = nan;
  EXPECT_TRUE(admitRecover(NavigatorState::HOLDING, hovering).accepted);
}
