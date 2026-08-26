// ROS-free (no domain, no rclcpp::init): the arbitration matrix, the release
// hysteresis and the latch revocation paths are arithmetic, so a test pins
// them exactly instead of trusting a flight -- same rule uav_navigation's
// carrot/costmap/goal_admission tests follow.
#include <cmath>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "uav_control_authority/authority_arbiter.hpp"

namespace uav_control_authority
{
namespace
{

ArbiterParams defaultParams()
{
  return ArbiterParams{};
}

CommandSnapshot positionCommand(uint8_t source, double stamp_sec = 0.0)
{
  CommandSnapshot command;
  command.stamp_sec = stamp_sec;
  command.frame_id = "odom";
  command.control_mode = kModePosition;
  command.source = source;
  command.position[0] = 1.0;
  command.position[1] = 2.0;
  command.position[2] = 3.0;
  command.yaw = 0.5;
  return command;
}

}  // namespace

// ===========================================================================
// Parameter validation: refuse to start on an unusable config (S:3d), same
// discipline as navigator_action_server_node::rejectUnusableTrajectoryLimits.
// ===========================================================================

TEST(ParamValidation, DefaultsAreAccepted)
{
  EXPECT_NO_THROW(AuthorityArbiter arbiter(defaultParams()));
}

TEST(ParamValidation, SourceTimeoutBelowThreeTicksAt20HzIsRejected)
{
  ArbiterParams params = defaultParams();
  params.source_timeout_sec = 0.10;    // < 3/20 = 0.15
  EXPECT_THROW(AuthorityArbiter arbiter(params), std::invalid_argument);
}

TEST(ParamValidation, SourceTimeoutAboveThePairingCeilingIsRejected)
{
  ArbiterParams params = defaultParams();
  params.source_timeout_sec = 0.40;    // > 0.6 * 0.50
  EXPECT_THROW(AuthorityArbiter arbiter(params), std::invalid_argument);
}

TEST(ParamValidation, NonPositiveDownstreamTimeoutIsRejected)
{
  ArbiterParams params = defaultParams();
  params.downstream_command_timeout_sec = 0.0;
  EXPECT_THROW(AuthorityArbiter arbiter(params), std::invalid_argument);
}

TEST(ParamValidation, NonPositiveReleaseDwellIsRejected)
{
  ArbiterParams params = defaultParams();
  params.release_dwell_sec = 0.0;      // the S:6 mutation, caught at construction
  EXPECT_THROW(AuthorityArbiter arbiter(params), std::invalid_argument);
}

TEST(ParamValidation, NegativeReleaseDwellIsRejected)
{
  ArbiterParams params = defaultParams();
  params.release_dwell_sec = -0.05;
  EXPECT_THROW(AuthorityArbiter arbiter(params), std::invalid_argument);
}

TEST(ParamValidation, ReleaseDwellBelowSourceTimeoutIsRejected)
{
  // Y3: otherwise NONE and the just-released channel can chatter at the
  // shared boundary (the exact bug computeWinner's exclude parameter papers
  // over for a single call, but should never be reachable via config at all).
  ArbiterParams params = defaultParams();
  params.source_timeout_sec = 0.20;
  params.release_dwell_sec = 0.15;
  EXPECT_THROW(AuthorityArbiter arbiter(params), std::invalid_argument);
}

TEST(ParamValidation, HandoverBudgetExceedingDownstreamTimeoutIsRejected)
{
  // Y1: release_dwell_sec + 2x monitor period + 1/stream_hz must leave the
  // gateway's 0.5 s stale cutoff a comfortable margin.
  ArbiterParams params = defaultParams();
  params.release_dwell_sec = 0.30;    // 0.30 + 0.10 + 0.05 = 0.45 > 0.8*0.50 = 0.40
  EXPECT_THROW(AuthorityArbiter arbiter(params), std::invalid_argument);
}

TEST(ParamValidation, NonPositiveLatchGraceIsRejected)
{
  ArbiterParams params = defaultParams();
  params.latch_grace_sec = 0.0;
  EXPECT_THROW(AuthorityArbiter arbiter(params), std::invalid_argument);
}

TEST(ParamValidation, LatchTimeoutNotAboveGraceIsRejected)
{
  ArbiterParams params = defaultParams();
  params.latch_timeout_sec = params.latch_grace_sec;    // must be strictly greater
  EXPECT_THROW(AuthorityArbiter arbiter(params), std::invalid_argument);
}

TEST(ParamValidation, ZeroMaxCommandAgeIsRejectedEvenWithCheckDisabled)
{
  // R-C1: a limit must never double as its own off switch.
  ArbiterParams params = defaultParams();
  params.check_command_age = false;
  params.max_command_age_sec = 0.0;
  EXPECT_THROW(AuthorityArbiter arbiter(params), std::invalid_argument);
}

TEST(ParamValidation, NonFiniteParamIsRejected)
{
  ArbiterParams params = defaultParams();
  params.release_dwell_sec = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(AuthorityArbiter arbiter(params), std::invalid_argument);
}

TEST(ParamValidation, EmptyOdomFrameIsRejected)
{
  ArbiterParams params = defaultParams();
  params.odom_frame.clear();
  EXPECT_THROW(AuthorityArbiter arbiter(params), std::invalid_argument);
}

// ===========================================================================
// Priority truth table: every ordered pair of {NONE, TEST, MISSION, OPERATOR,
// SAFETY} live at once, winner must be the higher one (5x5, S:6 item 1/G-CA1).
// ===========================================================================

TEST(PriorityTruthTable, HigherPriorityAlwaysWinsOverAnyOtherLiveChannel)
{
  const uint8_t sources[] = {kSourceNone, kSourceTest, kSourceMission, kSourceOperator,
    kSourceSafety};

  for (uint8_t a : sources) {
    for (uint8_t b : sources) {
      SCOPED_TRACE(
        "a=" + std::string(sourceName(a)) + " b=" + std::string(sourceName(b)));
      AuthorityArbiter arbiter(defaultParams());
      double now = 0.0;
      if (a != kSourceNone) {
        arbiter.onMessageArrived(a, positionCommand(a, now), now);
        now += 0.01;
      }
      if (b != kSourceNone) {
        arbiter.onMessageArrived(b, positionCommand(b, now), now);
        now += 0.01;
      }
      const uint8_t expected = std::max(a, b);
      EXPECT_EQ(arbiter.activeSource(), expected);
    }
  }
}

TEST(PriorityTruthTable, NoChannelLiveMeansNoneWithoutAnyMessage)
{
  AuthorityArbiter arbiter(defaultParams());
  EXPECT_EQ(arbiter.activeSource(), kSourceNone);
  EXPECT_EQ(arbiter.onTick(0.0).active_source, kSourceNone);
}

TEST(PriorityTruthTable, SafetyPreemptsAllThreeLowerChannelsAtOnce)
{
  AuthorityArbiter arbiter(defaultParams());
  arbiter.onMessageArrived(kSourceTest, positionCommand(kSourceTest), 0.0);
  arbiter.onMessageArrived(kSourceMission, positionCommand(kSourceMission), 0.01);
  arbiter.onMessageArrived(kSourceOperator, positionCommand(kSourceOperator), 0.02);
  ASSERT_EQ(arbiter.activeSource(), kSourceOperator);

  const ArrivalResult result =
    arbiter.onMessageArrived(kSourceSafety, positionCommand(kSourceSafety), 0.03);
  EXPECT_EQ(arbiter.activeSource(), kSourceSafety);
  EXPECT_TRUE(result.authority_changed);
  EXPECT_EQ(result.action, PublishAction::PUBLISH);
}

// ===========================================================================
// Silence invariant (N-b, Q3): total silence must never produce a setpoint.
// The literal "0 messages reach command_selected" claim needs a wire, so the
// node-level test owns that; this pins the arbiter side of the same claim --
// onTick() alone, driven for a while, must never resolve to anything but NONE.
// ===========================================================================

TEST(SilenceInvariant, AllSourcesSilentMeansNeverAnythingToPublish)
{
  AuthorityArbiter arbiter(defaultParams());
  for (double t = 0.0; t <= 2.0; t += 0.1) {
    EXPECT_EQ(arbiter.onTick(t).active_source, kSourceNone);
  }
  EXPECT_EQ(arbiter.activeSource(), kSourceNone);
}

// ===========================================================================
// Hysteresis: UP preempts on the very first message (no dwell); DOWN needs
// release_dwell_sec of continuous silence, so a single missed tick from the
// active channel never flips active_source (S:3b, S:6 item 5).
// ===========================================================================

TEST(Hysteresis, UpTransitionIsImmediateOnTheFirstMessageNoDwell)
{
  AuthorityArbiter arbiter(defaultParams());
  arbiter.onMessageArrived(kSourceMission, positionCommand(kSourceMission), 0.0);
  ASSERT_EQ(arbiter.activeSource(), kSourceMission);

  // A single OPERATOR message, no waiting, no prior OPERATOR traffic at all.
  const ArrivalResult result =
    arbiter.onMessageArrived(kSourceOperator, positionCommand(kSourceOperator), 0.05);
  EXPECT_EQ(arbiter.activeSource(), kSourceOperator);
  EXPECT_TRUE(result.authority_changed);
}

TEST(Hysteresis, OneMissedTickFromTheActiveChannelDoesNotFlipActiveSource)
{
  // Shipped release_dwell_sec = source_timeout_sec = 0.20 s = 4 ticks @ 20 Hz.
  // TEST streams throughout (a probe never stops); MISSION is active and skips
  // one tick, i.e. a ~0.10 s gap instead of ~0.05 s -- well inside the window.
  ArbiterParams params = defaultParams();
  AuthorityArbiter arbiter(params);
  arbiter.onMessageArrived(kSourceTest, positionCommand(kSourceTest), 0.0);
  arbiter.onMessageArrived(kSourceMission, positionCommand(kSourceMission), 0.0);
  ASSERT_EQ(arbiter.activeSource(), kSourceMission);

  // The monitor timer runs concurrently with message traffic in the real
  // node; drive both here so this actually exercises the release path
  // instead of passing vacuously because nothing was ever re-evaluated.
  int flips = 0;
  const double t = 0.10;    // one missed 20 Hz tick, safely < 0.20 s dwell
  arbiter.onMessageArrived(kSourceTest, positionCommand(kSourceTest), t);
  if (arbiter.activeSource() != kSourceMission) {
    ++flips;
  }
  arbiter.onTick(t);
  if (arbiter.activeSource() != kSourceMission) {
    ++flips;
  }
  EXPECT_EQ(flips, 0) << "MISSION was displaced before its dwell window elapsed";
  EXPECT_EQ(arbiter.activeSource(), kSourceMission);
}

TEST(Hysteresis, ActiveChannelIsReleasedOncePastDwellAndResumingReclaimsInstantly)
{
  ArbiterParams params = defaultParams();
  AuthorityArbiter arbiter(params);
  arbiter.onMessageArrived(kSourceTest, positionCommand(kSourceTest), 0.0);
  arbiter.onMessageArrived(kSourceMission, positionCommand(kSourceMission), 0.0);
  ASSERT_EQ(arbiter.activeSource(), kSourceMission);

  // MISSION goes fully silent. TEST keeps streaming AND the monitor timer
  // keeps ticking, exactly as both would run concurrently in the real node --
  // the release path only ever runs from onTick() (S:3a).
  double t = 0.0;
  for (int tick = 0; tick < 4; ++tick) {
    t += 0.10;
    arbiter.onMessageArrived(kSourceTest, positionCommand(kSourceTest), t);
    arbiter.onTick(t);
  }
  EXPECT_GE(t, params.release_dwell_sec);
  EXPECT_EQ(arbiter.activeSource(), kSourceTest)
    << "MISSION should have released after " << params.release_dwell_sec << " s of silence";

  // MISSION resumes: immediate reclaim, same up-transition rule as before.
  t += 0.05;
  const ArrivalResult result =
    arbiter.onMessageArrived(kSourceMission, positionCommand(kSourceMission), t);
  EXPECT_EQ(arbiter.activeSource(), kSourceMission);
  EXPECT_TRUE(result.authority_changed);
}

TEST(Hysteresis, ReleaseAlsoWorksThroughOnTickAloneWithNoRivalTraffic)
{
  // Same claim, but driven purely by the monitor timer (no TEST channel at
  // all) -- proves onTick(), not just message arrival, degrades to NONE.
  ArbiterParams params = defaultParams();
  AuthorityArbiter arbiter(params);
  arbiter.onMessageArrived(kSourceMission, positionCommand(kSourceMission), 0.0);
  ASSERT_EQ(arbiter.activeSource(), kSourceMission);

  EXPECT_EQ(arbiter.onTick(params.release_dwell_sec - 0.05).active_source, kSourceMission)
    << "still inside the dwell window";
  const TickResult result = arbiter.onTick(params.release_dwell_sec + 0.05);
  EXPECT_EQ(result.active_source, kSourceNone);
  EXPECT_TRUE(result.authority_changed);
}

// ===========================================================================
// Time boundaries: exactly at the threshold. Symbolic (params.*_sec), so
// these track whatever source_timeout_sec/release_dwell_sec ship as.
// ===========================================================================

TEST(TimeBoundary, ExactlyAtSourceTimeoutIsStillLiveForATakeoverCandidate)
{
  // Isolates isLive()'s own boundary ("<=", not "<") from the active
  // channel's dwell release: TEST is never active here (MISSION immediately
  // preempts it), so what gets checked at t == source_timeout_sec is purely
  // whether TEST still counts as an eligible REPLACEMENT once MISSION's own
  // dwell (same instant, by default) lets go.
  ArbiterParams params = defaultParams();
  AuthorityArbiter arbiter(params);
  arbiter.onMessageArrived(kSourceTest, positionCommand(kSourceTest), 0.0);
  arbiter.onMessageArrived(kSourceMission, positionCommand(kSourceMission), 0.0);
  ASSERT_EQ(arbiter.activeSource(), kSourceMission);

  EXPECT_EQ(arbiter.onTick(params.source_timeout_sec).active_source, kSourceTest)
    << "TEST last arrived exactly source_timeout_sec ago and must still be eligible";
}

TEST(TimeBoundary, JustPastSourceTimeoutDropsACandidateFromEligibility)
{
  ArbiterParams params = defaultParams();
  AuthorityArbiter arbiter(params);
  arbiter.onMessageArrived(kSourceTest, positionCommand(kSourceTest), 0.0);
  EXPECT_EQ(
    arbiter.onTick(params.source_timeout_sec + 0.001).active_source, kSourceNone);
}

TEST(TimeBoundary, ExactlyAtReleaseDwellReleasesTheActiveChannel)
{
  ArbiterParams params = defaultParams();
  AuthorityArbiter arbiter(params);
  arbiter.onMessageArrived(kSourceMission, positionCommand(kSourceMission), 0.0);
  EXPECT_EQ(arbiter.onTick(params.release_dwell_sec).active_source, kSourceNone);
}

TEST(TimeBoundary, JustBeforeReleaseDwellStillHoldsTheActiveChannel)
{
  ArbiterParams params = defaultParams();
  AuthorityArbiter arbiter(params);
  arbiter.onMessageArrived(kSourceMission, positionCommand(kSourceMission), 0.0);
  EXPECT_EQ(
    arbiter.onTick(params.release_dwell_sec - 0.001).active_source, kSourceMission);
}

// ===========================================================================
// Content filter (S:3c): mode first, then finiteness of only the fields the
// mode uses, then frame, then age. Wrong-source is corrected, never dropped.
// ===========================================================================

class ContentFilter : public ::testing::Test
{
protected:
  AuthorityArbiter arbiter_{ArbiterParams{}};
};

TEST_F(ContentFilter, ValidPositionCommandPublishesUnchanged)
{
  const ArrivalResult result =
    arbiter_.onMessageArrived(kSourceMission, positionCommand(kSourceMission, 0.0), 0.0);
  EXPECT_EQ(result.action, PublishAction::PUBLISH);
  EXPECT_EQ(result.drop_reason, DropReason::NONE);
}

TEST_F(ContentFilter, NanInAUsedFieldIsDroppedNotPublished)
{
  CommandSnapshot command = positionCommand(kSourceMission, 0.0);
  command.position[1] = std::numeric_limits<double>::quiet_NaN();
  const ArrivalResult result = arbiter_.onMessageArrived(kSourceMission, command, 0.0);
  EXPECT_EQ(result.action, PublishAction::DROP);
  EXPECT_EQ(result.drop_reason, DropReason::NOT_FINITE);
  EXPECT_EQ(arbiter_.droppedNotFiniteCount(), 1U);
}

TEST_F(ContentFilter, InfInAUsedFieldIsDropped)
{
  CommandSnapshot command = positionCommand(kSourceMission, 0.0);
  command.yaw = std::numeric_limits<double>::infinity();
  const ArrivalResult result = arbiter_.onMessageArrived(kSourceMission, command, 0.0);
  EXPECT_EQ(result.drop_reason, DropReason::NOT_FINITE);
}

TEST_F(ContentFilter, NanInAnUnusedFieldOfTheSameModeIsHarmless)
{
  // POSITION mode never reads velocity/acceleration -- NaN there must not drop.
  CommandSnapshot command = positionCommand(kSourceMission, 0.0);
  command.velocity[0] = std::numeric_limits<double>::quiet_NaN();
  command.acceleration[2] = std::numeric_limits<double>::quiet_NaN();
  const ArrivalResult result = arbiter_.onMessageArrived(kSourceMission, command, 0.0);
  EXPECT_EQ(result.action, PublishAction::PUBLISH);
}

TEST_F(ContentFilter, WrongFrameIsDropped)
{
  CommandSnapshot command = positionCommand(kSourceMission, 0.0);
  command.frame_id = "map";
  const ArrivalResult result = arbiter_.onMessageArrived(kSourceMission, command, 0.0);
  EXPECT_EQ(result.drop_reason, DropReason::WRONG_FRAME);
  EXPECT_EQ(arbiter_.droppedWrongFrameCount(), 1U);
}

// B1 (review 2026-08-21, real bug): a per-channel wrong_frame drop must
// count ONLY on ITS OWN channel -- the pre-B1 aggregate let one stray
// wrong-frame message on an UNRELATED channel latch FRAME_MISMATCH forever
// downstream (uav_safety reads whichever channel currently holds authority).
TEST_F(ContentFilter, WrongFrameCountIsPerChannelNotAggregate)
{
  CommandSnapshot mission_command = positionCommand(kSourceMission, 0.0);
  mission_command.frame_id = "map";
  arbiter_.onMessageArrived(kSourceMission, mission_command, 0.0);

  EXPECT_EQ(arbiter_.droppedWrongFrameCount(kSourceMission), 1U);
  EXPECT_EQ(arbiter_.droppedWrongFrameCount(kSourceOperator), 0U)
    << "an UNRELATED channel's counter must never rise from another channel's error";
  EXPECT_EQ(arbiter_.droppedWrongFrameCount(kSourceTest), 0U);
  EXPECT_EQ(arbiter_.droppedWrongFrameCount(kSourceSafety), 0U);
  EXPECT_EQ(arbiter_.droppedWrongFrameCount(), 1U) << "the aggregate must still count every channel";

  CommandSnapshot operator_command = positionCommand(kSourceOperator, 0.1);
  operator_command.frame_id = "map";
  arbiter_.onMessageArrived(kSourceOperator, operator_command, 0.1);

  EXPECT_EQ(arbiter_.droppedWrongFrameCount(kSourceMission), 1U) << "unaffected by OPERATOR's own error";
  EXPECT_EQ(arbiter_.droppedWrongFrameCount(kSourceOperator), 1U);
  EXPECT_EQ(arbiter_.droppedWrongFrameCount(), 2U);
}

TEST_F(ContentFilter, WrongFrameCountForANonSourceChannelIsZero)
{
  EXPECT_EQ(arbiter_.droppedWrongFrameCount(kSourceNone), 0U);
  EXPECT_EQ(arbiter_.droppedWrongFrameCount(200), 0U);
}

TEST_F(ContentFilter, UnsupportedModeIsDropped)
{
  CommandSnapshot command = positionCommand(kSourceMission, 0.0);
  command.control_mode = 4;    // MODE_ATTITUDE: gateway sends no setpoint for it
  const ArrivalResult result = arbiter_.onMessageArrived(kSourceMission, command, 0.0);
  EXPECT_EQ(result.drop_reason, DropReason::UNSUPPORTED_MODE);
  EXPECT_EQ(arbiter_.droppedModeCount(), 1U);
}

TEST_F(ContentFilter, StaleHeaderStampIsDroppedWhenAgeCheckEnabled)
{
  ArbiterParams params;
  params.max_command_age_sec = 0.5;
  AuthorityArbiter arbiter(params);
  CommandSnapshot command = positionCommand(kSourceMission, /*stamp_sec=*/0.0);
  const ArrivalResult result = arbiter.onMessageArrived(kSourceMission, command, /*now=*/1.0);
  EXPECT_EQ(result.drop_reason, DropReason::STALE);
  EXPECT_EQ(arbiter.droppedStaleCount(), 1U);
}

TEST_F(ContentFilter, StaleHeaderStampIsIgnoredWhenAgeCheckDisabled)
{
  ArbiterParams params;
  params.check_command_age = false;
  params.max_command_age_sec = 0.5;    // must stay positive even though unused (R-C1)
  AuthorityArbiter arbiter(params);
  CommandSnapshot command = positionCommand(kSourceMission, /*stamp_sec=*/0.0);
  const ArrivalResult result = arbiter.onMessageArrived(kSourceMission, command, /*now=*/1.0);
  EXPECT_EQ(result.action, PublishAction::PUBLISH);
}

TEST_F(ContentFilter, WrongSourceFieldIsRestampedNotDropped)
{
  CommandSnapshot command = positionCommand(/*source=*/kSourceTest, 0.0);
  const ArrivalResult result = arbiter_.onMessageArrived(kSourceMission, command, 0.0);
  EXPECT_EQ(result.action, PublishAction::PUBLISH_RESTAMPED);
  EXPECT_EQ(result.drop_reason, DropReason::NONE);
  EXPECT_EQ(arbiter_.restampedCount(), 1U);
}

TEST_F(ContentFilter, NotTheWinnerIsDroppedRegardlessOfContent)
{
  arbiter_.onMessageArrived(kSourceSafety, positionCommand(kSourceSafety), 0.0);
  const ArrivalResult result =
    arbiter_.onMessageArrived(kSourceMission, positionCommand(kSourceMission), 0.01);
  EXPECT_EQ(result.action, PublishAction::DROP);
  EXPECT_EQ(result.drop_reason, DropReason::NOT_WINNER);
}

// ===========================================================================
// Y7: NaN in a USED field of each of the 3 modes drops; NaN in a field that
// mode never reads is harmless -- mirrors publishTrajectorySetpoint()'s exact
// switch (px4_command_gateway_node.cpp:200-216).
// ===========================================================================

namespace
{
struct ModeFieldCase
{
  uint8_t mode;
  const char * name;
};
}  // namespace

class ContentFilterModeTable : public ::testing::TestWithParam<ModeFieldCase>
{
protected:
  AuthorityArbiter arbiter_{ArbiterParams{}};
};

TEST_P(ContentFilterModeTable, UsedFieldNanDropsUnusedFieldNanIsHarmless)
{
  const ModeFieldCase param = GetParam();
  SCOPED_TRACE(param.name);
  const double kNan = std::numeric_limits<double>::quiet_NaN();

  CommandSnapshot used_field_nan = positionCommand(kSourceMission, 0.0);
  used_field_nan.control_mode = param.mode;
  switch (param.mode) {
    case kModePosition: used_field_nan.position[0] = kNan; break;
    case kModeVelocity: used_field_nan.velocity[1] = kNan; break;
    case kModeAcceleration: used_field_nan.acceleration[2] = kNan; break;
  }
  const ArrivalResult dropped = arbiter_.onMessageArrived(kSourceMission, used_field_nan, 0.0);
  EXPECT_EQ(dropped.drop_reason, DropReason::NOT_FINITE) << "a field this mode reads was NaN";

  CommandSnapshot unused_field_nan = positionCommand(kSourceMission, 0.01);
  unused_field_nan.control_mode = param.mode;
  // A field belonging to a DIFFERENT mode's used set -- never read here.
  switch (param.mode) {
    case kModePosition: unused_field_nan.velocity[0] = kNan; break;
    case kModeVelocity: unused_field_nan.position[0] = kNan; break;
    case kModeAcceleration: unused_field_nan.velocity[0] = kNan; break;
  }
  const ArrivalResult published =
    arbiter_.onMessageArrived(kSourceMission, unused_field_nan, 0.01);
  EXPECT_EQ(published.action, PublishAction::PUBLISH) << "a field this mode never reads was NaN";
}

INSTANTIATE_TEST_SUITE_P(
  ThreeModes, ContentFilterModeTable,
  ::testing::Values(
    ModeFieldCase{kModePosition, "POSITION"},
    ModeFieldCase{kModeVelocity, "VELOCITY"},
    ModeFieldCase{kModeAcceleration, "ACCELERATION"}));

// ===========================================================================
// Y7 (Q2 pin): the content filter applies identically on SAFETY -- nothing
// about "this is the emergency channel" waives NaN/frame/mode checks.
// ===========================================================================

class SafetyContentFilter : public ::testing::Test
{
protected:
  AuthorityArbiter arbiter_{ArbiterParams{}};
};

TEST_F(SafetyContentFilter, NanIsDroppedEvenOnSafety)
{
  CommandSnapshot command = positionCommand(kSourceSafety, 0.0);
  command.position[0] = std::numeric_limits<double>::quiet_NaN();
  const ArrivalResult result = arbiter_.onMessageArrived(kSourceSafety, command, 0.0);
  EXPECT_EQ(result.action, PublishAction::DROP);
  EXPECT_EQ(result.drop_reason, DropReason::NOT_FINITE);
}

TEST_F(SafetyContentFilter, WrongFrameIsDroppedEvenOnSafety)
{
  CommandSnapshot command = positionCommand(kSourceSafety, 0.0);
  command.frame_id = "map";
  const ArrivalResult result = arbiter_.onMessageArrived(kSourceSafety, command, 0.0);
  EXPECT_EQ(result.drop_reason, DropReason::WRONG_FRAME);
}

TEST_F(SafetyContentFilter, UnsupportedModeIsDroppedEvenOnSafety)
{
  CommandSnapshot command = positionCommand(kSourceSafety, 0.0);
  command.control_mode = 5;    // MODE_BODY_RATE
  const ArrivalResult result = arbiter_.onMessageArrived(kSourceSafety, command, 0.0);
  EXPECT_EQ(result.drop_reason, DropReason::UNSUPPORTED_MODE);
}

TEST_F(SafetyContentFilter, ValidSafetyCommandStillPublishes)
{
  const ArrivalResult result =
    arbiter_.onMessageArrived(kSourceSafety, positionCommand(kSourceSafety, 0.0), 0.0);
  EXPECT_EQ(result.action, PublishAction::PUBLISH);
}

// ===========================================================================
// Channel enable/disable gating.
// ===========================================================================

TEST(ChannelGating, DisabledTestSourceNeverBecomesActive)
{
  ArbiterParams params = defaultParams();
  params.enable_test_source = false;
  AuthorityArbiter arbiter(params);
  const ArrivalResult result =
    arbiter.onMessageArrived(kSourceTest, positionCommand(kSourceTest), 0.0);
  EXPECT_EQ(result.action, PublishAction::DROP);
  EXPECT_EQ(arbiter.activeSource(), kSourceNone);
}

TEST(ChannelGating, DisabledOperatorSourceNeverBecomesActive)
{
  ArbiterParams params = defaultParams();
  params.enable_operator_source = false;
  AuthorityArbiter arbiter(params);
  arbiter.onMessageArrived(kSourceMission, positionCommand(kSourceMission), 0.0);
  arbiter.onMessageArrived(kSourceOperator, positionCommand(kSourceOperator), 0.01);
  EXPECT_EQ(arbiter.activeSource(), kSourceMission);
}

// ===========================================================================
// B2-d2 (review 2026-08-20): raw arrival age is diagnostics-only, but it must
// still track EVERY message -- valid or not -- so an operator can tell "this
// channel is dead" (age grows without bound) apart from "this channel is
// alive but 100% garbage" (age stays small, yet it never wins arbitration).
// ===========================================================================

TEST(RawArrivalAge, InfinityUntilFirstMessageThenTracksEveryArrivalRegardlessOfContent)
{
  AuthorityArbiter arbiter(defaultParams());
  EXPECT_TRUE(std::isinf(arbiter.rawArrivalAgeSec(kSourceMission, 5.0)))
    << "never arrived at all -- must read as infinitely old, not zero";

  CommandSnapshot garbage = positionCommand(kSourceMission, 0.0);
  garbage.frame_id = "map";    // invalid content -- must still count as a raw arrival
  const ArrivalResult result = arbiter.onMessageArrived(kSourceMission, garbage, 1.0);
  ASSERT_EQ(result.action, PublishAction::DROP) << "garbage is still dropped, as always";
  EXPECT_DOUBLE_EQ(arbiter.rawArrivalAgeSec(kSourceMission, 1.5), 0.5)
    << "raw age must advance on a DROPPED message too -- it is not liveness evidence (B2), "
    << "but it must still be seen for diagnostics";
}

// ===========================================================================
// B2 (review 2026-08-20): garbage must never count as liveness evidence.
// Before this fix, has_arrived_/last_arrival_ (raw arrival) fed isLive(),
// latch_ever_alive_, and the release-dwell clock directly -- a channel could
// win, hold a latch open forever, or keep authority purely by publishing
// fast, whether or not any of it passed validateContent(). These three
// tests pin the structural fix (last_valid_arrival_ is the only clock
// arbitration reads) directly against the three failure modes the review
// named; Y5's demote_after_bad_ticks counter is gone -- with valid-only
// liveness, a channel that is 100% garbage was never live to begin with, so
// it can never win, and a channel that degrades to garbage while holding
// authority simply falls out via release_dwell_sec like any other silence.
// ===========================================================================

TEST(GarbageChannel, HigherPriorityGarbageNeverPreemptsAValidLowerHolder)
{
  // B2(a): OPERATOR floods garbage at the shipped 20 Hz while MISSION --
  // lower priority -- flies with valid content the whole time. Before the
  // fix, OPERATOR's raw arrivals alone made it "live" and its priority (3 >
  // 2) would have preempted MISSION on the very first garbage message.
  AuthorityArbiter arbiter(defaultParams());
  arbiter.onMessageArrived(kSourceMission, positionCommand(kSourceMission), 0.0);
  ASSERT_EQ(arbiter.activeSource(), kSourceMission);

  CommandSnapshot garbage = positionCommand(kSourceOperator, 0.0);
  garbage.frame_id = "map";    // any drop reason exercises the same B2 path

  double t = 0.0;
  for (int tick = 0; tick < 40; ++tick) {    // 2 s at 20 Hz -- far past every timeout
    t += 0.05;
    garbage.stamp_sec = t;
    const ArrivalResult garbage_result = arbiter.onMessageArrived(kSourceOperator, garbage, t);
    EXPECT_EQ(garbage_result.action, PublishAction::DROP);
    ASSERT_EQ(arbiter.activeSource(), kSourceMission)
      << "garbage from OPERATOR must never win, even for one tick, at t=" << t;

    const ArrivalResult mission_result =
      arbiter.onMessageArrived(kSourceMission, positionCommand(kSourceMission, t), t);
    EXPECT_EQ(mission_result.action, PublishAction::PUBLISH)
      << "MISSION must publish on every single tick -- zero gap on the wire, at t=" << t;
  }
}

TEST(GarbageChannel, GarbageFromALiveLatchHolderDoesNotFeedTheLatchClock)
{
  // B2(b): OPERATOR proves itself alive once (latch becomes effective), then
  // floods garbage. Before the fix, last_arrival_ kept advancing on every
  // garbage message, so processLatchExpiry() never saw the latch go stale --
  // a permanent lockout, made worse because Y6 also blocks RELEASE at the
  // SAFETY level (not reachable here, OPERATOR, but the same clock is used).
  ArbiterParams params = defaultParams();
  AuthorityArbiter arbiter(params);
  arbiter.requestLatch(kSourceOperator, 0.0);
  arbiter.onMessageArrived(kSourceOperator, positionCommand(kSourceOperator), 0.1);
  ASSERT_TRUE(arbiter.latchEffective());

  CommandSnapshot garbage = positionCommand(kSourceOperator, 0.0);
  garbage.frame_id = "map";

  double t = 0.1;
  while (t < 0.1 + params.latch_timeout_sec - 0.05) {
    t += 0.05;
    garbage.stamp_sec = t;
    arbiter.onMessageArrived(kSourceOperator, garbage, t);
    arbiter.onTick(t);
  }
  EXPECT_TRUE(arbiter.latchActive())
    << "not yet past latch_timeout_sec measured from the last VALID message";

  t = 0.1 + params.latch_timeout_sec + 0.05;
  garbage.stamp_sec = t;
  arbiter.onMessageArrived(kSourceOperator, garbage, t);
  const TickResult result = arbiter.onTick(t);
  EXPECT_FALSE(arbiter.latchActive())
    << "garbage must never refresh the latch clock -- it must expire on schedule";
  EXPECT_EQ(result.latch_revocation, LatchRevocation::TIMED_OUT);
}

TEST(GarbageChannel, OneValidInTenGarbageStillLosesAuthorityWithinDwellPlusOneTick)
{
  // B2(c): MISSION holds authority, then degrades to 1 valid message per 10
  // (9 garbage in between) at the shipped 20 Hz -- garbage never resets
  // anything, so the gaps between valid messages (~0.45 s) comfortably
  // exceed release_dwell_sec (0.20 s) and MISSION must fall out almost
  // immediately, not linger indefinitely on the strength of one message in
  // ten. Before the fix, setActiveSource() reset a per-channel streak on
  // every valid message and raw arrivals alone kept it "live" regardless.
  ArbiterParams params = defaultParams();
  AuthorityArbiter arbiter(params);
  arbiter.onMessageArrived(kSourceTest, positionCommand(kSourceTest), 0.0);
  arbiter.onMessageArrived(kSourceMission, positionCommand(kSourceMission), 0.0);
  ASSERT_EQ(arbiter.activeSource(), kSourceMission);

  CommandSnapshot garbage = positionCommand(kSourceMission, 0.0);
  garbage.frame_id = "map";

  double t = 0.0;
  double lost_at = -1.0;
  for (int tick = 1; tick <= 60 && lost_at < 0.0; ++tick) {    // 3 s ceiling, generous valve
    t += 0.05;
    if (tick % 10 == 0) {
      arbiter.onMessageArrived(kSourceMission, positionCommand(kSourceMission, t), t);
    } else {
      garbage.stamp_sec = t;
      arbiter.onMessageArrived(kSourceMission, garbage, t);
    }
    arbiter.onTick(t);
    arbiter.onMessageArrived(kSourceTest, positionCommand(kSourceTest, t), t);    // standby, live throughout
    if (arbiter.activeSource() != kSourceMission) {
      lost_at = t;
    }
  }
  ASSERT_GT(lost_at, 0.0) << "MISSION never lost authority under a 1-valid:9-garbage pattern";
  EXPECT_LE(lost_at, params.release_dwell_sec + kMonitorPeriodSec + 0.001)
    << "should fall out within release_dwell_sec of its last VALID message, not linger";
  EXPECT_EQ(arbiter.activeSource(), kSourceTest);
}

// ===========================================================================
// Latch (S:4): grant/reject by priority, floor excludes lower live channels
// ONLY once the holder has proven itself alive (Y4), the three revocation
// paths, and Y6's SAFETY-latch release block.
// ===========================================================================

TEST(Latch, GrantSucceedsWhenNoLatchIsHeld)
{
  AuthorityArbiter arbiter(defaultParams());
  const LatchOutcome outcome = arbiter.requestLatch(kSourceOperator, 0.0);
  EXPECT_TRUE(outcome.granted);
  EXPECT_EQ(outcome.granted_source, kSourceOperator);
  EXPECT_TRUE(arbiter.latchActive());
  EXPECT_EQ(arbiter.latchLevel(), kSourceOperator);
  EXPECT_FALSE(arbiter.latchEffective()) << "granted, but the holder has never published";
}

TEST(Latch, RaisingTheFloorAboveTheCurrentHolderIsGranted)
{
  AuthorityArbiter arbiter(defaultParams());
  ASSERT_TRUE(arbiter.requestLatch(kSourceOperator, 0.0).granted);
  const LatchOutcome outcome = arbiter.requestLatch(kSourceSafety, 0.1);
  EXPECT_TRUE(outcome.granted);
  EXPECT_EQ(arbiter.latchLevel(), kSourceSafety);
}

TEST(Latch, RequestingBelowTheCurrentHolderIsRejected)
{
  AuthorityArbiter arbiter(defaultParams());
  ASSERT_TRUE(arbiter.requestLatch(kSourceOperator, 0.0).granted);
  const LatchOutcome outcome = arbiter.requestLatch(kSourceMission, 0.1);
  EXPECT_FALSE(outcome.granted);
  EXPECT_EQ(outcome.granted_source, kSourceOperator);
  EXPECT_EQ(arbiter.latchLevel(), kSourceOperator)
    << "a rejected request must not disturb the existing latch";
}

TEST(Latch, ExplicitReleaseClearsTheLatch)
{
  AuthorityArbiter arbiter(defaultParams());
  ASSERT_TRUE(arbiter.requestLatch(kSourceOperator, 0.0).granted);
  const LatchOutcome outcome = arbiter.requestLatch(kSourceNone, 0.1);
  EXPECT_TRUE(outcome.granted);
  EXPECT_FALSE(arbiter.latchActive());
  EXPECT_EQ(arbiter.releaseAttemptCount(), 1U);
  EXPECT_EQ(arbiter.releaseRejectedCount(), 0U);
}

TEST(Latch, ReleaseIsAllowedBelowSafetyButRejectedAtSafety)
{
  // Y6: only P8's ClearFault may clear a SAFETY latch.
  {
    AuthorityArbiter arbiter(defaultParams());
    arbiter.requestLatch(kSourceOperator, 0.0);
    const LatchOutcome outcome = arbiter.requestLatch(kSourceNone, 0.1);
    EXPECT_TRUE(outcome.granted);
    EXPECT_FALSE(arbiter.latchActive());
  }
  {
    AuthorityArbiter arbiter(defaultParams());
    arbiter.requestLatch(kSourceSafety, 0.0);
    const LatchOutcome outcome = arbiter.requestLatch(kSourceNone, 0.1);
    EXPECT_FALSE(outcome.granted);
    EXPECT_TRUE(arbiter.latchActive());
    EXPECT_EQ(arbiter.latchLevel(), kSourceSafety);
    EXPECT_EQ(arbiter.releaseAttemptCount(), 1U);
    EXPECT_EQ(arbiter.releaseRejectedCount(), 1U);
  }
}

TEST(Latch, NeverAliveDoesNotExcludeALowerLiveChannel)
{
  // Y4 chốt: a latch whose holder has never published has NO effect -- the
  // "operator latched then grabbed the wrong stick" failure mode must not
  // ground a mission that is actually flying.
  AuthorityArbiter arbiter(defaultParams());
  arbiter.onMessageArrived(kSourceMission, positionCommand(kSourceMission), 0.0);
  ASSERT_EQ(arbiter.activeSource(), kSourceMission);

  arbiter.requestLatch(kSourceOperator, 0.1);
  EXPECT_FALSE(arbiter.latchEffective());
  EXPECT_EQ(arbiter.activeSource(), kSourceMission)
    << "the latch has a holder, but that holder has never proven itself alive";

  arbiter.onMessageArrived(kSourceMission, positionCommand(kSourceMission), 0.2);
  EXPECT_EQ(arbiter.activeSource(), kSourceMission);
}

TEST(Latch, BecomesEffectiveOnceHolderPublishesAndThenPersistsThroughItsOwnSilence)
{
  // Y4's other half: once the holder proves itself alive, the floor DOES
  // exclude lower channels -- and, distinct from ordinary priority, it keeps
  // excluding them even after the holder itself goes quiet, until the
  // latch's own timeout gives up on it (S:4 mechanism 3).
  ArbiterParams params = defaultParams();
  AuthorityArbiter arbiter(params);
  arbiter.onMessageArrived(kSourceTest, positionCommand(kSourceTest), 0.0);
  arbiter.requestLatch(kSourceOperator, 0.0);
  ASSERT_EQ(arbiter.activeSource(), kSourceTest) << "not yet effective";

  arbiter.onMessageArrived(kSourceOperator, positionCommand(kSourceOperator), 0.1);
  ASSERT_TRUE(arbiter.latchEffective());
  ASSERT_EQ(arbiter.activeSource(), kSourceOperator);

  // OPERATOR goes silent; TEST keeps streaming and stays live throughout.
  // Stamp must track t here (unlike the short tests elsewhere in this file):
  // this loop runs well past max_command_age_sec (0.5 s default), so a fixed
  // stamp_sec=0.0 would make TEST's own messages read as STALE and silently
  // stop refreshing last_valid_arrival_ -- exactly the kind of accidental
  // reliance on raw-arrival liveness that B2 was written to no longer allow.
  double t = 0.1;
  while (t < 0.1 + params.release_dwell_sec + 0.05) {
    t += 0.05;
    arbiter.onMessageArrived(kSourceTest, positionCommand(kSourceTest, t), t);
    arbiter.onTick(t);
  }
  EXPECT_EQ(arbiter.activeSource(), kSourceNone)
    << "OPERATOR's own dwell elapsed, but the floor -- still effective -- excludes TEST too";

  // Now run past the latch's own timeout: it gives up, TEST reclaims.
  const double timeout_at = 0.1 + params.latch_timeout_sec + 0.05;
  while (t < timeout_at) {
    t += 0.05;
    arbiter.onMessageArrived(kSourceTest, positionCommand(kSourceTest, t), t);
    arbiter.onTick(t);
  }
  EXPECT_FALSE(arbiter.latchActive());
  EXPECT_EQ(arbiter.activeSource(), kSourceTest);
}

TEST(Latch, SafetyStillWinsOverAnOperatorLatchByPublishing)
{
  // "Latch dung san, khong dung tran": SAFETY always beats a latch below it.
  AuthorityArbiter arbiter(defaultParams());
  arbiter.requestLatch(kSourceOperator, 0.0);
  const ArrivalResult result =
    arbiter.onMessageArrived(kSourceSafety, positionCommand(kSourceSafety), 0.1);
  EXPECT_EQ(arbiter.activeSource(), kSourceSafety);
  EXPECT_EQ(result.action, PublishAction::PUBLISH);
}

TEST(Latch, GraceExpiryRevokesALatchThatNeverPublished)
{
  ArbiterParams params = defaultParams();
  AuthorityArbiter arbiter(params);
  arbiter.requestLatch(kSourceOperator, 0.0);
  ASSERT_TRUE(arbiter.latchActive());

  const TickResult before = arbiter.onTick(params.latch_grace_sec - 0.01);
  EXPECT_TRUE(arbiter.latchActive()) << "grace has not elapsed yet";
  EXPECT_EQ(before.latch_revocation, LatchRevocation::NONE);

  const TickResult after = arbiter.onTick(params.latch_grace_sec + 0.01);
  EXPECT_FALSE(arbiter.latchActive());
  EXPECT_EQ(after.latch_revocation, LatchRevocation::GRACE_EXPIRED);
}

TEST(Latch, TimeoutRevokesALatchThatWasAliveThenDied)
{
  ArbiterParams params = defaultParams();
  AuthorityArbiter arbiter(params);
  arbiter.requestLatch(kSourceOperator, 0.0);
  // OPERATOR proves itself alive well inside the grace window.
  arbiter.onMessageArrived(kSourceOperator, positionCommand(kSourceOperator), 0.5);
  ASSERT_TRUE(arbiter.latchActive());

  const TickResult still_alive = arbiter.onTick(0.5 + params.latch_timeout_sec - 0.01);
  EXPECT_TRUE(arbiter.latchActive());
  EXPECT_EQ(still_alive.latch_revocation, LatchRevocation::NONE);

  const TickResult timed_out = arbiter.onTick(0.5 + params.latch_timeout_sec + 0.01);
  EXPECT_FALSE(arbiter.latchActive());
  EXPECT_EQ(timed_out.latch_revocation, LatchRevocation::TIMED_OUT);
}

TEST(Latch, InvalidRequestedSourceIsRejected)
{
  AuthorityArbiter arbiter(defaultParams());
  const LatchOutcome outcome = arbiter.requestLatch(200, 0.0);
  EXPECT_FALSE(outcome.granted);
}

// ===========================================================================
// P8 R2: the SAFETY exception to Y4. A SAFETY latch is effective (and
// excludes lower channels) with ZERO proof of life -- the deliberate INHIBIT
// mode (policy-1: SAFETY may never publish a setpoint on a broken pose, so it
// can never "prove it's driving" the way OPERATOR/MISSION/TEST can). Every
// Latch.* test above is UNCHANGED and stays green -- it is the regression
// proof that OPERATOR/MISSION/TEST behave exactly as before this addition.
// ===========================================================================

TEST(SafetyLatch, NeverAliveIsEffectiveImmediatelyUnlikeAnyOtherLevel)
{
  AuthorityArbiter arbiter(defaultParams());
  const LatchOutcome outcome = arbiter.requestLatch(kSourceSafety, 0.0);
  ASSERT_TRUE(outcome.granted);
  EXPECT_FALSE(arbiter.latchEverAlive()) << "holder has published nothing";
  EXPECT_TRUE(arbiter.latchEffective())
    << "SAFETY must be effective on zero proof of life -- mutation pin for "
    << "the `|| latch_level_ == kSourceSafety` exception in latchEffective()";
  EXPECT_TRUE(arbiter.inhibitActive());
}

TEST(SafetyLatch, NeverAliveNeverExpiresPastGraceOrTimeout)
{
  // Mutation pin for the SAFETY early-return in processLatchExpiry(): without
  // it, this latch would GRACE_EXPIRE past latch_grace_sec, or -- if only the
  // early-return were removed while the timeout branch ran unconditionally
  // -- TIMED_OUT immediately (last_valid_arrival_ defaults to 0.0 for a
  // channel that has never sent anything at all).
  ArbiterParams params = defaultParams();
  AuthorityArbiter arbiter(params);
  arbiter.requestLatch(kSourceSafety, 0.0);
  ASSERT_TRUE(arbiter.latchActive());

  const TickResult past_grace = arbiter.onTick(params.latch_grace_sec + 0.5);
  EXPECT_TRUE(arbiter.latchActive()) << "GRACE must never apply to a SAFETY latch (INHIBIT)";
  EXPECT_EQ(past_grace.latch_revocation, LatchRevocation::NONE);
  EXPECT_TRUE(arbiter.latchEffective());
  EXPECT_TRUE(arbiter.inhibitActive());

  const TickResult past_timeout = arbiter.onTick(params.latch_timeout_sec + 5.0);
  EXPECT_TRUE(arbiter.latchActive())
    << "a never-alive SAFETY latch must persist indefinitely -- INHIBIT is unbounded by design";
  EXPECT_EQ(past_timeout.latch_revocation, LatchRevocation::NONE);
}

TEST(SafetyLatch, AliveThenSilentSafetyLatchNeverTimesOutEither)
{
  // R5 (coordinator decision, P8.5/G-S1 #8 finding, supersedes the old pin
  // here): a SAFETY latch that DID prove itself alive is STILL exempt from
  // TIMEOUT, not just GRACE -- HOLD proving liveness then deliberately going
  // silent (INHIBIT) must not read as "holder died". Latch.
  // TimeoutRevokesALatchThatWasAliveThenDied (OPERATOR) is the regression
  // proof this is unchanged for the other three levels.
  ArbiterParams params = defaultParams();
  AuthorityArbiter arbiter(params);
  arbiter.requestLatch(kSourceSafety, 0.0);
  arbiter.onMessageArrived(kSourceSafety, positionCommand(kSourceSafety, 0.5), 0.5);
  ASSERT_TRUE(arbiter.latchEverAlive());
  EXPECT_FALSE(arbiter.inhibitActive()) << "alive, so this is not INHIBIT anymore";

  const TickResult still_alive = arbiter.onTick(0.5 + params.latch_timeout_sec - 0.01);
  EXPECT_TRUE(arbiter.latchActive());
  EXPECT_EQ(still_alive.latch_revocation, LatchRevocation::NONE);

  const TickResult past_timeout = arbiter.onTick(0.5 + params.latch_timeout_sec + 0.01);
  EXPECT_TRUE(arbiter.latchActive())
    << "SAFETY latch must persist past latch_timeout_sec even after proving itself alive (R5)";
  EXPECT_EQ(past_timeout.latch_revocation, LatchRevocation::NONE);

  // Even much further out -- not just barely past the boundary.
  const TickResult far_past_timeout = arbiter.onTick(0.5 + params.latch_timeout_sec + 60.0);
  EXPECT_TRUE(arbiter.latchActive());
  EXPECT_EQ(far_past_timeout.latch_revocation, LatchRevocation::NONE);

  // Positive counter-check (R21): only clearSafetyLatch() ever releases it.
  const ClearSafetyLatchOutcome cleared =
    arbiter.clearSafetyLatch(0.5 + params.latch_timeout_sec + 60.0);
  EXPECT_TRUE(cleared.cleared);
  EXPECT_FALSE(arbiter.latchActive());
}

TEST(SafetyLatch, OperatorLatchStillTimesOutNormallyUnaffectedByR5)
{
  // Chống hồi quy tường minh cho R5: mức KHÁC SAFETY vẫn tự huỷ sau
  // latch_timeout_sec y hệt trước -- R5 chỉ chạm nhánh latch_level_==SAFETY.
  // (Latch.TimeoutRevokesALatchThatWasAliveThenDied đã pin ca này từ trước;
  // test này lặp lại có chủ đích trong nhóm SafetyLatch.* để đối chiếu trực
  // tiếp cạnh AliveThenSilentSafetyLatchNeverTimesOutEither ở trên.)
  ArbiterParams params = defaultParams();
  AuthorityArbiter arbiter(params);
  arbiter.requestLatch(kSourceOperator, 0.0);
  arbiter.onMessageArrived(kSourceOperator, positionCommand(kSourceOperator, 0.5), 0.5);
  ASSERT_TRUE(arbiter.latchEverAlive());

  const TickResult still_alive = arbiter.onTick(0.5 + params.latch_timeout_sec - 0.01);
  EXPECT_TRUE(arbiter.latchActive());
  EXPECT_EQ(still_alive.latch_revocation, LatchRevocation::NONE);

  const TickResult timed_out = arbiter.onTick(0.5 + params.latch_timeout_sec + 0.01);
  EXPECT_FALSE(arbiter.latchActive())
    << "R5 must NOT change OPERATOR/MISSION/TEST timeout behavior";
  EXPECT_EQ(timed_out.latch_revocation, LatchRevocation::TIMED_OUT);
}

TEST(SafetyLatch, InhibitFloorExcludesALiveMissionEvenThoughSafetyNeverPublished)
{
  AuthorityArbiter arbiter(defaultParams());
  arbiter.onMessageArrived(kSourceMission, positionCommand(kSourceMission), 0.0);
  ASSERT_EQ(arbiter.activeSource(), kSourceMission);

  arbiter.requestLatch(kSourceSafety, 0.1);
  EXPECT_EQ(arbiter.activeSource(), kSourceNone)
    << "INHIBIT: the floor sits at SAFETY, which nothing MISSION-level clears -- "
    << "and SAFETY itself never publishes, so active_source has nowhere live to land";

  arbiter.onMessageArrived(kSourceMission, positionCommand(kSourceMission), 0.2);
  EXPECT_EQ(arbiter.activeSource(), kSourceNone)
    << "a live MISSION message must still not break through the INHIBIT floor";
}

TEST(SafetyLatch, ClearSafetyLatchClearsAndEnforcesFloorImmediately)
{
  AuthorityArbiter arbiter(defaultParams());
  arbiter.onMessageArrived(kSourceMission, positionCommand(kSourceMission), 0.0);
  arbiter.requestLatch(kSourceSafety, 0.1);
  ASSERT_TRUE(arbiter.inhibitActive());
  ASSERT_EQ(arbiter.activeSource(), kSourceNone);

  const ClearSafetyLatchOutcome outcome = arbiter.clearSafetyLatch(0.2);
  EXPECT_TRUE(outcome.cleared);
  EXPECT_FALSE(arbiter.latchActive());
  EXPECT_FALSE(arbiter.inhibitActive());
  EXPECT_EQ(arbiter.clearSafetyLatchAttemptCount(), 1U);
  EXPECT_EQ(arbiter.clearSafetyLatchSuccessCount(), 1U);

  // MISSION is still live (last valid arrival at t=0.0, well inside
  // source_timeout_sec at t=0.2) -- enforceFloorNow() must hand it back
  // immediately, not wait for the next monitor tick.
  EXPECT_EQ(arbiter.activeSource(), kSourceMission)
    << "MISSION should reclaim within the SAME call that cleared the latch";
}

TEST(SafetyLatch, ClearSafetyLatchIsNoOpSuccessWhenNoSafetyLatchHeldAndLeavesOtherLatchesAlone)
{
  AuthorityArbiter arbiter(defaultParams());

  // No latch at all.
  const ClearSafetyLatchOutcome none_held = arbiter.clearSafetyLatch(0.0);
  EXPECT_TRUE(none_held.cleared);
  EXPECT_FALSE(arbiter.latchActive());

  // A non-SAFETY latch is held -- clearSafetyLatch must not disturb it: the
  // arbiter never judges, it only ever touches the SAFETY level.
  arbiter.requestLatch(kSourceOperator, 0.1);
  ASSERT_TRUE(arbiter.latchActive());
  ASSERT_EQ(arbiter.latchLevel(), kSourceOperator);

  const ClearSafetyLatchOutcome operator_untouched = arbiter.clearSafetyLatch(0.2);
  EXPECT_TRUE(operator_untouched.cleared);
  EXPECT_TRUE(arbiter.latchActive()) << "an OPERATOR latch must survive a clear_safety_latch call";
  EXPECT_EQ(arbiter.latchLevel(), kSourceOperator);
  EXPECT_EQ(arbiter.clearSafetyLatchAttemptCount(), 2U);
  EXPECT_EQ(arbiter.clearSafetyLatchSuccessCount(), 2U);
}

// ===========================================================================
// N-c (P10.8, Q-P10-7) + C4 (P10.8b): a rewound sim clock (bag replay/loop),
// or a stamp from a mismatched time source (e.g. a publisher that forgot
// use_sim_time), must never let "now - stamp" go negative and read as
// "definitely LIVE"/"definitely fresh" forever. ageSecOrInf() is the single
// place EVERY such subtraction in this file goes through -- isLive,
// rawArrivalAgeSec, processLatchExpiry x2, onTick's dwell check (N-c's first
// pass), PLUS validateContent()'s content-age (STALE) guard and
// latchAgeSec() (C4: these two were missed by N-c's first pass and are
// pinned separately below): now < stamp -> +inf (same value as "never
// arrived"), or NaN for latchAgeSec()'s display-only case, + counted in
// clock_regressions_, never silently swallowed.
// ===========================================================================

TEST(ClockRewind, MakesChannelNotLive)
{
  ArbiterParams params = defaultParams();
  AuthorityArbiter arbiter(params);
  arbiter.onMessageArrived(kSourceMission, positionCommand(kSourceMission, 100.0), 100.0);
  ASSERT_EQ(arbiter.activeSource(), kSourceMission);

  // The clock jumps backward past the channel's last valid arrival -- age
  // can no longer be measured, so the channel must NOT read as LIVE.
  const TickResult result = arbiter.onTick(50.0);
  EXPECT_EQ(result.active_source, kSourceNone);
  EXPECT_EQ(arbiter.activeSource(), kSourceNone)
    << "N-c: a clock rewind must never leave a dead channel holding authority forever";

  // Positive counter-check: the SAME channel, message re-arrives at a time
  // consistent with the (now rewound) clock -- LIVE again immediately, same
  // up-transition rule as any ordinary reclaim.
  const ArrivalResult reclaim =
    arbiter.onMessageArrived(kSourceMission, positionCommand(kSourceMission, 50.0), 50.0);
  EXPECT_EQ(arbiter.activeSource(), kSourceMission);
  EXPECT_EQ(reclaim.action, PublishAction::PUBLISH);
}

TEST(ClockRewind, DoesNotReleaseSafetyLatch)
{
  // SAFETY: both processLatchExpiry() early returns (never-alive INHIBIT,
  // and R5's alive-then-silent case) happen BEFORE any ageSecOrInf() call --
  // a rewind must have ZERO effect on the LATCH itself (latch_active_/
  // latch_level_/latchEffective()), same guarantee R5 already gives against
  // ordinary silence. Note: onTick()'s SEPARATE, generic active_source_
  // dwell-release check is a DIFFERENT code path that was never SAFETY-
  // exempt even before N-c (an ordinary silence past release_dwell_sec
  // already dropped active_source_ to NONE for SAFETY same as any channel)
  // -- it may still read a rewind as "SAFETY went quiet" and drop
  // active_source_ for one tick, but the FLOOR (latch_level_ = SAFETY)
  // never moves, so nothing lower can seize control during that gap. That
  // floor invariant -- not literal active_source_ continuity -- is what
  // this test pins.
  ArbiterParams params = defaultParams();
  AuthorityArbiter safety_arbiter(params);
  safety_arbiter.requestLatch(kSourceSafety, 0.0);
  safety_arbiter.onMessageArrived(kSourceSafety, positionCommand(kSourceSafety, 0.5), 0.5);
  ASSERT_TRUE(safety_arbiter.latchEverAlive());

  const TickResult result = safety_arbiter.onTick(-100.0);
  EXPECT_TRUE(safety_arbiter.latchActive())
    << "R5 + N-c: the SAFETY latch itself must survive a clock rewind unconditionally";
  EXPECT_EQ(result.latch_revocation, LatchRevocation::NONE);
  EXPECT_TRUE(safety_arbiter.latchEffective());
  EXPECT_EQ(safety_arbiter.latchLevel(), kSourceSafety);

  // The floor still excludes MISSION even in the tick where active_source_
  // momentarily read NONE above -- this is the invariant that matters
  // operationally (nobody lower ever seizes control), not raw continuity.
  const ArrivalResult mission_attempt = safety_arbiter.onMessageArrived(
    kSourceMission, positionCommand(kSourceMission, -99.9), -99.9);
  EXPECT_NE(safety_arbiter.activeSource(), kSourceMission)
    << "the SAFETY floor must still exclude MISSION even mid-rewind";
  EXPECT_EQ(mission_attempt.action, PublishAction::DROP);

  // And SAFETY itself reclaims active_source_ immediately on its own very
  // next message, at whatever (still-rewound) time base -- the up-transition
  // rule is unconditional, same as ordinary silence recovery.
  const ArrivalResult safety_reclaim = safety_arbiter.onMessageArrived(
    kSourceSafety, positionCommand(kSourceSafety, -99.9), -99.9);
  EXPECT_EQ(safety_arbiter.activeSource(), kSourceSafety);
  EXPECT_EQ(safety_reclaim.action, PublishAction::PUBLISH);

  // Counter-check the test actually bites: the SAME rewind against a MISSION
  // latch (Q-P10-7's accepted tradeoff) DOES revoke it via processLatchExpiry
  // itself -- proves the SAFETY result above is the exception, not a test
  // that can't fail.
  AuthorityArbiter mission_arbiter(params);
  mission_arbiter.requestLatch(kSourceMission, 0.0);
  mission_arbiter.onMessageArrived(kSourceMission, positionCommand(kSourceMission, 0.5), 0.5);
  ASSERT_TRUE(mission_arbiter.latchEverAlive());

  const TickResult mission_result = mission_arbiter.onTick(-100.0);
  EXPECT_FALSE(mission_arbiter.latchActive())
    << "Q-P10-7: OPERATOR/MISSION/TEST latches are accepted to release on clock rewind";
  EXPECT_EQ(mission_result.latch_revocation, LatchRevocation::TIMED_OUT);
}

TEST(ClockRewind, IsCountedNotSwallowed)
{
  // Chosen semantics (documented here since the task explicitly allows a
  // choice): clock_regressions_ increments by exactly 1 per ageSecOrInf()
  // CALL that observes now < stamp -- it is a per-call-site counter, not a
  // per-tick or per-channel dedup. How many of the 4 call sites are actually
  // *reached* with a rewound "now" during one onTick()/onMessageArrived()
  // depends on which channels/latch exist at the time (computeWinner() can
  // short-circuit on isLive()'s guard clauses before ever subtracting). This
  // test pins a scenario with exactly ONE reachable site per rewound call
  // (a single active channel, no latch, no other channel has ever sent) so
  // the count is exactly 1 per rewound tick here -- proving the fix is never
  // silently swallowed (0 is never a valid answer once a rewind happened).
  ArbiterParams params = defaultParams();
  AuthorityArbiter arbiter(params);
  EXPECT_EQ(arbiter.clockRegressionsCount(), 0U);

  arbiter.onMessageArrived(kSourceMission, positionCommand(kSourceMission, 100.0), 100.0);
  EXPECT_EQ(arbiter.clockRegressionsCount(), 0U) << "forward time so far -- nothing to count yet";

  arbiter.onTick(50.0);    // the rewind: only onTick's dwell check is reachable here
  EXPECT_EQ(arbiter.clockRegressionsCount(), 1U);

  // Counter-check (this is the "no rewind -> 0" half of the pin): a forward
  // tick right after must NOT add another count.
  arbiter.onTick(100.5);
  EXPECT_EQ(arbiter.clockRegressionsCount(), 1U) << "no rewind here -- count must stay put";

  // A second, independent call site (rawArrivalAgeSec, diagnostics-only)
  // rewound on its own also counts -- proves every one of the 4 sites is
  // actually wired through the helper, not just the dwell check above.
  const double stale_age = arbiter.rawArrivalAgeSec(kSourceMission, 50.0);
  EXPECT_TRUE(std::isinf(stale_age));
  EXPECT_EQ(arbiter.clockRegressionsCount(), 2U);
}

TEST(ClockRewind, ForwardTimeBehaviorMatchesPreFixArithmetic)
{
  // Task 4 (regression, forward time only): routing every subtraction
  // through ageSecOrInf() must be bit-for-bit invisible when now_sec >=
  // stamp_sec -- exactly now_sec - stamp_sec, no branch taken, no epsilon.
  // Re-runs 3 EXISTING boundary claims (one per non-SAFETY-latch call site:
  // isLive, onTick's dwell check, processLatchExpiry's grace branch)
  // verbatim from TimeBoundary.*/Latch.* above. The full pre-existing suite
  // (68 cases before this file, all of TimeBoundary/Hysteresis/Latch/
  // SafetyLatch/GarbageChannel in particular) reran green after the fix --
  // that full rerun is the primary forward-time regression proof, this test
  // additionally pins clockRegressionsCount() staying at 0 throughout.
  ArbiterParams params = defaultParams();
  AuthorityArbiter arbiter(params);
  arbiter.onMessageArrived(kSourceTest, positionCommand(kSourceTest), 0.0);
  arbiter.onMessageArrived(kSourceMission, positionCommand(kSourceMission), 0.0);
  ASSERT_EQ(arbiter.activeSource(), kSourceMission);
  EXPECT_EQ(arbiter.onTick(params.source_timeout_sec).active_source, kSourceTest)
    << "isLive() boundary unchanged: exactly at source_timeout_sec is still live";

  AuthorityArbiter dwell_arbiter(params);
  dwell_arbiter.onMessageArrived(kSourceMission, positionCommand(kSourceMission), 0.0);
  EXPECT_EQ(dwell_arbiter.onTick(params.release_dwell_sec).active_source, kSourceNone)
    << "dwell boundary unchanged: exactly at release_dwell_sec still releases";

  AuthorityArbiter latch_arbiter(params);
  latch_arbiter.requestLatch(kSourceOperator, 0.0);
  const TickResult grace = latch_arbiter.onTick(params.latch_grace_sec + 0.01);
  EXPECT_EQ(grace.latch_revocation, LatchRevocation::GRACE_EXPIRED)
    << "latch grace boundary unchanged";

  EXPECT_EQ(arbiter.clockRegressionsCount(), 0U);
  EXPECT_EQ(dwell_arbiter.clockRegressionsCount(), 0U);
  EXPECT_EQ(latch_arbiter.clockRegressionsCount(), 0U)
    << "forward time never counts as a regression";
}

// ===========================================================================
// C4 (P10.8b review finding): the two "now - stamp" subtractions N-c's first
// pass missed -- validateContent()'s content-age (STALE) guard and
// latchAgeSec(). Both used raw arithmetic that went NEGATIVE on a rewind,
// which is the single gate on this codebase's ENTIRE line that checks
// command CONTENT age (liveness above is judged by ARRIVAL time, never
// header.stamp) -- a negative age is never > max_command_age_sec, so a
// message whose age cannot be measured sailed through as the freshest thing
// the arbiter had ever seen, leaving no trace (clock_regressions_ untouched).
// ===========================================================================

TEST(ClockRewind, ContentAgeGuardDropsStaleInsteadOfSilentlyAcceptingAnUnmeasurableAge)
{
  ArbiterParams params = defaultParams();
  AuthorityArbiter arbiter(params);

  // A command stamped AFTER the arbiter's current "now" -- the signature of
  // either a bag-replay/loop rewind or a publisher that forgot use_sim_time
  // and stamped with a wall-clock value far ahead of sim time.
  const ArrivalResult result = arbiter.onMessageArrived(
    kSourceMission, positionCommand(kSourceMission, /*stamp_sec=*/100.0), /*now=*/50.0);

  EXPECT_EQ(result.drop_reason, DropReason::STALE)
    << "C4: an unmeasurable content age must DROP, never silently pass as fresh";
  EXPECT_EQ(result.action, PublishAction::DROP);
  EXPECT_EQ(arbiter.activeSource(), kSourceNone)
    << "the channel must not win authority off a command the STALE guard should have caught";
  EXPECT_EQ(arbiter.droppedStaleCount(), 1U);
  EXPECT_EQ(arbiter.clockRegressionsCount(), 1U)
    << "the rewind that used to defeat this guard silently must now be counted";
}

TEST(ClockRewind, ContentAgeGuardStillAcceptsAFreshCommandUnderOrdinaryForwardTime)
{
  // Positive counter-check (R27-3): proves the fix can tell a fresh command
  // apart from a stale/unmeasurable one -- it is not just "always DROP now".
  ArbiterParams params = defaultParams();
  AuthorityArbiter arbiter(params);

  const ArrivalResult result = arbiter.onMessageArrived(
    kSourceMission, positionCommand(kSourceMission, /*stamp_sec=*/10.0), /*now=*/10.05);

  EXPECT_EQ(result.drop_reason, DropReason::NONE);
  EXPECT_EQ(result.action, PublishAction::PUBLISH);
  EXPECT_EQ(arbiter.activeSource(), kSourceMission);
  EXPECT_EQ(arbiter.droppedStaleCount(), 0U);
  EXPECT_EQ(arbiter.clockRegressionsCount(), 0U)
    << "ordinary forward time must never be counted as a regression";
}

TEST(ClockRewind, LatchAgeSecIsNanWhenClockRewindsPastGrantTime)
{
  // Before this fix, latchAgeSec() computed a raw "now - granted_at" that
  // went NEGATIVE on a rewind -- the opposite of what the "latch_age_sec
  // growing unbounded means a dead SAFETY node" monitor (README §2 mục 6 /
  // control_authority_params.yaml comment) needs: a negative number reads
  // as "very fresh", not "cannot be measured".
  ArbiterParams params = defaultParams();
  AuthorityArbiter arbiter(params);
  arbiter.requestLatch(kSourceOperator, /*now=*/100.0);
  ASSERT_TRUE(arbiter.latchActive());

  const double age = arbiter.latchAgeSec(/*now=*/50.0);
  EXPECT_TRUE(std::isnan(age))
    << "C4/R30: an unmeasurable latch age must read as NaN, never a negative number";
}

TEST(ClockRewind, LatchAgeSecIsExactAndPositiveUnderOrdinaryForwardTime)
{
  // Positive counter-check for the same reason as above: NaN only on a
  // genuine rewind, never as a side effect of routing through ageSecOrInf().
  ArbiterParams params = defaultParams();
  AuthorityArbiter arbiter(params);
  arbiter.requestLatch(kSourceOperator, /*now=*/10.0);
  ASSERT_TRUE(arbiter.latchActive());

  EXPECT_DOUBLE_EQ(arbiter.latchAgeSec(/*now=*/12.5), 2.5);
}

TEST(ClockRewind, LatchAgeSecIsZeroWhenNoLatchIsHeldRegardlessOfNow)
{
  // Regression pin: the "no latch" branch must stay untouched by the fix --
  // it never calls ageSecOrInf() at all, so it can never itself register a
  // clock regression.
  AuthorityArbiter arbiter(defaultParams());
  EXPECT_DOUBLE_EQ(arbiter.latchAgeSec(/*now=*/-100.0), 0.0);
  EXPECT_EQ(arbiter.clockRegressionsCount(), 0U);
}

}  // namespace uav_control_authority
