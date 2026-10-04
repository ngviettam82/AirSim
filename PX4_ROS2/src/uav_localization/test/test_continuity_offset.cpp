#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "uav_localization/continuity_offset.hpp"

namespace
{

using uav_localization::ContinuityOffset;
using uav_localization::anchorOutlivesBlackout;

constexpr double kGrace = 0.5;

constexpr double kRate = 0.5;
constexpr double kTick = 0.1;

ContinuityOffset anchoredAt(double x, double y, double z)
{
  ContinuityOffset offset;
  offset.anchor({x, y, z}, {0.0, 0.0, 0.0});
  return offset;
}

TEST(ContinuityOffset, StartsInert)
{
  const ContinuityOffset offset;
  EXPECT_FALSE(offset.active());
  EXPECT_DOUBLE_EQ(offset.magnitude(), 0.0);
}

TEST(ContinuityOffset, AnchoringHoldsThePublishedPoseAcrossTheSwitch)
{
  ContinuityOffset offset;
  offset.anchor({10.0, 5.0, 2.0}, {9.0, 4.0, 1.5});

  EXPECT_TRUE(offset.active());
  EXPECT_DOUBLE_EQ(offset.value()[0], 1.0);
  EXPECT_DOUBLE_EQ(offset.value()[1], 1.0);
  EXPECT_DOUBLE_EQ(offset.value()[2], 0.5);
}

TEST(ContinuityOffset, EachAxisFadesAtTheDeclaredRate)
{
  ContinuityOffset offset = anchoredAt(1.0, 0.0, 0.0);

  offset.advance(kTick, kRate);

  EXPECT_NEAR(offset.value()[0], 0.95, 1e-12);
}

TEST(ContinuityOffset, TheFadeStopsAtZeroWithoutOvershootingIntoTheOtherSign)
{
  ContinuityOffset offset = anchoredAt(0.02, -0.02, 0.0);

  offset.advance(kTick, kRate);

  EXPECT_DOUBLE_EQ(offset.value()[0], 0.0);
  EXPECT_DOUBLE_EQ(offset.value()[1], 0.0);
  EXPECT_FALSE(offset.active());
}

TEST(ContinuityOffset, ANegativeOffsetFadesTowardZeroToo)
{
  ContinuityOffset offset = anchoredAt(-1.0, 0.0, 0.0);

  offset.advance(kTick, kRate);

  EXPECT_NEAR(offset.value()[0], -0.95, 1e-12);
}

// The B3 defect: the tick that lands on zero still moved the pose, and the
// mux reads this answer to decide whether it may claim a certain velocity.
TEST(ContinuityOffset, TheTickThatReachesZeroStillReportsThatThePoseMoved)
{
  ContinuityOffset offset = anchoredAt(0.02, 0.0, 0.0);

  const bool moved = offset.advance(kTick, kRate);

  EXPECT_FALSE(offset.active()) << "precondition: this tick lands exactly on zero";
  EXPECT_TRUE(moved) << "the pose slid 0.02 m during this very tick";
}

TEST(ContinuityOffset, AnInertOffsetReportsNoMovement)
{
  ContinuityOffset offset;

  EXPECT_FALSE(offset.advance(kTick, kRate));
}

TEST(ContinuityOffset, EveryTickOfAFadeReportsMovementIncludingTheLast)
{
  ContinuityOffset offset = anchoredAt(0.2, 0.0, 0.0);

  int moving_ticks = 0;
  while (offset.advance(kTick, kRate)) {
    ++moving_ticks;
    ASSERT_LT(moving_ticks, 100) << "the fade must terminate";
  }

  EXPECT_FALSE(offset.active());
  // 0.20 m at 0.05 m per tick; rounding may leave residue for one tick more.
  EXPECT_GE(moving_ticks, 4);
  EXPECT_FALSE(offset.advance(kTick, kRate)) << "an inert offset stays quiet";
}

TEST(ContinuityOffset, AStalledClockCannotFadeTheOffsetButStillReportsItInPlay)
{
  ContinuityOffset offset = anchoredAt(1.0, 0.0, 0.0);

  const bool moved = offset.advance(0.0, kRate);

  EXPECT_TRUE(moved);
  EXPECT_DOUBLE_EQ(offset.value()[0], 1.0);
}

TEST(ContinuityOffset, ANonPositiveRateLeavesTheOffsetStandingRatherThanErasingIt)
{
  ContinuityOffset offset = anchoredAt(1.0, 0.0, 0.0);

  EXPECT_TRUE(offset.advance(kTick, 0.0));
  EXPECT_DOUBLE_EQ(offset.value()[0], 1.0);
  EXPECT_TRUE(offset.advance(kTick, -1.0));
  EXPECT_DOUBLE_EQ(offset.value()[0], 1.0);
}

TEST(ContinuityOffset, ClearingDropsTheAnchorOutright)
{
  ContinuityOffset offset = anchoredAt(1.0, 1.0, 1.0);

  offset.clear();

  EXPECT_FALSE(offset.active());
  EXPECT_FALSE(offset.advance(kTick, kRate));
}

TEST(ContinuityOffset, MagnitudeIsTheThreeAxisLength)
{
  const ContinuityOffset offset = anchoredAt(3.0, 4.0, 12.0);

  EXPECT_NEAR(offset.magnitude(), 13.0, 1e-12);
}

TEST(ContinuityOffset, ReAnchoringReplacesTheOldOffsetInsteadOfStackingOnIt)
{
  ContinuityOffset offset = anchoredAt(1.0, 0.0, 0.0);

  offset.anchor({2.0, 0.0, 0.0}, {0.0, 0.0, 0.0});

  EXPECT_DOUBLE_EQ(offset.value()[0], 2.0);
}

// The grace boundary itself is arithmetic, so it is pinned here jitter-free;
// test_localization_mux_node.cpp brackets it on the running node.
TEST(AnchorGrace, ABlinkShorterThanTheGraceKeepsTheAnchor)
{
  EXPECT_TRUE(anchorOutlivesBlackout(0.0, kGrace));
  EXPECT_TRUE(anchorOutlivesBlackout(0.1, kGrace));
}

TEST(AnchorGrace, TheGraceItselfStillCounts)
{
  EXPECT_TRUE(anchorOutlivesBlackout(kGrace, kGrace));
}

TEST(AnchorGrace, ABlackoutPastTheGraceDropsTheAnchor)
{
  EXPECT_FALSE(anchorOutlivesBlackout(std::nextafter(kGrace, 1.0), kGrace));
  EXPECT_FALSE(anchorOutlivesBlackout(2.0, kGrace));
  EXPECT_FALSE(anchorOutlivesBlackout(std::numeric_limits<double>::infinity(), kGrace));
}

TEST(AnchorGrace, AGapThatCannotBeMeasuredIsTreatedAsALoss)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(anchorOutlivesBlackout(nan, kGrace)) << "a NaN gap must not keep the anchor";
  EXPECT_FALSE(anchorOutlivesBlackout(0.1, nan));
  EXPECT_FALSE(anchorOutlivesBlackout(-0.1, kGrace)) << "a clock that ran backwards proves nothing";
}

TEST(AnchorGrace, AZeroGraceDisablesTheHold)
{
  EXPECT_TRUE(anchorOutlivesBlackout(0.0, 0.0)) << "the tick of the loss itself is not a blackout";
  EXPECT_FALSE(anchorOutlivesBlackout(0.02, 0.0));
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
