// ROS-free (no domain, no rclcpp::init): the freshness arithmetic is pinned
// by a test instead of trusting a real publisher, same rule uav_safety's
// failsafe_policy tests follow.
#include <cmath>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "uav_observability/staleness_board.hpp"

namespace uav_observability
{
namespace
{

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

const ItemVerdict & verdictFor(const std::vector<ItemVerdict> & verdicts, const std::string & name)
{
  for (const auto & v : verdicts) {
    if (v.name == name) {
      return v;
    }
  }
  ADD_FAILURE() << "no verdict for " << name;
  static const ItemVerdict kMissing{};
  return kMissing;
}

}  // namespace

// ===========================================================================
// addItem validation.
// ===========================================================================

TEST(AddItem, ValidItemIsAccepted)
{
  StalenessBoard board;
  EXPECT_NO_THROW(board.addItem("odometry_fused", 10.0, 0.5, Level::kError));
}

TEST(AddItem, DuplicateNameIsRejected)
{
  StalenessBoard board;
  board.addItem("odometry_fused", 10.0, 0.5, Level::kError);
  EXPECT_THROW(board.addItem("odometry_fused", 10.0, 0.5, Level::kError), std::invalid_argument);
}

TEST(AddItem, ZeroExpectedHzIsRejected)
{
  StalenessBoard board;
  EXPECT_THROW(board.addItem("x", 0.0, 0.5, Level::kError), std::invalid_argument);
}

TEST(AddItem, NegativeExpectedHzIsRejected)
{
  StalenessBoard board;
  EXPECT_THROW(board.addItem("x", -5.0, 0.5, Level::kError), std::invalid_argument);
}

// V-O1: timeout must be >= 2/expected_hz.
TEST(AddItem, TimeoutBelowTwoOverHzIsRejected)
{
  StalenessBoard board;
  EXPECT_THROW(board.addItem("x", 10.0, 0.199, Level::kError), std::invalid_argument);
}

TEST(AddItem, TimeoutExactlyTwoOverHzIsAccepted)
{
  StalenessBoard board;
  EXPECT_NO_THROW(board.addItem("x", 10.0, 0.2, Level::kError));   // == 2/10 exactly
}

TEST(AddItem, LevelWhenMissingMustBeWarnOrError)
{
  StalenessBoard board;
  EXPECT_THROW(board.addItem("x", 10.0, 0.5, Level::kOk), std::invalid_argument);
  EXPECT_THROW(board.addItem("y", 10.0, 0.5, Level::kUnknown), std::invalid_argument);
}

TEST(OnArrival, UnregisteredNameIsRejected)
{
  StalenessBoard board;
  board.addItem("a", 10.0, 0.5, Level::kError);
  EXPECT_THROW(board.onArrival("b", 0.0), std::invalid_argument);
}

// ===========================================================================
// Invariant 1: never received -> UNKNOWN, age_sec NaN, measured_hz NaN.
// ===========================================================================

TEST(StalenessBoard, NeverReceivedIsUnknownWithNanAgeAndHz)
{
  StalenessBoard board;
  board.addItem("odometry_fused", 10.0, 0.5, Level::kError);
  const auto verdicts = board.evaluate(5.0);
  const auto & v = verdictFor(verdicts, "odometry_fused");
  EXPECT_EQ(v.level, Level::kUnknown);
  EXPECT_TRUE(std::isnan(v.age_sec));
  EXPECT_TRUE(std::isnan(v.measured_hz));
}

// Mutation counter: one arrival flips it out of UNKNOWN -- proves the
// branch is actually gated on `received`, not hardcoded UNKNOWN.
TEST(StalenessBoard, OneArrivalLeavesUnknownBehind)
{
  StalenessBoard board;
  board.addItem("odometry_fused", 10.0, 0.5, Level::kError);
  board.onArrival("odometry_fused", 1.0);
  const auto verdicts = board.evaluate(1.05);
  const auto & v = verdictFor(verdicts, "odometry_fused");
  EXPECT_NE(v.level, Level::kUnknown);
  EXPECT_EQ(v.level, Level::kOk);
}

// ===========================================================================
// Invariant 2: age > timeout -> level_when_missing; age <= timeout -> OK.
// ===========================================================================

TEST(StalenessBoard, AgeAtOrBelowTimeoutIsOk)
{
  StalenessBoard board;
  board.addItem("odometry_fused", 10.0, 0.5, Level::kError);
  board.onArrival("odometry_fused", 0.0);
  const auto verdicts = board.evaluate(0.5);   // exactly at timeout
  EXPECT_EQ(verdictFor(verdicts, "odometry_fused").level, Level::kOk);
}

TEST(StalenessBoard, AgePastTimeoutReportsConfiguredErrorSeverity)
{
  StalenessBoard board;
  board.addItem("odometry_fused", 10.0, 0.5, Level::kError);
  board.onArrival("odometry_fused", 0.0);
  const auto verdicts = board.evaluate(0.501);
  EXPECT_EQ(verdictFor(verdicts, "odometry_fused").level, Level::kError);
}

// Mutation counter: same scenario but level_when_missing=WARN -- the
// reported level must track the CONFIGURED severity, not a hardcoded ERROR.
TEST(StalenessBoard, AgePastTimeoutReportsConfiguredWarnSeverity)
{
  StalenessBoard board;
  board.addItem("perception_markers", 2.0, 1.0, Level::kWarn);
  board.onArrival("perception_markers", 0.0);
  const auto verdicts = board.evaluate(1.001);
  EXPECT_EQ(verdictFor(verdicts, "perception_markers").level, Level::kWarn);
}

// ===========================================================================
// Invariant 3: clock regression -> UNKNOWN + clockRegressions() += 1 per
// detection, never a negative age, never the held-over previous verdict.
// ===========================================================================

TEST(StalenessBoard, OnArrivalBackwardInTimeIsRejectedAndCounted)
{
  StalenessBoard board;
  board.addItem("odometry_fused", 10.0, 0.5, Level::kError);
  board.onArrival("odometry_fused", 5.0);
  board.onArrival("odometry_fused", 4.0);   // clock went backward
  EXPECT_EQ(board.clockRegressions(), 1u);
  // last_arrival_sec must be untouched (still 5.0) -- age at t=5.05 is 0.05.
  const auto verdicts = board.evaluate(5.05);
  const auto & v = verdictFor(verdicts, "odometry_fused");
  EXPECT_EQ(v.level, Level::kOk);
  EXPECT_NEAR(v.age_sec, 0.05, 1e-9);
}

// Mutation counter: a normal, forward-in-time sequence must never increment
// the counter -- proves it is not incremented unconditionally on every call.
TEST(StalenessBoard, ForwardArrivalsNeverIncrementClockRegressions)
{
  StalenessBoard board;
  board.addItem("odometry_fused", 10.0, 0.5, Level::kError);
  board.onArrival("odometry_fused", 1.0);
  board.onArrival("odometry_fused", 2.0);
  board.onArrival("odometry_fused", 3.0);
  board.evaluate(3.1);
  board.evaluate(4.0);
  EXPECT_EQ(board.clockRegressions(), 0u);
}

TEST(StalenessBoard, EvaluateOnARegressedClockIsUnknownWithNanAgeNotNegative)
{
  StalenessBoard board;
  board.addItem("odometry_fused", 10.0, 0.5, Level::kError);
  board.onArrival("odometry_fused", 10.0);
  const auto verdicts = board.evaluate(9.0);   // evaluate's own clock is behind the arrival
  const auto & v = verdictFor(verdicts, "odometry_fused");
  EXPECT_EQ(v.level, Level::kUnknown);
  EXPECT_TRUE(std::isnan(v.age_sec)) << "must never report a negative age";
  EXPECT_TRUE(std::isnan(v.measured_hz));
  EXPECT_EQ(board.clockRegressions(), 1u);
}

TEST(StalenessBoard, EvaluateRegressionDoesNotHoldOverThePreviousGoodVerdict)
{
  StalenessBoard board;
  board.addItem("odometry_fused", 10.0, 0.5, Level::kError);
  board.onArrival("odometry_fused", 10.0);
  const auto good = board.evaluate(10.05);
  ASSERT_EQ(verdictFor(good, "odometry_fused").level, Level::kOk);
  const auto regressed = board.evaluate(9.0);
  EXPECT_EQ(verdictFor(regressed, "odometry_fused").level, Level::kUnknown)
    << "must not silently keep reporting the last good OK verdict";
}

TEST(StalenessBoard, RepeatedEvaluateCallsOnTheSameRegressedClockEachCount)
{
  // "1 lan/lan lui": each DETECTION (each call where now < last_arrival) is
  // its own occurrence -- not a continuity window collapsed to one count.
  StalenessBoard board;
  board.addItem("odometry_fused", 10.0, 0.5, Level::kError);
  board.onArrival("odometry_fused", 10.0);
  board.evaluate(9.0);
  board.evaluate(9.0);
  board.evaluate(9.0);
  EXPECT_EQ(board.clockRegressions(), 3u);
}

// A NaN "now" is a distinct failure mode from a genuine backward-clock
// regression (R27-2) -- must yield UNKNOWN but must NOT itself advance
// clockRegressions().
TEST(StalenessBoard, NanNowInEvaluateIsUnknownButNotCountedAsRegression)
{
  StalenessBoard board;
  board.addItem("odometry_fused", 10.0, 0.5, Level::kError);
  board.onArrival("odometry_fused", 1.0);
  const auto verdicts = board.evaluate(kNan);
  const auto & v = verdictFor(verdicts, "odometry_fused");
  EXPECT_EQ(v.level, Level::kUnknown);
  EXPECT_TRUE(std::isnan(v.age_sec));
  EXPECT_TRUE(std::isnan(v.measured_hz));
  EXPECT_EQ(board.clockRegressions(), 0u);
}

TEST(StalenessBoard, NanNowInOnArrivalIsIgnoredWithNoStateChange)
{
  StalenessBoard board;
  board.addItem("odometry_fused", 10.0, 0.5, Level::kError);
  board.onArrival("odometry_fused", 1.0);
  board.onArrival("odometry_fused", kNan);   // must be silently rejected
  EXPECT_EQ(board.clockRegressions(), 0u);
  const auto verdicts = board.evaluate(1.05);
  const auto & v = verdictFor(verdicts, "odometry_fused");
  EXPECT_EQ(v.level, Level::kOk);
  EXPECT_NEAR(v.age_sec, 0.05, 1e-9) << "last_arrival_sec must still be 1.0, unpoisoned by the NaN";
}

// ===========================================================================
// Invariant 4: measured_hz from the SMALLEST gap ever seen, not the latest;
// fewer than 2 samples -> NaN.
// ===========================================================================

TEST(StalenessBoard, FewerThanTwoSamplesYieldsNanHz)
{
  StalenessBoard board;
  board.addItem("odometry_fused", 10.0, 0.5, Level::kError);
  board.onArrival("odometry_fused", 1.0);
  const auto verdicts = board.evaluate(1.01);
  EXPECT_TRUE(std::isnan(verdictFor(verdicts, "odometry_fused").measured_hz));
}

TEST(StalenessBoard, MeasuredHzIsInverseOfTheSmallestGap)
{
  StalenessBoard board;
  board.addItem("odometry_fused", 10.0, 0.5, Level::kError);
  board.onArrival("odometry_fused", 0.0);
  board.onArrival("odometry_fused", 0.10);   // gap 0.10 -> 10 Hz so far
  board.onArrival("odometry_fused", 0.15);   // gap 0.05 -> new min, 20 Hz
  const auto verdicts = board.evaluate(0.16);
  EXPECT_NEAR(verdictFor(verdicts, "odometry_fused").measured_hz, 20.0, 1e-6);
}

// Mutation counter: a LATER, wider gap (a dropped sample) must NOT drag
// measured_hz down -- it stays anchored to the tightest gap ever observed,
// same "infer rate from the minimum, not the latest" rule as
// camera_health_node.
TEST(StalenessBoard, ALaterWidenedGapDoesNotChangeMeasuredHz)
{
  StalenessBoard board;
  board.addItem("odometry_fused", 10.0, 0.5, Level::kError);
  board.onArrival("odometry_fused", 0.0);
  board.onArrival("odometry_fused", 0.05);   // tight gap -> 20 Hz
  board.onArrival("odometry_fused", 0.30);   // wide gap (a drop) -> would be ~4 Hz alone
  const auto verdicts = board.evaluate(0.31);
  EXPECT_NEAR(verdictFor(verdicts, "odometry_fused").measured_hz, 20.0, 1e-6)
    << "must stay anchored to the earlier, tighter minimum gap";
}

// R27-2: a NaN arrival inserted MID-sequence must not corrupt the running
// minimum via an unguarded std::min(a, NaN).
TEST(StalenessBoard, NanArrivalMidSequenceDoesNotCorruptMeasuredHz)
{
  StalenessBoard board;
  board.addItem("odometry_fused", 10.0, 0.5, Level::kError);
  board.onArrival("odometry_fused", 0.0);
  board.onArrival("odometry_fused", 0.10);   // gap 0.10 -> 10 Hz
  board.onArrival("odometry_fused", kNan);   // must be rejected, no effect
  board.onArrival("odometry_fused", 0.15);   // gap from 0.10 -> 0.05 -> new min, 20 Hz
  const auto verdicts = board.evaluate(0.16);
  const auto & v = verdictFor(verdicts, "odometry_fused");
  EXPECT_NEAR(v.measured_hz, 20.0, 1e-6);
  EXPECT_FALSE(std::isnan(v.measured_hz));
}

// ===========================================================================
// VÀNG#1: two arrivals sharing the exact same stamp (gap==0.0) must never
// produce measured_hz=+inf ("infinitely healthy" -- exactly the R30 family
// of bug this board exists to prevent).
// ===========================================================================

TEST(StalenessBoard, TwoArrivalsWithIdenticalStampNeverProduceInfiniteHz)
{
  StalenessBoard board;
  board.addItem("odometry_fused", 10.0, 0.5, Level::kError);
  board.onArrival("odometry_fused", 1.0);
  board.onArrival("odometry_fused", 1.0);   // same stamp -- gap 0.0, not a real second sample
  const auto verdicts = board.evaluate(1.01);
  const auto & v = verdictFor(verdicts, "odometry_fused");
  EXPECT_TRUE(std::isnan(v.measured_hz)) << "gap==0.0 must read as 'not yet two real samples', "
    "never as measured_hz=+inf";
}

// Mutation counter: a REAL gap arriving after the zero-gap duplicate must
// still be picked up correctly -- proves the zero-gap guard defers rather
// than permanently poisoning has_two_samples.
TEST(StalenessBoard, ARealGapAfterAZeroGapDuplicateIsStillMeasuredCorrectly)
{
  StalenessBoard board;
  board.addItem("odometry_fused", 10.0, 0.5, Level::kError);
  board.onArrival("odometry_fused", 1.0);
  board.onArrival("odometry_fused", 1.0);    // duplicate stamp, ignored for hz
  board.onArrival("odometry_fused", 1.1);    // real gap 0.1 -> 10 Hz
  const auto verdicts = board.evaluate(1.11);
  const auto & v = verdictFor(verdicts, "odometry_fused");
  ASSERT_FALSE(std::isnan(v.measured_hz));
  EXPECT_NEAR(v.measured_hz, 10.0, 1e-6);
}

// ===========================================================================
// Multi-item independence: one item's staleness/regression must never
// affect a sibling's verdict.
// ===========================================================================

TEST(StalenessBoard, ItemsAreIndependentOfEachOther)
{
  StalenessBoard board;
  board.addItem("fresh_item", 10.0, 0.5, Level::kError);
  board.addItem("stale_item", 10.0, 0.5, Level::kWarn);
  board.onArrival("fresh_item", 4.9);
  board.onArrival("stale_item", 0.0);
  const auto verdicts = board.evaluate(5.0);
  EXPECT_EQ(verdictFor(verdicts, "fresh_item").level, Level::kOk);
  EXPECT_EQ(verdictFor(verdicts, "stale_item").level, Level::kWarn);
}

}  // namespace uav_observability
