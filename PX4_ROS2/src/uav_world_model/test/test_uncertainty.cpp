#include <array>
#include <cmath>

#include <gtest/gtest.h>

#include "uav_world_model/uncertainty.hpp"

using uav_world_model::DriftModel;
using uav_world_model::UncertaintyFloor;

namespace
{

// P5.3 hovered at 2.5 m and 3.5 m; the lens saw 2.665 m and 3.665 m.
UncertaintyFloor markerFloor()
{
  return UncertaintyFloor{0.0185, 0.007};
}

std::array<double, 36> covarianceWithDiagonal(double xx, double yy, double zz)
{
  std::array<double, 36> covariance{};
  covariance[0] = xx;
  covariance[7] = yy;
  covariance[14] = zz;
  return covariance;
}

}  // namespace

TEST(PerceptionFloor, CoversTheMeasuredErrorAtTheMeasuredRanges)
{
  // Slant range, not hover height: marker_accuracy.py adds 0.240 - 0.065 - 0.01.
  const double near_floor = uav_world_model::floorValue(markerFloor(), 2.665);
  const double far_floor = uav_world_model::floorValue(markerFloor(), 3.665);

  EXPECT_GE(near_floor, 0.036) << "claiming better than the measured error is a lie";
  EXPECT_GE(far_floor, 0.043);
  EXPECT_LT(near_floor, 0.036 + 0.002) << "conservative, not arbitrary";
  EXPECT_LT(far_floor, 0.043 + 0.002);
}

TEST(PerceptionFloor, SentinelMinusOneBecomesTheFloor)
{
  EXPECT_NEAR(
    uav_world_model::perceptionUncertainty(-1.0, 2.5, markerFloor()),
    uav_world_model::floorValue(markerFloor(), 2.5), 1e-12);
}

TEST(PerceptionFloor, ZeroBecomesTheFloorToo)
{
  // marker_detector_node leaves the field at its default zero today.
  EXPECT_NEAR(uav_world_model::perceptionUncertainty(0.0, 2.5, markerFloor()), 0.036, 1e-12);
}

TEST(PerceptionFloor, AReportedValueBelowTheMeasuredErrorIsRaised)
{
  EXPECT_NEAR(uav_world_model::perceptionUncertainty(0.001, 2.5, markerFloor()), 0.036, 1e-12);
}

TEST(PerceptionFloor, AHonestLargerValueIsKept)
{
  EXPECT_NEAR(uav_world_model::perceptionUncertainty(0.5, 2.5, markerFloor()), 0.5, 1e-12);
}

TEST(Combination, PerceptionAndLocalizationAddInQuadrature)
{
  const double perception = uav_world_model::perceptionUncertainty(-1.0, 2.5, markerFloor());
  const double localization = 0.4;
  const double combined = uav_world_model::combineQuadrature({perception, localization});

  EXPECT_NEAR(combined, std::sqrt(0.036 * 0.036 + 0.16), 1e-12);
  EXPECT_NEAR(combined, 0.4016167, 1e-6);
  EXPECT_GT(combined, localization) << "adding a source of error may not shrink the total";
}

TEST(Combination, NeverReturnsZeroOrTheUnknownSentinel)
{
  const double combined = uav_world_model::combineQuadrature(
    {uav_world_model::perceptionUncertainty(-1.0, 0.0, markerFloor()), -1.0});
  EXPECT_GT(combined, 0.0);
}

TEST(Combination, IgnoresUnusableTerms)
{
  EXPECT_NEAR(
    uav_world_model::combineQuadrature({0.3, -1.0, 0.4}), 0.5, 1e-12);
}

TEST(FusedCovariance, WorstAxisWins)
{
  const auto covariance = covarianceWithDiagonal(0.04, 0.09, 0.01);
  EXPECT_NEAR(uav_world_model::worstAxisStddev(covariance), 0.3, 1e-12);
}

TEST(FusedCovariance, AllZerosMeansNoErrorWasStated)
{
  EXPECT_LT(uav_world_model::worstAxisStddev(covarianceWithDiagonal(0.0, 0.0, 0.0)), 0.0);
}

TEST(FusedCovariance, TheRosUnknownSentinelIsRejected)
{
  EXPECT_LT(uav_world_model::worstAxisStddev(covarianceWithDiagonal(-1.0, 0.04, 0.04)), 0.0);
}

TEST(DriftGrowth, ZeroAgeAddsNothing)
{
  EXPECT_NEAR(uav_world_model::driftGrowth(0.0, DriftModel{0.5, 120.0}), 0.0, 1e-12);
}

TEST(DriftGrowth, OneCorrelationTimeMatchesTheGaussMarkovModel)
{
  EXPECT_NEAR(uav_world_model::driftGrowth(120.0, DriftModel{0.5, 120.0}), 0.5621924, 1e-6);
}

TEST(DriftGrowth, SaturatesAtSigmaRootTwo)
{
  EXPECT_NEAR(
    uav_world_model::driftGrowth(1.0e6, DriftModel{0.5, 120.0}), 0.5 * std::sqrt(2.0), 1e-9);
}

TEST(DriftGrowth, GrowsWithAge)
{
  const DriftModel model{0.5, 120.0};
  EXPECT_GT(uav_world_model::driftGrowth(30.0, model), uav_world_model::driftGrowth(5.0, model));
}
