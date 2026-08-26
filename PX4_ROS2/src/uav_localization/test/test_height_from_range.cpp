#include <cmath>
#include <limits>

#include <geometry_msgs/msg/quaternion.hpp>
#include <gtest/gtest.h>

#include "uav_localization/height_from_range.hpp"

using geometry_msgs::msg::Quaternion;
using uav_localization::heightFromRange;
using uav_localization::tiltCosine;

namespace
{

/// Pitch: rotation about body Y, FLU convention.
Quaternion pitched(double pitch_rad)
{
  Quaternion q;
  q.w = std::cos(pitch_rad / 2.0);
  q.x = 0.0;
  q.y = std::sin(pitch_rad / 2.0);
  q.z = 0.0;
  return q;
}

Quaternion level()
{
  return pitched(0.0);
}

constexpr double kMinRange = 0.05;
constexpr double kMaxRange = 12.0;
constexpr double kMaxTilt = 0.6981;   // 40 degrees

TEST(TiltCosine, LevelAircraftGivesOne)
{
  EXPECT_NEAR(tiltCosine(level()), 1.0, 1e-9);
}

TEST(TiltCosine, MatchesTheCosineOfThePitchAngle)
{
  for (const double pitch : {0.1, 0.3, 0.5, 1.0}) {
    EXPECT_NEAR(tiltCosine(pitched(pitch)), std::cos(pitch), 1e-9)
      << "pitch " << pitch;
  }
}

TEST(TiltCosine, DegenerateQuaternionDoesNotPretendToBeLevel)
{
  Quaternion degenerate;
  degenerate.w = 0.0;   // a default-constructed Quaternion is the identity, w = 1
  degenerate.x = 0.0;
  degenerate.y = 0.0;
  degenerate.z = 0.0;
  EXPECT_DOUBLE_EQ(tiltCosine(degenerate), 0.0);
}

TEST(TiltCosine, DefaultConstructedQuaternionIsTheIdentity)
{
  const Quaternion unset;
  EXPECT_DOUBLE_EQ(unset.w, 1.0);
  EXPECT_NEAR(tiltCosine(unset), 1.0, 1e-9);
}

TEST(HeightFromRange, LevelFlightPassesTheDistanceThrough)
{
  const auto estimate = heightFromRange(3.0, level(), kMinRange, kMaxRange, kMaxTilt);
  ASSERT_TRUE(estimate.valid);
  EXPECT_NEAR(estimate.height_m, 3.0, 1e-9);
  EXPECT_NEAR(estimate.tilt_rad, 0.0, 1e-6);
}

TEST(HeightFromRange, TiltShortensTheBeamIntoTheRealHeight)
{
  const double pitch = 20.0 * M_PI / 180.0;
  const double true_height = 2.0;
  const double slant = true_height / std::cos(pitch);

  const auto estimate = heightFromRange(slant, pitched(pitch), kMinRange, kMaxRange, kMaxTilt);
  ASSERT_TRUE(estimate.valid);
  EXPECT_NEAR(estimate.height_m, true_height, 1e-6);
  EXPECT_NEAR(estimate.tilt_rad, pitch, 1e-6);
}

TEST(HeightFromRange, HoldingHeightWhileTiltedDoesNotDriftMoreThanThreePercent)
{
  const double true_height = 2.0;
  double worst_error = 0.0;

  for (int degrees = 0; degrees <= 20; ++degrees) {
    const double pitch = degrees * M_PI / 180.0;
    const double slant = true_height / std::cos(pitch);
    const auto estimate = heightFromRange(slant, pitched(pitch), kMinRange, kMaxRange, kMaxTilt);
    ASSERT_TRUE(estimate.valid) << "pitch " << degrees;
    worst_error = std::max(worst_error, std::abs(estimate.height_m - true_height) / true_height);
  }

  EXPECT_LT(worst_error, 0.03) << "tilt correction is not holding height steady";
}

TEST(HeightFromRange, WithoutCorrectionTwentyDegreesWouldOverreadBySixPercent)
{
  const double pitch = 20.0 * M_PI / 180.0;
  const double uncorrected_error = 1.0 / std::cos(pitch) - 1.0;
  EXPECT_GT(uncorrected_error, 0.06);
}

TEST(HeightFromRange, BeyondTheTiltLimitTheMeasurementIsRefused)
{
  const double pitch = 50.0 * M_PI / 180.0;
  const auto estimate = heightFromRange(3.0, pitched(pitch), kMinRange, kMaxRange, kMaxTilt);
  EXPECT_FALSE(estimate.valid);
  EXPECT_STREQ(estimate.reason, "tilted too far to describe the ground below");
}

TEST(HeightFromRange, OutOfIntervalReadingsAreRefused)
{
  EXPECT_FALSE(heightFromRange(0.01, level(), kMinRange, kMaxRange, kMaxTilt).valid);
  EXPECT_FALSE(heightFromRange(50.0, level(), kMinRange, kMaxRange, kMaxTilt).valid);
  EXPECT_FALSE(
    heightFromRange(
      std::numeric_limits<double>::infinity(), level(), kMinRange, kMaxRange, kMaxTilt).valid);
  EXPECT_FALSE(
    heightFromRange(
      std::numeric_limits<double>::quiet_NaN(), level(), kMinRange, kMaxRange, kMaxTilt).valid);
}

TEST(HeightFromRange, UpsideDownIsRefusedRatherThanNegated)
{
  const auto estimate = heightFromRange(3.0, pitched(M_PI), kMinRange, kMaxRange, kMaxTilt);
  EXPECT_FALSE(estimate.valid);
  EXPECT_STREQ(estimate.reason, "beam not pointing downwards");
}

}  // namespace
