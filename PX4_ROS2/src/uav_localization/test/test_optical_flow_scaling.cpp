#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "uav_localization/optical_flow_scaling.hpp"

using uav_localization::BodyRates;
using uav_localization::FlowGeometry;
using uav_localization::FlowLimits;
using uav_localization::FlowSample;
using uav_localization::velocityFromFlow;

namespace
{

FlowGeometry geometry(double height_m = 2.0)
{
  FlowGeometry value;
  value.fx = 500.0;
  value.fy = 500.0;
  value.height_m = height_m;
  return value;
}

FlowSample sample(double median_u_px, double median_v_px)
{
  FlowSample value;
  value.median_u_px = median_u_px;
  value.median_v_px = median_v_px;
  value.dt_sec = 0.1;
  value.tracked_features = 60;
  return value;
}

TEST(OpticalFlowScaling, StillGroundMeansStillAircraft)
{
  const auto velocity = velocityFromFlow(sample(0.0, 0.0), geometry(), BodyRates{}, FlowLimits{});

  ASSERT_TRUE(velocity.valid);
  EXPECT_NEAR(velocity.forward_mps, 0.0, 1e-9);
  EXPECT_NEAR(velocity.left_mps, 0.0, 1e-9);
}

TEST(OpticalFlowScaling, TranslationScalesWithHeightOverFocalLength)
{
  // 10 px, 2 m, f=500, 0.1 s -> 0.4 m/s
  const auto velocity = velocityFromFlow(sample(0.0, 10.0), geometry(), BodyRates{}, FlowLimits{});

  ASSERT_TRUE(velocity.valid);
  EXPECT_NEAR(velocity.forward_mps, 0.4, 1e-9);
  EXPECT_NEAR(velocity.left_mps, 0.0, 1e-9);
}

TEST(OpticalFlowScaling, TheSamePixelsMeanMoreSpeedHigherUp)
{
  const auto low = velocityFromFlow(sample(0.0, 10.0), geometry(2.0), BodyRates{}, FlowLimits{});
  const auto high = velocityFromFlow(sample(0.0, 10.0), geometry(8.0), BodyRates{}, FlowLimits{});

  EXPECT_NEAR(high.forward_mps, 4.0 * low.forward_mps, 1e-9);
}

TEST(OpticalFlowScaling, PitchingInPlaceIsNotFlyingForward)
{
  BodyRates rates;
  rates.pitch_rate = 0.5;
  // f*w*dt = 500*0.5*0.1 = 25 px, pure rotation
  const auto velocity = velocityFromFlow(sample(0.0, 25.0), geometry(), rates, FlowLimits{});

  ASSERT_TRUE(velocity.valid);
  EXPECT_NEAR(velocity.forward_mps, 0.0, 1e-9);
}

TEST(OpticalFlowScaling, RollingInPlaceIsNotFlyingSideways)
{
  BodyRates rates;
  rates.roll_rate = -0.4;
  const auto velocity = velocityFromFlow(sample(-20.0, 0.0), geometry(), rates, FlowLimits{});

  ASSERT_TRUE(velocity.valid);
  EXPECT_NEAR(velocity.left_mps, 0.0, 1e-9);
}

TEST(OpticalFlowScaling, WithoutCompensationRotationLooksLikeMetresPerSecond)
{
  BodyRates rates;
  rates.pitch_rate = 0.5;
  FlowLimits limits;
  limits.pitch_rate_sign = 0.0;      // compensation disabled

  const auto velocity = velocityFromFlow(sample(0.0, 25.0), geometry(), rates, limits);

  EXPECT_NEAR(velocity.forward_mps, 1.0, 1e-9);
}

TEST(OpticalFlowScaling, TexturelessGroundIsRefusedRatherThanGuessed)
{
  FlowSample bare = sample(0.0, 0.0);
  bare.tracked_features = 3;

  const auto velocity = velocityFromFlow(bare, geometry(), BodyRates{}, FlowLimits{});

  EXPECT_FALSE(velocity.valid);
  EXPECT_STREQ(velocity.reason, "too few tracked features");
}

TEST(OpticalFlowScaling, HeightMustBeKnownAndSensible)
{
  const FlowLimits limits;

  EXPECT_FALSE(
    velocityFromFlow(sample(0.0, 5.0), geometry(0.05), BodyRates{}, limits).valid);
  EXPECT_FALSE(
    velocityFromFlow(sample(0.0, 5.0), geometry(400.0), BodyRates{}, limits).valid);
  EXPECT_FALSE(
    velocityFromFlow(
      sample(0.0, 5.0), geometry(std::numeric_limits<double>::quiet_NaN()),
      BodyRates{}, limits).valid);
}

TEST(OpticalFlowScaling, MissingIntrinsicsOrTimeBaseAreRefused)
{
  FlowGeometry no_intrinsics = geometry();
  no_intrinsics.fx = 0.0;
  EXPECT_FALSE(
    velocityFromFlow(sample(0.0, 5.0), no_intrinsics, BodyRates{}, FlowLimits{}).valid);

  FlowSample no_time = sample(0.0, 5.0);
  no_time.dt_sec = 0.0;
  EXPECT_FALSE(velocityFromFlow(no_time, geometry(), BodyRates{}, FlowLimits{}).valid);
}

TEST(OpticalFlowScaling, AnAbsurdSpeedIsReportedAsBrokenTracking)
{
  const auto velocity =
    velocityFromFlow(sample(0.0, 5000.0), geometry(), BodyRates{}, FlowLimits{});

  EXPECT_FALSE(velocity.valid);
  EXPECT_NEAR(velocity.forward_mps, 0.0, 1e-9);
}

TEST(OpticalFlowScaling, UncertaintyGrowsWithHeight)
{
  const auto low = velocityFromFlow(sample(0.0, 5.0), geometry(2.0), BodyRates{}, FlowLimits{});
  const auto high = velocityFromFlow(sample(0.0, 5.0), geometry(8.0), BodyRates{}, FlowLimits{});

  EXPECT_GT(high.stddev_mps, low.stddev_mps);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
