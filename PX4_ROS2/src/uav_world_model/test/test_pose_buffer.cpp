#include <gtest/gtest.h>

#include "uav_world_model/pose_buffer.hpp"

using uav_world_model::PoseBuffer;
using uav_world_model::PoseBufferOptions;
using uav_world_model::PoseMatchResult;
using uav_world_model::TimedPose;
using uav_world_model::Vec3;

namespace
{

TimedPose poseAt(double stamp_sec, double x, double stddev = 0.1)
{
  TimedPose pose;
  pose.stamp_sec = stamp_sec;
  pose.odom_from_base.translation = Vec3{x, 0.0, 0.0};
  pose.position_stddev = stddev;
  return pose;
}

}  // namespace

TEST(PoseBuffer, AnEmptyBufferCannotPairAnything)
{
  const PoseBuffer buffer;
  EXPECT_EQ(buffer.lookup(100.0).result, PoseMatchResult::kNoPose);
}

TEST(PoseBuffer, APoseWithoutAStatedErrorIsRefused)
{
  PoseBuffer buffer;
  EXPECT_FALSE(buffer.add(poseAt(100.0, 0.0, -1.0)));
  EXPECT_TRUE(buffer.empty()) << "a pose that cannot state its error cannot be weighed";
}

TEST(PoseBuffer, APoseClaimingZeroErrorIsRefused)
{
  PoseBuffer buffer;
  EXPECT_FALSE(buffer.add(poseAt(100.0, 0.0, 0.0)));
  EXPECT_TRUE(buffer.empty());
}

TEST(PoseBuffer, AFreshObservationPairsWithTheNewestPose)
{
  PoseBuffer buffer;
  buffer.add(poseAt(100.0, 1.0));

  const auto match = buffer.lookup(100.1);
  ASSERT_TRUE(match.ok());
  EXPECT_NEAR(match.odom_from_base.translation.x, 1.0, 1e-12);
  EXPECT_NEAR(match.pairing_gap_sec, 0.1, 1e-12);
}

TEST(PoseBuffer, AnObservationBeyondTheGapIsRejected)
{
  PoseBufferOptions options;
  options.max_pose_gap_sec = 0.25;
  PoseBuffer buffer(options);
  buffer.add(poseAt(100.0, 1.0));

  EXPECT_EQ(buffer.lookup(100.5).result, PoseMatchResult::kObservationTooNew);
  EXPECT_EQ(buffer.lookup(99.5).result, PoseMatchResult::kObservationTooOld);
}

TEST(PoseBuffer, ObservationsAreInterpolatedNotSnappedToTheLatest)
{
  PoseBuffer buffer;
  buffer.add(poseAt(10.0, 0.0));
  buffer.add(poseAt(11.0, 2.0));

  const auto match = buffer.lookup(10.25);
  ASSERT_TRUE(match.ok());
  EXPECT_NEAR(match.odom_from_base.translation.x, 0.5, 1e-12);
  EXPECT_NEAR(match.pairing_gap_sec, 0.25, 1e-12);
}

TEST(PoseBuffer, InterpolationTakesTheLessCertainOfTheTwoPoses)
{
  PoseBuffer buffer;
  buffer.add(poseAt(10.0, 0.0, 0.1));
  buffer.add(poseAt(11.0, 2.0, 0.4));

  const auto match = buffer.lookup(10.5);
  ASSERT_TRUE(match.ok());
  EXPECT_NEAR(match.position_stddev, 0.4, 1e-12);
}

TEST(PoseBuffer, PairingErrorIsSpeedTimesTheTimeGap)
{
  PoseBuffer buffer;
  TimedPose pose = poseAt(100.0, 0.0);
  pose.body_velocity = Vec3{2.0, 0.0, 0.0};
  buffer.add(pose);

  const auto match = buffer.lookup(100.1);
  ASSERT_TRUE(match.ok());
  EXPECT_NEAR(match.pairingUncertainty(), 0.2, 1e-12);
}

TEST(PoseBuffer, AStationaryAircraftPaysNoPairingPenalty)
{
  PoseBuffer buffer;
  buffer.add(poseAt(100.0, 0.0));

  const auto match = buffer.lookup(100.2);
  ASSERT_TRUE(match.ok());
  EXPECT_NEAR(match.pairingUncertainty(), 0.0, 1e-12);
}

TEST(PoseBuffer, OldPosesAreDropped)
{
  PoseBufferOptions options;
  options.history_sec = 1.0;
  PoseBuffer buffer(options);

  buffer.add(poseAt(10.0, 0.0));
  buffer.add(poseAt(10.5, 1.0));
  buffer.add(poseAt(12.0, 2.0));

  EXPECT_EQ(buffer.size(), 1u);
  EXPECT_NEAR(buffer.newestStamp(), 12.0, 1e-12);
}

TEST(PoseBuffer, OutOfOrderPosesAreStillOrdered)
{
  PoseBuffer buffer;
  buffer.add(poseAt(11.0, 2.0));
  buffer.add(poseAt(10.0, 0.0));

  const auto match = buffer.lookup(10.5);
  ASSERT_TRUE(match.ok());
  EXPECT_NEAR(match.odom_from_base.translation.x, 1.0, 1e-12);
}
