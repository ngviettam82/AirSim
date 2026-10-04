#include <gtest/gtest.h>

#include <vector>

#include "uav_perception/camera_health_checks.hpp"

using uav_perception::ContentCheck;
using uav_perception::DeliveryCheck;
using uav_perception::GapCheck;
using uav_perception::HealthLevel;
using uav_perception::StreamDelivery;

namespace
{

// Sensor at source_hz; only every keep_every-th frame survives.
StreamDelivery streamLosing(double source_hz, int keep_every, int frames)
{
  StreamDelivery delivery;
  const double period = 1.0 / source_hz;
  for (int index = 0; index < frames; ++index) {
    if (index % keep_every == 0) {
      delivery.record(index * period);
    }
  }
  return delivery;
}

}  // namespace

TEST(StreamDelivery, InfersSourceRateFromTheShortestGap)
{
  // Pins: source rate is inferred, never configured.
  StreamDelivery delivery = streamLosing(15.0, 1, 30);
  EXPECT_NEAR(delivery.sourceHz(), 15.0, 0.1);
  EXPECT_NEAR(delivery.deliveredFraction(), 1.0, 0.01);
  EXPECT_EQ(delivery.lostFrames(), 0u);
}

TEST(StreamDelivery, SeesEverySecondFrameMissing)
{
  // Pins the known blind spot (see README).
  StreamDelivery delivery = streamLosing(15.0, 2, 40);
  EXPECT_NEAR(delivery.sourceHz(), 7.5, 0.1) <<
    "with no consecutive frames the shortest gap is two periods";
  EXPECT_NEAR(delivery.deliveredFraction(), 1.0, 0.01);
}

TEST(StreamDelivery, DetectsLossWhenSomeFramesAreConsecutive)
{
  // Confirms period holds even with mostly-missing frames.
  StreamDelivery delivery;
  const double period = 1.0 / 15.0;
  delivery.record(0.0);
  delivery.record(period);          // consecutive pair reveals the period
  delivery.record(3 * period);      // one frame lost
  delivery.record(5 * period);      // one more lost
  EXPECT_NEAR(delivery.sourceHz(), 15.0, 0.1);
  EXPECT_EQ(delivery.lostFrames(), 2u);
  EXPECT_LT(delivery.deliveredFraction(), 0.8);
}

TEST(StreamDelivery, LongestGapSurvivesAveraging)
{
  StreamDelivery delivery;
  delivery.record(0.0);
  delivery.record(0.1);
  delivery.record(1.9);             // a hole a tracker cannot coast across
  delivery.record(2.0);
  EXPECT_NEAR(delivery.longestGapSec(), 1.8, 1e-6);
}

TEST(StreamDelivery, IgnoresBackwardsStamps)
{
  // Pins: clock reset != early frame (see README).
  StreamDelivery delivery;
  delivery.record(10.0);
  delivery.record(10.1);
  delivery.record(0.0);
  delivery.record(0.1);
  EXPECT_GT(delivery.sourceHz(), 0.0);
  EXPECT_NEAR(delivery.sourceHz(), 10.0, 0.5);
}

TEST(StreamDelivery, WindowResetKeepsTheInferredPeriod)
{
  StreamDelivery delivery = streamLosing(15.0, 1, 20);
  const double before = delivery.sourceHz();
  delivery.startWindow();
  EXPECT_EQ(delivery.received(), 0u);
  EXPECT_NEAR(delivery.sourceHz(), before, 1e-9) <<
    "the period describes the camera, not the reporting window";
}

TEST(Classify, DeliveryThresholds)
{
  const DeliveryCheck check;
  EXPECT_EQ(uav_perception::classifyDelivery(0.95, check), HealthLevel::Ok);
  EXPECT_EQ(uav_perception::classifyDelivery(0.60, check), HealthLevel::Warn);
  EXPECT_EQ(uav_perception::classifyDelivery(0.45, check), HealthLevel::Error);
}

TEST(Classify, GapThresholds)
{
  const GapCheck check;
  EXPECT_EQ(uav_perception::classifyGap(0.1, check), HealthLevel::Ok);
  EXPECT_EQ(uav_perception::classifyGap(0.5, check), HealthLevel::Warn);
  EXPECT_EQ(uav_perception::classifyGap(1.8, check), HealthLevel::Error);
}

TEST(Classify, FlatImageOnlyWarns)
{
  // Pins: flat sky must never reach Error (see README).
  const ContentCheck check;
  EXPECT_EQ(uav_perception::classifyContent(1, check), HealthLevel::Warn);
  EXPECT_NE(uav_perception::classifyContent(1, check), HealthLevel::Error);
  EXPECT_EQ(uav_perception::classifyContent(54, check), HealthLevel::Ok);
}

TEST(DistinctValues, SamplesTheWholeFrameNotItsStart)
{
  // Pins the top-left sky sampling bug (see README).
  std::vector<std::uint8_t> frame(20000, 7);
  for (std::size_t index = frame.size() / 2; index < frame.size(); ++index) {
    frame[index] = static_cast<std::uint8_t>(index % 251);
  }
  EXPECT_GT(uav_perception::distinctValuesAcross(frame.data(), frame.size()), 5u);
}

TEST(DistinctValues, HandlesEmptyBuffer)
{
  EXPECT_EQ(uav_perception::distinctValuesAcross(nullptr, 0), 0u);
}

TEST(CameraInfo, CatchesTheSharedCameraInfoCollision)
{
  // Pins the shared camera_info collision bug (see README).
  EXPECT_TRUE(uav_perception::infoMatchesImage(640, 480, 432.5, 640, 480));
  EXPECT_FALSE(uav_perception::infoMatchesImage(1280, 720, 865.0, 640, 480));
  EXPECT_FALSE(uav_perception::infoMatchesImage(640, 480, 0.0, 640, 480));
}

TEST(Worst, KeepsTheHigherLevel)
{
  EXPECT_EQ(uav_perception::worst(HealthLevel::Ok, HealthLevel::Warn), HealthLevel::Warn);
  EXPECT_EQ(uav_perception::worst(HealthLevel::Error, HealthLevel::Warn), HealthLevel::Error);
}
