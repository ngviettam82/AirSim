#include <array>
#include <cmath>

#include <gtest/gtest.h>

#include "uav_world_model/frame_transforms.hpp"
#include "uav_world_model/pose_buffer.hpp"
#include "uav_world_model/uncertainty.hpp"
#include "uav_world_model/world_map.hpp"

using uav_world_model::LandmarkObservation;
using uav_world_model::ObstacleBatch;
using uav_world_model::ObstacleObservation;
using uav_world_model::Rigid;
using uav_world_model::SensorMount;
using uav_world_model::TargetObservation;
using uav_world_model::Vec3;
using uav_world_model::WorldMap;
using uav_world_model::WorldMapOptions;

namespace
{

SensorMount downCameraMount()
{
  SensorMount mount;
  mount.frame_id = "uav0/camera_down_optical";
  mount.reports_optical_axes = true;
  mount.base_from_sensor.translation = Vec3{0.06, 0.0, -0.065};
  mount.base_from_sensor.rotation = uav_world_model::fromRollPitchYaw(0.0, M_PI / 2.0, 0.0);
  return mount;
}

LandmarkObservation markerAt(double stamp_sec, double x, double uncertainty = 0.4)
{
  LandmarkObservation observation;
  observation.marker_id = 7;
  observation.family = 1;
  observation.position = Vec3{x, 0.0, 0.0};
  observation.position_uncertainty_m = uncertainty;
  observation.stamp_sec = stamp_sec;
  return observation;
}

WorldMapOptions defaultOptions()
{
  WorldMapOptions options;
  options.drift.sigma_m = 0.5;
  options.drift.correlation_time_sec = 120.0;
  return options;
}

// TargetTrack::STATUS_TRACKING and STATUS_COASTING; the node pins the pair.
constexpr uint8_t kStatusTracking = 1;
constexpr uint8_t kStatusCoasting = 2;
constexpr uint8_t kStatusLost = uav_world_model::kTargetStatusLost;

TargetObservation trackAt(
  int32_t track_id, double stamp_sec, uint8_t status, double time_since_seen_sec, double x)
{
  TargetObservation observation;
  observation.track_id = track_id;
  observation.status = status;
  observation.position = Vec3{x, 0.0, 1.0};
  observation.position_uncertainty_m = 0.3;
  observation.stamp_sec = stamp_sec;
  observation.time_since_seen_sec = time_since_seen_sec;
  return observation;
}

}  // namespace

TEST(WorldModel, MarkerAndOdometryGiveTheHandComputedLandmark)
{
  // Fused pose: level, base_link at (10, 5, 2.565), 0.4 m stated error.
  std::array<double, 36> covariance{};
  covariance[0] = 0.16;
  covariance[7] = 0.16;
  covariance[14] = 0.16;

  uav_world_model::TimedPose pose;
  pose.stamp_sec = 100.0;
  pose.odom_from_base.translation = Vec3{10.0, 5.0, 2.565};
  pose.position_stddev = uav_world_model::worstAxisStddev(covariance);
  ASSERT_NEAR(pose.position_stddev, 0.4, 1e-12);

  uav_world_model::PoseBuffer buffer;
  ASSERT_TRUE(buffer.add(pose));

  const auto match = buffer.lookup(100.0);
  ASSERT_TRUE(match.ok());

  // Marker straight below the lens, 2.5 m away, detector states no error.
  const Vec3 in_optical{0.0, 0.0, 2.5};
  const Vec3 in_odom =
    uav_world_model::sensorPointToOdom(in_optical, downCameraMount(), match.odom_from_base);

  EXPECT_NEAR(in_odom.x, 10.06, 1e-9);
  EXPECT_NEAR(in_odom.y, 5.0, 1e-9);
  EXPECT_NEAR(in_odom.z, 0.0, 1e-9);

  const double perception = uav_world_model::perceptionUncertainty(
    -1.0F, uav_world_model::norm(in_optical), uav_world_model::UncertaintyFloor{0.0185, 0.007});
  const double combined = uav_world_model::combineQuadrature(
    {perception, match.position_stddev, match.pairingUncertainty()});

  EXPECT_NEAR(perception, 0.036, 1e-12);
  EXPECT_NEAR(combined, 0.4016167, 1e-6);

  WorldMap map(defaultOptions());
  LandmarkObservation landmark;
  landmark.marker_id = 7;
  landmark.family = 1;
  landmark.position = in_odom;
  landmark.position_uncertainty_m = combined;
  landmark.stamp_sec = 100.0;
  map.observeLandmark(landmark);

  const auto reported = map.landmarks(100.0);
  ASSERT_EQ(reported.size(), 1u);
  EXPECT_NEAR(reported[0].position.x, 10.06, 1e-9);
  EXPECT_NEAR(reported[0].position_uncertainty_m, 0.4016167, 1e-6);
  EXPECT_NEAR(reported[0].time_since_seen_sec, 0.0, 1e-12);
}

TEST(WorldModel, AgeIsReportedAndInflatesTheUncertainty)
{
  WorldMap map(defaultOptions());
  map.observeLandmark(markerAt(100.0, 1.0, 0.4016167));

  const auto fresh = map.landmarks(100.0);
  const auto aged = map.landmarks(105.0);
  ASSERT_EQ(aged.size(), 1u);

  EXPECT_NEAR(aged[0].time_since_seen_sec, 5.0, 1e-9);
  EXPECT_NEAR(aged[0].position_uncertainty_m, 0.426266, 1e-5);
  EXPECT_GT(aged[0].position_uncertainty_m, fresh[0].position_uncertainty_m);
  EXPECT_NEAR(aged[0].position.x, fresh[0].position.x, 1e-12) << "ageing moves no landmark";
}

TEST(WorldModel, LandmarksAreForgottenAfterTheWindow)
{
  WorldMapOptions options = defaultOptions();
  options.landmark_forget_sec = 120.0;
  WorldMap map(options);
  map.observeLandmark(markerAt(100.0, 1.0));

  map.expire(219.0);
  EXPECT_EQ(map.landmarkCount(), 1u);

  map.expire(221.0);
  EXPECT_EQ(map.landmarkCount(), 0u);
}

TEST(WorldModel, RepeatSightingsReplaceAndAreNotAveraged)
{
  WorldMap map(defaultOptions());
  map.observeLandmark(markerAt(100.0, 1.0));
  map.observeLandmark(markerAt(101.0, 3.0));

  const auto reported = map.landmarks(101.0);
  ASSERT_EQ(reported.size(), 1u);
  EXPECT_NEAR(reported[0].position.x, 3.0, 1e-12) << "averaging correlated errors would lie";
}

TEST(WorldModel, AnOlderSightingDoesNotOverwriteANewerOne)
{
  WorldMap map(defaultOptions());
  map.observeLandmark(markerAt(101.0, 3.0));
  map.observeLandmark(markerAt(100.0, 1.0));

  const auto reported = map.landmarks(101.0);
  ASSERT_EQ(reported.size(), 1u);
  EXPECT_NEAR(reported[0].position.x, 3.0, 1e-12);
}

TEST(WorldModel, DifferentMarkerIdsAreDifferentLandmarks)
{
  WorldMap map(defaultOptions());
  LandmarkObservation first = markerAt(100.0, 1.0);
  LandmarkObservation second = markerAt(100.0, 5.0);
  second.marker_id = 17;

  map.observeLandmark(first);
  map.observeLandmark(second);
  EXPECT_EQ(map.landmarkCount(), 2u);
}

TEST(WorldModel, NoObstacleSourceMeansNoObstacleClaim)
{
  const WorldMap map(defaultOptions());
  EXPECT_FALSE(map.hasObstacleSource()) << "an empty map would read as nothing to avoid";
}

TEST(WorldModel, ObstacleBatchesAreStampedByTheirOldestEntry)
{
  WorldMap map(defaultOptions());

  ObstacleBatch older;
  older.sensing_frame = "uav0/depth_front_optical";
  older.stamp_sec = 100.0;
  older.sensing_range_m = 10.0;
  older.obstacles.push_back(ObstacleObservation{});

  ObstacleBatch newer;
  newer.sensing_frame = "uav0/lidar_down";
  newer.stamp_sec = 102.0;
  newer.sensing_range_m = 4.0;
  newer.obstacles.push_back(ObstacleObservation{});

  map.observeObstacles(older);
  map.observeObstacles(newer);

  EXPECT_TRUE(map.hasObstacleSource());
  EXPECT_NEAR(map.oldestObstacleStamp(), 100.0, 1e-12);
  EXPECT_NEAR(map.newestObstacleStamp(), 102.0, 1e-12);
  EXPECT_NEAR(map.obstacleSensingRange(), 4.0, 1e-12) << "vouch only for the shorter reach";
  EXPECT_EQ(map.obstacles(102.0).size(), 2u);
}

TEST(WorldModel, StaleObstacleBatchesExpire)
{
  WorldMapOptions options = defaultOptions();
  options.obstacle_forget_sec = 5.0;
  WorldMap map(options);

  ObstacleBatch batch;
  batch.sensing_frame = "uav0/depth_front_optical";
  batch.stamp_sec = 100.0;
  batch.obstacles.push_back(ObstacleObservation{});
  map.observeObstacles(batch);

  map.expire(106.0);
  EXPECT_TRUE(map.obstacles(106.0).empty());
  EXPECT_TRUE(map.hasObstacleSource()) << "the source existed; only its data went stale";
}

TEST(WorldModel, ObstacleUncertaintyGrowsWithTheAgeOfItsBatch)
{
  WorldMap map(defaultOptions());

  ObstacleBatch batch;
  batch.sensing_frame = "uav0/depth_front_optical";
  batch.stamp_sec = 100.0;
  ObstacleObservation obstacle;
  obstacle.position_uncertainty_m = 0.5;
  batch.obstacles.push_back(obstacle);
  map.observeObstacles(batch);

  const auto fresh = map.obstacles(100.0);
  const auto aged = map.obstacles(103.0);
  ASSERT_EQ(aged.size(), 1u);
  EXPECT_NEAR(aged[0].time_since_seen_sec, 3.0, 1e-9);
  EXPECT_GT(aged[0].observation.position_uncertainty_m, fresh[0].observation.position_uncertainty_m);
}

TEST(WorldModel, NoTargetUntilOneHasBeenSeen)
{
  WorldMap map(defaultOptions());
  EXPECT_FALSE(map.target(100.0).valid) << "no pose to invent, no uncertainty to state";

  TargetObservation target;
  target.track_id = 3;
  target.position = Vec3{2.0, 0.0, 1.0};
  target.position_uncertainty_m = 0.3;
  target.stamp_sec = 100.0;
  map.observeTarget(target);

  const auto reported = map.target(100.0);
  ASSERT_TRUE(reported.valid);
  EXPECT_EQ(reported.observation.track_id, 3);
  EXPECT_NEAR(reported.position_uncertainty_m, 0.3, 1e-12);
}

TEST(WorldModel, TargetsExpireAndTheirUncertaintyGrowsFirst)
{
  WorldMapOptions options = defaultOptions();
  options.target_forget_sec = 10.0;
  WorldMap map(options);

  TargetObservation target;
  target.position_uncertainty_m = 0.3;
  target.stamp_sec = 100.0;
  map.observeTarget(target);

  EXPECT_GT(map.target(108.0).position_uncertainty_m, 0.3);
  map.expire(111.0);
  EXPECT_FALSE(map.target(111.0).valid);
}

TEST(WorldModel, TargetFreshnessAddsTransportAgeToTheSightingAge)
{
  WorldMap map(defaultOptions());
  map.observeTarget(trackAt(3, 100.0, kStatusCoasting, 2.5, 2.0));

  const auto reported = map.target(100.4);
  ASSERT_TRUE(reported.valid);
  EXPECT_NEAR(reported.time_since_seen_sec, 2.9, 1e-9)
    << "a target hidden 2.5 s may not be reported as 0.4 s old";
}

TEST(WorldModel, AHiddenTargetIsMoreUncertainThanOneJustSeen)
{
  WorldMap just_seen(defaultOptions());
  just_seen.observeTarget(trackAt(3, 100.0, kStatusTracking, 0.0, 2.0));

  WorldMap hidden(defaultOptions());
  hidden.observeTarget(trackAt(3, 100.0, kStatusCoasting, 2.5, 2.0));

  EXPECT_GT(
    hidden.target(100.4).position_uncertainty_m,
    just_seen.target(100.4).position_uncertainty_m)
    << "the estimate dates from the sighting, not from the message";
}

TEST(WorldModel, ATargetHiddenPastTheForgetWindowIsDropped)
{
  WorldMapOptions options = defaultOptions();
  options.target_forget_sec = 10.0;
  WorldMap map(options);
  map.observeTarget(trackAt(3, 100.0, kStatusCoasting, 9.5, 2.0));

  map.expire(100.6);
  EXPECT_FALSE(map.target(100.6).valid)
    << "a fresh message about a 10.1 s old sighting keeps nothing alive";
}

TEST(WorldModel, TheLastTrackInABatchDoesNotStealTheSelection)
{
  WorldMap map(defaultOptions());
  map.observeTarget(trackAt(3, 100.0, kStatusTracking, 0.0, 2.0));
  map.observeTarget(trackAt(1, 100.0, kStatusLost, 2.5, 9.0));

  const auto reported = map.target(100.0);
  ASSERT_TRUE(reported.valid);
  EXPECT_EQ(reported.observation.track_id, 3);
  EXPECT_NEAR(reported.observation.position.x, 2.0, 1e-12) << "a 7 m jump in the aim point";
  EXPECT_EQ(map.targetTrackSwitchCount(), 0u);
  EXPECT_EQ(map.unselectedTargetTrackCount(), 1u);
}

TEST(WorldModel, SelectionWithinABatchDoesNotDependOnArrivalOrder)
{
  const TargetObservation freshest = trackAt(3, 100.0, kStatusTracking, 0.0, 2.0);
  const TargetObservation stale = trackAt(1, 100.0, kStatusTracking, 2.5, 9.0);

  WorldMap freshest_first(defaultOptions());
  freshest_first.observeTarget(freshest);
  freshest_first.observeTarget(stale);

  WorldMap stale_first(defaultOptions());
  stale_first.observeTarget(stale);
  stale_first.observeTarget(freshest);

  EXPECT_EQ(freshest_first.target(100.0).observation.track_id, 3);
  EXPECT_EQ(stale_first.target(100.0).observation.track_id, 3)
    << "the freshest sighting wins the batch, never the last message in it";
}

TEST(WorldModel, AnOtherwiseEqualBatchIsBrokenByTheSmallerTrackId)
{
  const TargetObservation lower = trackAt(1, 100.0, kStatusTracking, 0.0, 9.0);
  const TargetObservation higher = trackAt(3, 100.0, kStatusTracking, 0.0, 2.0);

  WorldMap lower_first(defaultOptions());
  lower_first.observeTarget(lower);
  lower_first.observeTarget(higher);

  WorldMap higher_first(defaultOptions());
  higher_first.observeTarget(higher);
  higher_first.observeTarget(lower);

  EXPECT_EQ(lower_first.target(100.0).observation.track_id, 1);
  EXPECT_EQ(higher_first.target(100.0).observation.track_id, 1)
    << "ties need a stated rule, or arrival order becomes the rule";
}

TEST(WorldModel, ALostTrackNeverStealsALiveSelection)
{
  WorldMap map(defaultOptions());
  map.observeTarget(trackAt(3, 100.0, kStatusTracking, 0.0, 2.0));
  map.observeTarget(trackAt(7, 100.1, kStatusLost, 0.0, 9.0));

  EXPECT_EQ(map.target(100.1).observation.track_id, 3);
  EXPECT_EQ(map.targetTrackSwitchCount(), 0u);
}

TEST(WorldModel, TheSelectionMovesOnWhenTheHeldTrackIsLost)
{
  WorldMap map(defaultOptions());
  map.observeTarget(trackAt(3, 100.0, kStatusTracking, 0.0, 2.0));
  map.observeTarget(trackAt(3, 100.1, kStatusLost, 3.0, 2.0));
  map.observeTarget(trackAt(1, 100.2, kStatusTracking, 0.0, 9.0));

  EXPECT_EQ(map.target(100.2).observation.track_id, 1);
  EXPECT_EQ(map.targetTrackSwitchCount(), 1u)
    << "changing target is an event the consumer must be able to see";
}

TEST(WorldModel, TheSelectionHoldsUntilTheHeldTrackGoesSilent)
{
  WorldMapOptions options = defaultOptions();
  options.target_track_switch_after_sec = 1.0;
  WorldMap map(options);
  map.observeTarget(trackAt(3, 100.0, kStatusTracking, 0.0, 2.0));

  map.observeTarget(trackAt(1, 100.5, kStatusTracking, 0.0, 9.0));
  EXPECT_EQ(map.target(100.5).observation.track_id, 3) << "one silent gap is not a lost track";

  map.observeTarget(trackAt(1, 101.2, kStatusTracking, 0.0, 9.0));
  EXPECT_EQ(map.target(101.2).observation.track_id, 1);
  EXPECT_EQ(map.targetTrackSwitchCount(), 1u);
}

TEST(WorldModel, AnOlderMessageFromTheHeldTrackDoesNotRewindIt)
{
  WorldMap map(defaultOptions());
  map.observeTarget(trackAt(3, 101.0, kStatusTracking, 0.0, 5.0));
  map.observeTarget(trackAt(3, 100.0, kStatusTracking, 0.0, 1.0));

  EXPECT_NEAR(map.target(101.0).observation.position.x, 5.0, 1e-12);
}

TEST(WorldModel, ReportedUncertaintyIsNeverZeroOrUnknown)
{
  WorldMap map(defaultOptions());
  map.observeLandmark(markerAt(100.0, 1.0, 0.4016167));

  for (const double now_sec : {100.0, 100.5, 110.0, 200.0}) {
    const auto reported = map.landmarks(now_sec);
    ASSERT_EQ(reported.size(), 1u);
    EXPECT_GT(reported[0].position_uncertainty_m, 0.0);
  }
}
