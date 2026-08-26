#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <uav_interfaces/msg/marker_observation.hpp>
#include <uav_interfaces/msg/obstacle_array.hpp>
#include <uav_interfaces/msg/semantic_landmark_array.hpp>
#include <uav_interfaces/msg/target_state.hpp>
#include <uav_interfaces/msg/target_track.hpp>

#include "uav_world_model/uncertainty.hpp"
#include "uav_world_model/world_model_node.hpp"

using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;
using uav_interfaces::msg::MarkerObservation;
using uav_interfaces::msg::Obstacle;
using uav_interfaces::msg::ObstacleArray;
using uav_interfaces::msg::SemanticLandmarkArray;
using uav_interfaces::msg::TargetState;
using uav_interfaces::msg::TargetTrack;
using namespace std::chrono_literals;

namespace
{

constexpr double kBaseLinkX = 10.0;
constexpr double kBaseLinkY = 5.0;
constexpr double kBaseLinkZ = 2.565;
constexpr double kFusedStddev = 0.4;
constexpr double kMarkerRange = 2.5;
constexpr double kMaxPoseGapSec = 0.1;

// Mount 0.06 ahead, 0.065 below base_link; P5.3 floor 0.0185 + 0.007 * range.
constexpr double kExpectedX = kBaseLinkX + 0.06;
constexpr double kExpectedY = kBaseLinkY;
constexpr double kExpectedZ = kBaseLinkZ - 0.065 - kMarkerRange;

constexpr double kFrontRange = 3.0;
// optical (0,0,3) -> body (3,0,0), plus mount (0.12, 0.03, 0.002).
constexpr double kFrontInBaseX = kFrontRange + 0.12;
constexpr double kFrontInBaseY = 0.03;
constexpr double kFrontInBaseZ = 0.002;

// Exactly what obstacle_extractor_node stamps on a front-camera batch.
const char kFrontFrame[] = "uav0/camera_front_optical";
const char kDownFrame[] = "uav0/camera_down_optical";
const double kExpectedWhenFresh =
  std::sqrt(std::pow(0.0185 + 0.007 * kMarkerRange, 2) + kFusedStddev * kFusedStddev);

// A landmark ages between being seen and being published; that is reported.
double expectedUncertaintyAt(float age_sec)
{
  return uav_world_model::combineQuadrature(
    {kExpectedWhenFresh,
      uav_world_model::driftGrowth(age_sec, uav_world_model::DriftModel{0.5, 120.0})});
}

class WorldModelFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();

    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("use_sim_time", false),
      rclcpp::Parameter("publish_rate_hz", 50.0),
      rclcpp::Parameter("max_pose_gap_sec", kMaxPoseGapSec),
      rclcpp::Parameter("report_period_sec", 0.0),
    });

    node_ = uav_world_model::createWorldModelNode(options);
    probe_ = std::make_shared<rclcpp::Node>("world_model_probe");

    odometry_publisher_ =
      probe_->create_publisher<Odometry>("/uav/uav0/state/odometry_fused", 20);
    marker_publisher_ =
      probe_->create_publisher<MarkerObservation>("/uav/uav0/perception/markers", 20);
    obstacle_publisher_ = probe_->create_publisher<ObstacleArray>(
      "/uav/uav0/perception/obstacles_local", rclcpp::SensorDataQoS());
    track_publisher_ = probe_->create_publisher<TargetTrack>(
      "/uav/uav0/perception/target_track", rclcpp::SensorDataQoS());

    landmark_subscription_ = probe_->create_subscription<SemanticLandmarkArray>(
      "/uav/uav0/world/semantic_landmarks", 10,
      [this](const SemanticLandmarkArray::SharedPtr message) {
        landmarks_ = *message;
        ++landmark_updates_;
      });
    reference_subscription_ = probe_->create_subscription<PoseStamped>(
      "/uav/uav0/world/mission_reference",
      rclcpp::QoS(1).transient_local().reliable(),
      [this](const PoseStamped::SharedPtr message) {
        mission_reference_ = *message;
        reference_received_ = true;
      });
    obstacle_subscription_ = probe_->create_subscription<ObstacleArray>(
      "/uav/uav0/world/obstacle_map_local", 10,
      [this](const ObstacleArray::SharedPtr message) {
        world_obstacles_ = *message;
        obstacles_received_ = true;
      });
    target_subscription_ = probe_->create_subscription<TargetState>(
      "/uav/uav0/world/target_state", 10,
      [this](const TargetState::SharedPtr message) {
        world_target_ = *message;
        target_received_ = true;
      });

    executor_->add_node(node_);
    executor_->add_node(probe_);
  }

  void TearDown() override
  {
    executor_->remove_node(probe_);
    executor_->remove_node(node_);
  }

  // Wall time is only the hang valve here; evidence decides.
  bool spinUntil(
    const std::function<bool()> & done, std::chrono::milliseconds timeout,
    bool stream_odometry = true)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (stream_odometry) {
        publishOdometry(kFusedStddev);
      }
      executor_->spin_some();
      if (done()) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return done();
  }

  // The node anchors its mission reference on the first usable pose.
  bool waitForTheNodeToHaveAPose()
  {
    return spinUntil([this]() {return reference_received_;}, 10s);
  }

  // A first sample lost to discovery would prove nothing.
  bool waitForTheNodeToSubscribe()
  {
    return spinUntil(
      [this]() {
        return odometry_publisher_->get_subscription_count() > 0 &&
               marker_publisher_->get_subscription_count() > 0;
      },
      10s, false);
  }

  bool waitForTheFusedPoseToGoStale()
  {
    return spinUntil(
      [this]() {return secondsSinceLastOdometry() > 5.0 * kMaxPoseGapSec;}, 5s, false);
  }

  // Its own publish cycle, not the clock, counts chances.
  bool spinForWorldUpdates(uint32_t updates)
  {
    const uint32_t target = landmark_updates_ + updates;
    return spinUntil([this, target]() {return landmark_updates_ >= target;}, 10s);
  }

  // Drives the shipped YAML itself, never a copy of its numbers.
  std::shared_ptr<rclcpp::Node> startNodeWithShippedConfig()
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", UAV_WORLD_MODEL_SHIPPED_CONFIG});
    options.parameter_overrides({
      rclcpp::Parameter("use_sim_time", false),
      rclcpp::Parameter("publish_rate_hz", 50.0),
      rclcpp::Parameter("max_pose_gap_sec", kMaxPoseGapSec),
      rclcpp::Parameter("report_period_sec", 0.0),
    });

    auto shipped = uav_world_model::createWorldModelNode(options);
    executor_->remove_node(node_);
    executor_->add_node(shipped);
    return shipped;
  }

  void stopShippedConfigNode(const std::shared_ptr<rclcpp::Node> & shipped)
  {
    executor_->remove_node(shipped);
    executor_->add_node(node_);
  }

  void publishOdometry(double position_stddev, double orientation_w = 1.0)
  {
    const rclcpp::Time stamp = probe_->now();

    Odometry message;
    message.header.stamp = stamp;
    message.header.frame_id = "odom";
    message.child_frame_id = "base_link";
    message.pose.pose.position.x = base_link_x_;
    message.pose.pose.position.y = kBaseLinkY;
    message.pose.pose.position.z = base_link_z_;
    // Scaling by orientation_w keeps the all-zero invalid case reachable.
    message.pose.pose.orientation.z = orientation_w * std::sin(0.5 * yaw_rad_);
    message.pose.pose.orientation.w = orientation_w * std::cos(0.5 * yaw_rad_);
    message.twist.twist.angular.z = yaw_rate_;
    const double variance = position_stddev * position_stddev;
    message.pose.covariance[0] = variance;
    message.pose.covariance[7] = variance;
    message.pose.covariance[14] = variance;
    odometry_publisher_->publish(message);
    last_odometry_stamp_ = stamp;
  }

  double secondsSinceLastOdometry() const
  {
    return (probe_->now() - last_odometry_stamp_).seconds();
  }

  void publishMarker(
    const std::string & frame_id, int32_t marker_id, float uncertainty,
    const rclcpp::Time & stamp)
  {
    MarkerObservation message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id;
    message.uav_id = "uav0";
    message.marker_id = marker_id;
    message.family = MarkerObservation::FAMILY_ARUCO;
    message.pose.position.z = kMarkerRange;
    message.pose.orientation.w = 1.0;
    message.size = 0.5F;
    message.confidence = 1.0F;
    message.position_uncertainty = uncertainty;
    marker_publisher_->publish(message);
  }

  void publishMarker(const std::string & frame_id, int32_t marker_id, float uncertainty = -1.0F)
  {
    publishMarker(frame_id, marker_id, uncertainty, probe_->now());
  }

  // Perception repeats itself; one shot could race the pose.
  bool mapMarkerFrom(const std::string & frame_id, int32_t marker_id, float uncertainty = -1.0F)
  {
    return spinUntil(
      [this, &frame_id, marker_id, uncertainty]() {
        if (findLandmark(marker_id) != nullptr) {
          return true;
        }
        publishMarker(frame_id, marker_id, uncertainty);
        return false;
      },
      5s);
  }

  bool mapMarker(int32_t marker_id, float uncertainty = -1.0F)
  {
    return mapMarkerFrom(kDownFrame, marker_id, uncertainty);
  }

  const uav_interfaces::msg::SemanticLandmark * findLandmark(int32_t marker_id) const
  {
    for (const auto & landmark : landmarks_.landmarks) {
      if (landmark.marker_id == marker_id) {
        return &landmark;
      }
    }
    return nullptr;
  }

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> probe_;
  rclcpp::Publisher<Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<MarkerObservation>::SharedPtr marker_publisher_;
  rclcpp::Subscription<SemanticLandmarkArray>::SharedPtr landmark_subscription_;
  rclcpp::Subscription<PoseStamped>::SharedPtr reference_subscription_;
  rclcpp::Subscription<ObstacleArray>::SharedPtr obstacle_subscription_;

  void publishObstacle(
    const std::string & frame_id, double optical_x, double optical_y, double optical_z,
    double extent_m)
  {
    ObstacleArray message;
    message.header.stamp = probe_->now();
    message.header.frame_id = frame_id;
    message.uav_id = "uav0";
    message.sensing_range = 10.0F;

    Obstacle obstacle;
    obstacle.obstacle_id = 1;
    obstacle.shape = Obstacle::SHAPE_BOX;
    obstacle.center.x = optical_x;
    obstacle.center.y = optical_y;
    obstacle.center.z = optical_z;
    obstacle.size.x = extent_m;
    obstacle.size.y = extent_m;
    obstacle.size.z = extent_m;
    obstacle.distance = 99.0F;          // wrong on purpose: the map must recompute
    obstacle.confidence = 1.0F;
    obstacle.position_uncertainty = -1.0F;
    message.obstacles.push_back(obstacle);
    obstacle_publisher_->publish(message);
  }

  // Best effort: a single sample can be dropped, so re-observe.
  bool observeObstacleUntilMapped(
    const std::string & frame_id, double optical_x, double optical_y, double optical_z,
    double extent_m)
  {
    return spinUntil(
      [&]() {
        if (obstacles_received_) {
          return true;
        }
        publishObstacle(frame_id, optical_x, optical_y, optical_z, extent_m);
        return false;
      },
      5s);
  }

  void publishTrack(
    const std::string & frame_id, int32_t track_id, uint8_t status, float time_since_seen_sec,
    double optical_x, double optical_y, double optical_z, const rclcpp::Time & stamp)
  {
    TargetTrack message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id;
    message.uav_id = "uav0";
    message.track_id = track_id;
    message.status = status;
    message.pose.position.x = optical_x;
    message.pose.position.y = optical_y;
    message.pose.position.z = optical_z;
    message.pose.orientation.w = 1.0;
    message.confidence = 1.0F;
    message.position_uncertainty = -1.0F;
    message.time_since_seen_sec = time_since_seen_sec;
    track_publisher_->publish(message);
  }

  void publishTrack(
    const std::string & frame_id, double optical_x, double optical_y, double optical_z)
  {
    publishTrack(
      frame_id, 5, TargetTrack::STATUS_TRACKING, 0.0F, optical_x, optical_y, optical_z,
      probe_->now());
  }

  bool observeTrackUntilMapped(
    const std::string & frame_id, double optical_x, double optical_y, double optical_z)
  {
    return spinUntil(
      [&]() {
        if (target_received_) {
          return true;
        }
        publishTrack(frame_id, optical_x, optical_y, optical_z);
        return false;
      },
      5s);
  }

  rclcpp::Publisher<ObstacleArray>::SharedPtr obstacle_publisher_;
  rclcpp::Publisher<TargetTrack>::SharedPtr track_publisher_;
  rclcpp::Subscription<TargetState>::SharedPtr target_subscription_;

  SemanticLandmarkArray landmarks_;
  PoseStamped mission_reference_;
  ObstacleArray world_obstacles_;
  TargetState world_target_;
  rclcpp::Time last_odometry_stamp_{0, 0, RCL_ROS_TIME};
  double base_link_x_{kBaseLinkX};
  double base_link_z_{kBaseLinkZ};
  double yaw_rad_{0.0};
  double yaw_rate_{0.0};
  uint32_t landmark_updates_{0};
  bool reference_received_{false};
  bool obstacles_received_{false};
  bool target_received_{false};
};

}  // namespace

TEST_F(WorldModelFixture, PublishesAnEmptyLandmarkMapBeforeAnythingIsSeen)
{
  ASSERT_TRUE(spinUntil([this]() {return landmark_updates_ > 0;}, 10s, false));
  EXPECT_EQ(landmarks_.header.frame_id, "odom");
  EXPECT_EQ(landmarks_.uav_id, "uav0");
  EXPECT_TRUE(landmarks_.landmarks.empty());
}

TEST_F(WorldModelFixture, MarkerPlusOdometryGivesTheHandComputedLandmark)
{
  ASSERT_TRUE(waitForTheNodeToHaveAPose());
  ASSERT_TRUE(mapMarker(7));

  const auto * landmark = findLandmark(7);
  ASSERT_NE(landmark, nullptr);
  EXPECT_NEAR(landmark->pose.position.x, kExpectedX, 1e-6);
  EXPECT_NEAR(landmark->pose.position.y, kExpectedY, 1e-6);
  EXPECT_NEAR(landmark->pose.position.z, kExpectedZ, 1e-6);
  EXPECT_NEAR(
    landmark->position_uncertainty,
    expectedUncertaintyAt(landmark->time_since_seen_sec), 1e-4);
  EXPECT_GT(landmark->position_uncertainty, kFusedStddev)
    << "perception error must be added, not dropped";
  EXPECT_GT(landmark->position_uncertainty, 0.0F) << "the world layer may never say -1 or 0";
  EXPECT_EQ(landmarks_.header.frame_id, "odom");
}

TEST_F(WorldModelFixture, TheDetectorSentinelAndZeroBothMeanTheMeasuredFloor)
{
  ASSERT_TRUE(waitForTheNodeToHaveAPose());
  ASSERT_TRUE(mapMarker(7, -1.0F));
  ASSERT_TRUE(mapMarker(8, 0.0F));

  for (const int32_t marker_id : {7, 8}) {
    const auto * landmark = findLandmark(marker_id);
    ASSERT_NE(landmark, nullptr);
    EXPECT_NEAR(
      landmark->position_uncertainty,
      expectedUncertaintyAt(landmark->time_since_seen_sec), 1e-4)
      << "marker " << marker_id << ": neither -1 nor 0 states an error";
  }
}

TEST_F(WorldModelFixture, AnObservationInAnUnconfiguredFrameIsDropped)
{
  ASSERT_TRUE(waitForTheNodeToHaveAPose());

  publishMarker("some_frame_nobody_configured", 42);
  // Same publisher and topic: 43 is handled after 42.
  ASSERT_TRUE(mapMarker(43)) << "control: the pipeline was live the whole time";

  EXPECT_EQ(findLandmark(42), nullptr) << "guessing a mount would move every landmark";
}

TEST_F(WorldModelFixture, AnObservationWithoutAFreshPoseIsDropped)
{
  ASSERT_TRUE(waitForTheNodeToHaveAPose());
  ASSERT_TRUE(waitForTheFusedPoseToGoStale());

  const rclcpp::Time stale_pose_stamp = last_odometry_stamp_;
  publishMarker(kDownFrame, 11);
  publishMarker(kDownFrame, 13, -1.0F, stale_pose_stamp);

  // Same publisher and topic: 13 is handled after 11.
  ASSERT_TRUE(spinUntil([this]() {return findLandmark(13) != nullptr;}, 5s, false))
    << "control: an observation stamped at the last pose still maps";
  EXPECT_EQ(findLandmark(11), nullptr) << "no fused pose within max_pose_gap_sec of it";

  ASSERT_TRUE(mapMarker(12)) << "control: it maps again once the pose is fresh";
}

TEST_F(WorldModelFixture, APoseWithoutAStatedErrorIsNotUsedForMapping)
{
  ASSERT_TRUE(waitForTheNodeToSubscribe());

  base_link_x_ = kBaseLinkX + 100.0;      // anchoring here would be unmistakable
  for (int index = 0; index < 8; ++index) {
    publishOdometry(0.0);                 // covariance all zeros: ROS default, no error stated
    executor_->spin_some();
    std::this_thread::sleep_for(5ms);
  }
  publishMarker(kDownFrame, 21);
  base_link_x_ = kBaseLinkX;

  // Aged out of reach: no later pose can rescue it.
  ASSERT_TRUE(waitForTheFusedPoseToGoStale());
  ASSERT_TRUE(waitForTheNodeToHaveAPose());
  ASSERT_TRUE(mapMarker(22)) << "control: a pose that states its error is accepted";

  EXPECT_EQ(findLandmark(21), nullptr) << "a map built on an unweighable pose is a guess";
  EXPECT_NEAR(mission_reference_.pose.position.x, kBaseLinkX, 1e-9)
    << "such a pose may not anchor the mission reference";
}

TEST_F(WorldModelFixture, APoseWithoutAnAttitudeIsNotUsedForMapping)
{
  ASSERT_TRUE(waitForTheNodeToSubscribe());

  base_link_x_ = kBaseLinkX + 100.0;
  for (int index = 0; index < 8; ++index) {
    publishOdometry(kFusedStddev, 0.0);   // all-zero quaternion, never a real attitude
    executor_->spin_some();
    std::this_thread::sleep_for(5ms);
  }
  publishMarker(kDownFrame, 31);
  base_link_x_ = kBaseLinkX;

  ASSERT_TRUE(waitForTheFusedPoseToGoStale());
  ASSERT_TRUE(waitForTheNodeToHaveAPose());
  ASSERT_TRUE(mapMarker(32)) << "control: a pose with an attitude is accepted";

  EXPECT_EQ(findLandmark(31), nullptr) << "identity would silently mean level facing north";
  EXPECT_NEAR(mission_reference_.pose.position.x, kBaseLinkX, 1e-9);
}

TEST_F(WorldModelFixture, AgeAndUncertaintyGrowWhileTheMarkerIsNotSeenAgain)
{
  ASSERT_TRUE(waitForTheNodeToHaveAPose());
  ASSERT_TRUE(mapMarker(7));

  const float fresh_age = findLandmark(7)->time_since_seen_sec;
  const float fresh_uncertainty = findLandmark(7)->position_uncertainty;
  const double fresh_x = findLandmark(7)->pose.position.x;

  ASSERT_TRUE(
    spinUntil(
      [this, fresh_age]() {
        const auto * ageing = findLandmark(7);
        return ageing != nullptr && ageing->time_since_seen_sec > fresh_age + 0.4F;
      },
      5s))
    << "the reported age must grow while the marker is not seen again";

  const auto * aged = findLandmark(7);
  ASSERT_NE(aged, nullptr);
  EXPECT_GT(aged->position_uncertainty, fresh_uncertainty);
  EXPECT_NEAR(aged->pose.position.x, fresh_x, 1e-9) << "ageing moves no landmark";
}

TEST_F(WorldModelFixture, TheMissionReferenceAnchorsOnTheFirstUsablePose)
{
  ASSERT_TRUE(waitForTheNodeToHaveAPose());

  EXPECT_EQ(mission_reference_.header.frame_id, "odom");
  EXPECT_NEAR(mission_reference_.pose.position.x, kBaseLinkX, 1e-9);
  EXPECT_NEAR(mission_reference_.pose.position.z, kBaseLinkZ, 1e-9);
}

TEST_F(WorldModelFixture, ObstacleRangeIsRecomputedAgainstThePoseThatPublishesIt)
{
  ASSERT_TRUE(waitForTheNodeToHaveAPose());
  ASSERT_TRUE(observeObstacleUntilMapped(kDownFrame, 0.0, 0.0, 2.5, 0.2));
  ASSERT_EQ(world_obstacles_.obstacles.size(), 1u);

  const float close_range = world_obstacles_.obstacles[0].distance;
  EXPECT_LT(close_range, 3.0F) << "the observed 99 m must not survive the transform";
  EXPECT_GT(close_range, 2.0F);

  base_link_z_ = kBaseLinkZ + 2.0;      // climb away without seeing it again
  EXPECT_TRUE(
    spinUntil(
      [this, close_range]() {
        return world_obstacles_.obstacles.size() == 1u &&
               world_obstacles_.obstacles[0].distance > close_range + 1.5F;
      },
      5s))
    << "range to the vehicle may not be frozen at its observed value";
}

TEST_F(WorldModelFixture, AMountMissingItsGeometryDropsItsObservations)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("use_sim_time", false),
    rclcpp::Parameter("publish_rate_hz", 50.0),
    rclcpp::Parameter("frames.names", std::vector<std::string>{"mystery_camera"}),
    rclcpp::Parameter("frames.mystery_camera.frame_id", "uav0/mystery_optical"),
  });
  auto strict_node = uav_world_model::createWorldModelNode(options);
  executor_->add_node(strict_node);
  executor_->remove_node(node_);

  ASSERT_TRUE(waitForTheNodeToHaveAPose());

  publishMarker("uav0/mystery_optical", 51);
  // base_link needs no mount; the control rides the same publisher.
  ASSERT_TRUE(mapMarkerFrom("base_link", 52))
    << "control: the node still maps the frame it has geometry for";

  EXPECT_EQ(findLandmark(51), nullptr) << "no geometry means no mount, not identity";

  executor_->remove_node(strict_node);
  executor_->add_node(node_);
}

TEST_F(WorldModelFixture, YawRateDoesNotGiveAStillTargetAPhantomVelocity)
{
  yaw_rate_ = 0.5;
  ASSERT_TRUE(waitForTheNodeToHaveAPose());

  // Off to one side, so the lever arm bites.
  ASSERT_TRUE(observeTrackUntilMapped(kDownFrame, 1.5, 0.0, 2.5));

  const double speed = std::sqrt(
    world_target_.velocity.x * world_target_.velocity.x +
    world_target_.velocity.y * world_target_.velocity.y +
    world_target_.velocity.z * world_target_.velocity.z);
  EXPECT_NEAR(world_target_.velocity.x, 0.75, 0.02) << "omega x r, 0.5 rad/s at 1.5 m";
  EXPECT_GT(speed, 0.5) << "a rotating camera carries the target with it";
}

TEST_F(WorldModelFixture, TheShippedConfigCarriesTheFrontCameraGeometry)
{
  auto shipped = startNodeWithShippedConfig();

  const auto names = shipped->get_parameter("frames.names").as_string_array();
  ASSERT_NE(std::find(names.begin(), names.end(), "camera_front"), names.end())
    << "unlisted means every front-camera observation is silently dropped";
  EXPECT_EQ(
    shipped->get_parameter("frames.camera_front.frame_id").as_string(), kFrontFrame)
    << "must equal what obstacle_extractor_node stamps, character for character";
  EXPECT_TRUE(shipped->get_parameter("frames.camera_front.optical").as_bool());

  // uav0_full/uav0_track mount .12 .03 .002; the sensor adds no pose.
  const auto translation =
    shipped->get_parameter("frames.camera_front.translation").as_double_array();
  ASSERT_EQ(translation.size(), 3u);
  EXPECT_NEAR(translation[0], 0.12, 1e-12);
  EXPECT_NEAR(translation[1], 0.03, 1e-12);
  EXPECT_NEAR(translation[2], 0.002, 1e-12);

  const auto rotation =
    shipped->get_parameter("frames.camera_front.rotation_rpy").as_double_array();
  ASSERT_EQ(rotation.size(), 3u);
  EXPECT_NEAR(rotation[0], 0.0, 1e-12);
  EXPECT_NEAR(rotation[1], 0.0, 1e-12) << "the front lens is not pitched down";
  EXPECT_NEAR(rotation[2], 0.0, 1e-12);

  stopShippedConfigNode(shipped);
}

TEST_F(WorldModelFixture, AParkedAircraftPutsTheFrontObstacleStraightAhead)
{
  auto shipped = startNodeWithShippedConfig();
  ASSERT_TRUE(waitForTheNodeToHaveAPose());
  ASSERT_TRUE(observeObstacleUntilMapped(kFrontFrame, 0.0, 0.0, kFrontRange, 0.2));
  ASSERT_EQ(world_obstacles_.obstacles.size(), 1u);

  const auto & centre = world_obstacles_.obstacles[0].center;
  EXPECT_NEAR(centre.x, kBaseLinkX + kFrontInBaseX, 1e-6);
  EXPECT_NEAR(centre.y, kBaseLinkY + kFrontInBaseY, 1e-6);
  EXPECT_NEAR(centre.z, kBaseLinkZ + kFrontInBaseZ, 1e-6);
  EXPECT_GT(world_obstacles_.obstacles[0].position_uncertainty, 0.0F)
    << "the world layer may never say -1 or 0";

  stopShippedConfigNode(shipped);
}

TEST_F(WorldModelFixture, YawCarriesTheFrontCameraLeverArmAround)
{
  yaw_rad_ = M_PI / 2.0;
  auto shipped = startNodeWithShippedConfig();
  ASSERT_TRUE(waitForTheNodeToHaveAPose());
  ASSERT_TRUE(observeObstacleUntilMapped(kFrontFrame, 0.0, 0.0, kFrontRange, 0.2));
  ASSERT_EQ(world_obstacles_.obstacles.size(), 1u);

  // yaw 90 turns base point (3.12, 0.03, 0.002) into (-0.03, 3.12, 0.002).
  const auto & centre = world_obstacles_.obstacles[0].center;
  EXPECT_NEAR(centre.x, kBaseLinkX - kFrontInBaseY, 1e-6);
  EXPECT_NEAR(centre.y, kBaseLinkY + kFrontInBaseX, 1e-6);
  EXPECT_NEAR(centre.z, kBaseLinkZ + kFrontInBaseZ, 1e-6);

  stopShippedConfigNode(shipped);
}

TEST_F(WorldModelFixture, TheFrontCameraTargetTrackRidesTheSameMount)
{
  auto shipped = startNodeWithShippedConfig();
  ASSERT_TRUE(waitForTheNodeToHaveAPose());

  // target_tracker_node copies the ObstacleArray frame onto the track.
  ASSERT_TRUE(observeTrackUntilMapped(kFrontFrame, 0.0, 0.0, kFrontRange));

  EXPECT_NEAR(world_target_.pose.position.x, kBaseLinkX + kFrontInBaseX, 1e-6);
  EXPECT_NEAR(world_target_.pose.position.y, kBaseLinkY + kFrontInBaseY, 1e-6);
  EXPECT_NEAR(world_target_.pose.position.z, kBaseLinkZ + kFrontInBaseZ, 1e-6);
  EXPECT_EQ(world_target_.header.frame_id, "odom");

  stopShippedConfigNode(shipped);
}

TEST_F(WorldModelFixture, TheReportedFreshnessIsTheSightingAgeNotTheMessageAge)
{
  ASSERT_TRUE(waitForTheNodeToHaveAPose());

  // 2.5 s hidden plus 0.4 s transport must read 2.9 s.
  ASSERT_TRUE(
    spinUntil(
      [this]() {
        if (target_received_) {
          return true;
        }
        publishTrack(
          kDownFrame, 5, TargetTrack::STATUS_COASTING, 2.5F, 0.0, 0.0, kMarkerRange,
          probe_->now() - rclcpp::Duration::from_seconds(0.4));
        return false;
      },
      5s));

  EXPECT_GE(world_target_.time_since_seen_sec, 2.85F)
    << "message age would report a target hidden for seconds as just seen";
  EXPECT_LT(world_target_.time_since_seen_sec, 3.6F);
}

TEST_F(WorldModelFixture, ALostTrackLaterInTheBatchDoesNotStealTheTargetState)
{
  ASSERT_TRUE(waitForTheNodeToHaveAPose());

  // One tracker frame, one message per track, same stamp.
  ASSERT_TRUE(
    spinUntil(
      [this]() {
        if (target_received_) {
          return true;
        }
        const rclcpp::Time frame_stamp = probe_->now();
        publishTrack(
          kDownFrame, 3, TargetTrack::STATUS_TRACKING, 0.0F, 0.0, 0.0, kMarkerRange, frame_stamp);
        publishTrack(
          kDownFrame, 1, TargetTrack::STATUS_LOST, 2.5F, 0.0, 0.0, 1.0, frame_stamp);
        return false;
      },
      5s));

  EXPECT_EQ(world_target_.track_id, 3) << "the last message of a batch is not the best track";
  EXPECT_NEAR(world_target_.pose.position.z, kExpectedZ, 1e-6)
    << "a discarded track would jump the aim point by 1.5 m";
}

TEST_F(WorldModelFixture, NoObstacleMapIsPublishedWithoutAnObstacleSource)
{
  ASSERT_TRUE(waitForTheNodeToHaveAPose());
  ASSERT_TRUE(mapMarker(7)) << "control: the node is alive and publishing";
  ASSERT_TRUE(spinForWorldUpdates(25)) << "the node needs its chances to publish one";

  EXPECT_FALSE(obstacles_received_) << "an empty map would read as nothing to avoid";
}
