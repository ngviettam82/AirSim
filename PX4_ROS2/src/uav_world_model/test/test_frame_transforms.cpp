#include <cmath>

#include <gtest/gtest.h>

#include "uav_perception/marker_pose.hpp"
#include "uav_world_model/frame_transforms.hpp"

using uav_world_model::Quat;
using uav_world_model::Rigid;
using uav_world_model::SensorMount;
using uav_world_model::Vec3;

namespace
{

// uav0_nav/uav0_full: module at 0.06 0 -0.05, sensor at 0 0 -0.015 pitch +90.
SensorMount downCameraMount()
{
  SensorMount mount;
  mount.frame_id = "uav0/camera_down_optical";
  mount.reports_optical_axes = true;
  mount.base_from_sensor.translation = Vec3{0.06, 0.0, -0.065};
  mount.base_from_sensor.rotation = uav_world_model::fromRollPitchYaw(0.0, M_PI / 2.0, 0.0);
  return mount;
}

// uav0_full/uav0_track: module at .12 .03 .002, sensor adds no pose.
SensorMount frontCameraMount()
{
  SensorMount mount;
  mount.frame_id = "uav0/camera_front_optical";
  mount.reports_optical_axes = true;
  mount.base_from_sensor.translation = Vec3{0.12, 0.03, 0.002};
  mount.base_from_sensor.rotation = uav_world_model::fromRollPitchYaw(0.0, 0.0, 0.0);
  return mount;
}

Rigid levelPose(double x, double y, double z)
{
  Rigid pose;
  pose.translation = Vec3{x, y, z};
  return pose;
}

}  // namespace

TEST(OpticalToBody, MatchesTheUavPerceptionVectorsExactly)
{
  // Same vectors as uav_perception/test/test_marker_pose.cpp.
  const Vec3 inputs[] = {
    Vec3{0.0, 0.0, 2.0}, Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, Vec3{0.3, -0.4, 2.0}};

  for (const auto & input : inputs) {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    uav_perception::opticalToBody(input.x, input.y, input.z, x, y, z);

    const Vec3 mine = uav_world_model::opticalToBody(input);
    EXPECT_DOUBLE_EQ(mine.x, x);
    EXPECT_DOUBLE_EQ(mine.y, y);
    EXPECT_DOUBLE_EQ(mine.z, z);
  }
}

TEST(OpticalToBody, QuaternionAgreesWithThePerAxisFunction)
{
  const Vec3 inputs[] = {
    Vec3{0.0, 0.0, 2.0}, Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, Vec3{0.3, -0.4, 2.0}};

  for (const auto & input : inputs) {
    const Vec3 by_axes = uav_world_model::opticalToBody(input);
    const Vec3 by_quaternion = uav_world_model::rotate(uav_world_model::kOpticalToBody, input);
    EXPECT_NEAR(by_quaternion.x, by_axes.x, 1e-12) << "position and orientation must agree";
    EXPECT_NEAR(by_quaternion.y, by_axes.y, 1e-12);
    EXPECT_NEAR(by_quaternion.z, by_axes.z, 1e-12);
  }
}

TEST(DownCamera, MarkerOnTheOpticalAxisLandsUnderTheAircraft)
{
  const Vec3 in_optical{0.0, 0.0, 2.5};
  const Vec3 in_odom = uav_world_model::sensorPointToOdom(
    in_optical, downCameraMount(), levelPose(10.0, 5.0, 2.565));

  EXPECT_NEAR(in_odom.x, 10.06, 1e-9) << "the lens sits 0.06 m ahead of base_link";
  EXPECT_NEAR(in_odom.y, 5.0, 1e-9);
  EXPECT_NEAR(in_odom.z, 0.0, 1e-9) << "2.5 m below a lens 0.065 m below base_link";
}

TEST(DownCamera, ImageRightIsTheAircraftRight)
{
  const Vec3 in_optical{0.5, 0.0, 2.0};
  const Vec3 in_odom = uav_world_model::sensorPointToOdom(
    in_optical, downCameraMount(), levelPose(0.0, 0.0, 2.065));

  EXPECT_NEAR(in_odom.x, 0.06, 1e-9);
  EXPECT_NEAR(in_odom.y, -0.5, 1e-9) << "body +Y is left, so image right is negative";
  EXPECT_NEAR(in_odom.z, 0.0, 1e-9);
}

TEST(DownCamera, YawRotatesTheWholeLeverArm)
{
  Rigid pose = levelPose(0.0, 0.0, 2.565);
  pose.rotation = uav_world_model::fromRollPitchYaw(0.0, 0.0, M_PI / 2.0);

  const Vec3 in_odom =
    uav_world_model::sensorPointToOdom(Vec3{0.0, 0.0, 2.5}, downCameraMount(), pose);

  EXPECT_NEAR(in_odom.x, 0.0, 1e-9) << "yaw 90 deg turns the 0.06 m offset onto +Y";
  EXPECT_NEAR(in_odom.y, 0.06, 1e-9);
  EXPECT_NEAR(in_odom.z, 0.0, 1e-9);
}

TEST(DownCamera, DistanceSurvivesTheWholeChain)
{
  const Vec3 in_optical{0.3, -0.4, 2.0};
  const SensorMount mount = downCameraMount();
  const Vec3 in_odom = uav_world_model::sensorPointToOdom(in_optical, mount, levelPose(0, 0, 0));

  const Vec3 lens = uav_world_model::apply(levelPose(0, 0, 0), mount.base_from_sensor.translation);
  const Vec3 offset{in_odom.x - lens.x, in_odom.y - lens.y, in_odom.z - lens.z};
  EXPECT_NEAR(uav_world_model::norm(offset), uav_world_model::norm(in_optical), 1e-12);
}

TEST(FrontCamera, PointOnTheOpticalAxisLandsAheadOfTheAircraft)
{
  // optical (0,0,3) -> body (3,0,0), plus mount (0.12, 0.03, 0.002).
  const Vec3 in_odom = uav_world_model::sensorPointToOdom(
    Vec3{0.0, 0.0, 3.0}, frontCameraMount(), levelPose(0.0, 0.0, 0.0));

  EXPECT_NEAR(in_odom.x, 3.12, 1e-9) << "3 m ahead of a lens 0.12 m ahead of base_link";
  EXPECT_NEAR(in_odom.y, 0.03, 1e-9) << "the lens sits 0.03 m to the left";
  EXPECT_NEAR(in_odom.z, 0.002, 1e-9);
}

TEST(FrontCamera, ImageRightAndImageUpFollowTheAircraftAxes)
{
  // optical (0.5,-0.4,3) -> body (3,-0.5,0.4), plus the mount.
  const Vec3 in_odom = uav_world_model::sensorPointToOdom(
    Vec3{0.5, -0.4, 3.0}, frontCameraMount(), levelPose(0.0, 0.0, 0.0));

  EXPECT_NEAR(in_odom.x, 3.12, 1e-9);
  EXPECT_NEAR(in_odom.y, -0.47, 1e-9) << "body +Y is left, so image right is negative";
  EXPECT_NEAR(in_odom.z, 0.402, 1e-9) << "optical -Y is up, so it climbs";
}

TEST(FrontCamera, YawRotatesTheWholeLeverArm)
{
  Rigid pose = levelPose(2.0, 1.0, 5.0);
  pose.rotation = uav_world_model::fromRollPitchYaw(0.0, 0.0, M_PI / 2.0);

  // yaw 90 turns base point (3.12, 0.03, 0.002) into (-0.03, 3.12, 0.002).
  const Vec3 in_odom =
    uav_world_model::sensorPointToOdom(Vec3{0.0, 0.0, 3.0}, frontCameraMount(), pose);

  EXPECT_NEAR(in_odom.x, 1.97, 1e-9);
  EXPECT_NEAR(in_odom.y, 4.12, 1e-9) << "the aircraft now looks along +Y";
  EXPECT_NEAR(in_odom.z, 5.002, 1e-9);
}

TEST(FrontCamera, TheDownMountWouldPutTheSamePointSomewhereElse)
{
  const Vec3 on_axis{0.0, 0.0, 3.0};
  const Rigid parked = levelPose(0.0, 0.0, 0.0);

  const Vec3 by_front = uav_world_model::sensorPointToOdom(on_axis, frontCameraMount(), parked);
  const Vec3 by_down = uav_world_model::sensorPointToOdom(on_axis, downCameraMount(), parked);

  EXPECT_NEAR(by_down.x, 0.06, 1e-9);
  EXPECT_NEAR(by_down.z, -3.065, 1e-9) << "3 m below a lens 0.065 m under base_link";
  EXPECT_GT(uav_world_model::norm(uav_world_model::subtract(by_front, by_down)), 4.0)
    << "confusing the two mounts must be loud, not subtle";
}

TEST(FrontCamera, VelocityKeepsItsDirectionAndDropsTheMountOffset)
{
  Rigid pose = levelPose(2.0, 1.0, 5.0);
  pose.rotation = uav_world_model::fromRollPitchYaw(0.0, 0.0, M_PI / 2.0);

  // optical (0,0,1.2) -> body (1.2,0,0) -> yaw 90 -> odom (0,1.2,0).
  const Vec3 in_odom =
    uav_world_model::sensorVectorToOdom(Vec3{0.0, 0.0, 1.2}, frontCameraMount(), pose);

  EXPECT_NEAR(in_odom.x, 0.0, 1e-9);
  EXPECT_NEAR(in_odom.y, 1.2, 1e-9);
  EXPECT_NEAR(in_odom.z, 0.0, 1e-9);
}

TEST(Velocity, MountOffsetDoesNotLeakIntoAVector)
{
  const Vec3 zero_velocity{0.0, 0.0, 0.0};
  const Vec3 in_odom = uav_world_model::sensorVectorToOdom(
    zero_velocity, downCameraMount(), levelPose(10.0, 5.0, 2.565));

  EXPECT_NEAR(uav_world_model::norm(in_odom), 0.0, 1e-12) << "a velocity has no origin";
}

TEST(Extent, RotatedBoxIsEnclosedNotShrunk)
{
  const Vec3 extent{1.0, 2.0, 0.5};
  const Quat yaw45 = uav_world_model::fromRollPitchYaw(0.0, 0.0, M_PI / 4.0);
  const Vec3 enveloped = uav_world_model::axisAlignedExtent(extent, yaw45);

  const double expected = (1.0 + 2.0) / std::sqrt(2.0);
  EXPECT_NEAR(enveloped.x, expected, 1e-12);
  EXPECT_NEAR(enveloped.y, expected, 1e-12);
  EXPECT_NEAR(enveloped.z, 0.5, 1e-12);
  EXPECT_GE(
    enveloped.x * enveloped.y * enveloped.z,
    extent.x * extent.y * extent.z) << "the envelope may not lose volume";
}

TEST(Extent, IdentityRotationLeavesTheBoxAlone)
{
  const Vec3 extent{1.0, 2.0, 0.5};
  const Vec3 enveloped = uav_world_model::axisAlignedExtent(extent, Quat{});
  EXPECT_NEAR(enveloped.x, 1.0, 1e-12);
  EXPECT_NEAR(enveloped.y, 2.0, 1e-12);
  EXPECT_NEAR(enveloped.z, 0.5, 1e-12);
}

TEST(Rotation, RollPitchYawFollowsTheSdfConvention)
{
  const Quat pitch90 = uav_world_model::fromRollPitchYaw(0.0, M_PI / 2.0, 0.0);
  const Vec3 forward = uav_world_model::rotate(pitch90, Vec3{1.0, 0.0, 0.0});

  EXPECT_NEAR(forward.x, 0.0, 1e-12);
  EXPECT_NEAR(forward.y, 0.0, 1e-12);
  EXPECT_NEAR(forward.z, -1.0, 1e-12) << "pitch +90 deg turns +X into -Z, as the SDF says";
}
