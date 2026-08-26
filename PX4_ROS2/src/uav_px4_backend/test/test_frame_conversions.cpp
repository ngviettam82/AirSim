// Guards against calling a conversion in the wrong direction.

#include <cmath>

#include <gtest/gtest.h>
#include <px4_ros_com/frame_transforms.h>

#include "uav_px4_backend/frame_conversions.hpp"

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Quaternion;
using geometry_msgs::msg::Vector3;
using uav_px4_backend::frames::enuToNedArray;
using uav_px4_backend::frames::frdToFluVector;
using uav_px4_backend::frames::nedToEnuPoint;
using uav_px4_backend::frames::px4ToRosQuaternion;
using uav_px4_backend::frames::rosToPx4Quaternion;
using uav_px4_backend::frames::yawEnuToNed;
using uav_px4_backend::frames::yawNedToEnu;

constexpr double kTol = 1e-6;

Point makePoint(double x, double y, double z)
{
  Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

// East in ENU is +X; east in NED is +Y.
TEST(FrameConversions, EastStaysEast)
{
  const auto ned = enuToNedArray(makePoint(1.0, 0.0, 0.0));
  EXPECT_NEAR(ned[0], 0.0, kTol);
  EXPECT_NEAR(ned[1], 1.0, kTol);
  EXPECT_NEAR(ned[2], 0.0, kTol);
}

// North in ENU is +Y; north in NED is +X.
TEST(FrameConversions, NorthStaysNorth)
{
  const auto ned = enuToNedArray(makePoint(0.0, 1.0, 0.0));
  EXPECT_NEAR(ned[0], 1.0, kTol);
  EXPECT_NEAR(ned[1], 0.0, kTol);
  EXPECT_NEAR(ned[2], 0.0, kTol);
}

// A sign error here crashes the drone.
TEST(FrameConversions, UpBecomesNegativeDown)
{
  const auto ned = enuToNedArray(makePoint(0.0, 0.0, 5.0));
  EXPECT_NEAR(ned[2], -5.0, kTol);
}

TEST(FrameConversions, AltitudeFromPx4IsPositiveUp)
{
  const auto enu = nedToEnuPoint({0.0F, 0.0F, -5.0F});
  EXPECT_NEAR(enu.z, 5.0, kTol);
}

TEST(FrameConversions, PositionRoundTrip)
{
  const auto original = makePoint(3.5, -7.25, 12.0);
  const auto back = nedToEnuPoint(enuToNedArray(original));
  EXPECT_NEAR(back.x, original.x, 1e-4);
  EXPECT_NEAR(back.y, original.y, 1e-4);
  EXPECT_NEAR(back.z, original.z, 1e-4);
}

// FRD +Y is right; FLU +Y is left.
TEST(FrameConversions, BodyRightBecomesNegativeLeft)
{
  const auto flu = frdToFluVector({0.0F, 1.0F, 0.0F});
  EXPECT_NEAR(flu.x, 0.0, kTol);
  EXPECT_NEAR(flu.y, -1.0, kTol);
  EXPECT_NEAR(flu.z, 0.0, kTol);
}

// ENU yaw 0 points east, which is NED yaw pi/2.
TEST(FrameConversions, YawZeroEnuIsEastInNed)
{
  EXPECT_NEAR(yawEnuToNed(0.0), M_PI_2, kTol);
}

// ENU yaw pi/2 points north, which is NED yaw 0.
TEST(FrameConversions, YawNorthIsZeroInNed)
{
  EXPECT_NEAR(yawEnuToNed(M_PI_2), 0.0, kTol);
}

TEST(FrameConversions, YawRoundTrip)
{
  const double original = 0.75;
  EXPECT_NEAR(yawNedToEnu(yawEnuToNed(original)), original, kTol);
}

// ROS identity points east, so PX4 yaw is pi/2.
TEST(FrameConversions, IdentityRosAttitudeIsEastInPx4)
{
  Quaternion identity;
  identity.w = 1.0;
  identity.x = 0.0;
  identity.y = 0.0;
  identity.z = 0.0;

  const auto q_px4 = rosToPx4Quaternion(identity);
  const auto eigen_q =
    px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(q_px4);
  const double yaw_ned =
    px4_ros_com::frame_transforms::utils::quaternion::quaternion_get_yaw(eigen_q);

  EXPECT_NEAR(yaw_ned, M_PI_2, 1e-5);
}

TEST(FrameConversions, AttitudeRoundTrip)
{
  Quaternion original;
  original.w = 0.9238795;
  original.x = 0.0;
  original.y = 0.0;
  original.z = 0.3826834;

  const auto back = px4ToRosQuaternion(rosToPx4Quaternion(original));

  // Quaternions q and -q describe the same rotation.
  const double dot = back.w * original.w + back.x * original.x +
    back.y * original.y + back.z * original.z;
  EXPECT_NEAR(std::abs(dot), 1.0, 1e-5);
}

// Facing east, an east vector must read as straight ahead.
TEST(FrameConversions, WorldVectorRotatesIntoBodyAxes)
{
  Quaternion identity;
  identity.w = 1.0;

  Vector3 east;
  east.x = 2.0;

  const auto body = uav_px4_backend::frames::rotateEnuToBodyFlu(east, identity);
  EXPECT_NEAR(body.x, 2.0, kTol);
  EXPECT_NEAR(body.y, 0.0, kTol);
  EXPECT_NEAR(body.z, 0.0, kTol);
}

// Facing north, the east vector sits to starboard.
TEST(FrameConversions, RotatedBodySeesEastOnItsRight)
{
  Quaternion yawed;
  yawed.w = std::cos(M_PI_4);
  yawed.z = std::sin(M_PI_4);

  Vector3 east;
  east.x = 1.0;

  const auto body = uav_px4_backend::frames::rotateEnuToBodyFlu(east, yawed);
  EXPECT_NEAR(body.x, 0.0, 1e-5);
  EXPECT_NEAR(body.y, -1.0, 1e-5);   // FLU: right is negative Y
}

// ===========================================================================
// The VELOCITY and RATE conversions. S3 coverage, 2026-08-26: these four were
// reachable only by flying -- every case above exercises the position path.
// A sign error on a velocity is as fatal as one on a position, and unlike the
// position path nothing here was ever checked against a known answer.
//   nedToEnuVector   px4_state_adapter_node.cpp:158  (twist out of PX4)
//   enuToNedArray    px4_command_gateway_node.cpp:206/211  (setpoint into PX4)
//   fluToFrdArray    px4_external_odometry_node.cpp:121   (body twist into EKF2)
//   yawRateEnuToNed  px4_command_gateway_node.cpp:208
// ===========================================================================

Vector3 makeVector(double x, double y, double z)
{
  Vector3 v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

// PX4 reports north as +X; ROS wants north on +Y.
TEST(FrameConversions, NorthVelocityFromPx4PointsNorthInEnu)
{
  const auto enu = uav_px4_backend::frames::nedToEnuVector({1.0F, 0.0F, 0.0F});
  EXPECT_NEAR(enu.x, 0.0, kTol);
  EXPECT_NEAR(enu.y, 1.0, kTol);
  EXPECT_NEAR(enu.z, 0.0, kTol);
}

// Descending in NED is +Z; the same motion is -Z going up in ENU.
TEST(FrameConversions, DescentRateFromPx4IsNegativeUpInEnu)
{
  const auto enu = uav_px4_backend::frames::nedToEnuVector({0.0F, 0.0F, 2.0F});
  EXPECT_NEAR(enu.z, -2.0, kTol);
}

TEST(FrameConversions, EastVelocityBecomesNorthAxisInNed)
{
  const auto ned = enuToNedArray(makeVector(1.0, 0.0, 0.0));
  EXPECT_NEAR(ned[0], 0.0, kTol);
  EXPECT_NEAR(ned[1], 1.0, kTol);
  EXPECT_NEAR(ned[2], 0.0, kTol);
}

// Commanding a climb must never reach PX4 as a descent.
TEST(FrameConversions, ClimbVelocityBecomesNegativeDown)
{
  const auto ned = enuToNedArray(makeVector(0.0, 0.0, 3.0));
  EXPECT_NEAR(ned[2], -3.0, kTol);
}

TEST(FrameConversions, VelocityRoundTripThroughNed)
{
  const auto ned = enuToNedArray(makeVector(1.5, -2.5, 0.75));
  const auto back = uav_px4_backend::frames::nedToEnuVector(ned);
  EXPECT_NEAR(back.x, 1.5, kTol);
  EXPECT_NEAR(back.y, -2.5, kTol);
  EXPECT_NEAR(back.z, 0.75, kTol);
}

// FLU forward is FRD forward: the axis that must NOT flip.
TEST(FrameConversions, BodyForwardVelocitySurvivesFluToFrd)
{
  const auto frd = uav_px4_backend::frames::fluToFrdArray(makeVector(2.0, 0.0, 0.0));
  EXPECT_NEAR(frd[0], 2.0, kTol);
  EXPECT_NEAR(frd[1], 0.0, kTol);
  EXPECT_NEAR(frd[2], 0.0, kTol);
}

// Left in FLU is right in FRD, and up is down: both flip.
TEST(FrameConversions, BodyLeftAndUpVelocitiesFlipInFrd)
{
  const auto left = uav_px4_backend::frames::fluToFrdArray(makeVector(0.0, 1.0, 0.0));
  EXPECT_NEAR(left[1], -1.0, kTol);
  const auto up = uav_px4_backend::frames::fluToFrdArray(makeVector(0.0, 0.0, 1.0));
  EXPECT_NEAR(up[2], -1.0, kTol);
}

// Yaw runs counter-clockwise in ENU and clockwise in NED, so the rate flips.
TEST(FrameConversions, YawRateFlipsSignIntoNed)
{
  EXPECT_NEAR(uav_px4_backend::frames::yawRateEnuToNed(0.5), -0.5, kTol);
  EXPECT_NEAR(uav_px4_backend::frames::yawRateEnuToNed(-0.25), 0.25, kTol);
  EXPECT_NEAR(uav_px4_backend::frames::yawRateEnuToNed(0.0), 0.0, kTol);
}

// A rate conversion that agrees with the ANGLE conversion it must stay consistent
// with: yaw advanced by w*dt in ENU has to land where the NED yaw rate says.
TEST(FrameConversions, YawRateAgreesWithTheAngleConversionItPairsWith)
{
  const double dt = 0.1;
  const double rate_enu = 0.4;
  const double yaw_enu_before = 0.2;
  const double yaw_ned_before = uav_px4_backend::frames::yawEnuToNed(yaw_enu_before);
  const double yaw_ned_after = uav_px4_backend::frames::yawEnuToNed(yaw_enu_before + rate_enu * dt);
  const double stepped = yaw_ned_before + uav_px4_backend::frames::yawRateEnuToNed(rate_enu) * dt;
  EXPECT_NEAR(std::sin(stepped), std::sin(yaw_ned_after), 1e-9);
  EXPECT_NEAR(std::cos(stepped), std::cos(yaw_ned_after), 1e-9);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
