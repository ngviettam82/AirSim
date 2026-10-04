#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "uav_perception/marker_pose.hpp"

using uav_perception::CameraIntrinsics;

TEST(Intrinsics, ReadsTheRowMajorMatrixRosCarries)
{
  // Row-major K layout; values from sim down camera.
  const std::array<double, 9> k{{432.5, 0.0, 320.0, 0.0, 432.5, 240.0, 0.0, 0.0, 1.0}};
  const CameraIntrinsics intrinsics = uav_perception::intrinsicsFrom(k);
  EXPECT_DOUBLE_EQ(intrinsics.fx, 432.5);
  EXPECT_DOUBLE_EQ(intrinsics.fy, 432.5);
  EXPECT_DOUBLE_EQ(intrinsics.cx, 320.0);
  EXPECT_DOUBLE_EQ(intrinsics.cy, 240.0);
  EXPECT_TRUE(intrinsics.valid());
}

TEST(Intrinsics, AnEmptyCameraInfoIsNotValid)
{
  // Pins: zero reads as absent CameraInfo (see README).
  const std::array<double, 9> zeros{};
  EXPECT_FALSE(uav_perception::intrinsicsFrom(zeros).valid());
}

TEST(Confidence, FallsWithReprojectionError)
{
  EXPECT_DOUBLE_EQ(uav_perception::confidenceFromReprojection(0.0, 5.0), 1.0);
  EXPECT_DOUBLE_EQ(uav_perception::confidenceFromReprojection(2.5, 5.0), 0.5);
  EXPECT_DOUBLE_EQ(uav_perception::confidenceFromReprojection(5.0, 5.0), 0.0);
}

TEST(Confidence, NeverLeavesTheUnitRange)
{
  EXPECT_DOUBLE_EQ(uav_perception::confidenceFromReprojection(50.0, 5.0), 0.0);
  EXPECT_DOUBLE_EQ(uav_perception::confidenceFromReprojection(-1.0, 5.0), 0.0);
  EXPECT_LE(uav_perception::confidenceFromReprojection(0.0, 5.0), 1.0);
}

TEST(Frames, StraightAheadInOpticalIsForwardInBody)
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  uav_perception::opticalToBody(0.0, 0.0, 2.0, x, y, z);
  EXPECT_DOUBLE_EQ(x, 2.0) << "optical +Z is along the lens, body +X is forward";
  EXPECT_DOUBLE_EQ(y, 0.0);
  EXPECT_DOUBLE_EQ(z, 0.0);
}

TEST(Frames, RightInOpticalIsNegativeYInBody)
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  uav_perception::opticalToBody(1.0, 0.0, 0.0, x, y, z);
  EXPECT_DOUBLE_EQ(y, -1.0) << "body +Y is LEFT; a marker to the right is negative";
  EXPECT_DOUBLE_EQ(x, 0.0);
  EXPECT_DOUBLE_EQ(z, 0.0);
}

TEST(Frames, DownInOpticalIsNegativeZInBody)
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  uav_perception::opticalToBody(0.0, 1.0, 0.0, x, y, z);
  EXPECT_DOUBLE_EQ(z, -1.0) << "optical +Y points down, body +Z points up";
}

TEST(Frames, ConversionPreservesDistance)
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  uav_perception::opticalToBody(0.3, -0.4, 2.0, x, y, z);
  EXPECT_NEAR(
    std::sqrt(x * x + y * y + z * z),
    std::sqrt(0.3 * 0.3 + 0.4 * 0.4 + 2.0 * 2.0), 1e-12) <<
    "a rotation cannot change how far away the marker is";
}

TEST(Frames, BodyToOpticalInvertsOpticalToBody)
{
  // New 2026-08-23 (ego_motion odom->optical back-transform, bug #10
  // follow-up): bodyToOptical must be the exact inverse of opticalToBody.
  const double ox = 0.3, oy = -1.4, oz = 2.7;
  double bx = 0.0, by = 0.0, bz = 0.0;
  uav_perception::opticalToBody(ox, oy, oz, bx, by, bz);
  double rx = 0.0, ry = 0.0, rz = 0.0;
  uav_perception::bodyToOptical(bx, by, bz, rx, ry, rz);
  EXPECT_NEAR(rx, ox, 1e-12);
  EXPECT_NEAR(ry, oy, 1e-12);
  EXPECT_NEAR(rz, oz, 1e-12);
}

TEST(Frames, ForwardInBodyIsStraightAheadInOptical)
{
  double ox = 0.0, oy = 0.0, oz = 0.0;
  uav_perception::bodyToOptical(2.0, 0.0, 0.0, ox, oy, oz);
  EXPECT_DOUBLE_EQ(oz, 2.0) << "body +X forward is optical +Z along the lens";
  EXPECT_DOUBLE_EQ(ox, 0.0);
  EXPECT_DOUBLE_EQ(oy, 0.0);
}

TEST(MarkerCorners, MatchOpenCvOrderAndSize)
{
  const auto corners = uav_perception::markerCorners(0.5);
  // OpenCV returns top-left, top-right, bottom-right, bottom-left.
  EXPECT_DOUBLE_EQ(corners[0][0], -0.25);
  EXPECT_DOUBLE_EQ(corners[0][1], 0.25);
  EXPECT_DOUBLE_EQ(corners[1][0], 0.25);
  EXPECT_DOUBLE_EQ(corners[2][1], -0.25);
  EXPECT_DOUBLE_EQ(corners[3][0], -0.25);
  for (const auto & corner : corners) {
    EXPECT_DOUBLE_EQ(corner[2], 0.0) << "the marker lies in its own plane";
  }
}

TEST(Quaternion, IdentityForZeroRotation)
{
  const auto q = uav_perception::quaternionFromRotationVector(0.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(q[3], 1.0);
  EXPECT_DOUBLE_EQ(q[0], 0.0);
  EXPECT_DOUBLE_EQ(q[1], 0.0);
  EXPECT_DOUBLE_EQ(q[2], 0.0);
}

TEST(Quaternion, HalfTurnAboutZ)
{
  const auto q = uav_perception::quaternionFromRotationVector(0.0, 0.0, M_PI);
  EXPECT_NEAR(q[2], 1.0, 1e-12);
  EXPECT_NEAR(q[3], 0.0, 1e-12);
}

TEST(Quaternion, IsNormalised)
{
  const auto q = uav_perception::quaternionFromRotationVector(0.3, -1.2, 0.7);
  const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  EXPECT_NEAR(norm, 1.0, 1e-12);
}
