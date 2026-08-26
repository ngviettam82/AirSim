#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "uav_perception/marker_pose.hpp"
#include "uav_perception/obstacle_extraction.hpp"

using uav_perception::AttitudeQuaternion;
using uav_perception::backprojectPixel;
using uav_perception::depthIntrinsicsFrom;
using uav_perception::DepthCameraIntrinsics;
using uav_perception::ExtractedObstacle;
using uav_perception::extractObstacles;
using uav_perception::ExtractionParams;
using uav_perception::isValidDepth;
using uav_perception::resolveGroundFilterAttitude;

namespace
{

// fx=fy=100, principal point at image centre; easy hand-computed geometry.
DepthCameraIntrinsics testIntrinsics(double cx = 50.0, double cy = 40.0)
{
  DepthCameraIntrinsics intrinsics;
  intrinsics.fx = 100.0;
  intrinsics.fy = 100.0;
  intrinsics.cx = cx;
  intrinsics.cy = cy;
  return intrinsics;
}

std::vector<float> emptyGrid(std::size_t width, std::size_t height)
{
  return std::vector<float>(width * height, std::numeric_limits<float>::infinity());
}

// [row0,row1) x [col0,col1), half-open, filled with depth_value.
void paintBlock(
  std::vector<float> & grid, std::size_t width,
  std::size_t row0, std::size_t row1, std::size_t col0, std::size_t col1,
  float depth_value)
{
  for (std::size_t r = row0; r < row1; ++r) {
    for (std::size_t c = col0; c < col1; ++c) {
      grid[r * width + c] = depth_value;
    }
  }
}

}  // namespace

TEST(Intrinsics, ReadsTheRowMajorMatrixRosCarries)
{
  const std::array<double, 9> k{{432.5, 0.0, 320.0, 0.0, 432.5, 240.0, 0.0, 0.0, 1.0}};
  const DepthCameraIntrinsics intrinsics = depthIntrinsicsFrom(k);
  EXPECT_DOUBLE_EQ(intrinsics.fx, 432.5);
  EXPECT_DOUBLE_EQ(intrinsics.fy, 432.5);
  EXPECT_DOUBLE_EQ(intrinsics.cx, 320.0);
  EXPECT_DOUBLE_EQ(intrinsics.cy, 240.0);
  EXPECT_TRUE(intrinsics.valid());
}

TEST(Intrinsics, AnEmptyCameraInfoIsNotValid)
{
  const std::array<double, 9> zeros{};
  EXPECT_FALSE(depthIntrinsicsFrom(zeros).valid());
}

TEST(Backprojection, MatchesPinholeModelAtPrincipalPoint)
{
  const auto intrinsics = testIntrinsics();
  double x = 0.0, y = 0.0, z = 0.0;
  backprojectPixel(50.0, 40.0, 4.0, intrinsics, x, y, z);
  EXPECT_DOUBLE_EQ(x, 0.0);
  EXPECT_DOUBLE_EQ(y, 0.0);
  EXPECT_DOUBLE_EQ(z, 4.0);
}

TEST(Backprojection, OffCenterPixelScalesWithDepth)
{
  const auto intrinsics = testIntrinsics();
  double x = 0.0, y = 0.0, z = 0.0;
  backprojectPixel(60.0, 40.0, 5.0, intrinsics, x, y, z);
  EXPECT_DOUBLE_EQ(x, 0.5) << "10 px right of centre at 5 m, fx=100";
  EXPECT_DOUBLE_EQ(y, 0.0);
  EXPECT_DOUBLE_EQ(z, 5.0);
}

TEST(ValidDepth, RejectsNonFiniteAndOutOfRange)
{
  ExtractionParams params;
  params.min_range_m = 0.2;
  params.max_range_m = 10.0;
  EXPECT_FALSE(isValidDepth(std::numeric_limits<float>::quiet_NaN(), params));
  EXPECT_FALSE(isValidDepth(std::numeric_limits<float>::infinity(), params));
  EXPECT_FALSE(isValidDepth(0.0f, params)) << "Gazebo no-return must not read as a wall at 0 m";
  EXPECT_FALSE(isValidDepth(-1.0f, params));
  EXPECT_FALSE(isValidDepth(0.1f, params)) << "below min_range_m";
  EXPECT_FALSE(isValidDepth(10.0f, params)) << "max_range_m is exclusive";
  EXPECT_TRUE(isValidDepth(5.0f, params));
}

TEST(ExtractObstacles, SingleBoxMatchesHandComputedGeometry)
{
  // 21x21 px block at 5 m, symmetric around the principal point (50,40).
  auto grid = emptyGrid(100, 80);
  paintBlock(grid, 100, 30, 51, 40, 61, 5.0f);

  ExtractionParams params;
  params.pixel_stride = 1;
  const auto obstacles = extractObstacles(grid.data(), 100, 80, testIntrinsics(), params);

  ASSERT_EQ(obstacles.size(), 1u);
  const auto & obstacle = obstacles[0];
  EXPECT_NEAR(obstacle.center_x, 0.0, 1e-6);
  EXPECT_NEAR(obstacle.center_y, 0.0, 1e-6);
  EXPECT_NEAR(obstacle.center_z, 5.0, 1e-6);
  EXPECT_NEAR(obstacle.size_x, 1.0, 1e-6) << "20 px at 5 m, fx=100 -> 1.0 m";
  EXPECT_NEAR(obstacle.size_y, 1.0, 1e-6);
  EXPECT_NEAR(obstacle.size_z, 0.0, 1e-6) << "single face: no observed thickness";
  EXPECT_NEAR(obstacle.distance, 5.0, 1e-6) << "principal-point pixel is in the block";
  EXPECT_DOUBLE_EQ(obstacle.confidence, 1.0) << "441 points saturates the default 150";
  EXPECT_LT(obstacle.point_count, static_cast<std::size_t>(100 * 80)) <<
    "sanity: did not label the whole frame";
}

TEST(ExtractObstacles, TwoSeparatedObjectsStayDistinct)
{
  // Same depth value, same row band, opposite ends of the frame: adjacency
  // (not depth similarity) must be what keeps them apart.
  auto grid = emptyGrid(100, 80);
  paintBlock(grid, 100, 10, 20, 10, 20, 3.0f);
  paintBlock(grid, 100, 10, 20, 70, 80, 3.0f);

  ExtractionParams params;
  params.pixel_stride = 1;
  params.min_cluster_points = 10;
  const auto obstacles = extractObstacles(grid.data(), 100, 80, testIntrinsics(), params);

  ASSERT_EQ(obstacles.size(), 2u);
  const bool first_is_left = obstacles[0].center_x < obstacles[1].center_x;
  const auto & left = first_is_left ? obstacles[0] : obstacles[1];
  const auto & right = first_is_left ? obstacles[1] : obstacles[0];
  EXPECT_LT(left.center_x, 0.0);
  EXPECT_GT(right.center_x, 0.0);
}

TEST(ExtractObstacles, IgnoresClusterSmallerThanMinimum)
{
  auto grid = emptyGrid(100, 80);
  paintBlock(grid, 100, 10, 13, 10, 13, 3.0f);  // 9 points

  ExtractionParams params;
  params.pixel_stride = 1;
  params.min_cluster_points = 30;
  const auto obstacles = extractObstacles(grid.data(), 100, 80, testIntrinsics(), params);
  EXPECT_TRUE(obstacles.empty()) << "9 points is noise, not an obstacle, at this threshold";
}

TEST(ExtractObstacles, EmptyFrameProducesNoObstacles)
{
  const auto grid = emptyGrid(100, 80);
  const auto obstacles = extractObstacles(
    grid.data(), 100, 80, testIntrinsics(), ExtractionParams{});
  EXPECT_TRUE(obstacles.empty());
}

TEST(ExtractObstacles, InvalidIntrinsicsProducesNoObstacles)
{
  auto grid = emptyGrid(100, 80);
  paintBlock(grid, 100, 30, 51, 40, 61, 5.0f);
  DepthCameraIntrinsics invalid;  // fx=fy=0, never-arrived CameraInfo
  const auto obstacles = extractObstacles(
    grid.data(), 100, 80, invalid, ExtractionParams{});
  EXPECT_TRUE(obstacles.empty());
}

TEST(ExtractObstacles, EachCallIsIndependentOfThePrevious)
{
  // No frame-to-frame state; gaps up to ~1.8 s must not corrupt the next call.
  ExtractionParams params;
  params.pixel_stride = 1;

  auto first_grid = emptyGrid(100, 80);
  paintBlock(first_grid, 100, 30, 51, 40, 61, 5.0f);
  const auto first = extractObstacles(first_grid.data(), 100, 80, testIntrinsics(), params);
  ASSERT_EQ(first.size(), 1u);
  EXPECT_NEAR(first[0].center_z, 5.0, 1e-6);

  auto second_grid = emptyGrid(100, 80);
  paintBlock(second_grid, 100, 5, 16, 5, 16, 8.0f);
  const auto second = extractObstacles(second_grid.data(), 100, 80, testIntrinsics(), params);
  ASSERT_EQ(second.size(), 1u);
  EXPECT_NEAR(second[0].center_z, 8.0, 1e-6) << "must reflect only the second frame";
}

TEST(ExtractObstacles, BoundsChainingAcrossAGradualRamp)
{
  // Ground-plane-at-grazing-angle stand-in: each step (0.05 m) is well inside
  // cluster_depth_tolerance_m, but the 2.45 m total span must not chain
  // into one cluster (see README ground-plane pitfall, found via P5.4 sim gate).
  const std::size_t width = 50;
  const std::size_t height = 10;
  std::vector<float> grid(width * height);
  for (std::size_t r = 0; r < height; ++r) {
    for (std::size_t c = 0; c < width; ++c) {
      grid[r * width + c] = 1.0f + static_cast<float>(c) * 0.05f;  // 1.00 .. 3.45 m
    }
  }

  ExtractionParams params;
  params.pixel_stride = 1;
  params.cluster_depth_tolerance_m = 0.3;
  params.max_cluster_depth_span_m = 2.0;
  params.min_cluster_points = 1;
  const auto obstacles = extractObstacles(
    grid.data(), width, height, testIntrinsics(), params);

  ASSERT_EQ(obstacles.size(), 2u) <<
    "a smooth 2.45 m ramp must not chain into a single cluster";
  // Column 0..40 (41 cols) stays within a 2.0 m span; 41..49 (9 cols) is the rest.
  EXPECT_EQ(obstacles[0].point_count, 41u * height);
  EXPECT_EQ(obstacles[1].point_count, 9u * height);
}

namespace
{

// Level camera (zero-tilt mount, see README) + flat ground at height H below
// the camera: the metric optical-down value of every ground point equals H,
// no matter the row/column -- see the derivation next to
// findDominantGroundLevel() in obstacle_extraction.cpp. z = H*fy/(v-cy).
void paintLevelGround(
  std::vector<float> & grid, std::size_t width, std::size_t height,
  double cx, double cy, double fx, double fy, double height_m,
  double min_range_m, double max_range_m)
{
  (void)cx;
  (void)fx;
  for (std::size_t r = 0; r < height; ++r) {
    const double denom = static_cast<double>(r) - cy;
    if (denom <= 0.0) {
      continue;  // at/above the horizon row: ground would be at infinite range
    }
    const double z = height_m * fy / denom;
    if (z <= min_range_m || z >= max_range_m) {
      continue;  // out of the sensor's valid range, leave as sky (infinity)
    }
    for (std::size_t c = 0; c < width; ++c) {
      grid[r * width + c] = static_cast<float>(z);
    }
  }
}

}  // namespace

TEST(ExtractObstacles, GroundFilterRemovesGrazingHorizonFragmentation)
{
  // Reproduces the P9 G-M3 gate bug: a flat ground seen at grazing angle by a
  // low-flying, level, forward-looking camera. Row-to-row depth jumps blow
  // past cluster_depth_tolerance_m near the horizon, so pre-fix this shatters
  // into a dozen-plus thin strips (README "phan manh nen"). No real object is
  // present, so a working filter must drive the obstacle count to ~0.
  const std::size_t width = 100;
  const std::size_t height = 80;
  auto grid = emptyGrid(width, height);
  const auto intrinsics = testIntrinsics();
  paintLevelGround(
    grid, width, height, intrinsics.cx, intrinsics.cy, intrinsics.fx, intrinsics.fy,
    /*height_m=*/0.5, /*min_range_m=*/0.2, /*max_range_m=*/19.1);

  ExtractionParams params;
  params.pixel_stride = 1;
  params.min_cluster_points = 5;
  params.max_obstacles = 50;  // do not let the cap hide a filter that failed

  ExtractionParams unfiltered = params;
  unfiltered.enable_ground_filter = false;
  const auto before = extractObstacles(grid.data(), width, height, intrinsics, unfiltered);
  ASSERT_GT(before.size(), 5u) <<
    "sanity: the synthetic grazing ground must actually fragment without the filter";

  const auto after = extractObstacles(grid.data(), width, height, intrinsics, params);
  EXPECT_LE(after.size(), 1u) <<
    "ground filter on: grazing-angle horizon must not read as obstacles";
}

TEST(ExtractObstacles, GroundFilterPreservesStandingBoxExtent)
{
  // Same level-ground floor as above, PLUS a box standing on it (front face
  // at constant depth, base touching the ground row where the two depths
  // agree) -- the stress case: a real obstacle whose base is exactly where
  // the filter is designed to remove points.
  const std::size_t width = 100;
  const std::size_t height = 80;
  auto grid = emptyGrid(width, height);
  const auto intrinsics = testIntrinsics();
  paintLevelGround(
    grid, width, height, intrinsics.cx, intrinsics.cy, intrinsics.fx, intrinsics.fy,
    /*height_m=*/0.5, /*min_range_m=*/0.2, /*max_range_m=*/19.1);

  // Box: 0.8 m tall, 0.5 m wide, front face at 3.0 m -- base row picked so
  // the ground reads ~3.0 m there too (box resting on the ground).
  const double box_depth_m = 3.0;
  const std::size_t v_bottom = 57;   // ground(57) = 0.5*100/17 = 2.94 m
  const std::size_t v_top = 30;      // (57-30)*3.0/100 = 0.81 m tall
  const std::size_t c_left = 42;
  const std::size_t c_right = 59;    // 17 px * 3.0/100 = 0.51 m wide
  paintBlock(grid, width, v_top, v_bottom + 1, c_left, c_right, static_cast<float>(box_depth_m));

  ExtractionParams params;
  params.pixel_stride = 1;
  params.min_cluster_points = 5;
  params.max_obstacles = 50;

  const auto obstacles = extractObstacles(grid.data(), width, height, intrinsics, params);

  // Ground fragments must be gone; the box is the only sizeable thing left.
  // (Box is ~17 px * ~28 rows = ~476 points; anything else surviving the
  // filter as noise is at most a handful of points.)
  const std::size_t kLargeClusterFloor = 100;
  const ExtractedObstacle * box = nullptr;
  for (const auto & obstacle : obstacles) {
    if (obstacle.point_count > kLargeClusterFloor) {
      ASSERT_EQ(box, nullptr) << "more than one large cluster survived the filter";
      box = &obstacle;
    }
  }
  ASSERT_NE(box, nullptr) << "the standing box must survive the ground filter";
  EXPECT_NEAR(box->center_z, box_depth_m, 1e-6);
  // Full 0.81 m span would need floating-point-perfect boundaries; the
  // filter trims the base band (documented limit), so allow it to shrink but
  // never by more than the ground_margin_m band (default 0.05 m) times a
  // small safety factor -- shrinking further would mean real box height was
  // eaten, not just the touching-the-floor row.
  EXPECT_GT(box->size_y, 0.81 - 3 * 0.05) << "box height must not have been eaten";
  EXPECT_LE(box->size_y, 0.81 + 1e-6);
}

TEST(ExtractObstacles, GroundFilterDisabledLeavesFragmentationUntouched)
{
  // The escape hatch: enable_ground_filter=false must reproduce the old,
  // unfiltered behaviour exactly (rollback path if the filter misbehaves).
  const std::size_t width = 100;
  const std::size_t height = 80;
  auto grid = emptyGrid(width, height);
  const auto intrinsics = testIntrinsics();
  paintLevelGround(
    grid, width, height, intrinsics.cx, intrinsics.cy, intrinsics.fx, intrinsics.fy,
    /*height_m=*/0.5, /*min_range_m=*/0.2, /*max_range_m=*/19.1);

  ExtractionParams params;
  params.pixel_stride = 1;
  params.min_cluster_points = 5;
  params.max_obstacles = 50;
  params.enable_ground_filter = false;

  const auto obstacles = extractObstacles(grid.data(), width, height, intrinsics, params);
  EXPECT_GT(obstacles.size(), 5u) <<
    "filter disabled: grazing ground must fragment exactly like before this change";
}

// --- Round 2 (P9 G-M3, follow_target): the camera tilts continuously while
// tracking, and the level-only invariant above silently retreats -- see
// AttitudeQuaternion/resolveGroundFilterAttitude in the header. Everything
// below is deliberately built with an INDEPENDENT ray-cast (different
// formula from obstacle_extraction.cpp's worldUpInBody() row-extraction
// shortcut), so these tests exercise the compensation as a black box.
namespace
{

struct Vec3T
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct QuatT
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

// Standard aerospace roll(X)-pitch(Y) composition, yaw fixed at 0 (yaw is
// dropped by design -- see README/coordinator decision).
QuatT quatFromRollPitch(double roll, double pitch)
{
  const double hr = roll / 2.0;
  const double hp = pitch / 2.0;
  const double sr = std::sin(hr);
  const double cr = std::cos(hr);
  const double sp = std::sin(hp);
  const double cp = std::cos(hp);
  QuatT q;
  q.x = sr * cp;
  q.y = cr * sp;
  q.z = -sr * sp;
  q.w = cr * cp;
  return q;
}

// Textbook quaternion-vector rotation (t=2*cross(u,v); v'=v+w*t+cross(u,t)),
// a from-scratch implementation independent of the production code path.
Vec3T rotateByQuat(const QuatT & q, const Vec3T & v)
{
  const Vec3T u{q.x, q.y, q.z};
  const Vec3T t{
    2.0 * (u.y * v.z - u.z * v.y),
    2.0 * (u.z * v.x - u.x * v.z),
    2.0 * (u.x * v.y - u.y * v.x)};
  return Vec3T{
    v.x + q.w * t.x + (u.y * t.z - u.z * t.y),
    v.y + q.w * t.y + (u.z * t.x - u.x * t.z),
    v.z + q.w * t.z + (u.x * t.y - u.y * t.x)};
}

AttitudeQuaternion toAttitude(const QuatT & q)
{
  return AttitudeQuaternion{q.x, q.y, q.z, q.w};
}

double degToRad(double degrees)
{
  return degrees * M_PI / 180.0;
}

// Ray-casts a flat world ground (z=0) plus an optional vertical wall patch
// (stand-in for a box's front face, world x=wall_forward_m, |y|<=half-width,
// 0<=z<=wall_height_m) against a camera tilted by (roll, pitch, yaw=0),
// sitting at world height camera_height_m. Genuine forward geometry -- this
// is what makes the true attitude quaternion recoverable independently of
// how obstacle_extraction.cpp computes its ground-invariant value.
std::vector<float> renderTiltedScene(
  std::size_t width, std::size_t height, double cx, double cy, double fx, double fy,
  double roll, double pitch, double camera_height_m,
  double min_range_m, double max_range_m,
  bool with_wall, double wall_forward_m, double wall_half_width_m, double wall_height_m)
{
  const QuatT attitude = quatFromRollPitch(roll, pitch);
  std::vector<float> grid(width * height, std::numeric_limits<float>::infinity());

  for (std::size_t r = 0; r < height; ++r) {
    for (std::size_t c = 0; c < width; ++c) {
      const Vec3T ray_optical{
        (static_cast<double>(c) - cx) / fx, (static_cast<double>(r) - cy) / fy, 1.0};
      double bx = 0.0;
      double by = 0.0;
      double bz = 0.0;
      uav_perception::opticalToBody(ray_optical.x, ray_optical.y, ray_optical.z, bx, by, bz);
      const Vec3T ray_world = rotateByQuat(attitude, Vec3T{bx, by, bz});

      double best_t = -1.0;
      if (ray_world.z < -1e-9) {
        const double t_ground = -camera_height_m / ray_world.z;
        if (t_ground > 0.0) {
          best_t = t_ground;
        }
      }
      if (with_wall && ray_world.x > 1e-9) {
        const double t_wall = wall_forward_m / ray_world.x;
        if (t_wall > 0.0) {
          const double hit_y = t_wall * ray_world.y;
          const double hit_z = camera_height_m + t_wall * ray_world.z;
          if (std::fabs(hit_y) <= wall_half_width_m && hit_z >= 0.0 && hit_z <= wall_height_m) {
            if (best_t < 0.0 || t_wall < best_t) {
              best_t = t_wall;
            }
          }
        }
      }
      if (best_t > min_range_m && best_t < max_range_m) {
        grid[r * width + c] = static_cast<float>(best_t);
      }
    }
  }
  return grid;
}

}  // namespace

TEST(ExtractObstacles, GroundFilterCompensatesPitchAndRollTilt)
{
  // ~follow_target tracking attitude, per coordinator's G-M3 gate report.
  const std::size_t width = 100;
  const std::size_t height = 80;
  const auto intrinsics = testIntrinsics();
  const double roll = degToRad(10.0);
  const double pitch = degToRad(15.0);
  const double camera_height_m = 0.5;
  const double wall_forward_m = 3.0;
  const double wall_half_width_m = 0.25;
  const double wall_height_m = 0.8;

  auto grid = renderTiltedScene(
    width, height, intrinsics.cx, intrinsics.cy, intrinsics.fx, intrinsics.fy,
    roll, pitch, camera_height_m, 0.2, 19.1,
    /*with_wall=*/true, wall_forward_m, wall_half_width_m, wall_height_m);

  ExtractionParams params;
  params.pixel_stride = 1;
  params.min_cluster_points = 5;
  params.max_obstacles = 50;

  // RED (this is round 1's fallback, unmodified): identity attitude on a
  // tilted scene is exactly the pre-round-2 behaviour -- the ratio gate
  // silently retreats every frame, precisely the G-M3 symptom (obstacle
  // count unchanged from before round 1's fix).
  const auto uncompensated = extractObstacles(grid.data(), width, height, intrinsics, params);
  EXPECT_GT(uncompensated.size(), 5u) <<
    "sanity: without attitude compensation, tilt must still fragment the ground";

  // GREEN: compensating with the true attitude restores the invariant.
  const AttitudeQuaternion attitude = toAttitude(quatFromRollPitch(roll, pitch));
  const auto compensated =
    extractObstacles(grid.data(), width, height, intrinsics, params, attitude);

  const std::size_t kLargeClusterFloor = 100;
  std::size_t large_clusters = 0;
  const ExtractedObstacle * box = nullptr;
  for (const auto & obstacle : compensated) {
    if (obstacle.point_count > kLargeClusterFloor) {
      ++large_clusters;
      box = &obstacle;
    }
  }
  EXPECT_EQ(large_clusters, 1u) <<
    "tilt-compensated: ground gone, exactly the wall/box survives";
  ASSERT_NE(box, nullptr) << "the standing box must survive tilt compensation";
  EXPECT_GT(box->center_z, 0.2);
  EXPECT_LT(box->center_z, 19.1);
}

TEST(ExtractObstacles, GroundFilterToleratesIncreasingTilt)
{
  // Sweep: the compensated filter must keep working across a realistic
  // tracking-pitch range, not just at one hand-picked angle.
  const std::size_t width = 100;
  const std::size_t height = 80;
  const auto intrinsics = testIntrinsics();
  const double camera_height_m = 0.5;

  for (const double tilt_deg : {0.0, 10.0, 25.0}) {
    const double pitch = degToRad(tilt_deg);
    const double roll = 0.0;
    auto grid = renderTiltedScene(
      width, height, intrinsics.cx, intrinsics.cy, intrinsics.fx, intrinsics.fy,
      roll, pitch, camera_height_m, 0.2, 19.1,
      /*with_wall=*/false, 0.0, 0.0, 0.0);

    ExtractionParams params;
    params.pixel_stride = 1;
    params.min_cluster_points = 5;
    params.max_obstacles = 50;

    const AttitudeQuaternion attitude = toAttitude(quatFromRollPitch(roll, pitch));
    const auto obstacles =
      extractObstacles(grid.data(), width, height, intrinsics, params, attitude);
    EXPECT_LE(obstacles.size(), 1u) <<
      "tilt " << tilt_deg << " deg: compensated filter must still clear pure ground";
  }
}

// --- Round 3 (P9 G-M3 gate re-block, 2026-08-22): the attitude-compensated
// filter still saw 18-19 obstacles/cycle on follow_target, WHILE ON THE WIRE
// the tracked attitude was near-perfectly level (|roll| max 1.08 deg, |pitch|
// max 5.38 deg). Diagnosed live against the real depth camera (432.5 fx/fy,
// 640x480) + the real target_box (0.5x0.5x0.8 m), dead level, drone
// stationary: dominant-bin ratio measured 0.7905 at 2.5 m standoff down to
// 0.3659 at 0.8 m -- follow_target's designed min_standoff_m (navigation_
// params.yaml) is 1.0 m, where the measured ratio was already only 0.5126.
// A near, frame-filling target starves the ratio gate even dead-level; that
// is what ground_min_depth_below_camera_m (below) is for.
TEST(ExtractObstacles, GroundFilterHandlesNearLargeTargetFrameComposition)
{
  // Real depth camera intrinsics (from a running sim's camera_info) and the
  // real target_box footprint, at the closest standoff measured on the wire.
  const std::size_t width = 640;
  const std::size_t height = 480;
  const DepthCameraIntrinsics intrinsics{432.5, 432.5, 320.0, 240.0};
  const double camera_height_m = 0.5;

  auto grid = renderTiltedScene(
    width, height, intrinsics.cx, intrinsics.cy, intrinsics.fx, intrinsics.fy,
    /*roll=*/0.0, /*pitch=*/0.0, camera_height_m, 0.2, 19.1,
    /*with_wall=*/true, /*wall_forward_m=*/0.8, /*wall_half_width_m=*/0.25,
    /*wall_height_m=*/0.8);

  // RED: round 1/2's ratio-only gate (0.5, no depth-below-camera gate --
  // -1e9 makes that AND term a no-op, reproducing the pre-round-3 code path
  // exactly) never clears this scene even though the camera is dead level.
  ExtractionParams old_params;
  old_params.pixel_stride = 2;
  old_params.min_cluster_points = 30;
  old_params.max_obstacles = 50;
  old_params.ground_min_inlier_ratio = 0.5;
  old_params.ground_min_depth_below_camera_m = -1e9;
  const auto before = extractObstacles(grid.data(), width, height, intrinsics, old_params);
  EXPECT_GT(before.size(), 5u) <<
    "sanity: the old 0.5 ratio gate must still fail this near-target scene";

  // GREEN: round 3 defaults (lowered ratio + the new depth-below-camera gate).
  ExtractionParams params;
  params.pixel_stride = 2;
  params.min_cluster_points = 30;
  params.max_obstacles = 50;
  const auto after = extractObstacles(grid.data(), width, height, intrinsics, params);

  const std::size_t kLargeClusterFloor = 500;
  std::size_t large_clusters = 0;
  const ExtractedObstacle * box = nullptr;
  for (const auto & obstacle : after) {
    if (obstacle.point_count > kLargeClusterFloor) {
      ++large_clusters;
      box = &obstacle;
    }
  }
  EXPECT_EQ(large_clusters, 1u) << "ground gone, only the near box remains";
  ASSERT_NE(box, nullptr) << "the near target must survive the frame-composition fix";
}

TEST(ExtractObstacles, GroundFilterNeverTreatsNearCameraHeightBandAsGround)
{
  // Adversarial regression for round 3's lowered ratio: a band of points
  // straddling the camera's OWN height (row == cy, so vertical-drop == 0
  // regardless of depth -- see backprojectPixel) covers 100% of valid
  // points here, clearing even the OLD 0.5 ratio with room to spare. Ground
  // is never at the camera's own height while flying -- this is exactly the
  // false-positive class ground_min_depth_below_camera_m exists to block,
  // and it is the reason lowering the ratio in round 3 is safe at all.
  const std::size_t width = 640;
  const std::size_t height = 480;
  const DepthCameraIntrinsics intrinsics{432.5, 432.5, 320.0, 240.0};
  auto grid = emptyGrid(width, height);
  // Rows cy-1..cy+1, full width, constant depth: |vertical-drop| <= 1*5/432.5
  // =~0.012 m for every point, one dominant bin, ratio 1.0 (only content).
  paintBlock(grid, width, 239, 242, 0, width, /*depth_value=*/5.0f);

  ExtractionParams params;
  params.pixel_stride = 1;
  params.min_cluster_points = 30;
  params.max_obstacles = 50;

  const auto obstacles = extractObstacles(grid.data(), width, height, intrinsics, params);
  ASSERT_EQ(obstacles.size(), 1u) <<
    "the near-camera-height band must survive as an obstacle, not be read as ground";
  EXPECT_NEAR(obstacles[0].center_z, 5.0, 1e-6);
  EXPECT_GT(obstacles[0].point_count, 500u);
}

TEST(ResolveGroundFilterAttitude, NoOdometryYetFallsBackToIdentity)
{
  const AttitudeQuaternion given{0.1, 0.2, 0.3, 0.9};
  const auto result = resolveGroundFilterAttitude(false, given, 0.0, 0.5);
  EXPECT_DOUBLE_EQ(result.x, 0.0);
  EXPECT_DOUBLE_EQ(result.y, 0.0);
  EXPECT_DOUBLE_EQ(result.z, 0.0);
  EXPECT_DOUBLE_EQ(result.w, 1.0);
}

TEST(ResolveGroundFilterAttitude, StaleOdometryFallsBackToIdentity)
{
  const AttitudeQuaternion given{0.0, 0.1305, 0.0, 0.9914};  // ~15 deg pitch
  const auto result = resolveGroundFilterAttitude(true, given, /*age_sec=*/0.51, /*max_age=*/0.5);
  EXPECT_DOUBLE_EQ(result.w, 1.0) << "just past max_age must degrade to identity";
}

TEST(ResolveGroundFilterAttitude, NegativeAgeFallsBackToIdentity)
{
  // A clock that appears to go backwards must not read as "very fresh".
  const AttitudeQuaternion given{0.0, 0.1305, 0.0, 0.9914};
  const auto result = resolveGroundFilterAttitude(true, given, /*age_sec=*/-0.01, /*max_age=*/0.5);
  EXPECT_DOUBLE_EQ(result.w, 1.0);
}

TEST(ResolveGroundFilterAttitude, NonFiniteQuaternionFallsBackToIdentity)
{
  const AttitudeQuaternion given{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 1.0};
  const auto result = resolveGroundFilterAttitude(true, given, /*age_sec=*/0.1, /*max_age=*/0.5);
  EXPECT_DOUBLE_EQ(result.w, 1.0);
}

TEST(ResolveGroundFilterAttitude, FreshFiniteOdometryPassesThrough)
{
  const AttitudeQuaternion given{0.0, 0.1305, 0.0, 0.9914};
  const auto result = resolveGroundFilterAttitude(true, given, /*age_sec=*/0.1, /*max_age=*/0.5);
  EXPECT_DOUBLE_EQ(result.x, given.x);
  EXPECT_DOUBLE_EQ(result.y, given.y);
  EXPECT_DOUBLE_EQ(result.z, given.z);
  EXPECT_DOUBLE_EQ(result.w, given.w);
}

TEST(ResolveGroundFilterAttitude, BoundaryAgeEqualsMaxIsStillFresh)
{
  const AttitudeQuaternion given{0.0, 0.1305, 0.0, 0.9914};
  const auto result = resolveGroundFilterAttitude(true, given, /*age_sec=*/0.5, /*max_age=*/0.5);
  EXPECT_DOUBLE_EQ(result.w, given.w) << "age == max_age must still count as fresh";
}

TEST(ExtractObstacles, ConfidenceGrowsWithClusterSizeAndSaturates)
{
  ExtractionParams params;
  params.pixel_stride = 1;
  params.confidence_saturating_points = 150.0;
  params.min_cluster_points = 1;

  auto small_grid = emptyGrid(100, 80);
  paintBlock(small_grid, 100, 10, 15, 10, 15, 3.0f);  // 25 points
  const auto small = extractObstacles(small_grid.data(), 100, 80, testIntrinsics(), params);

  auto large_grid = emptyGrid(100, 80);
  paintBlock(large_grid, 100, 10, 30, 10, 30, 3.0f);  // 400 points
  const auto large = extractObstacles(large_grid.data(), 100, 80, testIntrinsics(), params);

  ASSERT_EQ(small.size(), 1u);
  ASSERT_EQ(large.size(), 1u);
  EXPECT_NEAR(small[0].confidence, 25.0 / 150.0, 1e-6);
  EXPECT_DOUBLE_EQ(large[0].confidence, 1.0) << "400 points saturates the scale of 150";
  EXPECT_LT(small[0].confidence, large[0].confidence);
}
