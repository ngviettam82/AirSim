#include "uav_perception/obstacle_extraction.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <unordered_map>
#include <vector>

#include "uav_perception/marker_pose.hpp"

namespace uav_perception
{

bool DepthCameraIntrinsics::valid() const
{
  return fx > 0.0 && fy > 0.0;
}

// AttitudeQuaternion::valid() moved to marker_pose.cpp 2026-08-23 (type now
// lives in marker_pose.hpp).

DepthCameraIntrinsics depthIntrinsicsFrom(const std::array<double, 9> & k)
{
  DepthCameraIntrinsics intrinsics;
  intrinsics.fx = k[0];
  intrinsics.fy = k[4];
  intrinsics.cx = k[2];
  intrinsics.cy = k[5];
  return intrinsics;
}

bool isValidDepth(float value, const ExtractionParams & params)
{
  return std::isfinite(value) &&
         value > static_cast<float>(params.min_range_m) &&
         value < static_cast<float>(params.max_range_m);
}

void backprojectPixel(
  double u, double v, double depth, const DepthCameraIntrinsics & intrinsics,
  double & x, double & y, double & z)
{
  x = (u - intrinsics.cx) * depth / intrinsics.fx;
  y = (v - intrinsics.cy) * depth / intrinsics.fy;
  z = depth;
}

AttitudeQuaternion resolveGroundFilterAttitude(
  bool have_odometry, const AttitudeQuaternion & odometry_orientation,
  double age_sec, double max_age_sec)
{
  if (!have_odometry) {
    return AttitudeQuaternion{};  // Tier 2: nothing has arrived yet
  }
  if (!(std::isfinite(age_sec) && age_sec >= 0.0 && age_sec <= max_age_sec)) {
    return AttitudeQuaternion{};  // Tier 2: stale, or a clock that went backwards (R32)
  }
  if (!odometry_orientation.valid()) {
    return AttitudeQuaternion{};  // Tier 2: non-finite quaternion (R30)
  }
  return odometry_orientation;  // Tier 1
}

namespace
{
constexpr int kNoLabel = -1;

struct Vec3Local
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

// Never trust a single layer (R0): the node is expected to gate staleness
// (R32) before calling in, but the library re-checks and falls back to
// identity on its own rather than propagate a degenerate quaternion.
AttitudeQuaternion normalizedOrIdentity(const AttitudeQuaternion & attitude)
{
  if (!attitude.valid()) {
    return AttitudeQuaternion{};
  }
  const double norm = std::sqrt(
    attitude.x * attitude.x + attitude.y * attitude.y +
    attitude.z * attitude.z + attitude.w * attitude.w);
  return AttitudeQuaternion{
    attitude.x / norm, attitude.y / norm, attitude.z / norm, attitude.w / norm};
}

// World-up axis expressed in the BODY frame -- the third row of the
// body-to-world rotation matrix built from a unit quaternion. PROVABLY
// yaw-independent: a rotation about the world Z axis cannot change how much
// of any body axis points along world Z. That is what lets this compensate
// roll+pitch and "drop yaw" (per design) without ever extracting Euler
// angles, so there is no gimbal-lock edge case to get wrong.
Vec3Local worldUpInBody(const AttitudeQuaternion & attitude)
{
  const AttitudeQuaternion q = normalizedOrIdentity(attitude);
  return Vec3Local{
    2.0 * (q.x * q.z - q.w * q.y),
    2.0 * (q.y * q.z + q.w * q.x),
    1.0 - 2.0 * (q.x * q.x + q.y * q.y)};
}

// How far BELOW the camera a point sits, measured along world-vertical
// (positive = below), with vehicle roll/pitch removed. Identity attitude
// (the R30/R32 fallback) collapses this to EXACTLY the old optical-down
// value: opticalToBody maps optical y to body z = -optical_y, and identity
// attitude's up-in-body is (0,0,1), so verticalDrop = -body_z = optical_y.
double verticalDrop(
  double optical_x, double optical_y, double optical_z, const AttitudeQuaternion & attitude)
{
  double body_x = 0.0;
  double body_y = 0.0;
  double body_z = 0.0;
  opticalToBody(optical_x, optical_y, optical_z, body_x, body_y, body_z);
  const Vec3Local up = worldUpInBody(attitude);
  const double height_world = body_x * up.x + body_y * up.y + body_z * up.z;
  return -height_world;
}

// Level camera, flat ground: for any ground point the vertical-drop value
// equals the camera's height above the ground, REGARDLESS of range, bearing,
// or vehicle roll/pitch (verticalDrop already compensates for tilt -- see
// above). A real object's own surfaces are not parallel to the ground, so
// their points spread across many values instead of piling onto one -- that
// spread is what tells ground apart from obstacle here, not depth
// continuity (see README ground-plane pitfall).
bool findDominantGroundLevel(
  const std::vector<double> & drop_values, const ExtractionParams & params,
  double & out_ground_drop)
{
  if (drop_values.size() < params.ground_min_inlier_points || params.ground_margin_m <= 0.0) {
    return false;
  }

  const double bin_width = params.ground_margin_m;
  std::unordered_map<long long, std::size_t> bin_counts;
  std::unordered_map<long long, double> bin_sums;
  for (const double drop : drop_values) {
    const auto bin = static_cast<long long>(std::floor(drop / bin_width + 0.5));
    bin_counts[bin] += 1;
    bin_sums[bin] += drop;
  }

  long long best_bin = 0;
  std::size_t best_count = 0;
  for (const auto & entry : bin_counts) {
    if (entry.second > best_count) {
      best_count = entry.second;
      best_bin = entry.first;
    }
  }

  const double ratio =
    static_cast<double>(best_count) / static_cast<double>(drop_values.size());
  const double candidate_drop = bin_sums.at(best_bin) / static_cast<double>(best_count);
  // Round 3 (2026-08-22, G-M3 gate re-block): a near, large target can
  // legitimately push ground below a ratio majority even dead-level --
  // measured on the wire down to 0.37 at 0.8 m standoff. This third gate is
  // what makes lowering ground_min_inlier_ratio safe: no candidate at or
  // above camera height (a wall, a big object face filling the frame) can
  // ever pass it, no matter its ratio. Narrower than "blocks every non-ground
  // plane" (Y7, 2026-08-23): a large horizontal surface BELOW this margin
  // too (a box lid, a car roof) still clears all three gates -- see
  // ExtractionParams::ground_min_depth_below_camera_m and README derivation.
  const bool below_camera = candidate_drop >= params.ground_min_depth_below_camera_m;
  const bool triggers =
    best_count >= params.ground_min_inlier_points &&
    ratio >= params.ground_min_inlier_ratio &&
    below_camera;
  // Opt-in diagnostic (env-gated, zero cost otherwise) -- kept from the G-M3
  // round 3 investigation because this exact "gate retreats silently" class
  // of bug has recurred across all three rounds; see README.
  if (std::getenv("UAV_GROUND_FILTER_DEBUG") != nullptr) {
    std::fprintf(
      stderr,
      "[ground_filter_debug] n_valid=%zu best_count=%zu ratio=%.4f "
      "ratio_threshold=%.4f count_threshold=%zu candidate_drop=%.4f "
      "depth_threshold=%.4f triggers=%d\n",
      drop_values.size(), best_count, ratio, params.ground_min_inlier_ratio,
      params.ground_min_inlier_points, candidate_drop,
      params.ground_min_depth_below_camera_m, triggers ? 1 : 0);
  }
  if (!triggers) {
    return false;
  }

  out_ground_drop = candidate_drop;
  return true;
}
}  // namespace

std::vector<ExtractedObstacle> extractObstacles(
  const float * depth, std::size_t width, std::size_t height,
  const DepthCameraIntrinsics & intrinsics, const ExtractionParams & params,
  const AttitudeQuaternion & body_to_world_attitude)
{
  std::vector<ExtractedObstacle> obstacles;
  if (depth == nullptr || width == 0 || height == 0 || !intrinsics.valid()) {
    return obstacles;
  }

  // Subsample onto a grid; realtime budget, not full resolution (see README).
  const std::size_t stride = std::max<std::size_t>(1, params.pixel_stride);
  const std::size_t grid_w = (width + stride - 1) / stride;
  const std::size_t grid_h = (height + stride - 1) / stride;

  std::vector<std::size_t> sample_col(grid_w);
  for (std::size_t gc = 0; gc < grid_w; ++gc) {
    sample_col[gc] = std::min(gc * stride, width - 1);
  }
  std::vector<std::size_t> sample_row(grid_h);
  for (std::size_t gr = 0; gr < grid_h; ++gr) {
    sample_row[gr] = std::min(gr * stride, height - 1);
  }

  const std::size_t grid_size = grid_w * grid_h;
  std::vector<float> value(grid_size);
  std::vector<bool> valid(grid_size);
  for (std::size_t gr = 0; gr < grid_h; ++gr) {
    for (std::size_t gc = 0; gc < grid_w; ++gc) {
      const std::size_t idx = gr * grid_w + gc;
      const float sample = depth[sample_row[gr] * width + sample_col[gc]];
      value[idx] = sample;
      valid[idx] = isValidDepth(sample, params);
    }
  }

  // Ground-plane pre-filter: runs BEFORE clustering, on the (attitude-
  // compensated) vertical-drop value, not on depth-value neighbours, so it
  // strips ground regardless of how badly grazing-angle rows would otherwise
  // fragment under flood fill -- AND regardless of vehicle tilt (identity
  // attitude, the R30/R32 fallback, reduces this to the old level-only
  // behaviour exactly, see verticalDrop() above).
  if (params.enable_ground_filter) {
    std::vector<double> drop_values;
    drop_values.reserve(grid_size);
    for (std::size_t gr = 0; gr < grid_h; ++gr) {
      const double v = static_cast<double>(sample_row[gr]);
      for (std::size_t gc = 0; gc < grid_w; ++gc) {
        const std::size_t idx = gr * grid_w + gc;
        if (!valid[idx]) {
          continue;
        }
        const std::size_t u_col = sample_col[gc];
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        backprojectPixel(
          static_cast<double>(u_col), v, static_cast<double>(value[idx]), intrinsics, x, y, z);
        drop_values.push_back(verticalDrop(x, y, z, body_to_world_attitude));
      }
    }

    double ground_drop = 0.0;
    if (findDominantGroundLevel(drop_values, params, ground_drop)) {
      for (std::size_t gr = 0; gr < grid_h; ++gr) {
        const double v = static_cast<double>(sample_row[gr]);
        for (std::size_t gc = 0; gc < grid_w; ++gc) {
          const std::size_t idx = gr * grid_w + gc;
          if (!valid[idx]) {
            continue;
          }
          const std::size_t u_col = sample_col[gc];
          double x = 0.0;
          double y = 0.0;
          double z = 0.0;
          backprojectPixel(
            static_cast<double>(u_col), v, static_cast<double>(value[idx]), intrinsics, x, y, z);
          const double drop = verticalDrop(x, y, z, body_to_world_attitude);
          if (std::fabs(drop - ground_drop) <= params.ground_margin_m) {
            valid[idx] = false;  // on the ground plane, not an obstacle
          }
        }
      }
    }
  }

  // Flood fill over depth discontinuities; 4-connectivity on the grid.
  std::vector<int> label(grid_size, kNoLabel);
  int next_label = 0;
  std::vector<std::size_t> stack;
  std::vector<std::size_t> members;

  static const long kRowStep[4] = {-1, 1, 0, 0};
  static const long kColStep[4] = {0, 0, -1, 1};

  for (std::size_t start = 0; start < grid_size; ++start) {
    if (!valid[start] || label[start] != kNoLabel) {
      continue;
    }

    const int this_label = next_label++;
    stack.clear();
    members.clear();
    stack.push_back(start);
    label[start] = this_label;
    float cluster_min_depth = value[start];
    float cluster_max_depth = value[start];

    while (!stack.empty()) {
      const std::size_t idx = stack.back();
      stack.pop_back();
      members.push_back(idx);

      const long gr = static_cast<long>(idx / grid_w);
      const long gc = static_cast<long>(idx % grid_w);

      for (int n = 0; n < 4; ++n) {
        const long nr = gr + kRowStep[n];
        const long nc = gc + kColStep[n];
        if (nr < 0 || nc < 0 || nr >= static_cast<long>(grid_h) ||
          nc >= static_cast<long>(grid_w))
        {
          continue;
        }
        const std::size_t nidx =
          static_cast<std::size_t>(nr) * grid_w + static_cast<std::size_t>(nc);
        if (!valid[nidx] || label[nidx] != kNoLabel) {
          continue;
        }
        if (std::fabs(value[nidx] - value[idx]) > params.cluster_depth_tolerance_m) {
          continue;
        }
        const float candidate_min = std::min(cluster_min_depth, value[nidx]);
        const float candidate_max = std::max(cluster_max_depth, value[nidx]);
        if (candidate_max - candidate_min > static_cast<float>(params.max_cluster_depth_span_m)) {
          continue;
        }
        cluster_min_depth = candidate_min;
        cluster_max_depth = candidate_max;
        label[nidx] = this_label;
        stack.push_back(nidx);
      }
    }

    if (members.size() < params.min_cluster_points) {
      continue;
    }

    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    double closest = std::numeric_limits<double>::infinity();

    for (const auto member : members) {
      const std::size_t gr = member / grid_w;
      const std::size_t gc = member % grid_w;
      const double u = static_cast<double>(sample_col[gc]);
      const double v = static_cast<double>(sample_row[gr]);

      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      backprojectPixel(u, v, static_cast<double>(value[member]), intrinsics, x, y, z);

      min_x = std::min(min_x, x);
      max_x = std::max(max_x, x);
      min_y = std::min(min_y, y);
      max_y = std::max(max_y, y);
      min_z = std::min(min_z, z);
      max_z = std::max(max_z, z);
      closest = std::min(closest, std::sqrt(x * x + y * y + z * z));
    }

    ExtractedObstacle obstacle;
    obstacle.center_x = (min_x + max_x) / 2.0;
    obstacle.center_y = (min_y + max_y) / 2.0;
    obstacle.center_z = (min_z + max_z) / 2.0;
    obstacle.size_x = max_x - min_x;
    obstacle.size_y = max_y - min_y;
    obstacle.size_z = max_z - min_z;
    obstacle.distance = closest;
    obstacle.confidence = std::min(
      1.0,
      std::max(
        0.0,
        static_cast<double>(members.size()) /
        std::max(1e-6, params.confidence_saturating_points)));
    obstacle.point_count = members.size();
    obstacles.push_back(obstacle);

    if (obstacles.size() >= params.max_obstacles) {
      break;
    }
  }

  return obstacles;
}

}  // namespace uav_perception
