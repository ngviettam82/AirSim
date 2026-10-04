// Depth is Z (perpendicular to image plane), not ray length; see README.

#ifndef UAV_PERCEPTION__OBSTACLE_EXTRACTION_HPP_
#define UAV_PERCEPTION__OBSTACLE_EXTRACTION_HPP_

#include <array>
#include <cstddef>
#include <vector>

#include "uav_perception/marker_pose.hpp"  // AttitudeQuaternion

namespace uav_perception
{

// Mirrors marker_pose::CameraIntrinsics; kept separate to avoid coupling.
struct DepthCameraIntrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};

  // Never-arrived CameraInfo reads as zero, not invalid.
  bool valid() const;
};

/// Reads the 3x3 row-major intrinsic matrix ROS carries in CameraInfo.k.
DepthCameraIntrinsics depthIntrinsicsFrom(const std::array<double, 9> & k);

struct ExtractionParams
{
  double min_range_m{0.2};
  double max_range_m{19.1};
  double cluster_depth_tolerance_m{0.3};
  // Bounds chaining across gradual slopes (e.g. ground seen at grazing
  // angle): local steps can each pass cluster_depth_tolerance_m yet sum to
  // an unbounded span, see README ground-plane pitfall.
  double max_cluster_depth_span_m{2.0};
  std::size_t pixel_stride{2};
  std::size_t min_cluster_points{30};
  double confidence_saturating_points{150.0};
  std::size_t max_obstacles{20};

  // Ground-plane pre-filter (README "phan manh nen"): runs before clustering,
  // removes points sharing the dominant vertical-drop value (flat-ground
  // invariant, attitude-compensated -- see README for the derivation).
  // Defaults ON.
  bool enable_ground_filter{true};
  // Band half-width AND histogram bin width for the dominant-level search, m.
  double ground_margin_m{0.05};
  // Safety gate: only trust a dominant level as "ground" when it explains at
  // least this share of the frame's valid points. LOWERED in round 3
  // (2026-08-22, G-M3 gate re-block): a near target (follow_target's
  // designed min_standoff_m is 1.0 m) fills enough of the frame that ground
  // legitimately drops below a 50% majority even on a dead-level camera --
  // measured on the wire down to 0.37 at 0.8 m standoff, still tens of
  // thousands of points. Safe to lower ONLY because ground_min_depth_below_
  // camera_m below is now the primary discriminator against a false
  // positive, not this ratio.
  double ground_min_inlier_ratio{0.15};
  // Second safety gate: absolute floor so a nearly-empty frame cannot trip
  // the ratio gate on a handful of coincidentally aligned points.
  std::size_t ground_min_inlier_points{200};
  // Third safety gate (round 3, R0): a dominant level is only trusted as
  // ground if it sits at least this far BELOW the camera (positive
  // vertical-drop). Ground is always below a flying vehicle by roughly the
  // flight altitude; the mission-level floor is 0.5 m (navigation_params.yaml
  // min_altitude_m), so this default sits well under that with margin for
  // mount/calibration noise. PRECISE CLAIM (Y7, narrowed 2026-08-23): this
  // blocks a dominant surface AT OR ABOVE camera height (a wall, a big box
  // face facing the camera) no matter how low ground_min_inlier_ratio goes.
  // It does NOT distinguish true ground from any OTHER large horizontal
  // surface that happens to sit >= this far below the camera (a box lid, a
  // car roof, a low table seen from above) -- see README "hạn chế đã biết"
  // for the known-low-risk geometry this currently relies on
  // (forward-looking camera, objects mostly standing/vertical).
  double ground_min_depth_below_camera_m{0.15};
};

// AttitudeQuaternion moved to marker_pose.hpp 2026-08-23 (ego_motion needed
// it too; see that header). Identity (default) reproduces the old,
// uncompensated ground filter exactly -- see README/obstacle_extraction.cpp:
// this is also the R30/R32 fallback value when odometry is missing, stale,
// or not finite.

// One connected region of near-constant depth; not yet a ROS message.
struct ExtractedObstacle
{
  double center_x{0.0};
  double center_y{0.0};
  double center_z{0.0};
  double size_x{0.0};
  double size_y{0.0};
  double size_z{0.0};
  double distance{0.0};    // closest Euclidean range to the sensor, m
  double confidence{0.0};
  std::size_t point_count{0};
};

bool isValidDepth(float value, const ExtractionParams & params);

// Two-tier degrade order for the ground filter's attitude input (R30/R32,
// coordinator decision 2026-08-22): Tier 1 = fresh, finite odometry
// orientation; Tier 2 = identity (the old, level-only behaviour) whenever
// odometry has never arrived, is older than max_age_sec, or age_sec itself
// is not a sane elapsed time (negative/non-finite -- a clock going backwards
// must not read as "very fresh"). ROS-free and ROS-node-free on purpose so
// the degrade order is testable without spinning an executor.
AttitudeQuaternion resolveGroundFilterAttitude(
  bool have_odometry, const AttitudeQuaternion & odometry_orientation,
  double age_sec, double max_age_sec);

// Standard pinhole model: x=(u-cx)*z/fx, y=(v-cy)*z/fy, z=depth.
void backprojectPixel(
  double u, double v, double depth, const DepthCameraIntrinsics & intrinsics,
  double & x, double & y, double & z);

/// depth: row-major, width*height contiguous samples, no row padding.
/// Each call is independent; no state carries across frames (sparse-image safe).
/// body_to_world_attitude tilts the ground filter's invariant with the
/// vehicle (see README); identity (the default) is the pre-P9-G-M3-fix
/// behaviour, level-camera only.
std::vector<ExtractedObstacle> extractObstacles(
  const float * depth, std::size_t width, std::size_t height,
  const DepthCameraIntrinsics & intrinsics, const ExtractionParams & params,
  const AttitudeQuaternion & body_to_world_attitude = AttitudeQuaternion{});

}  // namespace uav_perception

#endif  // UAV_PERCEPTION__OBSTACLE_EXTRACTION_HPP_
