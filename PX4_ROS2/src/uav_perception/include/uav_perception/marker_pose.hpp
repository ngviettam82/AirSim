// solvePnP answers in OPTICAL frame, not body (see README).

#ifndef UAV_PERCEPTION__MARKER_POSE_HPP_
#define UAV_PERCEPTION__MARKER_POSE_HPP_

#include <array>
#include <cstddef>

namespace uav_perception
{

struct CameraIntrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};

  // Never-arrived CameraInfo reads as zero, not invalid (see README).
  bool valid() const;
};

/// Reads the 3x3 row-major intrinsic matrix ROS carries in CameraInfo.k.
CameraIntrinsics intrinsicsFrom(const std::array<double, 9> & k);

// Can't catch planar pose ambiguity (see README).
double confidenceFromReprojection(double error_px, double saturating_error_px = 5.0);

// Body-to-world/odom attitude (nav_msgs Odometry orientation convention).
// MOVED here 2026-08-23 (target_tracker ego-motion fix) from
// obstacle_extraction.hpp, which had the first use (ground-filter tilt
// compensation, round 2) -- relocated so a second, independent consumer
// (ego_motion.hpp) does not have to duplicate or depend the wrong way.
// obstacle_extraction.hpp now gets it from here; behaviour unchanged.
struct AttitudeQuaternion
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};

  // Finite and not near-zero norm. Does not require unit norm -- callers
  // normalize before use (defensive: never trust a single layer, see R0).
  bool valid() const;
};

// optical(right,down,fwd) -> body(fwd,left,up), per axis.
void opticalToBody(
  double optical_x, double optical_y, double optical_z,
  double & body_x, double & body_y, double & body_z);

// body(fwd,left,up) -> optical(right,down,fwd), the exact inverse of
// opticalToBody above. New 2026-08-23 for ego_motion's odom->optical
// back-transform (a track's internal odom-frame estimate, converted back to
// the published camera-optical frame without changing the contract).
void bodyToOptical(
  double body_x, double body_y, double body_z,
  double & optical_x, double & optical_y, double & optical_z);

// Corner order: OpenCV's top-left, top-right, bottom-right, bottom-left.
std::array<std::array<double, 3>, 4> markerCorners(double edge_length_m);

// (axis*angle) -> quaternion (x, y, z, w).
std::array<double, 4> quaternionFromRotationVector(
  double rx, double ry, double rz);

}  // namespace uav_perception

#endif  // UAV_PERCEPTION__MARKER_POSE_HPP_
