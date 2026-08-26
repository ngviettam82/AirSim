// Ego-motion compensation for target_tracker_node (bug #10 follow-up,
// G-M3/G-M4, 2026-08-23): a real, stationary object churned track_id while
// the drone orbited it, because association ran directly in the
// camera-optical frame -- a frame that moves with the vehicle. See README.

#ifndef UAV_PERCEPTION__EGO_MOTION_HPP_
#define UAV_PERCEPTION__EGO_MOTION_HPP_

#include "uav_perception/marker_pose.hpp"  // AttitudeQuaternion

namespace uav_perception
{

// Body-to-odom rigid pose (nav_msgs Odometry convention: pose.position +
// pose.orientation express odom_from_body). Camera-to-body MOUNT OFFSET is
// treated as zero throughout this file (see opticalPointToOdom) -- this
// struct is the vehicle's own pose, not the camera's.
struct BodyPose
{
  AttitudeQuaternion orientation;
  double x{0.0};
  double y{0.0};
  double z{0.0};

  // Finite position + a valid() orientation. Does not check freshness --
  // see resolveBodyPose for the R32 age gate.
  bool valid() const;
};

// R32 sample-and-hold freshness: finite, non-negative, within max_age_sec.
// A clock that appears to go backwards must not read as "very fresh".
bool isPoseFresh(double age_sec, double max_age_sec);

// Two-tier degrade (R32), mirrors obstacle_extraction's
// resolveGroundFilterAttitude: Tier 1 = fresh, finite odometry pose (returns
// true, out_pose filled in). Tier 2 = odometry missing, stale, or the pose
// itself not finite (returns false, out_pose untouched) -- the caller falls
// back to associating directly in the camera-optical frame, the pre-fix
// behaviour, and must reset any track state that was accumulated in the
// other frame (see target_tracker_node.cpp / README: mode switch = reset,
// same rule as a camera frame_id change).
bool resolveBodyPose(
  bool have_odometry, const BodyPose & odometry_pose,
  double age_sec, double max_age_sec, BodyPose & out_pose);

// optical point -> odom point: optical->body (opticalToBody, the fixed
// mount-rotation permutation) -> odom (rotate by the vehicle's orientation,
// then translate by its odom position). Used to bring a fresh observation
// into the odom frame BEFORE association, so a stationary object's odom
// position stays constant regardless of vehicle motion.
//
// Camera-to-body MOUNT TRANSLATION (~0.12-0.13 m off base_link, see
// obstacle_extractor_node README) is treated as zero here -- a documented,
// bounded simplification: it is small, CONSTANT, and does not itself grow
// with vehicle motion, unlike the multi-metre frame-to-frame jumps this fix
// targets (an orbiting drone's yaw/position change between frames). Revisit
// if sub-mount-offset precision is ever needed downstream.
void opticalPointToOdom(
  double optical_x, double optical_y, double optical_z, const BodyPose & body_to_odom,
  double & odom_x, double & odom_y, double & odom_z);

// Inverse of opticalPointToOdom: odom point -> optical point, as it would
// appear from the CURRENT camera pose. Used to convert a track's internal
// odom-frame position estimate back to the published camera-optical frame
// without changing the interface contract (frame_id, fields unchanged).
void odomPointToOptical(
  double odom_x, double odom_y, double odom_z, const BodyPose & body_to_odom,
  double & optical_x, double & optical_y, double & optical_z);

// Vector variant for velocity: rotation only, no translation, and -- on
// purpose -- no subtraction of the vehicle's own velocity. A target's
// ODOM-frame velocity is already ego-motion-free (that is the point of this
// fix): re-expressing it in the current camera axes needs no further
// correction. A stationary target's published velocity is now exactly zero
// regardless of vehicle motion, improving on the old, documented limitation
// ("velocity only absolute when the drone is stationary" -- see README).
void odomVectorToOptical(
  double odom_vx, double odom_vy, double odom_vz, const BodyPose & body_to_odom,
  double & optical_vx, double & optical_vy, double & optical_vz);

}  // namespace uav_perception

#endif  // UAV_PERCEPTION__EGO_MOTION_HPP_
