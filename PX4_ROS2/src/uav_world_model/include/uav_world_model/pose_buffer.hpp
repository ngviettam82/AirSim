#ifndef UAV_WORLD_MODEL__POSE_BUFFER_HPP_
#define UAV_WORLD_MODEL__POSE_BUFFER_HPP_

#include <cstddef>
#include <deque>

#include "uav_world_model/frame_transforms.hpp"

namespace uav_world_model
{

struct TimedPose
{
  double stamp_sec{0.0};
  Rigid odom_from_base;
  double position_stddev{-1.0};
  Vec3 body_velocity;
  Vec3 body_angular_velocity;
};

enum class PoseMatchResult
{
  kOk,
  kNoPose,
  kObservationTooNew,
  kObservationTooOld,
};

struct PoseMatch
{
  PoseMatchResult result{PoseMatchResult::kNoPose};
  Rigid odom_from_base;
  double position_stddev{-1.0};
  double pairing_gap_sec{0.0};
  Vec3 body_velocity;
  Vec3 body_angular_velocity;

  bool ok() const {return result == PoseMatchResult::kOk;}
  // How far the aircraft may have moved meanwhile.
  double pairingUncertainty() const {return pairing_gap_sec * norm(body_velocity);}
};

struct PoseBufferOptions
{
  double history_sec{3.0};
  double max_pose_gap_sec{0.25};
};

class PoseBuffer
{
public:
  explicit PoseBuffer(PoseBufferOptions options = {});

  // A pose that cannot state its error cannot be weighed.
  bool add(const TimedPose & pose);

  PoseMatch lookup(double stamp_sec) const;

  bool empty() const;
  std::size_t size() const;
  double newestStamp() const;
  const TimedPose * newest() const;

private:
  PoseBufferOptions options_;
  std::deque<TimedPose> poses_;
};

}  // namespace uav_world_model

#endif  // UAV_WORLD_MODEL__POSE_BUFFER_HPP_
