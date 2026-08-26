#include "uav_world_model/pose_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "uav_world_model/uncertainty.hpp"

namespace uav_world_model
{

PoseBuffer::PoseBuffer(PoseBufferOptions options)
: options_(std::move(options))
{
}

bool PoseBuffer::add(const TimedPose & pose)
{
  if (!std::isfinite(pose.stamp_sec) || pose.stamp_sec <= 0.0) {
    return false;
  }
  if (!isUsableUncertainty(pose.position_stddev)) {
    return false;
  }

  const auto position = std::upper_bound(
    poses_.begin(), poses_.end(), pose.stamp_sec,
    [](double stamp, const TimedPose & entry) {return stamp < entry.stamp_sec;});
  poses_.insert(position, pose);

  const double horizon = poses_.back().stamp_sec - options_.history_sec;
  while (poses_.size() > 1 && poses_.front().stamp_sec < horizon) {
    poses_.pop_front();
  }
  return true;
}

PoseMatch PoseBuffer::lookup(double stamp_sec) const
{
  PoseMatch match;
  if (poses_.empty() || !std::isfinite(stamp_sec)) {
    return match;
  }

  const TimedPose & oldest = poses_.front();
  const TimedPose & newest = poses_.back();

  if (stamp_sec >= newest.stamp_sec) {
    const double gap = stamp_sec - newest.stamp_sec;
    if (gap > options_.max_pose_gap_sec) {
      match.result = PoseMatchResult::kObservationTooNew;
      return match;
    }
    match.result = PoseMatchResult::kOk;
    match.odom_from_base = newest.odom_from_base;
    match.position_stddev = newest.position_stddev;
    match.pairing_gap_sec = gap;
    match.body_velocity = newest.body_velocity;
    match.body_angular_velocity = newest.body_angular_velocity;
    return match;
  }

  if (stamp_sec <= oldest.stamp_sec) {
    const double gap = oldest.stamp_sec - stamp_sec;
    if (gap > options_.max_pose_gap_sec) {
      match.result = PoseMatchResult::kObservationTooOld;
      return match;
    }
    match.result = PoseMatchResult::kOk;
    match.odom_from_base = oldest.odom_from_base;
    match.position_stddev = oldest.position_stddev;
    match.pairing_gap_sec = gap;
    match.body_velocity = oldest.body_velocity;
    match.body_angular_velocity = oldest.body_angular_velocity;
    return match;
  }

  const auto after = std::upper_bound(
    poses_.begin(), poses_.end(), stamp_sec,
    [](double stamp, const TimedPose & entry) {return stamp < entry.stamp_sec;});
  const auto before = std::prev(after);

  const double span = after->stamp_sec - before->stamp_sec;
  const double ratio = span > 0.0 ? (stamp_sec - before->stamp_sec) / span : 0.0;

  match.result = PoseMatchResult::kOk;
  match.odom_from_base.translation = uav_world_model::add(
    scale(before->odom_from_base.translation, 1.0 - ratio),
    scale(after->odom_from_base.translation, ratio));
  match.odom_from_base.rotation =
    slerp(before->odom_from_base.rotation, after->odom_from_base.rotation, ratio);
  match.position_stddev = std::max(before->position_stddev, after->position_stddev);
  match.pairing_gap_sec =
    std::min(stamp_sec - before->stamp_sec, after->stamp_sec - stamp_sec);
  const TimedPose & faster =
    norm(before->body_velocity) >= norm(after->body_velocity) ? *before : *after;
  match.body_velocity = faster.body_velocity;
  match.body_angular_velocity = faster.body_angular_velocity;
  return match;
}

bool PoseBuffer::empty() const
{
  return poses_.empty();
}

std::size_t PoseBuffer::size() const
{
  return poses_.size();
}

double PoseBuffer::newestStamp() const
{
  return poses_.empty() ? 0.0 : poses_.back().stamp_sec;
}

const TimedPose * PoseBuffer::newest() const
{
  return poses_.empty() ? nullptr : &poses_.back();
}

}  // namespace uav_world_model
