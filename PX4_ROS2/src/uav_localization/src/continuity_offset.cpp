#include "uav_localization/continuity_offset.hpp"

#include <cmath>

namespace uav_localization
{

bool ContinuityOffset::advance(double seconds, double decay_rate)
{
  // Read before decaying: the tick that reaches zero still moved the pose.
  const bool was_active = active();

  if (!(seconds > 0.0) || !(decay_rate > 0.0)) {
    return was_active;
  }

  const double step = decay_rate * seconds;
  for (double & axis : offset_) {
    if (std::abs(axis) <= step) {
      axis = 0.0;
    } else {
      axis -= std::copysign(step, axis);
    }
  }

  return was_active || active();
}

void ContinuityOffset::anchor(const Vector & held, const Vector & source)
{
  for (std::size_t axis = 0; axis < offset_.size(); ++axis) {
    offset_[axis] = held[axis] - source[axis];
  }
}

void ContinuityOffset::clear()
{
  offset_ = {0.0, 0.0, 0.0};
}

bool ContinuityOffset::active() const
{
  return offset_[0] != 0.0 || offset_[1] != 0.0 || offset_[2] != 0.0;
}

bool anchorOutlivesBlackout(double blackout_sec, double grace_sec)
{
  // Absorbing after a long loss would hide not knowing where we are.
  if (!std::isfinite(blackout_sec) || !std::isfinite(grace_sec) || blackout_sec < 0.0) {
    return false;
  }
  return blackout_sec <= grace_sec;
}

double ContinuityOffset::magnitude() const
{
  return std::sqrt(
    offset_[0] * offset_[0] + offset_[1] * offset_[1] + offset_[2] * offset_[2]);
}

}  // namespace uav_localization
