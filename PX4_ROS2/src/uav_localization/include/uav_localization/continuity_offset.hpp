#ifndef UAV_LOCALIZATION__CONTINUITY_OFFSET_HPP_
#define UAV_LOCALIZATION__CONTINUITY_OFFSET_HPP_

#include <array>

namespace uav_localization
{

/// Absorbs a source switch so the published pose never steps.
///
/// Design and the honesty contract it carries: see README and
/// docs/package-status.md section 5.
class ContinuityOffset
{
public:
  using Vector = std::array<double, 3>;

  /// Fades the offset toward zero, per axis.
  /// Returns true while this tick's pose still carries a moving offset --
  /// including the tick that lands on zero, which still slid the pose.
  bool advance(double seconds, double decay_rate);

  /// Pins the published pose across a switch to a source that disagrees.
  void anchor(const Vector & held, const Vector & source);

  void clear();

  bool active() const;

  double magnitude() const;

  const Vector & value() const {return offset_;}

private:
  Vector offset_{{0.0, 0.0, 0.0}};
};

/// Whether a held pose may still anchor a source that comes back.
/// A gap that cannot be measured counts as a long one.
bool anchorOutlivesBlackout(double blackout_sec, double grace_sec);

}  // namespace uav_localization

#endif  // UAV_LOCALIZATION__CONTINUITY_OFFSET_HPP_
