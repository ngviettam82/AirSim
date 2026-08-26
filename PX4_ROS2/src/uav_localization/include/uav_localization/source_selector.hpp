// Mux's safety-critical half: when switching is allowed. See README.

#ifndef UAV_LOCALIZATION__SOURCE_SELECTOR_HPP_
#define UAV_LOCALIZATION__SOURCE_SELECTOR_HPP_

#include <cstdint>
#include <vector>

namespace uav_localization
{

struct SourceSnapshot
{
  uint8_t source_id{0};
  bool eligible{false};
  double position_stddev{-1.0};
};

struct SelectionPolicy
{
  double switch_margin{0.7};    // rival stddev must fall below this fraction
  double min_hold_sec{1.0};     // and stay there this long
};

class SourceSelector
{
public:
  void configure(const SelectionPolicy & policy) {policy_ = policy;}

  /// now_sec must come from one monotonic clock across all calls.
  uint8_t select(const std::vector<SourceSnapshot> & sources, double now_sec);

  uint8_t active() const {return active_;}

  /// Lets the caller absorb the step before output.
  bool justSwitched() const {return just_switched_;}

private:
  SelectionPolicy policy_{};
  uint8_t active_{0};
  uint8_t candidate_{0};
  double candidate_since_sec_{0.0};
  bool just_switched_{false};
};

}  // namespace uav_localization

#endif  // UAV_LOCALIZATION__SOURCE_SELECTOR_HPP_
