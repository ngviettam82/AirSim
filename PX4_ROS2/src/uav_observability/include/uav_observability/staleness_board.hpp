#ifndef UAV_OBSERVABILITY__STALENESS_BOARD_HPP_
#define UAV_OBSERVABILITY__STALENESS_BOARD_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace uav_observability
{

/// ROS-free severity (no diagnostic_msgs), same rule as failsafe_policy's
/// ViolationStatus. A verdict past its timeout reports the SEVERITY
/// configured for that item at addItem() (kWarn/kError) directly as its
/// level -- "stale" itself is conveyed by age_sec exceeding timeout_sec,
/// not by a separate enum literal (P10.1 handoff: chosen reading of plan
/// S:2.2 "STALE voi severity = level_when_missing" -- the reported level
/// IS the severity; diagnostics_node's go/no-go only ever needs to bucket
/// UNKNOWN/OK/WARN/ERROR, per plan S:2.2's KeyValue list).
enum class Level : uint8_t
{
  kUnknown,
  kOk,
  kWarn,
  kError,
};

struct ItemVerdict
{
  std::string name;
  Level level = Level::kUnknown;
  double age_sec = 0.0;      // NaN: never received, or this tick's clock regressed (R30)
  double measured_hz = 0.0;  // NaN: fewer than 2 samples seen yet
};

/// Per-source freshness board (P10 plan O4/R32: every sample-and-hold
/// reading must carry an age). Pure timing -- never inspects message
/// content, same division of labour as AuthorityArbiter's raw-vs-valid
/// arrival clocks. No clock read anywhere: every "now" is a caller-supplied
/// second count (R21).
///
/// R24: not internally synchronised -- addItem/onArrival/evaluate must all
/// be called from a single, non-concurrent context, same discipline as
/// FailsafePolicy and AuthorityArbiter (the future node's ONE
/// MutuallyExclusive group is what guarantees that, not this class).
class StalenessBoard
{
public:
  /// Throws std::invalid_argument: duplicate name, expected_hz not finite
  /// positive, or timeout_sec < 2/expected_hz (V-O1: a freshness grace must
  /// outlive >= 2 sampling periods of the signal it grades, same B1-class
  /// rule as failsafe_policy's obstacle_map_timeout_copy_sec). Also throws
  /// if level_when_missing is not kWarn or kError.
  void addItem(
    const std::string & name, double expected_hz, double timeout_sec, Level level_when_missing);

  /// Records one arrival of `name` at now_sec. Throws std::invalid_argument
  /// if `name` was never registered via addItem() (fail loud, not silent).
  /// If now_sec is behind this item's own last recorded arrival (clock
  /// regression), the arrival is rejected as data (last_arrival_sec is left
  /// untouched) and counted once via clockRegressions() -- same "reject the
  /// corrupt sample, never let it silently roll the clock backward"
  /// discipline as AuthorityArbiter's last_valid_arrival_. A NaN now_sec is
  /// also rejected as data (R27-2: never let it poison min_gap_sec/
  /// last_arrival_sec via an always-false `<` comparison), but is NOT itself
  /// counted as a clock regression -- a different failure mode.
  void onArrival(const std::string & name, double now_sec);

  /// One verdict per registered item, in addItem() call order. A tick where
  /// now_sec is itself behind an item's last_arrival_sec (evaluate() called
  /// on a clock that has regressed since the arrival was recorded) yields
  /// UNKNOWN for that item and counts once via clockRegressions() -- never a
  /// negative age, never the previous tick's stale verdict held over. A NaN
  /// now_sec also yields UNKNOWN for every item (R27-2), without touching
  /// clockRegressions().
  std::vector<ItemVerdict> evaluate(double now_sec) const;

  /// Total count of onArrival()/evaluate() calls that detected now_sec
  /// behind the relevant last_arrival_sec, across all items. Never resets,
  /// never decrements (R30: "cannot measure" is itself evidence, not noise
  /// to be forgotten).
  unsigned clockRegressions() const {return clock_regressions_;}

private:
  struct Item
  {
    std::string name;
    double expected_hz = 0.0;
    double timeout_sec = 0.0;
    Level level_when_missing = Level::kWarn;
    bool received = false;
    double last_arrival_sec = 0.0;
    // Smallest gap ever seen between two consecutive arrivals -- same
    // "infer rate from the tightest observed gap" convention as
    // camera_health_node, so measured_hz needs no a-priori expected rate to
    // notice a dropped sample (a dropped one only ever WIDENS a gap, never
    // narrows it below the true minimum).
    double min_gap_sec = 0.0;
    bool has_two_samples = false;
  };

  // mutable: evaluate() is logically const (an observer, no state a caller
  // can retroactively see differently), but a regression detected purely by
  // READING now_sec against last_arrival_sec still needs the diagnostic
  // counter above to advance -- same "counter is diagnostics, not
  // arbitration state" split AuthorityArbiter's own counters use.
  mutable unsigned clock_regressions_ = 0;
  std::vector<Item> items_;
};

}  // namespace uav_observability

#endif  // UAV_OBSERVABILITY__STALENESS_BOARD_HPP_
