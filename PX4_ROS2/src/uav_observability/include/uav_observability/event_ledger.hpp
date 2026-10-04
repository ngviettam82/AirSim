#ifndef UAV_OBSERVABILITY__EVENT_LEDGER_HPP_
#define UAV_OBSERVABILITY__EVENT_LEDGER_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <string>

namespace uav_observability
{

/// ROS-free severity (no diagnostic_msgs), own small type -- same rule as
/// StalenessBoard::Level.
enum class EventLevel : uint8_t
{
  kInfo,
  kWarn,
  kError,
};

/// Edge-triggered event ledger (P10 plan S:2.3, O2): observe() emits a
/// JSONL line only when a (src, field) pair's value actually CHANGED since
/// the last call. The ledger never judges WHETHER a change matters, only
/// WHETHER one happened -- it must never become a second source of truth
/// (O2: `/mission/events` stays the only on-wire event channel; this class
/// only ever formats a FILE line, never publishes).
///
/// No clock read anywhere: every "now" is a caller-supplied second count
/// (R21). R24: not internally synchronised -- observe()/needsFsync()/
/// markFsynced() must all be called from a single, non-concurrent context,
/// same discipline as the other two P10 libraries.
class EventLedger
{
public:
  explicit EventLedger(double fsync_period_sec);

  /// Returns the JSONL line to write iff new_value differs from the value
  /// last observed for (src, field); nullopt otherwise (no line, no seq
  /// spent -- repeats never cost a sequence number). The very first
  /// observation of a (src, field) pair always counts as a change (from
  /// reported as ""). Fixed key order: t_sim, t_wall, seq, src, field,
  /// from, to, level, detail.
  std::optional<std::string> observe(
    const std::string & src, const std::string & field, const std::string & new_value,
    EventLevel level, double t_sim, double t_wall,
    const std::map<std::string, std::string> & detail = {});

  /// Pure decision, no I/O (the node performs the actual fsync): true iff
  /// level >= kError, or fsync_period_sec has elapsed since the last call to
  /// markFsynced(). A caller that gets true back is expected to actually
  /// fsync and then call markFsynced(t_wall) -- this method never assumes
  /// that happened on its own.
  bool needsFsync(EventLevel level, double t_wall) const;

  /// Records that a real fsync happened at t_wall, resetting the period
  /// clock needsFsync() measures against.
  void markFsynced(double t_wall);

  /// Number of lines emitted so far == next seq value that will be used.
  uint64_t sequenceCount() const {return next_seq_;}

private:
  struct FieldKey
  {
    std::string src;
    std::string field;
    bool operator<(const FieldKey & other) const;
  };

  std::map<FieldKey, std::string> last_value_;
  uint64_t next_seq_ = 0;
  double fsync_period_sec_;
  // 0.0 default (never explicitly fsynced yet) is a deliberately safe
  // reference point: any real t_wall (wall-clock UTC epoch seconds) reads
  // as already overdue, so the very first check after construction leans
  // toward "yes, fsync" rather than silently waiting a full period first.
  double last_fsync_t_wall_ = 0.0;
};

/// Escapes a string for embedding inside a JSON string literal (", \,
/// newline, carriage return, tab, and other control characters).
std::string escapeJson(const std::string & value);

}  // namespace uav_observability

#endif  // UAV_OBSERVABILITY__EVENT_LEDGER_HPP_
