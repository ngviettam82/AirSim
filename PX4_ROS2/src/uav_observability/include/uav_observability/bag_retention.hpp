#ifndef UAV_OBSERVABILITY__BAG_RETENTION_HPP_
#define UAV_OBSERVABILITY__BAG_RETENTION_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace uav_observability
{

/// One on-disk bag directory (mcap/sqlite3 both write a directory, never a
/// loose file -- CLAUDE.md S:2.1). Caller (rosbag_manager_node, P10.2) reads
/// these off the real filesystem; this library never touches disk itself,
/// same "pure arithmetic pinned by a test" discipline as uav_safety's
/// failsafe_policy and uav_control_authority's authority_arbiter.
struct BagDirInfo
{
  std::string path;
  double bytes = 0.0;
  // Directory's own wall-clock mtime (S:2.1: bag NAME is wall clock). NaN
  // means "could not be measured" (mtime_ec at the call site) -- plan()
  // sorts a NaN entry LAST (least eager to delete), never treats it as
  // epoch-1970-oldest (R30: "cannot tell how old" must not read as "delete
  // this one first").
  double mtime_wall_sec = 0.0;
};

struct RetentionBudget
{
  double max_total_bytes = 0.0;
  double min_free_bytes = 0.0;
};

struct RetentionPlan
{
  std::vector<std::string> delete_paths;
  // Every candidate consumed and the budget is still violated -- the node
  // must stop recording rather than silently keep writing past the floor
  // (P10 plan S:2.1's DISABLED_NO_SPACE path, R0: no silent degrade).
  bool must_stop_recording = false;
};

/// Throws std::invalid_argument if the budget is unusable -- refuse to
/// start, same rule as FailsafePolicy/AuthorityArbiter's own validate().
/// Deliberately NOT called from plan() itself: a bad budget must fail on
/// the ground (node construction), never mid-flight inside a retention tick.
void validateBudget(const RetentionBudget & budget);

/// Decide which OLD bag directories to delete so the total stays under
/// max_total_bytes and disk_free_bytes stays above min_free_bytes.
/// active_path is NEVER a delete candidate, regardless of its size/age --
/// it is the bag currently being written by rosbag2::Writer. Both active_path
/// and every bag.path are compared lexically-normalized (C1(b)) so a caller
/// that built active_path by naive string concatenation (a doubled '/') still
/// matches the SAME directory a real filesystem iterator reports. Deletes
/// oldest mtime first (NaN mtime -- unmeasurable -- sorts last, never treated
/// as epoch-1970-oldest, C1(d)), one whole directory at a time, stopping as
/// soon as the budget is satisfied; if every non-active candidate is gone and
/// the budget is still violated, must_stop_recording is set (never silent).
/// disk_free_bytes non-finite (caller could not measure it, e.g. fs::space()
/// erroring) reads as "cannot measure" (R30), never as "0 bytes free": the
/// free-space criterion contributes nothing to overBudget() in that case
/// (a NaN comparison is always false), while max_total_bytes -- independently
/// measured from bag.bytes -- still gates normally (C1(a)).
RetentionPlan plan(
  const std::vector<BagDirInfo> & bags,
  const std::string & active_path,
  const RetentionBudget & budget,
  double disk_free_bytes);

}  // namespace uav_observability

#endif  // UAV_OBSERVABILITY__BAG_RETENTION_HPP_
