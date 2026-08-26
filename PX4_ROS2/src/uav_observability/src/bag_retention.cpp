#include "uav_observability/bag_retention.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace uav_observability
{

namespace
{

void require(bool condition, const std::string & detail)
{
  if (!condition) {
    throw std::invalid_argument("bag retention budget: " + detail);
  }
}

// C1(b): lexical-only (no syscall, no disk touch -- still honours this
// library's "never touches disk itself" rule) so a caller-built active_path
// with a doubled separator (e.g. bag_root_ had a trailing '/') still
// compares equal to the SAME directory as reported by a real directory
// iterator. Without this, the actively-recording bag could fail to match
// its own active_path and become a normal deletion candidate.
std::string normalizedPath(const std::string & path)
{
  return std::filesystem::path(path).lexically_normal().string();
}

// C1(d): mtime that could not be measured (mtime_ec at the call site) is
// encoded as NaN, never 0.0 (epoch 1970 would sort as "oldest" and be
// deleted FIRST -- exactly backwards: "cannot tell how old" must never
// out-prioritize a bag whose age IS known). NaN entries sort to the END
// (least eager to delete), matching staleness_board's own R30 discipline.
// A plain `<` is not a strict weak ordering once NaN is possible (UB in
// std::stable_sort) -- this comparator explicitly defines the NaN case.
bool olderFirst(const BagDirInfo & a, const BagDirInfo & b)
{
  const bool a_unmeasurable = std::isnan(a.mtime_wall_sec);
  const bool b_unmeasurable = std::isnan(b.mtime_wall_sec);
  if (a_unmeasurable != b_unmeasurable) {
    return b_unmeasurable;   // the measurable one of the pair sorts first
  }
  if (a_unmeasurable) {
    return false;   // both unmeasurable -- no defined order between them
  }
  return a.mtime_wall_sec < b.mtime_wall_sec;
}

}  // namespace

void validateBudget(const RetentionBudget & budget)
{
  require(
    std::isfinite(budget.max_total_bytes) && budget.max_total_bytes >= 0.0,
    "max_total_bytes must be finite and non-negative");
  require(
    std::isfinite(budget.min_free_bytes) && budget.min_free_bytes >= 0.0,
    "min_free_bytes must be finite and non-negative");
}

RetentionPlan plan(
  const std::vector<BagDirInfo> & bags,
  const std::string & active_path,
  const RetentionBudget & budget,
  double disk_free_bytes)
{
  RetentionPlan result;

  double total_bytes = 0.0;
  for (const auto & bag : bags) {
    total_bytes += bag.bytes;
  }

  std::vector<BagDirInfo> candidates;
  candidates.reserve(bags.size());
  const std::string normalized_active = normalizedPath(active_path);
  for (const auto & bag : bags) {
    if (normalizedPath(bag.path) != normalized_active) {
      candidates.push_back(bag);
    }
  }
  // Oldest mtime first (NaN -- unmeasurable -- sorts last, C1(d)); stable so
  // equal-mtime ties keep input order.
  std::stable_sort(candidates.begin(), candidates.end(), olderFirst);

  double free_bytes = disk_free_bytes;
  auto overBudget = [&]() {
      return total_bytes > budget.max_total_bytes || free_bytes < budget.min_free_bytes;
    };

  for (const auto & candidate : candidates) {
    if (!overBudget()) {
      break;
    }
    result.delete_paths.push_back(candidate.path);
    total_bytes -= candidate.bytes;
    free_bytes += candidate.bytes;
  }

  result.must_stop_recording = overBudget();
  return result;
}

}  // namespace uav_observability
