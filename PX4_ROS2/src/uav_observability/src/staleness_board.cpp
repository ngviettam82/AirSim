#include "uav_observability/staleness_board.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace uav_observability
{

namespace
{

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

void require(bool condition, const std::string & detail)
{
  if (!condition) {
    throw std::invalid_argument("staleness board: " + detail);
  }
}

}  // namespace

void StalenessBoard::addItem(
  const std::string & name, double expected_hz, double timeout_sec, Level level_when_missing)
{
  require(
    std::none_of(
      items_.begin(), items_.end(), [&](const Item & item) {return item.name == name;}),
    "duplicate item name: " + name);
  require(std::isfinite(expected_hz) && expected_hz > 0.0, "expected_hz must be finite and positive: " + name);
  require(
    std::isfinite(timeout_sec) && timeout_sec >= 2.0 / expected_hz,
    "timeout_sec must be >= 2/expected_hz (V-O1): " + name);
  require(
    level_when_missing == Level::kWarn || level_when_missing == Level::kError,
    "level_when_missing must be kWarn or kError: " + name);

  Item item;
  item.name = name;
  item.expected_hz = expected_hz;
  item.timeout_sec = timeout_sec;
  item.level_when_missing = level_when_missing;
  items_.push_back(item);
}

void StalenessBoard::onArrival(const std::string & name, double now_sec)
{
  auto it = std::find_if(
    items_.begin(), items_.end(), [&](const Item & item) {return item.name == name;});
  require(it != items_.end(), "onArrival on unregistered item: " + name);

  // R27-2: a NaN "now" must never be compared with < (always false) and
  // fall through into updating last_arrival_sec/min_gap_sec with NaN,
  // poisoning every later evaluate() on this item. Ignored outright, no
  // state change -- distinct from a genuine backward-clock regression.
  if (std::isnan(now_sec)) {
    return;
  }

  if (it->received && now_sec < it->last_arrival_sec) {
    // Reject the corrupt sample entirely -- never let a backward-clock
    // arrival roll last_arrival_sec (or min_gap_sec) backward.
    ++clock_regressions_;
    return;
  }

  if (it->received) {
    const double gap = now_sec - it->last_arrival_sec;
    // VÀNG#1: two arrivals sharing the SAME stamp (gap==0.0 -- coarse clock
    // resolution, or two publishes inside one sim tick) must never enter
    // min_gap_sec -- 1.0/0.0 would report measured_hz=+inf ("infinitely
    // healthy"), the exact family of bug this board exists to prevent
    // (R30). Same gap<=0.0 guard as camera_health_checks.cpp's record().
    if (gap > 0.0) {
      it->min_gap_sec = it->has_two_samples ? std::min(it->min_gap_sec, gap) : gap;
      it->has_two_samples = true;
    }
  }
  it->last_arrival_sec = now_sec;
  it->received = true;
}

std::vector<ItemVerdict> StalenessBoard::evaluate(double now_sec) const
{
  std::vector<ItemVerdict> verdicts;
  verdicts.reserve(items_.size());

  for (const auto & item : items_) {
    ItemVerdict verdict;
    verdict.name = item.name;

    if (!item.received) {
      verdict.level = Level::kUnknown;
      verdict.age_sec = kNan;
      verdict.measured_hz = kNan;
      verdicts.push_back(verdict);
      continue;
    }

    // R27-2: a NaN "now" must never be compared with <= (always false) and
    // silently fall into "stale" below -- this is "cannot measure", not "a
    // measured, over-threshold age". Distinct from a genuine backward-clock
    // regression: it does not advance clockRegressions().
    if (std::isnan(now_sec)) {
      verdict.level = Level::kUnknown;
      verdict.age_sec = kNan;
      verdict.measured_hz = kNan;
      verdicts.push_back(verdict);
      continue;
    }

    if (now_sec < item.last_arrival_sec) {
      // Clock regressed since the arrival was recorded -- never report the
      // held-over previous verdict or a negative age (R30).
      ++clock_regressions_;
      verdict.level = Level::kUnknown;
      verdict.age_sec = kNan;
      verdict.measured_hz = kNan;
      verdicts.push_back(verdict);
      continue;
    }

    const double age = now_sec - item.last_arrival_sec;
    verdict.age_sec = age;
    verdict.level = (age <= item.timeout_sec) ? Level::kOk : item.level_when_missing;
    // VÀNG#1 belt-and-suspenders: min_gap_sec<=0.0 can only happen if a
    // future edit re-introduces the zero-gap bug at the write site above --
    // guard here too so measured_hz can never come back as +inf/nonsensical.
    verdict.measured_hz =
      (item.has_two_samples && item.min_gap_sec > 0.0) ? (1.0 / item.min_gap_sec) : kNan;
    verdicts.push_back(verdict);
  }

  return verdicts;
}

}  // namespace uav_observability
