#include "uav_localization/source_selector.hpp"

#include <algorithm>

namespace uav_localization
{

namespace
{

constexpr uint8_t kNoSource = 0;

const SourceSnapshot * find(const std::vector<SourceSnapshot> & sources, uint8_t source_id)
{
  const auto match = std::find_if(
    sources.begin(), sources.end(),
    [source_id](const SourceSnapshot & source) {return source.source_id == source_id;});
  return match == sources.end() ? nullptr : &(*match);
}

/// Ties break by id so a run repeats identically.
const SourceSnapshot * best(const std::vector<SourceSnapshot> & sources)
{
  const SourceSnapshot * winner = nullptr;
  for (const SourceSnapshot & source : sources) {
    if (!source.eligible || source.position_stddev < 0.0) {
      continue;
    }
    if (winner == nullptr ||
      source.position_stddev < winner->position_stddev ||
      (source.position_stddev == winner->position_stddev &&
      source.source_id < winner->source_id))
    {
      winner = &source;
    }
  }
  return winner;
}

}  // namespace

uint8_t SourceSelector::select(const std::vector<SourceSnapshot> & sources, double now_sec)
{
  just_switched_ = false;

  const SourceSnapshot * current = find(sources, active_);
  const bool current_usable =
    active_ != kNoSource && current != nullptr && current->eligible &&
    current->position_stddev >= 0.0;

  const SourceSnapshot * challenger = best(sources);

  // Hysteresis must never delay leaving a source already known dead.
  if (!current_usable) {
    const uint8_t chosen = challenger == nullptr ? kNoSource : challenger->source_id;
    just_switched_ = chosen != kNoSource && chosen != active_;
    active_ = chosen;
    candidate_ = kNoSource;
    return active_;
  }

  if (challenger == nullptr || challenger->source_id == active_) {
    candidate_ = kNoSource;
    return active_;
  }

  const bool clearly_better =
    challenger->position_stddev < current->position_stddev * policy_.switch_margin;

  if (!clearly_better) {
    candidate_ = kNoSource;
    return active_;
  }

  if (candidate_ != challenger->source_id) {
    candidate_ = challenger->source_id;
    candidate_since_sec_ = now_sec;
    return active_;
  }

  if (now_sec - candidate_since_sec_ >= policy_.min_hold_sec) {
    active_ = challenger->source_id;
    candidate_ = kNoSource;
    just_switched_ = true;
  }
  return active_;
}

}  // namespace uav_localization
