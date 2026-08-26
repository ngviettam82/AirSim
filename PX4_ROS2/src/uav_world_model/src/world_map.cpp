#include "uav_world_model/world_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace uav_world_model
{

namespace
{

constexpr double kSameBatchTolerance = 1.0e-6;
constexpr double kAgeTieTolerance = 1.0e-6;

double ageOf(double now_sec, double stamp_sec)
{
  const double age = now_sec - stamp_sec;
  return std::isfinite(age) && age > 0.0 ? age : 0.0;
}

// Sighting age: tracker age plus transport age.
double sightingAge(const TargetObservation & target, double now_sec)
{
  const double reported = target.time_since_seen_sec;
  const double when_stamped = std::isfinite(reported) && reported > 0.0 ? reported : 0.0;
  return when_stamped + ageOf(now_sec, target.stamp_sec);
}

}  // namespace

WorldMap::WorldMap(WorldMapOptions options)
: options_(std::move(options))
{
}

void WorldMap::observeLandmark(const LandmarkObservation & observation)
{
  const LandmarkKey key{observation.family, observation.marker_id};
  const auto existing = landmarks_.find(key);
  // Not averaged: their errors share one drifting pose.
  if (existing != landmarks_.end() && existing->second.stamp_sec > observation.stamp_sec) {
    return;
  }
  landmarks_[key] = observation;
}

void WorldMap::observeObstacles(const ObstacleBatch & batch)
{
  has_obstacle_source_ = true;
  newest_obstacle_stamp_ = std::max(newest_obstacle_stamp_, batch.stamp_sec);

  const auto existing = obstacle_batches_.find(batch.sensing_frame);
  if (existing != obstacle_batches_.end() && existing->second.stamp_sec > batch.stamp_sec) {
    return;
  }
  obstacle_batches_[batch.sensing_frame] = batch;
}

// Sticky one-track selection; policy in interface contract 2.13.
void WorldMap::observeTarget(const TargetObservation & observation)
{
  if (!has_target_) {
    selectTargetTrack(observation);
    return;
  }

  if (observation.track_id == target_.track_id) {
    if (observation.stamp_sec < target_.stamp_sec) {
      return;
    }
    target_ = observation;
    return;
  }

  if (!targetSelectionIsOpen(observation) || !prefersCandidateTargetTrack(observation)) {
    ++unselected_target_tracks_;
    return;
  }

  ++target_track_switches_;
  selectTargetTrack(observation);
}

void WorldMap::selectTargetTrack(const TargetObservation & observation)
{
  target_ = observation;
  has_target_ = true;
  target_selection_stamp_sec_ = observation.stamp_sec;
}

bool WorldMap::targetSelectionIsOpen(const TargetObservation & candidate) const
{
  if (target_.status == kTargetStatusLost) {
    return true;
  }
  // Same stamp: the batch that chose is still arriving.
  if (std::abs(candidate.stamp_sec - target_selection_stamp_sec_) <= kSameBatchTolerance) {
    return true;
  }
  return candidate.stamp_sec - target_.stamp_sec > options_.target_track_switch_after_sec;
}

bool WorldMap::prefersCandidateTargetTrack(const TargetObservation & candidate) const
{
  const bool candidate_is_lost = candidate.status == kTargetStatusLost;
  const bool held_is_lost = target_.status == kTargetStatusLost;
  if (candidate_is_lost != held_is_lost) {
    return held_is_lost;
  }

  const double candidate_age = sightingAge(candidate, candidate.stamp_sec);
  const double held_age = sightingAge(target_, candidate.stamp_sec);
  if (std::abs(candidate_age - held_age) > kAgeTieTolerance) {
    return candidate_age < held_age;
  }
  return candidate.track_id < target_.track_id;
}

void WorldMap::expire(double now_sec)
{
  for (auto entry = landmarks_.begin(); entry != landmarks_.end(); ) {
    if (ageOf(now_sec, entry->second.stamp_sec) > options_.landmark_forget_sec) {
      entry = landmarks_.erase(entry);
    } else {
      ++entry;
    }
  }

  for (auto entry = obstacle_batches_.begin(); entry != obstacle_batches_.end(); ) {
    if (ageOf(now_sec, entry->second.stamp_sec) > options_.obstacle_forget_sec) {
      entry = obstacle_batches_.erase(entry);
    } else {
      ++entry;
    }
  }

  if (has_target_ && sightingAge(target_, now_sec) > options_.target_forget_sec) {
    has_target_ = false;
  }
}

std::vector<ReportedLandmark> WorldMap::landmarks(double now_sec) const
{
  std::vector<ReportedLandmark> reported;
  reported.reserve(landmarks_.size());

  for (const auto & entry : landmarks_) {
    const double age = ageOf(now_sec, entry.second.stamp_sec);
    ReportedLandmark landmark;
    landmark.marker_id = entry.second.marker_id;
    landmark.family = entry.second.family;
    landmark.position = entry.second.position;
    landmark.orientation = entry.second.orientation;
    landmark.time_since_seen_sec = age;
    landmark.position_uncertainty_m = combineQuadrature(
      {entry.second.position_uncertainty_m, driftGrowth(age, options_.drift)});
    reported.push_back(landmark);
  }
  return reported;
}

std::vector<ReportedObstacle> WorldMap::obstacles(double now_sec) const
{
  std::vector<ReportedObstacle> reported;

  for (const auto & entry : obstacle_batches_) {
    const double age = ageOf(now_sec, entry.second.stamp_sec);
    const double growth = driftGrowth(age, options_.drift);
    for (const auto & obstacle : entry.second.obstacles) {
      ReportedObstacle item;
      item.observation = obstacle;
      item.observation.position_uncertainty_m =
        combineQuadrature({obstacle.position_uncertainty_m, growth});
      item.time_since_seen_sec = age;
      reported.push_back(item);
    }
  }
  return reported;
}

ReportedTarget WorldMap::target(double now_sec) const
{
  ReportedTarget reported;
  if (!has_target_) {
    return reported;
  }
  const double age = sightingAge(target_, now_sec);
  reported.valid = true;
  reported.observation = target_;
  reported.time_since_seen_sec = age;
  reported.position_uncertainty_m = combineQuadrature(
    {target_.position_uncertainty_m, driftGrowth(age, options_.drift)});
  return reported;
}

bool WorldMap::hasObstacleSource() const
{
  return has_obstacle_source_;
}

bool WorldMap::hasSurvivingBatch() const
{
  return !obstacle_batches_.empty();
}

double WorldMap::newestObstacleStamp() const
{
  return newest_obstacle_stamp_;
}

double WorldMap::oldestObstacleStamp() const
{
  double oldest = std::numeric_limits<double>::max();
  for (const auto & entry : obstacle_batches_) {
    oldest = std::min(oldest, entry.second.stamp_sec);
  }
  return obstacle_batches_.empty() ? 0.0 : oldest;
}

double WorldMap::obstacleSensingRange() const
{
  double range = std::numeric_limits<double>::max();
  for (const auto & entry : obstacle_batches_) {
    range = std::min(range, entry.second.sensing_range_m);
  }
  return obstacle_batches_.empty() ? 0.0 : range;
}

std::size_t WorldMap::landmarkCount() const
{
  return landmarks_.size();
}

uint64_t WorldMap::targetTrackSwitchCount() const
{
  return target_track_switches_;
}

uint64_t WorldMap::unselectedTargetTrackCount() const
{
  return unselected_target_tracks_;
}

}  // namespace uav_world_model
