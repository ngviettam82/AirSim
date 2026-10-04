#include "uav_perception/target_tracking.hpp"

#include <algorithm>
#include <cmath>

namespace uav_perception
{

void predictAxis(AxisState & axis, double dt, double accel_std_m_s2)
{
  if (!(dt > 0.0)) {
    return;
  }
  const double q = accel_std_m_s2 * accel_std_m_s2;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt3 * dt;
  const double q_pp = q * dt4 / 4.0;
  const double q_pv = q * dt3 / 2.0;
  const double q_vv = q * dt2;

  const double new_var_position =
    axis.var_position + 2.0 * dt * axis.covar_pv + dt2 * axis.var_velocity + q_pp;
  const double new_covar_pv = axis.covar_pv + dt * axis.var_velocity + q_pv;
  const double new_var_velocity = axis.var_velocity + q_vv;

  axis.position += dt * axis.velocity;
  // velocity unchanged: constant-velocity model.
  axis.var_position = new_var_position;
  axis.covar_pv = new_covar_pv;
  axis.var_velocity = new_var_velocity;
}

void updateAxis(AxisState & axis, double measurement, double measurement_variance)
{
  const double innovation = measurement - axis.position;
  const double innovation_covariance = axis.var_position + measurement_variance;
  const double gain_position = axis.var_position / innovation_covariance;
  const double gain_velocity = axis.covar_pv / innovation_covariance;

  axis.position += gain_position * innovation;
  axis.velocity += gain_velocity * innovation;
  axis.var_velocity -= gain_velocity * axis.covar_pv;
  axis.var_position = (1.0 - gain_position) * axis.var_position;
  axis.covar_pv = (1.0 - gain_position) * axis.covar_pv;
}

bool isPlausibleTargetExtent(
  double size_x, double size_y, double size_z, const TrackingParams & params)
{
  const double largest = std::max({size_x, size_y, size_z});
  const double smallest = std::min({size_x, size_y, size_z});
  if (largest > params.max_target_extent_m) {
    return false;
  }
  // Ground slivers at grazing angle: near-zero on the thin axis (see README).
  if (smallest < params.min_target_extent_m) {
    return false;
  }
  return true;
}

MultiTargetTracker::MultiTargetTracker(const TrackingParams & params)
: params_(params)
{
}

double MultiTargetTracker::varianceFor(const TrackObservation & observation) const
{
  const double stddev = observation.position_stddev_m > 0.0 ?
    observation.position_stddev_m : params_.default_observation_stddev_m;
  return stddev * stddev;
}

void MultiTargetTracker::predictTo(Track & track, double stamp_sec)
{
  const double dt = stamp_sec - track.last_predicted_stamp_sec;
  if (dt <= 0.0) {
    // Backward/duplicate stamp: ignore entirely (mirrors camera_health_checks
    // ignoring backward timestamps) so a clock reset cannot corrupt dt on the
    // next genuinely-forward call.
    return;
  }
  predictAxis(track.x, dt, params_.process_noise_accel_std_m_s2);
  predictAxis(track.y, dt, params_.process_noise_accel_std_m_s2);
  predictAxis(track.z, dt, params_.process_noise_accel_std_m_s2);
  track.last_predicted_stamp_sec = stamp_sec;
}

MultiTargetTracker::Track MultiTargetTracker::spawnTrack(
  const TrackObservation & observation, double stamp_sec)
{
  Track track;
  track.id = next_track_id_++;
  const double position_var = varianceFor(observation);
  const double velocity_var =
    params_.initial_velocity_stddev_m_s * params_.initial_velocity_stddev_m_s;

  const auto initAxis = [&](AxisState & axis, double position) {
      axis.position = position;
      axis.velocity = 0.0;
      axis.var_position = position_var;
      axis.var_velocity = velocity_var;
      axis.covar_pv = 0.0;
    };
  initAxis(track.x, observation.x);
  initAxis(track.y, observation.y);
  initAxis(track.z, observation.z);

  track.last_predicted_stamp_sec = stamp_sec;
  track.last_matched_stamp_sec = stamp_sec;
  track.first_seen_stamp_sec = stamp_sec;
  track.last_matched_confidence = observation.confidence;

  // Spawning IS the first hit/opportunity (a track only ever spawns from an
  // observation it just matched) -- see TrackingParams::confirm_hits_required.
  track.hits = 1;
  track.opportunities = 1;
  track.confirmed = track.hits >= params_.confirm_hits_required;
  return track;
}

TrackSnapshot MultiTargetTracker::snapshotOf(
  const Track & track, TrackStatus status, double now_sec) const
{
  TrackSnapshot snapshot;
  snapshot.track_id = track.id;
  snapshot.status = status;
  snapshot.x = track.x.position;
  snapshot.y = track.y.position;
  snapshot.z = track.z.position;
  snapshot.vx = track.x.velocity;
  snapshot.vy = track.y.velocity;
  snapshot.vz = track.z.velocity;
  snapshot.age_sec = std::max(0.0, now_sec - track.first_seen_stamp_sec);
  snapshot.time_since_seen_sec = std::max(0.0, now_sec - track.last_matched_stamp_sec);

  const double variance_sum =
    track.x.var_position + track.y.var_position + track.z.var_position;
  snapshot.position_uncertainty_m = std::sqrt(std::max(0.0, variance_sum) / 3.0);

  const double freshness = params_.lost_after_sec > 0.0 ?
    std::clamp(1.0 - snapshot.time_since_seen_sec / params_.lost_after_sec, 0.0, 1.0) :
    0.0;
  snapshot.confidence = track.last_matched_confidence * freshness;
  return snapshot;
}

std::vector<TrackSnapshot> MultiTargetTracker::expireOverdue(double now_sec)
{
  std::vector<TrackSnapshot> expired;
  std::vector<Track> survivors;
  survivors.reserve(tracks_.size());
  for (const auto & track : tracks_) {
    if (!track.confirmed) {
      // TENTATIVE lifecycle is opportunity-count driven (see update()), not
      // time driven, and it never emits a LOST either -- nothing was ever
      // published for it, so there is nothing to declare lost (see README).
      survivors.push_back(track);
      continue;
    }
    const double time_since_seen = now_sec - track.last_matched_stamp_sec;
    if (time_since_seen > params_.lost_after_sec) {
      expired.push_back(snapshotOf(track, TrackStatus::kLost, now_sec));
    } else {
      survivors.push_back(track);
    }
  }
  tracks_ = std::move(survivors);
  return expired;
}

std::vector<TrackSnapshot> MultiTargetTracker::ageOut(double now_sec)
{
  return expireOverdue(now_sec);
}

void MultiTargetTracker::reset()
{
  tracks_.clear();
}

std::vector<TrackSnapshot> MultiTargetTracker::update(
  const std::vector<TrackObservation> & observations, double stamp_sec)
{
  std::vector<TrackObservation> accepted;
  accepted.reserve(observations.size());
  for (const auto & observation : observations) {
    if (observation.confidence >= params_.min_observation_confidence) {
      accepted.push_back(observation);
    }
  }

  for (auto & track : tracks_) {
    predictTo(track, stamp_sec);
  }

  struct Candidate
  {
    std::size_t track_index;
    std::size_t observation_index;
    double distance;
  };
  std::vector<Candidate> candidates;
  for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {
    const double predicted_variance_sum =
      tracks_[ti].x.var_position + tracks_[ti].y.var_position + tracks_[ti].z.var_position;
    const double gate = std::max(
      params_.min_association_gate_m,
      params_.association_gate_sigma * std::sqrt(std::max(0.0, predicted_variance_sum) / 3.0));

    for (std::size_t oi = 0; oi < accepted.size(); ++oi) {
      const double dx = accepted[oi].x - tracks_[ti].x.position;
      const double dy = accepted[oi].y - tracks_[ti].y.position;
      const double dz = accepted[oi].z - tracks_[ti].z.position;
      const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (distance <= gate) {
        candidates.push_back({ti, oi, distance});
      }
    }
  }
  std::sort(
    candidates.begin(), candidates.end(),
    [](const Candidate & a, const Candidate & b) {return a.distance < b.distance;});

  std::vector<bool> track_taken(tracks_.size(), false);
  std::vector<bool> observation_taken(accepted.size(), false);
  for (const auto & candidate : candidates) {
    if (track_taken[candidate.track_index] || observation_taken[candidate.observation_index]) {
      continue;
    }
    track_taken[candidate.track_index] = true;
    observation_taken[candidate.observation_index] = true;

    Track & track = tracks_[candidate.track_index];
    const TrackObservation & observation = accepted[candidate.observation_index];
    const double measurement_variance = varianceFor(observation);
    updateAxis(track.x, observation.x, measurement_variance);
    updateAxis(track.y, observation.y, measurement_variance);
    updateAxis(track.z, observation.z, measurement_variance);
    // max(): a backward-timestamped match must never regress "last seen".
    track.last_matched_stamp_sec = std::max(track.last_matched_stamp_sec, stamp_sec);
    track.last_matched_confidence = observation.confidence;
  }

  // M-of-N confirmation bookkeeping + prune (bug #10, G-M4.4): one
  // "opportunity" per update() call for every TENTATIVE track that existed
  // before this cycle's spawns -- track_taken[] still indexes the pre-spawn
  // tracks_, so this MUST run before the spawn loop below. A track that
  // clears confirm_hits_required is latched confirmed permanently (never
  // re-evaluated); one that exhausts confirm_window_frames without doing so
  // is dropped here, silently -- it is never added to `survivors`, so it
  // never reaches expireOverdue() or the snapshot loop below and nothing
  // downstream ever learns it existed (see README/TrackingParams).
  {
    std::vector<Track> survivors;
    survivors.reserve(tracks_.size());
    for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {
      Track & track = tracks_[ti];
      if (!track.confirmed) {
        track.opportunities += 1;
        if (track_taken[ti]) {
          track.hits += 1;
        }
        if (track.hits >= params_.confirm_hits_required) {
          track.confirmed = true;
        } else if (track.opportunities >= params_.confirm_window_frames) {
          continue;  // TENTATIVE dies silently, see comment above
        }
      }
      survivors.push_back(std::move(track));
    }
    tracks_ = std::move(survivors);
  }

  for (std::size_t oi = 0; oi < accepted.size(); ++oi) {
    if (observation_taken[oi]) {
      continue;
    }
    if (tracks_.size() >= params_.max_tracks) {
      continue;
    }
    tracks_.push_back(spawnTrack(accepted[oi], stamp_sec));
  }

  auto result = expireOverdue(stamp_sec);
  for (const auto & track : tracks_) {
    if (!track.confirmed) {
      continue;  // TENTATIVE: never published, see comment above
    }
    const double time_since_seen = stamp_sec - track.last_matched_stamp_sec;
    const TrackStatus status =
      time_since_seen <= 0.0 ? TrackStatus::kTracking : TrackStatus::kCoasting;
    result.push_back(snapshotOf(track, status, stamp_sec));
  }
  return result;
}

}  // namespace uav_perception
