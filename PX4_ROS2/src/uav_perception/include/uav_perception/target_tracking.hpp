// Frame-neutral core: caller converts any 3D source into TrackObservation.

#ifndef UAV_PERCEPTION__TARGET_TRACKING_HPP_
#define UAV_PERCEPTION__TARGET_TRACKING_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace uav_perception
{

enum class TrackStatus : std::uint8_t
{
  kTracking = 0,
  kCoasting = 1,
  kLost = 2,
};

/// One 3D position measurement; obstacle/detection-specific fields (size,
/// shape) are filtered by the caller before this point (see README).
struct TrackObservation
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double confidence{1.0};
  double position_stddev_m{-1.0};  // -1: unknown, falls back to default_observation_stddev_m
};

struct TrackSnapshot
{
  std::int32_t track_id{0};
  TrackStatus status{TrackStatus::kTracking};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double vx{0.0};
  double vy{0.0};
  double vz{0.0};
  double confidence{0.0};
  double position_uncertainty_m{0.0};  // 1-sigma, from KF covariance
  double age_sec{0.0};
  double time_since_seen_sec{0.0};
};

struct TrackingParams
{
  // Candidate shape filter, applied by the caller (see README ground-plane
  // sliver pitfall in obstacle_extractor_node: size.y/size.z can read ~0).
  double min_target_extent_m{0.05};
  double max_target_extent_m{5.0};

  // Applied inside update(): source-neutral, any TrackObservation qualifies.
  double min_observation_confidence{0.0};

  // Constant-velocity model, white-noise-acceleration process noise (standard
  // discretization: Q_pp=q*dt^4/4, Q_pv=q*dt^3/2, Q_vv=q*dt^2, q=accel_std^2).
  double process_noise_accel_std_m_s2{1.0};

  // Falls back when an observation's own stddev is unknown (-1); current
  // obstacle_extractor_node always reports -1 (no depth noise model yet).
  double default_observation_stddev_m{0.3};
  double initial_velocity_stddev_m_s{2.0};

  // Association gate: max(min_association_gate_m, sigma * predicted stddev).
  double association_gate_sigma{3.0};
  double min_association_gate_m{0.5};

  // Sourced from P5 plan Sec 0.1b: measured max frame gap 1.8 s, plus margin
  // for one more missed cycle before declaring the target gone.
  double lost_after_sec{3.0};

  std::size_t max_tracks{20};

  // Track confirmation (M-of-N), bug #10 (G-M4.4, 2026-08-23): a track only
  // becomes visible -- returned by update(), i.e. published -- once it has
  // matched at least confirm_hits_required times within its first
  // confirm_window_frames update() opportunities since spawn. A track that
  // never clears that bar within the window is dropped SILENTLY: nothing
  // downstream (world_model, navigator, mission) ever saw it, and no LOST is
  // ever emitted for it either -- residual 1-2 frame clutter can no longer
  // spawn a "target" that resets consumers' own lost-target timers.
  // Defaults paired with the measured ~15.6 Hz obstacle-stream cadence
  // (README P5.2): 5 opportunities is about 0.32 s, comfortably under both
  // consumer timeouts this bug exploited (navigator N2 ceiling and mission
  // TargetSeen, both 1.0 s) -- confirming a REAL target does not itself risk
  // tripping them.
  std::size_t confirm_hits_required{3};
  std::size_t confirm_window_frames{5};
};

/// Per-axis constant-velocity Kalman state: [position, velocity], 2x2 covariance.
struct AxisState
{
  double position{0.0};
  double velocity{0.0};
  double var_position{0.0};
  double var_velocity{0.0};
  double covar_pv{0.0};
};

/// F=[[1,dt],[0,1]] plus white-noise-acceleration Q. dt<=0 is a no-op (clock
/// went backward or duplicate stamp; holding state beats corrupting it).
void predictAxis(AxisState & axis, double dt, double accel_std_m_s2);

/// Position-only measurement update (H=[1,0]). measurement_variance must be > 0.
void updateAxis(AxisState & axis, double measurement, double measurement_variance);

/// Ground-plane slivers read near-zero on at least one axis (see README);
/// oversized clusters are not a single trackable object either.
bool isPlausibleTargetExtent(
  double size_x, double size_y, double size_z, const TrackingParams & params);

class MultiTargetTracker
{
public:
  explicit MultiTargetTracker(const TrackingParams & params = TrackingParams{});

  void configure(const TrackingParams & params) {params_ = params;}
  const TrackingParams & parameters() const {return params_;}

  /// Predicts every track to stamp_sec, gates + nearest-neighbor associates
  /// observations, updates matches, spawns tracks for the rest. Returns every
  /// currently active CONFIRMED track (TRACKING/COASTING) plus any confirmed
  /// track that just expired -- a track still TENTATIVE (M-of-N not yet met)
  /// is never included, confirmed or dropped, see confirm_hits_required.
  std::vector<TrackSnapshot> update(
    const std::vector<TrackObservation> & observations, double stamp_sec);

  /// Timeout sweep independent of new observations (source went silent
  /// entirely). Only finalizes overdue tracks; does not touch survivors.
  std::vector<TrackSnapshot> ageOut(double now_sec);

  /// Frame changed under our feet: mixing positions across frames would
  /// silently corrupt every track (see README, mirrors marker_detector's
  /// no-frame-mixing rule). Caller detects the frame_id change.
  void reset();

  std::size_t activeTrackCount() const {return tracks_.size();}

private:
  struct Track
  {
    std::int32_t id{0};
    AxisState x, y, z;
    double last_predicted_stamp_sec{0.0};
    double last_matched_stamp_sec{0.0};
    double first_seen_stamp_sec{0.0};
    double last_matched_confidence{0.0};
    // M-of-N confirmation bookkeeping, see TrackingParams::confirm_hits_required.
    std::size_t hits{0};
    std::size_t opportunities{0};
    bool confirmed{false};
  };

  double varianceFor(const TrackObservation & observation) const;
  void predictTo(Track & track, double stamp_sec);
  Track spawnTrack(const TrackObservation & observation, double stamp_sec);
  TrackSnapshot snapshotOf(const Track & track, TrackStatus status, double now_sec) const;
  std::vector<TrackSnapshot> expireOverdue(double now_sec);

  TrackingParams params_{};
  std::vector<Track> tracks_;
  std::int32_t next_track_id_{1};
};

}  // namespace uav_perception

#endif  // UAV_PERCEPTION__TARGET_TRACKING_HPP_
