#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>
#include <set>
#include <vector>

#include "uav_perception/ego_motion.hpp"
#include "uav_perception/target_tracking.hpp"

using uav_perception::AttitudeQuaternion;
using uav_perception::AxisState;
using uav_perception::BodyPose;
using uav_perception::isPlausibleTargetExtent;
using uav_perception::isPoseFresh;
using uav_perception::MultiTargetTracker;
using uav_perception::odomPointToOptical;
using uav_perception::odomVectorToOptical;
using uav_perception::opticalPointToOdom;
using uav_perception::predictAxis;
using uav_perception::resolveBodyPose;
using uav_perception::TrackingParams;
using uav_perception::TrackObservation;
using uav_perception::TrackStatus;
using uav_perception::updateAxis;

namespace
{

TrackObservation observationAt(double x, double y, double z)
{
  TrackObservation observation;
  observation.x = x;
  observation.y = y;
  observation.z = z;
  observation.confidence = 1.0;
  observation.position_stddev_m = -1.0;
  return observation;
}

}  // namespace

// --- Raw per-axis Kalman math: hand-computed, pins the algebra directly ---

TEST(PredictAxis, PositionAdvancesByVelocityAndVarianceGrows)
{
  AxisState axis;
  axis.position = 0.0;
  axis.velocity = 2.0;
  axis.var_position = 0.1;
  axis.var_velocity = 0.5;
  axis.covar_pv = 0.0;

  predictAxis(axis, 1.0, 1.0);  // dt=1s, accel_std=1 m/s^2 -> q=1

  EXPECT_DOUBLE_EQ(axis.position, 2.0);
  EXPECT_DOUBLE_EQ(axis.velocity, 2.0) << "constant-velocity model: unchanged by predict";
  // Q_pp=q*dt^4/4=0.25, Q_pv=q*dt^3/2=0.5, Q_vv=q*dt^2=1.0 (see header comment).
  EXPECT_NEAR(axis.var_position, 0.1 + 0.5 + 0.25, 1e-9);
  EXPECT_NEAR(axis.covar_pv, 0.5 + 0.5, 1e-9);
  EXPECT_NEAR(axis.var_velocity, 0.5 + 1.0, 1e-9);
}

TEST(PredictAxis, NonPositiveDtIsANoOp)
{
  AxisState axis;
  axis.position = 3.0;
  axis.velocity = -1.0;
  axis.var_position = 0.2;
  axis.var_velocity = 0.3;
  axis.covar_pv = 0.05;
  const AxisState before = axis;

  predictAxis(axis, 0.0, 1.0);
  EXPECT_DOUBLE_EQ(axis.position, before.position);
  EXPECT_DOUBLE_EQ(axis.var_position, before.var_position);

  predictAxis(axis, -0.5, 1.0);
  EXPECT_DOUBLE_EQ(axis.position, before.position) << "backward dt must not move state";
  EXPECT_DOUBLE_EQ(axis.var_position, before.var_position);
}

TEST(UpdateAxis, MovesPositionByKalmanGainAndShrinksVarianceOnly)
{
  AxisState axis;
  axis.position = 0.0;
  axis.velocity = 0.0;
  axis.var_position = 1.0;
  axis.var_velocity = 1.0;
  axis.covar_pv = 0.0;  // no position-velocity correlation yet

  updateAxis(axis, 2.0, 1.0);  // S=1+1=2, gain_position=0.5, gain_velocity=0

  EXPECT_DOUBLE_EQ(axis.position, 1.0);
  EXPECT_DOUBLE_EQ(axis.velocity, 0.0) <<
    "zero covar_pv: a position fix alone must not move velocity";
  EXPECT_DOUBLE_EQ(axis.var_position, 0.5);
  EXPECT_DOUBLE_EQ(axis.var_velocity, 1.0) << "velocity variance untouched with covar_pv=0";
}

// --- Candidate shape filter ---

TEST(IsPlausibleTargetExtent, RejectsSliverAndOversizedAcceptsReasonable)
{
  TrackingParams params;  // defaults: min=0.05, max=5.0
  EXPECT_FALSE(isPlausibleTargetExtent(1.0, 0.3, 0.001, params)) <<
    "ground-plane sliver: near-zero on the thin axis";
  EXPECT_FALSE(isPlausibleTargetExtent(6.0, 0.5, 0.4, params)) << "too large to be one object";
  EXPECT_TRUE(isPlausibleTargetExtent(0.4, 0.6, 0.4, params));
}

// --- MultiTargetTracker behavior (P5.5 gate acceptance tests) ---
// NOTE (bug #10, G-M4.4, 2026-08-23): default TrackingParams now requires
// M-of-N confirmation (confirm_hits_required=3 within confirm_window_frames
// =5) before a track is ever returned by update(). Tests below that are not
// specifically about the confirmation policy warm a track up to CONFIRMED
// first (2-3 extra update() calls at the same position, well before the
// timeline the original test cared about) so the behaviour they pin stays
// exactly what it was; two tests (MaxTracksCapsNewSpawnsButKeepsExisting,
// ResetClearsAllTracks) set confirm_hits_required=confirm_window_frames=1 to
// stay decoupled from the confirmation policy entirely, since what they pin
// (the max_tracks cap, reset behaviour) is orthogonal to it.

TEST(MultiTargetTracker, ConstantVelocityWithoutNoiseConvergesAndKeepsId)
{
  MultiTargetTracker tracker;
  const double true_vx = 1.0;
  const double dt = 0.5;
  double t = 0.0;
  std::int32_t seen_id = 0;
  uav_perception::TrackSnapshot last;

  for (int step = 0; step < 13; ++step) {
    const double x = true_vx * t;
    const auto snapshots = tracker.update({observationAt(x, 0.0, 3.0)}, t);
    if (step < 2) {
      // Default M/N = 3/5 (bug #10): matches 1 and 2 stay TENTATIVE.
      EXPECT_TRUE(snapshots.empty()) << "step " << step;
    } else {
      ASSERT_EQ(snapshots.size(), 1u) << "step " << step;
      if (seen_id == 0) {
        seen_id = snapshots[0].track_id;
      } else {
        EXPECT_EQ(snapshots[0].track_id, seen_id) << "step " << step;
      }
      last = snapshots[0];
    }
    t += dt;
  }

  // Scope: the feed is exact, so this pins convergence and id keeping, NOT the
  // filter's behaviour under the measurement noise it declares. That claim needs
  // the seeded test below, because vx/vy feed the local planner's standoff.
  EXPECT_NEAR(last.vx, true_vx, 0.2);
  EXPECT_NEAR(last.vy, 0.0, 0.2);
  EXPECT_EQ(last.status, TrackStatus::kTracking);
}

namespace
{

/// Worst velocity error over `trials` seeded runs of a constant-velocity target
/// carrying measurement noise of `sigma` metres.
double worstVelocityErrorUnderNoise(double sigma, int trials)
{
  const double true_vx = 1.0;
  const double dt = 0.5;
  double worst = 0.0;

  for (int trial = 0; trial < trials; ++trial) {
    std::mt19937 rng(20260818u + static_cast<unsigned>(trial));
    std::normal_distribution<double> noise(0.0, sigma);

    MultiTargetTracker tracker;
    double t = 0.0;
    uav_perception::TrackSnapshot last;
    for (int step = 0; step < 40; ++step) {
      const auto snapshots =
        tracker.update({observationAt(true_vx * t + noise(rng), noise(rng), 3.0)}, t);
      if (!snapshots.empty()) {
        last = snapshots[0];
      }
      t += dt;
    }
    worst = std::max(worst, std::abs(last.vx - true_vx));
  }
  return worst;
}

}  // namespace

TEST(MultiTargetTracker, VelocityErrorUnderTheDeclaredNoiseIsBigEnoughToPlanAround)
{
  // TargetTrack.vx/vy sets how far the local planner stands off a moving target,
  // so this is a safety-path number. Measured, not wished for: at the declared
  // 0.3 m observation noise and dt 0.5 s the worst error over 20 seeds is 0.71 m/s
  // on a target truly moving at 1.0 m/s -- the same order as the two-point bound
  // sigma*sqrt(2)/dt = 0.85. P6.4 must budget for it, not assume it away.
  // (Robust to the M-of-N confirm delay unchanged: the helper already keeps
  // only the LAST non-empty snapshot over 40 steps, so 2 tentative steps at
  // the start do not affect this measurement.)
  const double shipped = worstVelocityErrorUnderNoise(0.3, 20);
  EXPECT_LT(shipped, 0.80) << "worst |vx - true| over 20 seeds: " << shipped;

  // Control on the real mechanism: double the measurement noise and the error must
  // grow. Without it, the bound above could pass on a filter that never updated.
  const double noisier = worstVelocityErrorUnderNoise(0.6, 20);
  EXPECT_GT(noisier, shipped * 1.2)
    << "noise doubled but the estimate did not degrade: " << noisier << " vs " << shipped;

  std::cout << "[ EVIDENCE ] worst |vx| error: sigma 0.3 -> " << shipped
            << " m/s, sigma 0.6 -> " << noisier << " m/s" << std::endl;
}

TEST(MultiTargetTracker, GapUpTo1_8SecondsCoastsThenReacquiresSameId)
{
  MultiTargetTracker tracker;
  // Warm up to confirmation (default M/N = 3/5) before exercising the gap.
  tracker.update({observationAt(0.0, 0.0, 3.0)}, 0.0);
  tracker.update({observationAt(0.0, 0.0, 3.0)}, 0.3);
  const auto first = tracker.update({observationAt(0.0, 0.0, 3.0)}, 0.6);
  ASSERT_EQ(first.size(), 1u) << "3rd match must confirm";
  const auto id = first[0].track_id;
  EXPECT_EQ(first[0].status, TrackStatus::kTracking);

  // Obstacle stream keeps running but misses this target for one cycle
  // (occlusion) - well inside the measured max gap of 1.8 s (P5 plan Sec 0.1b).
  const auto mid = tracker.update({}, 1.5);
  ASSERT_EQ(mid.size(), 1u);
  EXPECT_EQ(mid[0].track_id, id);
  EXPECT_EQ(mid[0].status, TrackStatus::kCoasting);

  // 1.8 s after the last real match (0.6 -> 2.4), same as the original intent.
  const auto reacquired = tracker.update({observationAt(0.0, 0.0, 3.0)}, 2.4);
  ASSERT_EQ(reacquired.size(), 1u);
  EXPECT_EQ(reacquired[0].track_id, id) << "must reacquire the SAME id, not spawn a new one";
  EXPECT_EQ(reacquired[0].status, TrackStatus::kTracking);
  EXPECT_EQ(tracker.activeTrackCount(), 1u);
}

TEST(MultiTargetTracker, CrossingPathsOutsideGateDoNotSwapIds)
{
  MultiTargetTracker tracker;
  const double dt = 0.5;
  const double speed = 1.0;
  const double separation_z = 8.0;  // safely outside the association gate throughout

  // Warm up both to confirmation (default M/N = 3/5) at their stationary
  // starting positions, before either target starts moving.
  tracker.update(
    {observationAt(-3.0, 0.0, 0.0), observationAt(3.0, 0.0, separation_z)}, -1.0);
  tracker.update(
    {observationAt(-3.0, 0.0, 0.0), observationAt(3.0, 0.0, separation_z)}, -0.5);
  const auto spawn = tracker.update(
    {observationAt(-3.0, 0.0, 0.0), observationAt(3.0, 0.0, separation_z)}, 0.0);
  ASSERT_EQ(spawn.size(), 2u) << "3rd match must confirm both";
  const bool first_is_low = spawn[0].z < spawn[1].z;
  const auto low_id = first_is_low ? spawn[0].track_id : spawn[1].track_id;
  const auto high_id = first_is_low ? spawn[1].track_id : spawn[0].track_id;
  EXPECT_NE(low_id, high_id);

  double t = dt;
  for (int step = 0; step < 8; ++step) {
    // "low" moves +x, "high" moves -x: x crosses around t~3s, but z=8m
    // separation keeps them outside each other's gate the whole time.
    const double low_x = -3.0 + speed * t;
    const double high_x = 3.0 - speed * t;
    const auto snapshots = tracker.update(
      {observationAt(low_x, 0.0, 0.0), observationAt(high_x, 0.0, separation_z)}, t);
    ASSERT_EQ(snapshots.size(), 2u) << "step " << step;
    for (const auto & snapshot : snapshots) {
      if (snapshot.track_id == low_id) {
        EXPECT_NEAR(snapshot.z, 0.0, 1.0) << "low track must not have jumped to the high one";
      } else if (snapshot.track_id == high_id) {
        EXPECT_NEAR(snapshot.z, separation_z, 1.0) <<
          "high track must not have jumped to the low one";
      } else {
        ADD_FAILURE() << "unexpected track id: one must have been swapped or re-spawned";
      }
    }
    t += dt;
  }
  EXPECT_EQ(tracker.activeTrackCount(), 2u);
}

TEST(MultiTargetTracker, IrregularStampsProduceNoNaNAndStableId)
{
  MultiTargetTracker tracker;
  // Jittered dt sequence spanning the measured 0.5-1.8s frame gap band
  // (P5 plan Sec 0.1b); never assumes a fixed rate.
  const std::vector<double> dts = {0.14, 1.79, 0.31, 0.88, 1.62, 0.05, 1.21};
  const double speed = 0.7;
  double t = 0.0;
  double x = 0.0;
  std::int32_t seen_id = 0;

  for (std::size_t step = 0; step < dts.size(); ++step) {
    const auto snapshots = tracker.update({observationAt(x, 1.0, 2.0)}, t);
    if (step < 2) {
      // Default M/N = 3/5 (bug #10): matches 1 and 2 stay TENTATIVE.
      EXPECT_TRUE(snapshots.empty()) << "step " << step;
    } else {
      ASSERT_EQ(snapshots.size(), 1u) << "step " << step;
      if (seen_id == 0) {
        seen_id = snapshots[0].track_id;
      } else {
        EXPECT_EQ(snapshots[0].track_id, seen_id) << "step " << step;
      }
      EXPECT_TRUE(std::isfinite(snapshots[0].x));
      EXPECT_TRUE(std::isfinite(snapshots[0].y));
      EXPECT_TRUE(std::isfinite(snapshots[0].z));
      EXPECT_TRUE(std::isfinite(snapshots[0].vx));
      EXPECT_TRUE(std::isfinite(snapshots[0].vy));
      EXPECT_TRUE(std::isfinite(snapshots[0].vz));
      EXPECT_TRUE(std::isfinite(snapshots[0].confidence));
      EXPECT_TRUE(std::isfinite(snapshots[0].position_uncertainty_m));
      EXPECT_GT(snapshots[0].position_uncertainty_m, 0.0);
    }

    t += dts[step];
    x += speed * dts[step];
  }
}

TEST(MultiTargetTracker, DuplicateOrBackwardStampsDoNotCorruptState)
{
  MultiTargetTracker tracker;
  // Warm up to confirmation (default M/N = 3/5) before the timeline the
  // original test cared about (stamp 5.0 onward).
  tracker.update({observationAt(0.0, 0.0, 3.0)}, 3.0);
  tracker.update({observationAt(0.0, 0.0, 3.0)}, 4.0);
  const auto first = tracker.update({observationAt(0.0, 0.0, 3.0)}, 5.0);
  ASSERT_EQ(first.size(), 1u) << "3rd match must confirm";
  const auto id = first[0].track_id;

  // Duplicate stamp (dt=0) then a stamp that goes backward (dt<0): both must
  // be a no-op, mirroring camera_health_checks' "ignore backward timestamps"
  // rule (see README) - never NaN, never a corrupted covariance.
  const auto duplicate = tracker.update({observationAt(0.0, 0.0, 3.0)}, 5.0);
  ASSERT_EQ(duplicate.size(), 1u);
  EXPECT_EQ(duplicate[0].track_id, id);
  EXPECT_TRUE(std::isfinite(duplicate[0].position_uncertainty_m));

  const auto backward = tracker.update({observationAt(0.0, 0.0, 3.0)}, 4.5);
  ASSERT_EQ(backward.size(), 1u);
  EXPECT_EQ(backward[0].track_id, id);
  EXPECT_TRUE(std::isfinite(backward[0].position_uncertainty_m));
  EXPECT_GE(backward[0].position_uncertainty_m, 0.0);
  EXPECT_GE(backward[0].time_since_seen_sec, 0.0) << "clamped, never negative (see README)";
}

TEST(MultiTargetTracker, ObjectGoneForLongerThanThresholdIsDeclaredLost)
{
  TrackingParams params;
  params.lost_after_sec = 1.0;  // shrunk from the 3.0s default purely for test speed
  MultiTargetTracker tracker(params);

  // Warm up to confirmation (default M/N = 3/5) before t=0.0, the timeline
  // the original test's lost_after_sec boundaries (0.9s/1.1s) are pinned to.
  tracker.update({observationAt(0.0, 0.0, 3.0)}, -1.0);
  tracker.update({observationAt(0.0, 0.0, 3.0)}, -0.5);
  const auto first = tracker.update({observationAt(0.0, 0.0, 3.0)}, 0.0);
  ASSERT_EQ(first.size(), 1u) << "3rd match must confirm";
  const auto id = first[0].track_id;

  const auto still_coasting = tracker.update({}, 0.9);
  ASSERT_EQ(still_coasting.size(), 1u) << "0.9s < lost_after_sec=1.0s: must still be alive";
  EXPECT_EQ(still_coasting[0].status, TrackStatus::kCoasting);
  EXPECT_EQ(tracker.activeTrackCount(), 1u);

  const auto lost = tracker.update({}, 1.1);
  ASSERT_EQ(lost.size(), 1u);
  EXPECT_EQ(lost[0].track_id, id);
  EXPECT_EQ(lost[0].status, TrackStatus::kLost);
  EXPECT_EQ(tracker.activeTrackCount(), 0u) << "lost track must be dropped internally";

  const auto after = tracker.update({}, 2.0);
  EXPECT_TRUE(after.empty()) << "must not resurrect a track already declared lost";
}

TEST(MultiTargetTracker, AgeOutDeclaresLostWithoutAnyFurtherMessages)
{
  TrackingParams params;
  params.lost_after_sec = 1.0;
  MultiTargetTracker tracker(params);

  // Warm up to confirmation (default M/N = 3/5) before t=0.0.
  tracker.update({observationAt(0.0, 0.0, 3.0)}, -1.0);
  tracker.update({observationAt(0.0, 0.0, 3.0)}, -0.5);
  const auto first = tracker.update({observationAt(0.0, 0.0, 3.0)}, 0.0);
  ASSERT_EQ(first.size(), 1u) << "3rd match must confirm";
  const auto id = first[0].track_id;

  // Source stops publishing entirely; only a clock-driven sweep (ageOut,
  // called from the node's own ROS timer) can still detect this (see README).
  EXPECT_TRUE(tracker.ageOut(0.5).empty()) << "still inside lost_after_sec";
  const auto expired = tracker.ageOut(1.5);
  ASSERT_EQ(expired.size(), 1u);
  EXPECT_EQ(expired[0].track_id, id);
  EXPECT_EQ(expired[0].status, TrackStatus::kLost);
  EXPECT_EQ(tracker.activeTrackCount(), 0u);
}

TEST(MultiTargetTracker, UncertaintyIsPositiveAndGrowsWhileCoasting)
{
  MultiTargetTracker tracker;
  // Warm up to confirmation (default M/N = 3/5) before t=0.0.
  tracker.update({observationAt(0.0, 0.0, 3.0)}, -1.0);
  tracker.update({observationAt(0.0, 0.0, 3.0)}, -0.5);
  const auto first = tracker.update({observationAt(0.0, 0.0, 3.0)}, 0.0);
  ASSERT_EQ(first.size(), 1u) << "3rd match must confirm";
  EXPECT_GT(first[0].position_uncertainty_m, 0.0);

  const auto coasting_1 = tracker.update({}, 0.5);
  ASSERT_EQ(coasting_1.size(), 1u);
  EXPECT_EQ(coasting_1[0].status, TrackStatus::kCoasting);
  EXPECT_GT(coasting_1[0].position_uncertainty_m, first[0].position_uncertainty_m) <<
    "predict-only steps must grow the covariance, never shrink it";

  const auto coasting_2 = tracker.update({}, 1.5);
  ASSERT_EQ(coasting_2.size(), 1u);
  EXPECT_GT(coasting_2[0].position_uncertainty_m, coasting_1[0].position_uncertainty_m) <<
    "uncertainty must keep growing the longer coasting continues";
}

TEST(MultiTargetTracker, MaxTracksCapsNewSpawnsButKeepsExisting)
{
  TrackingParams params;
  params.max_tracks = 2;
  // Decoupled from M-of-N confirmation (tested elsewhere): confirm on the
  // first match, same as this test pinned before bug #10.
  params.confirm_hits_required = 1;
  params.confirm_window_frames = 1;
  MultiTargetTracker tracker(params);

  const auto first = tracker.update(
    {observationAt(0.0, 0.0, 0.0), observationAt(10.0, 0.0, 0.0)}, 0.0);
  ASSERT_EQ(first.size(), 2u);
  EXPECT_EQ(tracker.activeTrackCount(), 2u);

  // A third, well-separated observation must not spawn a third track once
  // max_tracks is reached (mirrors the max_obstacles cap in obstacle_extraction).
  const auto second = tracker.update(
    {observationAt(0.1, 0.0, 0.0), observationAt(10.1, 0.0, 0.0), observationAt(50.0, 0.0, 0.0)},
    0.5);
  EXPECT_EQ(tracker.activeTrackCount(), 2u);
  EXPECT_EQ(second.size(), 2u) << "still exactly the two existing tracks, no third";
}

TEST(MultiTargetTracker, ResetClearsAllTracks)
{
  TrackingParams params;
  // Decoupled from M-of-N confirmation (tested elsewhere): confirm on the
  // first match, same as this test pinned before bug #10.
  params.confirm_hits_required = 1;
  params.confirm_window_frames = 1;
  MultiTargetTracker tracker(params);

  const auto first = tracker.update({observationAt(0.0, 0.0, 0.0)}, 0.0);
  ASSERT_EQ(first.size(), 1u);
  tracker.reset();
  EXPECT_EQ(tracker.activeTrackCount(), 0u);

  // Frame changed under our feet: the next observation must spawn a fresh id,
  // never silently reuse geometry computed in the old frame.
  const auto after = tracker.update({observationAt(0.0, 0.0, 0.0)}, 1.0);
  ASSERT_EQ(after.size(), 1u);
  EXPECT_EQ(after[0].status, TrackStatus::kTracking);
}

// --- Bug #10 (G-M4.4, 2026-08-23): M-of-N track confirmation ---
//
// Real symptom (~/gate_logs/gm4d_bringup.log): target box removed from the
// world, but residual depth-extraction clutter (1-18 spurious obstacles,
// intermittent) kept spawning tracks that were published as TRACKING from
// their very FIRST matched frame. world_model's sticky selection (contract
// v0.1 S:2.13, N3) correctly never abandons a not-LOST track for a LOST one
// -- but every fresh ghost track IS not-LOST, so the moment the previously-
// selected track went silent past target_track_switch_after_sec (1.0 s),
// the newest ghost won the switch. 18 switches (track_id 4->8->2->...->40)
// with no real object present, each one resetting BOTH the navigator's N2
// "haven't seen target" ceiling and the mission's TargetSeen timer (both
// 1.0 s) -- the drone "tracked" nothing, forever.

TEST(MultiTargetTracker, SingleFrameBlobNeverPublishes)
{
  // (a) A single 1-frame blob must never surface as CONFIRMED/published,
  // and once its confirm window is exhausted it must die SILENTLY -- no
  // LOST snapshot for something nobody downstream ever saw as a target.
  MultiTargetTracker tracker;  // default M=3, N=5
  const auto once = tracker.update({observationAt(0.0, 0.0, 3.0)}, 0.0);
  EXPECT_TRUE(once.empty()) << "one match must stay TENTATIVE, unpublished";
  EXPECT_EQ(tracker.activeTrackCount(), 1u) << "held internally, just not surfaced";

  const auto after1 = tracker.update({}, 0.06);
  EXPECT_TRUE(after1.empty());
  const auto after2 = tracker.update({}, 0.12);
  EXPECT_TRUE(after2.empty());
  const auto after3 = tracker.update({}, 0.18);
  EXPECT_TRUE(after3.empty());
  const auto after4 = tracker.update({}, 0.24);
  EXPECT_TRUE(after4.empty()) << "5th opportunity: window exhausted, must drop, not LOST";
  EXPECT_EQ(tracker.activeTrackCount(), 0u) << "dropped internally too, not just unpublished";
}

TEST(MultiTargetTracker, RealTargetConfirmsAfterMMatchesWithinWindow)
{
  // (b) A real, continuously-visible target must confirm in exactly M
  // matches (the first opportunity it mathematically can), never later.
  MultiTargetTracker tracker;  // default M=3, N=5
  const auto m1 = tracker.update({observationAt(0.0, 0.0, 3.0)}, 0.0);
  EXPECT_TRUE(m1.empty()) << "1/3: still TENTATIVE";
  const auto m2 = tracker.update({observationAt(0.0, 0.0, 3.0)}, 0.06);
  EXPECT_TRUE(m2.empty()) << "2/3: still TENTATIVE";
  const auto m3 = tracker.update({observationAt(0.0, 0.0, 3.0)}, 0.12);
  ASSERT_EQ(m3.size(), 1u) << "3/3: must confirm exactly now";
  EXPECT_EQ(m3[0].status, TrackStatus::kTracking);
  EXPECT_EQ(tracker.activeTrackCount(), 1u);
}

TEST(MultiTargetTracker, RealTargetGoneNoGhostTrackFromResidualClutter)
{
  // (c) The actual bug: object removed, but intermittent 1-frame clutter
  // (each blip a DIFFERENT, unrelated spot -- true residual noise, not a
  // persistent signal) must never surface as a track, and the real (now
  // silent) track's time_since_seen must climb monotonically to LOST,
  // never reset by a ghost.
  TrackingParams params;
  params.lost_after_sec = 1.0;  // shrunk purely for test speed
  MultiTargetTracker tracker(params);

  tracker.update({observationAt(0.0, 0.0, 3.0)}, 0.0);
  tracker.update({observationAt(0.0, 0.0, 3.0)}, 0.06);
  const auto confirmed = tracker.update({observationAt(0.0, 0.0, 3.0)}, 0.12);
  ASSERT_EQ(confirmed.size(), 1u);
  const auto real_id = confirmed[0].track_id;

  double last_time_since_seen = -1.0;
  double t = 0.12;
  for (int step = 0; step < 6; ++step) {
    t += 0.06;
    std::vector<TrackObservation> observations;
    if (step % 2 == 0) {
      // A fresh, unrelated blip each time -- a different spot every time,
      // never repeating, so it can never accumulate hits on its own.
      observations.push_back(observationAt(50.0 + step * 5.0, 50.0, 50.0));
    }
    const auto snapshots = tracker.update(observations, t);
    ASSERT_EQ(snapshots.size(), 1u) << "step " << step << ": only the real track, ever";
    EXPECT_EQ(snapshots[0].track_id, real_id) <<
      "step " << step << ": no ghost track must ever be published";
    EXPECT_GE(snapshots[0].time_since_seen_sec, last_time_since_seen) <<
      "step " << step << ": must increase monotonically, never reset by clutter";
    last_time_since_seen = snapshots[0].time_since_seen_sec;
  }

  const auto lost = tracker.update({}, t + 1.1);
  ASSERT_EQ(lost.size(), 1u);
  EXPECT_EQ(lost[0].track_id, real_id);
  EXPECT_EQ(lost[0].status, TrackStatus::kLost);
}

TEST(MultiTargetTracker, ConfirmationDelayStaysWellUnderConsumerTimeouts)
{
  // (d) The fix must not itself break the pinned P5.5 detection-latency
  // budget. Default M/N (3/5) paired with the measured ~15.6 Hz obstacle
  // cadence (README P5.2): worst-case confirm latency is confirm_window_
  // frames cycles, i.e. 5/15.6 = 0.32 s -- must stay comfortably under the
  // two consumer timeouts bug #10 exploited (navigator N2 ceiling and
  // mission TargetSeen, both 1.0 s), or fixing it would just trade one
  // failure mode for a slower one.
  MultiTargetTracker tracker;
  const double frame_period_sec = 1.0 / 15.6;
  const double worst_case_confirm_latency_sec =
    static_cast<double>(tracker.parameters().confirm_window_frames) * frame_period_sec;
  EXPECT_LT(worst_case_confirm_latency_sec, 0.5) <<
    "confirm delay must leave comfortable margin under the 1.0 s consumer timeouts";

  // A REAL, continuously-visible target confirms in exactly M matches (the
  // best case, and what a healthy 15.6 Hz stream gives): measure it
  // directly rather than trust the arithmetic above alone.
  double t = 0.0;
  std::size_t step = 0;
  for (; step < 10; ++step) {
    const auto snapshots = tracker.update({observationAt(0.0, 0.0, 3.0)}, t);
    if (!snapshots.empty()) {
      break;
    }
    t += frame_period_sec;
  }
  EXPECT_EQ(step + 1, tracker.parameters().confirm_hits_required) <<
    "a continuously-matched real target must confirm in exactly M frames, not later";
  EXPECT_LT(t, 1.0) << "measured confirm time must clear the 1.0 s consumer timeout with margin";
}

// --- Ego-motion compensation (bug #10 follow-up, G-M3/G-M4, 2026-08-23) ---
//
// Real symptom: even a REAL, stationary object (box still in the world)
// churned track_id while the drone orbited it (2-5 miss-reacquire cycles per
// lap on the wire; static bench measured 0 churn). Root cause: association
// ran directly in the camera-optical frame, which moves with the vehicle --
// a stationary object's APPARENT position jumps every time the camera
// rotates or translates, easily clearing the association gate even though
// nothing in the world moved. Fix: associate/track in ODOM (ego-motion
// compensated), converting back to camera-optical only at publish time.

TEST(EgoMotion, RoundTripPointThroughOdomAndBackIsIdentity)
{
  BodyPose pose;
  pose.orientation = AttitudeQuaternion{0.0, 0.0, 0.0, 1.0};
  pose.x = 1.0;
  pose.y = -2.0;
  pose.z = 0.5;

  double wx = 0.0, wy = 0.0, wz = 0.0;
  opticalPointToOdom(0.3, -0.4, 3.0, pose, wx, wy, wz);
  double ox = 0.0, oy = 0.0, oz = 0.0;
  odomPointToOptical(wx, wy, wz, pose, ox, oy, oz);
  EXPECT_NEAR(ox, 0.3, 1e-9);
  EXPECT_NEAR(oy, -0.4, 1e-9);
  EXPECT_NEAR(oz, 3.0, 1e-9);
}

TEST(EgoMotion, IdentityPoseAtOriginIsOpticalToBodyOnly)
{
  // A level camera at the odom origin: opticalPointToOdom must reduce to
  // exactly opticalToBody (no rotation, no translation).
  const BodyPose pose;  // default: identity orientation, position (0,0,0)
  double wx = 0.0, wy = 0.0, wz = 0.0;
  opticalPointToOdom(1.0, 0.0, 0.0, pose, wx, wy, wz);
  EXPECT_NEAR(wy, -1.0, 1e-12) << "matches Frames.RightInOpticalIsNegativeYInBody";
}

TEST(EgoMotion, StationaryPointStaysConstantAsCameraTranslates)
{
  // The core invariant this whole fix relies on: an odom-fixed point's
  // OPTICAL appearance changes with camera position, but converting it back
  // to odom (given the correct pose) must always recover the same point.
  const double object_x = 5.0, object_y = 0.0, object_z = 1.0;
  for (int k = 0; k < 8; ++k) {
    const double angle = k * M_PI / 4.0;
    BodyPose pose;
    pose.orientation = AttitudeQuaternion{0.0, 0.0, 0.0, 1.0};
    pose.x = 2.0 * std::cos(angle);
    pose.y = 2.0 * std::sin(angle);
    pose.z = 0.0;

    // Independent oracle (identity rotation: body == relative-to-camera;
    // hand-written optical convention, NOT calling production bodyToOptical).
    const double rel_x = object_x - pose.x;
    const double rel_y = object_y - pose.y;
    const double rel_z = object_z - pose.z;
    const double optical_x = -rel_y;
    const double optical_y = -rel_z;
    const double optical_z = rel_x;

    double odom_x = 0.0, odom_y = 0.0, odom_z = 0.0;
    opticalPointToOdom(optical_x, optical_y, optical_z, pose, odom_x, odom_y, odom_z);
    EXPECT_NEAR(odom_x, object_x, 1e-9) << "k=" << k;
    EXPECT_NEAR(odom_y, object_y, 1e-9) << "k=" << k;
    EXPECT_NEAR(odom_z, object_z, 1e-9) << "k=" << k;
  }
}

TEST(EgoMotion, VelocityRoundTripsAndStationaryTargetReadsZero)
{
  BodyPose pose;
  pose.orientation = AttitudeQuaternion{0.0, 0.0, 0.0, 1.0};
  pose.x = 3.0;
  pose.y = 1.0;
  pose.z = 0.0;

  double vx = 0.0, vy = 0.0, vz = 0.0;
  odomVectorToOptical(0.0, 0.0, 0.0, pose, vx, vy, vz);
  EXPECT_DOUBLE_EQ(vx, 0.0);
  EXPECT_DOUBLE_EQ(vy, 0.0);
  EXPECT_DOUBLE_EQ(vz, 0.0);

  odomVectorToOptical(0.2, 0.0, 0.0, pose, vx, vy, vz);
  EXPECT_NEAR(std::sqrt(vx * vx + vy * vy + vz * vz), 0.2, 1e-12) <<
    "rotation preserves magnitude for an identity-orientation pose";
}

TEST(IsPoseFresh, BoundaryAtMaxAgeIsStillFresh)
{
  EXPECT_TRUE(isPoseFresh(0.5, 0.5));
  EXPECT_FALSE(isPoseFresh(0.51, 0.5));
  EXPECT_FALSE(isPoseFresh(-0.01, 0.5)) << "a clock going backwards is not 'very fresh'";
  EXPECT_FALSE(isPoseFresh(std::numeric_limits<double>::quiet_NaN(), 0.5));
}

TEST(ResolveBodyPose, NoOdometryYetFallsBack)
{
  BodyPose given;
  given.orientation = AttitudeQuaternion{0.0, 0.0, 0.0, 1.0};
  BodyPose out;
  EXPECT_FALSE(resolveBodyPose(false, given, 0.0, 0.5, out));
}

TEST(ResolveBodyPose, StaleOdometryFallsBack)
{
  BodyPose given;
  given.orientation = AttitudeQuaternion{0.0, 0.0, 0.0, 1.0};
  given.x = 1.0;
  BodyPose out;
  EXPECT_FALSE(resolveBodyPose(true, given, /*age_sec=*/0.51, /*max_age=*/0.5, out));
}

TEST(ResolveBodyPose, NegativeAgeFallsBack)
{
  BodyPose given;
  given.orientation = AttitudeQuaternion{0.0, 0.0, 0.0, 1.0};
  BodyPose out;
  EXPECT_FALSE(resolveBodyPose(true, given, /*age_sec=*/-0.01, /*max_age=*/0.5, out));
}

TEST(ResolveBodyPose, FreshFinitePosePassesThrough)
{
  BodyPose given;
  given.orientation = AttitudeQuaternion{0.0, 0.0, 0.0, 1.0};
  given.x = 4.0;
  given.y = -1.0;
  given.z = 0.5;
  BodyPose out;
  ASSERT_TRUE(resolveBodyPose(true, given, /*age_sec=*/0.1, /*max_age=*/0.5, out));
  EXPECT_DOUBLE_EQ(out.x, 4.0);
  EXPECT_DOUBLE_EQ(out.y, -1.0);
  EXPECT_DOUBLE_EQ(out.z, 0.5);
}

TEST(ResolveBodyPose, NonFinitePoseFallsBack)
{
  BodyPose given;
  given.orientation = AttitudeQuaternion{0.0, 0.0, 0.0, 1.0};
  given.x = std::numeric_limits<double>::quiet_NaN();
  BodyPose out;
  EXPECT_FALSE(resolveBodyPose(true, given, /*age_sec=*/0.1, /*max_age=*/0.5, out));
}

namespace
{

// The exact composition target_tracker_node.cpp does in onObstacles()/
// publish(): transform in (if compensating), update(), transform out. Kept
// here so the test exercises the REAL pipeline shape, not a re-abstraction.
std::vector<uav_perception::TrackSnapshot> updateCompensated(
  MultiTargetTracker & tracker, const std::vector<TrackObservation> & raw_observations,
  double stamp_sec, bool compensate, const BodyPose & pose)
{
  std::vector<TrackObservation> observations = raw_observations;
  if (compensate) {
    for (auto & observation : observations) {
      opticalPointToOdom(
        observation.x, observation.y, observation.z, pose,
        observation.x, observation.y, observation.z);
    }
  }
  auto snapshots = tracker.update(observations, stamp_sec);
  if (compensate) {
    for (auto & snapshot : snapshots) {
      odomPointToOptical(
        snapshot.x, snapshot.y, snapshot.z, pose, snapshot.x, snapshot.y, snapshot.z);
      odomVectorToOptical(
        snapshot.vx, snapshot.vy, snapshot.vz, pose, snapshot.vx, snapshot.vy, snapshot.vz);
    }
  }
  return snapshots;
}

// Independent oracle (identity-orientation camera orbiting a fixed odom
// point/moving point): hand-written optical convention, deliberately NOT
// calling production opticalToBody/bodyToOptical, so this exercises the fix
// as a black box.
TrackObservation orbitObservationOf(
  double object_odom_x, double object_odom_y, double object_odom_z, const BodyPose & camera_pose)
{
  const double rel_x = object_odom_x - camera_pose.x;
  const double rel_y = object_odom_y - camera_pose.y;
  const double rel_z = object_odom_z - camera_pose.z;
  TrackObservation observation;
  observation.x = -rel_y;
  observation.y = -rel_z;
  observation.z = rel_x;
  observation.confidence = 1.0;
  observation.position_stddev_m = -1.0;
  return observation;
}

BodyPose orbitCameraPose(int step, double radius_m)
{
  const double angle = step * M_PI / 4.0;
  BodyPose pose;
  pose.orientation = AttitudeQuaternion{0.0, 0.0, 0.0, 1.0};
  pose.x = radius_m * std::cos(angle);
  pose.y = radius_m * std::sin(angle);
  pose.z = 0.0;
  return pose;
}

}  // namespace

TEST(MultiTargetTracker, StationaryTargetOrbitedByCameraKeepsOneIdWhenCompensated)
{
  // (a) RED-before-GREEN. A camera orbiting a STATIONARY odom object at
  // radius 2 m: consecutive optical-frame observations jump ~1.5 m each
  // step (see report), comfortably clearing the default association gate
  // (min_association_gate_m 0.5 m) -- this is the measured mechanism, not a
  // guess (see PR report for the full hand-derivation).
  const double object_x = 5.0, object_y = 0.0, object_z = 1.0;
  const double dt = 0.2;

  // RED: raw camera-frame association (pre-fix / odometry unavailable).
  {
    MultiTargetTracker tracker;
    std::set<std::int32_t> published_ids;
    for (int k = 0; k < 8; ++k) {
      const BodyPose pose = orbitCameraPose(k, 2.0);
      const auto observation = orbitObservationOf(object_x, object_y, object_z, pose);
      const auto snapshots = updateCompensated(
        tracker, {observation}, k * dt, /*compensate=*/false, pose);
      for (const auto & snapshot : snapshots) {
        published_ids.insert(snapshot.track_id);
      }
    }
    // Never a SINGLE stable id tracking the object throughout: either no
    // track ever confirms (M-of-N never accumulates against a jumping
    // apparent position) or more than one id appears -- both are the churn
    // failure mode this fix targets, see report for the measured count.
    EXPECT_NE(published_ids.size(), 1u) <<
      "sanity: uncompensated association must NOT settle on one stable id";
  }

  // GREEN: ego-motion compensated association.
  {
    MultiTargetTracker tracker;
    std::set<std::int32_t> published_ids;
    for (int k = 0; k < 8; ++k) {
      const BodyPose pose = orbitCameraPose(k, 2.0);
      const auto observation = orbitObservationOf(object_x, object_y, object_z, pose);
      const auto snapshots = updateCompensated(
        tracker, {observation}, k * dt, /*compensate=*/true, pose);
      for (const auto & snapshot : snapshots) {
        published_ids.insert(snapshot.track_id);
        // Published position (back-transformed to optical) must match what
        // the independent oracle says the object looks like from THIS pose.
        EXPECT_NEAR(snapshot.x, observation.x, 1e-6);
        EXPECT_NEAR(snapshot.y, observation.y, 1e-6);
        EXPECT_NEAR(snapshot.z, observation.z, 1e-6);
      }
    }
    ASSERT_EQ(published_ids.size(), 1u) <<
      "compensated association: exactly one stable id must track the stationary object";
  }
}

TEST(MultiTargetTracker, MovingTargetOrbitedByCameraKeepsOneIdWhenCompensated)
{
  // (b) A target moving in ODOM at 0.2 m/s while the camera also orbits.
  const double dt = 0.2;
  const double target_speed_odom = 0.2;  // m/s along odom Y

  MultiTargetTracker tracker;
  std::set<std::int32_t> published_ids;
  uav_perception::TrackSnapshot last;
  bool have_last = false;
  for (int k = 0; k < 10; ++k) {
    const double t = k * dt;
    const double object_x = 5.0;
    const double object_y = target_speed_odom * t;
    const double object_z = 1.0;
    const BodyPose pose = orbitCameraPose(k, 2.0);
    const auto observation = orbitObservationOf(object_x, object_y, object_z, pose);
    const auto snapshots = updateCompensated(tracker, {observation}, t, /*compensate=*/true, pose);
    for (const auto & snapshot : snapshots) {
      published_ids.insert(snapshot.track_id);
      last = snapshot;
      have_last = true;
    }
  }
  ASSERT_EQ(published_ids.size(), 1u) << "one stable id even while the target itself moves";
  ASSERT_TRUE(have_last);
  // Position update is smooth: the LAST optical-frame position must match
  // the independent oracle for the LAST camera pose (not lag behind it).
  const BodyPose last_pose = orbitCameraPose(9, 2.0);
  const auto expected_last = orbitObservationOf(5.0, target_speed_odom * 9 * dt, 1.0, last_pose);
  EXPECT_NEAR(last.x, expected_last.x, 0.1);
  EXPECT_NEAR(last.y, expected_last.y, 0.1);
  EXPECT_NEAR(last.z, expected_last.z, 0.1);
}

TEST(MultiTargetTracker, StaleOdometryFallsBackToRawAssociationWithoutCrashing)
{
  // (c) Odometry stale/unavailable: falls back to the pre-fix behaviour
  // (raw camera-frame association) -- must not crash or produce NaN, and
  // must behave EXACTLY like the 19 base tests above (which never touch
  // ego_motion at all).
  MultiTargetTracker tracker;
  BodyPose unused;  // never resolved (have_odometry=false), never read
  const auto snapshots = updateCompensated(
    tracker, {TrackObservation{}}, 0.0, /*compensate=*/false, unused);
  // First match: still TENTATIVE under default M/N, nothing published yet --
  // exactly the bug #10 behaviour, unaffected by this fix.
  EXPECT_TRUE(snapshots.empty());
  const auto second = updateCompensated(
    tracker, {TrackObservation{}}, 0.06, /*compensate=*/false, unused);
  EXPECT_TRUE(second.empty());
  const auto third = updateCompensated(
    tracker, {TrackObservation{}}, 0.12, /*compensate=*/false, unused);
  ASSERT_EQ(third.size(), 1u) << "3rd match confirms, same as the M/N tests above";
  EXPECT_TRUE(std::isfinite(third[0].x));
  EXPECT_TRUE(std::isfinite(third[0].y));
  EXPECT_TRUE(std::isfinite(third[0].z));
}
