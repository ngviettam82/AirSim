// Velocity is in the SENSING frame; world_model owns any further conversion.
// (Internally now associated/tracked in ODOM -- see README ego-motion fix.)

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <uav_interfaces/msg/obstacle.hpp>
#include <uav_interfaces/msg/obstacle_array.hpp>
#include <uav_interfaces/msg/target_track.hpp>

#include "uav_perception/ego_motion.hpp"
#include "uav_perception/target_tracking.hpp"

namespace
{

using nav_msgs::msg::Odometry;
using uav_interfaces::msg::Obstacle;
using uav_interfaces::msg::ObstacleArray;
using uav_interfaces::msg::TargetTrack;

// Same helper/formula as obstacle_extractor_node.cpp / uav_navigation's
// local_planner_node.cpp -- avoids rclcpp::Time-vs-builtin_interfaces::msg::
// Time clock-source mismatches.
double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

std::uint8_t statusToMsg(uav_perception::TrackStatus status)
{
  switch (status) {
    case uav_perception::TrackStatus::kTracking:
      return TargetTrack::STATUS_TRACKING;
    case uav_perception::TrackStatus::kCoasting:
      return TargetTrack::STATUS_COASTING;
    case uav_perception::TrackStatus::kLost:
      return TargetTrack::STATUS_LOST;
  }
  return TargetTrack::STATUS_UNKNOWN;
}

class TargetTrackerNode : public rclcpp::Node
{
public:
  TargetTrackerNode()
  : Node("target_tracker_node")
  {
    uav_id_ = declare_parameter<std::string>("uav_id", "uav0");

    uav_perception::TrackingParams params;
    params.min_target_extent_m = declare_parameter<double>("min_target_extent_m", 0.05);
    params.max_target_extent_m = declare_parameter<double>("max_target_extent_m", 5.0);
    params.min_observation_confidence =
      declare_parameter<double>("min_observation_confidence", 0.0);
    params.process_noise_accel_std_m_s2 =
      declare_parameter<double>("process_noise_accel_std_m_s2", 1.0);
    params.default_observation_stddev_m =
      declare_parameter<double>("default_observation_stddev_m", 0.3);
    params.initial_velocity_stddev_m_s =
      declare_parameter<double>("initial_velocity_stddev_m_s", 2.0);
    params.association_gate_sigma = declare_parameter<double>("association_gate_sigma", 3.0);
    params.min_association_gate_m = declare_parameter<double>("min_association_gate_m", 0.5);
    // Measured max frame gap 1.8s (P5 plan Sec 0.1b) plus one missed-cycle margin.
    params.lost_after_sec = declare_parameter<double>("lost_after_sec", 3.0);
    params.max_tracks = static_cast<std::size_t>(declare_parameter<int>("max_tracks", 20));
    // M-of-N track confirmation (bug #10, G-M4.4): a fresh track is
    // TENTATIVE and never published until it clears this bar -- see README
    // and TrackingParams::confirm_hits_required.
    params.confirm_hits_required = static_cast<std::size_t>(
      declare_parameter<int>("confirm_hits_required", 3));
    params.confirm_window_frames = static_cast<std::size_t>(
      declare_parameter<int>("confirm_window_frames", 5));
    params_ = params;
    // Ego-motion compensation (G-M3/G-M4, 2026-08-23): even a REAL,
    // stationary target churned track_id while the drone orbited it,
    // because association ran in the moving camera-optical frame -- see
    // README. R32: sample-and-hold must carry an age and self-expire; Tier 2
    // (stale/missing odometry) degrades to the old camera-frame behaviour.
    odometry_max_age_sec_ = declare_parameter<double>("odometry_max_age_sec", 0.5);
    validateConfirmationParams();
    tracker_.configure(params);

    const double age_check_period_sec =
      declare_parameter<double>("age_check_period_sec", 0.5);

    obstacles_subscription_ = create_subscription<ObstacleArray>(
      "/uav/" + uav_id_ + "/perception/obstacles_local", rclcpp::QoS(10),
      [this](const ObstacleArray::SharedPtr message) {onObstacles(*message);});
    // Full pose (position + orientation), unlike obstacle_extractor_node's
    // orientation-only ground filter -- association needs translation too.
    // Single-threaded spin() in main() (no callback groups/
    // MultiThreadedExecutor), so this and onObstacles/onAgeTimer never run
    // concurrently -- R24 is a non-issue here by construction, not by
    // locking; do not add a MultiThreadedExecutor without revisiting this.
    odometry_subscription_ = create_subscription<Odometry>(
      "/uav/" + uav_id_ + "/state/odometry_fused", rclcpp::SensorDataQoS(),
      [this](const Odometry::SharedPtr message) {latest_odometry_ = message;});

    publisher_ = create_publisher<TargetTrack>(
      "/uav/" + uav_id_ + "/perception/target_track", rclcpp::QoS(10));

    // Node clock (respects use_sim_time): sole safety net for a source that
    // stops publishing entirely, see README (nợ #1 lesson: never a wall timer).
    age_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(age_check_period_sec),
      [this]() {onAgeTimer();});

    RCLCPP_INFO(get_logger(), "target tracker ready for %s", uav_id_.c_str());
  }

private:
  // Refuses to start rather than silently confirm-on-first-match again (M=0
  // would let hits>=0 pass trivially) or ship a window a track can never
  // clear (N<M) -- same "refuse at startup" style as obstacle_extractor_node.
  void validateConfirmationParams()
  {
    std::string reason;
    if (params_.confirm_hits_required < 1) {
      reason = "confirm_hits_required must be at least 1";
    } else if (params_.confirm_window_frames < params_.confirm_hits_required) {
      reason = "confirm_window_frames must be >= confirm_hits_required";
    } else if (!std::isfinite(odometry_max_age_sec_) || odometry_max_age_sec_ <= 0.0) {
      reason = "odometry_max_age_sec must be finite and positive";
    }
    if (!reason.empty()) {
      RCLCPP_FATAL(get_logger(), "track confirmation params refused: %s", reason.c_str());
      throw std::runtime_error("track confirmation params refused: " + reason);
    }
  }

  // R32: fresh, finite odometry pose -> Tier 1 (ego-motion compensated).
  // Missing/stale/non-finite -> Tier 2, caller falls back to the old
  // camera-frame association. Age measured against node "now" (respects
  // use_sim_time), same convention as obstacle_extractor_node's ground
  // filter -- NOT the triggering observation's own stamp.
  bool resolveCurrentBodyPose(uav_perception::BodyPose & out_pose)
  {
    const bool have_odometry = static_cast<bool>(latest_odometry_);
    uav_perception::BodyPose candidate;
    double age_sec = std::numeric_limits<double>::infinity();
    if (have_odometry) {
      const auto & p = latest_odometry_->pose.pose.position;
      const auto & q = latest_odometry_->pose.pose.orientation;
      candidate.x = p.x;
      candidate.y = p.y;
      candidate.z = p.z;
      candidate.orientation = uav_perception::AttitudeQuaternion{q.x, q.y, q.z, q.w};
      age_sec = get_clock()->now().seconds() - stampSeconds(latest_odometry_->header.stamp);
    }
    return uav_perception::resolveBodyPose(
      have_odometry, candidate, age_sec, odometry_max_age_sec_, out_pose);
  }

  void onObstacles(const ObstacleArray & message)
  {
    // Frame changed under our feet: mixing positions across frames would
    // silently corrupt every track (see README, mirrors marker_detector).
    if (has_frame_ && message.header.frame_id != tracking_frame_) {
      RCLCPP_WARN(
        get_logger(), "observation frame changed %s -> %s, resetting all tracks",
        tracking_frame_.c_str(), message.header.frame_id.c_str());
      tracker_.reset();
    }
    tracking_frame_ = message.header.frame_id;
    has_frame_ = true;

    // Ego-motion compensation mode switch (bug: ID churn while orbiting a
    // real target, see README). A track's internal state must never mix
    // odom-frame and camera-frame values -- same rule as the frame_id check
    // above, so a mode change resets exactly like a frame change does.
    uav_perception::BodyPose resolved_pose;
    const bool odom_ready = resolveCurrentBodyPose(resolved_pose);
    if (odom_ready != associating_in_odom_) {
      RCLCPP_WARN(
        get_logger(), "association frame switched to %s, resetting all tracks",
        odom_ready ? "odom (ego-motion compensated)" : "camera-optical (odometry unavailable/stale)");
      tracker_.reset();
      associating_in_odom_ = odom_ready;
    }
    if (odom_ready) {
      last_resolved_pose_ = resolved_pose;
    }

    std::vector<uav_perception::TrackObservation> observations;
    observations.reserve(message.obstacles.size());
    for (const auto & obstacle : message.obstacles) {
      // Size/shape is obstacle-specific (see header: TrackObservation stays
      // source-neutral), so this filter lives here, not inside the tracker.
      if (!uav_perception::isPlausibleTargetExtent(
          obstacle.size.x, obstacle.size.y, obstacle.size.z, params_))
      {
        continue;
      }
      double x = obstacle.center.x;
      double y = obstacle.center.y;
      double z = obstacle.center.z;
      if (associating_in_odom_) {
        uav_perception::opticalPointToOdom(
          obstacle.center.x, obstacle.center.y, obstacle.center.z, last_resolved_pose_, x, y, z);
      }
      uav_perception::TrackObservation observation;
      observation.x = x;
      observation.y = y;
      observation.z = z;
      observation.confidence = obstacle.confidence;
      observation.position_stddev_m = obstacle.position_uncertainty;
      observations.push_back(observation);
    }

    // dt comes from the observation stamp (irregular arrival, see README);
    // never wall clock.
    const double stamp_sec = rclcpp::Time(message.header.stamp).seconds();
    const auto snapshots = tracker_.update(observations, stamp_sec);
    publish(message.header, snapshots);
  }

  void onAgeTimer()
  {
    if (!has_frame_) {
      return;  // never received an observation yet, nothing to age
    }
    const double now_sec = get_clock()->now().seconds();
    const auto expired = tracker_.ageOut(now_sec);
    if (expired.empty()) {
      return;
    }
    std_msgs::msg::Header header;
    header.stamp = get_clock()->now();
    header.frame_id = tracking_frame_;
    publish(header, expired);
  }

  void publish(
    const std_msgs::msg::Header & header,
    const std::vector<uav_perception::TrackSnapshot> & snapshots)
  {
    for (const auto & snapshot : snapshots) {
      TargetTrack track;
      track.header.stamp = header.stamp;
      // Propagated from the observation source, never hardcoded (see README).
      track.header.frame_id = header.frame_id;
      track.uav_id = uav_id_;
      track.track_id = snapshot.track_id;
      track.status = statusToMsg(snapshot.status);

      // Contract unchanged: frame_id/fields stay camera-optical (see
      // README). Internal state is odom-frame while associating_in_odom_,
      // so convert back here -- "as it would appear from the camera right
      // now", using the latest resolved pose (best available: onAgeTimer's
      // LOST snapshots have no fresher pose to use than this).
      double x = snapshot.x;
      double y = snapshot.y;
      double z = snapshot.z;
      double vx = snapshot.vx;
      double vy = snapshot.vy;
      double vz = snapshot.vz;
      if (associating_in_odom_) {
        uav_perception::odomPointToOptical(
          snapshot.x, snapshot.y, snapshot.z, last_resolved_pose_, x, y, z);
        uav_perception::odomVectorToOptical(
          snapshot.vx, snapshot.vy, snapshot.vz, last_resolved_pose_, vx, vy, vz);
      }

      track.pose.position.x = x;
      track.pose.position.y = y;
      track.pose.position.z = z;
      track.pose.orientation.w = 1.0;  // identity: shape gives no orientation

      track.velocity.x = vx;
      track.velocity.y = vy;
      track.velocity.z = vz;

      track.confidence = static_cast<float>(snapshot.confidence);
      track.position_uncertainty = static_cast<float>(snapshot.position_uncertainty_m);
      track.age_sec = static_cast<float>(snapshot.age_sec);
      track.time_since_seen_sec = static_cast<float>(snapshot.time_since_seen_sec);

      publisher_->publish(track);
    }
  }

  std::string uav_id_;
  std::string tracking_frame_;
  bool has_frame_{false};
  uav_perception::TrackingParams params_;
  uav_perception::MultiTargetTracker tracker_;

  double odometry_max_age_sec_{0.5};
  Odometry::SharedPtr latest_odometry_;
  bool associating_in_odom_{false};
  uav_perception::BodyPose last_resolved_pose_;

  rclcpp::Subscription<ObstacleArray>::SharedPtr obstacles_subscription_;
  rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Publisher<TargetTrack>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr age_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
