// Frames, mount offsets and uncertainty floors: see package README.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <uav_interfaces/msg/marker_observation.hpp>
#include <uav_interfaces/msg/obstacle_array.hpp>
#include <uav_interfaces/msg/semantic_landmark.hpp>
#include <uav_interfaces/msg/semantic_landmark_array.hpp>
#include <uav_interfaces/msg/target_state.hpp>
#include <uav_interfaces/msg/target_track.hpp>

#include "uav_world_model/frame_transforms.hpp"
#include "uav_world_model/pose_buffer.hpp"
#include "uav_world_model/uncertainty.hpp"
#include "uav_world_model/world_map.hpp"
#include "uav_world_model/world_model_node.hpp"

namespace
{

using geometry_msgs::msg::Point;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Quaternion;
using geometry_msgs::msg::Vector3;
using nav_msgs::msg::Odometry;
using uav_interfaces::msg::MarkerObservation;
using uav_interfaces::msg::Obstacle;
using uav_interfaces::msg::ObstacleArray;
using uav_interfaces::msg::SemanticLandmark;
using uav_interfaces::msg::SemanticLandmarkArray;
using uav_interfaces::msg::TargetState;
using uav_interfaces::msg::TargetTrack;

static_assert(
  static_cast<uint8_t>(TargetTrack::STATUS_LOST) == uav_world_model::kTargetStatusLost,
  "world map lost-status constant drifted from TargetTrack");

using uav_world_model::Quat;
using uav_world_model::Rigid;
using uav_world_model::SensorMount;
using uav_world_model::Vec3;

struct BuiltInMount
{
  const char * name;
  const char * frame_suffix;
  bool optical;
  double translation[3];
  double rotation_rpy[3];
};

// Simulator geometry; a real airframe must configure its own.
constexpr BuiltInMount kBuiltInMounts[] = {
  {"camera_down", "/camera_down_optical", true, {0.06, 0.0, -0.065}, {0.0, 1.5707963, 0.0}},
};

const BuiltInMount * findBuiltIn(const std::string & name)
{
  for (const auto & mount : kBuiltInMounts) {
    if (name == mount.name) {
      return &mount;
    }
  }
  return nullptr;
}

Vec3 toVec3(const Point & point)
{
  return Vec3{point.x, point.y, point.z};
}

Vec3 toVec3(const Vector3 & vector)
{
  return Vec3{vector.x, vector.y, vector.z};
}

Quat toQuat(const Quaternion & quaternion)
{
  return Quat{quaternion.x, quaternion.y, quaternion.z, quaternion.w};
}

Point toPoint(const Vec3 & value)
{
  Point point;
  point.x = value.x;
  point.y = value.y;
  point.z = value.z;
  return point;
}

Vector3 toVector3(const Vec3 & value)
{
  Vector3 vector;
  vector.x = value.x;
  vector.y = value.y;
  vector.z = value.z;
  return vector;
}

Quaternion toQuaternion(const Quat & value)
{
  Quaternion quaternion;
  quaternion.x = value.x;
  quaternion.y = value.y;
  quaternion.z = value.z;
  quaternion.w = value.w;
  return quaternion;
}

bool isFinite(const Vec3 & value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

// A zero quaternion normalises to identity: level facing north.
bool isUsableRotation(const Quat & value)
{
  const double squared = value.x * value.x + value.y * value.y +
    value.z * value.z + value.w * value.w;
  return std::isfinite(squared) && squared > 0.5 && squared < 2.0;
}

builtin_interfaces::msg::Time toRosTime(double seconds)
{
  builtin_interfaces::msg::Time stamp;
  const double whole = std::floor(seconds);
  stamp.sec = static_cast<int32_t>(whole);
  stamp.nanosec = static_cast<uint32_t>(std::lround((seconds - whole) * 1.0e9));
  if (stamp.nanosec >= 1000000000U) {
    stamp.nanosec -= 1000000000U;
    ++stamp.sec;
  }
  return stamp;
}

// Range decays as the aircraft moves, so it is recomputed, never carried.
double rangeToVehicle(
  const uav_world_model::ObstacleObservation & obstacle, const Vec3 & vehicle_in_odom)
{
  const double to_centre = uav_world_model::norm(
    uav_world_model::subtract(obstacle.center, vehicle_in_odom));
  const double half_diagonal = 0.5 * uav_world_model::norm(obstacle.size);
  return std::max(0.0, to_centre - half_diagonal);
}

class WorldModelNode : public rclcpp::Node
{
public:
  explicit WorldModelNode(const rclcpp::NodeOptions & options)
  : Node("world_model_node", options)
  {
    uav_id_ = declare_parameter<std::string>("uav_id", "uav0");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 10.0);
    report_period_sec_ = declare_parameter<double>("report_period_sec", 10.0);

    uav_world_model::PoseBufferOptions buffer_options;
    buffer_options.history_sec = declare_parameter<double>("pose_history_sec", 3.0);
    buffer_options.max_pose_gap_sec = declare_parameter<double>("max_pose_gap_sec", 0.25);
    max_pose_gap_sec_ = buffer_options.max_pose_gap_sec;
    max_future_stamp_sec_ = declare_parameter<double>("max_future_stamp_sec", 1.0);
    pose_buffer_ = uav_world_model::PoseBuffer(buffer_options);

    uav_world_model::WorldMapOptions map_options;
    map_options.landmark_forget_sec = declare_parameter<double>("landmark_forget_sec", 120.0);
    map_options.obstacle_forget_sec = declare_parameter<double>("obstacle_forget_sec", 5.0);
    map_options.target_forget_sec = declare_parameter<double>("target_forget_sec", 10.0);
    const double track_switch_after_sec =
      declare_parameter<double>("target_track_switch_after_sec", 1.0);
    if (!(track_switch_after_sec > 0.0)) {
      RCLCPP_ERROR(
        get_logger(), "target_track_switch_after_sec must be positive; keeping 1.0 s");
    }
    map_options.target_track_switch_after_sec =
      track_switch_after_sec > 0.0 ? track_switch_after_sec : 1.0;
    map_options.drift.sigma_m = declare_parameter<double>("odom_drift_sigma_m", 0.5);
    map_options.drift.correlation_time_sec =
      declare_parameter<double>("odom_drift_correlation_time_sec", 120.0);
    map_ = uav_world_model::WorldMap(map_options);

    marker_floor_.base_m = declare_parameter<double>("marker_uncertainty_base_m", 0.0185);
    marker_floor_.per_metre = declare_parameter<double>("marker_uncertainty_per_metre", 0.007);
    obstacle_floor_.base_m = declare_parameter<double>("obstacle_uncertainty_base_m", 0.10);
    obstacle_floor_.per_metre = declare_parameter<double>("obstacle_uncertainty_per_metre", 0.02);
    target_floor_.base_m = declare_parameter<double>("target_uncertainty_base_m", 0.10);
    target_floor_.per_metre = declare_parameter<double>("target_uncertainty_per_metre", 0.02);

    obstacle_floor_is_placeholder_ =
      !declare_parameter<bool>("obstacle_uncertainty_measured", false);
    target_floor_is_placeholder_ =
      !declare_parameter<bool>("target_uncertainty_measured", false);

    obstacle_source_timeout_sec_ =
      declare_parameter<double>("obstacle_source_timeout_sec", 5.0);
    target_velocity_is_relative_ =
      declare_parameter<bool>("target_velocity_is_relative", true);
    publish_mission_reference_ =
      declare_parameter<bool>("publish_mission_reference", true);

    loadMounts();

    const std::string prefix = "/uav/" + uav_id_;

    odometry_subscription_ = create_subscription<Odometry>(
      prefix + "/state/odometry_fused", rclcpp::QoS(20),
      [this](const Odometry::SharedPtr message) {onOdometry(*message);});
    marker_subscription_ = create_subscription<MarkerObservation>(
      prefix + "/perception/markers", rclcpp::QoS(20),
      [this](const MarkerObservation::SharedPtr message) {onMarker(*message);});
    // Best-effort requests also match a reliable publisher; their QoS is unchosen.
    obstacle_subscription_ = create_subscription<ObstacleArray>(
      prefix + "/perception/obstacles_local", rclcpp::SensorDataQoS(),
      [this](const ObstacleArray::SharedPtr message) {onObstacles(*message);});
    target_subscription_ = create_subscription<TargetTrack>(
      prefix + "/perception/target_track", rclcpp::SensorDataQoS(),
      [this](const TargetTrack::SharedPtr message) {onTarget(*message);});

    landmark_publisher_ = create_publisher<SemanticLandmarkArray>(
      prefix + "/world/semantic_landmarks", rclcpp::QoS(10));
    obstacle_publisher_ = create_publisher<ObstacleArray>(
      prefix + "/world/obstacle_map_local", rclcpp::QoS(10));
    target_publisher_ = create_publisher<TargetState>(
      prefix + "/world/target_state", rclcpp::QoS(10));
    mission_reference_publisher_ = create_publisher<PoseStamped>(
      prefix + "/world/mission_reference",
      rclcpp::QoS(1).transient_local().reliable());

    // Node clock, not wall clock: ages follow /clock.
    timer_ = rclcpp::create_timer(
      this, get_clock(),
      rclcpp::Duration::from_seconds(1.0 / std::max(publish_rate_hz, 0.1)),
      [this]() {onTimer();});

    RCLCPP_INFO(
      get_logger(), "world model ready for %s, publishing %s at %.1f Hz",
      uav_id_.c_str(), odom_frame_.c_str(), publish_rate_hz);
    RCLCPP_INFO(
      get_logger(), "target velocity read as %s to the aircraft",
      target_velocity_is_relative_ ? "relative" : "absolute");
  }

private:
  void loadMounts()
  {
    const std::vector<std::string> default_names{"camera_down"};
    const auto names = declare_parameter<std::vector<std::string>>("frames.names", default_names);

    // A body-frame observation needs no mount and must stay usable.
    SensorMount body_mount;
    body_mount.frame_id = base_frame_;
    body_mount.reports_optical_axes = false;
    mounts_[body_mount.frame_id] = body_mount;

    for (const auto & name : names) {
      const BuiltInMount * fallback = findBuiltIn(name);
      const std::string parameter_prefix = "frames." + name + ".";

      const std::string default_frame =
        fallback != nullptr ? uav_id_ + fallback->frame_suffix : std::string();

      SensorMount mount;
      mount.frame_id =
        declare_parameter<std::string>(parameter_prefix + "frame_id", default_frame);
      mount.reports_optical_axes = declare_parameter<bool>(
        parameter_prefix + "optical", fallback != nullptr ? fallback->optical : true);
      auto translation = declare_parameter<std::vector<double>>(
        parameter_prefix + "translation", std::vector<double>{});
      auto rotation = declare_parameter<std::vector<double>>(
        parameter_prefix + "rotation_rpy", std::vector<double>{});

      // A guessed mount moves the whole map by a constant.
      const bool unconfigured = translation.size() != 3 || rotation.size() != 3;
      if (unconfigured && fallback != nullptr) {
        translation.assign(fallback->translation, fallback->translation + 3);
        rotation.assign(fallback->rotation_rpy, fallback->rotation_rpy + 3);
        RCLCPP_WARN(
          get_logger(),
          "mount '%s' fell back to built-in SIMULATOR geometry; a real airframe must set "
          "frames.%s.translation and .rotation_rpy",
          name.c_str(), name.c_str());
      } else if (unconfigured || mount.frame_id.empty()) {
        RCLCPP_ERROR(
          get_logger(),
          "mount '%s' needs frame_id, translation[3] and rotation_rpy[3]; ignoring it",
          name.c_str());
        continue;
      }

      mount.base_from_sensor.translation = Vec3{translation[0], translation[1], translation[2]};
      mount.base_from_sensor.rotation =
        uav_world_model::fromRollPitchYaw(rotation[0], rotation[1], rotation[2]);
      mounts_[mount.frame_id] = mount;

      RCLCPP_INFO(
        get_logger(),
        "mount %s at (%.4f %.4f %.4f) m rpy (%.4f %.4f %.4f) rad, optical=%s",
        mount.frame_id.c_str(), translation[0], translation[1], translation[2],
        rotation[0], rotation[1], rotation[2], mount.reports_optical_axes ? "yes" : "no");
    }
  }

  const SensorMount * findMount(const std::string & frame_id)
  {
    const auto found = mounts_.find(frame_id);
    if (found != mounts_.end()) {
      return &found->second;
    }
    ++rejected_unknown_frame_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "no mount configured for frame '%s'; observation dropped", frame_id.c_str());
    return nullptr;
  }

  bool acceptsUav(const std::string & observed_uav_id)
  {
    if (observed_uav_id.empty() || observed_uav_id == uav_id_) {
      return true;
    }
    ++rejected_foreign_uav_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "observation from '%s' on the topics of '%s'; dropped",
      observed_uav_id.c_str(), uav_id_.c_str());
    return false;
  }

  static double seconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) + 1.0e-9 * static_cast<double>(stamp.nanosec);
  }

  double nowSeconds() const
  {
    return now().seconds();
  }

  uav_world_model::PoseMatch matchPose(double stamp_sec)
  {
    const auto match = pose_buffer_.lookup(stamp_sec);
    switch (match.result) {
      case uav_world_model::PoseMatchResult::kOk:
        break;
      case uav_world_model::PoseMatchResult::kNoPose:
        ++rejected_no_pose_;
        break;
      default:
        ++rejected_pose_gap_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "no fused pose within %.2f s of an observation; dropped", max_pose_gap_sec_);
        break;
    }
    return match;
  }

  void onOdometry(const Odometry & message)
  {
    const double stddev = uav_world_model::worstAxisStddev(message.pose.covariance);
    if (!uav_world_model::isUsableUncertainty(stddev)) {
      ++rejected_pose_no_uncertainty_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "fused odometry states no usable position error; not used for mapping");
      return;
    }

    const double stamp_sec = seconds(message.header.stamp);
    if (stamp_sec > nowSeconds() + max_future_stamp_sec_) {
      ++rejected_pose_unusable_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "fused odometry is stamped %.1f s in the future; check use_sim_time",
        stamp_sec - nowSeconds());
      return;
    }

    uav_world_model::TimedPose pose;
    pose.stamp_sec = stamp_sec;
    pose.odom_from_base.translation = toVec3(message.pose.pose.position);
    pose.odom_from_base.rotation = toQuat(message.pose.pose.orientation);
    pose.position_stddev = stddev;

    if (!isFinite(pose.odom_from_base.translation) ||
      !isUsableRotation(pose.odom_from_base.rotation))
    {
      ++rejected_pose_unusable_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "fused odometry carries no usable position or attitude; not used for mapping");
      return;
    }

    pose.body_velocity = toVec3(message.twist.twist.linear);
    pose.body_angular_velocity = toVec3(message.twist.twist.angular);
    if (!isFinite(pose.body_velocity)) {
      pose.body_velocity = Vec3{};
    }
    if (!isFinite(pose.body_angular_velocity)) {
      pose.body_angular_velocity = Vec3{};
    }

    if (!pose_buffer_.add(pose)) {
      ++rejected_pose_no_uncertainty_;
      return;
    }

    if (publish_mission_reference_ && !mission_reference_published_) {
      PoseStamped reference;
      reference.header.stamp = message.header.stamp;
      reference.header.frame_id = odom_frame_;
      reference.pose = message.pose.pose;
      mission_reference_publisher_->publish(reference);
      mission_reference_published_ = true;
      RCLCPP_INFO(
        get_logger(), "mission reference anchored at (%.3f %.3f %.3f) in %s",
        reference.pose.position.x, reference.pose.position.y, reference.pose.position.z,
        odom_frame_.c_str());
    }
  }

  void onMarker(const MarkerObservation & message)
  {
    if (!acceptsUav(message.uav_id)) {
      return;
    }
    const SensorMount * mount = findMount(message.header.frame_id);
    if (mount == nullptr) {
      return;
    }
    const double stamp_sec = seconds(message.header.stamp);
    const auto match = matchPose(stamp_sec);
    if (!match.ok()) {
      return;
    }

    const Vec3 in_sensor = toVec3(message.pose.position);
    const double range_m = uav_world_model::norm(in_sensor);

    uav_world_model::LandmarkObservation landmark;
    landmark.marker_id = message.marker_id;
    landmark.family = message.family;
    landmark.position =
      uav_world_model::sensorPointToOdom(in_sensor, *mount, match.odom_from_base);
    landmark.orientation = uav_world_model::sensorRotationToOdom(
      toQuat(message.pose.orientation), *mount, match.odom_from_base);
    landmark.position_uncertainty_m = uav_world_model::combineQuadrature(
      {uav_world_model::perceptionUncertainty(
          message.position_uncertainty, range_m, marker_floor_),
        match.position_stddev, match.pairingUncertainty()});
    landmark.stamp_sec = stamp_sec;

    map_.observeLandmark(landmark);
    ++accepted_markers_;
  }

  void onObstacles(const ObstacleArray & message)
  {
    if (!acceptsUav(message.uav_id)) {
      return;
    }
    const SensorMount * mount = findMount(message.header.frame_id);
    if (mount == nullptr) {
      return;
    }
    const double stamp_sec = seconds(message.header.stamp);
    const auto match = matchPose(stamp_sec);
    if (!match.ok()) {
      return;
    }

    warnAboutPlaceholderFloor(obstacle_floor_is_placeholder_, "obstacle");

    uav_world_model::ObstacleBatch batch;
    batch.sensing_frame = message.header.frame_id;
    batch.stamp_sec = stamp_sec;
    batch.sensing_range_m = message.sensing_range;
    batch.obstacles.reserve(message.obstacles.size());

    const Quat sensor_to_odom = uav_world_model::sensorRotationToOdom(
      Quat{}, *mount, match.odom_from_base);

    for (const auto & obstacle : message.obstacles) {
      const Vec3 in_sensor = toVec3(obstacle.center);
      const double range_m = uav_world_model::norm(in_sensor);

      uav_world_model::ObstacleObservation item;
      item.obstacle_id = obstacle.obstacle_id;
      item.shape = obstacle.shape;
      item.center =
        uav_world_model::sensorPointToOdom(in_sensor, *mount, match.odom_from_base);
      item.size = uav_world_model::axisAlignedExtent(toVec3(obstacle.size), sensor_to_odom);
      item.confidence = obstacle.confidence;
      item.position_uncertainty_m = uav_world_model::combineQuadrature(
        {uav_world_model::perceptionUncertainty(
            obstacle.position_uncertainty, range_m, obstacle_floor_),
          match.position_stddev, match.pairingUncertainty()});
      batch.obstacles.push_back(item);
    }

    map_.observeObstacles(batch);
    ++accepted_obstacle_batches_;
  }

  void onTarget(const TargetTrack & message)
  {
    if (!acceptsUav(message.uav_id)) {
      return;
    }
    const SensorMount * mount = findMount(message.header.frame_id);
    if (mount == nullptr) {
      return;
    }
    const double stamp_sec = seconds(message.header.stamp);
    const auto match = matchPose(stamp_sec);
    if (!match.ok()) {
      return;
    }

    warnAboutPlaceholderFloor(target_floor_is_placeholder_, "target");

    const Vec3 in_sensor = toVec3(message.pose.position);
    const double range_m = uav_world_model::norm(in_sensor);

    uav_world_model::TargetObservation target;
    target.track_id = message.track_id;
    target.status = message.status;
    target.position =
      uav_world_model::sensorPointToOdom(in_sensor, *mount, match.odom_from_base);
    target.orientation = uav_world_model::sensorRotationToOdom(
      toQuat(message.pose.orientation), *mount, match.odom_from_base);
    target.velocity = uav_world_model::sensorVectorToOdom(
      toVec3(message.velocity), *mount, match.odom_from_base);
    if (target_velocity_is_relative_) {
      const Vec3 lever = uav_world_model::subtract(
        target.position, match.odom_from_base.translation);
      const Vec3 spin = uav_world_model::rotate(
        match.odom_from_base.rotation, match.body_angular_velocity);
      const Vec3 carried = uav_world_model::add(
        uav_world_model::rotate(match.odom_from_base.rotation, match.body_velocity),
        uav_world_model::cross(spin, lever));
      target.velocity = uav_world_model::add(target.velocity, carried);
    }
    target.confidence = message.confidence;
    target.position_uncertainty_m = uav_world_model::combineQuadrature(
      {uav_world_model::perceptionUncertainty(
          message.position_uncertainty, range_m, target_floor_),
        match.position_stddev, match.pairingUncertainty()});
    target.stamp_sec = stamp_sec;
    target.time_since_seen_sec = message.time_since_seen_sec;

    map_.observeTarget(target);
    ++accepted_targets_;
  }

  void warnAboutPlaceholderFloor(bool is_placeholder, const char * kind)
  {
    if (!is_placeholder) {
      return;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "%s uncertainty floor is a guess, not a measurement", kind);
  }

  void onTimer()
  {
    const double now_sec = nowSeconds();
    map_.expire(now_sec);

    publishLandmarks(now_sec);
    publishObstacles(now_sec);
    publishTarget(now_sec);
    reportRejections(now_sec);
  }

  void publishLandmarks(double now_sec)
  {
    SemanticLandmarkArray message;
    message.header.stamp = now();
    message.header.frame_id = odom_frame_;
    message.uav_id = uav_id_;

    for (const auto & landmark : map_.landmarks(now_sec)) {
      SemanticLandmark entry;
      entry.marker_id = landmark.marker_id;
      entry.family = landmark.family;
      entry.pose.position = toPoint(landmark.position);
      entry.pose.orientation = toQuaternion(landmark.orientation);
      entry.position_uncertainty = static_cast<float>(landmark.position_uncertainty_m);
      entry.time_since_seen_sec = static_cast<float>(landmark.time_since_seen_sec);
      message.landmarks.push_back(entry);
    }
    landmark_publisher_->publish(message);
  }

  void publishObstacles(double now_sec)
  {
    // An empty map would read as nothing to avoid.
    if (!map_.hasObstacleSource()) {
      return;
    }
    if (!map_.hasSurvivingBatch() ||
      now_sec - map_.newestObstacleStamp() > obstacle_source_timeout_sec_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "obstacle source silent for %.1f s; not publishing a world obstacle map",
        now_sec - map_.newestObstacleStamp());
      return;
    }

    const uav_world_model::TimedPose * vehicle = pose_buffer_.newest();
    if (vehicle == nullptr) {
      return;
    }

    const auto obstacles = map_.obstacles(now_sec);
    const double oldest_sec = map_.oldestObstacleStamp();

    ObstacleArray message;
    // Never fresher than the source behind it.
    message.header.stamp =
      toRosTime(oldest_sec > 0.0 ? oldest_sec : map_.newestObstacleStamp());
    message.header.frame_id = odom_frame_;
    message.uav_id = uav_id_;
    message.sensing_range = static_cast<float>(map_.obstacleSensingRange());

    for (const auto & item : obstacles) {
      Obstacle entry;
      entry.obstacle_id = item.observation.obstacle_id;
      entry.shape = item.observation.shape;
      entry.center = toPoint(item.observation.center);
      entry.size = toVector3(item.observation.size);
      entry.distance = static_cast<float>(
        rangeToVehicle(item.observation, vehicle->odom_from_base.translation));
      entry.confidence = static_cast<float>(item.observation.confidence);
      entry.position_uncertainty =
        static_cast<float>(item.observation.position_uncertainty_m);
      message.obstacles.push_back(entry);
    }
    obstacle_publisher_->publish(message);
  }

  void publishTarget(double now_sec)
  {
    const auto reported = map_.target(now_sec);
    // No target seen yet: nothing to invent.
    if (!reported.valid) {
      return;
    }

    const uint64_t switches = map_.targetTrackSwitchCount();
    if (switches != reported_track_switches_) {
      reported_track_switches_ = switches;
      RCLCPP_WARN(
        get_logger(), "target selection switched to track %d (%lu switches so far)",
        reported.observation.track_id, static_cast<unsigned long>(switches));
    }

    TargetState message;
    message.header.stamp = now();
    message.header.frame_id = odom_frame_;
    message.uav_id = uav_id_;
    message.track_id = reported.observation.track_id;
    message.status = reported.observation.status;
    message.pose.position = toPoint(reported.observation.position);
    message.pose.orientation = toQuaternion(reported.observation.orientation);
    message.velocity = toVector3(reported.observation.velocity);
    message.position_uncertainty = static_cast<float>(reported.position_uncertainty_m);
    message.time_since_seen_sec = static_cast<float>(reported.time_since_seen_sec);
    target_publisher_->publish(message);
  }

  void reportRejections(double now_sec)
  {
    if (report_period_sec_ <= 0.0 || now_sec - last_report_sec_ < report_period_sec_) {
      return;
    }
    last_report_sec_ = now_sec;

    const uint64_t rejected = rejected_no_pose_ + rejected_pose_gap_ +
      rejected_unknown_frame_ + rejected_foreign_uav_ + rejected_pose_no_uncertainty_ +
      rejected_pose_unusable_ + map_.unselectedTargetTrackCount();
    if (rejected == reported_rejections_) {
      return;
    }
    reported_rejections_ = rejected;

    RCLCPP_WARN(
      get_logger(),
      "dropped observations: no_pose=%lu pose_gap=%lu unknown_frame=%lu foreign_uav=%lu "
      "pose_without_error=%lu pose_unusable=%lu unselected_track=%lu "
      "(accepted markers=%lu obstacles=%lu targets=%lu)",
      static_cast<unsigned long>(rejected_no_pose_),
      static_cast<unsigned long>(rejected_pose_gap_),
      static_cast<unsigned long>(rejected_unknown_frame_),
      static_cast<unsigned long>(rejected_foreign_uav_),
      static_cast<unsigned long>(rejected_pose_no_uncertainty_),
      static_cast<unsigned long>(rejected_pose_unusable_),
      static_cast<unsigned long>(map_.unselectedTargetTrackCount()),
      static_cast<unsigned long>(accepted_markers_),
      static_cast<unsigned long>(accepted_obstacle_batches_),
      static_cast<unsigned long>(accepted_targets_));
  }

  std::string uav_id_;
  std::string odom_frame_;
  std::string base_frame_;

  uav_world_model::PoseBuffer pose_buffer_;
  uav_world_model::WorldMap map_;
  std::map<std::string, SensorMount> mounts_;

  uav_world_model::UncertaintyFloor marker_floor_;
  uav_world_model::UncertaintyFloor obstacle_floor_;
  uav_world_model::UncertaintyFloor target_floor_;

  double obstacle_source_timeout_sec_{5.0};
  double report_period_sec_{10.0};
  double last_report_sec_{0.0};
  double max_pose_gap_sec_{0.25};
  double max_future_stamp_sec_{1.0};
  bool target_velocity_is_relative_{true};
  bool publish_mission_reference_{true};
  bool mission_reference_published_{false};
  bool obstacle_floor_is_placeholder_{true};
  bool target_floor_is_placeholder_{true};

  uint64_t rejected_no_pose_{0};
  uint64_t rejected_pose_gap_{0};
  uint64_t rejected_unknown_frame_{0};
  uint64_t rejected_foreign_uav_{0};
  uint64_t rejected_pose_no_uncertainty_{0};
  uint64_t rejected_pose_unusable_{0};
  uint64_t reported_rejections_{0};
  uint64_t reported_track_switches_{0};
  uint64_t accepted_markers_{0};
  uint64_t accepted_obstacle_batches_{0};
  uint64_t accepted_targets_{0};

  rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<MarkerObservation>::SharedPtr marker_subscription_;
  rclcpp::Subscription<ObstacleArray>::SharedPtr obstacle_subscription_;
  rclcpp::Subscription<TargetTrack>::SharedPtr target_subscription_;

  rclcpp::Publisher<SemanticLandmarkArray>::SharedPtr landmark_publisher_;
  rclcpp::Publisher<ObstacleArray>::SharedPtr obstacle_publisher_;
  rclcpp::Publisher<TargetState>::SharedPtr target_publisher_;
  rclcpp::Publisher<PoseStamped>::SharedPtr mission_reference_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

namespace uav_world_model
{

std::shared_ptr<rclcpp::Node> createWorldModelNode(const rclcpp::NodeOptions & options)
{
  return std::make_shared<WorldModelNode>(options);
}

}  // namespace uav_world_model
