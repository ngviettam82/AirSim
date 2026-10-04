// The one position the system may believe (R4). See README.

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <uav_interfaces/msg/localization_status.hpp>

#include "uav_localization/continuity_offset.hpp"
#include "uav_localization/localization_mux_node.hpp"
#include "uav_localization/source_channel.hpp"
#include "uav_localization/source_selector.hpp"

namespace uav_localization
{

using nav_msgs::msg::Odometry;
using uav_interfaces::msg::LocalizationStatus;

class LocalizationMuxNode : public rclcpp::Node
{
public:
  explicit LocalizationMuxNode(const rclcpp::NodeOptions & options)
  : Node("localization_mux_node", options)
  {
    const std::string uav_id = declare_parameter<std::string>("uav_id", "uav0");
    const std::string prefix = "/uav/" + uav_id;

    SourceChannel::Options channel_options;
    channel_options.source_id = LocalizationStatus::SOURCE_FUSED;
    channel_options.uav_id = uav_id;
    channel_options.odometry_topic = prefix + "/state/odometry_fused";
    channel_options.status_topic = prefix + "/state/localization_status";
    channel_options.odom_frame = declare_parameter<std::string>("odom_frame", "odom");
    channel_options.base_frame = declare_parameter<std::string>("base_frame", "base_link");
    channel_options.thresholds.degraded_above_m =
      declare_parameter<double>("degraded_above_m", 0.5);
    channel_options.thresholds.bad_above_m = declare_parameter<double>("bad_above_m", 2.0);
    channel_ = std::make_unique<SourceChannel>(*this, channel_options);

    SelectionPolicy policy;
    policy.switch_margin = declare_parameter<double>("switch_margin", 0.7);
    policy.min_hold_sec = declare_parameter<double>("min_hold_sec", 1.0);
    selector_.configure(policy);

    source_timeout_ = rclcpp::Duration::from_seconds(
      declare_parameter<double>("source_timeout_sec", 0.5));
    continuity_decay_rate_ = declare_parameter<double>("continuity_decay_rate_mps", 0.5);

    addSource(LocalizationStatus::SOURCE_GPS, "gps", prefix + "/localization/gps");
    addSource(LocalizationStatus::SOURCE_VIO, "vio", prefix + "/localization/vio");

    estimator_source_publisher_ = create_publisher<std_msgs::msg::String>(
      prefix + "/state/estimator_source", rclcpp::QoS(10).transient_local());

    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 10.0);

    // One timer, not per-callback: never compare sources at different instants.
    publish_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / publish_rate_hz),
      [this]() {tick();});

    RCLCPP_INFO(get_logger(), "localization mux ready for %s", uav_id.c_str());
  }

private:
  struct Source
  {
    uint8_t id{0};
    std::string name;
    Odometry odometry;
    LocalizationStatus status;
    rclcpp::Time odometry_stamp{0, 0, RCL_ROS_TIME};
    rclcpp::Time status_stamp{0, 0, RCL_ROS_TIME};
    bool has_odometry{false};
    bool has_status{false};
    rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription;
    rclcpp::Subscription<LocalizationStatus>::SharedPtr status_subscription;
  };

  void addSource(uint8_t id, const std::string & name, const std::string & topic_prefix)
  {
    auto source = std::make_unique<Source>();
    source->id = id;
    source->name = name;

    Source * raw = source.get();
    raw->odometry_subscription = create_subscription<Odometry>(
      topic_prefix + "_odometry", rclcpp::QoS(10),
      [this, raw](const Odometry::SharedPtr msg) {
        raw->odometry = *msg;
        raw->odometry_stamp = now();
        raw->has_odometry = true;
      });
    raw->status_subscription = create_subscription<LocalizationStatus>(
      topic_prefix + "_status", rclcpp::QoS(10),
      [this, raw](const LocalizationStatus::SharedPtr msg) {
        raw->status = *msg;
        raw->status_stamp = now();
        raw->has_status = true;
      });

    sources_.push_back(std::move(source));
  }

  bool isFresh(const rclcpp::Time & stamp, const rclcpp::Time & current) const
  {
    return stamp.nanoseconds() != 0 && (current - stamp) < source_timeout_;
  }

  void tick()
  {
    const rclcpp::Time current = now();

    std::vector<SourceSnapshot> snapshots;
    snapshots.reserve(sources_.size());
    for (const auto & source : sources_) {
      SourceSnapshot snapshot;
      snapshot.source_id = source->id;
      snapshot.eligible =
        source->has_odometry && source->has_status && source->status.is_valid &&
        isFresh(source->odometry_stamp, current) && isFresh(source->status_stamp, current);
      snapshot.position_stddev = source->status.position_uncertainty;
      snapshots.push_back(snapshot);
    }

    const uint8_t chosen = selector_.select(snapshots, current.seconds());
    const Source * source = findSource(chosen);

    if (source == nullptr) {
      channel_->publishUnavailable(current, "no usable position source");
      announce("none");
      offset_.clear();
      expireHeldPose(current);
      last_tick_ = current;
      return;
    }

    // Decay first, or the switch leaves one decay step.
    const double elapsed = last_tick_.nanoseconds() == 0 ?
      0.0 : (current - last_tick_).seconds();
    const bool offset_moved = offset_.advance(elapsed, continuity_decay_rate_);

    if (selector_.justSwitched() && has_last_pose_) {
      // Absorb the disagreement; a step reads as sudden real displacement.
      const auto & position = source->odometry.pose.pose.position;
      offset_.anchor(
        {last_pose_.x, last_pose_.y, last_pose_.z},
        {position.x, position.y, position.z});
      RCLCPP_WARN(
        get_logger(), "source -> %s, absorbing %.2f m of disagreement",
        source->name.c_str(), offset_.magnitude());
    }

    SourceReport report;
    report.stamp = current;
    report.pose = source->odometry.pose.pose;
    report.pose.position.x += offset_.value()[0];
    report.pose.position.y += offset_.value()[1];
    report.pose.position.z += offset_.value()[2];
    report.twist = source->odometry.twist.twist;
    report.twist_known = source->odometry.twist.covariance[0] >= 0.0;
    // Decaying the offset slides the pose without touching twist: declare that.
    report.twist_stddev = (offset_moved || offset_.active()) ? continuity_decay_rate_ : -1.0;

    // A known bias, not random error: kept out of covariance.
    report.position_stddev = source->status.position_uncertainty;
    report.heading_stddev = source->status.heading_uncertainty;
    report.detail = describe(*source);

    channel_->publish(report);
    announce(source->name);

    // Silence is silence downstream, whatever refused the pose.
    if (channel_->lastPublishWasValid()) {
      last_pose_ = report.pose.position;
      last_pose_stamp_ = current;
      has_last_pose_ = true;
    } else {
      expireHeldPose(current);
    }
    last_tick_ = current;
  }

  /// A blink keeps the anchor so the pose stays continuous when the source
  /// returns; a real loss drops it, because absorbing then would hide the gap.
  void expireHeldPose(const rclcpp::Time & current)
  {
    if (!has_last_pose_ ||
      anchorOutlivesBlackout((current - last_pose_stamp_).seconds(), source_timeout_.seconds()))
    {
      return;
    }
    has_last_pose_ = false;
    RCLCPP_WARN(
      get_logger(), "no source for %.2f s: dropping the held pose, the next one may step",
      (current - last_pose_stamp_).seconds());
  }

  std::string describe(const Source & source) const
  {
    const double residual = offset_.magnitude();
    if (residual < 0.01) {
      return source.name;
    }
    return source.name + ", continuity offset " + std::to_string(residual) + " m";
  }

  void announce(const std::string & name)
  {
    if (name == announced_source_) {
      return;
    }
    std_msgs::msg::String message;
    message.data = name;
    estimator_source_publisher_->publish(message);
    announced_source_ = name;
  }

  const Source * findSource(uint8_t id) const
  {
    for (const auto & source : sources_) {
      if (source->id == id) {
        return source.get();
      }
    }
    return nullptr;
  }

  std::unique_ptr<SourceChannel> channel_;
  SourceSelector selector_;
  std::vector<std::unique_ptr<Source>> sources_;

  rclcpp::Duration source_timeout_{0, 0};
  double continuity_decay_rate_{0.5};
  ContinuityOffset offset_;
  geometry_msgs::msg::Point last_pose_;
  rclcpp::Time last_pose_stamp_{0, 0, RCL_ROS_TIME};
  bool has_last_pose_{false};
  rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME};
  std::string announced_source_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr estimator_source_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

std::shared_ptr<rclcpp::Node> createLocalizationMuxNode(const rclcpp::NodeOptions & options)
{
  return std::make_shared<LocalizationMuxNode>(options);
}

}  // namespace uav_localization
