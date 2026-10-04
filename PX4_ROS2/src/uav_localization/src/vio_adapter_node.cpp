// Simulated VIO -> the shared position-source contract. See README.

#include <array>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <utility>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <uav_interfaces/msg/localization_status.hpp>

#include "uav_localization/pose_degrader.hpp"
#include "uav_localization/source_channel.hpp"

namespace uav_localization
{

using geometry_msgs::msg::Point;
using nav_msgs::msg::Odometry;
using uav_interfaces::msg::LocalizationStatus;

namespace
{

/// Body vector -> parent frame, without an Eigen dependency.
Point rotateByQuaternion(
  const geometry_msgs::msg::Quaternion & q, double x, double y, double z)
{
  const double cross_x = q.y * z - q.z * y;
  const double cross_y = q.z * x - q.x * z;
  const double cross_z = q.x * y - q.y * x;

  const double second_x = q.y * cross_z - q.z * cross_y;
  const double second_y = q.z * cross_x - q.x * cross_z;
  const double second_z = q.x * cross_y - q.y * cross_x;

  Point rotated;
  rotated.x = x + 2.0 * (q.w * cross_x + second_x);
  rotated.y = y + 2.0 * (q.w * cross_y + second_y);
  rotated.z = z + 2.0 * (q.w * cross_z + second_z);
  return rotated;
}

}  // namespace

class VioAdapterNode : public rclcpp::Node
{
public:
  VioAdapterNode()
  : Node("vio_adapter_node")
  {
    const std::string uav_id = declare_parameter<std::string>("uav_id", "uav0");
    const std::string prefix = "/uav/" + uav_id;

    SourceChannel::Options options;
    options.source_id = LocalizationStatus::SOURCE_VIO;
    options.uav_id = uav_id;
    options.odometry_topic = prefix + "/localization/vio_odometry";
    options.status_topic = prefix + "/localization/vio_status";
    options.odom_frame = declare_parameter<std::string>("odom_frame", "odom");
    options.base_frame = declare_parameter<std::string>("base_frame", "base_link");
    options.thresholds.degraded_above_m = declare_parameter<double>("degraded_above_m", 0.5);
    options.thresholds.bad_above_m = declare_parameter<double>("bad_above_m", 2.0);
    channel_ = std::make_unique<SourceChannel>(*this, options);

    // Simulator reports the body origin, this far below base_link.
    body_offset_z_ = declare_parameter<double>("body_offset_z", 0.24);

    // Ground truth's all-zero covariance claims certainty nobody measured.
    nominal_position_stddev_ = declare_parameter<double>("nominal_position_stddev", 0.10);
    nominal_heading_stddev_ = declare_parameter<double>("nominal_heading_stddev", 0.02);

    tracking_timeout_ = rclcpp::Duration::from_seconds(
      declare_parameter<double>("tracking_timeout_sec", 0.25));

    DegradeParameters degrade;
    degrade.enabled = declare_parameter<bool>("degrade.enabled", false);
    degrade.white_noise_stddev_m =
      declare_parameter<double>("degrade.white_noise_stddev_m", 0.03);
    degrade.drift.enabled = declare_parameter<bool>("degrade.drift.enabled", true);
    degrade.drift.correlation_time_sec =
      declare_parameter<double>("degrade.drift.correlation_time_sec", 120.0);
    degrade.drift.sigma_m = declare_parameter<double>("degrade.drift.sigma_m", 0.5);
    degrader_.configure(degrade);

    latency_ = rclcpp::Duration::from_seconds(
      declare_parameter<double>("degrade.latency_sec", 0.0));
    declare_parameter<bool>("degrade.force_tracking_loss", false);

    if (degrade.enabled || forcedTrackingLoss()) {
      RCLCPP_WARN(
        get_logger(), "VIO fault injection active: simulation only, never a real flight");
    }

    raw_subscription_ = create_subscription<Odometry>(
      prefix + "/localization/vio_odometry_raw", rclcpp::QoS(20),
      [this](const Odometry::SharedPtr msg) {onRaw(*msg);});

    // One timer serves both the latency queue and the watchdog.
    release_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.01),
      [this]() {releaseDue();});

    RCLCPP_INFO(get_logger(), "vio adapter ready for %s", uav_id.c_str());
  }

private:
  void onRaw(const Odometry & raw)
  {
    // Read live: losing lock mid-flight is the case tested.
    if (forcedTrackingLoss()) {
      // Drop input rather than fake a pose; no stale leak.
      pending_.clear();
      return;
    }
    pending_.emplace_back(now(), raw);
  }

  void releaseDue()
  {
    const rclcpp::Time current = now();

    while (!pending_.empty() && (current - pending_.front().first) >= latency_) {
      publish(pending_.front().second, current);
      pending_.pop_front();
    }

    if (last_publish_.nanoseconds() != 0 &&
      (current - last_publish_) > tracking_timeout_ && channel_->lastPublishWasValid())
    {
      channel_->publishUnavailable(current, "tracking lost");
    }
  }

  void publish(const Odometry & raw, const rclcpp::Time & stamp)
  {
    const double dt_sec = last_publish_.nanoseconds() == 0
      ? 0.0
      : (stamp - last_publish_).seconds();

    const auto & orientation = raw.pose.pose.orientation;
    const auto lever_arm = rotateByQuaternion(orientation, 0.0, 0.0, body_offset_z_);

    const std::array<double, 3> corrected{
      raw.pose.pose.position.x + lever_arm.x,
      raw.pose.pose.position.y + lever_arm.y,
      raw.pose.pose.position.z + lever_arm.z};
    const auto degraded = degrader_.apply(corrected, dt_sec);

    SourceReport report;
    report.stamp = stamp;
    report.pose.position.x = degraded[0];
    report.pose.position.y = degraded[1];
    report.pose.position.z = degraded[2];
    report.pose.orientation = orientation;

    // Lever-arm velocity term omitted: negligible at these rotation rates.
    report.twist = raw.twist.twist;
    report.twist_known = true;

    report.position_stddev = nominal_position_stddev_;
    report.heading_stddev = nominal_heading_stddev_;
    report.detail = degrader_.parameters().enabled ? "simulated degradation active" : "";

    channel_->publish(report);
    last_publish_ = stamp;
  }

  bool forcedTrackingLoss() {return get_parameter("degrade.force_tracking_loss").as_bool();}

  std::unique_ptr<SourceChannel> channel_;
  PoseDegrader degrader_;
  std::deque<std::pair<rclcpp::Time, Odometry>> pending_;

  double body_offset_z_{0.24};
  double nominal_position_stddev_{0.10};
  double nominal_heading_stddev_{0.02};
  rclcpp::Duration latency_{0, 0};
  rclcpp::Duration tracking_timeout_{0, 0};
  rclcpp::Time last_publish_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<Odometry>::SharedPtr raw_subscription_;
  rclcpp::TimerBase::SharedPtr release_timer_;
};

}  // namespace uav_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_localization::VioAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
