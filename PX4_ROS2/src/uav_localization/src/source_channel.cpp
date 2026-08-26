#include "uav_localization/source_channel.hpp"

#include <cmath>
#include <utility>

namespace uav_localization
{

using nav_msgs::msg::Odometry;
using uav_interfaces::msg::LocalizationStatus;

namespace
{

// Diagonal entries of the 6x6 row-major pose covariance.
constexpr size_t kCovarianceX = 0;
constexpr size_t kCovarianceY = 7;
constexpr size_t kCovarianceZ = 14;
constexpr size_t kCovarianceYaw = 35;

// ROS convention: -1 means unknown. Zero would claim certainty.
constexpr double kUnknownCovariance = -1.0;

bool isUsable(double stddev)
{
  return std::isfinite(stddev) && stddev >= 0.0;
}

}  // namespace

uint8_t classifyQuality(double position_stddev, const QualityThresholds & thresholds)
{
  if (!isUsable(position_stddev)) {
    return LocalizationStatus::QUALITY_UNKNOWN;
  }
  if (position_stddev > thresholds.bad_above_m) {
    return LocalizationStatus::QUALITY_BAD;
  }
  if (position_stddev > thresholds.degraded_above_m) {
    return LocalizationStatus::QUALITY_DEGRADED;
  }
  return LocalizationStatus::QUALITY_GOOD;
}

SourceChannel::SourceChannel(rclcpp::Node & node, Options options)
: node_(node), options_(std::move(options))
{
  odometry_publisher_ =
    node_.create_publisher<Odometry>(options_.odometry_topic, rclcpp::QoS(10));
  status_publisher_ =
    node_.create_publisher<LocalizationStatus>(options_.status_topic, rclcpp::QoS(10));
}

void SourceChannel::publish(const SourceReport & report)
{
  const uint8_t quality = classifyQuality(report.position_stddev, options_.thresholds);

  // A source that cannot state its error cannot be weighed.
  const bool trustworthy =
    quality != LocalizationStatus::QUALITY_BAD &&
    quality != LocalizationStatus::QUALITY_UNKNOWN;

  if (!trustworthy) {
    publishStatus(
      report.stamp, quality, false, report.position_stddev, report.heading_stddev,
      report.detail.empty() ? "uncertainty too large to publish a pose" : report.detail);
    last_valid_ = false;
    return;
  }

  Odometry odometry;
  odometry.header.stamp = report.stamp;
  odometry.header.frame_id = options_.odom_frame;
  odometry.child_frame_id = options_.base_frame;
  odometry.pose.pose = report.pose;
  odometry.twist.twist = report.twist;

  const double position_variance = report.position_stddev * report.position_stddev;
  odometry.pose.covariance[kCovarianceX] = position_variance;
  odometry.pose.covariance[kCovarianceY] = position_variance;
  odometry.pose.covariance[kCovarianceZ] = position_variance;

  // An identity quaternion otherwise reads as level and facing north.
  odometry.pose.covariance[kCovarianceYaw] = isUsable(report.heading_stddev)
    ? report.heading_stddev * report.heading_stddev
    : kUnknownCovariance;

  if (!report.twist_known) {
    odometry.twist.covariance[0] = kUnknownCovariance;
  } else if (isUsable(report.twist_stddev)) {
    const double twist_variance = report.twist_stddev * report.twist_stddev;
    odometry.twist.covariance[kCovarianceX] = twist_variance;
    odometry.twist.covariance[kCovarianceY] = twist_variance;
    odometry.twist.covariance[kCovarianceZ] = twist_variance;
  }

  odometry_publisher_->publish(odometry);
  publishStatus(
    report.stamp, quality, true, report.position_stddev, report.heading_stddev, report.detail);

  last_valid_stamp_ = report.stamp;
  last_valid_ = true;
}

void SourceChannel::publishUnavailable(const rclcpp::Time & stamp, const std::string & reason)
{
  publishStatus(stamp, LocalizationStatus::QUALITY_BAD, false, -1.0, -1.0, reason);
  last_valid_ = false;
}

void SourceChannel::publishStatus(
  const rclcpp::Time & stamp, uint8_t quality, bool is_valid,
  double position_stddev, double heading_stddev, const std::string & detail)
{
  LocalizationStatus status;
  status.header.stamp = stamp;
  status.header.frame_id = options_.odom_frame;
  status.uav_id = options_.uav_id;
  status.active_source = options_.source_id;
  status.quality = quality;
  status.is_valid = is_valid;
  status.position_uncertainty =
    isUsable(position_stddev) ? static_cast<float>(position_stddev) : -1.0F;
  status.heading_uncertainty =
    isUsable(heading_stddev) ? static_cast<float>(heading_stddev) : -1.0F;
  status.time_since_update_sec = last_valid_stamp_.nanoseconds() == 0
    ? -1.0F
    : static_cast<float>((stamp - last_valid_stamp_).seconds());
  status.detail = detail;
  status_publisher_->publish(status);
}

}  // namespace uav_localization
