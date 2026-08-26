// Downward camera -> body velocity, not a position source.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <uav_interfaces/msg/localization_status.hpp>

#include "uav_localization/optical_flow_scaling.hpp"

namespace uav_localization
{

using geometry_msgs::msg::TwistWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::Range;
using uav_interfaces::msg::LocalizationStatus;

class OpticalFlowAdapterNode : public rclcpp::Node
{
public:
  OpticalFlowAdapterNode()
  : Node("optical_flow_adapter_node")
  {
    uav_id_ = declare_parameter<std::string>("uav_id", "uav0");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    const std::string prefix = "/uav/" + uav_id_;

    limits_.min_features =
      static_cast<size_t>(declare_parameter<int>("min_features", 20));
    limits_.min_height_m = declare_parameter<double>("min_height_m", 0.3);
    limits_.max_height_m = declare_parameter<double>("max_height_m", 25.0);
    limits_.max_speed_mps = declare_parameter<double>("max_speed_mps", 20.0);
    limits_.pixel_noise_px = declare_parameter<double>("pixel_noise_px", 1.0);
    limits_.roll_rate_sign = declare_parameter<double>("roll_rate_sign", 1.0);
    limits_.pitch_rate_sign = declare_parameter<double>("pitch_rate_sign", 1.0);

    max_features_ = declare_parameter<int>("max_features", 150);
    feature_quality_ = declare_parameter<double>("feature_quality", 0.01);
    feature_min_distance_ = declare_parameter<double>("feature_min_distance", 10.0);

    // Trackable displacement ~ half window x 2^levels; frames arrive slowly.
    search_window_ = declare_parameter<int>("search_window_px", 31);
    pyramid_levels_ = declare_parameter<int>("pyramid_levels", 4);
    round_trip_tolerance_px_ = declare_parameter<double>("round_trip_tolerance_px", 1.0);
    input_timeout_ = rclcpp::Duration::from_seconds(
      declare_parameter<double>("input_timeout_sec", 0.5));

    rclcpp::QoS sensor_qos(rclcpp::KeepLast(5));
    sensor_qos.best_effort();

    image_subscription_ = create_subscription<Image>(
      prefix + "/perception/down/image_raw", sensor_qos,
      [this](const Image::SharedPtr msg) {onImage(msg);});
    camera_info_subscription_ = create_subscription<CameraInfo>(
      prefix + "/perception/down/camera_info", sensor_qos,
      [this](const CameraInfo::SharedPtr msg) {
        focal_x_ = msg->k[0];
        focal_y_ = msg->k[4];
      });
    range_subscription_ = create_subscription<Range>(
      prefix + "/localization/range", rclcpp::QoS(10),
      [this](const Range::SharedPtr msg) {
        height_m_ = msg->range;
        last_height_ = now();
      });

    rclcpp::QoS state_qos(rclcpp::KeepLast(10));
    state_qos.best_effort();
    odometry_subscription_ = create_subscription<Odometry>(
      prefix + "/state/odometry_raw", state_qos,
      [this](const Odometry::SharedPtr msg) {
        rates_.roll_rate = msg->twist.twist.angular.x;
        rates_.pitch_rate = msg->twist.twist.angular.y;
        last_rates_ = now();
      });

    velocity_publisher_ = create_publisher<TwistWithCovarianceStamped>(
      prefix + "/localization/flow_velocity", rclcpp::QoS(10));
    status_publisher_ = create_publisher<LocalizationStatus>(
      prefix + "/localization/flow_status", rclcpp::QoS(10));

    RCLCPP_INFO(get_logger(), "optical flow adapter ready for %s", uav_id_.c_str());
  }

private:
  void onImage(const Image::SharedPtr & message)
  {
    cv::Mat gray;
    try {
      gray = cv_bridge::toCvShare(message, "mono8")->image;
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "image conversion failed: %s", error.what());
      return;
    }

    const rclcpp::Time stamp(message->header.stamp);
    if (previous_gray_.empty()) {
      gray.copyTo(previous_gray_);
      previous_stamp_ = stamp;
      return;
    }

    const double dt_sec = (stamp - previous_stamp_).seconds();

    FlowSample sample;
    sample.dt_sec = dt_sec;
    measureFlow(gray, &sample);

    FlowGeometry geometry;
    geometry.fx = focal_x_;
    geometry.fy = focal_y_;
    geometry.height_m = hasFresh(last_height_) ? height_m_ : NAN;

    const BodyRates rates = hasFresh(last_rates_) ? rates_ : BodyRates{};
    const FlowVelocity velocity = velocityFromFlow(sample, geometry, rates, limits_);

    publish(stamp, velocity, sample.tracked_features, hasFresh(last_rates_));

    gray.copyTo(previous_gray_);
    previous_stamp_ = stamp;
  }

  /// Median, not mean: a few mistracks drag the estimate.
  void measureFlow(const cv::Mat & gray, FlowSample * sample)
  {
    std::vector<cv::Point2f> previous_corners;
    cv::goodFeaturesToTrack(
      previous_gray_, previous_corners, max_features_, feature_quality_,
      feature_min_distance_);

    if (previous_corners.empty()) {
      sample->tracked_features = 0;
      return;
    }

    const cv::Size window(search_window_, search_window_);
    std::vector<cv::Point2f> current_corners;
    std::vector<uchar> tracked;
    std::vector<float> error;
    cv::calcOpticalFlowPyrLK(
      previous_gray_, gray, previous_corners, current_corners, tracked, error,
      window, pyramid_levels_);

    // Forward-backward check: mistracks confidently report near-zero motion.
    std::vector<cv::Point2f> returned_corners;
    std::vector<uchar> tracked_back;
    std::vector<float> error_back;
    cv::calcOpticalFlowPyrLK(
      gray, previous_gray_, current_corners, returned_corners, tracked_back, error_back,
      window, pyramid_levels_);

    std::vector<double> displacement_u;
    std::vector<double> displacement_v;
    for (size_t i = 0; i < tracked.size(); ++i) {
      if (tracked[i] == 0 || tracked_back[i] == 0) {
        continue;
      }
      if (cv::norm(previous_corners[i] - returned_corners[i]) > round_trip_tolerance_px_) {
        continue;
      }
      displacement_u.push_back(current_corners[i].x - previous_corners[i].x);
      displacement_v.push_back(current_corners[i].y - previous_corners[i].y);
    }

    sample->tracked_features = displacement_u.size();
    if (displacement_u.empty()) {
      return;
    }
    sample->median_u_px = median(&displacement_u);
    sample->median_v_px = median(&displacement_v);
  }

  static double median(std::vector<double> * values)
  {
    const size_t middle = values->size() / 2;
    std::nth_element(values->begin(), values->begin() + middle, values->end());
    return (*values)[middle];
  }

  bool hasFresh(const rclcpp::Time & stamp) const
  {
    return stamp.nanoseconds() != 0 && (now() - stamp) < input_timeout_;
  }

  void publish(
    const rclcpp::Time & stamp, const FlowVelocity & velocity,
    size_t tracked_features, bool rates_fresh)
  {
    LocalizationStatus status;
    status.header.stamp = stamp;
    status.header.frame_id = base_frame_;
    status.uav_id = uav_id_;
    status.active_source = LocalizationStatus::SOURCE_OPTICAL_FLOW;
    status.is_valid = velocity.valid && rates_fresh;
    // Flow measures speed, never position.
    status.position_uncertainty = -1.0F;
    status.heading_uncertainty = -1.0F;

    if (!rates_fresh) {
      status.quality = LocalizationStatus::QUALITY_BAD;
      status.detail = "no recent body rates, rotation cannot be removed";
    } else if (!velocity.valid) {
      status.quality = LocalizationStatus::QUALITY_BAD;
      status.detail = velocity.reason;
    } else if (tracked_features < 2 * limits_.min_features) {
      status.quality = LocalizationStatus::QUALITY_DEGRADED;
      status.detail = "sparse texture";
    } else {
      status.quality = LocalizationStatus::QUALITY_GOOD;
      status.detail = "";
    }
    status_publisher_->publish(status);

    if (!status.is_valid) {
      return;
    }

    TwistWithCovarianceStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = base_frame_;
    message.twist.twist.linear.x = velocity.forward_mps;
    message.twist.twist.linear.y = velocity.left_mps;
    const double variance = velocity.stddev_mps * velocity.stddev_mps;
    message.twist.covariance[0] = variance;
    message.twist.covariance[7] = variance;
    message.twist.covariance[14] = -1.0;   // vertical speed is not measured here
    velocity_publisher_->publish(message);
  }

  std::string uav_id_;
  std::string base_frame_;
  FlowLimits limits_{};
  BodyRates rates_{};
  int max_features_{150};
  double feature_quality_{0.01};
  double feature_min_distance_{10.0};
  int search_window_{31};
  int pyramid_levels_{4};
  double round_trip_tolerance_px_{1.0};
  double focal_x_{0.0};
  double focal_y_{0.0};
  double height_m_{NAN};
  rclcpp::Duration input_timeout_{0, 0};
  rclcpp::Time last_height_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_rates_{0, 0, RCL_ROS_TIME};
  rclcpp::Time previous_stamp_{0, 0, RCL_ROS_TIME};
  cv::Mat previous_gray_;

  rclcpp::Subscription<Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<Range>::SharedPtr range_subscription_;
  rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Publisher<TwistWithCovarianceStamped>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<LocalizationStatus>::SharedPtr status_publisher_;
};

}  // namespace uav_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_localization::OpticalFlowAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
