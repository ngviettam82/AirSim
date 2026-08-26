// Skeleton: OpenCV HOG, no download (R12); real DNN is P11/Unreal.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/objdetect.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include "uav_perception/object_detector_node.hpp"

namespace uav_perception
{

namespace
{

using sensor_msgs::msg::Image;
using vision_msgs::msg::Detection2D;
using vision_msgs::msg::Detection2DArray;
using vision_msgs::msg::ObjectHypothesisWithPose;

// SVM decision value -> (0,1]; a monotonic squash, not a calibrated probability.
double squashScore(double weight)
{
  return 1.0 / (1.0 + std::exp(-weight));
}

class ObjectDetectorNode : public rclcpp::Node
{
public:
  explicit ObjectDetectorNode(const rclcpp::NodeOptions & options)
  : Node("object_detector_node", options)
  {
    uav_id_ = declare_parameter<std::string>("uav_id", "uav0");
    const std::string camera = declare_parameter<std::string>("camera", "front");
    optical_frame_ = declare_parameter<std::string>(
      "optical_frame", uav_id_ + "/camera_" + camera + "_optical");

    hit_threshold_ = declare_parameter<double>("hit_threshold", 0.0);
    scale_ = declare_parameter<double>("scale", 1.05);
    win_stride_px_ = declare_parameter<int>("win_stride_px", 8);
    group_threshold_ = declare_parameter<double>("group_threshold", 2.0);
    // Caps a heavy detector under the shared image budget (README).
    min_process_period_sec_ = declare_parameter<double>("min_process_period_sec", 0.2);

    hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());

    const std::string prefix = "/uav/" + uav_id_ + "/perception";
    image_subscription_ = create_subscription<Image>(
      prefix + "/" + camera + "/image_raw", rclcpp::SensorDataQoS(),
      [this](const Image::SharedPtr message) {onImage(*message);});

    publisher_ = create_publisher<Detection2DArray>(
      prefix + "/detections", rclcpp::QoS(10));

    RCLCPP_INFO(
      get_logger(), "object detector (HOG skeleton) ready for %s, %s camera",
      uav_id_.c_str(), camera.c_str());
  }

private:
  void onImage(const Image & message)
  {
    if (message.width == 0 || message.height == 0) {
      return;
    }

    const double stamp_sec = rclcpp::Time(message.header.stamp).seconds();
    if (has_last_stamp_ && stamp_sec - last_processed_stamp_sec_ < min_process_period_sec_) {
      return;  // pipeline throughput guard, not a quality gate (see README)
    }

    cv::Mat gray;
    try {
      gray = cv_bridge::toCvCopy(message, "mono8")->image;
    } catch (const cv_bridge::Exception & error) {
      throttledWarning(std::string("cannot convert image: ") + error.what());
      return;
    }

    std::vector<cv::Rect> found;
    std::vector<double> weights;
    try {
      hog_.detectMultiScale(
        gray, found, weights, hit_threshold_, cv::Size(win_stride_px_, win_stride_px_),
        cv::Size(), scale_, group_threshold_, false);
    } catch (const cv::Exception & error) {
      // e.g. image smaller than the 64x128 window; not fatal.
      throttledWarning(std::string("HOG detection failed: ") + error.what());
      found.clear();
      weights.clear();
    }

    publish(message.header, found, weights);
    has_last_stamp_ = true;
    last_processed_stamp_sec_ = stamp_sec;
  }

  void publish(
    const std_msgs::msg::Header & header,
    const std::vector<cv::Rect> & found, const std::vector<double> & weights)
  {
    Detection2DArray array;
    array.header.stamp = header.stamp;
    // frame_id names the optical frame, not the camera link.
    array.header.frame_id = optical_frame_;

    for (std::size_t index = 0; index < found.size(); ++index) {
      const cv::Rect & rect = found[index];
      Detection2D detection;
      detection.header = array.header;
      detection.bbox.center.position.x = rect.x + rect.width / 2.0;
      detection.bbox.center.position.y = rect.y + rect.height / 2.0;
      detection.bbox.size_x = rect.width;
      detection.bbox.size_y = rect.height;
      // 2D-only detector: no depth, results[].pose left at its ROS default.

      ObjectHypothesisWithPose hypothesis;
      hypothesis.hypothesis.class_id = "person";
      hypothesis.hypothesis.score = squashScore(index < weights.size() ? weights[index] : 0.0);
      detection.results.push_back(hypothesis);

      array.detections.push_back(detection);
    }

    publisher_->publish(array);
  }

  void throttledWarning(const std::string & text)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", text.c_str());
  }

  std::string uav_id_;
  std::string optical_frame_;
  double hit_threshold_{0.0};
  double scale_{1.05};
  int win_stride_px_{8};
  double group_threshold_{2.0};
  double min_process_period_sec_{0.2};
  bool has_last_stamp_{false};
  double last_processed_stamp_sec_{0.0};

  cv::HOGDescriptor hog_;

  rclcpp::Subscription<Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<Detection2DArray>::SharedPtr publisher_;
};

}  // namespace

std::shared_ptr<rclcpp::Node> createObjectDetectorNode(const rclcpp::NodeOptions & options)
{
  return std::make_shared<ObjectDetectorNode>(options);
}

}  // namespace uav_perception
