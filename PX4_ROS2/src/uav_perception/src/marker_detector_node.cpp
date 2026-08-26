// No attitude gate: solvePnP is full 6-DoF (see README).

#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <uav_interfaces/msg/marker_observation.hpp>

#include "uav_perception/marker_pose.hpp"

namespace
{

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using uav_interfaces::msg::MarkerObservation;

class MarkerDetectorNode : public rclcpp::Node
{
public:
  MarkerDetectorNode()
  : Node("marker_detector_node")
  {
    uav_id_ = declare_parameter<std::string>("uav_id", "uav0");
    marker_size_m_ = declare_parameter<double>("marker_size_m", 0.5);
    saturating_error_px_ = declare_parameter<double>("saturating_error_px", 5.0);
    min_confidence_ = declare_parameter<double>("min_confidence", 0.0);
    optical_frame_ = declare_parameter<std::string>(
      "optical_frame", uav_id_ + "/camera_down_optical");

    const std::string prefix = "/uav/" + uav_id_ + "/perception";
    const std::string camera = declare_parameter<std::string>("camera", "down");

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    image_subscription_ = create_subscription<Image>(
      prefix + "/" + camera + "/image_raw", rclcpp::SensorDataQoS(),
      [this](const Image::SharedPtr message) {onImage(*message);});
    info_subscription_ = create_subscription<CameraInfo>(
      prefix + "/" + camera + "/camera_info", rclcpp::SensorDataQoS(),
      [this](const CameraInfo::SharedPtr message) {onCameraInfo(*message);});

    publisher_ = create_publisher<MarkerObservation>(
      "/uav/" + uav_id_ + "/perception/markers", rclcpp::QoS(10));

    RCLCPP_INFO(
      get_logger(), "marker detector ready for %s, %s camera, %.2f m markers",
      uav_id_.c_str(), camera.c_str(), marker_size_m_);
  }

private:
  void onCameraInfo(const CameraInfo & message)
  {
    intrinsics_ = uav_perception::intrinsicsFrom(message.k);
    if (!intrinsics_.valid()) {
      return;
    }
    camera_matrix_ = (cv::Mat_<double>(3, 3) <<
      intrinsics_.fx, 0.0, intrinsics_.cx,
      0.0, intrinsics_.fy, intrinsics_.cy,
      0.0, 0.0, 1.0);
    distortion_ = message.d.empty() ?
      cv::Mat::zeros(1, 5, CV_64F) :
      cv::Mat(message.d, true).reshape(1, 1).clone();
    has_intrinsics_ = true;
  }

  void onImage(const Image & message)
  {
    // No intrinsics yet: pose would be meaningless.
    if (!has_intrinsics_) {
      throttledWarning("no camera_info yet, not estimating pose");
      return;
    }

    cv::Mat gray;
    try {
      gray = cv_bridge::toCvCopy(message, "mono8")->image;
    } catch (const cv_bridge::Exception & error) {
      throttledWarning(std::string("cannot convert image: ") + error.what());
      return;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(gray, dictionary_, corners, ids);
    if (ids.empty()) {
      return;
    }

    std::vector<cv::Vec3d> rotations;
    std::vector<cv::Vec3d> translations;
    cv::aruco::estimatePoseSingleMarkers(
      corners, marker_size_m_, camera_matrix_, distortion_, rotations, translations);

    for (std::size_t index = 0; index < ids.size(); ++index) {
      publish(message.header, ids[index], corners[index],
        rotations[index], translations[index]);
    }
  }

  double reprojectionError(
    const std::vector<cv::Point2f> & detected,
    const cv::Vec3d & rotation, const cv::Vec3d & translation) const
  {
    const auto model = uav_perception::markerCorners(marker_size_m_);
    std::vector<cv::Point3f> object_points;
    for (const auto & corner : model) {
      object_points.emplace_back(
        static_cast<float>(corner[0]), static_cast<float>(corner[1]),
        static_cast<float>(corner[2]));
    }

    std::vector<cv::Point2f> projected;
    cv::projectPoints(
      object_points, rotation, translation, camera_matrix_, distortion_, projected);

    double total = 0.0;
    for (std::size_t index = 0; index < projected.size() && index < detected.size(); ++index) {
      total += cv::norm(projected[index] - detected[index]);
    }
    return projected.empty() ? 0.0 : total / static_cast<double>(projected.size());
  }

  void publish(
    const std_msgs::msg::Header & header, int marker_id,
    const std::vector<cv::Point2f> & detected,
    const cv::Vec3d & rotation, const cv::Vec3d & translation)
  {
    const double error_px = reprojectionError(detected, rotation, translation);
    const double confidence = uav_perception::confidenceFromReprojection(
      error_px, saturating_error_px_);
    if (confidence < min_confidence_) {
      return;
    }

    MarkerObservation observation;
    observation.header.stamp = header.stamp;
    // frame_id names the optical frame, not the camera link.
    observation.header.frame_id = optical_frame_;
    observation.uav_id = uav_id_;
    observation.marker_id = marker_id;
    observation.family = MarkerObservation::FAMILY_ARUCO;
    observation.size = static_cast<float>(marker_size_m_);
    observation.confidence = static_cast<float>(confidence);

    observation.pose.position.x = translation[0];
    observation.pose.position.y = translation[1];
    observation.pose.position.z = translation[2];

    const auto quaternion = uav_perception::quaternionFromRotationVector(
      rotation[0], rotation[1], rotation[2]);
    observation.pose.orientation.x = quaternion[0];
    observation.pose.orientation.y = quaternion[1];
    observation.pose.orientation.z = quaternion[2];
    observation.pose.orientation.w = quaternion[3];

    publisher_->publish(observation);
  }

  void throttledWarning(const std::string & text)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", text.c_str());
  }

  std::string uav_id_;
  std::string optical_frame_;
  double marker_size_m_{0.5};
  double saturating_error_px_{5.0};
  double min_confidence_{0.0};

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  uav_perception::CameraIntrinsics intrinsics_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_;
  bool has_intrinsics_{false};

  rclcpp::Subscription<Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_subscription_;
  rclcpp::Publisher<MarkerObservation>::SharedPtr publisher_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
