#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "uav_perception/object_detector_node.hpp"

using sensor_msgs::msg::Image;
using vision_msgs::msg::Detection2DArray;

namespace
{

// Small enough for fast HOG (P5.7 is a plumbing gate, not a benchmark);
// still above the 64x128 detector window so the normal search path runs.
constexpr int kImageSize = 160;

class ObjectDetectorFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();

    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("use_sim_time", false),
      rclcpp::Parameter("min_process_period_sec", 0.0),  // test speed, not a real gate
    });
    node_ = uav_perception::createObjectDetectorNode(options);
    probe_ = std::make_shared<rclcpp::Node>("object_detector_probe");

    image_publisher_ = probe_->create_publisher<Image>(
      "/uav/uav0/perception/front/image_raw", rclcpp::SensorDataQoS());
    detections_subscription_ = probe_->create_subscription<Detection2DArray>(
      "/uav/uav0/perception/detections", rclcpp::QoS(10),
      [this](const Detection2DArray::SharedPtr message) {
        detections_ = *message;
        detections_received_ = true;
      });

    executor_->add_node(node_);
    executor_->add_node(probe_);
  }

  void TearDown() override
  {
    executor_->remove_node(probe_);
    executor_->remove_node(node_);
  }

  static Image makeImage(const cv::Mat & mat, int32_t stamp_sec, uint32_t stamp_nanosec)
  {
    std_msgs::msg::Header header;
    header.stamp.sec = stamp_sec;
    header.stamp.nanosec = stamp_nanosec;
    return *cv_bridge::CvImage(header, "mono8", mat).toImageMsg();
  }

  // Republishes while waiting: best-effort image QoS can drop the first copy,
  // and a stamp match (not just "any message") rules out a stale delivery.
  bool waitForMatchingDetections(const Image & image, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      image_publisher_->publish(image);
      executor_->spin_some();
      if (detections_received_ &&
        detections_.header.stamp.sec == image.header.stamp.sec &&
        detections_.header.stamp.nanosec == image.header.stamp.nanosec)
      {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> probe_;
  rclcpp::Publisher<Image>::SharedPtr image_publisher_;
  rclcpp::Subscription<Detection2DArray>::SharedPtr detections_subscription_;
  Detection2DArray detections_;
  bool detections_received_{false};
};

}  // namespace

TEST_F(ObjectDetectorFixture, BlankImageDoesNotCrashAndPublishesTheRightStructure)
{
  const cv::Mat blank(kImageSize, kImageSize, CV_8UC1, cv::Scalar(128));
  const Image image = makeImage(blank, 12345, 6789);

  ASSERT_TRUE(waitForMatchingDetections(image, std::chrono::seconds(10)));
  EXPECT_EQ(detections_.header.frame_id, "uav0/camera_front_optical");
  // Structurally valid regardless of count: an empty array is a legitimate result.
  for (const auto & detection : detections_.detections) {
    EXPECT_FALSE(detection.results.empty());
  }
}

TEST_F(ObjectDetectorFixture, FakePersonRectangleDoesNotCrashAndPublishesTheRightStructure)
{
  cv::Mat scene(kImageSize, kImageSize, CV_8UC1, cv::Scalar(200));
  cv::rectangle(scene, cv::Rect(60, 10, 40, 140), cv::Scalar(30), cv::FILLED);
  const Image image = makeImage(scene, 777, 0);

  ASSERT_TRUE(waitForMatchingDetections(image, std::chrono::seconds(10)));
  EXPECT_EQ(detections_.header.frame_id, "uav0/camera_front_optical");
  // Whether HOG fires on this synthetic silhouette is detector quality,
  // out of scope for the P5.7 skeleton gate (see README / plan P5 Sec 1).
  for (const auto & detection : detections_.detections) {
    ASSERT_FALSE(detection.results.empty());
    EXPECT_EQ(detection.results[0].hypothesis.class_id, "person");
    EXPECT_GE(detection.results[0].hypothesis.score, 0.0);
    EXPECT_LE(detection.results[0].hypothesis.score, 1.0);
    EXPECT_EQ(detection.header.frame_id, detections_.header.frame_id);
    EXPECT_EQ(detection.header.stamp.sec, image.header.stamp.sec);
  }
}

TEST_F(ObjectDetectorFixture, DistinctImageStampsProduceDistinctPublishedStamps)
{
  const cv::Mat blank(kImageSize, kImageSize, CV_8UC1, cv::Scalar(90));

  const Image first = makeImage(blank, 100, 0);
  ASSERT_TRUE(waitForMatchingDetections(first, std::chrono::seconds(10)));

  const Image second = makeImage(blank, 200, 0);
  ASSERT_TRUE(waitForMatchingDetections(second, std::chrono::seconds(10)))
    << "the published stamp must track the source image, not a publish-time clock";
}
