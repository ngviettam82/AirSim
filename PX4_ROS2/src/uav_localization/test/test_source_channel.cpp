#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <uav_interfaces/msg/localization_status.hpp>

#include "uav_localization/source_channel.hpp"

using nav_msgs::msg::Odometry;
using uav_interfaces::msg::LocalizationStatus;
using uav_localization::QualityThresholds;
using uav_localization::SourceChannel;
using uav_localization::SourceReport;
using namespace std::chrono_literals;

namespace
{

constexpr double kControlX = 42.0;

class SourceChannelTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("source_channel_test");

    SourceChannel::Options options;
    options.source_id = LocalizationStatus::SOURCE_GPS;
    options.uav_id = "uav0";
    options.odometry_topic = "/test/odometry";
    options.status_topic = "/test/status";
    options.thresholds = QualityThresholds{0.5, 2.0, 0.5};
    channel_ = std::make_unique<SourceChannel>(*node_, options);

    odometry_subscription_ = node_->create_subscription<Odometry>(
      "/test/odometry", rclcpp::QoS(10),
      [this](const Odometry::SharedPtr msg) {odometry_.push_back(*msg);});
    status_subscription_ = node_->create_subscription<LocalizationStatus>(
      "/test/status", rclcpp::QoS(10),
      [this](const LocalizationStatus::SharedPtr msg) {status_.push_back(*msg);});

    ASSERT_TRUE(
      spinUntil(
        [this]() {
          return odometry_subscription_->get_publisher_count() > 0 &&
                 status_subscription_->get_publisher_count() > 0;
        },
        10s))
      << "a first sample lost to discovery would prove nothing";
  }

  // Wall time is only the hang valve here; evidence decides.
  bool spinUntil(const std::function<bool()> & done, std::chrono::milliseconds timeout = 5s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node_);
      if (done()) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return done();
  }

  bool waitForPublications(std::size_t odometry_count, std::size_t status_count)
  {
    return spinUntil(
      [this, odometry_count, status_count]() {
        return odometry_.size() >= odometry_count && status_.size() >= status_count;
      });
  }

  SourceReport reportWith(double position_stddev)
  {
    SourceReport report;
    report.stamp = rclcpp::Time(100, 0, RCL_ROS_TIME);
    report.pose.position.x = 1.0;
    report.pose.position.y = 2.0;
    report.pose.position.z = 3.0;
    report.pose.orientation.w = 1.0;
    report.position_stddev = position_stddev;
    report.heading_stddev = 0.05;
    return report;
  }

  // Trustworthy and unmistakable, sent on the same writers afterwards.
  SourceReport controlReport()
  {
    SourceReport report = reportWith(0.1);
    report.pose.position.x = kControlX;
    return report;
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<SourceChannel> channel_;
  rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<LocalizationStatus>::SharedPtr status_subscription_;
  std::vector<Odometry> odometry_;
  std::vector<LocalizationStatus> status_;
};

TEST_F(SourceChannelTest, AGoodReportPublishesBothPoseAndStatus)
{
  channel_->publish(reportWith(0.1));
  ASSERT_TRUE(waitForPublications(1U, 1U));

  ASSERT_EQ(odometry_.size(), 1U);
  ASSERT_EQ(status_.size(), 1U);
  EXPECT_TRUE(status_[0].is_valid);
  EXPECT_EQ(status_[0].quality, LocalizationStatus::QUALITY_GOOD);
  EXPECT_EQ(status_[0].active_source, LocalizationStatus::SOURCE_GPS);
  EXPECT_DOUBLE_EQ(odometry_[0].pose.pose.position.x, 1.0);
  EXPECT_EQ(odometry_[0].header.frame_id, "odom");
  EXPECT_EQ(odometry_[0].child_frame_id, "base_link");
}

TEST_F(SourceChannelTest, AnUntrustworthyReportYieldsStatusButNoPose)
{
  channel_->publish(reportWith(5.0));   // beyond bad_above_m
  // Same writers, published after: the control cannot overtake it.
  channel_->publish(controlReport());
  ASSERT_TRUE(waitForPublications(1U, 2U)) << "control: the channel still publishes";

  EXPECT_EQ(odometry_.size(), 1U) << "a pose this uncertain must not reach the mux";
  EXPECT_DOUBLE_EQ(odometry_[0].pose.pose.position.x, kControlX)
    << "the only pose that arrived is the control's";
  EXPECT_FALSE(status_[0].is_valid);
  EXPECT_EQ(status_[0].quality, LocalizationStatus::QUALITY_BAD);
}

TEST_F(SourceChannelTest, UnknownUncertaintyIsTreatedAsUntrustworthy)
{
  channel_->publish(reportWith(-1.0));
  channel_->publish(controlReport());
  ASSERT_TRUE(waitForPublications(1U, 2U)) << "control: the channel still publishes";

  EXPECT_EQ(odometry_.size(), 1U);
  EXPECT_DOUBLE_EQ(odometry_[0].pose.pose.position.x, kControlX);
  EXPECT_FALSE(status_[0].is_valid);
  EXPECT_EQ(status_[0].quality, LocalizationStatus::QUALITY_UNKNOWN);
}

TEST_F(SourceChannelTest, DegradedStillFliesButSaysSo)
{
  channel_->publish(reportWith(1.0));   // between degraded and bad
  ASSERT_TRUE(waitForPublications(1U, 1U));

  ASSERT_EQ(odometry_.size(), 1U);
  ASSERT_EQ(status_.size(), 1U);
  EXPECT_TRUE(status_[0].is_valid);
  EXPECT_EQ(status_[0].quality, LocalizationStatus::QUALITY_DEGRADED);
}

TEST_F(SourceChannelTest, UnavailableIsAnnouncedRatherThanSilent)
{
  channel_->publishUnavailable(rclcpp::Time(100, 0, RCL_ROS_TIME), "tracking lost");
  EXPECT_FALSE(channel_->lastPublishWasValid());

  channel_->publish(controlReport());
  ASSERT_TRUE(waitForPublications(1U, 2U)) << "control: the channel still publishes";

  EXPECT_EQ(odometry_.size(), 1U);
  EXPECT_DOUBLE_EQ(odometry_[0].pose.pose.position.x, kControlX);
  EXPECT_FALSE(status_[0].is_valid);
  EXPECT_EQ(status_[0].detail, "tracking lost");
}

TEST_F(SourceChannelTest, UncertaintyReachesTheCovarianceDiagonal)
{
  channel_->publish(reportWith(0.2));
  ASSERT_TRUE(waitForPublications(1U, 1U));

  ASSERT_EQ(odometry_.size(), 1U);
  EXPECT_NEAR(odometry_[0].pose.covariance[0], 0.04, 1e-9);
  EXPECT_NEAR(odometry_[0].pose.covariance[7], 0.04, 1e-9);
  EXPECT_NEAR(odometry_[0].pose.covariance[14], 0.04, 1e-9);
  EXPECT_NEAR(odometry_[0].pose.covariance[35], 0.0025, 1e-9);
}

TEST_F(SourceChannelTest, MissingHeadingIsMarkedUnknownNotPerfect)
{
  SourceReport report = reportWith(0.2);
  report.heading_stddev = -1.0;
  channel_->publish(report);
  ASSERT_TRUE(waitForPublications(1U, 1U));

  ASSERT_EQ(odometry_.size(), 1U);
  EXPECT_DOUBLE_EQ(odometry_[0].pose.covariance[35], -1.0);
}

TEST_F(SourceChannelTest, AnUnsetTwistIsNotReportedAsAMeasuredZero)
{
  channel_->publish(reportWith(0.2));
  ASSERT_TRUE(waitForPublications(1U, 1U));

  ASSERT_EQ(odometry_.size(), 1U);
  EXPECT_DOUBLE_EQ(odometry_[0].twist.covariance[0], -1.0);
}

TEST_F(SourceChannelTest, AKnownTwistLeavesTheCovarianceUnmarked)
{
  SourceReport report = reportWith(0.2);
  report.twist_known = true;
  report.twist.linear.x = 1.5;
  channel_->publish(report);
  ASSERT_TRUE(waitForPublications(1U, 1U));

  ASSERT_EQ(odometry_.size(), 1U);
  EXPECT_DOUBLE_EQ(odometry_[0].twist.covariance[0], 0.0);
  EXPECT_DOUBLE_EQ(odometry_[0].twist.twist.linear.x, 1.5);
}

TEST_F(SourceChannelTest, AStatedTwistUncertaintyReachesAllThreeLinearAxes)
{
  SourceReport report = reportWith(0.2);
  report.twist_known = true;
  report.twist_stddev = 0.5;   // the mux states this while a continuity offset decays
  channel_->publish(report);
  ASSERT_TRUE(waitForPublications(1U, 1U));

  ASSERT_EQ(odometry_.size(), 1U);
  // Isotropic on purpose: the added motion lives in odom, twist lives in base_link.
  EXPECT_DOUBLE_EQ(odometry_[0].twist.covariance[0], 0.25);
  EXPECT_DOUBLE_EQ(odometry_[0].twist.covariance[7], 0.25);
  EXPECT_DOUBLE_EQ(odometry_[0].twist.covariance[14], 0.25);
}

TEST_F(SourceChannelTest, AnUnknownTwistIgnoresAStatedUncertainty)
{
  SourceReport report = reportWith(0.2);
  report.twist_known = false;
  report.twist_stddev = 0.5;
  channel_->publish(report);
  ASSERT_TRUE(waitForPublications(1U, 1U));

  ASSERT_EQ(odometry_.size(), 1U);
  EXPECT_DOUBLE_EQ(odometry_[0].twist.covariance[0], -1.0);
}

// Contract 2.17 row 4: a non-finite twist covariance must never reach the wire.
// Raw readers test `covariance[0] >= 0.0`, and +inf passes that as "stated".
TEST_F(SourceChannelTest, ANonFiniteTwistStddevIsPublishedAsUnstatedNotAsInfinity)
{
  for (const double stddev : {std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::quiet_NaN()}) {
    odometry_.clear();
    status_.clear();
    SourceReport report = reportWith(0.2);
    report.twist_known = true;
    report.twist.linear.x = 1.5;
    report.twist_stddev = stddev;
    channel_->publish(report);
    ASSERT_TRUE(waitForPublications(1U, 1U));

    ASSERT_EQ(odometry_.size(), 1U);
    EXPECT_TRUE(std::isfinite(odometry_[0].twist.covariance[0]));
    EXPECT_DOUBLE_EQ(odometry_[0].twist.covariance[0], 0.0);
    // The velocity itself is still good: only its uncertainty is unknown.
    EXPECT_DOUBLE_EQ(odometry_[0].twist.twist.linear.x, 1.5);
  }
}

TEST(QualityClassification, ThresholdsSeparateTheThreeBands)
{
  const QualityThresholds thresholds{0.5, 2.0, 0.5};

  EXPECT_EQ(classifyQuality(0.0, thresholds), LocalizationStatus::QUALITY_GOOD);
  EXPECT_EQ(classifyQuality(0.5, thresholds), LocalizationStatus::QUALITY_GOOD);
  EXPECT_EQ(classifyQuality(0.51, thresholds), LocalizationStatus::QUALITY_DEGRADED);
  EXPECT_EQ(classifyQuality(2.0, thresholds), LocalizationStatus::QUALITY_DEGRADED);
  EXPECT_EQ(classifyQuality(2.01, thresholds), LocalizationStatus::QUALITY_BAD);
}

TEST(QualityClassification, NonFiniteUncertaintyIsUnknownNotGood)
{
  const QualityThresholds thresholds{0.5, 2.0, 0.5};

  EXPECT_EQ(classifyQuality(-1.0, thresholds), LocalizationStatus::QUALITY_UNKNOWN);
  EXPECT_EQ(
    classifyQuality(std::numeric_limits<double>::quiet_NaN(), thresholds),
    LocalizationStatus::QUALITY_UNKNOWN);
  EXPECT_EQ(
    classifyQuality(std::numeric_limits<double>::infinity(), thresholds),
    LocalizationStatus::QUALITY_UNKNOWN);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
