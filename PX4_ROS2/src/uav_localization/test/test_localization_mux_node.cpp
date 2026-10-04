// The anchor grace, measured on the running mux: a source that blinks must not
// buy the right to step the pose. The boundary arithmetic is pinned jitter-free
// in test_continuity_offset.cpp (AnchorGrace); these two cases bracket it on the
// wire. This probe fabricates source odometry (R20), hence the isolated domain.
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <uav_interfaces/msg/localization_status.hpp>

#include "uav_localization/localization_mux_node.hpp"

using nav_msgs::msg::Odometry;
using uav_interfaces::msg::LocalizationStatus;
using namespace std::chrono_literals;

namespace
{

constexpr double kSourceTimeoutSec = 0.5;   // shipped value, localization_params.yaml
constexpr double kSourceStddev = 0.10;
constexpr double kStartX = 4.0;
constexpr double kCruiseZ = 2.0;

// Travel the source made while silent: what the mux would step through.
constexpr double kBlackoutTravelM = 1.0;
constexpr double kContinuityTolM = 1e-6;

// Grace runs from the last published pose, staleness from the last source
// message: a loss must outlast both before the anchor may be dropped.
constexpr auto kBeyondGrace =
  std::chrono::milliseconds(static_cast<int>(2000.0 * kSourceTimeoutSec)) + 600ms;

double distance(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::sqrt(
    (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}

class MuxFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();

    rclcpp::NodeOptions options;
    // Ticking faster than shipped only samples the same policy more finely;
    // the grace under test is the shipped source_timeout_sec.
    options.parameter_overrides({
      rclcpp::Parameter("use_sim_time", false),
      rclcpp::Parameter("publish_rate_hz", 50.0),
      rclcpp::Parameter("source_timeout_sec", kSourceTimeoutSec),
    });

    node_ = uav_localization::createLocalizationMuxNode(options);
    probe_ = std::make_shared<rclcpp::Node>("localization_mux_probe");

    odometry_publisher_ = probe_->create_publisher<Odometry>(
      "/uav/uav0/localization/vio_odometry", 10);
    status_publisher_ = probe_->create_publisher<LocalizationStatus>(
      "/uav/uav0/localization/vio_status", 10);

    fused_subscription_ = probe_->create_subscription<Odometry>(
      "/uav/uav0/state/odometry_fused", 20,
      [this](const Odometry::SharedPtr message) {fused_.push_back(*message);});
    status_subscription_ = probe_->create_subscription<LocalizationStatus>(
      "/uav/uav0/state/localization_status", 20,
      [this](const LocalizationStatus::SharedPtr message) {
        if (!message->is_valid) {
          last_unavailable_ = rclcpp::Time(message->header.stamp);
        }
      });

    executor_->add_node(node_);
    executor_->add_node(probe_);
  }

  void TearDown() override
  {
    executor_->remove_node(probe_);
    executor_->remove_node(node_);
  }

  void publishSource()
  {
    const rclcpp::Time stamp = probe_->now();

    Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = "odom";
    odometry.child_frame_id = "base_link";
    odometry.pose.pose.position.x = source_x_;
    odometry.pose.pose.position.z = kCruiseZ;
    odometry.pose.pose.orientation.w = 1.0;
    odometry.pose.covariance[0] = kSourceStddev * kSourceStddev;
    odometry_publisher_->publish(odometry);

    LocalizationStatus status;
    status.header.stamp = stamp;
    status.uav_id = "uav0";
    status.active_source = LocalizationStatus::SOURCE_VIO;
    status.quality = LocalizationStatus::QUALITY_GOOD;
    status.is_valid = true;
    status.position_uncertainty = kSourceStddev;
    status.heading_uncertainty = 0.05F;
    status.detail = "vio";
    status_publisher_->publish(status);
  }

  // Wall time is only the hang valve here; evidence decides.
  bool spinUntil(
    const std::function<bool()> & done, std::chrono::milliseconds timeout, bool stream_source)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (stream_source) {
        publishSource();
      }
      executor_->spin_some();
      if (done()) {
        return true;
      }
      std::this_thread::sleep_for(2ms);
    }
    return done();
  }

  // A first sample lost to discovery would prove nothing.
  bool waitForAPoseTheMuxBelieves()
  {
    return spinUntil([this]() {return fused_.size() >= 3;}, 10s, true);
  }

  /// Stamped, not counted: the mux also reports nothing before it ever has a
  /// source, and a late one of those would fake this whole scenario.
  bool waitUntilTheMuxReportsItHasNothing()
  {
    const rclcpp::Time since = probe_->now();
    return spinUntil([this, since]() {return last_unavailable_ > since;}, 5s, false);
  }

  /// Moves the silent source on, then names the first fused slot that could
  /// carry the result: index, so a late arrival cannot be mistaken for it.
  std::size_t advanceTheSilentSource()
  {
    source_x_ += kBlackoutTravelM;
    return fused_.size();
  }

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> probe_;
  rclcpp::Publisher<Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<LocalizationStatus>::SharedPtr status_publisher_;
  rclcpp::Subscription<Odometry>::SharedPtr fused_subscription_;
  rclcpp::Subscription<LocalizationStatus>::SharedPtr status_subscription_;

  std::vector<Odometry> fused_;
  rclcpp::Time last_unavailable_{0, 0, RCL_ROS_TIME};
  double source_x_{kStartX};
};

TEST_F(MuxFixture, ASourceThatBlinksDoesNotEarnTheRightToStepThePose)
{
  ASSERT_TRUE(waitForAPoseTheMuxBelieves()) << "no fused pose to hold in the first place";
  ASSERT_TRUE(waitUntilTheMuxReportsItHasNothing()) << "the mux never lost the source";

  const geometry_msgs::msg::Point held = fused_.back().pose.pose.position;
  const std::size_t first_after_return = advanceTheSilentSource();

  ASSERT_TRUE(
    spinUntil(
      [this, first_after_return]() {return fused_.size() > first_after_return;}, 5s, true))
    << "the mux never resumed publishing";

  const Odometry & resumed = fused_[first_after_return];
  ASSERT_GT(rclcpp::Time(resumed.header.stamp), last_unavailable_)
    << "measured a pose from before the blackout: this run proves nothing";
  EXPECT_NEAR(distance(resumed.pose.pose.position, held), 0.0, kContinuityTolM)
    << "the source moved " << kBlackoutTravelM << " m while silent and the mux stepped through it";
}

// The positive control for the case above: the same rig, the same measurement,
// and a loss long enough that hiding it would be the unsafe answer.
TEST_F(MuxFixture, ASourceLostBeyondTheGraceIsNotSmoothedOver)
{
  ASSERT_TRUE(waitForAPoseTheMuxBelieves()) << "no fused pose to hold in the first place";

  const auto blackout_started = std::chrono::steady_clock::now();
  ASSERT_TRUE(waitUntilTheMuxReportsItHasNothing()) << "the mux never lost the source";
  // Waiting longer only strengthens the premise, so load cannot flip this claim.
  spinUntil(
    [blackout_started]() {
      return std::chrono::steady_clock::now() - blackout_started > kBeyondGrace;
    },
    kBeyondGrace + 5s, false);
  ASSERT_GT(std::chrono::steady_clock::now() - blackout_started, kBeyondGrace);

  const geometry_msgs::msg::Point held = fused_.back().pose.pose.position;
  const std::size_t first_after_return = advanceTheSilentSource();

  ASSERT_TRUE(
    spinUntil(
      [this, first_after_return]() {return fused_.size() > first_after_return;}, 5s, true))
    << "the mux never resumed publishing";

  const Odometry & resumed = fused_[first_after_return];
  ASSERT_GT(rclcpp::Time(resumed.header.stamp), last_unavailable_)
    << "measured a pose from before the blackout: this run proves nothing";
  EXPECT_NEAR(distance(resumed.pose.pose.position, held), kBlackoutTravelM, kContinuityTolM)
    << "after a long loss the mux must report where the source says it is";
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
