// Continuity is a property of the DATA. These cases drive the real health node and pin
// that its verdict follows the publisher's stamp and the uncertainty the source declared,
// not the moment this machine happened to deliver a message. Fabricates fused odometry
// (R20), hence the isolated domain; a probe id of its own keeps it off the mux test's
// topics on that same domain.
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <uav_interfaces/msg/localization_status.hpp>

#include "uav_localization/localization_health_node.hpp"

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using nav_msgs::msg::Odometry;
using uav_interfaces::msg::LocalizationStatus;
using namespace std::chrono_literals;

namespace
{

constexpr char kProbeId[] = "healthprobe";
constexpr double kStreamPeriodSec = 0.1;      // 10 Hz, the shipped fused rate
constexpr double kDeclaredSigmaM = 0.9;       // measured 2026-08-25, bag uav0_20260825_073235Z
constexpr double kOrdinaryStepM = 0.5;        // 5 m/s of real flight
constexpr double kNoiseStepM = 2.217;         // that flight's worst hover step
constexpr double kTeleportM = 10.0;
constexpr size_t kSampleCount = 20;
constexpr double kReportPeriodSec = 1.0;      // the node's own timer

class HealthFixture : public ::testing::Test
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
      rclcpp::Parameter("uav_id", std::string(kProbeId)),
    });
    node_ = uav_localization::createLocalizationHealthNode(options);
    probe_ = std::make_shared<rclcpp::Node>("localization_health_probe");

    const std::string prefix = std::string("/uav/") + kProbeId;
    odometry_publisher_ = probe_->create_publisher<Odometry>(prefix + "/state/odometry_fused", 20);
    status_publisher_ =
      probe_->create_publisher<LocalizationStatus>(prefix + "/state/localization_status", 20);
    diagnostics_subscription_ = probe_->create_subscription<DiagnosticArray>(
      prefix + "/diagnostics/localization", 20,
      [this](const DiagnosticArray::SharedPtr message) {reports_.push_back(*message);});

    executor_->add_node(node_);
    executor_->add_node(probe_);
  }

  void TearDown() override
  {
    executor_->remove_node(probe_);
    executor_->remove_node(node_);
  }

  void spin(double seconds)
  {
    const auto end = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(seconds);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
      executor_->spin_some(10ms);
    }
  }

  void declareUncertainty(double sigma_m)
  {
    LocalizationStatus status;
    status.header.stamp = probe_->now();
    status.uav_id = kProbeId;
    status.active_source = LocalizationStatus::SOURCE_GPS;
    status.is_valid = sigma_m >= 0.0;
    status.position_uncertainty = sigma_m;
    status_publisher_->publish(status);
    spin(0.3);
  }

  /// Publishes kSampleCount samples spaced kStreamPeriodSec apart IN THEIR STAMPS, as
  /// fast as the machine will send them. Returns the wall seconds it actually took.
  double streamSamples(double step_m, double teleport_at_step_m = 0.0)
  {
    const double base = probe_->now().seconds();
    double travelled = 0.0;
    const auto started = std::chrono::steady_clock::now();
    for (size_t index = 0; index < kSampleCount; ++index) {
      const double advance = (teleport_at_step_m > 0.0 && index == kSampleCount / 2)
        ? teleport_at_step_m
        : step_m;
      travelled += advance;

      Odometry odometry;
      odometry.header.stamp =
        rclcpp::Time(static_cast<int64_t>((base + index * kStreamPeriodSec) * 1e9));
      odometry.header.frame_id = "odom";
      odometry.child_frame_id = "base_link";
      odometry.pose.pose.position.x = travelled;
      odometry.pose.pose.orientation.w = 1.0;
      odometry_publisher_->publish(odometry);
      executor_->spin_some(2ms);
    }
    const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - started;
    return elapsed.count();
  }

  /// Every continuity entry seen since the last clearReports(). The node's counters are
  /// windowed -- they are published once and reset -- so the evidence lives in exactly one
  /// report and reading only the newest one reads a window that was already emptied.
  std::vector<const DiagnosticStatus *> continuityEntries()
  {
    std::vector<const DiagnosticStatus *> found;
    for (const DiagnosticArray & report : reports_) {
      for (const DiagnosticStatus & status : report.status) {
        if (status.name == "localization: continuity") {
          found.push_back(&status);
        }
      }
    }
    return found;
  }

  static double valueOf(const DiagnosticStatus & status, const std::string & key)
  {
    for (const auto & entry : status.values) {
      if (entry.key == key) {
        return std::stod(entry.value);
      }
    }
    return -1.0;
  }

  double peak(const std::string & key)
  {
    double worst = 0.0;
    for (const DiagnosticStatus * status : continuityEntries()) {
      worst = std::max(worst, valueOf(*status, key));
    }
    return worst;
  }

  uint8_t worstLevel()
  {
    uint8_t worst = DiagnosticStatus::OK;
    for (const DiagnosticStatus * status : continuityEntries()) {
      worst = std::max(worst, status->level);
    }
    return worst;
  }

  void clearReports() {reports_.clear();}
  void awaitReport() {spin(kReportPeriodSec * 2.5);}

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> probe_;
  rclcpp::Publisher<Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<LocalizationStatus>::SharedPtr status_publisher_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr diagnostics_subscription_;
  std::vector<DiagnosticArray> reports_;
};

// The discriminating case. Timed by arrival, these twenty samples land in a fraction of
// the two seconds their stamps span, so the allowance collapses to the bare tolerance and
// ordinary flight reads as twenty jumps.
TEST_F(HealthFixture, DeliveryFasterThanTheDataIsNotAJump)
{
  declareUncertainty(kDeclaredSigmaM);
  clearReports();
  const double wall_seconds = streamSamples(kOrdinaryStepM);
  const double stamp_span = (kSampleCount - 1) * kStreamPeriodSec;

  // Positive control on this test's own premise: if the machine were slow enough that
  // arrival matched the stamps, the case would pass without discriminating anything.
  ASSERT_LT(wall_seconds, 0.25 * stamp_span)
    << "FAILED TO MEASURE: delivery took " << wall_seconds << " s for " << stamp_span
    << " s of stamps, so this run never exercised the difference between the two clocks";

  awaitReport();
  ASSERT_FALSE(continuityEntries().empty()) << "FAILED TO MEASURE: no continuity report";
  EXPECT_EQ(worstLevel(), DiagnosticStatus::OK);
  EXPECT_EQ(peak("jumps"), 0.0);
}

TEST_F(HealthFixture, NoiseTheSourceDeclaredIsNotAJump)
{
  declareUncertainty(kDeclaredSigmaM);
  clearReports();
  streamSamples(kNoiseStepM);
  awaitReport();

  ASSERT_FALSE(continuityEntries().empty()) << "FAILED TO MEASURE: no continuity report";
  EXPECT_EQ(worstLevel(), DiagnosticStatus::OK);
  EXPECT_EQ(peak("jumps"), 0.0);
}

// The other side: widening the allowance must not have switched the check off.
TEST_F(HealthFixture, ARealTeleportIsStillReported)
{
  declareUncertainty(kDeclaredSigmaM);
  clearReports();
  streamSamples(kOrdinaryStepM, kTeleportM);
  awaitReport();

  ASSERT_FALSE(continuityEntries().empty()) << "FAILED TO MEASURE: no continuity report";
  EXPECT_EQ(worstLevel(), DiagnosticStatus::ERROR);
  EXPECT_GE(peak("jumps"), 1.0);
  EXPECT_GE(peak("largest_jump_m"), kTeleportM - 0.001);
}

// O3: a source that declares nothing buys silence from neither verdict.
TEST_F(HealthFixture, AnUndeclaredUncertaintyIsReportedAsNotJudged)
{
  declareUncertainty(-1.0);
  clearReports();
  streamSamples(kOrdinaryStepM);
  awaitReport();

  ASSERT_FALSE(continuityEntries().empty()) << "FAILED TO MEASURE: no continuity report";
  EXPECT_EQ(worstLevel(), DiagnosticStatus::WARN);
  EXPECT_EQ(peak("jumps"), 0.0);
  EXPECT_GE(peak("unjudged"), 1.0);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
