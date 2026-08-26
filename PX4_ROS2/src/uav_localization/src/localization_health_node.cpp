// Publishes evidence only; acting on it is uav_safety's job.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <uav_interfaces/msg/localization_status.hpp>

#include "uav_localization/health_checks.hpp"

namespace uav_localization
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using nav_msgs::msg::Odometry;
using uav_interfaces::msg::LocalizationStatus;

class LocalizationHealthNode : public rclcpp::Node
{
public:
  explicit LocalizationHealthNode(const rclcpp::NodeOptions & options)
  : Node("localization_health_node", options)
  {
    uav_id_ = declare_parameter<std::string>("uav_id", "uav0");
    const std::string prefix = "/uav/" + uav_id_;

    jump_.max_speed_mps = declare_parameter<double>("max_speed_mps", 20.0);
    jump_.tolerance_m = declare_parameter<double>("jump_tolerance_m", 0.2);
    jump_.noise_sigmas = declare_parameter<double>("jump_noise_sigmas", 2.0);
    jump_.max_sigma_m = declare_parameter<double>("jump_max_sigma_m", 5.0);
    disagreement_.warn_sigma = declare_parameter<double>("disagreement_warn_sigma", 3.0);
    disagreement_.error_sigma = declare_parameter<double>("disagreement_error_sigma", 6.0);
    disagreement_.floor_m = declare_parameter<double>("disagreement_floor_m", 0.2);
    rate_.expected_hz = declare_parameter<double>("expected_fused_hz", 10.0);
    source_timeout_ = rclcpp::Duration::from_seconds(
      declare_parameter<double>("source_timeout_sec", 0.5));

    watch("gps", prefix + "/localization/gps");
    watch("vio", prefix + "/localization/vio");

    fused_subscription_ = create_subscription<Odometry>(
      prefix + "/state/odometry_fused", rclcpp::QoS(10),
      [this](const Odometry::SharedPtr msg) {onFused(*msg);});
    fused_status_subscription_ = create_subscription<LocalizationStatus>(
      prefix + "/state/localization_status", rclcpp::QoS(10),
      [this](const LocalizationStatus::SharedPtr msg) {
        fused_status_ = *msg;
        has_fused_status_ = true;
      });

    diagnostics_publisher_ = create_publisher<DiagnosticArray>(
      prefix + "/diagnostics/localization", rclcpp::QoS(10));
    summary_publisher_ = create_publisher<DiagnosticStatus>(
      prefix + "/state/localization_health", rclcpp::QoS(10));

    report_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0),
      [this]() {report();});

    RCLCPP_INFO(get_logger(), "localization health ready for %s", uav_id_.c_str());
  }

private:
  struct Watched
  {
    std::string name;
    Odometry odometry;
    LocalizationStatus status;
    rclcpp::Time last_update{0, 0, RCL_ROS_TIME};
    bool has_data{false};
    rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription;
    rclcpp::Subscription<LocalizationStatus>::SharedPtr status_subscription;
  };

  void watch(const std::string & name, const std::string & topic_prefix)
  {
    auto watched = std::make_unique<Watched>();
    watched->name = name;
    Watched * raw = watched.get();

    raw->odometry_subscription = create_subscription<Odometry>(
      topic_prefix + "_odometry", rclcpp::QoS(10),
      [this, raw](const Odometry::SharedPtr msg) {
        raw->odometry = *msg;
        raw->last_update = now();
        raw->has_data = true;
      });
    raw->status_subscription = create_subscription<LocalizationStatus>(
      topic_prefix + "_status", rclcpp::QoS(10),
      [raw](const LocalizationStatus::SharedPtr msg) {raw->status = *msg;});

    watched_.push_back(std::move(watched));
  }

  void onFused(const Odometry & fused)
  {
    ++fused_count_;
    // Continuity is a property of the DATA, so it is timed by the stamp the publisher
    // wrote, never by when this callback happened to run.
    const double stamp_sec = rclcpp::Time(fused.header.stamp).nanoseconds() * 1e-9;

    if (has_previous_fused_) {
      const double dt_sec = stamp_sec - previous_fused_stamp_sec_;
      const double step = std::hypot(
        std::hypot(
          fused.pose.pose.position.x - previous_fused_.x,
          fused.pose.pose.position.y - previous_fused_.y),
        fused.pose.pose.position.z - previous_fused_.z);
      const double declared_sigma = has_fused_status_
        ? fused_status_.position_uncertainty
        : -1.0;
      switch (classifyJump(step, dt_sec, declared_sigma, jump_)) {
        case JumpVerdict::Jumped:
          ++jump_count_;
          largest_jump_m_ = std::max(largest_jump_m_, step);
          break;
        case JumpVerdict::CannotJudge:
          ++unjudged_count_;
          break;
        case JumpVerdict::Continuous:
          break;
      }
    }

    previous_fused_ = fused.pose.pose.position;
    previous_fused_stamp_sec_ = stamp_sec;
    has_previous_fused_ = true;
  }

  void report()
  {
    const rclcpp::Time stamp = now();
    DiagnosticArray array;
    array.header.stamp = stamp;

    HealthLevel worst = HealthLevel::Ok;
    const double window_sec = window_started_.nanoseconds() == 0
      ? 0.0
      : (stamp - window_started_).seconds();
    const double fused_hz = window_sec > 0.0 ? fused_count_ / window_sec : 0.0;

    worst = std::max(worst, addRate(&array, fused_hz));
    worst = std::max(worst, addJumps(&array));
    for (const auto & watched : watched_) {
      worst = std::max(worst, addSource(&array, *watched, stamp));
    }
    worst = std::max(worst, addDisagreement(&array, stamp));

    DiagnosticStatus summary;
    summary.name = "localization";
    summary.hardware_id = uav_id_;
    summary.level = toDiagnosticLevel(worst);
    summary.message = worst == HealthLevel::Ok
      ? "localization healthy"
      : "localization degraded, see diagnostics";
    array.status.push_back(summary);

    diagnostics_publisher_->publish(array);
    summary_publisher_->publish(summary);

    window_started_ = stamp;
    fused_count_ = 0;
    jump_count_ = 0;
    unjudged_count_ = 0;
    largest_jump_m_ = 0.0;
  }

  HealthLevel addRate(DiagnosticArray * array, double fused_hz)
  {
    const HealthLevel level = has_fused_status_
      ? classifyRate(fused_hz, rate_)
      : HealthLevel::Error;

    DiagnosticStatus status;
    status.name = "localization: fused output";
    status.hardware_id = uav_id_;
    status.level = toDiagnosticLevel(level);
    status.message = has_fused_status_ ? "publishing" : "no fused solution seen";
    status.values.push_back(keyValue("rate_hz", fused_hz));
    if (has_fused_status_) {
      status.values.push_back(keyValue("valid", fused_status_.is_valid ? 1.0 : 0.0));
      status.values.push_back(
        keyValue("uncertainty_m", fused_status_.position_uncertainty));
      KeyValue detail;
      detail.key = "detail";
      detail.value = fused_status_.detail;
      status.values.push_back(detail);
    }
    array->status.push_back(status);
    return level;
  }

  HealthLevel addJumps(DiagnosticArray * array)
  {
    // O3: a step nobody could judge is not a step nobody took.
    HealthLevel level = HealthLevel::Ok;
    const char * message = "continuous";
    if (jump_count_ > 0) {
      level = HealthLevel::Error;
      message = "fused pose jumped";
    } else if (unjudged_count_ > 0) {
      level = HealthLevel::Warn;
      message = "continuity not judged: source declared no usable uncertainty";
    }

    DiagnosticStatus status;
    status.name = "localization: continuity";
    status.hardware_id = uav_id_;
    status.level = toDiagnosticLevel(level);
    status.message = message;
    status.values.push_back(keyValue("jumps", static_cast<double>(jump_count_)));
    status.values.push_back(keyValue("unjudged", static_cast<double>(unjudged_count_)));
    status.values.push_back(keyValue("largest_jump_m", largest_jump_m_));
    array->status.push_back(status);
    return level;
  }

  HealthLevel addSource(
    DiagnosticArray * array, const Watched & watched, const rclcpp::Time & stamp)
  {
    const bool stale = !watched.has_data ||
      watched.last_update.nanoseconds() == 0 ||
      (stamp - watched.last_update) > source_timeout_;

    HealthLevel level = HealthLevel::Ok;
    std::string message = "healthy";
    if (stale) {
      // A source may be legitimately absent; only total loss errors.
      level = HealthLevel::Warn;
      message = "no recent data";
    } else if (!watched.status.is_valid) {
      level = HealthLevel::Warn;
      message = watched.status.detail.empty() ? "reported invalid" : watched.status.detail;
    }

    DiagnosticStatus status;
    status.name = "localization: source " + watched.name;
    status.hardware_id = uav_id_;
    status.level = toDiagnosticLevel(level);
    status.message = message;
    status.values.push_back(keyValue("valid", watched.status.is_valid ? 1.0 : 0.0));
    status.values.push_back(keyValue("quality", watched.status.quality));
    status.values.push_back(
      keyValue("uncertainty_m", watched.status.position_uncertainty));
    array->status.push_back(status);
    return level;
  }

  /// Catches a source drifting while still claiming precision.
  HealthLevel addDisagreement(DiagnosticArray * array, const rclcpp::Time & stamp)
  {
    std::vector<const Watched *> usable;
    for (const auto & watched : watched_) {
      const bool fresh = watched->has_data &&
        watched->last_update.nanoseconds() != 0 &&
        (stamp - watched->last_update) <= source_timeout_;
      if (fresh && watched->status.is_valid) {
        usable.push_back(watched.get());
      }
    }

    DiagnosticStatus status;
    status.name = "localization: source agreement";
    status.hardware_id = uav_id_;

    status.values.push_back(
      keyValue("sources_compared", static_cast<double>(usable.size())));

    const HealthLevel cross_check = classifyCrossCheck(usable.size());
    if (cross_check != HealthLevel::Ok) {
      status.level = toDiagnosticLevel(cross_check);
      status.message = "single source, cross-check unavailable";
      array->status.push_back(status);
      return cross_check;
    }

    HealthLevel worst = HealthLevel::Ok;
    for (size_t i = 0; i < usable.size(); ++i) {
      for (size_t j = i + 1; j < usable.size(); ++j) {
        const auto & a = usable[i]->odometry.pose.pose.position;
        const auto & b = usable[j]->odometry.pose.pose.position;
        const double gap = std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
        const HealthLevel level = classifyDisagreement(
          gap, usable[i]->status.position_uncertainty,
          usable[j]->status.position_uncertainty, disagreement_);
        worst = std::max(worst, level);
        status.values.push_back(
          keyValue(usable[i]->name + "_vs_" + usable[j]->name + "_m", gap));
      }
    }

    status.level = toDiagnosticLevel(worst);
    status.message = worst == HealthLevel::Ok
      ? "sources agree within their stated uncertainty"
      : "sources disagree by more than they claim; at least one is wrong";
    array->status.push_back(status);
    return worst;
  }

  static KeyValue keyValue(const std::string & key, double value)
  {
    KeyValue pair;
    pair.key = key;
    pair.value = std::to_string(value);
    return pair;
  }

  static uint8_t toDiagnosticLevel(HealthLevel level)
  {
    switch (level) {
      case HealthLevel::Error: return DiagnosticStatus::ERROR;
      case HealthLevel::Warn: return DiagnosticStatus::WARN;
      default: return DiagnosticStatus::OK;
    }
  }

  std::string uav_id_;
  JumpCheck jump_{};
  DisagreementCheck disagreement_{};
  RateCheck rate_{};
  rclcpp::Duration source_timeout_{0, 0};

  std::vector<std::unique_ptr<Watched>> watched_;
  LocalizationStatus fused_status_;
  bool has_fused_status_{false};
  geometry_msgs::msg::Point previous_fused_;
  double previous_fused_stamp_sec_{0.0};
  bool has_previous_fused_{false};
  rclcpp::Time window_started_{0, 0, RCL_ROS_TIME};
  size_t fused_count_{0};
  size_t jump_count_{0};
  size_t unjudged_count_{0};
  double largest_jump_m_{0.0};

  rclcpp::Subscription<Odometry>::SharedPtr fused_subscription_;
  rclcpp::Subscription<LocalizationStatus>::SharedPtr fused_status_subscription_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<DiagnosticStatus>::SharedPtr summary_publisher_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

std::shared_ptr<rclcpp::Node> createLocalizationHealthNode(const rclcpp::NodeOptions & options)
{
  return std::make_shared<LocalizationHealthNode>(options);
}

}  // namespace uav_localization
