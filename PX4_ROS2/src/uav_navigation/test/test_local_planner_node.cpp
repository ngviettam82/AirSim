#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <uav_interfaces/msg/avoidance_advice.hpp>
#include <uav_interfaces/msg/obstacle_array.hpp>
#include <uav_interfaces/msg/trajectory3_d.hpp>

#include "uav_navigation/local_planner_node.hpp"

using nav_msgs::msg::Odometry;
using uav_interfaces::msg::AvoidanceAdvice;
using uav_interfaces::msg::Obstacle;
using uav_interfaces::msg::ObstacleArray;
using uav_interfaces::msg::Trajectory3D;
using uav_interfaces::msg::TrajectoryPoint;
using uav_navigation::Vec3;
using namespace std::chrono_literals;

namespace
{

/// Spins until the predicate holds. Wall time is only the hang guard: the assertion
/// is anchored on the event, never on the clock (rule R21).
template<typename Predicate>
bool spinUntil(
  rclcpp::Executor & executor, Predicate predicate, std::chrono::seconds limit = 10s)
{
  const auto deadline = std::chrono::steady_clock::now() + limit;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    executor.spin_some(50ms);
  }
  return predicate();
}

class LocalPlannerNodeFixture : public ::testing::Test
{
protected:
  void start(bool require_map)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("uav_id", "uav0"),
        rclcpp::Parameter("require_obstacle_feed", require_map),
        rclcpp::Parameter("advise_hz", 20.0)});
    node_ = std::make_shared<uav_navigation::LocalPlannerNode>(options);
    probe_ = std::make_shared<rclcpp::Node>("avoidance_probe");

    advice_.clear();
    advice_subscription_ = probe_->create_subscription<AvoidanceAdvice>(
      "/uav/uav0/planning/avoidance", rclcpp::QoS(10),
      [this](AvoidanceAdvice::ConstSharedPtr message) {advice_.push_back(*message);});

    odometry_publisher_ =
      probe_->create_publisher<Odometry>("/uav/uav0/state/odometry_fused", rclcpp::SensorDataQoS());
    obstacle_publisher_ = probe_->create_publisher<ObstacleArray>(
      "/uav/uav0/world/obstacle_map_local", rclcpp::SensorDataQoS());
    trajectory_publisher_ = probe_->create_publisher<Trajectory3D>(
      "/uav/uav0/planning/trajectory", rclcpp::QoS(1).transient_local());

    executor_.add_node(node_);
    executor_.add_node(probe_);

    // Discovery is an event too: counting subscribers beats sleeping (R21).
    ASSERT_TRUE(
      spinUntil(
        executor_, [this] {
          return odometry_publisher_->get_subscription_count() > 0 &&
          obstacle_publisher_->get_subscription_count() > 0 &&
          trajectory_publisher_->get_subscription_count() > 0 &&
          advice_subscription_->get_publisher_count() > 0;
        }));
  }

  void TearDown() override
  {
    pose_timer_.reset();
    // A test may refuse to start the node on purpose; removing null aborts.
    if (probe_) {
      executor_.remove_node(probe_);
    }
    if (node_) {
      executor_.remove_node(node_);
    }
  }

  /// Real odometry keeps arriving. The advisor now times a source that stops out,
  /// so a one-shot pose would make every test here a race against that timeout.
  void sendPose(double x, double y, double z)
  {
    pose_ = Vec3{x, y, z};
    publishPose();
    if (!pose_timer_) {
      pose_timer_ = probe_->create_wall_timer(50ms, [this]() {publishPose();});
    }
  }

  void stopPose()
  {
    pose_timer_.reset();
  }

  void publishPose()
  {
    Odometry odometry;
    odometry.header.stamp = probe_->now();
    odometry.header.frame_id = "odom";
    odometry.pose.pose.position.x = pose_.x;
    odometry.pose.pose.position.y = pose_.y;
    odometry.pose.pose.position.z = pose_.z;
    odometry.pose.pose.orientation.w = 1.0;
    odometry_publisher_->publish(odometry);
  }

  /// A straight run along x at the given altitude, sampled every 0.25 m.
  void sendTrajectory(double from_x, double to_x, double z, bool valid = true)
  {
    Trajectory3D trajectory;
    trajectory.header.stamp = probe_->now();
    trajectory.header.frame_id = "odom";
    trajectory.uav_id = "uav0";
    trajectory.is_valid = valid;
    for (double x = from_x; x <= to_x + 1e-9; x += 0.25) {
      TrajectoryPoint point;
      point.position.x = x;
      point.position.y = 0.0;
      point.position.z = z;
      trajectory.points.push_back(point);
    }
    trajectory_publisher_->publish(trajectory);
  }

  void sendObstacle(double x, double y, double z, double side)
  {
    ObstacleArray array;
    array.header.stamp = probe_->now();
    array.header.frame_id = "odom";
    Obstacle obstacle;
    obstacle.shape = Obstacle::SHAPE_BOX;
    obstacle.center.x = x;
    obstacle.center.y = y;
    obstacle.center.z = z;
    obstacle.size.x = side;
    obstacle.size.y = side;
    obstacle.size.z = side;
    obstacle.position_uncertainty = 0.0F;
    array.obstacles.push_back(obstacle);
    obstacle_publisher_->publish(array);
  }

  bool waitForAdvice(std::size_t count)
  {
    return spinUntil(executor_, [this, count] {return advice_.size() >= count;});
  }

  /// The newest advice that actually looked at something.
  bool waitForCheckedAdvice()
  {
    return spinUntil(
      executor_, [this] {
        for (const AvoidanceAdvice & entry : advice_) {
          if (entry.checked_horizon_m > 0.0F) {
            return true;
          }
        }
        return false;
      });
  }

  const AvoidanceAdvice & lastChecked() const
  {
    for (auto entry = advice_.rbegin(); entry != advice_.rend(); ++entry) {
      if (entry->checked_horizon_m > 0.0F) {
        return *entry;
      }
    }
    return advice_.back();
  }

  std::shared_ptr<uav_navigation::LocalPlannerNode> node_;
  rclcpp::Node::SharedPtr probe_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::vector<AvoidanceAdvice> advice_;
  Vec3 pose_;
  rclcpp::TimerBase::SharedPtr pose_timer_;
  rclcpp::Subscription<AvoidanceAdvice>::SharedPtr advice_subscription_;
  rclcpp::Publisher<Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<ObstacleArray>::SharedPtr obstacle_publisher_;
  rclcpp::Publisher<Trajectory3D>::SharedPtr trajectory_publisher_;
};

}  // namespace

TEST_F(LocalPlannerNodeFixture, ItSpeaksBeforeItHasAnythingToSayAboutRatherThanGoingSilent)
{
  start(false);

  ASSERT_TRUE(waitForAdvice(1));
  EXPECT_EQ(advice_.front().advice, AvoidanceAdvice::ADVICE_CLEAR);
  EXPECT_FLOAT_EQ(advice_.front().checked_horizon_m, 0.0F)
    << "zero horizon is how it says nothing was checked";
  EXPECT_FALSE(advice_.front().reason.empty());
}

TEST_F(LocalPlannerNodeFixture, AnAdviceAlwaysCarriesTheOdomFrameAndTheUavId)
{
  start(false);

  ASSERT_TRUE(waitForAdvice(1));
  EXPECT_EQ(advice_.front().header.frame_id, "odom");
  EXPECT_EQ(advice_.front().uav_id, "uav0");
}

TEST_F(LocalPlannerNodeFixture, AClearRunAheadIsReportedClearWithARealHorizon)
{
  start(false);
  sendPose(0.0, 0.0, 2.0);
  sendObstacle(0.0, 9.0, 2.0, 0.4);
  sendTrajectory(0.0, 5.0, 2.0);

  ASSERT_TRUE(waitForCheckedAdvice());
  const AvoidanceAdvice & advice = lastChecked();
  EXPECT_EQ(advice.advice, AvoidanceAdvice::ADVICE_CLEAR) << advice.reason;
  EXPECT_GT(advice.checked_horizon_m, 0.0F);
  EXPECT_TRUE(advice.map_fresh);
}

TEST_F(LocalPlannerNodeFixture, AnObstacleOnTheFlownTrajectoryProducesAFiniteEscapePoint)
{
  start(false);
  sendPose(0.0, 0.0, 2.0);
  sendObstacle(3.0, 0.0, 2.0, 1.0);
  sendTrajectory(0.0, 6.0, 2.0);

  ASSERT_TRUE(
    spinUntil(
      executor_, [this] {
        for (const AvoidanceAdvice & entry : advice_) {
          if (entry.advice == AvoidanceAdvice::ADVICE_ESCAPE) {
            return true;
          }
        }
        return false;
      }));

  for (const AvoidanceAdvice & entry : advice_) {
    if (entry.advice == AvoidanceAdvice::ADVICE_ESCAPE) {
      EXPECT_TRUE(std::isfinite(entry.escape_point.x));
      EXPECT_TRUE(std::isfinite(entry.escape_point.z));
      EXPECT_GE(entry.escape_point.z, 2.0) << "it must not duck under the obstacle";
      return;
    }
  }
}

TEST_F(LocalPlannerNodeFixture, AnEscapePointIsNaNOnEveryAdviceThatIsNotAnEscape)
{
  start(false);
  sendPose(0.0, 0.0, 2.0);
  sendObstacle(0.0, 9.0, 2.0, 0.4);
  sendTrajectory(0.0, 5.0, 2.0);

  ASSERT_TRUE(waitForCheckedAdvice());
  for (const AvoidanceAdvice & entry : advice_) {
    if (entry.advice != AvoidanceAdvice::ADVICE_ESCAPE) {
      EXPECT_TRUE(std::isnan(entry.escape_point.x)) << entry.reason;
    }
  }
}

TEST_F(LocalPlannerNodeFixture, AWithdrawnPlanStopsBeingCheckedInsteadOfBeingFlownOn)
{
  start(false);
  sendPose(0.0, 0.0, 2.0);
  sendObstacle(0.0, 9.0, 2.0, 0.4);
  sendTrajectory(0.0, 5.0, 2.0);
  ASSERT_TRUE(waitForCheckedAdvice());

  const std::size_t before = advice_.size();
  sendTrajectory(0.0, 5.0, 2.0, /*valid=*/false);

  ASSERT_TRUE(
    spinUntil(
      executor_, [this, before] {
        for (std::size_t index = before; index < advice_.size(); ++index) {
          if (advice_[index].checked_horizon_m == 0.0F) {
            return true;
          }
        }
        return false;
      }));
}

// Decision 4 of the plan, and the difference between sim and a real airframe.
TEST_F(LocalPlannerNodeFixture, WithTheMapRequiredAMissingFeedHoldsTheAircraft)
{
  start(true);
  sendPose(0.0, 0.0, 2.0);
  sendTrajectory(0.0, 5.0, 2.0);        // no obstacle feed at all

  ASSERT_TRUE(
    spinUntil(
      executor_, [this] {
        for (const AvoidanceAdvice & entry : advice_) {
          if (entry.advice == AvoidanceAdvice::ADVICE_HOLD) {
            return true;
          }
        }
        return false;
      }));

  for (const AvoidanceAdvice & entry : advice_) {
    if (entry.advice == AvoidanceAdvice::ADVICE_HOLD) {
      EXPECT_FALSE(entry.map_fresh);
      EXPECT_FALSE(entry.reason.empty());
      return;
    }
  }
}

TEST_F(LocalPlannerNodeFixture, WithTheMapOptionalAMissingFeedIsSoftenedButNeverSilently)
{
  start(false);
  sendPose(0.0, 0.0, 2.0);
  sendTrajectory(0.0, 5.0, 2.0);        // no obstacle feed at all

  ASSERT_TRUE(waitForAdvice(3));
  for (const AvoidanceAdvice & entry : advice_) {
    EXPECT_NE(entry.advice, AvoidanceAdvice::ADVICE_HOLD);
    if (!entry.map_fresh) {
      EXPECT_FLOAT_EQ(entry.checked_horizon_m, 0.0F)
        << "softening may not claim a horizon it never checked";
    }
  }
}

TEST_F(LocalPlannerNodeFixture, RefusedAvoidanceLimitsStopTheNodeInsteadOfDisarmingIt)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("uav_id", "uav0"),
      rclcpp::Parameter("avoid.horizon_m", 0.0)});

  EXPECT_THROW(
    std::make_shared<uav_navigation::LocalPlannerNode>(options), std::runtime_error);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

// An advisor still clearing the horizon from a pose nobody stands behind is exactly
// how a wall gets declared clear.
TEST_F(LocalPlannerNodeFixture, AnOdometrySourceThatStopsIsNotFlownOnAsAClearHorizon)
{
  start(false);
  sendPose(0.0, 0.0, 2.0);
  sendObstacle(0.0, 9.0, 2.0, 0.4);
  sendTrajectory(0.0, 5.0, 2.0);
  ASSERT_TRUE(waitForCheckedAdvice()) << "it never checked anything to begin with";

  stopPose();
  const std::size_t before = advice_.size();
  ASSERT_TRUE(
    spinUntil(
      executor_, [this, before] {
        for (std::size_t index = before; index < advice_.size(); ++index) {
          if (advice_[index].checked_horizon_m == 0.0F &&
          advice_[index].reason.find("odometry_fused went stale") != std::string::npos)
          {
            return true;
          }
        }
        return false;
      }))
    << "the advisor kept checking on a dead pose: " << advice_.back().reason;
}

// The age lives in the costmap and is only recomputed inside update(). Every tick
// that returns early skips update(), so a frozen age reads as a fresh map forever.
TEST_F(LocalPlannerNodeFixture, TheMapAgeKeepsRunningOnTicksThatCheckNothing)
{
  start(false);
  sendPose(0.0, 0.0, 2.0);
  sendObstacle(0.0, 9.0, 2.0, 0.4);
  sendTrajectory(0.0, 5.0, 2.0);
  ASSERT_TRUE(waitForCheckedAdvice());

  stopPose();
  // The last age update() could have produced: after this tick nothing calls it.
  ASSERT_TRUE(
    spinUntil(
      executor_, [this] {
        return !advice_.empty() &&
        advice_.back().reason.find("odometry_fused went stale") != std::string::npos;
      }));
  const float frozen = advice_.back().map_age_sec;
  ASSERT_TRUE(std::isfinite(frozen)) << "the map was never fed, so nothing can freeze";

  ASSERT_TRUE(
    spinUntil(
      executor_, [this, frozen] {
        return advice_.back().map_age_sec > frozen + 0.5F;
      }))
    << "map_age_sec stood still at " << advice_.back().map_age_sec
    << " s while nothing was updating the map";
}
