#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <uav_interfaces/msg/obstacle_array.hpp>
#include <uav_interfaces/msg/path3_d.hpp>

#include "uav_navigation/route_planner_node.hpp"

using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;
using uav_interfaces::msg::Obstacle;
using uav_interfaces::msg::ObstacleArray;
using uav_interfaces::msg::Path3D;
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

class RouteNodeFixture : public ::testing::Test
{
protected:
  void start(bool require_map)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("uav_id", "uav0"),
        rclcpp::Parameter("require_obstacle_feed", require_map),
        rclcpp::Parameter("plan_hz", 20.0)});
    node_ = std::make_shared<uav_navigation::RoutePlannerNode>(options);
    probe_ = std::make_shared<rclcpp::Node>("route_probe");

    routes_.clear();
    route_subscription_ = probe_->create_subscription<Path3D>(
      "/uav/uav0/planning/route", rclcpp::QoS(10),
      [this](Path3D::ConstSharedPtr message) {routes_.push_back(*message);});

    odometry_publisher_ =
      probe_->create_publisher<Odometry>("/uav/uav0/state/odometry_fused", rclcpp::SensorDataQoS());
    obstacle_publisher_ = probe_->create_publisher<ObstacleArray>(
      "/uav/uav0/world/obstacle_map_local", rclcpp::SensorDataQoS());
    goal_publisher_ =
      probe_->create_publisher<PoseStamped>("/uav/uav0/planning/route_goal", rclcpp::QoS(1));

    executor_.add_node(node_);
    executor_.add_node(probe_);

    // Discovery is an event too: counting subscribers beats sleeping (R21).
    ASSERT_TRUE(
      spinUntil(
        executor_, [this] {
          return odometry_publisher_->get_subscription_count() > 0 &&
          goal_publisher_->get_subscription_count() > 0 &&
          obstacle_publisher_->get_subscription_count() > 0 &&
          route_subscription_->get_publisher_count() > 0;
        }));
  }

  void TearDown() override
  {
    executor_.remove_node(probe_);
    executor_.remove_node(node_);
  }

  void sendPose(double x, double y)
  {
    Odometry odometry;
    odometry.header.stamp = probe_->now();
    odometry.header.frame_id = "odom";
    odometry.pose.pose.position.x = x;
    odometry.pose.pose.position.y = y;
    odometry.pose.pose.orientation.w = 1.0;
    odometry_publisher_->publish(odometry);
  }

  void sendGoal(double x, double y)
  {
    PoseStamped goal;
    goal.header.stamp = probe_->now();
    goal.header.frame_id = "odom";
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.orientation.w = 1.0;
    goal_publisher_->publish(goal);
  }

  void sendObstacle(double x, double y, double side)
  {
    ObstacleArray array;
    array.header.stamp = probe_->now();
    array.header.frame_id = "odom";
    Obstacle obstacle;
    obstacle.shape = Obstacle::SHAPE_BOX;
    obstacle.center.x = x;
    obstacle.center.y = y;
    obstacle.size.x = side;
    obstacle.size.y = side;
    obstacle.size.z = 2.0;
    obstacle.position_uncertainty = 0.0F;
    array.obstacles.push_back(obstacle);
    obstacle_publisher_->publish(array);
  }

  void sendEmptyMap()
  {
    ObstacleArray array;
    array.header.stamp = probe_->now();
    array.header.frame_id = "odom";
    obstacle_publisher_->publish(array);
  }

  std::shared_ptr<uav_navigation::RoutePlannerNode> node_;
  rclcpp::Node::SharedPtr probe_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<Path3D>::SharedPtr route_subscription_;
  rclcpp::Publisher<Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<ObstacleArray>::SharedPtr obstacle_publisher_;
  rclcpp::Publisher<PoseStamped>::SharedPtr goal_publisher_;
  std::vector<Path3D> routes_;
};

}  // namespace

TEST_F(RouteNodeFixture, AGoalWithAKnownObstacleGivesARouteThatGoesAround)
{
  start(false);
  sendObstacle(0.0, 0.0, 2.0);
  sendPose(-5.0, 0.0);
  sendGoal(5.0, 0.0);

  ASSERT_TRUE(
    spinUntil(
      executor_, [this] {
        return !routes_.empty() && routes_.back().is_valid;
      })) << "no valid route was ever published";

  const Path3D & route = routes_.back();
  EXPECT_EQ(route.header.frame_id, "odom");
  EXPECT_EQ(route.uav_id, "uav0");
  EXPECT_EQ(route.plan_state, Path3D::PLAN_STATE_VALID);
  EXPECT_GE(route.waypoints.size(), 3u) << "a straight line cannot clear this box";
  for (const geometry_msgs::msg::Pose & pose : route.waypoints) {
    EXPECT_GT(std::hypot(pose.position.x, pose.position.y), 1.0)
      << "a waypoint landed inside the obstacle";
  }
}

TEST_F(RouteNodeFixture, AClearWindowStillProducesARoute)
{
  start(false);
  sendEmptyMap();
  sendPose(-5.0, 0.0);
  sendGoal(5.0, 0.0);

  ASSERT_TRUE(
    spinUntil(
      executor_, [this] {
        return !routes_.empty() && routes_.back().is_valid;
      }));
  EXPECT_EQ(routes_.back().planner_name, "a_star");
  EXPECT_EQ(routes_.back().plan_state, Path3D::PLAN_STATE_VALID);
}

TEST_F(RouteNodeFixture, RealFlightRefusesToRouteThroughAMapItNeverReceived)
{
  start(true);   // require_obstacle_feed, the real.launch.py setting
  sendPose(-5.0, 0.0);
  sendGoal(5.0, 0.0);

  ASSERT_TRUE(
    spinUntil(
      executor_, [this] {
        return !routes_.empty();
      }));
  const Path3D & route = routes_.back();
  EXPECT_FALSE(route.is_valid);
  EXPECT_EQ(route.plan_state, Path3D::PLAN_STATE_FAILED);
  EXPECT_NE(route.reason.find("stale or absent"), std::string::npos)
    << "got: " << route.reason;

  // Positive control: the same node routes as soon as a map actually arrives.
  sendObstacle(0.0, 0.0, 2.0);
  sendPose(-5.0, 0.0);
  ASSERT_TRUE(
    spinUntil(
      executor_, [this] {
        return routes_.back().is_valid;
      }));
  EXPECT_EQ(routes_.back().plan_state, Path3D::PLAN_STATE_VALID);
}

TEST_F(RouteNodeFixture, AFailedPlanIsPublishedWithItsReasonRatherThanSilence)
{
  start(false);
  // A wall right across the window: there is no corridor at any heading.
  sendObstacle(2.0, 0.0, 0.5);
  ObstacleArray wall;
  wall.header.stamp = probe_->now();
  wall.header.frame_id = "odom";
  Obstacle bar;
  bar.shape = Obstacle::SHAPE_BOX;
  bar.center.x = 2.0;
  bar.size.x = 0.5;
  bar.size.y = 40.0;
  bar.size.z = 2.0;
  bar.position_uncertainty = 0.0F;
  wall.obstacles.push_back(bar);
  obstacle_publisher_->publish(wall);

  sendPose(0.0, 0.0);
  sendGoal(8.0, 0.0);

  ASSERT_TRUE(
    spinUntil(
      executor_, [this] {
        return !routes_.empty() && !routes_.back().is_valid;
      })) << "a blocked goal must be reported, not left as silence";
  EXPECT_EQ(routes_.back().plan_state, Path3D::PLAN_STATE_FAILED);
  EXPECT_FALSE(routes_.back().reason.empty());
  EXPECT_EQ(routes_.back().planner_name, "a_star")
    << "planner_name must stay the algorithm name, not the failure reason";
}

TEST_F(RouteNodeFixture, NoOdometryYetIsReportedAsNoGoalNotAsAFailure)
{
  start(false);
  sendGoal(5.0, 0.0);  // a goal exists, but odometry_fused never arrived

  ASSERT_TRUE(
    spinUntil(
      executor_, [this] {
        return !routes_.empty();
      })) << "missing odometry must still be reported, not left as silence";
  const Path3D & route = routes_.back();
  EXPECT_FALSE(route.is_valid);
  EXPECT_EQ(route.plan_state, Path3D::PLAN_STATE_NO_GOAL);
  EXPECT_NE(route.reason.find("odometry"), std::string::npos) << "got: " << route.reason;
  EXPECT_TRUE(route.planner_name.empty()) << "no algorithm ever ran";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
