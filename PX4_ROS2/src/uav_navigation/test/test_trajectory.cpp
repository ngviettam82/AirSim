#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "uav_navigation/carrot.hpp"
#include "uav_navigation/trajectory.hpp"

using uav_navigation::distance3;
using uav_navigation::Trajectory;
using uav_navigation::TrajectoryLimits;
using uav_navigation::TrajectorySample;
using uav_navigation::Vec3;
using uav_navigation::wrapAngle;

namespace
{

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// The shipped config: navigation_params.yaml.
TrajectoryLimits shippedLimits()
{
  return TrajectoryLimits{};
}

double norm(const Vec3 & v)
{
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

bool sampleIsFinite(const TrajectorySample & s)
{
  return uav_navigation::isFinite(s.position) && uav_navigation::isFinite(s.velocity) &&
         uav_navigation::isFinite(s.acceleration) && std::isfinite(s.yaw) &&
         std::isfinite(s.yaw_rate);
}

// Everything below measures the SHIPPED POSITIONS. The analytic velocity and
// acceleration fields are blind to steps at segment joins, so an assertion made
// on them can hold while the commanded motion violates the limit.
double maxChordSpeed(const Trajectory & t)
{
  const std::vector<TrajectorySample> & grid = t.grid();
  double peak = 0.0;
  for (std::size_t i = 0; i + 1 < grid.size(); ++i) {
    peak = std::max(peak, distance3(grid[i].position, grid[i + 1].position) / t.gridPeriod());
  }
  return peak;
}

double maxChordVerticalSpeed(const Trajectory & t)
{
  const std::vector<TrajectorySample> & grid = t.grid();
  double peak = 0.0;
  for (std::size_t i = 0; i + 1 < grid.size(); ++i) {
    peak = std::max(peak, std::abs(grid[i + 1].position.z - grid[i].position.z) / t.gridPeriod());
  }
  return peak;
}

double maxChordAcceleration(const Trajectory & t)
{
  const std::vector<TrajectorySample> & grid = t.grid();
  const double h = t.gridPeriod();
  double peak = 0.0;
  for (std::size_t i = 1; i + 1 < grid.size(); ++i) {
    const Vec3 second{
      grid[i + 1].position.x - 2.0 * grid[i].position.x + grid[i - 1].position.x,
      grid[i + 1].position.y - 2.0 * grid[i].position.y + grid[i - 1].position.y,
      grid[i + 1].position.z - 2.0 * grid[i].position.z + grid[i - 1].position.z};
    peak = std::max(peak, norm(second) / (h * h));
  }
  return peak;
}

// The consumer runs at 20 Hz, not at sample_hz; a peak between grid nodes still flies.
double maxStreamSpeed(const Trajectory & t)
{
  constexpr double kStreamTick = 0.05;
  double peak = 0.0;
  for (double time = 0.0; time < t.duration(); time += kStreamTick) {
    const Vec3 from = t.sample(time).position;
    const Vec3 to = t.sample(std::min(time + kStreamTick, t.duration())).position;
    peak = std::max(peak, distance3(from, to) / kStreamTick);
  }
  return peak;
}

double closestApproach(const Trajectory & t, const Vec3 & point)
{
  double best = std::numeric_limits<double>::max();
  for (const TrajectorySample & s : t.grid()) {
    best = std::min(best, distance3(s.position, point));
  }
  return best;
}

}  // namespace

// ---------------------------------------------------------------------------
// Contract of build(): what a caller may rely on before touching the grid.
// ---------------------------------------------------------------------------

TEST(Trajectory, BuildTerminatesOnAnyFiniteInitialYaw)
{
  // 1e17 - 2*pi == 1e17, so a subtract-loop wrap would never return.
  for (const double huge : {1e18, 1e17, 1e10, -1e17}) {
    std::atomic<bool> returned{false};
    std::atomic<double> first_yaw{0.0};
    std::thread worker([&]() {
        std::string reason;
        const Trajectory t =
          Trajectory::build({{0.0, 0.0, 2.0}, {5.0, 0.0, 2.0}}, huge, shippedLimits(), reason);
        first_yaw = t.valid() ? t.grid().front().yaw : 0.0;
        returned = true;
      });
    worker.detach();

    // Wall time is the anti-hang valve only, never the assertion (R21).
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!returned && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_TRUE(returned) << "build() with initial_yaw " << huge << " did not return within 5 s";
    EXPECT_LE(std::abs(first_yaw.load()), M_PI + 1e-9) << "initial_yaw " << huge;
  }
}

TEST(Trajectory, DegenerateInputsFailClosedWithDistinctReasons)
{
  const std::vector<std::vector<Vec3>> refused = {
    {},
    {{kNan, 0.0, 2.0}, {1.0, 0.0, 2.0}},
    {{0.0, 0.0, 2.0}, {kInf, 0.0, 2.0}},
  };

  std::vector<std::string> reasons;
  for (const std::vector<Vec3> & waypoints : refused) {
    std::string reason = "SENTINEL stale";
    const Trajectory t = Trajectory::build(waypoints, 0.0, shippedLimits(), reason);
    EXPECT_FALSE(t.valid());
    EXPECT_TRUE(t.grid().empty());
    EXPECT_DOUBLE_EQ(t.duration(), 0.0);
    EXPECT_NE(reason, "SENTINEL stale") << "build() left the caller's string untouched";
    EXPECT_FALSE(reason.empty());
    reasons.push_back(reason);
  }
  EXPECT_NE(reasons[0], reasons[1]) << "an empty list and a NaN read the same";
}

TEST(Trajectory, WaypointsThatCollapseToOnePointBecomeAValidHold)
{
  std::string reason = "SENTINEL stale";
  // 0.5 mm apart, inside the 1e-3 duplicate filter.
  const Trajectory t = Trajectory::build(
    {{1.0, 2.0, 3.0}, {1.0005, 2.0, 3.0}}, 0.25, shippedLimits(), reason);

  ASSERT_TRUE(t.valid()) << reason;
  EXPECT_EQ(t.grid().size(), 1u);
  EXPECT_DOUBLE_EQ(t.duration(), 0.0);
  EXPECT_NEAR(t.grid().front().yaw, 0.25, 1e-12);
}

TEST(Trajectory, BuildOutputContractIsCompleteOnBothPaths)
{
  std::string reason = "stale failure from the previous goal";
  const Trajectory good =
    Trajectory::build({{0.0, 0.0, 2.0}, {6.0, 0.0, 2.0}}, 0.0, shippedLimits(), reason);
  ASSERT_TRUE(good.valid());
  EXPECT_TRUE(reason.empty()) << "a success left a stale failure reason behind";
  EXPECT_GT(good.peakSpeed(), 0.0);

  std::string second = "SENTINEL stale";
  const Trajectory bad = Trajectory::build({}, 0.0, shippedLimits(), second);
  EXPECT_FALSE(bad.valid());
  EXPECT_TRUE(bad.grid().empty());
  EXPECT_DOUBLE_EQ(bad.duration(), 0.0);
  EXPECT_DOUBLE_EQ(bad.peakSpeed(), 0.0);
  EXPECT_DOUBLE_EQ(bad.peakAcceleration(), 0.0);
}

TEST(Trajectory, LimitsAreValidatedNotSilentlyFloored)
{
  const std::vector<Vec3> waypoints = {{0.0, 0.0, 2.0}, {6.0, 0.0, 2.0}};
  struct Broken
  {
    const char * what;
    TrajectoryLimits limits;
  };

  std::vector<Broken> cases;
  for (const double bad : {0.0, -1.0, kNan}) {
    TrajectoryLimits speed = shippedLimits();
    speed.max_speed = bad;
    cases.push_back({"max_speed", speed});
    TrajectoryLimits accel = shippedLimits();
    accel.max_acceleration = bad;
    cases.push_back({"max_acceleration", accel});
    TrajectoryLimits vertical = shippedLimits();
    vertical.max_vertical_speed = bad;
    cases.push_back({"max_vertical_speed", vertical});
  }
  TrajectoryLimits duration = shippedLimits();
  duration.max_duration = kNan;    // NaN makes every > comparison false: fail-open
  cases.push_back({"max_duration", duration});
  for (const double bad : {0.0, -1.0, kNan, kInf, 1e30}) {
    TrajectoryLimits rate = shippedLimits();
    rate.sample_hz = bad;
    cases.push_back({"sample_hz", rate});
  }
  TrajectoryLimits yaw = shippedLimits();
  yaw.max_yaw_rate = -1.0;
  cases.push_back({"max_yaw_rate", yaw});

  for (const Broken & broken : cases) {
    std::string reason = "SENTINEL stale";
    const Trajectory t = Trajectory::build(waypoints, 0.0, broken.limits, reason);
    EXPECT_FALSE(t.valid()) << broken.what << " was silently accepted";
    EXPECT_NE(reason.find(broken.what), std::string::npos)
      << "reason \"" << reason << "\" does not name " << broken.what;
  }
}

TEST(Trajectory, EveryShippedSampleIsFiniteOrTheBuildIsInvalid)
{
  // 1e308 passes isFinite() but overflows inside the basis, and std::max swallows NaN.
  for (const std::vector<Vec3> & waypoints :
    std::vector<std::vector<Vec3>>{
      {{-1e308, 0.0, 0.0}, {1e308, 0.0, 0.0}},
      {{0.0, 0.0, 0.0}, {1e308, 0.0, 0.0}}})
  {
    std::string reason;
    const Trajectory t = Trajectory::build(waypoints, 0.0, shippedLimits(), reason);
    if (!t.valid()) {
      EXPECT_FALSE(reason.empty());
      continue;
    }
    for (const TrajectorySample & s : t.grid()) {
      ASSERT_TRUE(sampleIsFinite(s)) << "build() accepted a grid holding NaN or inf";
    }
  }
}

// ---------------------------------------------------------------------------
// The grid itself: shape, time base, endpoints.
// ---------------------------------------------------------------------------

TEST(Trajectory, EndpointsAreExactAndBothEndsRestOnTheShippedGrid)
{
  // Odd coordinates, away from the origin: Vec3{} is what every failure path returns.
  const Vec3 first{1.7, -2.3, 2.9};
  const Vec3 last{8.1, 3.4, 5.2};
  std::string reason;
  const Trajectory t = Trajectory::build({first, last}, 0.0, shippedLimits(), reason);

  ASSERT_TRUE(t.valid()) << reason;
  ASSERT_GT(t.grid().size(), 50u);
  EXPECT_LT(distance3(t.grid().front().position, first), 1e-9);
  EXPECT_LT(distance3(t.grid().back().position, last), 1e-9);
  EXPECT_LT(distance3(t.finalPosition(), last), 1e-9);
  EXPECT_LT(norm(t.grid().front().velocity), 1e-9);
  EXPECT_LT(norm(t.grid().back().velocity), 1e-9);
}

TEST(Trajectory, GridTimeBaseIsUniformAndSampleAgreesAtEveryNode)
{
  std::string reason;
  const Trajectory t =
    Trajectory::build({{1.7, -2.3, 2.9}, {8.1, 3.4, 5.2}}, 0.0, shippedLimits(), reason);
  ASSERT_TRUE(t.valid()) << reason;
  ASSERT_GT(t.grid().size(), 2u);

  EXPECT_NEAR(t.duration(), (t.grid().size() - 1) * t.gridPeriod(), 1e-9)
    << "sample() assumes an evenly spaced grid starting at t=0";

  for (std::size_t i = 0; i < t.grid().size(); ++i) {
    const TrajectorySample at_node = t.sample(i * t.gridPeriod());
    EXPECT_LT(distance3(at_node.position, t.grid()[i].position), 1e-12) << "node " << i;
    EXPECT_NEAR(at_node.yaw, t.grid()[i].yaw, 1e-12) << "node " << i;
  }
}

TEST(Trajectory, GridNeverHasASingleNodeWhenDurationIsPositive)
{
  // 1.5 mm apart: survives the 1e-3 duplicate filter but every segment hits the
  // minimum-duration floor, so duration is positive while the path is a speck.
  std::string reason;
  const Trajectory t = Trajectory::build(
    {{1.0, 2.0, 3.0}, {1.0015, 2.0, 3.0}}, 0.0, shippedLimits(), reason);

  if (t.valid() && t.duration() > 0.0) {
    ASSERT_GE(t.grid().size(), 2u) << "size()-2 underflows and sample() reads out of bounds";
    for (double time = 0.0; time <= t.duration(); time += t.duration() / 7.0) {
      EXPECT_TRUE(sampleIsFinite(t.sample(time)));
    }
  }
}

TEST(Trajectory, SampleOutsideTheRangeHoldsTheEndpointAndPreservesYaw)
{
  std::string reason;
  const Trajectory t = Trajectory::build(
    {{0.0, 0.0, 2.0}, {5.0, 0.0, 2.0}, {5.0, 5.0, 2.0}}, 0.0, shippedLimits(), reason);
  ASSERT_TRUE(t.valid()) << reason;

  const double final_yaw = t.grid().back().yaw;
  ASSERT_GT(std::abs(final_yaw), 0.3) << "an L turn must leave a yaw worth preserving";

  for (const double before : {-1e9, -1.0, -0.0}) {
    const TrajectorySample s = t.sample(before);
    EXPECT_LT(distance3(s.position, t.grid().front().position), 1e-12);
    EXPECT_LT(norm(s.velocity), 1e-12);
    EXPECT_NEAR(s.yaw_rate, 0.0, 1e-12);
  }
  for (const double after : {t.duration(), t.duration() + 1e9}) {
    const TrajectorySample s = t.sample(after);
    EXPECT_LT(distance3(s.position, t.grid().back().position), 1e-12);
    EXPECT_LT(norm(s.velocity), 1e-12);
    EXPECT_NEAR(s.yaw, final_yaw, 1e-12) << "holding still is not a reason to forget the heading";
  }
}

// ---------------------------------------------------------------------------
// Limits, measured from the commanded positions.
// ---------------------------------------------------------------------------

TEST(Trajectory, ChordSpeedRespectsMaxSpeedOnTheGridAndAtTwentyHertz)
{
  std::string reason;
  const Trajectory t =
    Trajectory::build({{0.0, 0.0, 2.0}, {10.0, 0.0, 2.0}}, 0.0, shippedLimits(), reason);
  ASSERT_TRUE(t.valid()) << reason;
  ASSERT_GT(t.grid().size(), 100u);

  EXPECT_LE(maxChordSpeed(t), shippedLimits().max_speed * (1.0 + 1e-6));
  EXPECT_LE(maxStreamSpeed(t), shippedLimits().max_speed * (1.0 + 1e-6));
  // Without this the test would also pass on a trajectory that never moves.
  EXPECT_GT(maxChordSpeed(t), 0.5 * shippedLimits().max_speed);
}

TEST(Trajectory, AccelerationFromTheShippedPositionsRespectsMaxAcceleration)
{
  std::string reason;
  const Trajectory t =
    Trajectory::build({{0.0, 0.0, 2.0}, {10.0, 0.0, 2.0}}, 0.0, shippedLimits(), reason);
  ASSERT_TRUE(t.valid()) << reason;
  ASSERT_GT(t.grid().size(), 100u);

  // Per-segment durations make dP/dt jump at every interior join; the analytic
  // acceleration field cannot see that, so measure the second difference.
  EXPECT_LE(maxChordAcceleration(t), shippedLimits().max_acceleration * (1.0 + 1e-3));
  EXPECT_LE(t.peakAcceleration(), shippedLimits().max_acceleration * (1.0 + 1e-6));
  // Not measuring zero: a 10 m rest-to-rest move needs about v^2/L of acceleration.
  EXPECT_GT(maxChordAcceleration(t), 0.5 * t.peakSpeed() * t.peakSpeed() / 10.0);
}

TEST(Trajectory, ReportedPeaksAgreeWithTheGridTheyDescribe)
{
  std::string reason;
  const Trajectory t =
    Trajectory::build({{0.0, 0.0, 2.0}, {6.0, 4.0, 3.0}}, 0.0, shippedLimits(), reason);
  ASSERT_TRUE(t.valid()) << reason;

  // A reported number that does not describe the shipped grid is worse than none.
  EXPECT_GE(t.peakSpeed(), maxChordSpeed(t) - 1e-6);
  EXPECT_GE(t.peakAcceleration(), maxChordAcceleration(t) * 0.5)
    << "peakAcceleration() understates what the aircraft is commanded to do";
}

TEST(Trajectory, VerticalSpeedLimitBindsAndIsRespected)
{
  TrajectoryLimits limits = shippedLimits();
  limits.max_vertical_speed = 0.3;
  std::string reason;
  const Trajectory t = Trajectory::build({{0.0, 0.0, 2.0}, {0.0, 0.0, 8.0}}, 0.0, limits, reason);
  ASSERT_TRUE(t.valid()) << reason;

  EXPECT_LE(maxChordVerticalSpeed(t), limits.max_vertical_speed * (1.0 + 1e-6));
  EXPECT_GT(maxChordVerticalSpeed(t), 0.9 * limits.max_vertical_speed)
    << "the vertical cap must be the one that binds, or this proves nothing";
  EXPECT_LT(t.peakSpeed(), 0.9 * limits.max_speed);
}

TEST(Trajectory, StretchingRetimesWithoutReshapingThePath)
{
  const std::vector<Vec3> waypoints = {{0.0, 0.0, 2.0}, {5.0, 0.0, 2.0}, {5.0, 5.0, 2.0}};
  // Headroom on acceleration so SPEED is what binds in both runs; at the shipped
  // 1.0 m/s^2 the fast run is acceleration-bound and the ratio would be 1.91.
  TrajectoryLimits quick = shippedLimits();
  quick.max_acceleration = 5.0;
  TrajectoryLimits slow = quick;
  slow.max_speed = quick.max_speed / 2.0;

  std::string reason;
  const Trajectory fast = Trajectory::build(waypoints, 0.0, quick, reason);
  ASSERT_TRUE(fast.valid()) << reason;
  const Trajectory halved = Trajectory::build(waypoints, 0.0, slow, reason);
  ASSERT_TRUE(halved.valid()) << reason;

  EXPECT_NEAR(halved.duration() / fast.duration(), 2.0, 0.05);
  for (int step = 0; step <= 200; ++step) {
    const double u = step / 200.0;
    EXPECT_LT(
      distance3(fast.sample(u * fast.duration()).position,
      halved.sample(u * halved.duration()).position), 1e-2) << "progress " << u;
  }
}

TEST(Trajectory, AZigZagOfTwelveWaypointsActuallyBuilds)
{
  std::vector<Vec3> zigzag;
  for (int i = 0; i < 12; ++i) {
    zigzag.push_back({static_cast<double>(i), (i % 2 == 0) ? -1.0 : 1.0, 2.0});
  }
  std::string reason;
  const Trajectory t = Trajectory::build(zigzag, 0.0, shippedLimits(), reason);

  ASSERT_TRUE(t.valid()) << "a twelve-waypoint path is ordinary planner output: " << reason;
  EXPECT_LE(maxChordSpeed(t), shippedLimits().max_speed * (1.0 + 1e-6));
  EXPECT_LE(maxChordAcceleration(t), shippedLimits().max_acceleration * (1.0 + 1e-3));
}

// ---------------------------------------------------------------------------
// Shape: what the planner above must budget for.
// ---------------------------------------------------------------------------

TEST(Trajectory, ApproximationErrorAgainstThePolylineIsPinnedToNumbers)
{
  std::string reason;
  const Trajectory corner = Trajectory::build(
    {{0.0, 0.0, 2.0}, {5.0, 0.0, 2.0}, {5.0, 5.0, 2.0}}, 0.0, shippedLimits(), reason);
  ASSERT_TRUE(corner.valid()) << reason;

  // |W0 - 2*W1 + W2| / 6 for a 5 m right angle; P6.3 inflation must carry this.
  EXPECT_NEAR(closestApproach(corner, {5.0, 0.0, 2.0}), 5.0 * std::sqrt(2.0) / 6.0, 1e-3);

  const Trajectory ridge = Trajectory::build(
    {{0.0, 0.0, 2.0}, {10.0, 0.0, 6.0}, {20.0, 0.0, 2.0}}, 0.0, shippedLimits(), reason);
  ASSERT_TRUE(ridge.valid()) << reason;
  double highest = 0.0;
  for (const TrajectorySample & s : ridge.grid()) {
    highest = std::max(highest, s.position.z);
  }
  EXPECT_LT(highest, 6.0) << "an interior waypoint is approached, never reached";
  EXPECT_GT(highest, 2.0);
}

// ---------------------------------------------------------------------------
// Yaw.
// ---------------------------------------------------------------------------

TEST(Trajectory, YawStartsAtTheGivenHeadingAndHoldsItWhileStill)
{
  std::string reason;
  // 2.5 rad, not 0: atan2(0,0) is 0 and would hide a yaw that was never set.
  const Trajectory t =
    Trajectory::build({{0.0, 0.0, 2.0}, {10.0, 0.0, 2.0}}, 2.5, shippedLimits(), reason);
  ASSERT_TRUE(t.valid()) << reason;

  EXPECT_NEAR(t.grid().front().yaw, wrapAngle(2.5), 1e-12);
  EXPECT_NEAR(t.grid().front().yaw_rate, 0.0, 1e-12);
}

TEST(Trajectory, YawFollowsTheTravelDirectionOnADiagonal)
{
  std::string reason;
  // Two different diagonal headings: a straight +x leg cannot tell atan2(y,x)
  // from a swapped atan2(x,y), and both would read zero.
  const Trajectory t = Trajectory::build(
    {{0.0, 0.0, 2.0}, {8.66, 5.0, 2.0}, {0.0, 10.0, 2.0}}, 0.5, shippedLimits(), reason);
  ASSERT_TRUE(t.valid()) << reason;

  // Read each leg away from the corner: the nose slews at max_yaw_rate and is
  // meant to lag through the turn itself, so a sample there proves nothing.
  const std::size_t nodes = t.grid().size();
  ASSERT_GT(nodes, 20u);
  const TrajectorySample & early = t.grid()[nodes / 5];
  const TrajectorySample & late = t.grid()[(nodes * 4) / 5];

  for (const TrajectorySample & s : {early, late}) {
    const double travel = std::atan2(s.velocity.y, s.velocity.x);
    // Above the deadband is exactly where yaw is required to follow travel.
    ASSERT_GT(std::hypot(s.velocity.x, s.velocity.y), shippedLimits().yaw_deadband_speed)
      << "sample is not moving";
    EXPECT_LT(std::abs(wrapAngle(s.yaw - travel)), 0.3) << "yaw is not following the path";
  }
  EXPECT_GT(std::abs(wrapAngle(late.yaw - early.yaw)), 1.5)
    << "a constant yaw would pass a one-heading test";
}

TEST(Trajectory, YawRateIsLimitedWrappedAndTheReportedFieldAgrees)
{
  TrajectoryLimits limits = shippedLimits();
  limits.max_speed = 5.0;
  limits.max_acceleration = 8.0;
  limits.max_yaw_rate = 0.5;

  std::string reason;
  // A right-angle turn taken fast, so the raw heading change outruns the cap.
  const Trajectory t = Trajectory::build(
    {{0.0, 0.0, 2.0}, {2.0, 0.0, 2.0}, {2.0, 2.0, 2.0}}, 0.0, limits, reason);
  ASSERT_TRUE(t.valid()) << reason;

  double peak_rate = 0.0;
  for (std::size_t i = 0; i + 1 < t.grid().size(); ++i) {
    const double stepped = std::abs(wrapAngle(t.grid()[i + 1].yaw - t.grid()[i].yaw));
    peak_rate = std::max(peak_rate, stepped / t.gridPeriod());
    EXPECT_LT(stepped, M_PI) << "yaw stepped the long way round at node " << i;
  }
  EXPECT_LE(peak_rate, limits.max_yaw_rate * (1.0 + 1e-6));
  EXPECT_GT(peak_rate, 0.5 * limits.max_yaw_rate) << "the yaw cap never bound: nothing proven";
}

TEST(Trajectory, TheNoseLagsThroughASharpTurnAndSaysSoByHowMuch)
{
  TrajectoryLimits limits = shippedLimits();
  limits.max_speed = 5.0;
  limits.max_acceleration = 8.0;

  std::string reason;
  // A near reversal: the path is not slowed to let the nose keep up, because
  // that would make an ordinary GotoPose outrun goto_timeout_sec.
  const Trajectory t = Trajectory::build(
    {{0.0, 0.0, 2.0}, {4.0, 0.0, 2.0}, {0.0, 0.5, 2.0}}, 0.0, limits, reason);
  ASSERT_TRUE(t.valid()) << reason;

  EXPECT_GT(t.peakYawLag(), 0.2) << "a turn this sharp cannot leave the nose on the path";
  EXPECT_LE(t.peakYawLag(), M_PI + 1e-9);

  double measured = 0.0;
  for (const TrajectorySample & s : t.grid()) {
    if (std::hypot(s.velocity.x, s.velocity.y) < shippedLimits().yaw_deadband_speed) {
      continue;
    }
    const double travel = std::atan2(s.velocity.y, s.velocity.x);
    measured = std::max(measured, std::abs(wrapAngle(travel - s.yaw)));
  }
  EXPECT_NEAR(t.peakYawLag(), measured, 1e-9) << "the reported lag must describe this grid";
}

// ---------------------------------------------------------------------------
// The central invariant, swept.
// ---------------------------------------------------------------------------

TEST(Trajectory, NoFalseAcceptAcrossASeededSweep)
{
  std::mt19937 rng(20260817);   // fixed seed: a red case must be reproducible
  std::uniform_real_distribution<double> horizontal(-30.0, 30.0);
  std::uniform_real_distribution<double> vertical(1.0, 15.0);
  std::uniform_int_distribution<int> count(2, 8);

  // The sweep probes geometry, not the mission time budget: a 30 m box holds
  // paths longer than the shipped speed covers in max_duration.
  TrajectoryLimits roomy = shippedLimits();
  roomy.max_duration = 3000.0;

  std::vector<TrajectoryLimits> limit_sets;
  limit_sets.push_back(roomy);
  TrajectoryLimits slow_vertical = roomy;
  slow_vertical.max_vertical_speed = 0.3;
  limit_sets.push_back(slow_vertical);
  TrajectoryLimits brisk = roomy;
  brisk.max_speed = 4.0;
  brisk.max_acceleration = 3.0;
  limit_sets.push_back(brisk);

  int accepted = 0;
  for (int trial = 0; trial < 200; ++trial) {
    std::vector<Vec3> waypoints;
    const int n = count(rng);
    for (int i = 0; i < n; ++i) {
      waypoints.push_back({horizontal(rng), horizontal(rng), vertical(rng)});
    }
    const TrajectoryLimits & limits = limit_sets[trial % limit_sets.size()];

    std::string reason;
    const Trajectory t = Trajectory::build(waypoints, 0.3, limits, reason);
    if (!t.valid()) {
      EXPECT_FALSE(reason.empty()) << "trial " << trial << " refused without saying why";
      continue;
    }
    ++accepted;
    EXPECT_LE(maxChordSpeed(t), limits.max_speed * (1.0 + 1e-3)) << "trial " << trial;
    EXPECT_LE(maxChordVerticalSpeed(t), limits.max_vertical_speed * (1.0 + 1e-3))
      << "trial " << trial;
    EXPECT_LE(maxChordAcceleration(t), limits.max_acceleration * (1.0 + 1e-2))
      << "trial " << trial;
    for (const TrajectorySample & s : t.grid()) {
      ASSERT_TRUE(sampleIsFinite(s)) << "trial " << trial;
    }
  }
  EXPECT_GT(accepted, 150) << "most ordinary paths must build, or the sweep proves nothing";
}
