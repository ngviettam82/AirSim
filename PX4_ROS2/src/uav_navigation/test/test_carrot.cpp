#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>
#include <thread>

#include <gtest/gtest.h>

#include "uav_navigation/carrot.hpp"

using uav_navigation::advanceCarrot;
using uav_navigation::CarrotLimits;
using uav_navigation::CarrotStep;
using uav_navigation::distance3;
using uav_navigation::hasUsableOrientation;
using uav_navigation::horizontalDistance;
using uav_navigation::stepYaw;
using uav_navigation::Vec3;
using uav_navigation::wrapAngle;
using uav_navigation::yawFromQuaternion;

namespace
{

// The shipped config: 0.55 m/s across, 0.45 m/s up, 0.8 m and 0.6 m of lead.
CarrotLimits shippedLimits()
{
  return CarrotLimits{0.55, 0.45, 0.8, 0.6};
}

constexpr double kTick = 0.05;      // 20 Hz stream

}  // namespace

TEST(Carrot, HorizontalStepNeverExceedsMaxSpeedTimesDt)
{
  const CarrotLimits limits = shippedLimits();
  const Vec3 start{0.0, 0.0, 2.0};
  const Vec3 target{50.0, 50.0, 2.0};

  const Vec3 next = advanceCarrot(start, target, start, limits, kTick).setpoint;
  EXPECT_LE(horizontalDistance(start, next), limits.max_speed * kTick + 1e-9);
  EXPECT_NEAR(horizontalDistance(start, next), limits.max_speed * kTick, 1e-9);
}

TEST(Carrot, VerticalStepUsesItsOwnSlowerCap)
{
  const CarrotLimits limits = shippedLimits();
  const Vec3 start{0.0, 0.0, 0.0};
  const Vec3 target{0.0, 0.0, 50.0};

  const Vec3 next = advanceCarrot(start, target, start, limits, kTick).setpoint;
  EXPECT_NEAR(next.z - start.z, limits.max_vertical_speed * kTick, 1e-9);
  EXPECT_LT(limits.max_vertical_speed, limits.max_speed);
}

TEST(Carrot, DiagonalMotionRespectsBothCapsAtOnce)
{
  const CarrotLimits limits = shippedLimits();
  const Vec3 start{0.0, 0.0, 0.0};
  const Vec3 target{50.0, 0.0, 50.0};

  const Vec3 next = advanceCarrot(start, target, start, limits, kTick).setpoint;
  EXPECT_NEAR(next.x, limits.max_speed * kTick, 1e-9);
  EXPECT_NEAR(next.z, limits.max_vertical_speed * kTick, 1e-9);
}

TEST(Carrot, LastStepLandsExactlyOnTheTargetWithoutOvershoot)
{
  const CarrotLimits limits = shippedLimits();
  const Vec3 target{0.01, 0.0, 2.0};
  const Vec3 start{0.0, 0.0, 2.0};

  const Vec3 next = advanceCarrot(start, target, start, limits, kTick).setpoint;
  EXPECT_DOUBLE_EQ(next.x, target.x);
  EXPECT_DOUBLE_EQ(next.y, target.y);
  EXPECT_DOUBLE_EQ(next.z, target.z);
}

TEST(Carrot, VerticalDescentStopsExactlyOnTheTarget)
{
  const CarrotLimits limits = shippedLimits();
  const Vec3 start{0.0, 0.0, 2.0};
  const Vec3 target{0.0, 0.0, 1.99};

  const Vec3 next = advanceCarrot(start, target, start, limits, kTick).setpoint;
  EXPECT_DOUBLE_EQ(next.z, target.z);
}

TEST(Carrot, ConvergesMonotonicallyAndStaysPutOnArrival)
{
  const CarrotLimits limits = shippedLimits();
  const Vec3 target{3.0, -2.0, 2.5};
  Vec3 carrot{0.0, 0.0, 0.2};
  double previous = distance3(carrot, target);

  for (int tick = 0; tick < 400; ++tick) {
    // A perfectly tracking aircraft, so the leash never interferes.
    const Vec3 next = advanceCarrot(carrot, target, carrot, limits, kTick).setpoint;
    const double remaining = distance3(next, target);
    EXPECT_LE(remaining, previous + 1e-12) << "tick " << tick;
    previous = remaining;
    carrot = next;
  }

  EXPECT_LT(distance3(carrot, target), 1e-9);
  const Vec3 settled = advanceCarrot(carrot, target, carrot, limits, kTick).setpoint;
  EXPECT_LT(distance3(settled, target), 1e-9);
}

TEST(Carrot, ArrivalTimeMatchesTheCommandedSpeed)
{
  const CarrotLimits limits = shippedLimits();
  const Vec3 target{1.5, 0.0, 0.0};
  Vec3 carrot{0.0, 0.0, 0.0};

  int ticks = 0;
  while (distance3(carrot, target) > 1e-9 && ticks < 1000) {
    carrot = advanceCarrot(carrot, target, carrot, limits, kTick).setpoint;
    ++ticks;
  }
  // Derived, not copied: a hardcoded count silently lies when the ceiling moves.
  const int expected = static_cast<int>(std::ceil(1.5 / (limits.max_speed * kTick)));
  EXPECT_GE(ticks, expected - 1);
  EXPECT_LE(ticks, expected + 1);
}

TEST(Carrot, LeashHoldsTheSetpointWhenTheAircraftFallsBehind)
{
  const CarrotLimits limits = shippedLimits();
  const Vec3 lagging{0.0, 0.0, 2.0};
  const Vec3 carrot{limits.max_lead_horizontal, 0.0, 2.0};
  const Vec3 target{50.0, 0.0, 2.0};

  const CarrotStep step = advanceCarrot(carrot, target, lagging, limits, kTick);
  EXPECT_TRUE(step.clamped_horizontal);
  EXPECT_TRUE(step.clamped());
  EXPECT_DOUBLE_EQ(step.setpoint.x, carrot.x);
  EXPECT_DOUBLE_EQ(step.setpoint.y, carrot.y);
  EXPECT_DOUBLE_EQ(step.setpoint.z, carrot.z);
}

TEST(Carrot, LeashSaturatesAtTheBudgetOverManyTicksAndKeepsSayingSo)
{
  const CarrotLimits limits = shippedLimits();
  const Vec3 parked{0.0, 0.0, 2.0};
  const Vec3 target{50.0, 0.0, 2.0};
  Vec3 carrot = parked;

  int clamped_ticks = 0;
  double worst_lead = 0.0;
  for (int tick = 0; tick < 200; ++tick) {
    const CarrotStep step = advanceCarrot(carrot, target, parked, limits, kTick);
    carrot = step.setpoint;
    worst_lead = std::max(worst_lead, horizontalDistance(parked, carrot));
    if (step.clamped()) {
      ++clamped_ticks;
    }
  }

  EXPECT_LE(worst_lead, limits.max_lead_horizontal + 1e-9)
    << "the setpoint ran past its horizontal budget";
  EXPECT_GE(worst_lead, limits.max_lead_horizontal - limits.max_speed * kTick)
    << "the setpoint stopped short of the budget";
  // Once saturated it stays saturated: an aircraft that never moves is the fault case.
  // Derived, not copied: the free run is however long the budget takes to spend.
  const int free_ticks =
    static_cast<int>(std::ceil(limits.max_lead_horizontal / (limits.max_speed * kTick)));
  EXPECT_GT(clamped_ticks, 200 - free_ticks - 1);
}

namespace
{

/// Closed loop against PX4's own position loop: a P controller with K_p 0.95,
/// so the aircraft trails the setpoint by v/K_p. Returns the worst acceleration
/// seen in the emitted carrot sequence.
double peakEmittedAcceleration(double max_speed, double max_vertical_speed)
{
  CarrotLimits limits = shippedLimits();
  limits.max_speed = max_speed;
  limits.max_vertical_speed = max_vertical_speed;

  const double gain = 0.95;             // PX4 MPC_XY_P default, no airframe override
  const Vec3 target{40.0, 0.0, 2.0};
  Vec3 carrot{0.0, 0.0, 2.0};
  Vec3 aircraft = carrot;

  std::vector<double> speeds;
  for (int tick = 0; tick < 400; ++tick) {
    const Vec3 previous = carrot;
    carrot = advanceCarrot(carrot, target, aircraft, limits, kTick).setpoint;
    aircraft.x += gain * (carrot.x - aircraft.x) * kTick;
    aircraft.y += gain * (carrot.y - aircraft.y) * kTick;
    aircraft.z += gain * (carrot.z - aircraft.z) * kTick;
    speeds.push_back(distance3(previous, carrot) / kTick);
  }

  double peak = 0.0;
  for (size_t i = 1; i < speeds.size(); ++i) {
    peak = std::max(peak, std::abs(speeds[i] - speeds[i - 1]) / kTick);
  }
  return peak;
}

}  // namespace

TEST(Carrot, TheShippedCeilingRidesSmoothWhereTheOldOneStepped)
{
  const CarrotLimits shipped = shippedLimits();
  const double calm = peakEmittedAcceleration(shipped.max_speed, shipped.max_vertical_speed);
  const double old_ceiling = peakEmittedAcceleration(1.5, 1.0);

  // No invented threshold: the claim is the ratio, and both ends are measured.
  EXPECT_GT(old_ceiling, 0.0) << "the control never moved, so it proves nothing";
  EXPECT_GT(old_ceiling, 10.0 * calm)
    << "shipped " << calm << " m/s2 vs old ceiling " << old_ceiling << " m/s2";
  RecordProperty("peak_acceleration_shipped", std::to_string(calm));
  RecordProperty("peak_acceleration_old_ceiling", std::to_string(old_ceiling));
  std::cout << "[ EVIDENCE ] closed loop at K_p 0.95: shipped " << calm
            << " m/s2, old ceiling " << old_ceiling << " m/s2" << std::endl;
}

TEST(Carrot, TheShippedDefaultsLeaveLeadBudgetAtTheRealTrackingGain)
{
  const CarrotLimits limits = shippedLimits();
  const double gain = 0.95;

  // The leash is a fault detector; a speed that needs more lead than the budget
  // turns it into a permanent rate limiter and the stall evidence goes with it.
  EXPECT_LT(limits.max_speed / gain, limits.max_lead_horizontal)
    << "horizontal speed needs " << limits.max_speed / gain << " m of lead";
  EXPECT_LT(limits.max_vertical_speed / gain, limits.max_lead_vertical)
    << "vertical speed needs " << limits.max_vertical_speed / gain << " m of lead";
}

TEST(Carrot, AStuckAltitudeDoesNotFreezeTheFlightPath)
{
  const CarrotLimits limits = shippedLimits();
  // Vertical mismatch far past the vertical budget, as a frame offset would give.
  const Vec3 sunken{0.0, 0.0, 0.8};
  const Vec3 target{50.0, 0.0, 3.0};
  Vec3 carrot{0.0, 0.0, 2.0};

  bool vertical_ever_clamped = false;
  for (int tick = 0; tick < 20; ++tick) {
    const CarrotStep step = advanceCarrot(carrot, target, sunken, limits, kTick);
    vertical_ever_clamped = vertical_ever_clamped || step.clamped_vertical;
    carrot = step.setpoint;
  }

  EXPECT_TRUE(vertical_ever_clamped) << "the vertical budget was never the binding one";
  EXPECT_GT(carrot.x, 0.5) << "a vertical lag froze horizontal progress";
  EXPECT_LE(horizontalDistance(sunken, carrot), limits.max_lead_horizontal + 1e-9);
}

TEST(Carrot, VerticalBudgetStopsTheSetpointFromClimbingAwayFromTheAircraft)
{
  const CarrotLimits limits = shippedLimits();
  const Vec3 stuck{0.0, 0.0, 1.0};
  const Vec3 target{0.0, 0.0, 50.0};
  Vec3 carrot = stuck;

  double worst_lead = 0.0;
  for (int tick = 0; tick < 200; ++tick) {
    const CarrotStep step = advanceCarrot(carrot, target, stuck, limits, kTick);
    carrot = step.setpoint;
    worst_lead = std::max(worst_lead, std::abs(carrot.z - stuck.z));
  }
  EXPECT_LE(worst_lead, limits.max_lead_vertical + 1e-9);
}

TEST(Carrot, ConvergedSetpointNeverReportsClamping)
{
  const CarrotLimits limits = shippedLimits();
  // Held far from the aircraft, as a frozen hold point after a frame offset would be.
  const Vec3 aircraft{0.0, 0.0, 1.0};
  const Vec3 carrot{0.0, 0.0, 2.2};

  const CarrotStep step = advanceCarrot(carrot, carrot, aircraft, limits, kTick);
  EXPECT_FALSE(step.clamped()) << "a hold that asks for no motion is not a stall";
  EXPECT_DOUBLE_EQ(step.setpoint.z, carrot.z);
}

TEST(Carrot, LeashStillAllowsMotionBackTowardTheAircraft)
{
  const CarrotLimits limits = shippedLimits();
  const Vec3 position{0.0, 0.0, 2.0};
  const Vec3 carrot{2.0, 0.0, 2.0};
  const Vec3 target{0.0, 0.0, 2.0};

  const Vec3 next = advanceCarrot(carrot, target, position, limits, kTick).setpoint;
  EXPECT_LT(next.x, carrot.x);
}

TEST(Carrot, ZeroLeadDisablesTheLeash)
{
  CarrotLimits limits = shippedLimits();
  limits.max_lead_horizontal = 0.0;
  limits.max_lead_vertical = 0.0;
  const Vec3 lagging{0.0, 0.0, 2.0};
  const Vec3 carrot{5.0, 0.0, 2.0};
  const Vec3 target{50.0, 0.0, 2.0};

  const Vec3 next = advanceCarrot(carrot, target, lagging, limits, kTick).setpoint;
  EXPECT_GT(next.x, carrot.x);
}

TEST(Carrot, NonPositiveOrNonFiniteDtLeavesTheSetpointAlone)
{
  const CarrotLimits limits = shippedLimits();
  const Vec3 carrot{1.0, 2.0, 3.0};
  const Vec3 target{9.0, 9.0, 9.0};

  for (const double dt : {0.0, -0.05, std::numeric_limits<double>::quiet_NaN()}) {
    const Vec3 next = advanceCarrot(carrot, target, carrot, limits, dt).setpoint;
    EXPECT_DOUBLE_EQ(next.x, carrot.x);
    EXPECT_DOUBLE_EQ(next.y, carrot.y);
    EXPECT_DOUBLE_EQ(next.z, carrot.z);
  }
}

TEST(Carrot, NonFiniteTargetLeavesTheSetpointAlone)
{
  const CarrotLimits limits = shippedLimits();
  const Vec3 carrot{1.0, 2.0, 3.0};
  const Vec3 target{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};

  const Vec3 next = advanceCarrot(carrot, target, carrot, limits, kTick).setpoint;
  EXPECT_DOUBLE_EQ(next.x, carrot.x);
  EXPECT_DOUBLE_EQ(next.y, carrot.y);
  EXPECT_DOUBLE_EQ(next.z, carrot.z);
}

TEST(Carrot, HorizontalDistanceIgnoresAltitude)
{
  const Vec3 low{0.0, 0.0, 0.0};
  const Vec3 high{3.0, 4.0, 100.0};
  EXPECT_DOUBLE_EQ(horizontalDistance(low, high), 5.0);
  EXPECT_GT(distance3(low, high), 100.0);
}

TEST(Carrot, WrapAngleKeepsEverythingWithinPi)
{
  EXPECT_NEAR(wrapAngle(3.0 * M_PI), M_PI, 1e-9);
  EXPECT_NEAR(wrapAngle(-3.0 * M_PI), -M_PI, 1e-9);
  EXPECT_NEAR(wrapAngle(0.5), 0.5, 1e-12);
  EXPECT_NEAR(wrapAngle(2.0 * M_PI + 0.25), 0.25, 1e-9);
}

TEST(Carrot, YawSlewTakesTheShortWayAroundPi)
{
  const double current = 3.0;           // just under +pi
  const double target = -3.0;           // just over -pi, 0.28 rad away the short way
  const double next = stepYaw(current, target, 0.5, 0.05);

  EXPECT_GT(next, current);
  EXPECT_LE(std::abs(wrapAngle(next - current)), 0.5 * 0.05 + 1e-12);
  EXPECT_LT(std::abs(wrapAngle(next - target)), std::abs(wrapAngle(current - target)));
}

TEST(Carrot, YawSlewRespectsTheRateLimitAndThenSettles)
{
  double yaw = 0.0;
  const double target = 1.0;
  for (int tick = 0; tick < 5; ++tick) {
    const double next = stepYaw(yaw, target, 0.5, 0.05);
    EXPECT_LE(std::abs(wrapAngle(next - yaw)), 0.5 * 0.05 + 1e-12);
    yaw = next;
  }
  for (int tick = 0; tick < 200; ++tick) {
    yaw = stepYaw(yaw, target, 0.5, 0.05);
  }
  EXPECT_NEAR(yaw, target, 1e-9);
}

TEST(Carrot, YawFromQuaternionMatchesTheKnownCorners)
{
  EXPECT_NEAR(yawFromQuaternion(0.0, 0.0, 0.0, 1.0), 0.0, 1e-9);
  const double half = std::sqrt(0.5);
  EXPECT_NEAR(yawFromQuaternion(0.0, 0.0, half, half), M_PI / 2.0, 1e-9);
  EXPECT_NEAR(yawFromQuaternion(0.0, 0.0, -half, half), -M_PI / 2.0, 1e-9);
}

TEST(Carrot, WrapAngleReturnsForHugeFiniteInputInsteadOfSpinningForever)
{
  // 1e17 - 2*pi == 1e17, so the old subtract loop could never exit.
  for (const double huge : {1e17, -1e17, 1e18, 1e10, -1e10}) {
    std::atomic<bool> returned{false};
    std::atomic<double> result{0.0};
    std::thread worker([&]() {
        result = wrapAngle(huge);
        returned = true;
      });
    worker.detach();

    // Wall time is only the anti-hang valve here, never the assertion (R21).
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!returned && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    ASSERT_TRUE(returned) << "wrapAngle(" << huge << ") did not return within 5 s";
    EXPECT_LE(std::abs(result.load()), M_PI + 1e-12) << "input " << huge;
  }
}

TEST(Carrot, WrapAngleKeepsTheCallerSideOfPi)
{
  EXPECT_NEAR(wrapAngle(3.0 * M_PI), M_PI, 1e-9);
  EXPECT_NEAR(wrapAngle(-3.0 * M_PI), -M_PI, 1e-9);
  EXPECT_NEAR(wrapAngle(M_PI), M_PI, 1e-12);
  EXPECT_NEAR(wrapAngle(-M_PI), -M_PI, 1e-12);
}

TEST(Carrot, AnAllZeroQuaternionMeansNoYawWasRequested)
{
  EXPECT_FALSE(hasUsableOrientation(0.0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(
    hasUsableOrientation(0.0, 0.0, 0.0, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_TRUE(hasUsableOrientation(0.0, 0.0, 0.0, 1.0));
}
