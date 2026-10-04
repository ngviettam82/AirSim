#ifndef UAV_NAVIGATION__CARROT_HPP_
#define UAV_NAVIGATION__CARROT_HPP_

namespace uav_navigation
{

struct Vec3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

bool isFinite(const Vec3 & point);
double distance3(const Vec3 & from, const Vec3 & to);
double horizontalDistance(const Vec3 & from, const Vec3 & to);

struct CarrotLimits
{
  double max_speed = 0.55;
  double max_vertical_speed = 0.45;

  // Separate budgets: a vertical lag must not stop horizontal progress.
  // hypot(0.8, 0.6) = 1.0, so the 3D lead keeps its old ceiling. 0 disables an axis.
  double max_lead_horizontal = 0.8;
  double max_lead_vertical = 0.6;
};

struct CarrotStep
{
  Vec3 setpoint;
  bool clamped_horizontal = false;
  bool clamped_vertical = false;

  bool clamped() const {return clamped_horizontal || clamped_vertical;}
};

/// One streaming tick of the moving setpoint. P6.2 changed where the target comes
/// from, not this: the leash and its stall detector still sit on every path.
CarrotStep advanceCarrot(
  const Vec3 & carrot, const Vec3 & target, const Vec3 & position,
  const CarrotLimits & limits, double dt);

double wrapAngle(double radians);
double stepYaw(double current, double target, double max_rate, double dt);
double yawFromQuaternion(double x, double y, double z, double w);

/// Below this norm the caller sent no orientation, so yaw must be left alone.
bool hasUsableOrientation(double x, double y, double z, double w);

}  // namespace uav_navigation

#endif  // UAV_NAVIGATION__CARROT_HPP_
