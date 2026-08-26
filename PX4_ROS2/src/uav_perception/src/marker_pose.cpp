#include "uav_perception/marker_pose.hpp"

#include <algorithm>
#include <cmath>

namespace uav_perception
{

bool CameraIntrinsics::valid() const
{
  return fx > 0.0 && fy > 0.0 && cx > 0.0 && cy > 0.0;
}

CameraIntrinsics intrinsicsFrom(const std::array<double, 9> & k)
{
  CameraIntrinsics intrinsics;
  intrinsics.fx = k[0];
  intrinsics.cx = k[2];
  intrinsics.fy = k[4];
  intrinsics.cy = k[5];
  return intrinsics;
}

double confidenceFromReprojection(double error_px, double saturating_error_px)
{
  if (!(error_px >= 0.0) || saturating_error_px <= 0.0) {
    return 0.0;
  }
  const double scaled = 1.0 - error_px / saturating_error_px;
  return std::clamp(scaled, 0.0, 1.0);
}

bool AttitudeQuaternion::valid() const
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(w) &&
         (x * x + y * y + z * z + w * w) > 1e-9;
}

void opticalToBody(
  double optical_x, double optical_y, double optical_z,
  double & body_x, double & body_y, double & body_z)
{
  body_x = optical_z;
  body_y = -optical_x;
  body_z = -optical_y;
}

void bodyToOptical(
  double body_x, double body_y, double body_z,
  double & optical_x, double & optical_y, double & optical_z)
{
  optical_x = -body_y;
  optical_y = -body_z;
  optical_z = body_x;
}

std::array<std::array<double, 3>, 4> markerCorners(double edge_length_m)
{
  const double half = edge_length_m / 2.0;
  return {{
    {{-half, half, 0.0}},
    {{half, half, 0.0}},
    {{half, -half, 0.0}},
    {{-half, -half, 0.0}},
  }};
}

std::array<double, 4> quaternionFromRotationVector(double rx, double ry, double rz)
{
  const double angle = std::sqrt(rx * rx + ry * ry + rz * rz);
  if (angle < 1e-12) {
    return {{0.0, 0.0, 0.0, 1.0}};
  }
  const double half = angle / 2.0;
  const double scale = std::sin(half) / angle;
  return {{rx * scale, ry * scale, rz * scale, std::cos(half)}};
}

}  // namespace uav_perception
