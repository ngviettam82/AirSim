#include "uav_localization/geodetic_projection.hpp"

#include <algorithm>
#include <cmath>

namespace uav_localization
{

namespace
{

// PX4 CONSTANTS_RADIUS_OF_EARTH, src/lib/geo/geo.h.
constexpr double kEarthRadiusM = 6371000.0;
constexpr double kDegreesToRadians = M_PI / 180.0;

}  // namespace

LocalPoint projectToLocalEnu(const GeodeticPoint & datum, const GeodeticPoint & point)
{
  const double datum_latitude = datum.latitude_deg * kDegreesToRadians;
  const double datum_longitude = datum.longitude_deg * kDegreesToRadians;
  const double latitude = point.latitude_deg * kDegreesToRadians;
  const double longitude = point.longitude_deg * kDegreesToRadians;

  const double sin_latitude = std::sin(latitude);
  const double cos_latitude = std::cos(latitude);
  const double sin_datum_latitude = std::sin(datum_latitude);
  const double cos_datum_latitude = std::cos(datum_latitude);
  const double delta_longitude = longitude - datum_longitude;
  const double cos_delta_longitude = std::cos(delta_longitude);

  const double cos_arc = std::clamp(
    sin_datum_latitude * sin_latitude + cos_datum_latitude * cos_latitude * cos_delta_longitude,
    -1.0, 1.0);
  const double arc = std::acos(cos_arc);

  // Arc length, not chord; degenerates to 1 at datum.
  const double scale = (std::abs(arc) < 1e-12) ? 1.0 : arc / std::sin(arc);

  LocalPoint local;
  local.north_m = scale * kEarthRadiusM *
    (cos_datum_latitude * sin_latitude - sin_datum_latitude * cos_latitude * cos_delta_longitude);
  local.east_m = scale * kEarthRadiusM * cos_latitude * std::sin(delta_longitude);
  local.up_m = point.altitude_m - datum.altitude_m;
  return local;
}

}  // namespace uav_localization
