// Geodetic -> local ENU on PX4's datum. See README.

#ifndef UAV_LOCALIZATION__GEODETIC_PROJECTION_HPP_
#define UAV_LOCALIZATION__GEODETIC_PROJECTION_HPP_

namespace uav_localization
{

struct GeodeticPoint
{
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double altitude_m{0.0};   // AMSL
};

struct LocalPoint
{
  double east_m{0.0};
  double north_m{0.0};
  double up_m{0.0};
};

/// Azimuthal equidistant, mirroring PX4 MapProjection exactly.
LocalPoint projectToLocalEnu(const GeodeticPoint & datum, const GeodeticPoint & point);

}  // namespace uav_localization

#endif  // UAV_LOCALIZATION__GEODETIC_PROJECTION_HPP_
