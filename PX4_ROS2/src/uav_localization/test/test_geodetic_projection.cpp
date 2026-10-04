#include <cmath>

#include <gtest/gtest.h>

#include "uav_localization/geodetic_projection.hpp"

using uav_localization::GeodeticPoint;
using uav_localization::projectToLocalEnu;

namespace
{

// PX4's default simulation origin.
constexpr double kDatumLatitudeDeg = 47.397742;
constexpr double kDatumLongitudeDeg = 8.545594;
constexpr double kDatumAltitudeM = 488.0;

// 6371000 * pi / 180, PX4's spherical earth.
constexpr double kMetresPerDegreeLatitude = 111194.92664455873;

GeodeticPoint datum()
{
  return {kDatumLatitudeDeg, kDatumLongitudeDeg, kDatumAltitudeM};
}

TEST(GeodeticProjection, TheDatumItselfIsTheOrigin)
{
  const auto local = projectToLocalEnu(datum(), datum());

  EXPECT_NEAR(local.east_m, 0.0, 1e-9);
  EXPECT_NEAR(local.north_m, 0.0, 1e-9);
  EXPECT_NEAR(local.up_m, 0.0, 1e-9);
}

TEST(GeodeticProjection, ADegreeOfLatitudeIsTheExpectedNumberOfMetres)
{
  GeodeticPoint north = datum();
  north.latitude_deg += 1.0e-5;

  const auto local = projectToLocalEnu(datum(), north);

  EXPECT_NEAR(local.north_m, kMetresPerDegreeLatitude * 1.0e-5, 1e-3);
  EXPECT_NEAR(local.east_m, 0.0, 1e-6);
}

TEST(GeodeticProjection, LongitudeShrinksWithTheCosineOfLatitude)
{
  GeodeticPoint east = datum();
  east.longitude_deg += 1.0e-5;

  const double expected_east =
    kMetresPerDegreeLatitude * 1.0e-5 * std::cos(kDatumLatitudeDeg * M_PI / 180.0);

  const auto local = projectToLocalEnu(datum(), east);

  EXPECT_NEAR(local.east_m, expected_east, 1e-3);
  EXPECT_NEAR(local.north_m, 0.0, 1e-5);
}

TEST(GeodeticProjection, StaysAccurateAcrossArenaDistances)
{
  GeodeticPoint far_north = datum();
  far_north.latitude_deg += 1.0e-3;

  const auto local = projectToLocalEnu(datum(), far_north);

  EXPECT_NEAR(local.north_m, kMetresPerDegreeLatitude * 1.0e-3, 1e-3);
}

TEST(GeodeticProjection, DirectionsCarryTheExpectedSigns)
{
  GeodeticPoint south = datum();
  south.latitude_deg -= 1.0e-4;
  GeodeticPoint west = datum();
  west.longitude_deg -= 1.0e-4;

  EXPECT_LT(projectToLocalEnu(datum(), south).north_m, 0.0);
  EXPECT_LT(projectToLocalEnu(datum(), west).east_m, 0.0);
}

TEST(GeodeticProjection, OppositeOffsetsMirrorEachOther)
{
  GeodeticPoint north = datum();
  north.latitude_deg += 1.0e-4;
  GeodeticPoint south = datum();
  south.latitude_deg -= 1.0e-4;

  EXPECT_NEAR(
    projectToLocalEnu(datum(), north).north_m,
    -projectToLocalEnu(datum(), south).north_m, 1e-6);
}

TEST(GeodeticProjection, AltitudeIsMeasuredFromTheDatum)
{
  GeodeticPoint higher = datum();
  higher.altitude_m += 12.5;

  EXPECT_NEAR(projectToLocalEnu(datum(), higher).up_m, 12.5, 1e-9);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
