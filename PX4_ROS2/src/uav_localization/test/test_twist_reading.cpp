// No ROS here on purpose: the convention is arithmetic, so it is pinned
// jitter-free and on no domain at all.
#include <array>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "uav_localization/twist_reading.hpp"

namespace
{

using uav_localization::TwistTrust;
using uav_localization::readTwistTrust;
using uav_localization::statedTwistStddev;

std::array<double, 36> covarianceWith(double linear_x_variance)
{
  std::array<double, 36> covariance{};
  covariance[0] = linear_x_variance;
  return covariance;
}

TEST(TwistReading, MinusOneMeansThereIsNoTwistToRead)
{
  const auto covariance = covarianceWith(-1.0);

  EXPECT_EQ(readTwistTrust(covariance), TwistTrust::Unknown);
  EXPECT_FALSE(statedTwistStddev(covariance).has_value());
}

TEST(TwistReading, ZeroMeansTheErrorWasNeverDeclaredNotThatItIsPerfect)
{
  const auto covariance = covarianceWith(0.0);

  EXPECT_EQ(readTwistTrust(covariance), TwistTrust::Unstated);
  EXPECT_FALSE(statedTwistStddev(covariance).has_value())
    << "reading 0 as a perfect velocity is the whole trap this header exists for";
}

TEST(TwistReading, APositiveEntryIsADeclaredSigmaAndTheEntryIsAVariance)
{
  const auto covariance = covarianceWith(0.25);

  EXPECT_EQ(readTwistTrust(covariance), TwistTrust::Stated);
  ASSERT_TRUE(statedTwistStddev(covariance).has_value());
  EXPECT_NEAR(*statedTwistStddev(covariance), 0.5, 1e-12) << "0.25 is sigma squared, not sigma";
}

TEST(TwistReading, ANotANumberIsNotAnInvitationToGuess)
{
  const auto covariance = covarianceWith(std::numeric_limits<double>::quiet_NaN());

  EXPECT_EQ(readTwistTrust(covariance), TwistTrust::Unknown);
  EXPECT_FALSE(statedTwistStddev(covariance).has_value());
}

TEST(TwistReading, AnUnexpectedNegativeIsReadAsUnknownRatherThanRepaired)
{
  const auto covariance = covarianceWith(-5.0);

  EXPECT_EQ(readTwistTrust(covariance), TwistTrust::Unknown)
    << "only -1 is the agreed sentinel, but no negative variance may become a sigma";
  EXPECT_EQ(readTwistTrust(covarianceWith(std::numeric_limits<double>::infinity())),
    TwistTrust::Unknown);
}

// The convention is only worth anything if it matches what the mux itself does.
// localization_mux_node.cpp:168  report.twist_known = twist.covariance[0] >= 0.0
// source_channel.cpp:88-95       !twist_known -> -1 | stddev >= 0 -> stddev^2 | else leave 0
TEST(TwistReading, TheReadingAgreesWithWhatTheMuxCallsAKnownTwist)
{
  for (const double variance : {-1.0, 0.0, 0.25, std::numeric_limits<double>::quiet_NaN(), -5.0}) {
    const auto covariance = covarianceWith(variance);
    const bool mux_calls_it_known = covariance[0] >= 0.0;

    EXPECT_EQ(readTwistTrust(covariance) != TwistTrust::Unknown, mux_calls_it_known)
      << "disagreed with the mux on variance " << variance;
  }
}

TEST(TwistReading, TheThreeValuesTheChannelCanWriteMapToTheThreeStates)
{
  // Exactly the three branches of source_channel.cpp:88-95, in order.
  EXPECT_EQ(readTwistTrust(covarianceWith(-1.0)), TwistTrust::Unknown);
  EXPECT_EQ(readTwistTrust(covarianceWith(0.5 * 0.5)), TwistTrust::Stated);
  EXPECT_EQ(readTwistTrust(covarianceWith(0.0)), TwistTrust::Unstated);
  EXPECT_NEAR(*statedTwistStddev(covarianceWith(0.5 * 0.5)), 0.5, 1e-12)
    << "the mux declares continuity_decay_rate_mps 0.5 while it slides the pose";
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
