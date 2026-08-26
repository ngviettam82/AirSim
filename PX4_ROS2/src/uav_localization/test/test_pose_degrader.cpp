#include <cmath>
#include <numeric>
#include <vector>

#include <gtest/gtest.h>

#include "uav_localization/pose_degrader.hpp"

using uav_localization::DegradeParameters;
using uav_localization::PoseDegrader;

namespace
{

const std::array<double, 3> kTruth{{1.0, -2.0, 3.0}};

std::vector<double> runAxis(const DegradeParameters & parameters, int steps)
{
  PoseDegrader degrader(11U);
  degrader.configure(parameters);

  std::vector<double> errors;
  errors.reserve(static_cast<size_t>(steps));
  for (int step = 0; step < steps; ++step) {
    errors.push_back(degrader.apply(kTruth, 0.02)[0] - kTruth[0]);
  }
  return errors;
}

double stddev(const std::vector<double> & values)
{
  const double average =
    std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
  double sum_squares = 0.0;
  for (const double value : values) {
    sum_squares += (value - average) * (value - average);
  }
  return std::sqrt(sum_squares / static_cast<double>(values.size()));
}

double autocorrelation(const std::vector<double> & values, size_t lag)
{
  const double average =
    std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
  double covariance = 0.0;
  double variance = 0.0;
  for (size_t i = 0; i < values.size(); ++i) {
    variance += (values[i] - average) * (values[i] - average);
    if (i + lag < values.size()) {
      covariance += (values[i] - average) * (values[i + lag] - average);
    }
  }
  return variance > 0.0 ? covariance / variance : 0.0;
}

TEST(PoseDegrader, PassesTruthThroughUntouchedWhenDisabled)
{
  PoseDegrader degrader(1U);
  DegradeParameters parameters;
  parameters.white_noise_stddev_m = 1.0;
  parameters.drift.enabled = true;
  parameters.drift.sigma_m = 10.0;
  parameters.enabled = false;               // the master switch overrules the rest
  degrader.configure(parameters);

  const auto out = degrader.apply(kTruth, 0.02);

  EXPECT_DOUBLE_EQ(out[0], kTruth[0]);
  EXPECT_DOUBLE_EQ(out[1], kTruth[1]);
  EXPECT_DOUBLE_EQ(out[2], kTruth[2]);
}

TEST(PoseDegrader, WhiteNoiseHasTheRequestedSpreadAndNoMemory)
{
  DegradeParameters parameters;
  parameters.enabled = true;
  parameters.white_noise_stddev_m = 0.05;
  parameters.drift.enabled = false;

  const auto errors = runAxis(parameters, 6000);

  EXPECT_GT(stddev(errors), 0.04);
  EXPECT_LT(stddev(errors), 0.06);
  EXPECT_LT(std::abs(autocorrelation(errors, 1)), 0.1);
}

TEST(PoseDegrader, DriftAddsMemoryThatWhiteNoiseDoesNotHave)
{
  DegradeParameters parameters;
  parameters.enabled = true;
  parameters.white_noise_stddev_m = 0.0;
  parameters.drift.enabled = true;
  parameters.drift.correlation_time_sec = 30.0;
  parameters.drift.sigma_m = 0.5;

  const auto errors = runAxis(parameters, 20000);

  EXPECT_GT(autocorrelation(errors, 1), 0.9);
  EXPECT_GT(stddev(errors), 0.25);
}

TEST(PoseDegrader, TurningItOffClearsAccumulatedDrift)
{
  PoseDegrader degrader(3U);
  DegradeParameters parameters;
  parameters.enabled = true;
  parameters.drift.enabled = true;
  parameters.drift.correlation_time_sec = 30.0;
  parameters.drift.sigma_m = 1.0;
  degrader.configure(parameters);
  for (int step = 0; step < 2000; ++step) {
    degrader.apply(kTruth, 0.02);
  }

  parameters.enabled = false;
  degrader.configure(parameters);

  EXPECT_DOUBLE_EQ(degrader.apply(kTruth, 0.02)[0], kTruth[0]);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
