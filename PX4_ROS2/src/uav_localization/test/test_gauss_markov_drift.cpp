#include <cmath>
#include <numeric>
#include <vector>

#include <gtest/gtest.h>

#include "uav_localization/gauss_markov_drift.hpp"

using uav_localization::DriftParameters;
using uav_localization::GaussMarkovDrift;

namespace
{

constexpr double kStepSec = 0.125;        // PX4 simulates GPS at 8 Hz
constexpr int kStepsPerHundredSeconds = 800;

std::vector<double> runEastAxis(
  const DriftParameters & parameters, int steps, uint32_t seed = 1U)
{
  GaussMarkovDrift drift(seed);
  drift.configure(parameters);

  std::vector<double> samples;
  samples.reserve(static_cast<size_t>(steps));
  for (int step = 0; step < steps; ++step) {
    samples.push_back(drift.advance(kStepSec)[0]);
  }
  return samples;
}

double mean(const std::vector<double> & samples)
{
  return std::accumulate(samples.begin(), samples.end(), 0.0) /
         static_cast<double>(samples.size());
}

double stddev(const std::vector<double> & samples)
{
  const double average = mean(samples);
  double sum_squares = 0.0;
  for (const double sample : samples) {
    sum_squares += (sample - average) * (sample - average);
  }
  return std::sqrt(sum_squares / static_cast<double>(samples.size()));
}

double autocorrelation(const std::vector<double> & samples, size_t lag)
{
  const double average = mean(samples);
  double covariance = 0.0;
  double variance = 0.0;
  for (size_t i = 0; i < samples.size(); ++i) {
    variance += (samples[i] - average) * (samples[i] - average);
    if (i + lag < samples.size()) {
      covariance += (samples[i] - average) * (samples[i + lag] - average);
    }
  }
  return variance > 0.0 ? covariance / variance : 0.0;
}

DriftParameters realisticDrift()
{
  return DriftParameters{true, 60.0, 5.0};
}

TEST(GaussMarkovDrift, StaysSilentUnlessTurnedOn)
{
  const auto samples = runEastAxis(DriftParameters{}, 1000);

  for (const double sample : samples) {
    EXPECT_DOUBLE_EQ(sample, 0.0) << "injection must be opt-in";
  }
}

TEST(GaussMarkovDrift, ZeroSigmaProducesNoDrift)
{
  const auto samples = runEastAxis(DriftParameters{true, 60.0, 0.0}, 1000);

  EXPECT_DOUBLE_EQ(samples.back(), 0.0);
}

TEST(GaussMarkovDrift, ANonPositiveStepLeavesTheOffsetAlone)
{
  GaussMarkovDrift drift(1U);
  drift.configure(realisticDrift());
  drift.advance(kStepSec);

  const auto before = drift.offset();
  drift.advance(0.0);
  drift.advance(-1.0);

  EXPECT_DOUBLE_EQ(drift.offset()[0], before[0]);
}

TEST(GaussMarkovDrift, SettlesAroundTheRequestedSpread)
{
  auto samples = runEastAxis(realisticDrift(), 40 * kStepsPerHundredSeconds);
  samples.erase(samples.begin(), samples.begin() + 5 * kStepsPerHundredSeconds);   // warm-up

  EXPECT_GT(stddev(samples), 3.0);
  EXPECT_LT(stddev(samples), 7.5);
}

TEST(GaussMarkovDrift, ErrorIsCorrelatedInTimeUnlikeWhiteNoise)
{
  const auto correlated = runEastAxis(realisticDrift(), 40 * kStepsPerHundredSeconds);
  const auto white = runEastAxis(DriftParameters{true, 0.0, 5.0}, 40 * kStepsPerHundredSeconds);

  EXPECT_GT(autocorrelation(correlated, 1), 0.9);
  EXPECT_LT(autocorrelation(white, 1), 0.2) << "no correlation time means no memory";
}

// One tau apart the error decays by e, not vanishes.
TEST(GaussMarkovDrift, CorrelationDecaysOverTheCorrelationTime)
{
  const auto samples = runEastAxis(realisticDrift(), 80 * kStepsPerHundredSeconds);
  const size_t lag_of_one_tau = 60.0 / kStepSec;

  EXPECT_GT(autocorrelation(samples, lag_of_one_tau), 0.15);
  EXPECT_LT(autocorrelation(samples, lag_of_one_tau), 0.65);
}

TEST(GaussMarkovDrift, WandersFarBeyondTheSimulatorsWhiteNoise)
{
  const auto samples = runEastAxis(realisticDrift(), 40 * kStepsPerHundredSeconds);

  double largest = 0.0;
  for (const double sample : samples) {
    largest = std::max(largest, std::abs(sample));
  }
  EXPECT_GT(largest, 2.0) << "PX4's simulated GPS only ever errs by about 0.2 m";
}

TEST(GaussMarkovDrift, ASeedMakesARunReproducible)
{
  const auto first = runEastAxis(realisticDrift(), 500, 7U);
  const auto again = runEastAxis(realisticDrift(), 500, 7U);
  const auto different = runEastAxis(realisticDrift(), 500, 8U);

  EXPECT_EQ(first, again);
  EXPECT_NE(first, different);
}

/// The constructor seed is unreachable from a launch file -- a yaml can only
/// reach DriftParameters. G4 varies seeds through exactly that path, so the path
/// needs its own pin; ASeedMakesARunReproducible above only covers the ctor.
TEST(GaussMarkovDrift, TheSeedCarriedInParametersChangesTheRealisation)
{
  DriftParameters eleven = realisticDrift();
  eleven.seed = 11U;
  DriftParameters twelve = realisticDrift();
  twelve.seed = 12U;

  // Identical construction seed on purpose: only DriftParameters::seed differs.
  const auto first = runEastAxis(eleven, 500, 3U);
  const auto again = runEastAxis(eleven, 500, 3U);
  const auto other = runEastAxis(twelve, 500, 3U);

  EXPECT_EQ(first, again) << "the same parameter seed must replay the same drift";
  EXPECT_NE(first, other) << "a different parameter seed must not replay the same drift";
}

TEST(GaussMarkovDrift, ResetReturnsToNoError)
{
  GaussMarkovDrift drift(1U);
  drift.configure(realisticDrift());
  for (int step = 0; step < 500; ++step) {
    drift.advance(kStepSec);
  }

  drift.reset();

  EXPECT_DOUBLE_EQ(drift.offset()[0], 0.0);
}

TEST(GaussMarkovDrift, TurningInjectionOffClearsTheAccumulatedError)
{
  GaussMarkovDrift drift(1U);
  drift.configure(realisticDrift());
  for (int step = 0; step < 500; ++step) {
    drift.advance(kStepSec);
  }

  drift.configure(DriftParameters{});

  EXPECT_DOUBLE_EQ(drift.offset()[0], 0.0);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
