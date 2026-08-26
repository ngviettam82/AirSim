#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "uav_localization/health_checks.hpp"

using uav_localization::DisagreementCheck;
using uav_localization::HealthLevel;
using uav_localization::JumpCheck;
using uav_localization::JumpVerdict;
using uav_localization::RateCheck;
using uav_localization::classifyCrossCheck;
using uav_localization::classifyJump;
using uav_localization::classifyDisagreement;
using uav_localization::classifyRate;
using uav_localization::isImplausibleJump;

namespace
{

TEST(JumpCheck, NormalFlightIsNotAJump)
{
  const JumpCheck check{20.0, 0.2};

  EXPECT_FALSE(isImplausibleJump(0.5, 0.1, check));   // 5 m/s
  EXPECT_FALSE(isImplausibleJump(2.0, 0.1, check));   // 20 m/s, right at the limit
}

TEST(JumpCheck, AStepNoAircraftCouldFlyIsAJump)
{
  const JumpCheck check{20.0, 0.2};

  EXPECT_TRUE(isImplausibleJump(5.0, 0.1, check));
}

TEST(JumpCheck, NonsenseInputsDoNotRaiseAnAlarm)
{
  const JumpCheck check{20.0, 0.2};

  EXPECT_FALSE(isImplausibleJump(5.0, 0.0, check));
  EXPECT_FALSE(isImplausibleJump(std::numeric_limits<double>::quiet_NaN(), 0.1, check));
}

// The numbers below are one measured flight, not invented limits: bag
// uav0_20260825_073235Z, /state/odometry_fused, GPS selected. See README.
constexpr double kMeasuredNoiseStepM = 2.217;   // largest hover step of that flight
constexpr double kMeasuredDtSec = 0.1;          // the stream runs at 10 Hz
constexpr double kDeclaredSigmaM = 0.9;         // LocalizationStatus.position_uncertainty

TEST(ClassifyJump, TheOldRuleCalledThatFlightsWorstNoiseStepAJump)
{
  const JumpCheck check{20.0, 0.2};

  // Two-sided anchor: without this the widened rule below proves nothing, because a
  // rule that never fired would also pass. It cleared the old bar by 17 mm.
  EXPECT_TRUE(isImplausibleJump(kMeasuredNoiseStepM, kMeasuredDtSec, check));
}

TEST(ClassifyJump, NoiseTheSourceDeclaredIsNotAJump)
{
  const JumpCheck check{20.0, 0.2};

  EXPECT_EQ(
    classifyJump(kMeasuredNoiseStepM, kMeasuredDtSec, kDeclaredSigmaM, check),
    JumpVerdict::Continuous);
}

TEST(ClassifyJump, ARealTeleportStillFiresThroughTheNoiseAllowance)
{
  const JumpCheck check{20.0, 0.2};

  EXPECT_EQ(classifyJump(10.0, kMeasuredDtSec, kDeclaredSigmaM, check), JumpVerdict::Jumped);
}

TEST(ClassifyJump, TheAllowanceIsExactlyTheDeclaredSigmaWide)
{
  const JumpCheck check{20.0, 0.2};   // allowance = 20*0.1 + 0.2 + 2*0.9 = 4.0 m

  EXPECT_EQ(classifyJump(3.99, kMeasuredDtSec, kDeclaredSigmaM, check), JumpVerdict::Continuous);
  EXPECT_EQ(classifyJump(4.01, kMeasuredDtSec, kDeclaredSigmaM, check), JumpVerdict::Jumped);
}

TEST(ClassifyJump, AnUndeclaredUncertaintyIsNotJudged)
{
  const JumpCheck check{20.0, 0.2};

  // -1.0 is what LocalizationStatus carries before a source is valid; it was present in
  // 12 of 674 samples of the measured flight. O3: not judged is not the same as fine.
  EXPECT_EQ(classifyJump(1.0, kMeasuredDtSec, -1.0, check), JumpVerdict::CannotJudge);
}

TEST(ClassifyJump, AnAbsurdUncertaintyIsNotJudgedEither)
{
  const JumpCheck check{20.0, 0.2};

  // The trap this closes: a source claiming 50 m would otherwise buy silence for any
  // step at all, turning the check off exactly when it is needed.
  EXPECT_EQ(classifyJump(40.0, kMeasuredDtSec, 50.0, check), JumpVerdict::CannotJudge);
}

TEST(ClassifyJump, NonsenseTimingIsNotJudged)
{
  const JumpCheck check{20.0, 0.2};

  EXPECT_EQ(classifyJump(5.0, 0.0, kDeclaredSigmaM, check), JumpVerdict::CannotJudge);
  EXPECT_EQ(classifyJump(5.0, -0.1, kDeclaredSigmaM, check), JumpVerdict::CannotJudge);
  EXPECT_EQ(
    classifyJump(std::numeric_limits<double>::quiet_NaN(), 0.1, kDeclaredSigmaM, check),
    JumpVerdict::CannotJudge);
  EXPECT_EQ(
    classifyJump(1.0, 0.1, std::numeric_limits<double>::quiet_NaN(), check),
    JumpVerdict::CannotJudge);
}

TEST(Disagreement, SourcesWithinTheirStatedUncertaintyAreFine)
{
  const DisagreementCheck check{3.0, 6.0, 0.2};

  EXPECT_EQ(classifyDisagreement(0.5, 0.9, 0.1, check), HealthLevel::Ok);
}

// GNSS case: 5 m out, still claiming 0.9 m.
TEST(Disagreement, ADriftingSourceIsCaughtEvenWhileItClaimsPrecision)
{
  const DisagreementCheck check{3.0, 6.0, 0.2};

  EXPECT_EQ(classifyDisagreement(3.0, 0.9, 0.1, check), HealthLevel::Warn);
  EXPECT_EQ(classifyDisagreement(8.0, 0.9, 0.1, check), HealthLevel::Error);
}

TEST(Disagreement, SourcesThatAdmitBeingVagueAreJudgedMoreGently)
{
  const DisagreementCheck check{3.0, 6.0, 0.2};

  EXPECT_EQ(classifyDisagreement(3.0, 2.0, 2.0, check), HealthLevel::Ok);
}

TEST(Disagreement, AnOverconfidentPairCannotForceADivideByZero)
{
  const DisagreementCheck check{3.0, 6.0, 0.2};

  EXPECT_EQ(classifyDisagreement(0.1, 0.0, 0.0, check), HealthLevel::Ok);
  EXPECT_EQ(classifyDisagreement(10.0, 0.0, 0.0, check), HealthLevel::Error);
}

TEST(Disagreement, AnUnstatedUncertaintyIsNotJudgedHere)
{
  const DisagreementCheck check{3.0, 6.0, 0.2};

  // SourceChannel already refuses these; reporting twice is noise.
  EXPECT_EQ(classifyDisagreement(50.0, -1.0, 0.1, check), HealthLevel::Ok);
}

TEST(RateCheck, GradesTheOutputRateInThreeBands)
{
  const RateCheck check{10.0, 0.8, 0.5};

  EXPECT_EQ(classifyRate(10.0, check), HealthLevel::Ok);
  EXPECT_EQ(classifyRate(9.0, check), HealthLevel::Ok);
  EXPECT_EQ(classifyRate(7.0, check), HealthLevel::Warn);
  EXPECT_EQ(classifyRate(2.0, check), HealthLevel::Error);
  EXPECT_EQ(classifyRate(0.0, check), HealthLevel::Error);
}

TEST(CrossCheck, OneSourceAloneIsAWarningNotAnAgreement)
{
  EXPECT_EQ(classifyCrossCheck(0U), HealthLevel::Warn);
  EXPECT_EQ(classifyCrossCheck(1U), HealthLevel::Warn);
  EXPECT_EQ(classifyCrossCheck(2U), HealthLevel::Ok);
  EXPECT_EQ(classifyCrossCheck(3U), HealthLevel::Ok);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
