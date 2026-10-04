// ROS-free (no domain, no rclcpp::init): mission_params validation and the
// XML whitelist are arithmetic/text checks, so a test pins them exactly
// instead of trusting a flight -- same rule mission_policy's own test file
// follows.
#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "uav_mission/mission_registry.hpp"

#ifndef UAV_MISSION_CONFIG_DIR
#error "UAV_MISSION_CONFIG_DIR must be defined by CMake"
#endif

namespace uav_mission
{
namespace
{

std::string missionsDir()
{
  return std::string(UAV_MISSION_CONFIG_DIR) + "/missions";
}

MissionRegistryParams defaultParams()
{
  return MissionRegistryParams{};
}

/// Copies the 3 real shipped XML files into a scratch directory, then lets
/// the caller overwrite one -- so a whitelist-violation test still exercises
/// the OTHER two real files, not a fabricated stand-in for all three.
class ScratchMissionsDir
{
public:
  ScratchMissionsDir()
  {
    dir_ = std::filesystem::temp_directory_path() /
      ("uav_mission_test_scratch_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
    std::filesystem::create_directories(dir_);
    for (const char * name : {"indoor_patrol.xml", "follow_target.xml", "inspect_point.xml"}) {
      std::filesystem::copy_file(
        std::filesystem::path(missionsDir()) / name, dir_ / name,
        std::filesystem::copy_options::overwrite_existing);
    }
  }

  ~ScratchMissionsDir()
  {
    std::error_code ec;
    std::filesystem::remove_all(dir_, ec);
  }

  void overwrite(const std::string & filename, const std::string & content) const
  {
    std::ofstream out(dir_ / filename, std::ios::trunc);
    out << content;
  }

  std::string path() const {return dir_.string();}

private:
  std::filesystem::path dir_;
};

}  // namespace

// ===========================================================================
// Parameter validation (MissionRegistry::validate()).
// ===========================================================================

TEST(ParamValidation, DefaultsAreAccepted)
{
  EXPECT_NO_THROW(MissionRegistry registry(defaultParams(), missionsDir()));
}

TEST(ParamValidation, MaxAltNotAboveMinAltIsRejected)
{
  MissionRegistryParams params = defaultParams();
  params.max_alt_copy_m = params.min_alt_copy_m;
  EXPECT_THROW(MissionRegistry registry(params, missionsDir()), std::invalid_argument);
}

TEST(ParamValidation, GotoReachAboveSpeedTimesTimeoutIsRejected)
{
  MissionRegistryParams params = defaultParams();
  params.goto_reach_copy_m =
    params.navigator_max_speed_copy_mps * params.navigator_goto_timeout_copy_sec + 1.0;
  EXPECT_THROW(MissionRegistry registry(params, missionsDir()), std::invalid_argument);
}

TEST(ParamValidation, TargetSpeedMaxAboveHalfNavigatorSpeedIsRejected)
{
  MissionRegistryParams params = defaultParams();
  params.target_speed_max_mps = 0.5 * params.navigator_max_speed_copy_mps + 0.01;
  EXPECT_THROW(MissionRegistry registry(params, missionsDir()), std::invalid_argument);
}

TEST(ParamValidation, TargetSpeedMaxAtExactlyHalfNavigatorSpeedIsAccepted)
{
  MissionRegistryParams params = defaultParams();
  params.target_speed_max_mps = 0.5 * params.navigator_max_speed_copy_mps;
  EXPECT_NO_THROW(MissionRegistry registry(params, missionsDir()));
}

TEST(ParamValidation, MaxLoopsZeroIsRejected)
{
  MissionRegistryParams params = defaultParams();
  params.max_loops = 0;
  EXPECT_THROW(MissionRegistry registry(params, missionsDir()), std::invalid_argument);
}

// ===========================================================================
// Construction validates all 3 shipped XML files against the whitelist.
// ===========================================================================

TEST(ShippedXml, AllThreeLoadCleanly)
{
  EXPECT_NO_THROW(MissionRegistry registry(defaultParams(), missionsDir()));
}

TEST(ShippedXml, NonWhitelistedTagIsRejectedAtConstruction)
{
  ScratchMissionsDir scratch;
  scratch.overwrite(
    "indoor_patrol.xml",
    "<root main_tree_to_execute=\"MainTree\">"
    "<BehaviorTree ID=\"MainTree\"><EvilNode/></BehaviorTree></root>");
  EXPECT_THROW(MissionRegistry registry(defaultParams(), scratch.path()), std::invalid_argument);
}

TEST(ShippedXml, MalformedXmlIsRejectedAtConstruction)
{
  ScratchMissionsDir scratch;
  scratch.overwrite("follow_target.xml", "<root><unterminated>");
  EXPECT_THROW(MissionRegistry registry(defaultParams(), scratch.path()), std::invalid_argument);
}

// ===========================================================================
// list() / describe().
// ===========================================================================

TEST(Catalog, ListReturnsExactlyThreeMissions)
{
  MissionRegistry registry(defaultParams(), missionsDir());
  const std::vector<MissionDescriptor> missions = registry.list();
  ASSERT_EQ(missions.size(), 3u);
}

TEST(Catalog, DescribeUnknownMissionThrows)
{
  MissionRegistry registry(defaultParams(), missionsDir());
  EXPECT_THROW(registry.describe("no_such_mission"), std::invalid_argument);
}

// ===========================================================================
// load(): total_steps per mission (definitions pinned in mission_registry.hpp).
// ===========================================================================

TEST(Load, IndoorPatrolDefaultTotalStepsEqualsDefaultLoops)
{
  MissionRegistry registry(defaultParams(), missionsDir());
  const MissionPlan plan = registry.load("indoor_patrol", "");
  EXPECT_EQ(plan.total_steps, 4u);
  EXPECT_EQ(plan.indoor_patrol.loops, 4);
  EXPECT_FALSE(plan.indoor_patrol.waypoints.empty());
}

TEST(Load, FollowTargetTotalStepsFixedAtOne)
{
  MissionRegistry registry(defaultParams(), missionsDir());
  const MissionPlan plan = registry.load("follow_target", "");
  EXPECT_EQ(plan.total_steps, 1u);
}

TEST(Load, InspectPointTotalStepsFixedAtFour)
{
  MissionRegistry registry(defaultParams(), missionsDir());
  const MissionPlan plan = registry.load("inspect_point", "");
  EXPECT_EQ(plan.total_steps, 4u);
}

TEST(Load, UnknownMissionIdThrows)
{
  MissionRegistry registry(defaultParams(), missionsDir());
  EXPECT_THROW(registry.load("no_such_mission", ""), std::invalid_argument);
}

TEST(Load, IndoorPatrolLoopsOverrideChangesTotalSteps)
{
  MissionRegistry registry(defaultParams(), missionsDir());
  const MissionPlan plan = registry.load("indoor_patrol", "loops: 10");
  EXPECT_EQ(plan.total_steps, 10u);
}

// ===========================================================================
// mission_params validation: unknown key.
// ===========================================================================

TEST(ParamsValidation, UnknownKeyIsRejected)
{
  MissionRegistry registry(defaultParams(), missionsDir());
  EXPECT_THROW(registry.load("indoor_patrol", "not_a_real_key: 1"), std::invalid_argument);
}

TEST(ParamsValidation, UnknownKeyInsideWaypointIsRejected)
{
  MissionRegistry registry(defaultParams(), missionsDir());
  EXPECT_THROW(
    registry.load("indoor_patrol", "waypoints: [{x: 0, y: 0, z: 1, w: 9}]"), std::invalid_argument);
}

// ===========================================================================
// mission_params validation: non-finite / malformed numbers.
// ===========================================================================

TEST(ParamsValidation, NonNumericValueIsRejected)
{
  MissionRegistry registry(defaultParams(), missionsDir());
  EXPECT_THROW(
    registry.load("follow_target", "target_speed_mps: not_a_number"), std::invalid_argument);
}

TEST(ParamsValidation, NonFiniteValueIsRejected)
{
  MissionRegistry registry(defaultParams(), missionsDir());
  EXPECT_THROW(registry.load("follow_target", "timeout_sec: .nan"), std::invalid_argument);
}

// 🟡-3 (2026-08-23, review round 2): a caller-supplied timeout_sec above
// step_timeout_copy_sec would let Y1's own step_timeout_sec cancel a
// healthy mid-track leg first, reporting a confusing ABORTED_TIMEOUT --
// reject the goal instead. defaultParams().step_timeout_copy_sec == 300.0.
TEST(ParamsValidation, FollowTargetTimeoutAboveStepTimeoutCopyIsRejected)
{
  MissionRegistry registry(defaultParams(), missionsDir());
  EXPECT_THROW(registry.load("follow_target", "timeout_sec: 300.1"), std::invalid_argument);
}

TEST(ParamsValidation, FollowTargetTimeoutAtExactlyStepTimeoutCopyIsAccepted)
{
  MissionRegistry registry(defaultParams(), missionsDir());
  EXPECT_NO_THROW(registry.load("follow_target", "timeout_sec: 300.0"));
}

// ===========================================================================
// mission_params validation: z outside [min_alt_copy_m, max_alt_copy_m].
// ===========================================================================

TEST(ParamsValidation, WaypointZAboveMaxAltIsRejected)
{
  MissionRegistry registry(defaultParams(), missionsDir());
  EXPECT_THROW(
    registry.load("indoor_patrol", "waypoints: [{x: 0, y: 0, z: 100.0}]"), std::invalid_argument);
}

TEST(ParamsValidation, WaypointZBelowMinAltIsRejected)
{
  MissionRegistry registry(defaultParams(), missionsDir());
  EXPECT_THROW(
    registry.load("indoor_patrol", "waypoints: [{x: 0, y: 0, z: 0.1}]"), std::invalid_argument);
}

TEST(ParamsValidation, InspectPointApproachZOutOfRangeIsRejected)
{
  MissionRegistry registry(defaultParams(), missionsDir());
  EXPECT_THROW(
    registry.load("inspect_point", "approach: {x: 0, y: 0, z: 50.0}"), std::invalid_argument);
}

// ===========================================================================
// mission_params validation: leg distance > goto_reach_copy_m.
// ===========================================================================

TEST(ParamsValidation, LegLongerThanGotoReachIsRejected)
{
  MissionRegistry registry(defaultParams(), missionsDir());
  EXPECT_THROW(
    registry.load(
      "indoor_patrol", "waypoints: [{x: 0, y: 0, z: 1.5}, {x: 100.0, y: 0, z: 1.5}]"),
    std::invalid_argument);
}

TEST(ParamsValidation, LegAtExactlyGotoReachIsAccepted)
{
  MissionRegistry registry(defaultParams(), missionsDir());
  // Two-point loop: leg out AND leg back both == goto_reach_copy_m (40.0).
  EXPECT_NO_THROW(
    registry.load(
      "indoor_patrol", "waypoints: [{x: 0, y: 0, z: 1.5}, {x: 40.0, y: 0, z: 1.5}]"));
}

// ===========================================================================
// mission_params validation: target_speed_mps > target_speed_max_mps.
// ===========================================================================

TEST(ParamsValidation, TargetSpeedAboveCapIsRejected)
{
  MissionRegistry registry(defaultParams(), missionsDir());
  EXPECT_THROW(registry.load("follow_target", "target_speed_mps: 0.3"), std::invalid_argument);
}

TEST(ParamsValidation, TargetSpeedAtExactlyCapIsAccepted)
{
  MissionRegistry registry(defaultParams(), missionsDir());
  EXPECT_NO_THROW(registry.load("follow_target", "target_speed_mps: 0.275"));
}

TEST(ParamsValidation, EmptyWaypointsListIsRejected)
{
  MissionRegistry registry(defaultParams(), missionsDir());
  EXPECT_THROW(registry.load("indoor_patrol", "waypoints: []"), std::invalid_argument);
}

TEST(ParamsValidation, LoopsAboveMaxLoopsIsRejected)
{
  MissionRegistry registry(defaultParams(), missionsDir());
  EXPECT_THROW(
    registry.load("indoor_patrol", "loops: 1001"), std::invalid_argument);
}

TEST(ParamsValidation, MarkerIdBelowNegativeOneIsRejected)
{
  MissionRegistry registry(defaultParams(), missionsDir());
  EXPECT_THROW(registry.load("inspect_point", "marker_id: -2"), std::invalid_argument);
}

TEST(ParamsValidation, MarkerIdAnyOrValidIdIsAccepted)
{
  MissionRegistry registry(defaultParams(), missionsDir());
  EXPECT_NO_THROW(registry.load("inspect_point", "marker_id: -1"));
  EXPECT_NO_THROW(registry.load("inspect_point", "marker_id: 17"));
}

}  // namespace uav_mission
