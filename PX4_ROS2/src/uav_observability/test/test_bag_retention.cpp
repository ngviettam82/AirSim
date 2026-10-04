// ROS-free (no domain, no rclcpp::init): the retention arithmetic is pinned
// by a test instead of trusting a real disk, same rule uav_safety's
// failsafe_policy tests follow.
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "uav_observability/bag_retention.hpp"

namespace uav_observability
{
namespace
{

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

RetentionBudget defaultBudget()
{
  RetentionBudget budget;
  budget.max_total_bytes = 1000.0;
  budget.min_free_bytes = 100.0;
  return budget;
}

}  // namespace

// ===========================================================================
// validateBudget: refuse to start on an unusable budget (fail on the
// ground), same discipline as FailsafePolicy::validate().
// ===========================================================================

TEST(ValidateBudget, DefaultBudgetIsAccepted)
{
  EXPECT_NO_THROW(validateBudget(defaultBudget()));
}

TEST(ValidateBudget, NegativeMaxTotalIsRejected)
{
  RetentionBudget budget = defaultBudget();
  budget.max_total_bytes = -1.0;
  EXPECT_THROW(validateBudget(budget), std::invalid_argument);
}

TEST(ValidateBudget, NegativeMinFreeIsRejected)
{
  RetentionBudget budget = defaultBudget();
  budget.min_free_bytes = -1.0;
  EXPECT_THROW(validateBudget(budget), std::invalid_argument);
}

TEST(ValidateBudget, NanMaxTotalIsRejected)
{
  RetentionBudget budget = defaultBudget();
  budget.max_total_bytes = kNan;
  EXPECT_THROW(validateBudget(budget), std::invalid_argument);
}

TEST(ValidateBudget, InfiniteMinFreeIsRejected)
{
  RetentionBudget budget = defaultBudget();
  budget.min_free_bytes = kInf;
  EXPECT_THROW(validateBudget(budget), std::invalid_argument);
}

TEST(ValidateBudget, ZeroBudgetIsAccepted)
{
  // 0 is a legitimate (if degenerate) "keep nothing, always tight" budget --
  // only negative/NaN/infinite are refused.
  RetentionBudget budget;
  budget.max_total_bytes = 0.0;
  budget.min_free_bytes = 0.0;
  EXPECT_NO_THROW(validateBudget(budget));
}

// plan() itself never validates (S:2.1: fail on the ground, not mid-flight)
// -- an unusable budget passed straight to plan() must not throw here.
TEST(ValidateBudget, PlanNeverThrowsOnABadBudgetItself)
{
  RetentionBudget budget;
  budget.max_total_bytes = -1.0;
  budget.min_free_bytes = -1.0;
  EXPECT_NO_THROW(plan({}, "", budget, 0.0));
}

// ===========================================================================
// Invariant 1: NEVER deletes active_path, even if it is the oldest/largest.
// ===========================================================================

TEST(BagRetention, NeverDeletesActivePathEvenIfOldestAndLargest)
{
  const std::vector<BagDirInfo> bags = {
    {"/bags/a", 900.0, 1.0},    // oldest AND largest, but this run's active
    {"/bags/b", 50.0, 5.0},
  };
  RetentionBudget budget;
  budget.max_total_bytes = 500.0;   // total 950 -- forces a real deletion attempt
  budget.min_free_bytes = 0.0;
  const RetentionPlan result = plan(bags, "/bags/a", budget, kInf);
  for (const auto & deleted : result.delete_paths) {
    EXPECT_NE(deleted, "/bags/a");
  }
  EXPECT_EQ(result.delete_paths, std::vector<std::string>{"/bags/b"})
    << "only the non-active candidate can ever be deleted";
  EXPECT_TRUE(result.must_stop_recording)
    << "b alone (50) cannot bring 900-byte active-only total under 500";
}

// Mutation counter (R27-3): swap WHICH bag is active -- the deletion must
// follow the active_path argument, not some incidental property (e.g. "the
// first bag in the vector is always kept").
TEST(BagRetention, SwappingActiveToADifferentBagDeletesTheFormerActiveInstead)
{
  const std::vector<BagDirInfo> bags = {
    {"/bags/a", 900.0, 1.0},
    {"/bags/b", 50.0, 5.0},
  };
  RetentionBudget budget;
  budget.max_total_bytes = 500.0;
  budget.min_free_bytes = 0.0;
  const RetentionPlan result = plan(bags, "/bags/b", budget, kInf);
  ASSERT_EQ(result.delete_paths.size(), 1u);
  EXPECT_EQ(result.delete_paths[0], "/bags/a")
    << "active_path moved to b -- a is now the only deletable candidate";
  EXPECT_FALSE(result.must_stop_recording) << "deleting a (900) brings total to 50 <= 500";
}

// ===========================================================================
// Invariant 2: deletes oldest mtime first, stops as soon as under budget
// (no over-deletion).
// ===========================================================================

TEST(BagRetention, DeletesOldestMtimeFirstAndStopsOnceUnderBudget)
{
  const std::vector<BagDirInfo> bags = {
    {"/bags/oldest", 400.0, 1.0},
    {"/bags/middle", 400.0, 2.0},
    {"/bags/newest", 400.0, 3.0},
    {"/bags/active", 0.0, 4.0},
  };
  RetentionBudget budget;
  budget.max_total_bytes = 900.0;   // total 1200 -> must free 300, one delete suffices
  budget.min_free_bytes = 0.0;
  const RetentionPlan result = plan(bags, "/bags/active", budget, kInf);
  ASSERT_EQ(result.delete_paths.size(), 1u);
  EXPECT_EQ(result.delete_paths[0], "/bags/oldest");
  EXPECT_FALSE(result.must_stop_recording);
}

// Mutation counter: reverse WHICH bag is oldest -- the deleted path must
// follow mtime, not insertion order or size (both bags here have equal
// bytes, so only mtime can be driving the choice).
TEST(BagRetention, DeletedPathTracksMtimeNotInsertionOrder)
{
  const std::vector<BagDirInfo> bags = {
    {"/bags/newest", 400.0, 3.0},   // now listed FIRST but is the newest
    {"/bags/oldest", 400.0, 1.0},
    {"/bags/active", 0.0, 4.0},
  };
  RetentionBudget budget;
  budget.max_total_bytes = 700.0;   // total 800 -> must free 100, one delete suffices
  budget.min_free_bytes = 0.0;
  const RetentionPlan result = plan(bags, "/bags/active", budget, kInf);
  ASSERT_EQ(result.delete_paths.size(), 1u);
  EXPECT_EQ(result.delete_paths[0], "/bags/oldest");
}

// ===========================================================================
// Invariant 3: deletes whole bag DIRECTORIES (one entry == one path), never
// a partial/file-level unit.
// ===========================================================================

TEST(BagRetention, DeletePathsAreWholeDirectoryEntriesUnaltered)
{
  const std::vector<BagDirInfo> bags = {
    {"/bags/2026-08-23T10-00-00", 900.0, 1.0},
    {"/bags/active", 0.0, 4.0},
  };
  RetentionBudget budget;
  budget.max_total_bytes = 500.0;
  budget.min_free_bytes = 0.0;
  const RetentionPlan result = plan(bags, "/bags/active", budget, kInf);
  ASSERT_EQ(result.delete_paths.size(), 1u);
  EXPECT_EQ(result.delete_paths[0], "/bags/2026-08-23T10-00-00")
    << "must be the exact directory path, no sub-path decomposition";
}

// Mutation counter: two directories with IDENTICAL byte totals but
// different path depths/shapes -- confirms the returned unit is the path
// string itself, not a byte-derived or reconstructed identifier.
TEST(BagRetention, EqualByteEntriesAreStillDistinguishedByPathNotBytes)
{
  const std::vector<BagDirInfo> bags = {
    {"/bags/nested/deep/2026-08-20T00-00-00", 400.0, 1.0},
    {"/bags/2026-08-21T00-00-00", 400.0, 2.0},
    {"/bags/active", 0.0, 3.0},
  };
  RetentionBudget budget;
  budget.max_total_bytes = 700.0;   // must free 100 -> only the oldest goes
  budget.min_free_bytes = 0.0;
  const RetentionPlan result = plan(bags, "/bags/active", budget, kInf);
  ASSERT_EQ(result.delete_paths.size(), 1u);
  EXPECT_EQ(result.delete_paths[0], "/bags/nested/deep/2026-08-20T00-00-00");
}

// ===========================================================================
// Invariant 4: candidates exhausted but STILL over budget -> must_stop_
// recording=true (never silent).
// ===========================================================================

TEST(BagRetention, ExhaustedCandidatesStillOverBudgetSetsMustStopRecording)
{
  const std::vector<BagDirInfo> bags = {
    {"/bags/active", 2000.0, 1.0},   // active alone already exceeds the budget
  };
  const RetentionPlan result = plan(bags, "/bags/active", defaultBudget(), 500.0);
  EXPECT_TRUE(result.delete_paths.empty()) << "active can never be a candidate";
  EXPECT_TRUE(result.must_stop_recording);
}

// Mutation counter: same shape, but budget loose enough that deleting ALL
// non-active candidates IS enough -- must_stop_recording must be false here,
// proving the flag is actually computed, not hardcoded true whenever
// candidates run out.
TEST(BagRetention, ExhaustingCandidatesButNowUnderBudgetLeavesMustStopFalse)
{
  const std::vector<BagDirInfo> bags = {
    {"/bags/old", 50.0, 1.0},
    {"/bags/active", 100.0, 2.0},
  };
  RetentionBudget budget;
  budget.max_total_bytes = 120.0;   // deleting /bags/old (50) brings total to 100 <= 120
  budget.min_free_bytes = 0.0;
  const RetentionPlan result = plan(bags, "/bags/active", budget, kInf);
  EXPECT_EQ(result.delete_paths, std::vector<std::string>{"/bags/old"});
  EXPECT_FALSE(result.must_stop_recording);
}

// ===========================================================================
// Invariant 5 (validateBudget) already covered above.
// ===========================================================================

// ===========================================================================
// Invariant 6: disk_free sufficient AND total under cap -> empty plan,
// must_stop=false.
// ===========================================================================

TEST(BagRetention, SufficientFreeSpaceAndUnderCapYieldsEmptyPlan)
{
  const std::vector<BagDirInfo> bags = {
    {"/bags/a", 100.0, 1.0},
    {"/bags/active", 50.0, 2.0},
  };
  const RetentionPlan result = plan(bags, "/bags/active", defaultBudget(), 500.0);
  EXPECT_TRUE(result.delete_paths.empty());
  EXPECT_FALSE(result.must_stop_recording);
}

// Mutation counter: identical bags/total-bytes, but disk_free is now BELOW
// min_free_bytes -- the plan must react (delete something / must_stop),
// proving the disk_free check is live, not a dead/hardcoded pass.
TEST(BagRetention, SameBagsButInsufficientDiskFreeTriggersDeletion)
{
  const std::vector<BagDirInfo> bags = {
    {"/bags/a", 100.0, 1.0},
    {"/bags/active", 50.0, 2.0},
  };
  // disk_free (50) is below min_free_bytes (100); deleting /bags/a (100
  // bytes) raises free space to 150, clearing the floor.
  const RetentionPlan result = plan(bags, "/bags/active", defaultBudget(), 50.0);
  EXPECT_EQ(result.delete_paths, std::vector<std::string>{"/bags/a"});
  EXPECT_FALSE(result.must_stop_recording);
}

// ===========================================================================
// Misc edge cases.
// ===========================================================================

TEST(BagRetention, EmptyBagListYieldsEmptyPlan)
{
  const RetentionPlan result = plan({}, "/bags/active", defaultBudget(), 500.0);
  EXPECT_TRUE(result.delete_paths.empty());
  EXPECT_FALSE(result.must_stop_recording);
}

TEST(BagRetention, ActivePathNotPresentInBagsListStillNeverDeletedByConstruction)
{
  // active_path need not literally appear in `bags` (e.g. writer just
  // opened it, directory listing hasn't caught up yet) -- nothing special
  // required, it simply can never match a candidate's path.
  const std::vector<BagDirInfo> bags = {
    {"/bags/old", 2000.0, 1.0},
  };
  const RetentionPlan result = plan(bags, "/bags/active-not-listed", defaultBudget(), 0.0);
  EXPECT_EQ(result.delete_paths, std::vector<std::string>{"/bags/old"});
}

// ===========================================================================
// C1(a): disk_free_bytes could not be measured (NaN, e.g. the caller's
// fs::space() erroring) must never read as "0 bytes free" -- the free-space
// criterion must contribute nothing, only max_total_bytes (independently
// measured from bag.bytes) may still gate.
// ===========================================================================

TEST(BagRetention, NanDiskFreeNeverTriggersDeletionWhenTotalBytesIsUnderCap)
{
  const std::vector<BagDirInfo> bags = {
    {"/bags/a", 100.0, 1.0},
    {"/bags/active", 50.0, 2.0},
  };
  // total_bytes (150) is comfortably under max_total_bytes (1000) -- only an
  // UNGUARDED "NaN reads as 0 free bytes" bug would delete anything here.
  const RetentionPlan result = plan(bags, "/bags/active", defaultBudget(), kNan);
  EXPECT_TRUE(result.delete_paths.empty())
    << "an unmeasurable free-space reading must never be treated as a full disk";
  EXPECT_FALSE(result.must_stop_recording);
}

// Mutation counter: same NaN disk_free, but max_total_bytes IS exceeded --
// the max_total_bytes criterion must still fire (proves NaN only neutralises
// the FREE-SPACE criterion, not the whole overBudget() check).
TEST(BagRetention, NanDiskFreeStillLeavesMaxTotalBytesCriterionLive)
{
  const std::vector<BagDirInfo> bags = {
    {"/bags/old", 900.0, 1.0},
    {"/bags/active", 50.0, 2.0},
  };
  RetentionBudget budget;
  budget.max_total_bytes = 500.0;   // total 950 -- over cap regardless of free space
  budget.min_free_bytes = 100.0;
  const RetentionPlan result = plan(bags, "/bags/active", budget, kNan);
  EXPECT_EQ(result.delete_paths, std::vector<std::string>{"/bags/old"});
}

// ===========================================================================
// C1(b): active_path built with a doubled separator (bag_root_ had a
// trailing slash, naively concatenated) must still match the SAME directory
// a real filesystem iterator would report without the doubled slash.
// ===========================================================================

TEST(BagRetention, ActivePathWithDoubledSeparatorStillMatchesNormalizedCandidate)
{
  const std::vector<BagDirInfo> bags = {
    {"/bags/active", 900.0, 1.0},   // as a real directory_iterator would report it
  };
  RetentionBudget budget;
  budget.max_total_bytes = 500.0;
  budget.min_free_bytes = 0.0;
  const RetentionPlan result = plan(bags, "/bags//active", budget, kInf);
  EXPECT_TRUE(result.delete_paths.empty())
    << "the active bag must never be deleted just because active_path has a doubled separator";
}

// ===========================================================================
// C1(d): a bag whose mtime could not be measured (NaN) must sort LAST --
// never deleted before a bag whose age IS known.
// ===========================================================================

TEST(BagRetention, UnmeasurableMtimeSortsLastNeverDeletedBeforeAKnownAgeBag)
{
  const std::vector<BagDirInfo> bags = {
    {"/bags/unmeasurable", 400.0, kNan},   // mtime unreadable, listed FIRST
    {"/bags/known_old", 400.0, 1.0},
    {"/bags/active", 0.0, 5.0},
  };
  RetentionBudget budget;
  budget.max_total_bytes = 700.0;   // must free 100 -- exactly one delete suffices
  budget.min_free_bytes = 0.0;
  const RetentionPlan result = plan(bags, "/bags/active", budget, kInf);
  ASSERT_EQ(result.delete_paths.size(), 1u);
  EXPECT_EQ(result.delete_paths[0], "/bags/known_old")
    << "a bag with an unmeasurable mtime must never be deleted before one whose age is known";
}

// Mutation counter: with the known-age bag ALSO gone (budget tight enough to
// need both), the unmeasurable one must be the LAST resort, not skipped
// forever -- proves it is deferred, not excluded outright.
TEST(BagRetention, UnmeasurableMtimeIsDeletedOnlyAsTheLastResort)
{
  const std::vector<BagDirInfo> bags = {
    {"/bags/unmeasurable", 400.0, kNan},
    {"/bags/known_old", 400.0, 1.0},
    {"/bags/active", 0.0, 5.0},
  };
  RetentionBudget budget;
  budget.max_total_bytes = 300.0;   // must free 500 -- both non-active bags needed
  budget.min_free_bytes = 0.0;
  const RetentionPlan result = plan(bags, "/bags/active", budget, kInf);
  ASSERT_EQ(result.delete_paths.size(), 2u);
  EXPECT_EQ(result.delete_paths[0], "/bags/known_old");
  EXPECT_EQ(result.delete_paths[1], "/bags/unmeasurable");
}

}  // namespace uav_observability
