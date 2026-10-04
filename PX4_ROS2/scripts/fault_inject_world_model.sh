#!/bin/bash
# Break uav_world_model on purpose and check the tests turn red.
# Usage: fault_inject_world_model.sh
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
SRC=/mnt/c/code/PX4_ROS2/src/uav_world_model
source /opt/ros/humble/setup.bash

restore() {
  rm -rf "${WORKSPACE:?}/src/uav_world_model"
  cp -r "$SRC" "$WORKSPACE/src/"
}

apply_fault() {
  python3 - "$1" <<'PY'
import sys, pathlib
name = sys.argv[1]
root = pathlib.Path.home() / "PX4_ROS2/src/uav_world_model"

FAULTS = {
  # stale poses accepted: the gap check stops rejecting
  "F1_stale_pose_accepted": [
    ("src/pose_buffer.cpp", "if (gap > options_.max_pose_gap_sec) {", "if (false) {", 2),
  ],
  # a pose that states no error gets a made-up one
  "F2_unweighable_pose_accepted": [
    ("src/world_model_node.cpp",
     "    const double stddev = uav_world_model::worstAxisStddev(message.pose.covariance);\n"
     "    if (!uav_world_model::isUsableUncertainty(stddev)) {",
     "    double stddev = uav_world_model::worstAxisStddev(message.pose.covariance);\n"
     "    if (!uav_world_model::isUsableUncertainty(stddev)) {stddev = 1.0;}\n"
     "    if (false) {", 1),
  ],
  # an all-zero quaternion is taken as an attitude
  "F3_zero_attitude_accepted": [
    ("src/world_model_node.cpp",
     "  return std::isfinite(squared) && squared > 0.5 && squared < 2.0;",
     "  return std::isfinite(squared);", 1),
  ],
  # an unknown frame silently gets the body mount
  "F4_unknown_frame_guessed": [
    ("src/world_model_node.cpp",
     "    ++rejected_unknown_frame_;\n",
     "    ++rejected_unknown_frame_;\n    return &mounts_[base_frame_];\n", 1),
  ],
  # an empty obstacle map is published although nothing was seen
  "F5_empty_obstacle_map_published": [
    ("src/world_model_node.cpp",
     "    if (!map_.hasObstacleSource()) {\n      return;\n    }",
     "    if (!map_.hasObstacleSource()) {\n      ObstacleArray empty;\n"
     "      empty.header.frame_id = odom_frame_;\n      empty.uav_id = uav_id_;\n"
     "      obstacle_publisher_->publish(empty);\n      return;\n    }", 1),
  ],
  # range frozen at its observed value instead of recomputed
  "F6_frozen_obstacle_range": [
    ("src/world_model_node.cpp",
     "  return std::max(0.0, to_centre - half_diagonal);",
     "  (void)to_centre; (void)half_diagonal; return 2.5;", 1),
  ],
  # a landmark never grows more uncertain while it ages
  "F7_no_drift_growth": [
    ("src/uncertainty.cpp",
     "double driftGrowth(double age_sec, const DriftModel & model)\n{",
     "double driftGrowth(double age_sec, const DriftModel & model)\n{\n  (void)age_sec; (void)model; return 0.0;", 1),
  ],
}

for rel, old, new, count in FAULTS[name]:
    path = root / rel
    text = path.read_text()
    found = text.count(old)
    if found != count:
        sys.exit(f"patch anchor for {name} in {rel}: expected {count}, found {found}")
    path.write_text(text.replace(old, new))
print(f"applied {name}")
PY
}

declare -A EXPECT_RED=(
  [F1_stale_pose_accepted]="*AnObservationWithoutAFreshPoseIsDropped*"
  [F2_unweighable_pose_accepted]="*APoseWithoutAStatedErrorIsNotUsedForMapping*"
  [F3_zero_attitude_accepted]="*APoseWithoutAnAttitudeIsNotUsedForMapping*"
  [F4_unknown_frame_guessed]="*AnObservationInAnUnconfiguredFrameIsDropped*:*AMountMissingItsGeometryDropsItsObservations*"
  [F5_empty_obstacle_map_published]="*NoObstacleMapIsPublishedWithoutAnObstacleSource*"
  [F6_frozen_obstacle_range]="*ObstacleRangeIsRecomputedAgainstThePoseThatPublishesIt*"
  [F7_no_drift_growth]="*AgeAndUncertaintyGrowWhileTheMarkerIsNotSeenAgain*"
)

ORDER="F1_stale_pose_accepted F2_unweighable_pose_accepted F3_zero_attitude_accepted \
F4_unknown_frame_guessed F5_empty_obstacle_map_published F6_frozen_obstacle_range F7_no_drift_growth"

summary=""
for fault in $ORDER; do
  echo
  echo "================ $fault ================"
  restore
  apply_fault "$fault" || { summary="$summary\n$fault PATCH-FAILED"; continue; }
  cd "$WORKSPACE" || exit 1
  if ! colcon build --packages-select uav_world_model >/tmp/fault_build.log 2>&1; then
    echo "build failed"; tail -20 /tmp/fault_build.log
    summary="$summary\n$fault BUILD-FAILED"
    continue
  fi
  source "$WORKSPACE/install/setup.bash"
  ROS_DOMAIN_ID=62 "$WORKSPACE/build/uav_world_model/test_world_model_node" \
    --gtest_filter="${EXPECT_RED[$fault]}" >/tmp/fault_test.log 2>&1
  status=$?
  grep -E "^\[  FAILED  \]|^\[       OK \]|^\[  PASSED  \]" /tmp/fault_test.log | head -10
  if [ $status -ne 0 ]; then
    echo "RESULT: red as required"
    summary="$summary\n$fault -> RED (good)"
  else
    echo "RESULT: STILL GREEN - the test does not catch this"
    summary="$summary\n$fault -> GREEN (BAD)"
  fi
done

restore
cd "$WORKSPACE" && colcon build --packages-select uav_world_model >/tmp/fault_build.log 2>&1
echo
echo "=== restored clean source and rebuilt ==="
# shellcheck disable=SC2059
printf "$summary\n"
