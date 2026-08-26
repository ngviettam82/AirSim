#!/bin/bash
# Break uav_localization's source channel on purpose and check the tests turn red.
# Usage: fault_inject_source_channel.sh
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
SRC=/mnt/c/code/PX4_ROS2/src/uav_localization
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

restore() {
  rm -rf "${WORKSPACE:?}/src/uav_localization"
  cp -r "$SRC" "$WORKSPACE/src/"
}

apply_fault() {
  python3 - "$1" <<'PY'
import sys, pathlib
name = sys.argv[1]
root = pathlib.Path.home() / "PX4_ROS2/src/uav_localization"

FAULTS = {
  # an untrustworthy report publishes a pose anyway
  "L1_untrustworthy_pose_published": [
    ("src/source_channel.cpp", "  if (!trustworthy) {", "  if (false) {", 1),
  ],
  # going unavailable still claims the last publish was valid
  "L2_unavailable_claims_valid": [
    ("src/source_channel.cpp",
     "  publishStatus(stamp, LocalizationStatus::QUALITY_BAD, false, -1.0, -1.0, reason);\n"
     "  last_valid_ = false;",
     "  publishStatus(stamp, LocalizationStatus::QUALITY_BAD, false, -1.0, -1.0, reason);\n"
     "  last_valid_ = true;", 1),
  ],
  # unknown covariance reported as a measured zero
  "L3_unknown_covariance_becomes_zero": [
    ("src/source_channel.cpp",
     "constexpr double kUnknownCovariance = -1.0;",
     "constexpr double kUnknownCovariance = 0.0;", 1),
  ],
  # the pose is never actually put on the wire
  "L4_pose_never_published": [
    ("src/source_channel.cpp",
     "  odometry_publisher_->publish(odometry);",
     "  (void)odometry;", 1),
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
  [L1_untrustworthy_pose_published]="*AnUntrustworthyReportYieldsStatusButNoPose*:*UnknownUncertaintyIsTreatedAsUntrustworthy*"
  [L2_unavailable_claims_valid]="*UnavailableIsAnnouncedRatherThanSilent*"
  [L3_unknown_covariance_becomes_zero]="*MissingHeadingIsMarkedUnknownNotPerfect*:*AnUnsetTwistIsNotReportedAsAMeasuredZero*"
  [L4_pose_never_published]="*AGoodReportPublishesBothPoseAndStatus*:*DegradedStillFliesButSaysSo*"
)

ORDER="L1_untrustworthy_pose_published L2_unavailable_claims_valid \
L3_unknown_covariance_becomes_zero L4_pose_never_published"

summary=""
for fault in $ORDER; do
  echo
  echo "================ $fault ================"
  restore
  apply_fault "$fault" || { summary="$summary\n$fault PATCH-FAILED"; continue; }
  cd "$WORKSPACE" || exit 1
  if ! colcon build --packages-select uav_localization >/tmp/lfault_build.log 2>&1; then
    echo "build failed"; tail -20 /tmp/lfault_build.log
    summary="$summary\n$fault BUILD-FAILED"
    continue
  fi
  ROS_DOMAIN_ID=95 "$WORKSPACE/build/uav_localization/test_source_channel" \
    --gtest_filter="${EXPECT_RED[$fault]}" >/tmp/lfault_test.log 2>&1
  status=$?
  grep -E "^\[  FAILED  \]|^\[       OK \]|^\[  PASSED  \]" /tmp/lfault_test.log | head -10
  if [ $status -ne 0 ]; then
    echo "RESULT: red as required"
    summary="$summary\n$fault -> RED (good)"
  else
    echo "RESULT: STILL GREEN - the test does not catch this"
    summary="$summary\n$fault -> GREEN (BAD)"
  fi
done

restore
cd "$WORKSPACE" && colcon build --packages-select uav_localization >/tmp/lfault_build.log 2>&1
echo
echo "=== restored clean source and rebuilt ==="
# shellcheck disable=SC2059
printf "$summary\n"
