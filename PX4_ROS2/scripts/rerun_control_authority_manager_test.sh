#!/bin/bash
# C4 fix: isolate whether ReleaseHandoverGapStaysUnderTheGatewayTimeout is a
# pre-existing wall-clock flake (R21) or a real regression -- reruns just
# this ctest target N times back-to-back without rebuilding.
set -o pipefail
WORKSPACE=$HOME/PX4_ROS2
RUNS=${1:-5}

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
cd "$WORKSPACE/build/uav_control_authority" || exit 1

for i in $(seq 1 "$RUNS"); do
  echo "=== run $i/$RUNS ==="
  ROS_DOMAIN_ID=96 ./test_control_authority_manager_node \
    --gtest_filter='ControlAuthorityFixture.ReleaseHandoverGapStaysUnderTheGatewayTimeout' \
    2>&1 | grep -E 'RUN|OK|FAILED|release_handover_gap_sec|Failure'
done
