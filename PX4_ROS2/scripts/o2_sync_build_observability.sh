#!/bin/bash
# P10.7 G-O2: sync the Windows-canonical uav_observability source into the
# WSL-native build tree, then rebuild+test just that package. Same pattern
# as verify_cleanup.sh S:1/2 (that script's PACKAGES list predates P10 and
# doesn't include uav_observability). Not part of the gate itself -- a
# one-off build helper, kept in scripts/ per ops-playbook ("script phụ trợ
# dùng nhiều lần thì để ở $HOME/scripts, đừng để /tmp").
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
SRC_WIN=/mnt/c/code/PX4_ROS2/src

source /opt/ros/humble/setup.bash

echo "=== sync uav_observability Windows -> WSL ==="
rm -rf "${WORKSPACE:?}/src/uav_observability"
cp -r "$SRC_WIN/uav_observability" "$WORKSPACE/src/"
diff -rq "$SRC_WIN/uav_observability" "$WORKSPACE/src/uav_observability" \
  --exclude __pycache__ && echo "diff: IDENTICAL"

echo
echo "=== build ==="
cd "$WORKSPACE" || exit 1
[ -f "$WORKSPACE/install/setup.bash" ] && source "$WORKSPACE/install/setup.bash"
colcon build --packages-select uav_observability 2>&1 | tail -20
build_status=${PIPESTATUS[0]}

echo
echo "=== test ==="
source "$WORKSPACE/install/setup.bash"
colcon test --packages-select uav_observability 2>&1 | tail -6
colcon test-result --all 2>&1 | tail -12

echo
echo "=== installed yaml sanity check (flight_only_expected_hz) ==="
grep -A4 'flight_only_expected_hz' \
  "$WORKSPACE/install/uav_observability/share/uav_observability/config/observability_params.yaml"

echo
echo "build exit status: $build_status"
exit "$build_status"
