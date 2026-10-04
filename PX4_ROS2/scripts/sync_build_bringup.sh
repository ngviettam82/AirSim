#!/bin/bash
# P10.8b: sync the Windows-canonical uav_bringup source into the WSL-native
# build tree, then rebuild just that package. Same pattern as
# o2_sync_build_observability.sh (verify_cleanup.sh's PACKAGES list predates
# neither package but doing a full 8-package rebuild for one launch-file
# edit is wasteful) -- one-off build helper, kept in scripts/.
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
SRC_WIN=/mnt/c/code/PX4_ROS2/src

source /opt/ros/humble/setup.bash

echo "=== sync uav_bringup Windows -> WSL ==="
rm -rf "${WORKSPACE:?}/src/uav_bringup"
cp -r "$SRC_WIN/uav_bringup" "$WORKSPACE/src/"
diff -rq "$SRC_WIN/uav_bringup" "$WORKSPACE/src/uav_bringup" \
  --exclude __pycache__ && echo "diff: IDENTICAL"

echo
echo "=== build ==="
cd "$WORKSPACE" || exit 1
[ -f "$WORKSPACE/install/setup.bash" ] && source "$WORKSPACE/install/setup.bash"
colcon build --packages-select uav_bringup 2>&1 | tail -20
build_status=${PIPESTATUS[0]}

echo
echo "=== installed launch file sanity check (camera_health_node wired) ==="
grep -n "camera_health_node" \
  "$WORKSPACE/install/uav_bringup/share/uav_bringup/launch/sim.launch.py"

echo
echo "build exit status: $build_status"
exit "$build_status"
