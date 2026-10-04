#!/bin/bash
# C4 fix (P10.8b review): sync the Windows-canonical uav_control_authority
# source into the WSL-native build tree, then build + test JUST that package
# -- another agent is working uav_observability in parallel, so this must
# never touch --packages-select beyond this one package.
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
SRC_WIN=/mnt/c/code/PX4_ROS2/src
PACKAGE=uav_control_authority

source /opt/ros/humble/setup.bash

echo "=== sync $PACKAGE Windows -> WSL ==="
rm -rf "${WORKSPACE:?}/src/$PACKAGE"
cp -r "$SRC_WIN/$PACKAGE" "$WORKSPACE/src/"
diff -rq "$SRC_WIN/$PACKAGE" "$WORKSPACE/src/$PACKAGE" --exclude __pycache__ && echo "diff: IDENTICAL"

echo
echo "=== build ==="
cd "$WORKSPACE" || exit 1
[ -f "$WORKSPACE/install/setup.bash" ] && source "$WORKSPACE/install/setup.bash"
colcon build --packages-select "$PACKAGE" 2>&1 | tail -30
build_status=${PIPESTATUS[0]}

if [ "$build_status" -ne 0 ]; then
  echo "build exit status: $build_status"
  exit "$build_status"
fi

echo
echo "=== test ==="
source "$WORKSPACE/install/setup.bash"
colcon test --packages-select "$PACKAGE" --event-handlers console_direct+ 2>&1 | tail -200
test_status=${PIPESTATUS[0]}

echo
echo "=== test-result ==="
colcon test-result --all 2>&1 | tail -20

echo
echo "build exit status: $build_status, test exit status: $test_status"
exit "$test_status"
