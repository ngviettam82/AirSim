#!/bin/bash
# Sync ONE Windows-canonical package into the WSL build tree and rebuild just it.
#
# Generalises sync_build_bringup.sh and o2_sync_build_observability.sh, which are the
# same eight lines with a name baked in -- and a hand-baked name is exactly the shape
# R34 was written about. Takes the package as an argument and verifies it exists rather
# than trusting the caller.
#
# 🪤 The touch at the end is not cosmetic. Copying from /mnt/c (NTFS) to ext4 carries
# mtimes that can land in the future, and make then reports "Clock skew detected. Your
# build may be incomplete" -- which is literal: a source newer than its own object can
# be skipped, so you get a build that silently did not rebuild what you changed
# (ops-playbook S:7).
#
# Usage: bash scripts/sync_build_package.sh <package> [--test]
set -o pipefail

PKG=$1
[ -n "$PKG" ] || { echo "usage: sync_build_package.sh <package> [--test]"; exit 2; }
WORKSPACE=$HOME/PX4_ROS2
SRC_WIN=/mnt/c/code/PX4_ROS2/src

[ -d "$SRC_WIN/$PKG" ] || { echo "FATAL: no such package: $SRC_WIN/$PKG"; exit 2; }

# 🪤 `pgrep -x colcon` alone is WRONG here, and it fired falsely the first time this ran.
# Docker containers share the host PID view, so a `colcon build` running INSIDE the
# arm64/amd64 image build is visible to pgrep on the host -- and that build cannot touch
# this workspace's install/ at all. Filter to host processes: a containerised process
# has /proc/PID/root pointing into the container's filesystem, the host's points at /.
host_builders() {
  local pid found=0
  for pid in $(pgrep -x 'colcon|ctest' 2>/dev/null); do
    [ "$(readlink "/proc/$pid/root" 2>/dev/null)" = "/" ] && found=$((found + 1))
  done
  echo "$found"
}
if [ "$(host_builders)" -gt 0 ]; then
  echo "FATAL: another build/test run on this host holds install/"; exit 2
fi

source /opt/ros/humble/setup.bash

echo "=== sync $PKG: Windows -> WSL ==="
rm -rf "${WORKSPACE:?}/src/$PKG"
cp -r "$SRC_WIN/$PKG" "$WORKSPACE/src/"
find "$WORKSPACE/src/$PKG" -exec touch {} +
diff -rq "$SRC_WIN/$PKG" "$WORKSPACE/src/$PKG" --exclude __pycache__ && echo "diff: IDENTICAL"

echo
echo "=== build $PKG ==="
cd "$WORKSPACE" || exit 1
[ -f "$WORKSPACE/install/setup.bash" ] && source "$WORKSPACE/install/setup.bash"
colcon build --packages-select "$PKG" 2>&1 | tail -15
build_status=${PIPESTATUS[0]}
[ "$build_status" -eq 0 ] || { echo "build exit status: $build_status"; exit "$build_status"; }

if [ "$2" = "--test" ]; then
  echo
  echo "=== test $PKG ==="
  colcon test --packages-select "$PKG" 2>&1 | tail -8
  colcon test-result --test-result-base "build/$PKG" --all 2>&1 | tail -12
fi

echo
echo "build exit status: $build_status"
exit "$build_status"
