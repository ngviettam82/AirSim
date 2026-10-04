#!/bin/bash
# Build and test every package after the comment cleanup, then re-audit.
# Six agents edited in parallel without building; this is the gate.
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
SRC=/mnt/c/code/PX4_ROS2/src
# R34: derived from the canonical tree, never hand-copied. The old hand-kept
# list froze at 8 packages and silently skipped safety/mission/observability.
PACKAGES=$(find "$SRC" -maxdepth 1 -mindepth 1 -type d -printf '%f\n' | sort | tr '\n' ' ')
if [ -z "$PACKAGES" ]; then
  echo "FATAL: no packages found under $SRC"
  exit 1
fi

source /opt/ros/humble/setup.bash

echo "=== 0/4 packages derived from $SRC ==="
echo "  $(echo "$PACKAGES" | wc -w) packages: $PACKAGES"

echo "=== 1/4 sync canonical sources into the build workspace ==="
for package in $PACKAGES; do
  rm -rf "${WORKSPACE:?}/src/$package"
  cp -r "$SRC/$package" "$WORKSPACE/src/"
  # NTFS mtimes land slightly in WSL's future, so gmake prints "Clock skew
  # detected. Your build may be incomplete" -- and it means it: a source that
  # looks newer than its own object can be skipped. Restamp to now.
  find "$WORKSPACE/src/$package" -exec touch {} +
done
echo "synced: $PACKAGES"

echo
echo "=== 2/4 build ==="
cd "$WORKSPACE" || exit 1
# CLEAN=1 forces a from-scratch build of our packages; px4_msgs is left alone
# because rebuilding it costs far more than it proves.
if [ "${CLEAN:-0}" = "1" ]; then
  for package in $PACKAGES; do
    rm -rf "build/$package" "install/$package"
  done
  echo "removed build/ and install/ for: $PACKAGES"
fi
# px4_msgs lives in install/ only; without this, re-configuring the backend fails.
[ -f "$WORKSPACE/install/setup.bash" ] && source "$WORKSPACE/install/setup.bash"
# shellcheck disable=SC2086
colcon build --packages-select $PACKAGES 2>&1 | tail -15
build_status=${PIPESTATUS[0]}

echo
echo "=== 3/4 test ==="
source "$WORKSPACE/install/setup.bash"
# shellcheck disable=SC2086
colcon test --packages-select $PACKAGES 2>&1 | tail -6
colcon test-result --all 2>&1 | tail -12

echo
echo "=== 3b/4 syntax-check installed python ==="
# colcon copies these without parsing them, so a syntax error would only appear
# mid-flight.
find "$SRC" -name '*.py' -not -path '*__pycache__*' -print0 \
  | xargs -0 -n1 python3 -m py_compile \
  && echo "  all python parses clean" || echo "  PYTHON SYNTAX ERROR ABOVE"

echo
echo "=== 4/4 re-audit comment density ==="
bash "$SRC/../scripts/audit_comments.sh" 2>&1 | head -12

echo
echo "build exit status: $build_status"
exit "$build_status"
