#!/bin/bash
# G-N1 on the vision-primary profile, with the estimator watched while flying.
# The 1 Hz EV re-init is harmless while parked; its real cost (velocity forced to
# the EV value every second, covariance reset) can only show up in motion.
# Usage: bash scripts/verify_navigator_monitored.sh
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
SRC=/mnt/c/code/PX4_ROS2/src
# R34: derived, never hand-copied. The old list predated P7 and would fly this
# gate against a stale control_authority -- which owns command_selected.
PACKAGES=$(find "$SRC" -maxdepth 1 -mindepth 1 -type d -printf '%f\n' | sort | tr '\n' ' ')
if [ -z "$PACKAGES" ]; then
  echo "FATAL: no packages found under $SRC"
  exit 1
fi

source /opt/ros/humble/setup.bash

echo "=== 0/3 sync canonical sources (the airframe is a symlink into this tree) ==="
echo "  $(echo "$PACKAGES" | wc -w) packages: $PACKAGES"
for package in $PACKAGES; do
  rm -rf "${WORKSPACE:?}/src/$package"
  cp -r "$SRC/$package" "$WORKSPACE/src/"
done
cd "$WORKSPACE" || exit 1
# shellcheck disable=SC2086
colcon build --packages-select $PACKAGES 2>&1 | tail -3
build_status=${PIPESTATUS[0]}
if [ "$build_status" != "0" ]; then
  echo "FATAL: build failed, not flying"
  exit 1
fi
echo -n "vision-primary profile present in the build tree: "
grep -c 'EKF2_HGT_REF 3' "$WORKSPACE/src/uav_sim_gz/airframes/4102_gz_uav0_nav"

echo
echo "=== 1/3 gate flight ==="
bash "$WORKSPACE/scripts/verify_navigator.sh"
verdict=$?

echo
echo "=== 2/3 estimator behaviour during that flight ==="
ULOG=$(ls -t "$HOME"/PX4-Autopilot/build/px4_sitl_default/rootfs/log/*/*.ulg 2>/dev/null | head -1)
if [ -z "$ULOG" ]; then
  echo "  no ulog found; estimator evidence unavailable"
else
  echo "  ulog: $ULOG"
  PYTHONNOUSERSITE=1 python3 "$WORKSPACE/src/uav_sim_gz/tools/probe_estimator_aiding.py" "$ULOG" 2>&1 | tail -25
  PYTHONNOUSERSITE=1 python3 "$WORKSPACE/src/uav_sim_gz/tools/check_ev_duplicate.py" "$ULOG" 2>&1 | tail -5
fi

echo
echo "=== 3/3 verdict ==="
echo "G-N1 exit status: $verdict"
exit "$verdict"
