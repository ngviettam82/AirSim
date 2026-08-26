#!/bin/bash
# Wrapper so S13 can be launched without the caller's command line containing a package
# path -- see docs/ops-playbook.md section 3, the "stop_sim.sh called inline commits
# suicide (exit 15)" entry.
#
# WHAT HAPPENED (2026-08-25). S13 was started as
#
#     wsl -e bash -lc 'cp -r .../src/uav_bringup/* ... && colcon build --packages-select
#                      uav_bringup && bash .../run_s13_obstacle_feed.sh'
#
# stop_sim.sh derives its kill patterns from the installed package names, so it runs
# pkill -f 'uav_bringup/'. That pattern matched the CALLER's own argv, which still held
# "src/uav_bringup/" from the copy step. The run killed the shell that started it, one
# line into the first arm, and reported exit 15 with a 60-byte log.
#
# The playbook already records this shape and its fix -- call through a file so the
# pattern never appears in a command line. Hitting it anyway is the argument for reading
# section 3 before writing a runner, not after.
#
# Usage: bash scripts/run_s13_wrapper.sh
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
CANON=/mnt/c/code/PX4_ROS2
LOGDIR=$HOME/gate_logs
mkdir -p "$LOGDIR"

source /opt/ros/humble/setup.bash
cd "$WORKSPACE" || exit 1

# Deliberately built with the package name split, for the same reason: this file's own
# argv is short, but the commands it runs are not, and a build line here would put the
# pattern back into a live process table entry.
PKG=uav_bringup
cp -r "$CANON/src/$PKG/." "$WORKSPACE/src/$PKG/"
colcon build --packages-select "$PKG" > "$LOGDIR/s13_build.log" 2>&1 \
  || { echo "BUILD FAILED"; tail -15 "$LOGDIR/s13_build.log"; exit 1; }
echo "BUILD OK"

bash "$CANON/scripts/run_s13_obstacle_feed.sh" > "$LOGDIR/s13_run.log" 2>&1
rc=$?
echo "S13 exit=$rc"
grep -E 'EVIDENCE|RESULT|FAIL|verdict|arm:' "$LOGDIR/s13_run.log" | tail -30
exit "$rc"
