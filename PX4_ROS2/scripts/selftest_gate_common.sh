#!/bin/bash
# Self-test for gate_common.sh's assert_single_writer -- cheap (bare ros2 CLI
# only, no Gazebo, no PX4, no colcon build) but it DOES touch a live ROS graph
# and `ros2 daemon`. Do not run this while another gate or another agent's
# bringup is live on the same ROS_DOMAIN_ID -- it will read as topic noise on
# their measurement, same reason sim runs are serialized (see caller brief).
# Usage: bash scripts/selftest_gate_common.sh
#
# NOT RUN as part of this patch (P9-debt, 2026-08-24): uav_observability was
# being bench-tested on the shared ROS graph at the time, and this script's
# own justification is "don't add ROS traffic while someone else is
# measuring" -- running it anyway would have been the exact mistake it exists
# to catch. Run it once that constraint is lifted; five checks below.
#
# N5 fix (P10-gate-debt review round 2, 2026-08-24) added cases 4/5 below --
# STILL NOT RUN for the same shared-graph reason above. The equivalent logic
# WAS proven offline (no ROS at all) with a fake `ros2` CLI shim that scripts
# canned `ros2 topic info` output; see the patch's own verification notes.
# Run cases 4/5 for real, on a live graph, once that becomes safe.
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
source /opt/ros/humble/setup.bash
source "$WORKSPACE/scripts/gate_common.sh"

FAILURES=0
check() {
  local desc="$1" got="$2" want="$3"
  if [ "$got" != "$want" ]; then
    echo "SELFTEST FAIL: $desc -- got '$got', want '$want'"
    FAILURES=$((FAILURES + 1))
  else
    echo "SELFTEST OK: $desc"
  fi
}

echo "=== case 1: nobody ever advertised this topic, expecting 1 -> FAILED TO MEASURE (2) ==="
# `ros2 topic info` on a topic with zero history returns an empty Publisher
# count line forever, so this must hit the deadline branch, never read as 0
# publishers found (that would silently look like a measurable "wrong count").
assert_single_writer /gate_common_selftest/nobody_publishes_here 1 5
check "unknown topic vs expected=1" "$?" "2"

echo
echo "=== case 2: this shell is the only publisher, expecting 1 -> OK (0) ==="
ros2 topic pub /gate_common_selftest/one_publisher std_msgs/msg/Empty "{}" -r 2 \
  > /tmp/selftest_pub.log 2>&1 < /dev/null &
PUB_PID=$!
sleep 3   # let discovery catch up before asserting
assert_single_writer /gate_common_selftest/one_publisher 1 10
check "single self-publisher vs expected=1" "$?" "0"

echo
echo "=== case 3: same publisher, but the caller expected 0 -> FAIL (1), not FTM ==="
assert_single_writer /gate_common_selftest/one_publisher 0 10
check "single publisher vs expected=0 (real mismatch)" "$?" "1"
kill "$PUB_PID" 2>/dev/null

echo
echo "=== case 4 (N5): /uav/<id>/ prefix present but NOBODY publishes state/vehicle ->"
echo "    FAILED TO MEASURE (2), even though a real double-writer sits on the target"
echo "    topic -- proves the positive control is fail-closed on its own, not just a"
echo "    convenience: without proof discovery converged, a genuine mismatch must NOT"
echo "    be reported as a clean FAIL(1)."
ros2 topic pub /uav/fakegate0/world/obstacle_map_local std_msgs/msg/Empty "{}" -r 2 \
  > /tmp/selftest_pub4.log 2>&1 < /dev/null &
PUB4_PID=$!
sleep 3
assert_single_writer /uav/fakegate0/world/obstacle_map_local 0 5
check "double-writer present but control topic never seen" "$?" "2"
kill "$PUB4_PID" 2>/dev/null

echo
echo "=== case 5 (N5): writer on the target topic appears AFTER the first sample ->"
echo "    FAIL (1), where the OLD single-read code silently PASSED (proven offline"
echo "    with a fake ros2 shim during this patch -- see gate_common.sh's own"
echo "    assert_single_writer doc comment)."
ros2 topic pub /uav/fakegate0/state/vehicle std_msgs/msg/Empty "{}" -r 2 \
  > /tmp/selftest_pub5_control.log 2>&1 < /dev/null &
CONTROL5_PID=$!
sleep 3
# "Late" double writer -- starts ~1s in, i.e. after what would have been the
# OLD code's single (and only) read, but well inside the NEW code's 3-sample
# / >=2s-apart hold window.
( sleep 1
  ros2 topic pub /uav/fakegate0/world/obstacle_map_local std_msgs/msg/Empty "{}" -r 2 \
    > /tmp/selftest_pub5_late.log 2>&1 < /dev/null ) &
LATE5_SUBSHELL=$!
assert_single_writer /uav/fakegate0/world/obstacle_map_local 0 15
check "late double-writer caught by the hold (not missed by a single early read)" "$?" "1"
kill "$CONTROL5_PID" 2>/dev/null
kill "$LATE5_SUBSHELL" 2>/dev/null
pkill -f 'ros2 topic pub /uav/fakegate0/world/obstacle_map_local' 2>/dev/null

echo
if [ "$FAILURES" -gt 0 ]; then
  echo "SELFTEST RESULT: $FAILURES check(s) FAILED"
  exit 1
fi
echo "SELFTEST RESULT: all checks PASS"
exit 0
