#!/bin/bash
# P11.2 -- prove real.launch.py is INERT without reviewed:=true, and that the guard is
# a guard and not a wall.
#
# The file inherits three decisions that cannot be closed from the simulator (P6's
# require_obstacle_feed, P8's safety_enforcement review, P10's blackbox default). Its
# defence is that a bare `ros2 launch` starts nothing and says why. A guard nobody has
# watched engage is a claim, so this engages it both ways:
#
#   1. bare launch          -> zero nodes, and the explanation is printed
#   2. reviewed:=true       -> the nodes actually come up
#
# Case 2 matters as much as case 1: a guard that never lets anything through would also
# pass case 1, and would be discovered on the day of the first flight.
#
# 🔴 This does NOT fly anything and does NOT touch a flight controller. It brings the
# stack up against whatever is on the bus and counts nodes. There is no aircraft.
#
# Usage: bash scripts/verify_real_launch_inert.sh
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
OUT=$WORKSPACE/gate_logs/p11_2
mkdir -p "$OUT"

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

if pgrep -x colcon >/dev/null || pgrep -x ctest >/dev/null; then
  echo "FATAL: a build/test run holds install/"; exit 2
fi

PID=""
cleanup() {
  [ -n "$PID" ] && { kill "$PID" 2>/dev/null; wait "$PID" 2>/dev/null; }
  pkill -f '[r]eal.launch.py' 2>/dev/null
  sleep 2
}
trap cleanup EXIT

count_nodes() {
  timeout 40 ros2 node list --no-daemon --spin-time 5 2>/dev/null \
    | grep -cE 'px4_state_adapter_node|localization_mux_node|safety_supervisor_node|control_authority_manager_node|navigator_action_server_node'
}

fails=0

echo "############ case 1: bare launch must start NOTHING ############"
ros2 launch uav_bringup real.launch.py > "$OUT/bare.log" 2>&1 &
PID=$!
sleep 20
n=$(count_nodes)
echo "  stack nodes seen: ${n:-0} (expected 0)"
if [ "${n:-0}" -ne 0 ]; then
  echo "  🔴 FAIL: real.launch.py started nodes without reviewed:=true"
  fails=$((fails + 1))
fi
if grep -q 'did NOT start anything' "$OUT/bare.log"; then
  echo "  ok  the refusal was explained on stdout"
else
  echo "  🔴 FAIL: it started nothing but never said why -- a silent no-op is worse"
  echo "          than a loud one; nobody would know to pass the flag."
  fails=$((fails + 1))
fi
cleanup
PID=""

echo
echo "############ case 2: reviewed:=true must let the stack up ############"
ros2 launch uav_bringup real.launch.py reviewed:=true > "$OUT/reviewed.log" 2>&1 &
PID=$!
sleep 25
n=$(count_nodes)
echo "  stack nodes seen: ${n:-0} (expected >= 4)"
if [ "${n:-0}" -lt 4 ]; then
  echo "  🔴 FAIL: the guard is a wall -- nothing comes up even when acknowledged."
  echo "          Last lines of the launch log:"
  tail -15 "$OUT/reviewed.log" | sed 's/^/            /'
  fails=$((fails + 1))
fi
if grep -q 'NOTE on blackbox default' "$OUT/reviewed.log"; then
  echo "  ok  the blackbox caveat was surfaced to whoever is about to fly"
else
  echo "  WARN: the blackbox note did not appear (not fatal, but it is the one"
  echo "        default most worth re-reading before a first flight)"
fi
cleanup
PID=""

echo
if [ "$fails" -ne 0 ]; then
  echo "RESULT: FAILED ($fails)"
  echo "(logs: $OUT)"
  exit 1
fi
echo "RESULT: PASS -- inert by default, live when acknowledged."
echo "(logs: $OUT)"
