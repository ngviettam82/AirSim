#!/bin/bash
# Runs the P11 gates that need a simulator, back to back, and reports all of them even
# if an early one fails -- a run that stops at the first red tells you less than one
# that tells you which of three things broke.
#
#   1. verify_real_launch_inert.sh  P11.2  real.launch.py is inert until acknowledged
#   2. verify_pilot_override.sh     P11.6  the stack stands down for a pilot, and only then
#   3. run_m5_regression.sh         R14    mandatory: this session changed product code in
#                                          uav_px4_backend (offboard_session_manager_node)
#
# M5 is last on purpose. It is the RTF-sensitive one, and the other two leave the machine
# in a known-idle state when they finish.
#
# Usage: bash scripts/run_p11_sim_gates.sh
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
OUT=$WORKSPACE/gate_logs/p11_sim
mkdir -p "$OUT"

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

# Containers share the host PID view; a colcon inside a Docker build is not a competitor
# for this workspace's install/.
host_builders() {
  local pid found=0
  for pid in $(pgrep -x 'colcon|ctest' 2>/dev/null); do
    [ "$(readlink "/proc/$pid/root" 2>/dev/null)" = "/" ] && found=$((found + 1))
  done
  echo "$found"
}
[ "$(host_builders)" -eq 0 ] || { echo "FATAL: a build/test run holds install/"; exit 2; }

declare -A RC
run() {
  local name=$1; shift
  echo
  echo "################################################################"
  echo "### $name"
  echo "################################################################"
  bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1
  sleep 3
  "$@" > "$OUT/$name.log" 2>&1
  RC[$name]=$?
  tail -22 "$OUT/$name.log"
  echo "  --> $name exit ${RC[$name]}"
}

run real_launch_inert bash "$WORKSPACE/scripts/verify_real_launch_inert.sh"
run pilot_override    bash "$WORKSPACE/scripts/verify_pilot_override.sh"
run m5_regression     bash "$WORKSPACE/scripts/run_m5_regression.sh"

bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1

echo
echo "############ P11 SIM GATE SUMMARY ############"
fails=0
for name in real_launch_inert pilot_override m5_regression; do
  printf '  %-20s exit %s\n' "$name" "${RC[$name]}"
  [ "${RC[$name]}" -ne 0 ] && fails=$((fails + 1))
done
echo
if [ "$fails" -ne 0 ]; then
  echo "  RESULT: $fails gate(s) not met. Logs: $OUT"
  exit 1
fi
echo "  RESULT: all three met. Logs: $OUT"
