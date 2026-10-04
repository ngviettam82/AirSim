#!/bin/bash
# P11.6 gate -- the stack must stand down when a pilot takes the aircraft, and must NOT
# stand down when nobody has.
#
# Runs src/uav_bringup/test/pilot_override_probe.py twice against a fresh simulator:
#
#   takeover  PX4 driven into POSCTL (what an RC override produces). Expect: no FAULT,
#             the detail names the pilot, and the aircraft stays in POSCTL.
#   control   PX4 driven into HOLD instead. Neither predicate covers AUTO_LOITER, so the
#             stack SHOULD re-engage offboard. This round is what proves the probe can
#             observe a re-grab; without it the takeover round is unfalsifiable.
#
# 🔴 OUTCOME 2026-08-25: THIS GATE CANNOT BE RUN ON THIS RIG. Kept in the tree to be run
# the day a transmitter exists. What was measured, one variable at a time:
#
#   armed OK  -> PX4 REFUSED the switch to POSCTL
#                (set_mode returned success=False, "autopilot did not enter requested
#                 mode"; the aircraft stayed in OFFBOARD, the session manager stayed
#                 ACTIVE -- nothing took anything back, the switch simply never happened)
#   armed DENIED ("Resolve system health failures first")
#             -> PX4 ACCEPTED POSCTL, flight_mode reached 3
#                and the latch correctly did NOT close, because updatePilotOverride()
#                clears on DISARMED by design -- on the ground every flight sits in
#                POSCTL and latching there would stop the stack ever engaging.
#
# So the rig cannot produce the one state this gate needs: ARMED **and** in a stick-flown
# mode. That is not a defect and not a scripting problem -- an RC override is by
# definition a transmitter action, and there is no transmitter. It is the same blocker as
# P11.5, reached from a different direction.
#
# The fix itself is verified where it can be: px4_interop's pilotIsFlying() and
# updatePilotOverride() carry 8 unit tests including the two that matter most (the ground
# state must never latch; the latch must survive the nav_state moving off a pilot mode),
# and M5 3/3 proves the latch does not block normal engagement.
#
# ⚠️ Known limit of the latch, worth re-reading when the radio arrives: it closes on
# seeing a pilot mode in /fmu/out/vehicle_status, which PX4 publishes at ~2 Hz. A
# takeover shorter than one sample would be invisible to it. A real RC override leaves
# the aircraft with the pilot for far longer than 500 ms, so this is a documented edge,
# not a live hazard -- but it is the first thing to check on hardware.
#
# Every path is inside this file (stop_sim.sh kill patterns -- ops-playbook S:22).
#
# Usage: bash scripts/verify_pilot_override.sh
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
OUT=$WORKSPACE/gate_logs/p11_6
mkdir -p "$OUT"
export UAV_MODEL=${UAV_MODEL:-uav0}
export UAV_WORLD=${UAV_WORLD:-uav_arena}
PROBE=$WORKSPACE/src/uav_bringup/test/pilot_override_probe.py

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

[ -f "$PROBE" ] || { echo "FATAL: $PROBE missing"; exit 2; }

# Containers share the host PID view, so a colcon inside a Docker image build looks like
# a competing host build. Only host processes can touch this workspace's install/.
host_builders() {
  local pid found=0
  for pid in $(pgrep -x 'colcon|ctest' 2>/dev/null); do
    [ "$(readlink "/proc/$pid/root" 2>/dev/null)" = "/" ] && found=$((found + 1))
  done
  echo "$found"
}
[ "$(host_builders)" -eq 0 ] || { echo "FATAL: a build/test run holds install/"; exit 2; }

STACK_PID=""
cleanup() {
  [ -n "$STACK_PID" ] && { kill "$STACK_PID" 2>/dev/null; wait "$STACK_PID" 2>/dev/null; }
  bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1
}
trap cleanup EXIT

run_round() {
  local round=$1
  echo
  echo "############ round: $round ############"
  bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1
  sleep 3
  find /dev/shm -maxdepth 1 -name 'fastrtps_*' -delete 2>/dev/null
  bash "$WORKSPACE/scripts/start_sim.sh" > "$OUT/start_$round.log" 2>&1

  local deadline=$((SECONDS + 420))
  until timeout 20 ros2 topic list --no-daemon 2>/dev/null | grep -q '/fmu/out/vehicle_odometry'; do
    [ $SECONDS -gt $deadline ] && { echo "  FAILED TO MEASURE: PX4 never published"; return 2; }
    sleep 10
  done

  ros2 launch uav_bringup sim.launch.py > "$OUT/stack_$round.log" 2>&1 &
  STACK_PID=$!
  sleep 30

  python3 "$PROBE" --round "$round" 2>&1 | tee "$OUT/probe_$round.log"
  local rc=${PIPESTATUS[0]}

  kill "$STACK_PID" 2>/dev/null; wait "$STACK_PID" 2>/dev/null; STACK_PID=""
  bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1
  return "$rc"
}

run_round takeover; rc_t=$?
run_round control;  rc_c=$?

echo
echo "############ VERDICT ############"
echo "  takeover round exit $rc_t   (0 = stack stood down for the pilot)"
echo "  control  round exit $rc_c   (0 = stack re-engaged when nobody had the aircraft)"
echo
if [ "$rc_t" -eq 2 ] || [ "$rc_c" -eq 2 ]; then
  echo "  RESULT: FAILED TO MEASURE -- no conclusion (R30)."
  echo "  (evidence: $OUT)"
  exit 2
fi
if [ "$rc_t" -ne 0 ] || [ "$rc_c" -ne 0 ]; then
  echo "  RESULT: P11.6 NOT MET."
  echo "  (evidence: $OUT)"
  exit 1
fi
echo "  RESULT: P11.6 MET -- the stack stands down for a pilot and only for a pilot."
echo "  (evidence: $OUT)"
