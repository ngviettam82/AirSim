#!/bin/bash
# Why did PX4 refuse FLIGHT_MODE_POSITION when the P11.6 gate asked for it?
#
# The gate's takeover round got `success=False, 'autopilot did not enter requested mode'`
# while the SAME service switched to HOLD in the control round without complaint. So the
# mapping is not obviously wrong (kMainPosition = 3.0 matches PX4's
# PX4_CUSTOM_MAIN_MODE_POSCTL), and the question is what the commander objected to.
#
# Leading hypothesis: PX4 refuses stick-flown modes when no RC transmitter is present.
# That would be CORRECT behaviour -- POSCTL means "a human flies this with the sticks",
# and with no sticks attached there would be nothing holding the aircraft up. If that is
# the reason, it is also the answer to a bigger question: an RC override cannot be
# simulated without RC, so the stick-mode half of P11.6 is blocked by exactly the missing
# hardware that blocks P11.5 -- not by a defect, and not by anything a cleverer script
# could work around.
#
# Reads the reason out of the PX4 console rather than inferring it.
#
# Usage: bash scripts/diagnose_posctl_rejection.sh
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
OUT=$WORKSPACE/gate_logs/p11_6
mkdir -p "$OUT"
export UAV_MODEL=uav0
export UAV_WORLD=uav_arena

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

STACK_PID=""
cleanup() {
  [ -n "$STACK_PID" ] && { kill "$STACK_PID" 2>/dev/null; wait "$STACK_PID" 2>/dev/null; }
  bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1
}
trap cleanup EXIT

bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1
sleep 3
bash "$WORKSPACE/scripts/start_sim.sh" > "$OUT/diag_start.log" 2>&1
deadline=$((SECONDS + 420))
until timeout 20 ros2 topic list --no-daemon 2>/dev/null | grep -q '/fmu/out/vehicle_odometry'; do
  [ $SECONDS -gt $deadline ] && { echo "FATAL: PX4 never published"; exit 2; }
  sleep 10
done

echo "=== RC-related PX4 parameters as the rig actually has them ==="
# COM_RC_IN_MODE governs whether the commander demands a transmitter.
#   0 = RC transmitter required   1 = joystick only   4 = stick input disabled
for p in COM_RC_IN_MODE COM_RC_OVERRIDE COM_RCL_EXCEPT NAV_RCL_ACT; do
  val=$(grep -aoE "$p[^0-9-]*(-?[0-9]+)" /tmp/px4.log 2>/dev/null | tail -1)
  echo "  $p: ${val:-not seen in console}"
done

ros2 launch uav_bringup sim.launch.py > "$OUT/diag_stack.log" 2>&1 &
STACK_PID=$!
sleep 30

echo
echo "=== round 1: DISARMED, no setpoint stream ==="
before=$(wc -l < /tmp/px4.log)
timeout 30 ros2 service call /uav/uav0/backend/set_mode uav_interfaces/srv/SetFlightMode \
  "{mode: 3}" 2>&1 | tail -6
sleep 3
timeout 30 ros2 service call /uav/uav0/backend/set_mode uav_interfaces/srv/SetFlightMode \
  "{mode: 4}" >/dev/null 2>&1

echo
echo "=== round 2: ARMED and STREAMING -- the condition the gate ran in ==="
# One variable at a time: round 1 differed from the gate in TWO ways (disarmed, no
# stream), so its success did not identify which one mattered.
python3 "$WORKSPACE/src/uav_bringup/test/pilot_override_probe.py" --round takeover \
  > "$OUT/diag_armed_round.log" 2>&1 &
probe_pid=$!
sleep 55
echo "  probe said:"
sed 's/^/    /' "$OUT/diag_armed_round.log" | head -12
kill "$probe_pid" 2>/dev/null; wait "$probe_pid" 2>/dev/null

echo
echo "=== PX4 console across BOTH rounds ==="

sleep 5
echo
echo "=== what PX4's console said in response ==="
tail -n +"$((before + 1))" /tmp/px4.log 2>/dev/null \
  | grep -aiE 'reject|denied|not available|refus|mode|rc |manual|fail' | tail -20 \
  | tee "$OUT/px4_reason.log"

echo
echo "=== and the same request for HOLD, which the gate saw succeed ==="
before=$(wc -l < /tmp/px4.log)
timeout 30 ros2 service call /uav/uav0/backend/set_mode uav_interfaces/srv/SetFlightMode \
  "{mode: 5}" 2>&1 | tail -4
sleep 5
tail -n +"$((before + 1))" /tmp/px4.log 2>/dev/null \
  | grep -aiE 'reject|denied|not available|refus|mode|rc |manual|fail' | tail -10

echo
echo "(evidence: $OUT)"
