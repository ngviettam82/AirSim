#!/bin/bash
# GATE G-E1 (P11.0) -- does EKF2 really restart all external-vision aiding once per
# second, as the open debt records?
#
# THE DEBT: starting_vision_{pos,vel,yaw}_fusion + reset_{pos,vel}_to_vision +
# reset_hgt_to_ev counted 71 times in 70 s at a period of exactly 1.004 s, on uav0_nav
# parked. Read as a product fault it means the estimator's states and covariances are
# re-initialised at 1 Hz, and in flight the velocity is yanked to the EV measurement
# every second -- harmless in sim where EV is ground truth, dangerous with real VIO.
#
# WHAT THIS GATE MEASURES INSTEAD. EKF2.cpp:1129 (v1.15.4) republishes the previous
# estimator_event_flags message once per second whenever nothing new happens, every
# latched bit still set. So counting messages counts republications. The honest quantity
# is `information_event_changes`, a counter EKF2 moves only when it has a real event.
#
# TWO ARMS, AND THE SECOND ONE IS THE POINT.
#
#   A  nominal      uav0_nav parked, full stack, no interference.
#                   Expected: real-event rate <= 0.10 /s (start-up transient only).
#
#   B  fault-injected  the same run, but the EV publisher is SUSPENDED and RESUMED on a
#                   fixed cycle. EKF2 stops all four EV sources after 2*EV_MAX_INTERVAL
#                   = 400 ms without data (ev_control.cpp) and restarts them on resume,
#                   so this manufactures GENUINE repeated events.
#                   Expected: rate >= 0.50 /s.
#
# Arm A alone proves nothing. A measure that has only ever returned one answer is an
# assertion, not a measurement -- and a sweep of all 391 logs in this project's history
# found NOT ONE above 0.376 /s, so the corpus cannot supply the other answer either.
# Arm B is what makes arm A's number mean something: same instrument, same rig, opposite
# verdict, on demand. (ops-playbook S:2: "health monitor chưa từng thấy lỗi là health
# monitor chưa được kiểm chứng".)
#
# PASSING THIS GATE DOES NOT PATCH ANYTHING. If arm A lands in the artefact band, the
# fix is to the RECORD, not to the code: nothing was ever wrong with EV aiding. That is
# the outcome the owner's boundary (params + our publish behaviour only) explicitly
# allows for -- no PX4 source is touched either way.
#
# Every path is inside this file (stop_sim.sh kill patterns -- ops-playbook S:22).
#
# Usage: bash scripts/gate_e1_ev_reset.sh
# Env:   WINDOW (default 90) seconds of logging per arm
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
CANON=/mnt/c/code/PX4_ROS2
LOGROOT=$HOME/PX4-Autopilot/build/px4_sitl_default/rootfs/log
OUT=$WORKSPACE/gate_logs/p11_0
mkdir -p "$OUT"

WINDOW=${WINDOW:-90}
export UAV_MODEL=uav0_nav
export UAV_WORLD=${UAV_WORLD:-uav_arena}

# Sized from EKF2: all four EV sources stop after 2*EV_MAX_INTERVAL = 400 ms with no
# data, so 0.6 s suspended guarantees a stop and 1.0 s resumed guarantees a restart.
# 1.6 s per cycle puts the manufactured rate near 0.6 /s, clear of the 0.5 band edge.
STOP_SEC=0.6
RUN_SEC=1.0

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

if pgrep -x colcon >/dev/null || pgrep -x ctest >/dev/null; then
  echo "FATAL: a build/test run holds install/"; exit 2
fi

STACK_PID=""
INJECT_PID=""
cleanup() {
  [ -n "$INJECT_PID" ] && kill "$INJECT_PID" 2>/dev/null
  # Never leave the EV publisher suspended: a SIGSTOPped process survives SIGTERM.
  pkill -CONT -f '[u]av_px4_backend/px4_external_odometry_node' 2>/dev/null
  [ -n "$STACK_PID" ] && { kill "$STACK_PID" 2>/dev/null; wait "$STACK_PID" 2>/dev/null; }
  bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1
}
trap cleanup EXIT

newest_ulg() {
  find "$LOGROOT" -name '*.ulg' -printf '%T@ %p\n' 2>/dev/null | sort -rn | head -1 | cut -d' ' -f2-
}

run_arm() {
  local arm=$1
  echo
  echo "############ ARM $arm ############"
  bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1
  sleep 3
  find /dev/shm -maxdepth 1 -name 'fastrtps_*' -delete 2>/dev/null

  local before
  before=$(newest_ulg)

  bash "$WORKSPACE/scripts/start_sim.sh" > "$OUT/start_$arm.log" 2>&1
  local deadline=$((SECONDS + 420))
  until timeout 20 ros2 topic list --no-daemon 2>/dev/null | grep -q '/fmu/out/vehicle_odometry'; do
    [ $SECONDS -gt $deadline ] && { echo "  FAILED TO MEASURE: PX4 never published"; return 2; }
    sleep 10
  done

  ros2 launch uav_bringup sim.launch.py > "$OUT/stack_$arm.log" 2>&1 &
  STACK_PID=$!
  sleep 30

  # The EV path must actually be alive, or both arms measure an absent stream.
  local evpid
  evpid=$(pgrep -f '[u]av_px4_backend/px4_external_odometry_node' | head -1)
  if [ -z "$evpid" ]; then
    echo "  FAILED TO MEASURE: px4_external_odometry_node is not running"
    return 2
  fi
  echo "  EV publisher pid $evpid"

  if [ "$arm" = "B" ]; then
    echo "  injecting: SUSPEND ${STOP_SEC}s / RESUME ${RUN_SEC}s for ${WINDOW}s"
    (
      end=$((SECONDS + WINDOW))
      while [ $SECONDS -lt $end ]; do
        kill -STOP "$evpid" 2>/dev/null
        sleep "$STOP_SEC"
        kill -CONT "$evpid" 2>/dev/null
        sleep "$RUN_SEC"
      done
    ) &
    INJECT_PID=$!
  else
    echo "  nominal: no interference"
  fi

  sleep "$WINDOW"

  if [ -n "$INJECT_PID" ]; then
    kill "$INJECT_PID" 2>/dev/null; wait "$INJECT_PID" 2>/dev/null; INJECT_PID=""
    kill -CONT "$evpid" 2>/dev/null
    sleep 2
  fi

  kill "$STACK_PID" 2>/dev/null; wait "$STACK_PID" 2>/dev/null; STACK_PID=""
  bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1
  sleep 3

  local after
  after=$(newest_ulg)
  if [ "$after" = "$before" ] || [ -z "$after" ]; then
    echo "  FAILED TO MEASURE: no new ulog was produced by arm $arm"
    return 2
  fi
  echo "  ulog: $after"
  python3 "$CANON/scripts/ev_reset_cadence_report.py" "$after" > "$OUT/report_$arm.txt" 2>&1
  local rc=$?
  sed 's/^/    /' "$OUT/report_$arm.txt"
  return $rc
}

rateof() {
  grep -oE 'real events [0-9]+ over [0-9.]+ s = [0-9.]+ /s' "$1" \
    | tail -1 | grep -oE '[0-9.]+ /s' | grep -oE '^[0-9.]+'
}

# Independent corroboration, asserted rather than merely printed. vehicle_local_position's
# xy_reset_counter is a different topic on a different code path, incremented inside the
# very function the debt says runs 71 times. If the two measures ever disagree, one of
# them is broken and the gate must not pass on the strength of the other.
xy_reset_rate() {
  grep -oE 'xy_reset_counter +moved +[0-9]+ time\(s\) over [0-9.]+ s = [0-9.]+ /s' "$1" \
    | head -1 | grep -oE '= [0-9.]+ /s' | grep -oE '[0-9.]+'
}

run_arm A; rc_a=$?
run_arm B; rc_b=$?

echo
echo "############ VERDICT ############"
if [ "$rc_a" -eq 2 ] || [ "$rc_b" -eq 2 ]; then
  echo "  RESULT: FAILED TO MEASURE -- no conclusion drawn (R30)."
  exit 2
fi
rate_a=$(rateof "$OUT/report_A.txt")
rate_b=$(rateof "$OUT/report_B.txt")
echo "  arm A (nominal)        real-event rate: ${rate_a:-?} /s   (must be <= 0.10)"
echo "  arm B (fault-injected) real-event rate: ${rate_b:-?} /s   (must be >= 0.50)"
echo

fail=0
awk -v r="${rate_a:-99}" 'BEGIN{exit !(r <= 0.10)}' || {
  echo "  FAIL: arm A is above the artefact band -- EV events really do recur."; fail=1; }
awk -v r="${rate_b:-0}" 'BEGIN{exit !(r >= 0.50)}' || {
  echo "  FAIL: arm B did not reach the real band. The instrument has NOT been shown to"
  echo "        detect genuine repeated events, so arm A's low number proves nothing."
  fail=1; }

xy_a=$(xy_reset_rate "$OUT/report_A.txt")
xy_b=$(xy_reset_rate "$OUT/report_B.txt")
echo "  corroboration, vehicle_local_position.xy_reset_counter:"
echo "    arm A ${xy_a:-?} /s (must be <= 0.10)   arm B ${xy_b:-?} /s (must be >= 0.50)"
awk -v r="${xy_a:-99}" 'BEGIN{exit !(r <= 0.10)}' || {
  echo "  FAIL: arm A's estimator really did reset position repeatedly -- the two"
  echo "        measures disagree, so neither may be used."; fail=1; }
awk -v r="${xy_b:-0}" 'BEGIN{exit !(r >= 0.50)}' || {
  echo "  FAIL: arm B injected genuine EV outages but the position reset counter did"
  echo "        not follow. The corroborating measure is not measuring what it claims."
  fail=1; }

echo
if [ "$fail" -ne 0 ]; then
  echo "  RESULT: G-E1 NOT MET."
  echo "  (evidence: $OUT)"
  exit 1
fi
echo "  RESULT: G-E1 MET -- the recorded debt is a MEASUREMENT ARTEFACT, and the"
echo "  measurement that says so is proven able to return the opposite answer."
echo "  (evidence: $OUT)"
