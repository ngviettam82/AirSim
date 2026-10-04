#!/bin/bash
# G4: injected GNSS drift must be CAUGHT, not silently followed.
# Usage: bash scripts/run_g4_gps_disagreement.sh
# Env:   G4_SEEDS (default "11 12 13")  G4_WINDOW_SEC (default 120)
#        UAV_MODEL (default uav0_nav)   UAV_WORLD (default uav_arena)
# What each check means: src/uav_bringup/test/g4_gps_disagreement_gate.py
#
# Why FOUR runs and not twelve. The detection mechanism is deterministic:
# classifyDisagreement() bites as soon as the gap exceeds the sources' stated
# uncertainty (VIO ~0.1 m, GPS eph 0.9 m), and over 120 s a tau=60 s sigma=5 m
# process reaches ~4.6 m -- tens of times the threshold. Repeating a
# deterministic mechanism adds no evidence. What DOES add evidence is (a) more
# than one random realisation, so a PASS is not one lucky draw, and (b) one
# control run with the injector OFF, which is the only thing that shows the
# monitor can tell a fault from quiet. The full 3x4 matrix stays reachable via
# G4_SEEDS/G4_REPEATS for the pre-flight campaign at P11, where coverage is
# worth more than wall-clock.
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
export UAV_MODEL=${UAV_MODEL:-uav0_nav}
export UAV_WORLD=${UAV_WORLD:-uav_arena}
SEEDS=${G4_SEEDS:-"11 12 13"}
REPEATS=${G4_REPEATS:-1}
WINDOW=${G4_WINDOW_SEC:-120}
LOGDIR=$HOME/gate_logs
mkdir -p "$LOGDIR"

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

PROBE=$WORKSPACE/src/uav_bringup/test/g4_gps_disagreement_gate.py
PARAMS=$WORKSPACE/install/uav_bringup/share/uav_bringup/config/localization_params.yaml
CANONICAL_PARAMS=/mnt/c/code/PX4_ROS2/src/uav_bringup/config/localization_params.yaml
SETTER=$WORKSPACE/src/uav_bringup/test/g4_set_drift.py
[ -f "$PROBE" ] || { echo "FATAL: $PROBE missing"; exit 2; }
[ -f "$SETTER" ] || { echo "FATAL: $SETTER missing"; exit 2; }
[ -f "$PARAMS" ] || { echo "FATAL: $PARAMS missing, build uav_bringup"; exit 2; }
[ -f "$CANONICAL_PARAMS" ] || { echo "FATAL: $CANONICAL_PARAMS missing"; exit 2; }

# R0: the injector must never be left on. Restore the shipped yaml whatever
# happens -- a sim that quietly keeps corrupting GNSS is a trap for the next run.
restore_params() {
  cp "$CANONICAL_PARAMS" "$PARAMS"
  echo "injector params restored from the canonical tree"
}
trap restore_params EXIT

pass=0
fail=0
unmeasured=0

run_round() {
  local label=$1 drift=$2 seed=$3 expectation=$4
  echo
  echo "############ $label ############"
  bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1
  sleep 3
  bash "$WORKSPACE/scripts/start_sim.sh" >/tmp/g4_start.log 2>&1
  local deadline=$((SECONDS + 420))
  until timeout 20 ros2 topic list --no-daemon 2>/dev/null \
      | grep -q '/fmu/out/vehicle_odometry'; do
    if [ $SECONDS -gt $deadline ]; then
      echo "  FAILED TO MEASURE: PX4 never published"
      unmeasured=$((unmeasured + 1))
      return
    fi
    sleep 10
  done

  # The stimulus goes in BEFORE the node exists, and is never read back from the
  # thing being measured (the spec's "separate stimulus from response").
  if ! python3 "$SETTER" "$PARAMS" "$drift" "$seed"; then
    echo "  FAILED TO MEASURE: could not write the stimulus"
    unmeasured=$((unmeasured + 1))
    return
  fi
  ros2 launch uav_bringup sim.launch.py \
    localization:=true perception:=false mission:=false \
    >"$LOGDIR/g4_stack_${seed}.log" 2>&1 &
  local stack_pid=$!
  sleep 15
  echo "  window=${WINDOW}s"

  python3 "$PROBE" --uav-id "${UAV_ID:-uav0}" --window-sec "$WINDOW" "$expectation"
  local verdict=$?
  case $verdict in
    0) pass=$((pass + 1)) ;;
    2) unmeasured=$((unmeasured + 1)) ;;
    *) fail=$((fail + 1)) ;;
  esac

  kill "$stack_pid" 2>/dev/null
  wait "$stack_pid" 2>/dev/null
}

for seed in $SEEDS; do
  for repeat in $(seq 1 "$REPEATS"); do
    run_round "drift ON seed=$seed run=$repeat" true "$seed" --expect-detection
  done
done
# The control comes LAST so a machine that degrades over the session cannot make
# it pass by being quiet early.
run_round "CONTROL drift OFF" false 0 --expect-quiet

bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1

echo
echo "############ G4 SUMMARY ############"
echo "  pass=$pass  fail=$fail  failed-to-measure=$unmeasured"
if [ "$fail" -ne 0 ]; then
  echo "  RESULT: FAIL"
  exit 1
fi
if [ "$unmeasured" -ne 0 ]; then
  echo "  RESULT: FAILED TO MEASURE (never folded into PASS -- R27-1)"
  exit 2
fi
echo "  RESULT: PASS"
