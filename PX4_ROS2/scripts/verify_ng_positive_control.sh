#!/bin/bash
# N-g had a guard but no positive control: the stall witness in
# test_control_authority_manager_node had never been observed to bite, and the
# project's own doctrine says a shield never seen to bite is not evidence.
# This builds the new control test and runs the whole target N times, so the
# control is proved AND the guard is shown not to have started swallowing the
# real assertions (the failure mode a guard like this introduces).
#
# Usage: bash scripts/verify_ng_positive_control.sh [runs]
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
CANON=/mnt/c/code/PX4_ROS2
RUNS=${1:-6}

ps -eo comm | grep -qE '^colcon$|^cc1plus$' && { echo "FATAL: a build is running"; exit 2; }

source /opt/ros/humble/setup.bash

rm -rf "$WORKSPACE/src/uav_control_authority"
cp -r "$CANON/src/uav_control_authority" "$WORKSPACE/src/"
( cd "$WORKSPACE" && colcon build --packages-select uav_control_authority ) \
  || { echo "FATAL: build failed"; exit 2; }
source "$WORKSPACE/install/setup.bash"

BIN=$WORKSPACE/build/uav_control_authority/test_control_authority_manager_node
[ -x "$BIN" ] || { echo "FATAL: $BIN missing"; exit 2; }

echo
echo "########## 1/2 the positive control alone, $RUNS runs ##########"
control_ok=0
for i in $(seq 1 "$RUNS"); do
  out=$(ROS_DOMAIN_ID=96 "$BIN" \
    --gtest_filter='*TheStallWitnessSeesAnInjectedStallAndStillJudgesAHealthyProbe*' 2>&1)
  verdict=$(echo "$out" | grep -cE '^\[  PASSED  \] 1 test')
  witnessed=$(echo "$out" | grep -oE 'witness saw [0-9.]+' | grep -oE '[0-9.]+')
  echo "  run $i: passed=$verdict  witnessed_stall=${witnessed:-<none>} s"
  control_ok=$((control_ok + verdict))
done

echo
echo "########## 2/2 the WHOLE target, $RUNS runs -- the guard must not start ##########"
echo "##########     skipping the real claim on a healthy machine        ##########"
pass=0; skipped_total=0
for i in $(seq 1 "$RUNS"); do
  out=$(ROS_DOMAIN_ID=96 "$BIN" 2>&1)
  code=$?
  n_pass=$(echo "$out" | grep -oE '^\[  PASSED  \] [0-9]+' | grep -oE '[0-9]+$' | head -1)
  n_skip=$(echo "$out" | grep -cE '^\[  SKIPPED \]')
  gap=$(echo "$out" | grep -oE 'max gap [0-9.]+' | grep -oE '[0-9.]+' | head -1)
  stall=$(echo "$out" | grep -oE 'probe stall [0-9.]+' | grep -oE '[0-9.]+' | head -1)
  echo "  run $i: exit=$code passed=${n_pass:-?} skipped=$n_skip  gap=${gap:-?} s stall=${stall:-?} s"
  [ "$code" -eq 0 ] && pass=$((pass + 1))
  skipped_total=$((skipped_total + n_skip))
done

echo
echo "########## VERDICT ##########"
echo "  positive control passed      : $control_ok/$RUNS  (the witness SEES an injected stall)"
echo "  whole target exit 0          : $pass/$RUNS"
echo "  runs where the guard fired   : $skipped_total  (expected 0 on an idle machine --"
echo "                                 nonzero here would mean the guard is over-eager)"
[ "$control_ok" -eq "$RUNS" ] && [ "$pass" -eq "$RUNS" ] || exit 1
