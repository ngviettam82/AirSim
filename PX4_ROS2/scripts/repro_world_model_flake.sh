#!/bin/bash
# Hunt flakes in the uav_world_model node test under CPU and DDS contention.
# Usage: repro_world_model_flake.sh [runs] [load_procs] [gtest_filter] [copies] [isolate]
# Env: isolate=1 gives each copy its own ROS_DOMAIN_ID, leaving only CPU contention.
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
RUNS=${1:-10}
LOAD=${2:-24}
FILTER=${3:-*}
COPIES=${4:-1}
ISOLATE=${5:-1}

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

BINARY=$WORKSPACE/build/uav_world_model/test_world_model_node
if [ ! -x "$BINARY" ]; then
  echo "missing $BINARY"
  exit 1
fi

echo "spawning $LOAD busy loops, $COPIES copies per run, isolate=$ISOLATE"
pids=""
for _ in $(seq 1 "$LOAD"); do
  ( while :; do :; done ) &
  pids="$pids $!"
done

fails=0
total=0
for run in $(seq 1 "$RUNS"); do
  copy_pids=""
  for copy in $(seq 1 "$COPIES"); do
    if [ "$ISOLATE" = "1" ]; then
      export ROS_DOMAIN_ID=$((60 + copy))
    fi
    "$BINARY" --gtest_filter="$FILTER" >"/tmp/wm_flake_${run}_${copy}.log" 2>&1 &
    copy_pids="$copy_pids $!"
  done
  for pid in $copy_pids; do
    if ! wait "$pid"; then
      fails=$((fails + 1))
    fi
    total=$((total + 1))
  done
  for copy in $(seq 1 "$COPIES"); do
    log="/tmp/wm_flake_${run}_${copy}.log"
    if grep -q "^\[  FAILED  \]" "$log"; then
      echo "--- run $run copy $copy FAILED ---"
      grep -E "^\[  FAILED  \]|Failure$|Value of|Actual:|Expected|control:|which is:" "$log" | head -14
    fi
  done
  echo "run $run done (failures so far: $fails/$total)"
done

# shellcheck disable=SC2086
kill $pids 2>/dev/null
wait 2>/dev/null

echo "failures: $fails / $total"
