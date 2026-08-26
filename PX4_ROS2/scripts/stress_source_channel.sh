#!/bin/bash
# Run the source-channel test under CPU contention, the way parallel colcon does.
# Usage: stress_source_channel.sh [runs] [load_procs]
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
RUNS=${1:-5}
LOAD=${2:-24}

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

pids=""
for _ in $(seq 1 "$LOAD"); do
  ( while :; do :; done ) &
  pids="$pids $!"
done

fails=0
for run in $(seq 1 "$RUNS"); do
  if ROS_DOMAIN_ID=95 "$WORKSPACE/build/uav_localization/test_source_channel" \
      >"/tmp/sc_stress_$run.log" 2>&1; then
    echo "run $run ok"
  else
    fails=$((fails + 1))
    echo "--- run $run FAILED ---"
    grep -E "^\[  FAILED  \]|Failure$|Value of|Actual:|Expected|control:" "/tmp/sc_stress_$run.log" | head -12
  fi
done

# shellcheck disable=SC2086
kill $pids 2>/dev/null
wait 2>/dev/null
echo "failures: $fails / $RUNS"
