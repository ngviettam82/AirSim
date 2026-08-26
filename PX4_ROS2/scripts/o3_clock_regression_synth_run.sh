#!/bin/bash
# Runner for o3_clock_regression_synth.py -- see that file's docstring.
set -o pipefail
WORKSPACE=$HOME/PX4_ROS2
CA_PARAMS=/mnt/c/code/PX4_ROS2/src/uav_control_authority/config/control_authority_params.yaml
SCRIPT=/mnt/c/code/PX4_ROS2/src/uav_observability/test/o3_clock_regression_synth.py

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
export ROS_DOMAIN_ID=99

pkill -f 'control_authority_manager_node' 2>/dev/null
pkill -f 'o3_clock_regression_synth.py' 2>/dev/null
sleep 1

CA_LOG=$HOME/gate_logs/o3_clock_synth_ca.log
: > "$CA_LOG"
timeout 60 ros2 run uav_control_authority control_authority_manager_node --ros-args \
  --params-file "$CA_PARAMS" -p uav_id:=uav0 -p use_sim_time:=true \
  > "$CA_LOG" 2>&1 < /dev/null &
CA_PID=$!

DEADLINE=$((SECONDS + 30))
until timeout 15 ros2 node list --no-daemon 2>/dev/null | grep -q control_authority_manager_node; do
  if [ $SECONDS -gt $DEADLINE ]; then
    echo "FATAL: control_authority_manager_node never came up"; tail -30 "$CA_LOG"
    kill -9 "$CA_PID" 2>/dev/null; exit 1
  fi
  sleep 1
done

python3 "$SCRIPT" --uav-id uav0
RESULT=$?

kill -9 "$CA_PID" 2>/dev/null
wait "$CA_PID" 2>/dev/null
unset ROS_DOMAIN_ID
echo "synthetic test exit=$RESULT"
exit "$RESULT"
