#!/bin/bash
# P5.2: cross-check frame rate with independent C++ and Python consumers.
# See docs/ops-playbook.md S2 (debt #10, no set -u).
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
export UAV_MODEL=${UAV_MODEL:-uav0_full}
export UAV_WORLD=${UAV_WORLD:-uav_arena}
# Needs decimal point: bare int fails this double param.
SECONDS_WINDOW=${SECONDS_WINDOW:-30.0}

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

echo "=== simulator (model=$UAV_MODEL) ==="
bash "$WORKSPACE/scripts/start_sim.sh" 2>&1 | tail -2

deadline=$((SECONDS + 420))
until timeout 20 ros2 topic list --no-daemon 2>/dev/null \
    | grep -q '/fmu/out/vehicle_odometry'; do
  if [ $SECONDS -gt $deadline ]; then echo "FATAL: PX4 never published"; exit 1; fi
  sleep 10
done
echo "PX4 publishing after ${SECONDS}s"
sleep 10

echo
echo "=== C++ consumer ==="
ros2 run uav_sim_gz image_rate_probe --ros-args -p seconds:="$SECONDS_WINDOW" 2>&1 \
  | grep -v '^\['

echo
echo "=== Python consumer, same streams, for comparison ==="
python3 -u "$WORKSPACE/src/uav_sim_gz/tools/check_front_sensors.py" \
  --rgb-width 640 --rgb-height 480 --seconds "$SECONDS_WINDOW" 2>&1 | grep -E 'rate|RESULT'

echo
echo "=== real-time factor during the measurement ==="
bash "$WORKSPACE/scripts/sample_rtf.sh" "$UAV_WORLD" 60 | sed 's/^/  /'

bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1
