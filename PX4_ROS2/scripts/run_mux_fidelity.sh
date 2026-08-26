#!/bin/bash
# Gate G2-B/G2-E: the mux must not add error to the source it selected.
# Usage: run_mux_fidelity.sh          (no Gazebo, no PX4, no RTF - a few seconds)
# Env:   ROS_DOMAIN_ID (default 95, the uav_localization slot - R20)
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
SRC=/mnt/c/code/PX4_ROS2/src
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-95}

echo "=== 1/3 sync and build uav_localization ==="
rm -rf "${WORKSPACE:?}/src/uav_localization"
cp -r "$SRC/uav_localization" "$WORKSPACE/src/"
cd "$WORKSPACE" || exit 1
source /opt/ros/humble/setup.bash
colcon build --packages-select uav_localization 2>&1 | tail -3
source "$WORKSPACE/install/setup.bash"

BENCH=$WORKSPACE/install/uav_localization/lib/uav_localization/mux_fidelity_bench.py
[ -x "$BENCH" ] || { echo "FATAL: $BENCH not installed"; exit 1; }

echo
echo "=== 2/3 start the real mux (domain $ROS_DOMAIN_ID) ==="
# setsid detaches, so $! is not the node: kill by pattern or every run leaves one
# behind and the next measures a chorus. Twelve had piled up before this landed.
pkill -f uav_localization/localization_mux_node 2>/dev/null
sleep 1
setsid nohup ros2 run uav_localization localization_mux_node \
  > /tmp/mux_fidelity_mux.log 2>&1 < /dev/null &
sleep 3
running=$(pgrep -c -f uav_localization/localization_mux_node)
echo "mux instances running: $running"
if [ "$running" != 1 ]; then
  echo "FATAL: expected exactly one mux, found $running - this would measure a chorus"
  pkill -f uav_localization/localization_mux_node 2>/dev/null
  exit 2
fi

echo
echo "=== 3/3 drive it with two stub sources ==="
python3 "$BENCH" --label "${LABEL:-shipped}"
status=$?

pkill -f uav_localization/localization_mux_node 2>/dev/null

echo
echo "mux fidelity exit: $status  (0 pass, 1 fail, 2 failed to measure)"
exit "$status"
