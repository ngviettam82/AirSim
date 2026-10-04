#!/bin/bash
# Open the BK world alone in the Gazebo GUI.
# See docs/ops-playbook.md S1 (gz sim attaches to a running server).
set -o pipefail

WORLD=${WORLD:-$HOME/PX4_ROS2/install/uav_sim_gz/share/uav_sim_gz/worlds/uav_arena_bk.sdf}
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA

echo "stopping anything already simulating..."
pkill -f 'px4_sitl_default/bin/px4' 2>/dev/null
pkill -f 'gz sim' 2>/dev/null
pkill -f 'ruby.*gz' 2>/dev/null
sleep 3
remaining=$(pgrep -fc 'gz sim' 2>/dev/null || echo 0)
echo "gz processes still alive: $remaining"

echo "opening: $WORLD"
grep -c '<model name=' "$WORLD"
exec gz sim "$WORLD"
