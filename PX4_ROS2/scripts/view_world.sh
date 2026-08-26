#!/bin/bash
# Open a project world in the Gazebo GUI.
# See docs/ops-playbook.md S1 (why a file, not a one-liner pkill).
# Usage: bash scripts/view_world.sh [uav_arena|uav_arena_indoor|uav_arena_outdoor|uav_arena_bk]
set -o pipefail

WORLD_NAME=${1:-uav_arena}
SHARE=$HOME/PX4_ROS2/install/uav_sim_gz/share/uav_sim_gz
WORLD=$SHARE/worlds/$WORLD_NAME.sdf
# bashrc forces CPU render; unset before setting the GPU hint.
unset LIBGL_ALWAYS_SOFTWARE
# Two GPUs present; without this D3D12 picks the iGPU.
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA

# Our models inherit PX4's x500_base mesh; both paths needed.
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}:$SHARE/models:$HOME/PX4-Autopilot/Tools/simulation/gz/models"

if [ ! -f "$WORLD" ]; then
  echo "world not found: $WORLD"
  echo "available:"
  ls "$(dirname "$WORLD")" | sed 's/\.sdf$//' | sed 's/^/  /'
  exit 1
fi

# Match the binary path, not a human-typed word.
pkill -f 'bin/gz' >/dev/null 2>&1
pkill -f 'gz-sim-server' >/dev/null 2>&1
pkill -f 'gz-sim-gui' >/dev/null 2>&1
sleep 2

echo "world   : $WORLD_NAME"
echo "models  : $(grep -c '<model name=' "$WORLD")"
echo "opening the GUI - press the play button at the bottom left to start physics"
exec gz sim "$WORLD"
