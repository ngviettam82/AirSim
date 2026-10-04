#!/bin/bash
# Bring up sim stack: Agent + Gazebo + PX4 SITL.
# See docs/ops-playbook.md S1 for the standalone-Gazebo rationale.
# Usage: ./start_sim.sh [gui]
# Env:   UAV_MODEL (default uav0_full)   UAV_WORLD (default uav_arena)

set -e

AGENT="$HOME/.local/bin/MicroXRCEAgent"
WORKSPACE="$HOME/PX4_ROS2"
PX4_DIR="$HOME/PX4-Autopilot"
UAV_MODEL="${UAV_MODEL:-uav0}"

# uav_arena default: untextured baseline isolates code from world faults.
if [ -z "$UAV_WORLD" ]; then
  case "$UAV_MODEL" in
    *_indoor) UAV_WORLD=uav_arena_indoor ;;
    *) UAV_WORLD=uav_arena ;;
  esac
fi

export LD_LIBRARY_PATH="$HOME/.local/lib:$LD_LIBRARY_PATH"

# Required (see ops-playbook S3): iGPU segfaults; software GL tanks RTF.
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
unset LIBGL_ALWAYS_SOFTWARE

# Debt #10: a 640x480 camera frame does not fit the stock Fast DDS shared-memory
# segment (549 408 B), falls back to fragmented UDP, and loses whole frames -- depth
# ran at 4.8 of 15.6 Hz with a 1256 ms worst gap while feeding obstacle avoidance.
# This profile raises the segment to 16 MiB and degrades no sensor.
# Cleared before wiring, 2026-08-24: gate R0 (scripts/gate_r0_dds_profile.sh) measured
# the PX4 link under this exact arrangement -- the profile is exported BEFORE the
# agent starts, so the agent inherits it, which is where fastdds_shm_only.xml died.
# Sim-normalised profile/control: odometry 0.958, status 0.962, sensor_combined 0.959,
# worst gap better in all three. Then verify_obstacle_extractor + verify_marker_detector
# + M5 3/3 under the profile (R14).
# UAV_DDS_PROFILE=none opts out, so gate R0's control arm can still run stock
# transport. A gate that cannot be re-run is a gate that rots.
DDS_PROFILE_FILE="$WORKSPACE/install/uav_bringup/share/uav_bringup/config/fastdds_large_samples.xml"
if [ "${UAV_DDS_PROFILE:-}" = "none" ]; then
  unset FASTRTPS_DEFAULT_PROFILES_FILE
  echo "DDS profile disabled by UAV_DDS_PROFILE=none (stock transport)"
elif [ -f "$DDS_PROFILE_FILE" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_PROFILE_FILE"
else
  # Loud, not silent: without it the cameras quietly starve again, and a quiet
  # perception starve is what cost this project debt #10 for eleven days.
  echo "WARNING: large-samples DDS profile not installed; cameras will starve." >&2
  echo "         build uav_bringup to restore it: $DDS_PROFILE_FILE" >&2
fi

# Snap build (v1.0.2) is too old for PX4 v1.15.
if [ ! -x "$AGENT" ]; then
  echo "ERROR: agent not found at $AGENT" >&2
  exit 1
fi

pkill -f 'MicroXRCEAgent' 2>/dev/null || true
pkill -f 'px4_sitl_default/bin/px4' 2>/dev/null || true
pkill -f 'gz sim' 2>/dev/null || true
pkill -f 'parameter_bridge' 2>/dev/null || true
pkill -f 'image_bridge' 2>/dev/null || true

# Clears sim wrappers only; build stays synchronous, done below.
pkill -f 'make px4_sitl' 2>/dev/null || true
pkill -f 'sleep infinity' 2>/dev/null || true
sleep 2

# Leftover nodes serve stale binaries and win service calls.
pkill -f 'uav_px4_backend/' 2>/dev/null || true
pkill -f 'ros2 launch uav_bringup' 2>/dev/null || true
pkill -f 'ros2 run uav_px4_backend' 2>/dev/null || true
sleep 3

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
# Adds PX4's x500/OakD-Lite models; workspace hook adds ours.
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$PX4_DIR/Tools/simulation/gz/models"

WORLD_FILE="$WORKSPACE/install/uav_sim_gz/share/uav_sim_gz/worlds/$UAV_WORLD.sdf"
if [ ! -f "$WORLD_FILE" ]; then
  echo "ERROR: world not found: $WORLD_FILE (build uav_sim_gz first)" >&2
  exit 1
fi

setsid nohup "$AGENT" udp4 -p 8888 > /tmp/agent.log 2>&1 < /dev/null &
sleep 2
echo "agent started (log: /tmp/agent.log)"

if [ "$1" = "gui" ]; then
  setsid nohup gz sim -r "$WORLD_FILE" > /tmp/gz.log 2>&1 < /dev/null &
else
  setsid nohup gz sim -r -s "$WORLD_FILE" > /tmp/gz.log 2>&1 < /dev/null &
fi
echo "gazebo starting, world=$UAV_WORLD (log: /tmp/gz.log)"

for _ in $(seq 1 60); do
  if gz topic -l 2>/dev/null | grep -q "^/world/$UAV_WORLD/clock$"; then
    echo "gazebo ready"
    break
  fi
  sleep 1
done

if ! gz topic -l 2>/dev/null | grep -q "^/world/$UAV_WORLD/clock$"; then
  echo "ERROR: gazebo did not publish /world/$UAV_WORLD/clock within 60s" >&2
  exit 1
fi

cd "$PX4_DIR"

# Build first: combined build+run hides a stale-build compile.
echo "checking the PX4 build"
if ! make px4_sitl > /tmp/px4_build.log 2>&1; then
  echo "ERROR: PX4 build failed. Last lines of /tmp/px4_build.log:" >&2
  tail -15 /tmp/px4_build.log >&2
  exit 1
fi

export PX4_GZ_STANDALONE=1
export PX4_GZ_WORLD="$UAV_WORLD"
# Blocking stdin needed: /dev/null makes PX4 shell EOF-loop, log explodes.
setsid nohup bash -c "sleep infinity | make px4_sitl gz_$UAV_MODEL" > /tmp/px4.log 2>&1 &

# Bridge = sim's sensor-driver layer; only source of /clock.
setsid nohup ros2 launch uav_sim_gz gz_bridge.launch.py "model:=$UAV_MODEL" \
  "images:=${UAV_BRIDGE_IMAGES:-true}" ${UAV_BRIDGE_CONFIG:+"config:=$UAV_BRIDGE_CONFIG"} \
  > /tmp/bridge.log 2>&1 < /dev/null &

echo "PX4 starting, model=$UAV_MODEL (log: /tmp/px4.log), allow ~30s"
echo "bridge started (log: /tmp/bridge.log)"
echo "verify: ros2 topic hz /fmu/out/vehicle_odometry"
echo "verify: ros2 topic hz /clock"
