#!/bin/bash
# Discriminating measurement for the G-N1 vertical failures: record what the
# navigator commanded next to what the aircraft did, during one takeoff.
# A stalled command trace blames the carrot; a followed-but-ignored trace blames
# the setpoint contract with the backend.
# Usage: bash scripts/trace_navigator_setpoints.sh
# Env: UAV_MODEL UAV_WORLD ALTITUDE TRACE_SECONDS
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
# uav0 has no vision source; odometry_fused would be GPS-grade (sigma_z 0.5 m).
export UAV_MODEL=${UAV_MODEL:-uav0_nav}
export UAV_WORLD=${UAV_WORLD:-uav_arena}
UAV_ID=${UAV_ID:-uav0}
ALTITUDE=${ALTITUDE:-2.5}
TRACE_SECONDS=${TRACE_SECONDS:-45}

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

PROBE=$WORKSPACE/src/uav_navigation/test/setpoint_trace.py
CONFIG=$WORKSPACE/install/uav_navigation/share/uav_navigation/config/navigation_params.yaml
[ -f "$PROBE" ] || { echo "FATAL: $PROBE missing, sync uav_navigation"; exit 1; }

echo "=== 1/4 simulator (model=$UAV_MODEL) ==="
bash "$WORKSPACE/scripts/start_sim.sh" 2>&1 | tail -2

deadline=$((SECONDS + 420))
until timeout 20 ros2 topic list --no-daemon 2>/dev/null \
    | grep -q '/fmu/out/vehicle_odometry'; do
  if [ $SECONDS -gt $deadline ]; then echo "FATAL: PX4 never published"; exit 1; fi
  sleep 10
done
echo "PX4 publishing after ${SECONDS}s"

echo
echo "=== 2/4 backend and localization ==="
# Killing the launch wrapper leaves its node processes orphaned, and a second
# localization stack publishing the same topic looks exactly like a noisy sensor.
for node in navigator_action_server_node localization_mux_node vio_adapter_node \
    gps_adapter_node rangefinder_adapter_node optical_flow_adapter_node \
    localization_health_node px4_state_adapter_node px4_frame_bridge_node \
    px4_command_gateway_node offboard_session_manager_node px4_external_odometry_node; do
  pkill -f "$node" 2>/dev/null
done
pkill -f 'ros2 launch uav_bringup' 2>/dev/null
sleep 5
leftover=$(pgrep -cf 'localization_mux_node|px4_state_adapter_node' || true)
echo "  stale localization nodes after cleanup: ${leftover:-0}"
# navigator:=false since P7.4 made it default true: step 3 starts the navigator
# under test, and two of them would both write /control/cmd_mission. The
# per-node pkill above runs BEFORE this launch, so it cannot catch the one
# bringup itself would start -- the flag is the only fix that actually works.
setsid nohup ros2 launch uav_bringup sim.launch.py uav_id:="$UAV_ID" navigator:=false \
  > /tmp/trace_launch.log 2>&1 < /dev/null &
sleep 25

# Two publishers on one odometry topic read as a noisy sensor, not as a fault.
echo "  publishers on odometry_fused:"
timeout 20 ros2 topic info "/uav/$UAV_ID/state/odometry_fused" 2>&1 | sed 's/^/    /'
echo "  publishers on odometry_raw:"
timeout 20 ros2 topic info "/uav/$UAV_ID/state/odometry_raw" 2>&1 | sed 's/^/    /'

echo
echo "=== 3/4 navigator ==="
setsid nohup ros2 run uav_navigation navigator_action_server_node \
  --ros-args --params-file "$CONFIG" -p uav_id:="$UAV_ID" -p use_sim_time:=true \
  > /tmp/trace_navigator.log 2>&1 < /dev/null &
sleep 8
tail -2 /tmp/trace_navigator.log

echo
echo "=== 4/4 trace one takeoff ==="
python3 -u "$PROBE" --uav-id "$UAV_ID" --altitude "$ALTITUDE" --seconds "$TRACE_SECONDS" 2>&1 | tail -45
status=${PIPESTATUS[0]}

echo
echo "--- navigator log ---"
tail -12 /tmp/trace_navigator.log

pkill -f 'navigator_action_server_node' 2>/dev/null
pkill -f 'ros2 launch uav_bringup' 2>/dev/null
bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1

echo
echo "trace exit status: $status"
exit "$status"
