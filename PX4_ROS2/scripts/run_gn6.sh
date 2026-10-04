#!/bin/bash
# G-N6: inject Recover mid-Goto three times (HOLD / CLIMB / RETURN_HOME) plus a
# TYPE_LAND rejection check. Only run this AFTER the Recover server (B4b) lands.
# Usage: bash scripts/run_gn6.sh
# Env: UAV_ID UAV_WORLD
# NOT RUN by the author - sim access is reserved for the verifier-runner.
# What each check means: src/uav_bringup/test/gn6_recover_gate.py
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
export UAV_MODEL=uav0_nav
export UAV_WORLD=${UAV_WORLD:-uav_arena}
UAV_ID=${UAV_ID:-uav0}
LOGDIR=$HOME/gate_logs
mkdir -p "$LOGDIR"

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
source "$WORKSPACE/scripts/gate_common.sh"

PROBE=$WORKSPACE/src/uav_bringup/test/gn6_recover_gate.py
CONFIG=$WORKSPACE/install/uav_navigation/share/uav_navigation/config/navigation_params.yaml
NODE=$WORKSPACE/install/uav_navigation/lib/uav_navigation/navigator_action_server_node

[ -f "$PROBE" ] || { echo "FATAL: $PROBE missing"; exit 1; }
[ -f "$CONFIG" ] || { echo "FATAL: $CONFIG missing, build uav_navigation"; exit 1; }
[ -x "$NODE" ] || { echo "FATAL: navigator_action_server_node not installed"; exit 1; }

echo "=== 1/5 simulator (model=$UAV_MODEL world=$UAV_WORLD) ==="
bash "$WORKSPACE/scripts/start_sim.sh" 2>&1 | tail -3

deadline=$((SECONDS + 420))
until timeout 20 ros2 topic list --no-daemon 2>/dev/null \
    | grep -q '/fmu/out/vehicle_odometry'; do
  if [ $SECONDS -gt $deadline ]; then echo "FATAL: PX4 never published"; exit 1; fi
  sleep 10
done
echo "PX4 publishing after ${SECONDS}s"

echo
echo "=== 2/5 backend + localization + navigation advisors (sim.launch.py) ==="
pkill -f 'ros2 launch uav_bringup' 2>/dev/null
pkill -f 'install/uav_px4_backend/lib' 2>/dev/null
pkill -f 'install/uav_localization/lib' 2>/dev/null
pkill -f 'install/uav_navigation/lib' 2>/dev/null
pkill -f 'navigator_action_server_node' 2>/dev/null
sleep 4
# navigator:=false since P7.4 made it default true: step 4 starts the navigator
# under test, and two of them would both write /control/cmd_mission.
setsid nohup ros2 launch uav_bringup sim.launch.py uav_id:="$UAV_ID" navigator:=false \
  > "$LOGDIR/gn6_launch.log" 2>&1 < /dev/null &
sleep 20

echo
echo "=== 3/5 waiting for the bringup nodes BY NAME, not by a process count ==="
# R34: a hardcoded process count ("13") silently goes stale the moment
# sim.launch.py's node roster changes (it already did once, at P7.4). Name
# checks instead -- list mirrors sim.launch.py's BACKEND_NODES +
# LOCALIZATION_NODES + NAVIGATION_NODES; update it if that file's roster
# changes. navigator_action_server_node is deliberately NOT in this list: it
# stays off (navigator:=false above) until step 4 starts it manually.
wait_nodes px4_state_adapter_node px4_frame_bridge_node px4_command_gateway_node \
  offboard_session_manager_node px4_external_odometry_node \
  rangefinder_adapter_node gps_adapter_node vio_adapter_node optical_flow_adapter_node \
  localization_mux_node localization_health_node \
  route_planner_node local_planner_node || exit 1

echo
echo "=== 4/5 navigator (manual - not part of bringup, uav_navigation README S7) ==="
# Nothing to kill first: navigator:=false above means bringup never started
# one, so this is a plain start, not a swap.
setsid nohup ros2 run uav_navigation navigator_action_server_node \
  --ros-args --params-file "$CONFIG" -p uav_id:="$UAV_ID" -p use_sim_time:=true \
  -p use_trajectory:=true > "$LOGDIR/gn6_navigator.log" 2>&1 < /dev/null &
sleep 8
if ! grep -q 'goto path' "$LOGDIR/gn6_navigator.log"; then
  echo "FATAL: navigator did not report ready"
  tail -15 "$LOGDIR/gn6_navigator.log"
  exit 1
fi
# The Recover action server is new (B4b); confirm it actually registered before
# spending a whole flight on a probe that would just time out waiting for it.
# Retried: a fresh graph observer discovers 7 action servers slower than names.
# No --no-daemon here: ros2 ACTION list does not take that flag (topic/node do),
# and with stderr swallowed the argparse error read as "not advertised" forever.
deadline=$((SECONDS + 60))
until timeout 15 ros2 action list 2>/dev/null | grep -q "/uav/$UAV_ID/planning/recover"; do
  if [ $SECONDS -gt $deadline ]; then
    echo "FATAL: /uav/$UAV_ID/planning/recover is not advertised - is B4b built and installed?"
    timeout 15 ros2 action list 2>&1
    exit 1
  fi
  sleep 3
done
echo "navigator ready, Recover action server confirmed by name"

echo
echo "=== 5/5 probe: TYPE_LAND check + 3 recover trials (allow several minutes) ==="
python3 -u "$PROBE" --uav-id "$UAV_ID" 2>&1 | tee "$LOGDIR/gn6_probe.log" | tail -120
verdict=${PIPESTATUS[0]}

echo
echo "=== teardown ==="
pkill -f 'navigator_action_server_node' 2>/dev/null
pkill -f 'install/uav_navigation/lib' 2>/dev/null
pkill -f 'ros2 launch uav_bringup' 2>/dev/null
sleep 2
bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1

echo
echo "--- navigator log (last 30) ---"
tail -30 "$LOGDIR/gn6_navigator.log" 2>/dev/null | sed 's/^/  /'

echo
echo "G-N6 exit status: $verdict   (0=PASS 1=FAIL 2=FAILED TO MEASURE)"
exit "$verdict"
