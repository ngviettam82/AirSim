#!/bin/bash
# G-N4b: uav0_track flies a Goto leg, the runner spawns a real box mid-flight
# via gz service, and the navigator must react without colliding or descending.
# Usage: bash scripts/run_gn4b.sh
# Env: UAV_ID UAV_WORLD
# NOT RUN by the author - sim access is reserved for the verifier-runner.
# What each check means: src/uav_bringup/test/gn4b_avoidance_gate.py
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
export UAV_MODEL=uav0_track
export UAV_WORLD=${UAV_WORLD:-uav_arena}
UAV_ID=${UAV_ID:-uav0}
LOGDIR=$HOME/gate_logs
mkdir -p "$LOGDIR"

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
source "$WORKSPACE/scripts/gate_common.sh"

PROBE=$WORKSPACE/src/uav_bringup/test/gn4b_avoidance_gate.py
CONFIG=$WORKSPACE/install/uav_navigation/share/uav_navigation/config/navigation_params.yaml
NODE=$WORKSPACE/install/uav_navigation/lib/uav_navigation/navigator_action_server_node
REQUEST_FILE=$LOGDIR/gn4b_obstacle_request.txt
SPAWNED_FLAG=$LOGDIR/gn4b_obstacle_spawned.flag
GZ_REQ_FILE=$LOGDIR/gn4b_gz_request.pbtxt

[ -f "$PROBE" ] || { echo "FATAL: $PROBE missing"; exit 1; }
[ -f "$CONFIG" ] || { echo "FATAL: $CONFIG missing, build uav_navigation"; exit 1; }
[ -x "$NODE" ] || { echo "FATAL: navigator_action_server_node not installed"; exit 1; }

# Stale files from a previous run must not be mistaken for this run's evidence.
rm -f "$REQUEST_FILE" "$SPAWNED_FLAG" "$GZ_REQ_FILE"

echo "=== 1/7 simulator (model=$UAV_MODEL world=$UAV_WORLD) ==="
bash "$WORKSPACE/scripts/start_sim.sh" 2>&1 | tail -3

deadline=$((SECONDS + 420))
until timeout 20 ros2 topic list --no-daemon 2>/dev/null \
    | grep -q '/fmu/out/vehicle_odometry'; do
  if [ $SECONDS -gt $deadline ]; then echo "FATAL: PX4 never published"; exit 1; fi
  sleep 10
done
echo "PX4 publishing after ${SECONDS}s"

echo
echo "=== 2/7 backend + localization + navigation advisors (sim.launch.py) ==="
# ops-playbook S3: killing the launch parent alone has left orphaned children
# behind before (the 12-mux and 22-process incidents) - kill each lib pattern.
pkill -f 'ros2 launch uav_bringup' 2>/dev/null
pkill -f 'install/uav_px4_backend/lib' 2>/dev/null
pkill -f 'install/uav_localization/lib' 2>/dev/null
pkill -f 'install/uav_navigation/lib' 2>/dev/null
pkill -f 'navigator_action_server_node' 2>/dev/null
sleep 4
# navigator:=false since P7.4 made it default true: step 4 starts the navigator
# under test, and two of them would both write /control/cmd_mission.
setsid nohup ros2 launch uav_bringup sim.launch.py uav_id:="$UAV_ID" navigator:=false \
  > "$LOGDIR/gn4b_launch.log" 2>&1 < /dev/null &
sleep 20

echo
echo "=== 3/7 waiting for the bringup nodes BY NAME, not by a process count ==="
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
echo "=== 4/7 navigator (manual - not part of bringup, uav_navigation README S7) ==="
# Nothing to kill first: navigator:=false above means bringup never started
# one, so this is a plain start, not a swap.
setsid nohup ros2 run uav_navigation navigator_action_server_node \
  --ros-args --params-file "$CONFIG" -p uav_id:="$UAV_ID" -p use_sim_time:=true \
  -p use_trajectory:=true > "$LOGDIR/gn4b_navigator.log" 2>&1 < /dev/null &
sleep 8
if ! grep -q 'goto path' "$LOGDIR/gn4b_navigator.log"; then
  echo "FATAL: navigator did not report ready"
  tail -15 "$LOGDIR/gn4b_navigator.log"
  exit 1
fi
echo "navigator ready"

echo
echo "=== 5/7 probe + obstacle spawn handshake ==="
python3 -u "$PROBE" --uav-id "$UAV_ID" \
  --request-file "$REQUEST_FILE" --spawned-flag "$SPAWNED_FLAG" \
  > "$LOGDIR/gn4b_probe.log" 2>&1 < /dev/null &
PROBE_PID=$!

# Poll for the probe's request, event-based (R21): no fixed sleep-and-hope.
deadline=$((SECONDS + 150))
until [ -s "$REQUEST_FILE" ]; do
  if [ $SECONDS -gt $deadline ] || ! kill -0 "$PROBE_PID" 2>/dev/null; then
    echo "FATAL: probe never wrote an obstacle request (trajectory not established, or it died)"
    kill "$PROBE_PID" 2>/dev/null
    tail -40 "$LOGDIR/gn4b_probe.log"
    exit 1
  fi
  sleep 1
done
read -r OBX OBY OBZ OSX OSY OSZ < "$REQUEST_FILE"
echo "obstacle requested at ($OBX $OBY $OBZ) size ($OSX $OSY $OSZ)"

# Written to a file, not inlined: avoids the quote-nesting trap (ops-playbook
# S8c/S9) when this whole script is itself invoked through another shell layer.
# The visual and collision geometry are identical on purpose (CLAUDE.md S3:
# a collision box must never be smaller than its visual).
cat > "$GZ_REQ_FILE" <<EOF
sdf: '<sdf version="1.9"><model name="gn4b_obstacle_box"><static>true</static><link name="link"><collision name="collision"><geometry><box><size>$OSX $OSY $OSZ</size></box></geometry></collision><visual name="visual"><geometry><box><size>$OSX $OSY $OSZ</size></box></geometry></visual></link></model></sdf>'
name: 'gn4b_obstacle_box'
pose: {position: {x: $OBX, y: $OBY, z: $OBZ}}
EOF

SPAWN_REPLY=$(gz service -s "/world/$UAV_WORLD/create" \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
  --req "$(cat "$GZ_REQ_FILE")" 2>&1)
echo "gz service reply: $SPAWN_REPLY"
if echo "$SPAWN_REPLY" | grep -q 'data: true'; then
  : > "$SPAWNED_FLAG"
  echo "obstacle spawn confirmed - the box is real in Gazebo"
else
  echo "WARNING: gz service did not confirm the spawn; the probe will time out"
  echo "waiting for it and report FAILED TO MEASURE on its own"
fi

echo
echo "=== 6/7 waiting for the probe to land and finish ==="
wait "$PROBE_PID"
verdict=$?

echo
echo "=== 7/7 teardown ==="
pkill -f 'navigator_action_server_node' 2>/dev/null
pkill -f 'install/uav_navigation/lib' 2>/dev/null
pkill -f 'ros2 launch uav_bringup' 2>/dev/null
sleep 2
bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1

echo
echo "--- probe log (last 80) ---"
tail -80 "$LOGDIR/gn4b_probe.log" | sed 's/^/  /'
echo "--- navigator log (last 20) ---"
tail -20 "$LOGDIR/gn4b_navigator.log" 2>/dev/null | sed 's/^/  /'

echo
echo "G-N4b exit status: $verdict   (0=PASS 1=FAIL 2=FAILED TO MEASURE)"
exit "$verdict"
