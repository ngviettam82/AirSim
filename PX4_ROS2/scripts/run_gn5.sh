#!/bin/bash
# G-N5: FollowPath (4 waypoints) then a TrackTarget slice (orbit / recede / lost).
# Usage: bash scripts/run_gn5.sh
# Env: UAV_ID UAV_WORLD
# NOT RUN by the author - sim access is reserved for the verifier-runner.
# What each check means: src/uav_bringup/test/gn5_followtrack_gate.py
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
export UAV_MODEL=uav0_nav
export UAV_WORLD=${UAV_WORLD:-uav_arena}
UAV_ID=${UAV_ID:-uav0}
LOGDIR=$HOME/gate_logs
mkdir -p "$LOGDIR"

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

PROBE=$WORKSPACE/src/uav_bringup/test/gn5_followtrack_gate.py
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
  > "$LOGDIR/gn5_launch.log" 2>&1 < /dev/null &
sleep 20

echo
echo "=== 3/5 waiting for the 13 bringup processes BY NAME, not by sleeping ==="
deadline=$((SECONDS + 90))
until [ "$(pgrep -c -f 'uav_px4_backend/|uav_localization/|uav_navigation/' 2>/dev/null)" = "13" ]; do
  if [ $SECONDS -gt $deadline ]; then
    echo "FATAL: expected 13 backend+localization+navigation processes, got:"
    pgrep -af 'uav_px4_backend/|uav_localization/|uav_navigation/'
    exit 1
  fi
  sleep 3
done
# Discovery lags the process table: a fresh graph observer needs a few seconds.
deadline=$((SECONDS + 60))
for node in route_planner_node local_planner_node; do
  until timeout 15 ros2 node list --no-daemon 2>/dev/null | grep -q "$node"; do
    if [ $SECONDS -gt $deadline ]; then
      echo "FATAL: $node never appeared in ros2 node list"
      timeout 15 ros2 node list --no-daemon 2>/dev/null
      exit 1
    fi
    sleep 3
  done
done
echo "13 processes up; route_planner_node + local_planner_node confirmed by name"

echo
echo "=== 4/5 navigator (manual - not part of bringup, uav_navigation README S7) ==="
pkill -f 'navigator_action_server_node' 2>/dev/null
sleep 2
setsid nohup ros2 run uav_navigation navigator_action_server_node \
  --ros-args --params-file "$CONFIG" -p uav_id:="$UAV_ID" -p use_sim_time:=true \
  -p use_trajectory:=true > "$LOGDIR/gn5_navigator.log" 2>&1 < /dev/null &
sleep 8
if ! grep -q 'goto path' "$LOGDIR/gn5_navigator.log"; then
  echo "FATAL: navigator did not report ready"
  tail -15 "$LOGDIR/gn5_navigator.log"
  exit 1
fi
echo "navigator ready"

echo
echo "=== 5/5 probe: FollowPath then TrackTarget (allow several minutes) ==="
python3 -u "$PROBE" --uav-id "$UAV_ID" 2>&1 | tee "$LOGDIR/gn5_probe.log" | tail -90
verdict=${PIPESTATUS[0]}

echo
echo "=== teardown ==="
pkill -f 'navigator_action_server_node' 2>/dev/null
pkill -f 'install/uav_navigation/lib' 2>/dev/null
pkill -f 'ros2 launch uav_bringup' 2>/dev/null
sleep 2
bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1

echo
echo "--- navigator log (last 20) ---"
tail -20 "$LOGDIR/gn5_navigator.log" 2>/dev/null | sed 's/^/  /'

echo
echo "G-N5 exit status: $verdict   (0=PASS 1=FAIL 2=FAILED TO MEASURE)"
exit "$verdict"
