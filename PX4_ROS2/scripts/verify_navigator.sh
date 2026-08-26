#!/bin/bash
# G-N1: fly M5 through the navigator actions - takeoff, goto, cancel-hold, land.
# Usage: bash scripts/verify_navigator.sh
# Env: UAV_MODEL UAV_WORLD UAV_ID
# NOT RUN by the author - sim access is reserved for the verifier.
# What each check means: src/uav_navigation/README.md
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
# uav0 bridges only /clock, so there is no vision source and odometry_fused
# falls back to GPS (sigma_z = 0.5 m) - below this gate's 0.3 m acceptance.
export UAV_MODEL=${UAV_MODEL:-uav0_nav}
export UAV_WORLD=${UAV_WORLD:-uav_arena}
UAV_ID=${UAV_ID:-uav0}

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

PROBE=$WORKSPACE/src/uav_navigation/test/navigator_gate.py
CONFIG=$WORKSPACE/install/uav_navigation/share/uav_navigation/config/navigation_params.yaml
NODE=$WORKSPACE/install/uav_navigation/lib/uav_navigation/navigator_action_server_node

[ -f "$PROBE" ] || { echo "FATAL: $PROBE missing"; exit 1; }
[ -f "$CONFIG" ] || { echo "FATAL: $CONFIG missing, build uav_navigation"; exit 1; }
[ -x "$NODE" ] || { echo "FATAL: navigator_action_server_node not installed"; exit 1; }

echo "=== 1/5 simulator (model=$UAV_MODEL world=$UAV_WORLD) ==="
bash "$WORKSPACE/scripts/start_sim.sh" 2>&1 | tail -2

deadline=$((SECONDS + 420))
until timeout 20 ros2 topic list --no-daemon 2>/dev/null \
    | grep -q '/fmu/out/vehicle_odometry'; do
  if [ $SECONDS -gt $deadline ]; then echo "FATAL: PX4 never published"; exit 1; fi
  sleep 10
done
echo "PX4 publishing after ${SECONDS}s"

echo
echo "=== 2/5 backend and localization (sim.launch.py, perception off) ==="
pkill -f 'ros2 launch uav_bringup' 2>/dev/null
pkill -f 'navigator_action_server_node' 2>/dev/null
sleep 4
# navigator:=false since P7.4 made it default true: step 3 starts the navigator
# under test, and two of them would both write /control/cmd_mission.
setsid nohup ros2 launch uav_bringup sim.launch.py uav_id:="$UAV_ID" navigator:=false \
  > /tmp/gn1_launch.log 2>&1 < /dev/null &
sleep 25
grep -c 'process started' /tmp/gn1_launch.log

echo
echo "=== 3/5 navigator ==="
# use_sim_time matters here: the carrot step and every timeout follow /clock.
setsid nohup ros2 run uav_navigation navigator_action_server_node \
  --ros-args --params-file "$CONFIG" -p uav_id:="$UAV_ID" -p use_sim_time:=true \
  > /tmp/gn1_navigator.log 2>&1 < /dev/null &
sleep 8
tail -3 /tmp/gn1_navigator.log

echo
echo "=== 4/5 gate flight ==="
# RTF is only trustworthy while the aircraft is actually flying, so sample now.
bash "$WORKSPACE/scripts/sample_rtf.sh" "$UAV_WORLD" 60 > /tmp/gn1_rtf.txt 2>&1 &
rtf_pid=$!
python3 -u "$PROBE" --uav-id "$UAV_ID" 2>&1 | tail -25
verdict=${PIPESTATUS[0]}

echo
echo "=== 5/5 teardown ==="
wait "$rtf_pid" 2>/dev/null
cat /tmp/gn1_rtf.txt
pkill -f 'navigator_action_server_node' 2>/dev/null
pkill -f 'ros2 launch uav_bringup' 2>/dev/null
bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1

# Echo the evidence instead of pointing at /tmp: WSL wipes it on idle restart,
# and a gate whose reasons vanish costs another whole run.
echo
echo "--- navigator log (last 30) ---"
tail -30 /tmp/gn1_navigator.log 2>/dev/null | sed 's/^/  /'
echo "--- localization decisions ---"
grep -iE 'source ->|uncertainty|not trusted|absorbing|estimator' /tmp/gn1_launch.log 2>/dev/null \
  | tail -10 | sed 's/^/  /'

echo
echo "G-N1 exit status: $verdict"
exit "$verdict"
