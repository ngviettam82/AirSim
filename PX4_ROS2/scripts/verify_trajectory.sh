#!/bin/bash
# G-N2: three Goto legs on a trajectory, then the same leg on the carrot as control.
# Usage: bash scripts/verify_trajectory.sh
# Env: UAV_MODEL UAV_WORLD UAV_ID
# NOT RUN by the author - sim access is reserved for the verifier.
# What each check means: src/uav_navigation/README.md
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
# uav0 bridges only /clock, so odometry_fused falls back to GPS (sigma_z 0.5 m),
# which is above this gate's 0.3 m acceptance.
export UAV_MODEL=${UAV_MODEL:-uav0_nav}
export UAV_WORLD=${UAV_WORLD:-uav_arena}
UAV_ID=${UAV_ID:-uav0}

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

PROBE=$WORKSPACE/src/uav_navigation/test/trajectory_gate.py
CONFIG=$WORKSPACE/install/uav_navigation/share/uav_navigation/config/navigation_params.yaml
NODE=$WORKSPACE/install/uav_navigation/lib/uav_navigation/navigator_action_server_node

[ -f "$PROBE" ] || { echo "FATAL: $PROBE missing"; exit 1; }
[ -f "$CONFIG" ] || { echo "FATAL: $CONFIG missing, build uav_navigation"; exit 1; }
[ -x "$NODE" ] || { echo "FATAL: navigator_action_server_node not installed"; exit 1; }

start_navigator() {
  # This pkill is for the INTRA-script swap (called again below to switch
  # trajectory -> carrot on the same navigator process) -- it is no longer
  # covering for a bringup-started navigator, since step 2 now launches with
  # navigator:=false. On the first call there is nothing to kill; harmless.
  pkill -f 'navigator_action_server_node' 2>/dev/null
  sleep 3
  setsid nohup ros2 run uav_navigation navigator_action_server_node \
    --ros-args --params-file "$CONFIG" -p uav_id:="$UAV_ID" -p use_sim_time:=true \
    -p use_trajectory:="$1" > "$2" 2>&1 < /dev/null &
  sleep 8
  # A navigator that never came up would otherwise surface 30 s later as
  # "action server never appeared", which reads like a probe bug.
  if ! grep -q "goto path $3" "$2"; then
    echo "FATAL: navigator did not start on the $3 path"
    tail -5 "$2"
    exit 1
  fi
  grep 'goto path' "$2" | tail -1
}

echo "=== 1/6 simulator (model=$UAV_MODEL world=$UAV_WORLD) ==="
bash "$WORKSPACE/scripts/start_sim.sh" 2>&1 | tail -2

deadline=$((SECONDS + 420))
until timeout 20 ros2 topic list --no-daemon 2>/dev/null \
    | grep -q '/fmu/out/vehicle_odometry'; do
  if [ $SECONDS -gt $deadline ]; then echo "FATAL: PX4 never published"; exit 1; fi
  sleep 10
done
echo "PX4 publishing after ${SECONDS}s"

echo
echo "=== 2/6 backend and localization (sim.launch.py, perception off) ==="
pkill -f 'ros2 launch uav_bringup' 2>/dev/null
pkill -f 'navigator_action_server_node' 2>/dev/null
sleep 4
# navigator:=false since P7.4 made it default true: start_navigator() below
# starts the navigator under test, and two of them would both write
# /control/cmd_mission -- the old reliance on start_navigator()'s own pkill to
# also clean up a bringup-spawned navigator left a real ~3s SIGTERM race.
setsid nohup ros2 launch uav_bringup sim.launch.py uav_id:="$UAV_ID" navigator:=false \
  > /tmp/gn2_launch.log 2>&1 < /dev/null &
sleep 25
grep -c 'process started' /tmp/gn2_launch.log

echo
echo "=== 3/6 navigator on the trajectory path ==="
start_navigator true /tmp/gn2_navigator.log trajectory

echo
echo "=== 4/6 gate flight ==="
python3 -u "$PROBE" --uav-id "$UAV_ID" 2>&1 | tail -40
verdict=${PIPESTATUS[0]}

echo
echo "=== 5/6 positive control: the same leg on the carrot ==="
start_navigator false /tmp/gn2_control.log carrot
python3 -u "$PROBE" --uav-id "$UAV_ID" --mode control 2>&1 | tail -12
control=${PIPESTATUS[0]}

echo
echo "=== 6/6 teardown ==="
pkill -f 'navigator_action_server_node' 2>/dev/null
pkill -f 'ros2 launch uav_bringup' 2>/dev/null
bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1

# Echo the evidence: WSL wipes /tmp on an idle restart, and a gate whose reasons
# vanish costs another whole run.
echo
echo "--- navigator log, trajectory run (last 30) ---"
tail -30 /tmp/gn2_navigator.log 2>/dev/null | sed 's/^/  /'
echo "--- navigator log, control run (last 10) ---"
tail -10 /tmp/gn2_control.log 2>/dev/null | sed 's/^/  /'

echo
echo "G-N2 exit status: $verdict (positive control exit status: $control)"
# A gate whose positive control did not fire has proven nothing, so it fails too.
if [ "$control" != "0" ]; then
  echo "FAIL: the positive control did not step the stream, so check c2 is unproven"
  exit 1
fi
exit "$verdict"
