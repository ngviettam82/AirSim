#!/bin/bash
# BK world vs arena flight accuracy (RTF isolated via uav0).
# See docs/ops-playbook.md (S2, S6).
set -o pipefail

W=$HOME/PX4_ROS2
source /opt/ros/humble/setup.bash
export PX4_GZ_MODEL_POSE="-9.7,-94.9,0.25,0,0,0"

run_case() {
  local world=$1 pose=$2
  echo
  echo "############ world=$world model=uav0 ############"
  PX4_GZ_MODEL_POSE="$pose" UAV_WORLD="$world" UAV_MODEL=uav0 \
    bash "$W/scripts/start_sim.sh" >/tmp/cmp_start.log 2>&1

  local deadline=$((SECONDS + 420))
  until timeout 20 ros2 topic list --no-daemon 2>/dev/null | grep -q '/fmu/out/vehicle_odometry'; do
    if [ $SECONDS -gt $deadline ]; then echo "  PX4 never came up"; return 1; fi
    sleep 10
  done
  source "$W/install/setup.bash"
  setsid nohup ros2 launch uav_bringup sim.launch.py >/tmp/cmp_launch.log 2>&1 </dev/null &
  sleep 30

  ( timeout 120 gz topic -e -t "/world/$world/stats" 2>/dev/null \
      | grep --line-buffered real_time_factor > /tmp/cmp_rtf.txt ) &

  python3 "$W/install/uav_bringup/lib/uav_bringup/smoke_flight.py" --flights 3 2>&1 \
    | grep -E 'flight [0-9]|completed|violations|RESULT|FAILURE'

  echo -n "  RTF during the flight: "
  awk '{s+=$2; n++} END {if (n) printf "mean %.3f over %d samples\n", s/n, n; else print "no samples"}' /tmp/cmp_rtf.txt
  pkill -f 'ros2 launch uav_bringup' 2>/dev/null
  sleep 3
}

run_case uav_arena_bk "-9.7,-94.9,0.25,0,0,0"
run_case uav_arena "0,0,0.25,0,0,0"
echo
echo "############ done ############"
