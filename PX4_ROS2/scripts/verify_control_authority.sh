#!/bin/bash
# G-CA2: flight gate for uav_control_authority. TEST flies a Goto leg, a fake
# OPERATOR steals authority mid-leg and holds position, then releases it.
# Usage: bash scripts/verify_control_authority.sh
# Env: UAV_MODEL (default uav0_nav, same as G-N2) UAV_WORLD (default uav_arena) UAV_ID (default uav0)
# What each check means: .claude/plan/P7-control-authority.md S:6 G-CA2
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
export UAV_MODEL=${UAV_MODEL:-uav0_nav}
export UAV_WORLD=${UAV_WORLD:-uav_arena}
UAV_ID=${UAV_ID:-uav0}

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

PROBE=$WORKSPACE/src/uav_control_authority/test/authority_gate.py
[ -f "$PROBE" ] || { echo "FATAL: $PROBE missing"; exit 1; }

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
echo "=== 2/5 autonomy stack (sim.launch.py, all P7 defaults) ==="
setsid nohup ros2 launch uav_bringup sim.launch.py uav_id:="$UAV_ID" \
  > /tmp/gca2_launch.log 2>&1 < /dev/null &
sleep 30
echo "process started count: $(grep -c 'process started' /tmp/gca2_launch.log)"
echo "nodes up: $(timeout 25 ros2 node list --no-daemon 2>/dev/null | wc -l)"

echo
echo "=== 3/5 independent witness: /fmu/in/offboard_control_mode rate (no px4_msgs in the probe) ==="
timeout 180 ros2 topic hz /fmu/in/offboard_control_mode --window 400 \
  > /tmp/gca2_fmu_hz.log 2>&1 &
FMU_HZ_PID=$!

echo
echo "=== 4/5 gate flight ==="
python3 -u "$PROBE" --uav-id "$UAV_ID" 2>&1 | tail -80
verdict=${PIPESTATUS[0]}

kill "$FMU_HZ_PID" 2>/dev/null
wait "$FMU_HZ_PID" 2>/dev/null

echo
echo "real-time factor during the run:"
bash "$WORKSPACE/scripts/sample_rtf.sh" "$UAV_WORLD" 40 | sed 's/^/  /'

echo
echo "independent /fmu/in/offboard_control_mode rate (last 5 samples):"
grep 'average rate' /tmp/gca2_fmu_hz.log 2>/dev/null | tail -5 | sed 's/^/  /'

echo
echo "=== 5/5 teardown ==="
pkill -f 'ros2 launch uav_bringup' 2>/dev/null
sleep 3
bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1

echo
echo "G-CA2 exit status: $verdict"
exit "$verdict"
