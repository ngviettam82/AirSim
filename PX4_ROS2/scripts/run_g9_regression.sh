#!/bin/bash
# G9: M5 regression flight (arm-takeoff-goto-land) in the BK world.
# See docs/ops-playbook.md S6 for the spawn-pose trap.
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
SRC=/mnt/c/code/PX4_ROS2/src/uav_sim_gz
export UAV_WORLD=uav_arena_bk
export UAV_MODEL=${UAV_MODEL:-uav0_nav}
export PX4_GZ_MODEL_POSE="${PX4_GZ_MODEL_POSE:--9.7,-94.9,0.25,0,0,0}"

echo "=== 1/5 sync uav_sim_gz and rebuild ==="
rm -rf "$WORKSPACE/src/uav_sim_gz"
cp -r "$SRC" "$WORKSPACE/src/"
cd "$WORKSPACE" || exit 1
source /opt/ros/humble/setup.bash
colcon build --packages-select uav_sim_gz 2>&1 | tail -5
ls -la "$WORKSPACE/install/uav_sim_gz/share/uav_sim_gz/worlds/" | grep -E "bk|arena" || true

echo
echo "=== 2/5 start simulator (world=$UAV_WORLD model=$UAV_MODEL) ==="
echo "spawn pose: $PX4_GZ_MODEL_POSE"
bash "$WORKSPACE/scripts/start_sim.sh" 2>&1 | tail -20
sim_status=$?
echo "start_sim exit: $sim_status"

echo
echo "=== 3/5 wait for PX4 to actually publish ==="
source "$WORKSPACE/install/setup.bash"
# Wait for evidence (topic), not a fixed sleep.
deadline=$((SECONDS + 420))
until timeout 20 ros2 topic list --no-daemon 2>/dev/null | grep -q '/fmu/out/vehicle_odometry'; do
  if [ $SECONDS -gt $deadline ]; then
    echo "FATAL: /fmu/out/vehicle_odometry never appeared in 420 s"
    pgrep -af 'px4|cmake' | cut -c1-80 | head -5
    exit 1
  fi
  sleep 10
done
echo "PX4 publishing after ${SECONDS}s"

echo
echo "=== 3b/5 autonomy stack ==="
setsid nohup ros2 launch uav_bringup sim.launch.py > /tmp/g9_launch.log 2>&1 < /dev/null &
sleep 30
echo "nodes up:"
timeout 20 ros2 node list 2>/dev/null | head -20

echo
echo "=== 4/5 regression flight ==="
python3 "$WORKSPACE/install/uav_bringup/lib/uav_bringup/smoke_flight.py" --flights 3 2>&1 | tail -40
flight=${PIPESTATUS[0]}

echo
echo "=== 5/5 real-time factor under the full stack ==="
timeout 10 gz topic -e -t "/world/$UAV_WORLD/stats" -n 3 2>/dev/null \
  | grep real_time_factor | tail -3 || echo "  (stats unavailable)"

echo
echo "G9 flight exit status: $flight"
exit "$flight"
