#!/bin/bash
# Verify uav0_full: RTF trust threshold AND live forward-sensor data, together.
# See docs/ops-playbook.md S2 for why both, and why repeated.
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
RUNS=${RUNS:-3}
export UAV_MODEL=uav0_full
export UAV_WORLD=${UAV_WORLD:-uav_arena}

echo "=== sync and rebuild ==="
rm -rf "$WORKSPACE/src/uav_sim_gz"
cp -r /mnt/c/code/PX4_ROS2/src/uav_sim_gz "$WORKSPACE/src/"
cd "$WORKSPACE" || exit 1
source /opt/ros/humble/setup.bash
colcon build --packages-select uav_sim_gz 2>&1 | tail -2
source "$WORKSPACE/install/setup.bash"

for run in $(seq 1 "$RUNS"); do
  echo
  echo "=== run $run/$RUNS ==="
  bash "$WORKSPACE/scripts/start_sim.sh" > "/tmp/verify_full_$run.log" 2>&1

  deadline=$((SECONDS + 420))
  until timeout 20 ros2 topic list --no-daemon 2>/dev/null \
      | grep -q '/fmu/out/vehicle_odometry'; do
    if [ $SECONDS -gt $deadline ]; then echo "  PX4 never came up"; break; fi
    sleep 10
  done

  echo "  --- forward sensors ---"
  python3 -u "$WORKSPACE/src/uav_sim_gz/tools/check_front_sensors.py" \
    --rgb-width 640 --rgb-height 480 --seconds 20 2>&1 | sed 's/^/  /'

  echo "  --- real-time factor ---"
  bash "$WORKSPACE/scripts/sample_rtf.sh" "$UAV_WORLD" 120 | sed 's/^/  /'

  bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1
done
