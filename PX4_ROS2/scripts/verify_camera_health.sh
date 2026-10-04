#!/bin/bash
# Verify camera_health_node: healthy baseline, then fault-injection (kill bridge).
# See docs/ops-playbook.md S2 (untested health monitor rationale).
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
export UAV_MODEL=${UAV_MODEL:-uav0_full}
export UAV_WORLD=${UAV_WORLD:-uav_arena}

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

echo "=== 1/5 simulator ==="
bash "$WORKSPACE/scripts/start_sim.sh" 2>&1 | tail -2

deadline=$((SECONDS + 420))
until timeout 20 ros2 topic list --no-daemon 2>/dev/null \
    | grep -q '/fmu/out/vehicle_odometry'; do
  if [ $SECONDS -gt $deadline ]; then echo "FATAL: PX4 never published"; exit 1; fi
  sleep 10
done
echo "PX4 publishing after ${SECONDS}s"
sleep 10

echo
echo "=== 2/5 camera_health_node ==="
setsid nohup ros2 run uav_perception camera_health_node \
  > /tmp/camera_health.log 2>&1 < /dev/null &
sleep 15

READER="$WORKSPACE/src/uav_perception/test/read_camera_health.py"

echo
echo "=== 3/5 healthy baseline: what the monitor reports ==="
python3 -u "$READER" --seconds 8 2>&1

echo
echo "=== 4/5 cross-check against image_rate_probe (independent implementation) ==="
ros2 run uav_sim_gz image_rate_probe --ros-args -p seconds:=20.0 2>&1 | grep -v '^\['

echo
echo "=== 5/5 fault injection: kill the image bridges ==="
pkill -f 'gz_image_bridge' 2>/dev/null
pkill -f 'image_bridge' 2>/dev/null
echo "image bridges killed, waiting for the monitor to notice"
sleep 12
python3 -u "$READER" --seconds 8 2>&1

pkill -f 'camera_health_node' 2>/dev/null
bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1
