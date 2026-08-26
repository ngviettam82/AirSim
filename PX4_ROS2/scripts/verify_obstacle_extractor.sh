#!/bin/bash
# P5.4 gate: depth obstacle distance/size vs a box at a known pose.
# See docs/ops-playbook.md S4 (runtime spawn safe; sensors not). Static test, no flight.
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
export UAV_MODEL=${UAV_MODEL:-uav0_full}
export UAV_WORLD=${UAV_WORLD:-uav_arena}
STANDOFF=${STANDOFF:-3.0}
BOX_THICKNESS=${BOX_THICKNESS:-0.4}
BOX_WIDTH=${BOX_WIDTH:-1.0}
BOX_HEIGHT=${BOX_HEIGHT:-0.6}
# Grid the clusterer actually works on is (width+stride-1)/stride. Shipped depth
# is 640x480 with stride 2 -> a 320x240 grid, and every threshold below it
# (min_cluster_points, neighbour connectivity) is tuned to THAT grid, not to the
# camera. Exposed here so a resolution change can keep the grid constant.
PIXEL_STRIDE=${PIXEL_STRIDE:-2}

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

# Regenerated every run; not a diagnostic frame, safe in /tmp (ops-playbook S2).
BOX_SDF=/tmp/p54_known_box.sdf
cat > "$BOX_SDF" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='p54_known_box'>
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>$BOX_THICKNESS $BOX_WIDTH $BOX_HEIGHT</size></box></geometry>
        <material><ambient>0.9 0.1 0.1 1</ambient><diffuse>0.9 0.1 0.1 1</diffuse></material>
      </visual>
      <collision name="collision">
        <geometry><box><size>$BOX_THICKNESS $BOX_WIDTH $BOX_HEIGHT</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
EOF

echo "=== 1/6 simulator (model=$UAV_MODEL) ==="
bash "$WORKSPACE/scripts/start_sim.sh" 2>&1 | tail -2

deadline=$((SECONDS + 420))
until timeout 20 ros2 topic list --no-daemon 2>/dev/null \
    | grep -q '/uav/uav0/localization/vio_odometry_raw'; do
  if [ $SECONDS -gt $deadline ]; then echo "FATAL: ground-truth odometry never appeared"; exit 1; fi
  sleep 10
done
echo "ground truth odometry publishing after ${SECONDS}s"

echo
echo "=== 2/6 read drone pose, compute box spawn pose ==="
python3 -u "$WORKSPACE/src/uav_perception/test/obstacle_accuracy.py" \
  --pose-only --standoff "$STANDOFF" > /tmp/p54_pose.log 2>&1
POSE_LINE=$(grep '^BOX_POSE ' /tmp/p54_pose.log || true)
if [ -z "$POSE_LINE" ]; then
  echo "FATAL: could not compute box pose:"
  cat /tmp/p54_pose.log
  bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1
  exit 1
fi
read -r _ BOX_X BOX_Y BOX_Z YAW <<< "$POSE_LINE"
echo "  box centre: x=$BOX_X y=$BOX_Y z=$BOX_Z (drone yaw=$YAW rad)"

echo
echo "=== 3/6 spawn the box ==="
gz service -s "/world/$UAV_WORLD/create" \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
  --req "sdf_filename: '$BOX_SDF', name: 'p54_known_box', \
         pose: {position: {x: $BOX_X, y: $BOX_Y, z: $BOX_Z}}" 2>&1 | tail -2
sleep 3
echo -n "  box present in the world: "
gz model --list 2>/dev/null | grep -c "p54_known_box"

echo
echo "=== 4/6 obstacle extractor node (pixel_stride=$PIXEL_STRIDE) ==="
setsid nohup ros2 run uav_perception obstacle_extractor_node \
  --ros-args -p camera:=front -p use_sim_time:=true -p pixel_stride:="$PIXEL_STRIDE" \
  > /tmp/obstacle_extractor.log 2>&1 < /dev/null &
sleep 5
tail -5 /tmp/obstacle_extractor.log

echo
echo "=== 5/6 measure ==="
python3 -u "$WORKSPACE/src/uav_perception/test/obstacle_accuracy.py" \
  --standoff "$STANDOFF" --box-thickness "$BOX_THICKNESS" \
  --box-width "$BOX_WIDTH" --box-height "$BOX_HEIGHT" 2>&1 | tail -30
status=${PIPESTATUS[0]}

echo
echo "=== 6/6 teardown ==="
pkill -f 'obstacle_extractor_node' 2>/dev/null
bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1

echo
echo "P5.4 exit status: $status"
exit "$status"
