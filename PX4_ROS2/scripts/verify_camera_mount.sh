#!/bin/bash
# P5 closing gate (b): camera_front mount vs a box at a known world pose,
# checked at TWO standoffs so a constant (mount translation) separates from
# a range-proportional term (mount rotation / range scale). Static test, no
# flight; see docs/ops-playbook.md S4 (runtime spawn safe; sensors not).
# Usage: bash scripts/verify_camera_mount.sh
# Env: UAV_MODEL UAV_WORLD STANDOFF_NEAR STANDOFF_FAR
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
export UAV_MODEL=${UAV_MODEL:-uav0_track}
export UAV_WORLD=${UAV_WORLD:-uav_arena}
STANDOFF_NEAR=${STANDOFF_NEAR:-3.0}
STANDOFF_FAR=${STANDOFF_FAR:-5.0}
BOX_THICKNESS=0.4
BOX_WIDTH=1.0
BOX_HEIGHT=0.6
BOX_NAME=pb_mount_box

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

PROBE=$WORKSPACE/src/uav_world_model/test/camera_mount_accuracy.py
[ -f "$PROBE" ] || { echo "FATAL: $PROBE missing, sync uav_world_model"; exit 1; }
[ -x "$WORKSPACE/install/uav_world_model/lib/uav_world_model/world_model_node" ] || {
  echo "FATAL: world_model_node not installed, build uav_world_model"; exit 1; }
CONFIG=$WORKSPACE/install/uav_world_model/share/uav_world_model/config/world_model_params.yaml
SHIPPED=$WORKSPACE/install/uav_bringup/share/uav_bringup/config/localization_params.yaml

# Regenerated every run; not a diagnostic frame, safe in /tmp (ops-playbook S2).
BOX_SDF=/tmp/pb_mount_box.sdf
cat > "$BOX_SDF" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='$BOX_NAME'>
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

echo "=== 1/6 simulator (model=$UAV_MODEL world=$UAV_WORLD) ==="
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
  --pose-only --standoff "$STANDOFF_NEAR" > /tmp/pb_pose.log 2>&1
POSE_LINE=$(grep '^BOX_POSE ' /tmp/pb_pose.log || true)
if [ -z "$POSE_LINE" ]; then
  echo "FATAL: could not compute box pose:"
  cat /tmp/pb_pose.log
  bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1
  exit 1
fi
read -r _ BOX_X BOX_Y BOX_Z YAW <<< "$POSE_LINE"
echo "  box centre: x=$BOX_X y=$BOX_Y z=$BOX_Z (drone yaw=$YAW rad)"

echo
echo "=== 3/6 spawn the box ==="
gz service -s "/world/$UAV_WORLD/create" \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
  --req "sdf_filename: '$BOX_SDF', name: '$BOX_NAME', \
         pose: {position: {x: $BOX_X, y: $BOX_Y, z: $BOX_Z}}" 2>&1 | tail -2
sleep 3
echo -n "  box present in the world: "
gz model --list 2>/dev/null | grep -c "$BOX_NAME"

echo
echo "=== 4/6 autonomy stack: backend + localization, then extractor + world model ==="
setsid nohup ros2 launch uav_bringup sim.launch.py localization_params:="$SHIPPED" \
  > /tmp/pb_launch.log 2>&1 < /dev/null &
sleep 25
setsid nohup ros2 run uav_perception obstacle_extractor_node \
  --ros-args -p camera:=front -p use_sim_time:=true \
  > /tmp/pb_extractor.log 2>&1 < /dev/null &
setsid nohup ros2 run uav_world_model world_model_node \
  --ros-args --params-file "$CONFIG" -p use_sim_time:=true \
  > /tmp/pb_world.log 2>&1 < /dev/null &
sleep 8
tail -3 /tmp/pb_world.log

echo
echo "=== 5/6 measure at two standoffs ==="
python3 -u "$PROBE" \
  --world "$UAV_WORLD" --box-name "$BOX_NAME" \
  --box-x "$BOX_X" --box-y "$BOX_Y" --box-z "$BOX_Z" \
  --box-thickness "$BOX_THICKNESS" \
  --standoff-near "$STANDOFF_NEAR" --standoff-far "$STANDOFF_FAR" \
  --out /tmp/pb_result.json 2>&1 | tail -30
status=${PIPESTATUS[0]}

echo
echo "=== 6/6 teardown ==="
pkill -f 'obstacle_extractor_node' 2>/dev/null
pkill -f 'world_model_node' 2>/dev/null
pkill -f 'ros2 launch uav_bringup' 2>/dev/null
bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1

echo
echo "P5 gate (b) exit status: $status"
exit "$status"
