#!/bin/bash
# P5.5 gate: target_tracker_node id stability + velocity vs a box moving at a
# known world-Z velocity (ground truth = the gz set_pose commands themselves).
# Drone stays parked (not flying): this exercises the Kalman filter and
# association, not the control loop, so it is NOT blocked by the uav0_full
# RTF gate - see .claude/plan/P5-perception.md Sec 0.1 ("drone HOVER, target
# moving" is measurable now) and Sec 3 (formal D1 still needs real flight).
# See docs/ops-playbook.md S4 (runtime spawn safe; sensors not).
#
# NOT RUN by the author of this script (task boundary: sim access reserved
# for the verifier this round). `gz service .../set_pose` (gz.msgs.Pose
# request) is the same UserCommands service family already proven by
# .../create in verify_marker_detector.sh/verify_obstacle_extractor.sh, but
# the exact request schema has NOT been exercised here. If
# target_track_accuracy.py prints "every gz set_pose call failed", run
# `gz service -l` and `gz service -i -s /world/$UAV_WORLD/set_pose` first and
# fix set_box_pose() there before trusting a FAIL.
#
# Vertical FOV budget (sensor_camera_front/model.sdf: horizontal_fov=1.274 rad,
# 640x480): vertical_fov ~= 2*atan(tan(0.637)/(640/480)) ~= 1.011 rad (29.0 deg
# half-angle). At the default 4.0 m standoff that is a +-2.2 m visible band;
# the default 0.3 m/s * 8 s motion spans only +-1.2 m, centred on the
# boresight, leaving ~1.8x margin either side.
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
export UAV_MODEL=${UAV_MODEL:-uav0_full}
export UAV_WORLD=${UAV_WORLD:-uav_arena}
STANDOFF=${STANDOFF:-4.0}
VERTICAL_SPEED=${VERTICAL_SPEED:-0.3}
DURATION=${DURATION:-8.0}
BOX_NAME=p55_moving_box

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

# Regenerated every run; not a diagnostic frame, safe in /tmp (ops-playbook S2).
BOX_SDF=/tmp/p55_moving_box.sdf
cat > "$BOX_SDF" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='$BOX_NAME'>
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>0.4 0.4 0.4</size></box></geometry>
        <material><ambient>0.1 0.6 0.9 1</ambient><diffuse>0.1 0.6 0.9 1</diffuse></material>
      </visual>
      <collision name="collision">
        <geometry><box><size>0.4 0.4 0.4</size></box></geometry>
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
echo "=== 2/6 read drone pose, compute box spawn pose (t=0, centred on boresight over the motion) ==="
python3 -u "$WORKSPACE/src/uav_perception/test/target_track_accuracy.py" \
  --pose-only --standoff "$STANDOFF" --vertical-speed "$VERTICAL_SPEED" --duration "$DURATION" \
  > /tmp/p55_pose.log 2>&1
POSE_LINE=$(grep '^BOX_POSE ' /tmp/p55_pose.log || true)
if [ -z "$POSE_LINE" ]; then
  echo "FATAL: could not compute box pose:"
  cat /tmp/p55_pose.log
  bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1
  exit 1
fi
read -r _ BOX_X BOX_Y BOX_Z YAW <<< "$POSE_LINE"
echo "  box start centre: x=$BOX_X y=$BOX_Y z=$BOX_Z (drone yaw=$YAW rad)"

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
echo "=== 4/6 obstacle extractor + target tracker ==="
setsid nohup ros2 run uav_perception obstacle_extractor_node \
  --ros-args -p camera:=front -p use_sim_time:=true \
  > /tmp/obstacle_extractor.log 2>&1 < /dev/null &
setsid nohup ros2 run uav_perception target_tracker_node \
  --ros-args -p use_sim_time:=true \
  > /tmp/target_tracker.log 2>&1 < /dev/null &
sleep 5
tail -5 /tmp/obstacle_extractor.log
tail -5 /tmp/target_tracker.log

echo
echo "=== 5/6 move the box at a known velocity and measure ==="
python3 -u "$WORKSPACE/src/uav_perception/test/target_track_accuracy.py" \
  --standoff "$STANDOFF" --vertical-speed "$VERTICAL_SPEED" --duration "$DURATION" \
  --box-name "$BOX_NAME" 2>&1 | tail -30
status=${PIPESTATUS[0]}

echo
echo "=== 6/6 teardown ==="
pkill -f 'obstacle_extractor_node' 2>/dev/null
pkill -f 'target_tracker_node' 2>/dev/null
bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1

echo
echo "P5.5 exit status: $status"
echo "Reminder: this measures the Sec 0.1 'drone parked, target moving' slice."
echo "Formal D1 (target moving AND drone flying) still needs uav0_full RTF >= 0.95."
exit "$status"
