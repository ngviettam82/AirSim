#!/bin/bash
# P5.3 gate: ArUco detection/pose vs a marker at known pose.
# See docs/ops-playbook.md S1, S4 (runtime spawn safe; sensors not).
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
export UAV_MODEL=${UAV_MODEL:-uav0_nav}
export UAV_WORLD=${UAV_WORLD:-uav_arena}
MARKER_ID=${MARKER_ID:-7}

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

# Clear old frames; stale ones fake a scene-fault verdict.
rm -f "$HOME/marker_debug"/marker_hover_*.png

echo "=== 1/5 simulator (model=$UAV_MODEL) ==="
bash "$WORKSPACE/scripts/start_sim.sh" 2>&1 | tail -2

deadline=$((SECONDS + 420))
until timeout 20 ros2 topic list --no-daemon 2>/dev/null \
    | grep -q '/fmu/out/vehicle_odometry'; do
  if [ $SECONDS -gt $deadline ]; then echo "FATAL: PX4 never published"; exit 1; fi
  sleep 10
done
echo "PX4 publishing after ${SECONDS}s"

echo
echo "=== 2/5 spawn the marker under the aircraft ==="
bash "$WORKSPACE/scripts/spawn_marker.sh" "$UAV_WORLD" "$MARKER_ID" 0 0

echo
echo "=== 3/5 autonomy stack ==="
setsid nohup ros2 launch uav_bringup sim.launch.py \
  > /tmp/marker_launch.log 2>&1 < /dev/null &
sleep 25

echo "=== 4/5 marker detector ==="
setsid nohup ros2 run uav_perception marker_detector_node \
  --ros-args -p marker_size_m:=0.5 -p camera:=down \
  > /tmp/marker_detector.log 2>&1 < /dev/null &
sleep 8
tail -2 /tmp/marker_detector.log

echo
echo "=== 5/5 hover over the marker at two heights ==="
python3 -u "$WORKSPACE/src/uav_perception/test/marker_accuracy.py" \
  --marker-id "$MARKER_ID" 2>&1 | tail -20
status=${PIPESTATUS[0]}

pkill -f 'marker_detector_node' 2>/dev/null
bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1

# Re-check offline on the same frames; avoids flying again.
if [ "$status" -ne 0 ]; then
  echo
  echo "=== is the failure in the node or in the scene? ==="
  echo "    running the same detector offline on the frames the camera produced"
  PYTHONNOUSERSITE=1 python3 - <<'PY'
import glob
import os
import cv2

frames = sorted(glob.glob(os.path.expanduser('~/marker_debug/marker_hover_*.png')))
if not frames:
    print('    no frames were saved; the node never received an image')
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
for path in frames:
    image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        print('    %s unreadable' % path)
        continue
    corners, ids, rejected = cv2.aruco.detectMarkers(image, dictionary)
    found = ids.flatten().tolist() if ids is not None else []
    span = cv2.norm(corners[0][0][0] - corners[0][0][1]) if corners else 0.0
    print('    %-44s ids=%s  rejected_candidates=%d  marker_span=%.0f px'
          % (os.path.basename(path), found, len(rejected), span))
    if found:
        print('      -> the frame IS detectable: the fault is in the node')
    else:
        print('      -> the frame is NOT detectable: the fault is in the scene')
PY
fi

echo
echo "P5.3 exit status: $status"
exit "$status"
