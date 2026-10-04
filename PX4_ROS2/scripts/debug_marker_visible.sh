#!/bin/bash
# Ground-truth check: is the marker visible before flying at all?
# See docs/ops-playbook.md S4 (texture-load-white trap).
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
export UAV_MODEL=${UAV_MODEL:-uav0_nav}
export UAV_WORLD=${UAV_WORLD:-uav_arena}
MARKER_ID=${MARKER_ID:-7}
# 0.2: fits camera FOV at 0.24 m altitude.
SCALE=${SCALE:-0.2}

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

MARKER_SDF=$WORKSPACE/install/uav_sim_gz/share/uav_sim_gz/models/aruco_$MARKER_ID/model.sdf

bash "$WORKSPACE/scripts/start_sim.sh" > /tmp/debug_marker_sim.log 2>&1
deadline=$((SECONDS + 300))
until timeout 20 ros2 topic list --no-daemon 2>/dev/null | grep -q '/perception/down/image_raw'; do
  if [ $SECONDS -gt $deadline ]; then echo "FATAL: camera never appeared"; exit 1; fi
  sleep 10
done
echo "camera publishing after ${SECONDS}s"

echo
echo "=== frame with NO marker (control) ==="
python3 -u "$WORKSPACE/src/uav_sim_gz/tools/grab_frame.py" --out /tmp/frame_no_marker.png

echo
echo "=== spawn the board, scaled to $SCALE so it fits the ground-level view ==="
gz service -s "/world/$UAV_WORLD/create" \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
  --req "sdf_filename: '$MARKER_SDF', name: 'aruco_probe', \
         pose: {position: {x: 0.06, y: 0, z: 0.01}}" 2>&1 | tail -1
sleep 4

echo
echo "=== frame WITH marker ==="
python3 -u "$WORKSPACE/src/uav_sim_gz/tools/grab_frame.py" --out /tmp/frame_marker.png

bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1
