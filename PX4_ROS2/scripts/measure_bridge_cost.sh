#!/bin/bash
# Bisect uav0_full RTF cost across 4 bridge configs.
# See docs/ops-playbook.md S2 (isolation method, no set -u).
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
export UAV_MODEL=uav0_full
export UAV_WORLD=${UAV_WORLD:-uav_arena}
SAMPLES=${SAMPLES:-120}

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

SHIPPED=$WORKSPACE/install/uav_sim_gz/share/uav_sim_gz/config/bridge_uav0_full.yaml
NO_POINTS=/tmp/bridge_uav0_full_no_points.yaml

# Verify entry count; a silent no-op must not go unnoticed.
python3 - "$SHIPPED" "$NO_POINTS" <<'PY'
import sys
text = open(sys.argv[1]).read()
blocks = text.split('\n- ')
kept = [b for b in blocks if 'depth_camera/points' not in b]
open(sys.argv[2], 'w', newline='\n').write('\n- '.join(kept))
print("bridge entries: %d -> %d" % (len(blocks), len(kept)))
PY
grep -q 'depth_camera/points' "$NO_POINTS" && { echo "FATAL: point cloud still bridged"; exit 1; }

measure() {
  local label=$1
  echo
  echo "=== $label ==="
  bash "$WORKSPACE/scripts/start_sim.sh" > "/tmp/bridge_cost_$label.log" 2>&1

  local deadline=$((SECONDS + 420))
  until timeout 20 ros2 topic list --no-daemon 2>/dev/null \
      | grep -q '/fmu/out/vehicle_odometry'; do
    if [ $SECONDS -gt $deadline ]; then echo "  PX4 never came up"; return 1; fi
    sleep 10
  done

  echo -n "  bridged image topics: "
  timeout 20 ros2 topic list --no-daemon 2>/dev/null | grep -cE 'image_raw|depth_image'
  echo -n "  bridged cloud topics: "
  timeout 20 ros2 topic list --no-daemon 2>/dev/null | grep -c '/points'

  timeout 150 gz topic -e -t "/world/$UAV_WORLD/stats" -n "$SAMPLES" 2>/dev/null \
    | grep real_time_factor | awk '{print $2}' | sort -g \
    | awk '{v[NR]=$1; s+=$1}
           END {if (!NR) {print "  RTF: no data"; exit}
                printf "  RTF  min %.3f | p50 %.3f | max %.3f | mean %.3f (n=%d)\n",
                       v[1], v[int((NR+1)/2)], v[NR], s/NR, NR}'
  bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1
}

UAV_BRIDGE_IMAGES=true  UAV_BRIDGE_CONFIG=            measure "A-everything-as-shipped"
UAV_BRIDGE_IMAGES=true  UAV_BRIDGE_CONFIG=$NO_POINTS  measure "B-no-point-cloud"
UAV_BRIDGE_IMAGES=false UAV_BRIDGE_CONFIG=            measure "C-no-images"
UAV_BRIDGE_IMAGES=false UAV_BRIDGE_CONFIG=$NO_POINTS  measure "D-neither"

echo
echo "done"
