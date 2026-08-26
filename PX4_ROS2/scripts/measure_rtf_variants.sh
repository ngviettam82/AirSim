#!/bin/bash
# BK-world RTF cost: physics is cheap, sensors are the suspect.
# See docs/ops-playbook.md S6 for BK-world context.
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
export PX4_GZ_MODEL_POSE="-9.7,-94.9,0.25,0,0,0"

# Was an inlined 8-sample mean. ops-playbook S2 wants min/p50/max/mean together
# on a bimodal distribution, and 8 samples cannot carry that -- call the canonical
# sampler rather than keep a second, weaker copy of the same measurement.
RTF_SAMPLES="${RTF_SAMPLES:-120}"
sample_rtf() {
  bash "$WORKSPACE/scripts/sample_rtf.sh" "$1" "$RTF_SAMPLES"
}

for combo in "uav_arena_bk uav0" "uav_arena_bk uav0_nav" "uav_arena uav0_nav"; do
  set -- $combo
  world=$1; model=$2
  echo "=== world=$world model=$model ==="
  UAV_WORLD=$world UAV_MODEL=$model bash "$WORKSPACE/scripts/start_sim.sh" >/tmp/rtf_start.log 2>&1
  deadline=$((SECONDS + 300))
  until timeout 15 ros2 topic list --no-daemon 2>/dev/null | grep -q '/fmu/out/vehicle_odometry'; do
    if [ $SECONDS -gt $deadline ]; then break; fi
    sleep 10
  done
  # R27-1: check the measurement before the object. An RTF printed for a world
  # PX4 never joined reads as a result, and it is not one.
  if ! timeout 15 ros2 topic list --no-daemon 2>/dev/null | grep -q '/fmu/out/vehicle_odometry'; then
    echo "  FAILED TO MEASURE: PX4 never came up, no RTF reported for this combo"
    continue
  fi
  echo -n "  "; sample_rtf "$world"
  echo -n "  render sensor topics: "
  timeout 15 ros2 topic list --no-daemon 2>/dev/null \
    | grep -cE 'image_raw|points|scan|depth' || echo 0
done
echo "done"
