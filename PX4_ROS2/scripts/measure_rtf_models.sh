#!/bin/bash
# Re-measure RTF per drone variant after the LIBGL/GPU fix.
# See docs/ops-playbook.md S2 (mean vs p50), S3 (GPU fix).
set -o pipefail

WORKSPACE="$HOME/PX4_ROS2"
WORLD="${UAV_WORLD:-uav_arena}"
SAMPLES="${RTF_SAMPLES:-150}"
MODELS="${RTF_MODELS:-uav0 uav0_nav uav0_full}"
WINDOW="${RTF_WINDOW:-60}"

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

echo "renderer: $(MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA glxinfo -B 2>/dev/null \
  | grep -i 'OpenGL renderer' | cut -d: -f2)"
echo "world: $WORLD  samples/variant: $SAMPLES"
echo

summarise_rtf() {
  timeout 120 gz topic -e -t "/world/$WORLD/stats" -n "$SAMPLES" 2>/dev/null \
    | grep real_time_factor | awk '{print $2}' | sort -g \
    | awk '{v[NR]=$1; s+=$1}
           END {if (!NR) {print "  RTF: no data"; exit}
                printf "  RTF  min %.3f | p50 %.3f | max %.3f | mean %.3f  (n=%d)\n",
                       v[1], v[int((NR+1)/2)], v[NR], s/NR, NR}'
}

# Averaged RTF via /clock delta; --field avoids nanosec match.
sim_clock_seconds() {
  timeout 25 ros2 topic echo /clock --field clock.sec --once 2>/dev/null \
    | tr -dc '0-9'
}

averaged_rtf() {
  local before after wall_before wall_after
  before=$(sim_clock_seconds); wall_before=$(date +%s.%N)
  sleep "$WINDOW"
  after=$(sim_clock_seconds); wall_after=$(date +%s.%N)
  if [ -z "$before" ] || [ -z "$after" ] || [ "$before" = "$after" ]; then
    echo "  RTF averaged: unusable (/clock read '$before' -> '$after')"
    return
  fi
  awk -v a="$before" -v b="$after" -v wa="$wall_before" -v wb="$wall_after" \
    'BEGIN {printf "  RTF averaged over %.0fs wall: %.3f  (sim advanced %.1fs)\n",
            wb-wa, (b-a)/(wb-wa), b-a}' WINDOW="$WINDOW"
}

for model in $MODELS; do
  echo "=== model=$model ==="
  UAV_WORLD="$WORLD" UAV_MODEL="$model" bash "$WORKSPACE/scripts/start_sim.sh" \
    > "/tmp/rtf_start_$model.log" 2>&1
  if [ $? -ne 0 ]; then
    echo "  start_sim FAILED, see /tmp/rtf_start_$model.log"
    tail -5 "/tmp/rtf_start_$model.log"
    continue
  fi

  deadline=$((SECONDS + 240))
  until timeout 15 ros2 topic list --no-daemon 2>/dev/null \
      | grep -q '/fmu/out/vehicle_odometry'; do
    if [ $SECONDS -gt $deadline ]; then echo "  PX4 never came up"; break; fi
    sleep 10
  done

  sensors=$(timeout 15 ros2 topic list --no-daemon 2>/dev/null \
    | grep -cE 'image_raw|/points|scan|depth')
  echo "  render sensor topics: $sensors"
  summarise_rtf
  averaged_rtf
  echo
done

pkill -f 'MicroXRCEAgent'; pkill -f 'px4_sitl_default/bin/px4'
pkill -f 'gz sim'; pkill -f 'parameter_bridge'; pkill -f 'image_bridge'
pkill -f 'sleep infinity'
echo "done (stack stopped)"
