#!/bin/bash
# What actually produces the deep RTF stalls that block D3 (S10).
#
# WHY THIS EXISTS. Three D3 runs measured p50 = 1.000 with min 0.102-0.204: the simulator
# runs at real time nearly all the time and `mean` is dragged down by rare deep stalls. The
# standing explanation was coincident sensor render, and one proposal was to merge
# camera_front and depth_front into a single rgbd_camera. Arithmetic refuses that on its
# own: the four render sensors are 15/15/30/20 Hz, so all four coincide every 200 ms -- five
# times a second, which is not rare, and /world/*/stats averages over a window of about that
# size. A once-per-200-ms cost cannot make a bimodal distribution. So the cause is unknown,
# and this measures it instead of assuming it.
#
# TWO ARMS, and the comparison is the point (R27-3):
#   A  simulator alone      -- Gazebo, its sensors, no ROS stack at all
#   B  simulator + stack    -- same world, plus the bridge, perception and the autonomy nodes
# Stalls in A are the simulator's own render loop. Stalls that appear only in B are the load
# this project puts NEXT to the simulator, and the fix would be a different one entirely.
#
# What is recorded per stats message: wall arrival, sim_time, real_time and the RTF Gazebo
# itself reports. Instantaneous RTF is recomputed from the two clocks rather than trusted,
# because a stall that swallows the stats publisher would show as a gap in ARRIVALS while
# the reported field stays smooth -- and that difference is itself the diagnosis.
#
# Usage: bash scripts/diagnose_rtf_stalls.sh [seconds_per_arm]
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
CANON=/mnt/c/code/PX4_ROS2
SECONDS_PER_ARM=${1:-90}
export UAV_MODEL=${UAV_MODEL:-uav0_full}
export UAV_WORLD=${UAV_WORLD:-uav_arena}
LOGDIR=$HOME/gate_logs/rtf_stalls
mkdir -p "$LOGDIR"

busy=$(ps -eo comm | grep -cE '^(colcon|ctest|cc1plus)$')
if [ "$busy" -ne 0 ]; then
  echo "FATAL: $busy build/test process(es) running -- an RTF measurement taken beside a"
  echo "       build measures the build. Wait for it."
  exit 2
fi

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

collect() {
  local arm=$1
  echo "=== collecting ${SECONDS_PER_ARM}s of stats: arm $arm ==="
  timeout $((SECONDS_PER_ARM + 15)) gz topic -e -t "/world/$UAV_WORLD/stats" \
    > "$LOGDIR/stats_$arm.txt" 2>/dev/null &
  local pid=$!
  # Timestamp on OUR side: the gz text dump carries no arrival time of its own.
  ( local start end
    start=$(date +%s.%N)
    while kill -0 $pid 2>/dev/null; do
      echo "$(date +%s.%N)" >> "$LOGDIR/heartbeat_$arm.txt"
      sleep 0.05
    done ) &
  sleep "$SECONDS_PER_ARM"
  kill $pid 2>/dev/null
  wait $pid 2>/dev/null
  echo "  $(grep -c real_time_factor "$LOGDIR/stats_$arm.txt" 2>/dev/null) stats messages"
}

echo "################ arm A: simulator alone ################"
bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1
sleep 3
bash "$WORKSPACE/scripts/start_sim.sh" 2>&1 | tail -2
deadline=$((SECONDS + 420))
until timeout 20 ros2 topic list --no-daemon 2>/dev/null | grep -q '/fmu/out/vehicle_odometry'; do
  if [ $SECONDS -gt $deadline ]; then echo "FATAL: PX4 never published"; exit 1; fi
  sleep 10
done
echo "PX4 publishing after ${SECONDS}s"
collect A

echo
echo "################ arm B: simulator + full stack, perception ON ################"
setsid nohup ros2 launch uav_bringup sim.launch.py perception:=true \
  > "$LOGDIR/launch_B.log" 2>&1 < /dev/null &
sleep 40
echo "nodes up: $(timeout 25 ros2 node list --no-daemon 2>/dev/null | wc -l)"
collect B

bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1
pkill -f 'ros2 launch uav_bringup' 2>/dev/null

echo
echo "################ verdict ################"
python3 "$CANON/scripts/rtf_stall_verdict.py" "$LOGDIR"
