#!/bin/bash
# measure_image_budget.sh with PERCEPTION=1 returned "down/image_raw never
# delivered a frame" for BOTH configurations while reporting the perception stack
# live 5/5 -- and printed nothing at all from the probe, not even its own
# "only N messages" branch. So the probe produced no report; the question is why.
#
# Controlled comparison inside ONE simulator boot, so nothing else can differ:
#   1. probe with NO autonomy stack   (the condition that has worked all along)
#   2. probe WITH perception:=true    (the condition that failed)
# Full probe output is kept both times -- the parent script's grep filter is what
# swallowed the evidence, and that is fixed there too.
#
# Usage: bash scripts/diagnose_probe_under_stack.sh
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
export UAV_MODEL=uav0_full
export UAV_WORLD=${UAV_WORLD:-uav_arena}
WINDOW=${WINDOW:-15}
# `-p seconds:=60` types the value from its LITERAL form, so an integer for a
# double parameter aborts the node at declare time -- the probe then prints
# nothing at all and the run reads as "no frames delivered" (2026-08-24).
case "$WINDOW" in *.*) ;; *) WINDOW="${WINDOW}.0" ;; esac

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

ps -eo comm | grep -qE '^ctest$|^colcon$' && { echo "FATAL: a build/test run holds install/"; exit 2; }

bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1
sleep 3
bash "$WORKSPACE/scripts/start_sim.sh" >/tmp/diag_sim.log 2>&1
deadline=$((SECONDS + 420))
until timeout 20 ros2 topic list --no-daemon 2>/dev/null \
    | grep -q '/fmu/out/vehicle_odometry'; do
  if [ $SECONDS -gt $deadline ]; then echo "FAILED TO MEASURE: PX4 never published"; exit 2; fi
  sleep 10
done
echo "PX4 up after ${SECONDS}s"
sleep 10

show_topics() {
  echo "  image topics visible:"
  timeout 20 ros2 topic list 2>/dev/null | grep -E 'perception/(front|down)' | sed 's/^/    /'
  echo "  publisher count on down/image_raw:"
  timeout 20 ros2 topic info /uav/uav0/perception/down/image_raw 2>&1 | sed 's/^/    /'
}

echo
echo "########## 1/2 NO autonomy stack (the condition that has always worked) ##########"
show_topics
ros2 run uav_sim_gz image_rate_probe --ros-args -p seconds:="$WINDOW" \
  >/tmp/diag_probe_nostack.log 2>&1
echo "  probe exit: $?  (full output below)"
sed 's/^/    /' /tmp/diag_probe_nostack.log

echo
echo "########## 2/2 WITH perception:=true ##########"
ros2 launch uav_bringup sim.launch.py perception:=true >/tmp/diag_stack.log 2>&1 &
STACK=$!
sleep 35
nodes=$(timeout 20 ros2 node list 2>/dev/null \
  | grep -cE 'camera_health_node|marker_detector_node|object_detector_node|target_tracker_node|obstacle_extractor_node')
echo "  perception nodes up: ${nodes:-0}/5"
show_topics
ros2 run uav_sim_gz image_rate_probe --ros-args -p seconds:="$WINDOW" \
  >/tmp/diag_probe_stack.log 2>&1
echo "  probe exit: $?  (full output below)"
sed 's/^/    /' /tmp/diag_probe_stack.log

echo
echo "  what the stack logged about images/bridge:"
grep -iE 'image|bridge|camera|error|fail' /tmp/diag_stack.log | tail -12 | sed 's/^/    /'

kill "$STACK" 2>/dev/null
wait "$STACK" 2>/dev/null
bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1

echo
echo "########## READ IT LIKE THIS ##########"
echo "  1 works, 2 empty  -> the autonomy stack is what starves/blocks the probe"
echo "  both empty        -> not the stack: uav0_full itself is not delivering images now"
echo "  probe exit != 0   -> the probe never ran; read its output above, not the rates"
echo "DIAGNOSIS DONE"
