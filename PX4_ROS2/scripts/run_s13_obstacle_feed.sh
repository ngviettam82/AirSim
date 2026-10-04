#!/bin/bash
# S13: fly the configuration the aircraft will actually fly, and prove the flag matters.
#
# THE GAP (P12.4, 2026-08-25). sim.launch.py has always run require_obstacle_feed FALSE
# and real.launch.py runs it TRUE (P6 Decision 4). A grep over scripts/ found no gate that
# has ever run with true, so the branch the aircraft takes had never executed anywhere.
# The sharpest is in local_planner_node.cpp: a Hold is softened into a Clear ONLY when the
# flag is false. On the aircraft the Hold stands -- and had never stood.
#
# WHY perception:=false. S13 is not an avoidance test. It asks what the planner does when
# the obstacle MAP IS MISSING, which is the aircraft's actual situation today: there is no
# real camera driver yet (P11.3), and real.launch.py ships perception off for exactly that
# reason. Running with no feed at all is therefore not a contrived case -- it is the case.
#
# WHY BOTH ARMS. A single green run with the flag on would prove nothing: an aircraft whose
# map never went stale would pass it too. The control arm has to show the softening
# happening, or the two configurations are indistinguishable and the first arm is empty
# (R27-3). Each arm judges itself; both must pass.
#
# Usage: bash scripts/run_s13_obstacle_feed.sh
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
CANON=/mnt/c/code/PX4_ROS2
export UAV_MODEL=${UAV_MODEL:-uav0_track}   # the depth-capable variant; uav0/uav0_nav have none
export UAV_WORLD=${UAV_WORLD:-uav_arena}
UAV_ID=${UAV_ID:-uav0}
LOGDIR=$HOME/gate_logs
EVIDENCE=$WORKSPACE/gate_logs/sim_closeout
mkdir -p "$LOGDIR" "$EVIDENCE"

# R15 #2. Comparing process NAMES, not command lines: `pgrep -af '[c]olcon'` matches this
# script's own argv when it runs through bash -lc, and a gate that accuses itself is a gate
# people learn to skip.
busy=$(ps -eo comm | grep -cE '^(colcon|ctest|cc1plus)$')
if [ "$busy" -ne 0 ]; then
  echo "FATAL: $busy build/test process(es) running. S13 needs the workspace to itself."
  exit 2
fi

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
PROBE=$WORKSPACE/src/uav_bringup/test/s13_obstacle_feed_gate.py
[ -f "$PROBE" ] || { echo "FATAL: $PROBE missing"; exit 1; }

run_arm() {
  local arm=$1
  echo
  echo "############ arm: require_obstacle_feed=$arm ############"
  bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1
  pkill -f 'ros2 launch uav_bringup' 2>/dev/null
  sleep 3

  echo "=== 1/3 simulator (model=$UAV_MODEL world=$UAV_WORLD) ==="
  bash "$WORKSPACE/scripts/start_sim.sh" 2>&1 | tail -2
  local deadline=$((SECONDS + 420))
  until timeout 20 ros2 topic list --no-daemon 2>/dev/null \
      | grep -q '/fmu/out/vehicle_odometry'; do
    if [ $SECONDS -gt $deadline ]; then echo "FATAL: PX4 never published"; return 1; fi
    sleep 10
  done
  echo "PX4 publishing after ${SECONDS}s"

  # ops-playbook section 3: `pkill -f 'ros2 launch uav_bringup'` does NOT reliably cascade
  # SIGTERM to the children, so nodes from the previous arm survive it. Two
  # px4_command_gateway_node serving the same /backend/arm is UNDEFINED BEHAVIOUR in ROS 2
  # and the arm call simply never returns -- which is exactly what both arms reported on
  # 2026-08-25 ("no reply within 20 s"), after the first run had armed fine.
  #
  # The playbook prescribes counting rather than trusting the kill. Doing that.
  local orphans
  orphans=$(pgrep -c -f 'install/uav_px4_backend/lib\|install/uav_localization/lib' 2>/dev/null || echo 0)
  if [ "$orphans" -ne 0 ]; then
    echo "  $orphans orphaned backend/localization node(s) survived the kill -- sweeping"
    pkill -f 'install/uav_px4_backend/lib' 2>/dev/null
    pkill -f 'install/uav_localization/lib' 2>/dev/null
    pkill -f 'install/uav_navigation/lib' 2>/dev/null
    pkill -f 'install/uav_control_authority/lib' 2>/dev/null
    pkill -f 'install/uav_safety/lib' 2>/dev/null
    sleep 5
    orphans=$(pgrep -c -f 'install/uav_px4_backend/lib\|install/uav_localization/lib' 2>/dev/null || echo 0)
  fi
  if [ "$orphans" -ne 0 ]; then
    echo "FATAL: $orphans node(s) still alive before starting arm '$arm'. Two stacks would"
    echo "       make /backend/arm hang forever with no error (ops-playbook section 3)."
    return 1
  fi
  echo "  orphan check: 0 surviving nodes"

  echo "=== 2/3 stack (perception OFF: no obstacle feed, which is the point) ==="
  setsid nohup ros2 launch uav_bringup sim.launch.py \
    uav_id:="$UAV_ID" perception:=false require_obstacle_feed:="$arm" \
    > "$LOGDIR/s13_launch_$arm.log" 2>&1 < /dev/null &
  sleep 30
  echo "nodes up: $(timeout 25 ros2 node list --no-daemon 2>/dev/null | wc -l)"
  # Wait for the thing the flight actually needs, instead of guessing a sleep length.
  local armdl=$((SECONDS + 90))
  until timeout 10 ros2 service list --no-daemon 2>/dev/null | grep -q "/uav/$UAV_ID/backend/arm"; do
    if [ $SECONDS -gt $armdl ]; then echo "FATAL: /backend/arm never advertised"; return 1; fi
    sleep 5
  done
  echo "  /backend/arm advertised"

  echo "=== 3/3 flight + advice probe ==="
  python3 -u "$PROBE" --uav-id "$UAV_ID" --require-feed "$arm" \
    2>&1 | tee "$LOGDIR/s13_probe_$arm.log" | tail -20
  local rc=${PIPESTATUS[0]}

  # The warning the softening branch throttles out. Reading it is a second, independent
  # witness: the probe sees the message, this sees the node say it.
  local unguarded
  unguarded=$(grep -c 'flying unguarded' "$LOGDIR/s13_launch_$arm.log" 2>/dev/null || echo 0)
  echo "  node log said 'flying unguarded' $unguarded time(s)"

  bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1
  pkill -f 'ros2 launch uav_bringup' 2>/dev/null
  return "$rc"
}

# Control arm first. If the softening cannot be observed at all, the required arm cannot
# mean anything either, and finding that out after two flights instead of one is waste.
run_arm false
control_rc=$?
run_arm true
required_rc=$?

echo
echo "################ S13 verdict ################"
echo "  control  (require_obstacle_feed=false): $([ $control_rc -eq 0 ] && echo PASS || echo FAIL)"
echo "  required (require_obstacle_feed=true) : $([ $required_rc -eq 0 ] && echo PASS || echo FAIL)"

if [ $control_rc -eq 0 ] && [ $required_rc -eq 0 ]; then
  {
    echo "S13 PASS: the flag changes behaviour, both arms flown"
    grep -h 'EVIDENCE.*advice samples\|EVIDENCE.*stale-map\|EVIDENCE.*softened' \
      "$LOGDIR/s13_probe_false.log" "$LOGDIR/s13_probe_true.log" 2>/dev/null
  } > "$EVIDENCE/S13.ok"
  echo
  echo "RESULT: S13 PASS -- the aircraft's configuration has now been flown"
  exit 0
fi
rm -f "$EVIDENCE/S13.ok"
echo
echo "RESULT: S13 FAIL"
exit 1
