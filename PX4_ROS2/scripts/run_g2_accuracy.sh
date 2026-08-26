#!/bin/bash
# G2: odometry_fused accuracy vs ground truth, injectors off vs on.
# See docs/ops-playbook.md S2: condition B is the real test.
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
SRC=/mnt/c/code/PX4_ROS2/src
export UAV_WORLD=${UAV_WORLD:-uav_arena}
export UAV_MODEL=${UAV_MODEL:-uav0_nav}

G2=$WORKSPACE/install/uav_bringup/lib/uav_bringup/g2_fused_accuracy.py

echo "=== 1/5 sync uav_bringup and rebuild ==="
rm -rf "$WORKSPACE/src/uav_bringup"
cp -r "$SRC/uav_bringup" "$WORKSPACE/src/"
cd "$WORKSPACE" || exit 1
source /opt/ros/humble/setup.bash
colcon build --packages-select uav_bringup 2>&1 | tail -5
source "$WORKSPACE/install/setup.bash"
[ -x "$G2" ] || { echo "FATAL: $G2 not installed"; exit 1; }

echo
echo "=== 2/5 start simulator (world=$UAV_WORLD model=$UAV_MODEL) ==="
bash "$WORKSPACE/scripts/start_sim.sh" 2>&1 | tail -8

echo
echo "=== 3/5 wait for PX4 ==="
deadline=$((SECONDS + 420))
until timeout 20 ros2 topic list --no-daemon 2>/dev/null | grep -q '/fmu/out/vehicle_odometry'; do
  if [ $SECONDS -gt $deadline ]; then echo "FATAL: PX4 never published"; exit 1; fi
  sleep 10
done
echo "PX4 publishing after ${SECONDS}s"

run_condition() {
  local label=$1; shift
  echo
  echo "=== autonomy stack: $label ==="
  pkill -f 'ros2 launch uav_bringup' 2>/dev/null
  pkill -f 'uav_localization/' 2>/dev/null
  pkill -f 'uav_px4_backend/' 2>/dev/null
  sleep 4
  setsid nohup ros2 launch uav_bringup sim.launch.py "$@" \
    > "/tmp/g2_launch_$label.log" 2>&1 < /dev/null &
  sleep 30
  local nodes
  # --no-daemon: plain node list can read empty while running.
  nodes=$(timeout 25 ros2 node list --no-daemon 2>/dev/null | wc -l)
  echo "nodes up: $nodes"
  python3 "$G2" --label "$label" ${G2_EXTRA_ARGS} 2>&1 | tail -25
  return "${PIPESTATUS[0]}"
}

CONDITIONS=${G2_CONDITIONS:-AB}
status_a=0
status_b=0

echo
echo "=== 4/5 condition A - injectors off (ships-as-is) ==="
if [[ "$CONDITIONS" == *A* ]]; then
  run_condition "A-injectors-off"
  status_a=$?
else
  echo "  skipped (G2_CONDITIONS=$CONDITIONS)"
fi

# Derived from shipped config; never ship injectors pre-armed.
SHIPPED=$WORKSPACE/install/uav_bringup/share/uav_bringup/config/localization_params.yaml
INJECTED=/tmp/g2_localization_injected.yaml
sed 's/^      enabled: false$/      enabled: true/' "$SHIPPED" > "$INJECTED"
flipped=$(grep -c '^      enabled: true$' "$INJECTED")
if [ "$flipped" != "2" ]; then
  echo "FATAL: expected 2 injectors armed, got $flipped - condition B would be a lie"
  exit 1
fi
echo "injectors armed: gps drift + vio degrade (verified $flipped flags)"

echo
echo "=== 5/5 condition B - GPS drift + VIO degraded ==="
if [[ "$CONDITIONS" == *B* ]]; then
  # Measured offset (z=-0.240), not estimated: avoids absorbing t=0 drift.
  export G2_EXTRA_ARGS="--frame-offset 0,0,-0.240"
  run_condition "B-injectors-on" "localization_params:=$INJECTED"
  status_b=$?
else
  echo "  skipped (G2_CONDITIONS=$CONDITIONS)"
fi

echo
echo "G2 condition A exit: $status_a   condition B exit: $status_b"
# Reporting only A hid every condition-B failure this gate ever had.
if [ "$status_a" != 0 ]; then exit "$status_a"; fi
exit "$status_b"
