#!/bin/bash
# G4 left one question open: localization_mux_node comes up but never announces
# a source when the aircraft is PARKED under `perception:=false`, so
# state/estimator_source never appears and G4 cannot reach a verdict (R27-1).
#
# Two branches to tell apart, and only one of them is a product finding:
#   (a) the mux genuinely has no valid source in this configuration
#       -> a real R0 question about when localization is trusted while parked
#   (b) the harness asked for the wrong thing (launch args / topic / timing)
#
# This script does not guess: it prints what each stage actually publishes.
# Usage: bash scripts/adjudicate_mux_when_parked.sh
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
export UAV_MODEL=${UAV_MODEL:-uav0_nav}
export UAV_WORLD=${UAV_WORLD:-uav_arena}

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1
sleep 3
bash "$WORKSPACE/scripts/start_sim.sh" >/tmp/adj_sim.log 2>&1
deadline=$((SECONDS + 300))
until timeout 20 ros2 topic list --no-daemon 2>/dev/null \
    | grep -q '/fmu/out/vehicle_odometry'; do
  if [ $SECONDS -gt $deadline ]; then echo "FAILED TO MEASURE: PX4 never published"; exit 2; fi
  sleep 10
done
echo "PX4 up after ${SECONDS}s"

ros2 launch uav_bringup sim.launch.py \
  localization:=true perception:=false mission:=false >/tmp/adj_stack.log 2>&1 &
STACK=$!
sleep 40

echo
echo "=== upstream: do the two SOURCES publish at all? ==="
for topic in localization/gps_odometry localization/vio_odometry; do
  n=$(timeout 12 ros2 topic hz "/uav/uav0/$topic" --once 2>/dev/null | head -2 | tail -1)
  echo "  $topic: ${n:-<no data in 12s>}"
done

echo
echo "=== their STATUS (is_valid is what the mux gates on) ==="
for topic in localization/gps_status localization/vio_status; do
  echo "  --- $topic"
  timeout 12 ros2 topic echo "/uav/uav0/$topic" --once 2>&1 \
    | grep -E 'is_valid|source|position_uncertainty|detail' | head -4 | sed 's/^/    /'
done

echo
echo "=== mux OUTPUT ==="
echo "  --- state/localization_status"
timeout 12 ros2 topic echo /uav/uav0/state/localization_status --once 2>&1 \
  | grep -E 'is_valid|source|detail|quality' | head -5 | sed 's/^/    /'
echo "  --- state/estimator_source"
timeout 12 ros2 topic echo /uav/uav0/state/estimator_source --once 2>&1 | head -3 | sed 's/^/    /'
echo "  --- state/odometry_fused (does a pose come out at all?)"
timeout 12 ros2 topic echo /uav/uav0/state/odometry_fused --once 2>&1 \
  | grep -E 'frame_id|^    x:|^    y:|^    z:' | head -4 | sed 's/^/    /'

echo
echo "=== what the mux itself said ==="
grep -iE 'mux|estimator_source|no valid|selected|rejected' /tmp/adj_stack.log | tail -8 | sed 's/^/  /'

kill "$STACK" 2>/dev/null
wait "$STACK" 2>/dev/null
bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1
echo
echo "=== READ IT LIKE THIS ==="
echo "  vio_status.is_valid=true but estimator_source silent -> branch (b), harness"
echo "  vio_status.is_valid=false while parked                -> branch (a), PRODUCT question"
echo "ADJUDICATION DONE"
