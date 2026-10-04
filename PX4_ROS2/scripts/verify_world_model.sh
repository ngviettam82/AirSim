#!/bin/bash
# P5.6 gates S5/S6: landmark in odom vs a marker at a known pose, uncertainty A vs B.
# Usage: bash scripts/verify_world_model.sh
# Env: UAV_MODEL UAV_WORLD MARKER_ID MARKER_X MARKER_Y UNCERTAINTY_GROWTH
# Why condition B disqualifies VIO instead of injecting drift: src/uav_world_model/README.md
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
export UAV_MODEL=${UAV_MODEL:-uav0_nav}
export UAV_WORLD=${UAV_WORLD:-uav_arena}
MARKER_ID=${MARKER_ID:-7}

# Off-origin on both axes: a node echoing the drone pose cannot pass by accident.
MARKER_X=${MARKER_X:-1.2}
MARKER_Y=${MARKER_Y:-0.8}
MARKER_Z=0.01

# Condition B must raise the stated error by at least this factor.
UNCERTAINTY_GROWTH=${UNCERTAINTY_GROWTH:-3.0}

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

ACCURACY=$WORKSPACE/src/uav_world_model/test/world_model_accuracy.py
[ -f "$ACCURACY" ] || { echo "FATAL: $ACCURACY missing"; exit 1; }
[ -x "$WORKSPACE/install/uav_world_model/lib/uav_world_model/world_model_node" ] || {
  echo "FATAL: world_model_node not installed, build uav_world_model"; exit 1; }

CONFIG=$WORKSPACE/install/uav_world_model/share/uav_world_model/config/world_model_params.yaml
MARKER_SDF=$WORKSPACE/install/uav_sim_gz/share/uav_sim_gz/models/aruco_$MARKER_ID/model.sdf
[ -f "$MARKER_SDF" ] || { echo "FATAL: $MARKER_SDF missing, build uav_sim_gz"; exit 1; }

echo "=== 1/6 simulator (model=$UAV_MODEL world=$UAV_WORLD) ==="
bash "$WORKSPACE/scripts/start_sim.sh" 2>&1 | tail -2

deadline=$((SECONDS + 420))
until timeout 20 ros2 topic list --no-daemon 2>/dev/null \
    | grep -q '/fmu/out/vehicle_odometry'; do
  if [ $SECONDS -gt $deadline ]; then echo "FATAL: PX4 never published"; exit 1; fi
  sleep 10
done
echo "PX4 publishing after ${SECONDS}s"

echo
echo "=== 2/6 spawn the marker at a known world pose ==="
gz service -s "/world/$UAV_WORLD/create" \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
  --req "sdf_filename: '$MARKER_SDF', name: 'aruco_$MARKER_ID', \
         pose: {position: {x: $MARKER_X, y: $MARKER_Y, z: $MARKER_Z}}" 2>&1 | tail -2
sleep 3
echo -n "  marker present in the world: "
gz model --list 2>/dev/null | grep -c "aruco_$MARKER_ID"

run_condition() {
  local label=$1
  local params=$2
  shift 2

  pkill -f 'ros2 launch uav_bringup' 2>/dev/null
  pkill -f 'uav_localization/' 2>/dev/null
  pkill -f 'uav_px4_backend/' 2>/dev/null
  pkill -f 'marker_detector_node' 2>/dev/null
  pkill -f 'world_model_node' 2>/dev/null
  sleep 4

  setsid nohup ros2 launch uav_bringup sim.launch.py localization_params:="$params" \
    > "/tmp/p56_launch_$label.log" 2>&1 < /dev/null &
  sleep 25

  setsid nohup ros2 run uav_perception marker_detector_node \
    --ros-args -p marker_size_m:=0.5 -p camera:=down -p use_sim_time:=true \
    > "/tmp/p56_detector_$label.log" 2>&1 < /dev/null &

  # use_sim_time matters here: ages and stamps must follow /clock.
  setsid nohup ros2 run uav_world_model world_model_node \
    --ros-args --params-file "$CONFIG" -p use_sim_time:=true \
    > "/tmp/p56_world_$label.log" 2>&1 < /dev/null &
  sleep 8
  tail -3 "/tmp/p56_world_$label.log"

  python3 -u "$ACCURACY" --marker-id "$MARKER_ID" \
    --marker-x "$MARKER_X" --marker-y "$MARKER_Y" --marker-z "$MARKER_Z" \
    --label "$label" --out "/tmp/p56_result_$label.json" "$@" 2>&1 | tail -20
  return "${PIPESTATUS[0]}"
}

echo
echo "=== 3/6 condition A - shipped config, VIO healthy ==="
SHIPPED=$WORKSPACE/install/uav_bringup/share/uav_bringup/config/localization_params.yaml
run_condition "A" "$SHIPPED" --check-position
status_a=$?

echo
echo "=== 4/6 arm condition B - disqualify VIO so the mux must fall back ==="
# Drift injection cannot move this gate: both adapters keep a fixed stated error.
INJECTED=/tmp/p56_localization_vio_lost.yaml
sed 's/^      force_tracking_loss: false$/      force_tracking_loss: true/' \
  "$SHIPPED" > "$INJECTED"
flipped=$(grep -c '^      force_tracking_loss: true$' "$INJECTED")
if [ "$flipped" != "1" ]; then
  echo "FATAL: expected 1 flag flipped, got $flipped - condition B would be a lie"
  exit 1
fi
echo "VIO tracking loss armed (verified $flipped flag)"

echo
echo "=== 5/6 condition B - VIO lost, mux falls back to GPS ==="
run_condition "B" "$INJECTED"
status_b=$?

echo
echo "=== 6/6 verdict ==="
pkill -f 'marker_detector_node' 2>/dev/null
pkill -f 'world_model_node' 2>/dev/null
bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1

PYTHONNOUSERSITE=1 python3 - "$status_a" "$status_b" "$UNCERTAINTY_GROWTH" <<'PY'
import json
import sys

status_a, status_b, growth = int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])
failures = []


def load(label):
    try:
        with open('/tmp/p56_result_%s.json' % label) as handle:
            return json.load(handle)
    except (IOError, ValueError):
        return None


a, b = load('A'), load('B')
if status_a != 0 or a is None:
    failures.append('condition A did not produce a result')
if status_b != 0 or b is None:
    failures.append('condition B did not produce a result')

if a is not None:
    print('  S5 landmark error      %.3f m over %d samples (sources %s)'
          % (a['distance'], a['samples'], ','.join(a['sources'])))
    if a.get('frame_offset'):
        print('     frame offset A      (%+.3f %+.3f %+.3f) m' % tuple(a['frame_offset']))
    print('  S6 uncertainty A       %.4f m' % a['uncertainty'])
    if a['uncertainty'] <= 0.0:
        failures.append('condition A uncertainty is not positive')

if b is not None:
    print('  S6 uncertainty B       %.4f m (sources %s)'
          % (b['uncertainty'], ','.join(b['sources'])))
    print('     landmark error B    %.3f m  (looser: the pose behind it is worse)'
          % b['distance'])
    if b['uncertainty'] <= 0.0:
        failures.append('condition B uncertainty is not positive')

if a is not None and b is not None:
    ratio = b['uncertainty'] / a['uncertainty'] if a['uncertainty'] > 0 else 0.0
    print('  S6 growth              x%.2f (needs x%.1f)' % (ratio, growth))
    if ratio < growth:
        failures.append('uncertainty grew only x%.2f' % ratio)

if failures:
    print('RESULT: FAIL (%s)' % '; '.join(failures))
    sys.exit(1)
print('RESULT: PASS')
PY
verdict=$?

echo
echo "P5.6 exit status: $verdict"
exit "$verdict"
