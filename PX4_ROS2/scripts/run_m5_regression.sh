#!/bin/bash
# M5 regression flight: arm-takeoff-goto-land-disarm, three times.
# See docs/ops-playbook.md S1 (must be a file, not inline).
# Env: UAV_MODEL (default uav0)   UAV_WORLD (default uav_arena)
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
CANON_SRC=${CANON_SRC:-$WORKSPACE/src/uav_bringup}

# install/ and src/ are separate copies, and the flight runs the installed one while the
# unit tests import the source one. Editing src without rebuilding therefore produces
# green tests over a flight of the OLD binary -- silently. Refuse rather than fly a
# version nobody is looking at.
for f in test/smoke_flight.py; do
  src_copy=$CANON_SRC/$f
  installed=$WORKSPACE/install/uav_bringup/lib/uav_bringup/$(basename "$f")
  if [ -f "$src_copy" ] && [ -f "$installed" ] && ! cmp -s "$src_copy" "$installed"; then
    echo "FATAL: install/ does not match src/ for $(basename "$f")."
    echo "       colcon build --packages-select uav_bringup, then rerun."
    exit 2
  fi
done

export UAV_MODEL=${UAV_MODEL:-uav0}
export UAV_WORLD=${UAV_WORLD:-uav_arena}
FLIGHTS=${FLIGHTS:-3}

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

echo "=== 1/4 simulator (world=$UAV_WORLD model=$UAV_MODEL) ==="
bash "$WORKSPACE/scripts/start_sim.sh" 2>&1 | tail -3

echo
echo "=== 2/4 wait for PX4 ==="
deadline=$((SECONDS + 420))
until timeout 20 ros2 topic list --no-daemon 2>/dev/null \
    | grep -q '/fmu/out/vehicle_odometry'; do
  if [ $SECONDS -gt $deadline ]; then echo "FATAL: PX4 never published"; exit 1; fi
  sleep 10
done
echo "PX4 publishing after ${SECONDS}s"

echo
echo "=== 3/4 autonomy stack ==="
setsid nohup ros2 launch uav_bringup sim.launch.py \
  > /tmp/m5_launch.log 2>&1 < /dev/null &
sleep 30
echo "nodes up: $(timeout 25 ros2 node list --no-daemon 2>/dev/null | wc -l)"

echo
echo "=== 4/4 regression flight ==="
python3 "$WORKSPACE/install/uav_bringup/lib/uav_bringup/smoke_flight.py" \
  --flights "$FLIGHTS" > /tmp/m5_out.txt 2>&1
status=$?
tail -25 /tmp/m5_out.txt

echo
echo "real-time factor during the run:"
bash "$WORKSPACE/scripts/sample_rtf.sh" "$UAV_WORLD" 40 | sed 's/^/  /'

bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1
echo
# G-SIM S8 evidence, written by the run itself. Hand-writing it would make the
# gate a record of what someone believed rather than of what happened -- and the
# gate compares its timestamp against the source tree precisely so a stale belief
# cannot pass.
EVIDENCE=$HOME/PX4_ROS2/gate_logs/sim_closeout
mkdir -p "$EVIDENCE"
# The headline is READ OUT OF THE RUN, never asserted. An earlier version printed the
# constant "safety verdict active" here, which said the flight had a safety verdict
# whether or not it did -- a green line produced instead of measured.
safety_line=$(grep -E "^safety     :" /tmp/m5_out.txt 2>/dev/null)
if [ "$status" -eq 0 ] && [ -n "$safety_line" ] \
   && ! printf '%s' "$safety_line" | grep -q "NO SAMPLE"; then
  {
    echo "M5 $FLIGHTS/$FLIGHTS PASS | ${safety_line#safety     : }"
    grep -E "^flight [0-9]|^offboard   |^safety     " /tmp/m5_out.txt 2>/dev/null
  } > "$EVIDENCE/S8.ok"
else
  rm -f "$EVIDENCE/S8.ok"
fi

echo "M5 exit status: $status"
exit "$status"
