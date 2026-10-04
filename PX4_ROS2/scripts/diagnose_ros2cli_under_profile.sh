#!/bin/bash
# Why did `ros2 node list --no-daemon` report 0 of 5 perception nodes under the
# large-samples DDS profile while the very same command reported 5 of 5 without it?
#
# The stack was NOT down. /tmp/gate_r0/stack_profile.log shows rosbag_manager_node
# RECORDING 46/46 topics and world_model_node accepting 305 obstacles / 600 targets
# in the same window -- i.e. the autonomy pipeline was processing camera data at full
# tilt while the CLI claimed nothing existed. So the question is whether the CLI is
# blind under this profile, and if so why -- because a profile that makes `ros2 node
# list` / `ros2 topic echo` unreliable is a real operational cost of wiring it in,
# even though it is not fatal to the aircraft.
#
# The gate that hit this threw stderr away (2>/dev/null) and so destroyed the one
# piece of evidence that would have answered it in the first round. This script keeps
# stderr and varies exactly one thing at a time: the discovery spin time.
#
# Hypotheses, cheapest discriminating observation first:
#   H1  discovery is merely SLOWER under the profile (allocating a 16 MiB segment per
#       participant costs more than 549 kB) and the CLI default --spin-time 1.0 s is
#       too short. Predicts: a longer spin time finds the nodes.
#   H2  the CLI participant fails to CREATE under the profile (e.g. /dev/shm pressure
#       from ~30 participants x 16 MiB) and returns empty rather than erroring loudly.
#       Predicts: stderr carries a Fast DDS/SHM error, and no spin time helps.
#   H3  the CLI is fine and the nodes really were absent at that instant.
#       Predicts: the launch log has no ready lines -- already REFUTED above, kept
#       here only so the refutation is on the record.
#
# Usage: bash scripts/diagnose_ros2cli_under_profile.sh
set -o pipefail

CANON=/mnt/c/code/PX4_ROS2
WORKSPACE=$HOME/PX4_ROS2
PROFILE=$CANON/src/uav_bringup/config/fastdds_large_samples.xml
OUT=/tmp/diag_cli
mkdir -p "$OUT"; rm -f "$OUT"/*

export UAV_MODEL=uav0_full
export UAV_WORLD=uav_arena

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

STACK_PID=""
cleanup() {
  [ -n "$STACK_PID" ] && { kill "$STACK_PID" 2>/dev/null; wait "$STACK_PID" 2>/dev/null; }
  bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1
}
trap cleanup EXIT

bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1
ros2 daemon stop >/dev/null 2>&1
sleep 3
find /dev/shm -maxdepth 1 -name 'fastrtps_*' -delete 2>/dev/null

export FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILE"
echo "profile in force; /dev/shm free:"
df -h /dev/shm | tail -1

bash "$WORKSPACE/scripts/start_sim.sh" > "$OUT/start.log" 2>&1
deadline=$((SECONDS + 420))
until timeout 20 ros2 topic list --no-daemon 2>/dev/null | grep -q '/fmu/out/vehicle_odometry'; do
  [ $SECONDS -gt $deadline ] && { echo "FATAL: PX4 never published"; exit 2; }
  sleep 10
done
echo "PX4 up after ${SECONDS}s"

ros2 launch uav_bringup sim.launch.py perception:=true > "$OUT/stack.log" 2>&1 &
STACK_PID=$!
sleep 35

echo
echo "=== H3 control: does the launch log say the nodes are alive? ==="
grep -cE 'camera_health_node|marker_detector_node|object_detector_node|target_tracker_node|obstacle_extractor_node' "$OUT/stack.log"
echo "  world_model throughput (proves images are actually flowing):"
grep -oE 'accepted markers=[0-9]+ obstacles=[0-9]+ targets=[0-9]+' "$OUT/stack.log" | tail -1

echo
echo "=== H1/H2: ros2 node list --no-daemon at increasing spin times, stderr KEPT ==="
for st in 1 5 15 30; do
  echo "--- spin-time ${st}s ---"
  timeout 90 ros2 node list --no-daemon --spin-time "$st" > "$OUT/nodes_$st.log" 2>"$OUT/nodes_${st}.err"
  echo "  matched perception nodes: $(grep -cE 'camera_health_node|marker_detector_node|object_detector_node|target_tracker_node|obstacle_extractor_node' "$OUT/nodes_$st.log")"
  echo "  total nodes listed:       $(wc -l < "$OUT/nodes_$st.log")"
  if [ -s "$OUT/nodes_${st}.err" ]; then
    echo "  stderr (this is what the gate threw away):"
    sed 's/^/    /' "$OUT/nodes_${st}.err" | head -10
  else
    echo "  stderr: empty"
  fi
done

echo
echo "=== does the DAEMON path see them, now that it would start under the profile? ==="
ros2 daemon stop >/dev/null 2>&1; sleep 2
timeout 60 ros2 node list > "$OUT/nodes_daemon.log" 2>"$OUT/nodes_daemon.err"
echo "  matched: $(grep -cE 'camera_health_node|marker_detector_node|object_detector_node|target_tracker_node|obstacle_extractor_node' "$OUT/nodes_daemon.log")  total: $(wc -l < "$OUT/nodes_daemon.log")"

echo
echo "=== can the CLI read DATA (not just names) under the profile? ==="
if timeout 30 ros2 topic echo /uav/uav0/state/camera_health --once > "$OUT/camera_health.log" 2>&1; then
  echo "  camera_health: READ OK"
else
  echo "  camera_health: could not read -- see $OUT/camera_health.log"
fi
if timeout 30 ros2 topic echo /fmu/out/vehicle_status --once > "$OUT/vstatus.log" 2>&1; then
  echo "  /fmu/out/vehicle_status: READ OK (the link the R0 gate is about)"
else
  echo "  /fmu/out/vehicle_status: could not read"
fi

echo
echo "  evidence: $OUT"
