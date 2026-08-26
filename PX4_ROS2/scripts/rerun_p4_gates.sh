#!/bin/bash
# Re-run the P4 gates the EKF2 back door invalidated: indoor flight, then G2 A/B.
# Usage: rerun_p4_gates.sh          (sequential on purpose - two sims would ruin RTF)
# Env:   FLIGHTS (default 3)
# See .claude/memory.md S5 "PHAT HIEN LON 2026-08-16".
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
SRC=/mnt/c/code/PX4_ROS2/src
LOGS=$HOME/gate_logs
mkdir -p "$LOGS"

source /opt/ros/humble/setup.bash

echo "=== 1/4 sync the packages these gates fly on ==="
for pkg in uav_sim_gz uav_localization uav_px4_backend uav_bringup; do
  rm -rf "${WORKSPACE:?}/src/$pkg"
  cp -r "$SRC/$pkg" "$WORKSPACE/src/"
done
cd "$WORKSPACE" || exit 1
colcon build --packages-select uav_sim_gz uav_localization uav_px4_backend uav_bringup 2>&1 | tail -6
source "$WORKSPACE/install/setup.bash"

echo
echo "=== 2/4 push models and airframes into the PX4 tree ==="
bash "$WORKSPACE/src/uav_sim_gz/scripts/install_to_px4.sh" 2>&1 | tail -4

echo
echo "=== 3/4 GATE indoor: no GPS, no compass, vision only ==="
UAV_MODEL=uav0_nav_indoor FLIGHTS=${FLIGHTS:-3} \
  bash "$WORKSPACE/scripts/run_m5_regression.sh" 2>&1 | tee "$LOGS/indoor.log" | tail -30
indoor=${PIPESTATUS[0]}

sleep 20

echo
echo "=== 4/4 GATE G2: fused accuracy, injectors off then on ==="
bash "$WORKSPACE/scripts/run_g2_accuracy.sh" 2>&1 | tee "$LOGS/g2.log" | tail -40
g2=${PIPESTATUS[0]}

bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1

echo
echo "==================== SUMMARY ===================="
echo "indoor 3/3 exit: $indoor   (log $LOGS/indoor.log)"
echo "G2 A/B     exit: $g2       (log $LOGS/g2.log)"
grep -h "estimator_source\|RESULT\|RMSE\|condition" "$LOGS/indoor.log" "$LOGS/g2.log" 2>/dev/null | tail -20
[ "$indoor" = 0 ] && [ "$g2" = 0 ]
