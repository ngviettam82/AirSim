#!/usr/bin/env bash
# ==============================================================================
# One-Click Execution Script for AirSim + PX4 ROS 2 Autonomy Stack
# ==============================================================================
# Usage:
#   ./run_autonomy.sh [obstacle_avoidance|scanning_patrol|target_guiding|area_search]
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ALGORITHM="${1:-obstacle_avoidance}"
VEHICLE_NAME="${2:-drone1}"

# Source ROS 2 base distribution
if [ -z "${ROS_DISTRO}" ]; then
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source "/opt/ros/humble/setup.bash"
    elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source "/opt/ros/jazzy/setup.bash"
    fi
fi

# Source workspace overlay
if [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
    source "${SCRIPT_DIR}/install/setup.bash"
elif [ -f "${HOME}/ros2_ws/install/setup.bash" ]; then
    source "${HOME}/ros2_ws/install/setup.bash"
fi

echo "======================================================================"
echo " Launching AirSim + PX4 Autonomy Stack"
echo " Active Algorithm : ${ALGORITHM}"
echo " Vehicle Name     : ${VEHICLE_NAME}"
echo "======================================================================"

# Check if MicroXRCEAgent is already running
AGENT_PID=""
cleanup() {
    if [ -n "${AGENT_PID}" ] && kill -0 "${AGENT_PID}" 2>/dev/null; then
        echo "[Info] Stopping spawned MicroXRCEAgent (PID: ${AGENT_PID})..."
        kill "${AGENT_PID}" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

if ! pgrep -x "MicroXRCEAgent" > /dev/null; then
    echo "[Info] MicroXRCEAgent is not running. Starting MicroXRCEAgent on UDP port 8888..."
    MicroXRCEAgent udp4 -p 8888 &
    AGENT_PID=$!
    echo "[Info] MicroXRCEAgent started (PID: ${AGENT_PID})"
    sleep 1
else
    echo "[Info] MicroXRCEAgent is already running."
fi

# Launch autonomy node
ros2 launch px4_airsim_autonomy autonomy.launch.py \
    algorithm:="${ALGORITHM}" \
    vehicle_name:="${VEHICLE_NAME}"

