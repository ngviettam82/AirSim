#!/bin/bash
# =============================================================================
# PX4 Autopilot SITL Launcher for AirSim
# Repository Location: PX4Scripts/run_px4_sitl.sh
# Supports single-vehicle (default: 0) and multi-vehicle (0, 1, 2, ...) instances
# =============================================================================
set -e

# 1. Parse instance number (default: 0)
instance_num=0
if [ -n "$1" ]; then
    instance_num="$1"
fi

# 2. Resolve repository directories
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AIRSIM_ROOT="$(dirname "$SCRIPT_DIR")"

# 3. Locate PX4 Autopilot installation
if [ -n "$PX4_AUTOPILOT_DIR" ] && [ -d "$PX4_AUTOPILOT_DIR" ]; then
    PX4_DIR="$PX4_AUTOPILOT_DIR"
elif [ -d "$AIRSIM_ROOT/PX4-Autopilot" ]; then
    PX4_DIR="$AIRSIM_ROOT/PX4-Autopilot"
elif [ -d "$HOME/PX4-Autopilot" ]; then
    PX4_DIR="$HOME/PX4-Autopilot"
elif [ -d "/home/ubuntu/PX4-Autopilot" ]; then
    PX4_DIR="/home/ubuntu/PX4-Autopilot"
else
    echo "[ERROR] PX4-Autopilot directory not found!"
    echo "Checked: \$PX4_AUTOPILOT_DIR, $AIRSIM_ROOT/PX4-Autopilot, $HOME/PX4-Autopilot, /home/ubuntu/PX4-Autopilot"
    exit 1
fi

# 4. Auto-detect Windows Host IP from WSL default gateway
WIN_HOST_IP=$(ip route | awk '/default/ {print $3}')
if [ -z "$WIN_HOST_IP" ]; then
    WIN_HOST_IP="127.0.0.1"
fi

# Port calculations for logging banner:
# PX4 SITL defaults:
# - Simulator TCP: 4560 + instance_num
# - MAVLink remote: 14550 + instance_num
# - MAVLink onboard: 14540 + instance_num (local: 14580 + instance_num)
TCP_PORT=$((4560 + instance_num))
UDP_GCS_PORT=$((14550 + instance_num))
UDP_ONBOARD_PORT=$((14540 + instance_num))

echo "========================================================================="
echo "  PX4 Autopilot SITL - AirSim Bridge"
echo "========================================================================="
echo "  - Script Source     : $SCRIPT_DIR/run_px4_sitl.sh"
echo "  - PX4 Codebase      : $PX4_DIR"
echo "  - Instance Index    : $instance_num"
echo "  - AirSim Host TCP   : $WIN_HOST_IP:$TCP_PORT"
echo "  - Airframe Model    : 3DR Iris Quadrotor X (SYS_AUTOSTART=10016)"
echo "  - QGC Telemetry     : UDP Port $UDP_GCS_PORT -> $WIN_HOST_IP"
echo "  - Onboard Telemetry : UDP Port $UDP_ONBOARD_PORT"
echo "========================================================================="

# 5. Configure PX4 SITL Environment
export PX4_SYS_AUTOSTART=10016
export PX4_SIM_MODEL=none
export PX4_SIM_HOSTNAME="$WIN_HOST_IP"

BUILD_DIR="$PX4_DIR/ROMFS/px4fmu_common"
instance_path="$PX4_DIR/build/px4_sitl_default"
BIN_DIR="$instance_path/bin/px4"
TEST_DATA="$PX4_DIR/test_data"
working_dir="$instance_path/instance_$instance_num"

mkdir -p "$working_dir"
cd "$working_dir"

# 6. Clean parameter caches and eeprom for this instance to prevent dirty state cross-contamination
rm -f parameters*.bson eeprom 2>/dev/null || true
rm -f rootfs/eeprom 2>/dev/null || true

# 7. Execute PX4 POSIX SITL for this instance
exec "$BIN_DIR" \
    -i "$instance_num" \
    "$BUILD_DIR" \
    -s etc/init.d-posix/rcS \
    -t "$TEST_DATA"
