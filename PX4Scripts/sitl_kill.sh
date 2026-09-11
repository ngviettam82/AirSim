#!/bin/bash
# =============================================================================
# PX4 Autopilot SITL Clean Process & Lock Termination Utility
# Repository Location: PX4Scripts/sitl_kill.sh
# =============================================================================

echo "Terminating all PX4 SITL instances..."

# 1. Force kill all running PX4 SITL processes
killall -9 px4 2>/dev/null || true
pkill -9 -x px4 2>/dev/null || true

# 2. Clean temporary sockets and lock files to avoid permission or bind errors
rm -rf /tmp/px4* /tmp/uorb* /tmp/mavlink* 2>/dev/null || true

echo "All PX4 SITL processes stopped and lock files cleared."