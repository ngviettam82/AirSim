#!/bin/bash
# R14 gate: parse, load, count entities, measure RTF.
set -uo pipefail

WORLD=${WORLD:-/mnt/c/code/PX4_ROS2/src/uav_sim_gz/worlds/uav_arena_bk.sdf}
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA

echo "=== 1. SDF parse ==="
if gz sdf -k "$WORLD" 2>&1 | head -20; then echo "  parse: reported above"; fi

echo
echo "=== 2. model count declared in the file ==="
grep -c '<model name=' "$WORLD"

echo
echo "=== 3. headless load, 20 s, then read stats ==="
rm -f /tmp/bk_world.log
timeout 75 gz sim -s -r -v2 "$WORLD" >/tmp/bk_world.log 2>&1 &
sim=$!
sleep 25

echo "--- entities seen by the running server ---"
gz model --list 2>/dev/null | head -50 || echo "  (gz model unavailable)"

echo "--- clock / RTF ---"
timeout 8 gz topic -e -t /stats -n 3 2>/dev/null | grep -E "real_time_factor|sim_time" -A2 | head -20 \
  || timeout 8 gz topic -e -t /world/uav_arena_bk/stats -n 2 2>/dev/null | head -30 \
  || echo "  (no stats topic)"

wait $sim 2>/dev/null
echo
echo "=== 4. server log tail ==="
tail -25 /tmp/bk_world.log
echo
echo "=== 5. errors/warnings in the log ==="
grep -icE "error|severe" /tmp/bk_world.log || true
grep -iE "error|severe" /tmp/bk_world.log | head -10 || true
