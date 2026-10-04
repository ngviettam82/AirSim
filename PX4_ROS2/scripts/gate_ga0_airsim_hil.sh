#!/usr/bin/env bash
# G-A0: prove PX4 v1.15.4 runs under AirSim's MAVLink HIL and still speaks uXRCE-DDS.
#
# Run this AFTER the Unreal/AirSim environment is up on the Windows host. Three
# things about this pairing are counter-intuitive and cost a session to find:
#   1. PX4 v1.15.4 is the TCP CLIENT. AirSim listens; PX4 dials PX4_SIM_HOSTNAME.
#   2. AirSim's HIL side is acceptTcp() -- ONE connection, then it stops
#      listening. Never probe 4560 to test liveness; probe RPC 41451 instead.
#   3. AirSim reads its config from a OneDrive-redirected Documents folder, so
#      the launcher passes -settings=<file> rather than writing there.
# Full context: .claude/plan/airsim-migration-proposal.md sections 2c and 7b.
set -o pipefail

AGENT="$HOME/.local/bin/MicroXRCEAgent"
export LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH:-}"
PX4_BIN="$HOME/PX4-Autopilot/build/px4_sitl_default/bin/px4"
PX4_DIR="$HOME/PX4-Autopilot/build/px4_sitl_default/rootfs"
# Not /tmp: WSL discards it on idle shutdown, taking the evidence with it.
LOG="$HOME/gate_logs/ga0_px4.log"
HZ_FLOOR=40.0
TOPIC_FLOOR=40
LOG_MB_CEIL=50

source /opt/ros/humble/setup.bash
source "$HOME/PX4_ROS2/install/setup.bash" 2>/dev/null

# Derived, never literal: the WSL/host pair changes on every reboot.
HOST_IP="$(ip route show default | cut -d' ' -f3)"
[ -n "$HOST_IP" ] || { echo "FAIL: could not derive Windows host IP"; exit 1; }

echo "=== G-A0 · PX4 v1.15.4 <-> AirSim HIL ==="
echo "Windows host (AirSim): $HOST_IP  (HIL 4560, RPC 41451)"

if ! (exec 3<>/dev/tcp/"$HOST_IP"/41451) 2>/dev/null; then
  echo "FAIL: AirSim RPC is not answering on $HOST_IP:41451."
  echo "      Start the Unreal/AirSim environment first."
  exit 1
fi
echo "  [ok] AirSim RPC is up; leaving 4560 untouched for PX4"

[ -x "$AGENT" ] || { echo "FAIL: agent not found at $AGENT"; exit 1; }
pkill -x MicroXRCEAgent 2>/dev/null
P=$(pgrep -x px4); [ -n "$P" ] && kill -9 "$P" 2>/dev/null
sleep 2

setsid nohup "$AGENT" udp4 -p 8888 > /tmp/ga0_agent.log 2>&1 < /dev/null &
sleep 3
pgrep -x MicroXRCEAgent >/dev/null || {
  echo "FAIL: uXRCE-DDS agent died:"; head -3 /tmp/ga0_agent.log; exit 1; }
echo "  [ok] uXRCE-DDS agent up on udp4:8888"

cd "$PX4_DIR" || exit 1
mkdir -p "$(dirname "$LOG")"
# -d drops the pxh shell: on EOF it redraws forever, 6.5 GB log and 136% CPU.
setsid nohup env PX4_SYS_AUTOSTART=10016 PX4_SIM_HOSTNAME="$HOST_IP" \
  "$PX4_BIN" -d > "$LOG" 2>&1 < /dev/null &
echo "  PX4 dialling out, waiting up to 90 s for telemetry..."

for _ in $(seq 90); do
  ros2 topic list 2>/dev/null | grep -q '/fmu/out/vehicle_odometry' && break
  sleep 1
done

grep -aq "Simulator connected on TCP port 4560" "$LOG" && HIL=ok || HIL=no
FMU_COUNT=$(ros2 topic list 2>/dev/null | grep -c '^/fmu/')

# Let EKF2 settle: "ekf2 missing data" at startup is transient and clears.
sleep 15
HZ=$(timeout 25 ros2 topic hz /fmu/out/vehicle_odometry --window 200 2>/dev/null \
       | grep -oP 'average rate: \K[0-9.]+' | tail -1)
HZ=${HZ:-0}

ARM_OUT=$(timeout 60 python3 "$HOME/PX4_ROS2/scripts/ga0_try_arm.py" 2>&1)
echo "$ARM_OUT" | grep -q "RESULT: ARMED" && ARMED=yes || ARMED=no

# A gate that lets PX4 burn a core is measuring its own noise.
LOG_MB=$(( $(stat -c%s "$LOG" 2>/dev/null || echo 0) / 1024 / 1024 ))
PX4_CPU=$(ps -o pcpu= -p "$(pgrep -x px4)" 2>/dev/null | tr -d ' ')

echo
echo "=== KET QUA G-A0 ==="
printf '  %-40s %s\n' "a. AirSim nhan ket noi HIL" "$HIL"
printf '  %-40s %s Hz (nguong %s)\n' "b. nhip vehicle_odometry" "$HZ" "$HZ_FLOOR"
printf '  %-40s %s (nguong %s)\n' "c. so topic /fmu/*" "$FMU_COUNT" "$TOPIC_FLOOR"
printf '  %-40s %s\n' "d. PX4 arm duoc" "$ARMED"
printf '  %-40s %s MB (tran %s)\n' "e. log PX4 khong phinh" "$LOG_MB" "$LOG_MB_CEIL"
printf '  %-40s %s %%  (tham khao)\n' "   CPU cua px4 luc do" "${PX4_CPU:-?}"
echo "$ARM_OUT" | sed 's/^/     /'

VERDICT=PASS
[ "$HIL" = ok ] || VERDICT=FAIL
awk -v h="$HZ" -v f="$HZ_FLOOR" 'BEGIN{exit !(h+0 >= f+0)}' || VERDICT=FAIL
[ "$FMU_COUNT" -ge "$TOPIC_FLOOR" ] || VERDICT=FAIL
[ "$ARMED" = yes ] || VERDICT=FAIL
[ "$LOG_MB" -le "$LOG_MB_CEIL" ] || VERDICT=FAIL
echo "  RESULT: $VERDICT"
echo "  log PX4: $LOG"
[ "$VERDICT" = PASS ]
