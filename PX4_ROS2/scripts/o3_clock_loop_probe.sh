#!/bin/bash
# P10.8b diagnostic (NOT part of the o3-* gate rounds): does `ros2 bag play
# --loop` actually make /clock go BACKWARD at the wrap point, or does it
# keep /clock monotonic and just replay the DATA again on a continuing
# timeline? G-O3 Task D's premise (a --loop wrap = a real clock regression)
# depends on the answer -- found live: the loop round's clock_regressions
# stayed 0 for the entire ~427s run despite the 248s bag looping at least
# once, which should not happen if the wrap is a true rewind.
set -o pipefail
WORKSPACE=$HOME/PX4_ROS2
LOGDIR=$HOME/gate_logs
mkdir -p "$LOGDIR"
BAG_DIR=$(cat "$LOGDIR/o3_bag_dir.txt")

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
export ROS_DOMAIN_ID=99

pkill -f 'ros2 bag play' 2>/dev/null
pkill -f 'ros2 topic echo /clock' 2>/dev/null
sleep 1

CLOCK_LOG="$LOGDIR/o3_clock_probe.log"
: > "$CLOCK_LOG"

timeout 270 ros2 bag play "$BAG_DIR" --clock --loop > "$LOGDIR/o3_clock_probe_play.log" 2>&1 &
PLAY_PID=$!
sleep 6   # let play actually start publishing /clock before echo tries to resolve its type

timeout 260 ros2 topic echo /clock --field clock.sec > "$CLOCK_LOG" 2>&1 &
ECHO_PID=$!

wait "$PLAY_PID" 2>/dev/null
echo "play ended"

wait "$ECHO_PID" 2>/dev/null

echo "=== /clock.sec sequence: first 5, then every regression found ==="
python3 -c "
vals = []
for line in open('$CLOCK_LOG'):
    line = line.strip()
    if not line or line == '---':
        continue
    try:
        vals.append(int(line))
    except ValueError:
        pass
print('n_samples=%d first=%s last=%s min=%s max=%s' % (
    len(vals), vals[:5], vals[-5:], min(vals) if vals else None, max(vals) if vals else None))
regressions = [(i, vals[i-1], vals[i]) for i in range(1, len(vals)) if vals[i] < vals[i-1]]
print('n_regressions=%d' % len(regressions))
for i, prev, cur in regressions[:10]:
    print('  at sample %d: %d -> %d (delta %d)' % (i, prev, cur, cur - prev))
"
unset ROS_DOMAIN_ID
