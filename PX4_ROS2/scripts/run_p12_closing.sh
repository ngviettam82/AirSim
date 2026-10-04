#!/bin/bash
# Everything that has to run AFTER src/ stops changing, in the one order that works.
#
# WHY AN ORDER AT ALL (P12, 2026-08-25). G-SIM refuses evidence older than the source it
# describes, so any src/ edit invalidates every recorded line taken before it. Running S8
# and then editing a probe -- which is what happened today -- turns a green line stale and
# costs the whole flight. The rule that falls out of that:
#
#     S1 IS ALWAYS LAST, and nothing may touch src/ or scripts/ while this script runs.
#
# S3 goes after S1 rather than before because it builds into build-cov/install-cov: a
# separate tree specifically so that verify_workspace.sh keeps measuring the ordinary
# binaries. Running it first would leave the machine busy for hours before the cheap
# lines were banked.
#
# The sync is step 0 and it TOUCHES, because `rsync -a` carries the canonical tree's older
# mtimes across and ninja then skips the rebuild in silence (ops-playbook section 8/24b --
# cost a full wrong verdict on 2026-08-25).
#
# Called through a file, never inline: stop_sim.sh derives its kill patterns from package
# names, and a caller whose command line contains one gets killed by its own callee
# (ops-playbook section 3, exit 15 -- hit twice today).
#
# Usage: bash scripts/run_p12_closing.sh
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
CANON=/mnt/c/code/PX4_ROS2
LOGDIR=$HOME/gate_logs
mkdir -p "$LOGDIR"
source /opt/ros/humble/setup.bash
cd "$WORKSPACE" || exit 1

echo "################ 0/5  sync canonical -> workspace, then touch ################"
rsync -a --delete "$CANON/src/" "$WORKSPACE/src/"
find "$WORKSPACE/src" -type f -exec touch {} +
diff -rq "$CANON/src" "$WORKSPACE/src" > "$LOGDIR/closing_diff.log" 2>&1
echo "diff -rq canonical vs workspace: $(wc -l < "$LOGDIR/closing_diff.log") difference(s)"

echo
echo "################ 1/5  build everything ################"
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release > "$LOGDIR/closing_build.log" 2>&1
echo "build exit=$?"
tail -3 "$LOGDIR/closing_build.log"

echo
echo "################ 2/5  S8: M5 regression ################"
bash "$CANON/scripts/run_m5_regression.sh" > "$LOGDIR/closing_m5.log" 2>&1
echo "M5 exit=$?"
grep -E '^flight [0-9]|^completed|^offboard   |^safety     |^safety FAIL|^RESULT|^  RTF' \
  "$LOGDIR/closing_m5.log" | tail -12

echo
echo "################ 3/5  S1: three consecutive clean runs ################"
for i in 1 2 3; do
  bash "$CANON/scripts/verify_workspace.sh" > "$LOGDIR/closing_ws$i.log" 2>&1
  rc=$?
  echo "  run $i: exit=$rc  $(grep -E '^TOTAL|by unit' "$LOGDIR/closing_ws$i.log" | head -2 | tr '\n' ' ')"
  if [ "$rc" -ne 0 ]; then
    echo "  (a dirty run resets the streak; the remaining runs still execute so the rate is measured)"
    grep -E '^\s+[0-9]+ - .*Failed' "$LOGDIR/closing_ws$i.log" | head -5
  fi
done
echo "  history:"
tail -3 "$WORKSPACE/gate_logs/sim_closeout/S1.history" | sed 's/^/    /'

echo
echo "################ 4/5  S3: coverage ################"
bash "$CANON/scripts/measure_coverage.sh" > "$LOGDIR/closing_cov.log" 2>&1
echo "coverage exit=$?"
tail -12 "$LOGDIR/closing_cov.log"

echo
echo "################ 5/5  the board ################"
bash "$CANON/scripts/gate_sim_closeout.sh" 2>&1 | tail -30
