#!/bin/bash
# Positive control for scripts/check_sim_real_parity.py.
#
# That checker is the mechanism enforcing R7 -- "sim and real run the same nodes" -- on
# a file that has never run on an aircraft and cannot be tested by flying it. Its whole
# value is that it fails when the two bringups drift, so it has to be watched failing.
#
# Works on a scratch COPY of the launch directory. Mutating the real one and relying on
# a trap to undo it is one interrupted session away from leaving a broken real.launch.py
# in the tree.
#
# Cases:
#   1. the tree as it stands                     -> PASS
#   2. a node deleted from real only             -> FAIL (the R7 breach that matters:
#                                                   present in sim, absent in flight)
#   3. real exposing localization_params         -> FAIL (debt #9: noise injection must
#                                                   not be constructible on an aircraft)
#   4. real defaulting reviewed to true          -> FAIL (a bare launch would fly with
#                                                   three unreviewed inherited decisions)
#
# Cases 5-9 were added by P12.0 on 2026-08-25, after the four above were shown to be
# the wrong shape for the hole that actually existed. Cases 2-4 all mutate something
# the checker looked at. Nothing mutated what it did NOT look at -- and it did not look
# at conditions, nor at the flag defaults that decide what starts on the aircraft. So
# turning the safety supervisor off left the report reading "identical node sets /
# parity PASSED", which is the single most dangerous sentence this gate can print.
#
#   5. safety supervisor under a constant-false condition -> FAIL (present in the file,
#                                                   can never start; the docstring has
#                                                   claimed this was caught since P11.2)
#   6. safety defaulting to false                -> FAIL (no supervisor on the aircraft)
#   7. safety_enforcement defaulting to false    -> FAIL (preflight D2 silently undone)
#   8. require_obstacle_feed set false on real   -> FAIL (preflight D1; with it false the
#                                                   local planner softens a Hold into a
#                                                   Clear on a stale map -- flying blind)
#   9. a new unsigned launch argument            -> FAIL (an undeclared decision about
#                                                   what runs on the aircraft)
#
# Usage: bash scripts/selftest_sim_real_parity.sh
set -o pipefail

CANON=$(cd "$(dirname "$0")/.." && pwd)
SRC=$CANON/src/uav_bringup/launch
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
fails=0

source /opt/ros/humble/setup.bash
[ -f "$HOME/PX4_ROS2/install/setup.bash" ] && source "$HOME/PX4_ROS2/install/setup.bash"

seed() {
  rm -rf "$TMP/launch"
  mkdir -p "$TMP/launch"
  cp "$SRC"/*.py "$TMP/launch/"
}

expect() {
  local name=$1 want=$2 needle=$3
  python3 "$CANON/scripts/check_sim_real_parity.py" "$TMP/launch" > "$TMP/out.txt" 2>&1
  local got=$?
  if [ "$got" != "$want" ]; then
    echo "  FAIL  $name: expected exit $want, got $got"
    sed 's/^/          /' "$TMP/out.txt" | tail -14
    fails=$((fails + 1))
    return
  fi
  if [ -n "$needle" ] && ! grep -q "$needle" "$TMP/out.txt"; then
    echo "  FAIL  $name: exit was right but the report never said '$needle'"
    sed 's/^/          /' "$TMP/out.txt" | tail -14
    fails=$((fails + 1))
    return
  fi
  echo "  ok    $name (exit $got)"
}

echo "=== check_sim_real_parity.py positive control ==="

seed
expect "the tree as it stands passes" 0 "parity PASSED"

# 2. Drop the safety supervisor from the aircraft only. This is the exact shape of the
#    failure the checker exists for: every sim gate still passes, and the thing guarding
#    the aircraft is not running.
seed
python3 - "$TMP/launch/real.launch.py" <<'PY'
import re, sys
p = sys.argv[1]
s = open(p).read()
s = s.replace("            executable='safety_supervisor_node',",
              "            executable='safety_supervisor_node_REMOVED',")
open(p, "w").write(s)
PY
expect "a node missing on the aircraft is caught" 1 "MISSING ON THE AIRCRAFT"

# 3. Re-expose the sim-only noise-injection override on the real side.
seed
python3 - "$TMP/launch/real.launch.py" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
s = s.replace("        DeclareLaunchArgument('navigation', default_value='true'),",
              "        DeclareLaunchArgument('localization_params', default_value='x'),\n"
              "        DeclareLaunchArgument('navigation', default_value='true'),")
open(p, "w").write(s)
PY
expect "real exposing localization_params is caught" 1 "localization_params"

# 4. Make the aircraft launch live by default.
seed
python3 - "$TMP/launch/real.launch.py" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
s = s.replace("            'reviewed',\n            default_value='false',",
              "            'reviewed',\n            default_value='true',")
open(p, "w").write(s)
PY
expect "reviewed defaulting to true is caught" 1 "reviewed"

# 5. Leave the safety supervisor in the file but wire it under a condition that can
#    never be true. This is the mutation that proved the gate blind on 2026-08-25.
seed
python3 - "$TMP/launch/real.launch.py" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
s = s.replace("            condition=gate(safety),",
              "            condition=IfCondition('false'),")
open(p, "w").write(s)
PY
expect "a node that can never start is caught" 1 "can never start"

# 6. Turn the supervisor off through its flag instead. Same aircraft, same outcome.
seed
python3 - "$TMP/launch/real.launch.py" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
s = s.replace("DeclareLaunchArgument('safety', default_value='true')",
              "DeclareLaunchArgument('safety', default_value='false')")
open(p, "w").write(s)
PY
expect "safety defaulting to false is caught" 1 "safety defaults to"

# 7. Undo preflight decision D2 quietly.
seed
python3 - "$TMP/launch/real.launch.py" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
s = s.replace("            'safety_enforcement',\n            default_value='true',",
              "            'safety_enforcement',\n            default_value='false',")
open(p, "w").write(s)
PY
expect "safety_enforcement defaulting to false is caught" 1 "safety_enforcement"

# 8. Undo preflight decision D1. Nothing in the node set changes; the aircraft simply
#    stops refusing to plan without an obstacle feed.
seed
python3 - "$TMP/launch/real.launch.py" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
s = s.replace("'require_obstacle_feed': True,", "'require_obstacle_feed': False,")
open(p, "w").write(s)
PY
expect "require_obstacle_feed flipped false is caught" 1 "require_obstacle_feed"

# 9. A new flag governing the aircraft that nobody signed.
seed
python3 - "$TMP/launch/real.launch.py" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
s = s.replace("        DeclareLaunchArgument('navigation', default_value='true'),",
              "        DeclareLaunchArgument('experimental_mode', default_value='true'),\n"
              "        DeclareLaunchArgument('navigation', default_value='true'),")
open(p, "w").write(s)
PY
expect "an unsigned launch argument is caught" 1 "nobody signed"

# The other half of the contract: after nine mutations, an untouched tree must still
# pass. A gate that fails everything proves nothing.
seed
expect "an untouched tree still passes (shield is not amnesty)" 0 "parity PASSED"

echo
if [ "$fails" -ne 0 ]; then
  echo "RESULT: $fails case(s) wrong -- the parity checker does not behave as documented."
  exit 1
fi
echo "RESULT: 10/10 -- the checker fails on every drift shape it claims to catch."
