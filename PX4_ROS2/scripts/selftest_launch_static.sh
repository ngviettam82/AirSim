#!/bin/bash
# Positive control for src/uav_bringup/test/test_launch_static.py (P12.2, 2026-08-25).
#
# Those tests pin what real.launch.py promises about an aircraft nobody has flown. Their
# whole value is refusing a change that quietly alters what would fly, so each refusal has
# to be watched happening at least once.
#
# Case 2 is the guard P11.2 exists for: real.launch.py must start NOTHING until someone
# types reviewed:=true, because it carries three decisions that cannot be closed from a
# simulator. Case 3 is the mutation that walked past the parity gate on this same day --
# the safety supervisor turned off, every gate still green. Case 6 is the quiet one: a node
# that never receives use_sim_time follows the wall clock while everything around it
# follows /clock, and nothing says so.
#
# Works on a scratch COPY through UAV_BRINGUP_LAUNCH_DIR. The real launch files are never
# touched, so an interrupted run cannot leave a broken bringup in the tree.
#
# Usage: bash scripts/selftest_launch_static.sh
set -o pipefail

CANON=/mnt/c/code/PX4_ROS2
TESTS=$CANON/src/uav_bringup/test/test_launch_static.py
SRC=$CANON/src/uav_bringup/launch
fails=0

source /opt/ros/humble/setup.bash
[ -f "$HOME/PX4_ROS2/install/setup.bash" ] && source "$HOME/PX4_ROS2/install/setup.bash"

expect() {   # expect <name> <want_exit> <mutation-fn> [test-filter]
  local name=$1 want=$2 mutate=$3 filter=${4:-}
  local tmp
  tmp=$(mktemp -d)
  mkdir -p "$tmp/launch"
  cp "$SRC"/*.py "$tmp/launch/"
  "$mutate" "$tmp/launch"

  local args=(-q)
  [ -n "$filter" ] && args+=(-k "$filter")
  UAV_BRINGUP_LAUNCH_DIR="$tmp/launch" python3 -m pytest "$TESTS" "${args[@]}" \
    > "$tmp/out.txt" 2>&1
  local got=$?
  if [ "$got" -eq "$want" ]; then
    echo "  ok    $name (exit $got as expected)"
  else
    echo "  FAIL  $name: expected exit $want, got $got"
    tail -12 "$tmp/out.txt" | sed 's/^/          /'
    fails=$((fails + 1))
  fi
  rm -rf "$tmp"
}

untouched() { :; }

reviewed_defaults_true() {
  python3 - "$1/real.launch.py" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
s = s.replace("            'reviewed',\n            default_value='false',",
              "            'reviewed',\n            default_value='true',")
open(p, "w").write(s)
PY
}

safety_off() {
  python3 - "$1/real.launch.py" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
s = s.replace("DeclareLaunchArgument('safety', default_value='true')",
              "DeclareLaunchArgument('safety', default_value='false')")
open(p, "w").write(s)
PY
}

blackbox_on() {
  python3 - "$1/real.launch.py" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
s = s.replace("            'blackbox',\n            default_value='false',",
              "            'blackbox',\n            default_value='true',")
open(p, "w").write(s)
PY
}

mission_on() {
  python3 - "$1/real.launch.py" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
s = s.replace("DeclareLaunchArgument('mission', default_value='false')",
              "DeclareLaunchArgument('mission', default_value='true')")
open(p, "w").write(s)
PY
}

drop_use_sim_time() {
  python3 - "$1/real.launch.py" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
# One node loses its clock source. Everything else still gets it, which is what makes
# this the quiet failure rather than an obvious one.
s = s.replace("""                    'uav_id': uav_id,
                    'use_sim_time': use_sim_time,
                    'enforcement_enabled': safety_enforcement,""",
              """                    'uav_id': uav_id,
                    'enforcement_enabled': safety_enforcement,""")
open(p, "w").write(s)
PY
}

expose_localization_params() {
  python3 - "$1/real.launch.py" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
s = s.replace("        DeclareLaunchArgument('navigation', default_value='true'),",
              "        DeclareLaunchArgument('localization_params', default_value='x'),\n"
              "        DeclareLaunchArgument('navigation', default_value='true'),")
open(p, "w").write(s)
PY
}

echo "=== selftest: test_launch_static.py ==="
expect "1 the tree as it stands passes (shield is not amnesty)" 0 untouched
expect "2 a bare launch that starts nodes is caught"            1 reviewed_defaults_true \
        "bare_real_launch"
expect "3 the safety supervisor missing is caught"              1 safety_off \
        "safety_supervisor_is_on_the_aircraft"
expect "4 the blackbox switched on is caught"                   1 blackbox_on \
        "blackbox_stays_off"
expect "5 the mission layer switched on is caught"              1 mission_on \
        "mission_layer_stays_off"
expect "6 a node losing use_sim_time is caught"                 1 drop_use_sim_time \
        "told_the_time_source"
expect "7 real exposing localization_params is caught"          1 expose_localization_params \
        "two_signed_exceptions"
expect "8 after seven mutations an untouched tree still passes" 0 untouched

echo
if [ "$fails" -eq 0 ]; then
  echo "SELFTEST PASS (8/8)"
  exit 0
fi
echo "SELFTEST FAIL ($fails case(s))"
exit 1
