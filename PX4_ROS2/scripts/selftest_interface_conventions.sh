#!/bin/bash
# Positive control for src/uav_interfaces/test/test_interface_conventions.py
# (P12.2, 2026-08-25).
#
# These tests guard the definitions eleven packages compile against. A convention broken
# here still builds -- that is the whole point -- so the only thing standing between a
# headerless published message and the whole stack is a test that has been watched
# refusing one.
#
# Case 2 is the expensive one. Every staleness and freshness check in this project reads
# header.stamp, so a published message without a header is a sample nothing downstream can
# age, and the code that reads it keeps working while quietly losing the ability to say
# "this is old".
#
# Cases 5 and 6 are the ones that keep the exception list honest in both directions: an
# entry that stopped being an element type, and an entry that no longer needs its
# exemption. Same shape as the stale-allowlist check in check_px4_msgs_boundary.sh.
#
# Works on a scratch COPY through UAV_INTERFACES_DIR; the real definitions are untouched.
#
# Usage: bash scripts/selftest_interface_conventions.sh
set -o pipefail

CANON=/mnt/c/code/PX4_ROS2
PKG=$CANON/src/uav_interfaces
TESTS=$PKG/test/test_interface_conventions.py
fails=0

expect() {   # expect <name> <want_exit> <mutation-fn> [filter]
  local name=$1 want=$2 mutate=$3 filter=${4:-}
  local tmp
  tmp=$(mktemp -d)
  cp -r "$PKG/msg" "$PKG/srv" "$PKG/action" "$tmp/"
  "$mutate" "$tmp"

  local args=(-q)
  [ -n "$filter" ] && args+=(-k "$filter")
  UAV_INTERFACES_DIR="$tmp" python3 -m pytest "$TESTS" "${args[@]}" > "$tmp/out.txt" 2>&1
  local got=$?
  if [ "$got" -eq "$want" ]; then
    echo "  ok    $name (exit $got as expected)"
  else
    echo "  FAIL  $name: expected exit $want, got $got"
    tail -10 "$tmp/out.txt" | sed 's/^/          /'
    fails=$((fails + 1))
  fi
  rm -rf "$tmp"
}

untouched() { :; }

drop_a_header()      { sed -i '/^std_msgs\/Header header/d' "$1/msg/VehicleState.msg"; }
drop_a_uav_id()      { sed -i '/^string uav_id/d'           "$1/msg/SafetyState.msg"; }
demote_the_header()  {
  # Header still present, but no longer first. The convention is about LEADING with it, so
  # a file that merely contains one somewhere must still be refused.
  #
  # The first version of this mutation compared whole lines and silently did nothing: the
  # real field is `std_msgs/Header header    # frame_id empty; non-spatial status`, with a
  # trailing comment. It removed nothing and appended a duplicate, the header stayed first,
  # and the test passed -- which looked like the test being weak and was the mutation being
  # wrong. Strip comments the same way the test does.
  python3 - "$1/msg/MissionStatus.msg" <<'PY'
import sys
p = sys.argv[1]
lines = open(p).read().split("\n")
kept, header = [], None
for line in lines:
    if line.split("#")[0].strip() == "std_msgs/Header header":
        header = line
        continue
    kept.append(line)
assert header is not None, "no header field found to demote"
for i, line in enumerate(kept):
    if line.split("#")[0].strip().startswith("string uav_id"):
        kept.insert(i + 1, header)
        break
else:
    raise AssertionError("no uav_id field to demote the header below")
open(p, "w").write("\n".join(kept))
PY
}
add_a_headerless_message() {
  printf 'string note\n' > "$1/msg/NewThing.msg"
}
orphan_an_exception() {
  # Obstacle stops being an element of anything, so its exemption has no basis left.
  sed -i 's/^Obstacle\[\] obstacles/string obstacles_removed/' "$1/msg/ObstacleArray.msg"
}
exception_gains_a_header() {
  sed -i '1i std_msgs/Header header' "$1/msg/Obstacle.msg"
}
action_loses_result_code() {
  sed -i '/^uint8 result_code/d' "$1/action/Land.action"
}
result_table_gains_a_field() {
  printf 'string detail\n' >> "$1/msg/ResultCode.msg"
}

echo "=== selftest: test_interface_conventions.py ==="
expect "1 the definitions as they stand pass (shield is not amnesty)" 0 untouched
expect "2 a published message losing its header is caught"   1 drop_a_header    "leads_with_a_header"
expect "3 a published message losing uav_id is caught"       1 drop_a_uav_id    "names_its_aircraft"
expect "4 a header that is no longer first is caught"        1 demote_the_header "leads_with_a_header"
expect "5 a new headerless message is caught"                1 add_a_headerless_message
expect "6 an exception that stopped being an element type"   1 orphan_an_exception "polices_itself"
expect "7 an exception that no longer needs its exemption"   1 exception_gains_a_header \
        "exempted_message_really_lacks"
expect "8 an action losing result_code is caught"            1 action_loses_result_code \
        "result_code"
expect "9 a field creeping into the result table is caught"  1 result_table_gains_a_field \
        "result_table_is_a_table"
expect "10 after nine mutations the real tree still passes"  0 untouched

echo
if [ "$fails" -eq 0 ]; then
  echo "SELFTEST PASS (10/10)"
  exit 0
fi
echo "SELFTEST FAIL ($fails case(s))"
exit 1
