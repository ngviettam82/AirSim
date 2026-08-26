#!/bin/bash
# Positive control for check_px4_msgs_boundary.sh (P12.0, 2026-08-25).
#
# R1 is the boundary the whole architecture rests on: only uav_px4_backend may touch
# the PX4 message package. On 2026-08-24 that boundary became an automatically checked
# invariant, which was the right move -- but nobody ever asked the checker to refuse
# anything. Two holes were found the first time somebody did:
#
#   1. The dependency forms were hand-listed as three literals and <test_depend> was
#      not among them, so any package could pull the dependency in through its test
#      tree and the gate still printed "violations: 0  RESULT: PASS". (case 2)
#   2. The anti-comment guard was a leading '^[^#/*]*' in the regex, which also refused
#      to step over the '/' of an XML closing tag. A manifest with two tags on one line
#      slipped through completely. (case 3)
#
# Cases 4-7 are forms that already worked, kept so a future edit cannot quietly drop
# one. Cases 1, 8, 9 are the other half of the contract, and they matter just as much:
# a checker that shouts at everything is as useless as one that never shouts. A clean
# tree must pass, the backend itself must stay exempt, and a comment that merely says
# the words must not be called a crossing.
#
# WHY THE TEST DATA IS BUILT FROM A VARIABLE: this file lives in scripts/, which the
# checker scans. Written literally, the fixtures below would make the real gate report
# this very file as a violation. The honest fix is to not contain the strings -- not to
# grow the allowlist, which is meant for declared architectural exceptions and would
# lose its meaning if it also absorbed test scaffolding.
#
# HOW IT TESTS WITHOUT TOUCHING THE REAL TREE: check_px4_msgs_boundary.sh starts with
# `cd "$(dirname "$0")/.."` and scans src/ and scripts/ from there, so a copy placed in
# a throwaway tree scans that throwaway tree. No env-var override was added to the
# product script on purpose: an override is a bypass, and a gate with a bypass is a
# gate that will one day be bypassed.
#
# Usage: bash scripts/selftest_px4_msgs_boundary.sh
set -o pipefail

CANON=/mnt/c/code/PX4_ROS2
CHECKER="$CANON/scripts/check_px4_msgs_boundary.sh"
P=px4_msgs
fails=0

run_case() {
  local name=$1 want=$2 setup=$3
  local tmp
  tmp=$(mktemp -d)
  mkdir -p "$tmp/scripts" "$tmp/src/uav_safety" "$tmp/src/uav_px4_backend"
  cp "$CHECKER" "$tmp/scripts/"
  : > "$tmp/scripts/px4_msgs_boundary_allowlist.txt"
  "$setup" "$tmp"

  bash "$tmp/scripts/check_px4_msgs_boundary.sh" > "$tmp/out.txt" 2>&1
  local got=$?
  if [ "$got" -eq "$want" ]; then
    echo "  ok    $name (exit $got as expected)"
  else
    echo "  FAIL  $name: expected exit $want, got $got"
    sed 's/^/          /' "$tmp/out.txt"
    fails=$((fails + 1))
  fi
  rm -rf "$tmp"
}

clean_tree() {
  printf '<package>\n  <name>uav_safety</name>\n  <depend>rclcpp</depend>\n</package>\n' \
    > "$1/src/uav_safety/package.xml"
}

test_depend_crossing() {
  printf '<package>\n  <name>uav_safety</name>\n  <test_depend>%s</test_depend>\n</package>\n' \
    "$P" > "$1/src/uav_safety/package.xml"
}

one_line_manifest_crossing() {
  printf '<package><name>uav_safety</name><test_depend>%s</test_depend></package>\n' \
    "$P" > "$1/src/uav_safety/package.xml"
}

plain_depend_crossing() {
  printf '<package>\n  <depend>%s</depend>\n</package>\n' "$P" \
    > "$1/src/uav_safety/package.xml"
}

export_depend_crossing() {
  printf '<package>\n  <build_export_depend>%s</build_export_depend>\n</package>\n' "$P" \
    > "$1/src/uav_safety/package.xml"
}

include_crossing() {
  clean_tree "$1"
  printf '#include <%s/msg/vehicle_status.hpp>\n' "$P" > "$1/src/uav_safety/node.cpp"
}

cmake_link_crossing() {
  clean_tree "$1"
  printf 'target_link_libraries(node\n  %s::%s\n)\n' "$P" "$P" \
    > "$1/src/uav_safety/CMakeLists.txt"
}

backend_may_cross() {
  clean_tree "$1"
  printf '<package>\n  <depend>%s</depend>\n  <test_depend>%s</test_depend>\n</package>\n' \
    "$P" "$P" > "$1/src/uav_px4_backend/package.xml"
  printf '#include <%s/msg/vehicle_command.hpp>\n' "$P" > "$1/src/uav_px4_backend/gw.cpp"
}

comment_is_not_a_crossing() {
  clean_tree "$1"
  printf '# this package must never depend on %s\n' "$P" > "$1/src/uav_safety/notes.sh"
  printf '// %s::VehicleStatus is deliberately not used here\n' "$P" \
    > "$1/src/uav_safety/node.cpp"
  printf '<package>\n  <!-- do not add %s here -->\n</package>\n' "$P" \
    > "$1/src/uav_safety/package.xml"
}

allowlisted_crossing() {
  clean_tree "$1"
  printf 'import %s\n' "$P" > "$1/src/uav_safety/probe.py"
  echo 'src/uav_safety/probe.py  # declared exception for this selftest' \
    > "$1/scripts/px4_msgs_boundary_allowlist.txt"
}

stale_allowlist_entry() {
  clean_tree "$1"
  echo 'src/uav_safety/gone.py  # points at a file that does not cross' \
    > "$1/scripts/px4_msgs_boundary_allowlist.txt"
}

echo "=== selftest: check_px4_msgs_boundary.sh ==="
run_case "1  clean tree passes (shield is not amnesty)"       0 clean_tree
run_case "2  <test_depend> is a crossing (hole 1)"            1 test_depend_crossing
run_case "3  one-line manifest is a crossing (hole 2)"        1 one_line_manifest_crossing
run_case "4  <depend> is a crossing"                          1 plain_depend_crossing
run_case "5  <build_export_depend> is a crossing"             1 export_depend_crossing
run_case "6  #include is a crossing"                          1 include_crossing
run_case "7  the CMake link form is a crossing"               1 cmake_link_crossing
run_case "8  uav_px4_backend itself stays exempt"             0 backend_may_cross
run_case "9  shell, C++ and XML comments are not crossings"   0 comment_is_not_a_crossing
run_case "10 a declared allowlist entry passes"               0 allowlisted_crossing
run_case "11 a stale allowlist entry fails"                   1 stale_allowlist_entry

echo
if [ "$fails" -eq 0 ]; then
  echo "SELFTEST PASS (11/11)"
  exit 0
fi
echo "SELFTEST FAIL ($fails case(s))"
exit 1
