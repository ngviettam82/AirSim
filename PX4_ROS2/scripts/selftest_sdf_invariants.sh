#!/bin/bash
# Positive control for src/uav_sim_gz/test/sdf_invariants.py (P12.2, 2026-08-25).
#
# The rule this guards is the one CLAUDE.md section 5 calls a killer: rendering sensors
# read <visual>, physics reads <collision>, so a visual larger than its collision makes
# the drone SEE a wall and FLY THROUGH it -- the avoidance test passes in simulation and
# the real aircraft hits something. Case 2 is exactly that shape.
#
# Case 5 is the one the previous tool got backwards. check_world_invariants.py compared
# geometry for EQUALITY, so a collision LARGER than its visual -- which is the SAFE
# direction, and common when a rough box wraps a detailed shape -- was reported as a
# mismatch. A gate that cries at safe geometry teaches people to ignore it.
#
# Cases 6 and 7 pin the two "cannot compute from the file" answers. Both must FAIL rather
# than pass: an unreadable mesh and a freely-rotated box are unknowns, and an unknown
# landing on the safe side is the failure mode this project has paid for twice.
#
# Case 8 pins the ground-decal exemption to its precondition. A flat road is excused
# because the ground plane underneath it is what the aircraft would actually touch. Take
# the ground plane away and the excuse has to disappear with it.
#
# Fixtures are synthetic and minimal, not a copy of the real package: each case is then
# one obvious file rather than a diff against 918 links.
#
# Cases 10-14 guard a DIFFERENT subject: the pin in test_sdf_invariants.py that says WHICH
# geometry is allowed the aircraft-part and external-model exemptions. They mutate a copy of
# the real package because that is what the pin reads. They exist because on 2026-08-26 the
# pin was a raw total, it went red for a benign variant, and it was bumped 11 -> 13 from
# reasoning instead of from the tool -- the measured delta was +4. A total says a number
# moved; these cases require the gate to NAME the geometry that moved.
#
# Usage: bash selftest_sdf_invariants.sh [path-to-sdf_invariants.py]
set -o pipefail

CHECKER=${1:-/mnt/c/code/PX4_ROS2/src/uav_sim_gz/test/sdf_invariants.py}
fails=0

emit_world() {   # emit_world <dir> <extra-links> [omit-ground]
  local dir=$1 extra=$2 omit=$3
  mkdir -p "$dir/worlds" "$dir/models"
  {
    echo '<?xml version="1.0"?><sdf version="1.9"><world name="t">'
    if [ -z "$omit" ]; then
      echo '  <model name="ground"><link name="g">'
      echo '    <collision name="c"><geometry><plane><normal>0 0 1</normal></plane></geometry></collision>'
      echo '    <visual name="v"><geometry><plane><normal>0 0 1</normal></plane></geometry></visual>'
      echo '  </link></model>'
    fi
    printf '%s\n' "$extra"
    echo '</world></sdf>'
  } > "$dir/worlds/t.sdf"
}

run_case() {
  local name=$1 want=$2 extra=$3 omit=$4
  local tmp
  tmp=$(mktemp -d)
  emit_world "$tmp" "$extra" "$omit"
  python3 "$CHECKER" "$tmp" > "$tmp/out.txt" 2>&1
  local got=$?
  if [ "$got" -eq "$want" ]; then
    echo "  ok    $name (exit $got as expected)"
  else
    echo "  FAIL  $name: expected exit $want, got $got"
    sed 's/^/          /' "$tmp/out.txt" | head -12
    fails=$((fails + 1))
  fi
  rm -rf "$tmp"
}

HONEST_WALL='  <model name="w"><link name="wall">
    <visual name="v"><geometry><box><size>4 0.2 3</size></box></geometry></visual>
    <collision name="c"><geometry><box><size>4 0.2 3</size></box></geometry></collision>
  </link></model>'

LYING_WALL='  <model name="w"><link name="wall">
    <visual name="v"><geometry><box><size>4 0.2 3</size></box></geometry></visual>
    <collision name="c"><geometry><box><size>4 0.2 0.4</size></box></geometry></collision>
  </link></model>'

GHOST_WALL='  <model name="w"><link name="wall">
    <visual name="v"><geometry><box><size>4 0.2 3</size></box></geometry></visual>
  </link></model>'

FLAT_ROAD='  <model name="r"><link name="road_1">
    <visual name="v"><pose>0 0 0.02 0 0 0</pose><geometry><box><size>20 3 0.04</size></box></geometry></visual>
  </link></model>'

RAISED_ROAD='  <model name="r"><link name="road_1">
    <visual name="v"><pose>0 0 0.50 0 0 0</pose><geometry><box><size>20 3 0.04</size></box></geometry></visual>
  </link></model>'

GENEROUS_COLLISION='  <model name="w"><link name="wall">
    <visual name="v"><geometry><cylinder><radius>0.3</radius><length>2</length></cylinder></geometry></visual>
    <collision name="c"><geometry><box><size>1 1 2.4</size></box></geometry></collision>
  </link></model>'

MESH_NO_COLLISION='  <model name="w"><link name="statue">
    <visual name="v"><geometry><mesh><uri>model://elsewhere/thing.dae</uri></mesh></geometry></visual>
    <collision name="c"><geometry><box><size>1 1 1</size></box></geometry></collision>
  </link></model>'

ROTATED_BOX='  <model name="w"><link name="wall">
    <visual name="v"><pose>0 0 1.5 0 0 0.7854</pose><geometry><box><size>4 0.2 3</size></box></geometry></visual>
    <collision name="c"><geometry><box><size>4 0.2 3</size></box></geometry></collision>
  </link></model>'

echo "=== selftest: sdf_invariants.py ==="
run_case "1 an honest wall passes (shield is not amnesty)"          0 "$HONEST_WALL"
run_case "2 a visual taller than its collision is caught"           1 "$LYING_WALL"
run_case "3 a wall with no collision at all is caught"              1 "$GHOST_WALL"
run_case "4 a flat road over a ground plane is exempt"              0 "$FLAT_ROAD"
run_case "5 a collision LARGER than its visual is NOT flagged"      0 "$GENEROUS_COLLISION"
run_case "6 an unreadable mesh fails, it does not pass"             1 "$MESH_NO_COLLISION"
run_case "7 a freely-rotated box fails, it is not guessed at"       1 "$ROTATED_BOX"
run_case "8 a raised road is caught (above the decal ceiling)"      1 "$RAISED_ROAD"
run_case "9 a road with NO ground plane loses the exemption"        1 "$FLAT_ROAD" omit

PKG=$(dirname "$CHECKER")/..

run_pin_case() {   # run_pin_case <name> <want-exit> <mutator run inside the package copy>
  local name=$1 want=$2 mutate=$3 tmp got out
  tmp=$(mktemp -d)
  cp -r "$PKG" "$tmp/uav_sim_gz"
  ( cd "$tmp/uav_sim_gz" && eval "$mutate" ) > /dev/null 2>&1
  out=$(cd "$tmp" && python3 -m pytest uav_sim_gz/test/test_sdf_invariants.py         -k structural_exemptions -q 2>&1)
  got=$?; [ "$got" -ne 0 ] && got=1
  if [ "$got" -eq "$want" ]; then
    echo "  ok    $name (exit $got as expected)"
    if [ "$want" -eq 1 ]; then
      echo "$out" | grep -oE '(newly exempted|newly outside): \[[^]]*\] -- gone: \[[^]]*\]'         | sed 's/^/          /'
    fi
  else
    echo "  FAIL  $name: expected exit $want, got $got"
    echo "$out" | sed 's/^/          /' | head -12
    fails=$((fails + 1))
  fi
  rm -rf "$tmp"
}

BOGUS_SENSOR='<?xml version="1.0"?><sdf version="1.9"><model name="sensor_bogus">''<link name="sensor_bogus/base_link"><visual name="v"><pose>0 0 1.5 0 0 0</pose>''<geometry><box><size>1 1 3</size></box></geometry></visual></link></model></sdf>'

PAYLOAD='<link name="payload"><visual name="v"><pose>0 0 0.4 0 0 0</pose>''<geometry><box><size>0.3 0.3 0.3</size></box></geometry></visual></link>'

EXTERNAL_INCLUDE='<include><uri>model://not_here_at_all</uri></include>'

insert_before_close() {   # insert_before_close <model.sdf> <fragment>
  python3 -c 'import sys
p, frag = sys.argv[1], sys.argv[2]
s = open(p).read()
i = s.rindex("</model>")
open(p, "w").write(s[:i] + frag + s[i:])' "$1" "$2"
}

echo
echo "=== selftest: the exemption pin names what changed ==="
run_pin_case "10 the shipped package passes (shield is not amnesty)"  0 "true"
run_pin_case "11 a NEW sensor housing enters the exempt set"          1   "mkdir -p models/sensor_bogus && printf '%s' '$BOGUS_SENSOR' > models/sensor_bogus/model.sdf"
run_pin_case "12 a blessed sensor housing disappears"                 1   "rm -rf models/sensor_lidar_down"
run_pin_case "13 a payload bolted on the airframe has no collision"   1   "insert_before_close models/uav0_full/model.sdf '$PAYLOAD'"
run_pin_case "14 a NEW external model:// include appears"             1   "insert_before_close models/uav0_frame/model.sdf '$EXTERNAL_INCLUDE'"

echo
if [ "$fails" -eq 0 ]; then
  echo "SELFTEST PASS (14/14)"
  exit 0
fi
echo "SELFTEST FAIL ($fails case(s))"
exit 1
