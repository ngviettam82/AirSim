#!/bin/bash
# Positive control for gate_r0_verdict.py's non-finite guard.
#
# A guard nobody has seen bite is a guard nobody knows works. This feeds the verdict
# four hand-made evidence pairs whose right answers are known in advance, and checks
# it gives each one. Two of them are the fault the guard was added for on 2026-08-24:
# the arm script writes the literal "nan" when the RTF sampler returns nothing, and
# before the fix Python compared nan against every threshold as False, so a run with
# NO USABLE DATA printed PASS (R30 -- not-measurable silently becoming acceptable).
#
# The other half matters just as much: a shield must not become an amnesty. Case 1
# proves healthy evidence still passes, and case 4 proves a genuinely bad profile arm
# is still caught. Same shape as scripts/verify_ng_positive_control.sh.
#
# Usage: bash scripts/selftest_gate_r0_verdict.sh
set -o pipefail

CANON=/mnt/c/code/PX4_ROS2
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
fails=0

check() {
  local name=$1 want=$2 control=$3 profile=$4
  printf '%s\n' "$control" > "$TMP/c.txt"
  printf '%s\n' "$profile" > "$TMP/p.txt"
  python3 "$CANON/scripts/gate_r0_verdict.py" "$TMP/c.txt" "$TMP/p.txt" > "$TMP/out.txt" 2>&1
  local got=$?
  if [ "$got" -eq "$want" ]; then
    echo "  ok    $name (exit $got as expected)"
  else
    echo "  FAIL  $name: expected exit $want, got $got"
    sed 's/^/          /' "$TMP/out.txt"
    fails=$((fails + 1))
  fi
}

HEALTHY_C='rtf 0.761
vehicle_odometry 71.116 0.702 93.45
vehicle_status 1.409 1.378 1.85
sensor_combined 71.114 0.700 93.45'

HEALTHY_P='rtf 0.791
vehicle_odometry 70.836 0.682 89.55
vehicle_status 1.406 1.347 1.78
sensor_combined 70.906 0.678 89.64'

echo "=== gate_r0_verdict.py positive control ==="

# 1. The real 2026-08-24 evidence must still pass -- the guard is not an obstacle.
check "healthy evidence passes" 0 "$HEALTHY_C" "$HEALTHY_P"

# 2. The exact shape the guard exists for: RTF unmeasurable, so every normalised rate
#    is the literal nan. Before the fix this printed PASS.
NAN_P='rtf nan
vehicle_odometry 70.836 0.682 nan
vehicle_status 1.406 1.347 nan
sensor_combined 70.906 0.678 nan'
check "nan rates must NOT pass" 1 "$HEALTHY_C" "$NAN_P"

# 3. RTF missing while the rates happen to parse -- the rates are unanchored, because
#    each one is a division by that very RTF.
NORTF_P='rtf nan
vehicle_odometry 70.836 0.682 89.55
vehicle_status 1.406 1.347 1.78
sensor_combined 70.906 0.678 89.64'
check "missing RTF must NOT pass" 1 "$HEALTHY_C" "$NORTF_P"

# 4. Amnesty check: a profile arm delivering half the control's rate must still fail.
HALF_P='rtf 0.791
vehicle_odometry 70.836 0.682 46.00
vehicle_status 1.406 1.347 1.78
sensor_combined 70.906 0.678 89.64'
check "a genuinely degraded link still fails" 1 "$HEALTHY_C" "$HALF_P"

echo
if [ "$fails" -ne 0 ]; then
  echo "RESULT: $fails case(s) wrong -- the verdict does not behave as documented."
  exit 1
fi
echo "RESULT: 4/4 -- the guard bites on bad evidence and stays out of the way on good."
