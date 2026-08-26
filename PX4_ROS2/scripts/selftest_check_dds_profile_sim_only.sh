#!/bin/bash
# Positive control for scripts/check_dds_profile_sim_only.sh.
#
# The check exists to stop the large-samples DDS profile from reaching real-aircraft
# code by inheritance. A guard nobody has seen bite is a guard nobody knows works, and
# this one guards a flight-safety decision, so it gets a control that makes it bite on
# demand -- and a control that proves it is not simply refusing everything.
#
# It works on a scratch COPY of the repo tree, never the real one: a check that had to
# be tested by temporarily creating a real.launch.py in the working tree would be one
# interrupted session away from leaving that file behind.
#
# Cases:
#   1. the tree as it stands must PASS                  (the guard is not an obstacle)
#   2. a real.launch.py that loads the profile          -> must FAIL, named as real-side
#   3. an unlisted sim-side file that loads the profile -> must FAIL as a violation
#   4. an allowlist entry for a file that stopped using it -> must FAIL as stale
#
# Usage: bash scripts/selftest_check_dds_profile_sim_only.sh
set -o pipefail

CANON=$(cd "$(dirname "$0")/.." && pwd)
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
fails=0

# A scratch tree with only what the check reads.
seed() {
  rm -rf "$TMP/repo"
  mkdir -p "$TMP/repo/scripts" "$TMP/repo/src/uav_bringup/launch" "$TMP/repo/src/uav_bringup/config"
  cp "$CANON/scripts/check_dds_profile_sim_only.sh" "$TMP/repo/scripts/"
  cp "$CANON/scripts/dds_profile_sim_only_allowlist.txt" "$TMP/repo/scripts/"
  # Must mirror every path in the allowlist: a listed file missing from the scratch tree
  # reads as a STALE entry and fails case 1 for a reason that has nothing to do with the
  # thing under test. (Caught when check_sim_real_parity.py was added, 2026-08-25.)
  for f in start_sim.sh gate_r0_dds_profile.sh run_no10_closeout.sh verify_no10_wiring.sh \
           diagnose_ros2cli_under_profile.sh run_dds_transport_trial.sh measure_image_budget.sh \
           check_sim_real_parity.py selftest_check_dds_profile_sim_only.sh; do
    [ -f "$CANON/scripts/$f" ] && cp "$CANON/scripts/$f" "$TMP/repo/scripts/"
  done
  cp "$CANON/src/uav_bringup/launch/sim.launch.py" "$TMP/repo/src/uav_bringup/launch/"
  cp "$CANON/src/uav_bringup/config/fastdds_large_samples.xml" "$TMP/repo/src/uav_bringup/config/"
}

run_check() {
  ( cd "$TMP/repo" && bash scripts/check_dds_profile_sim_only.sh ) > "$TMP/out.txt" 2>&1
  echo $?
}

expect() {
  local name=$1 want=$2 needle=$3
  local got
  got=$(run_check)
  if [ "$got" != "$want" ]; then
    echo "  FAIL  $name: expected exit $want, got $got"
    sed 's/^/          /' "$TMP/out.txt" | tail -12
    fails=$((fails + 1))
    return
  fi
  if [ -n "$needle" ] && ! grep -q "$needle" "$TMP/out.txt"; then
    echo "  FAIL  $name: exit $got was right but the report never said '$needle'"
    sed 's/^/          /' "$TMP/out.txt" | tail -12
    fails=$((fails + 1))
    return
  fi
  echo "  ok    $name (exit $got)"
}

echo "=== check_dds_profile_sim_only.sh positive control ==="

# 1. The tree as it stands. If this fails, every other case below is meaningless.
seed
expect "the tree as it stands passes" 0 "still sim-only"

# 2. The named trap: real.launch.py copied from sim.launch.py, profile and all.
seed
cp "$TMP/repo/src/uav_bringup/launch/sim.launch.py" "$TMP/repo/src/uav_bringup/launch/real.launch.py"
expect "real.launch.py loading the profile is caught" 1 "REAL-SIDE REFERENCE"

# 3. An ordinary sim-side file that nobody declared.
seed
printf 'export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_large_samples.xml\n' \
  > "$TMP/repo/scripts/some_new_helper.sh"
expect "an undeclared file is a violation" 1 "VIOLATION"

# 4. Allowlist rot: an entry whose file no longer references the profile.
seed
printf 'echo nothing here\n' > "$TMP/repo/scripts/run_dds_transport_trial.sh"
expect "a stale allowlist entry is caught" 1 "STALE ALLOWLIST ENTRY"

echo
if [ "$fails" -ne 0 ]; then
  echo "RESULT: $fails case(s) wrong -- the check does not behave as documented."
  exit 1
fi
echo "RESULT: 4/4 -- the guard bites on all three failure shapes and stays out of the way otherwise."
