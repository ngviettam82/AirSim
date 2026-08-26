#!/bin/bash
# G-SIM -- the gate that decides whether the simulation phase may be called closed.
#
# WHAT THIS IS FOR (P12, 2026-08-25). The project owner asked for the simulation work to
# be finished and stamped, so that from then on only hardware research remains. A stamp
# like that is worth exactly as much as the thing that can refuse it. So the claim was
# turned into a script first, before any of the work: it is red today, and each task in
# P12 is "make one line of this go green". Progress reads as N/13, not as a feeling.
#
# 🔴 WHAT A GREEN G-SIM DOES NOT MEAN. These 13 lines measure STRUCTURE -- nodes exist,
# package boundaries hold, documents resolve, tests pass. They barely measure BEHAVIOUR
# on the life-saving path. A fully green board is still consistent with an aircraft that
# flies blind. The sentence a green board earns is "the software scope inside simulation
# is closed", never "simulation is complete" and never "this is safe". See
# .claude/plan/P12-sim-closeout.md section 1b, and docs/sim-boundary-statement.md when
# S12 lands.
#
# THE STANDARD EVERY LINE MUST MEET (G-SIM.0): a line counts as existing only once a
# mutation of its own shape has been watched turning it red. Five gates in this project
# were found on 2026-08-25 to have no such proof, and three of them could not fail at
# all. Each line below names the positive control that earns it.
#
# NOT RUN IS NOT PASS. A line with no evidence is red, never green. That is O3, and it
# is the failure mode this project has paid for twice (peakAcceleration reading 0.897
# for a real 55.7; counting republished messages instead of events, off by 43x).
#
# THE FREEZE RULE. Evidence older than the newest source file is evidence about code
# that no longer exists. Until the repo is under version control (question Q1 in the
# plan -- it is not a git repo today), the newest mtime across src/ and scripts/ stands
# in for "the commit under test". Crude, but it refuses the real failure it is aimed at:
# a board assembled from measurements taken on twenty different days against twenty
# different trees.
#
# Usage: bash scripts/gate_sim_closeout.sh
set -o pipefail

CANON=/mnt/c/code/PX4_ROS2
WORKSPACE=$HOME/PX4_ROS2
EVIDENCE=$WORKSPACE/gate_logs/sim_closeout
mkdir -p "$EVIDENCE"

# S6 imports the launch files straight out of the canonical tree, so python drops
# __pycache__ next to them -- and S12's `diff -rq` then reports differences the gate
# itself created. Ten of those false lines showed up on 2026-08-25; noise like that is
# how a fingerprint check stops being read.
export PYTHONDONTWRITEBYTECODE=1

green=0
red=0
declare -a BOARD

newest_source_epoch() {
  # THIS FILE IS EXCLUDED, and only this file. A scoreboard cannot change what was flown,
  # so letting it age the evidence it reads is strictness with no safety in it: on
  # 2026-08-25 a one-line fix to the S6 invocation turned a just-completed M5 3/3 stale
  # and would have cost another ten-minute flight to say the same thing. That is how a
  # gate teaches people to route around it.
  #
  # Everything else under scripts/ STAYS IN, deliberately -- run_m5_regression.sh,
  # start_sim.sh and the probes are part of how a flight was produced, so touching them
  # really does invalidate its result.
  find "$CANON/src" "$CANON/scripts" -type f \
    ! -path "$CANON/scripts/gate_sim_closeout.sh" \
    \( -name '*.py' -o -name '*.cpp' -o -name '*.hpp' -o -name '*.sh' \
       -o -name '*.xml' -o -name '*.yaml' -o -name '*.txt' -o -name '*.sdf' \) \
    -printf '%T@\n' 2>/dev/null | sort -rn | head -1 | cut -d. -f1
}

SOURCE_EPOCH=$(newest_source_epoch)

# S1's own reference. The suite is built from src/ and judged by exactly two scripts, so
# those are what can invalidate a run of it. Using the global reference here meant a new,
# unrelated file under scripts/ reset a 35-minute three-run campaign -- strictness with no
# safety in it, and the kind that teaches people to route around the gate.
suite_epoch() {
  {
    find "$CANON/src" -type f \
      \( -name '*.py' -o -name '*.cpp' -o -name '*.hpp' -o -name '*.sh' \
         -o -name '*.xml' -o -name '*.yaml' -o -name '*.txt' -o -name '*.sdf' \) \
      -printf '%T@\n' 2>/dev/null
    stat -c '%Y' "$CANON/scripts/verify_workspace.sh" "$CANON/scripts/workspace_verdict.py" \
      2>/dev/null
  } | sort -rn | head -1 | cut -d. -f1
}

SUITE_EPOCH=$(suite_epoch)

pass() { BOARD+=("  🟢 $1  $2"); green=$((green + 1)); }
fail() { BOARD+=("  🔴 $1  $2"); red=$((red + 1)); }

# live <id> <label> <command...> -- run a fast check now.
live() {
  local id=$1 label=$2
  shift 2
  if "$@" > "$EVIDENCE/$id.log" 2>&1; then
    pass "$id" "$label"
  else
    fail "$id" "$label  [see $EVIDENCE/$id.log]"
  fi
}

# recorded <id> <label> <runner> -- a check too slow to run here. Its runner must leave
# an evidence file; missing or older than the source tree is red.
recorded() {
  local id=$1 label=$2 runner=$3
  local file=$EVIDENCE/$id.ok
  if [ ! -f "$file" ]; then
    fail "$id" "$label  [NOT RUN -- $runner]"
    return
  fi
  local stamp
  stamp=$(stat -c %Y "$file")
  if [ "$stamp" -lt "$SOURCE_EPOCH" ]; then
    fail "$id" "$label  [STALE: evidence predates the source tree -- rerun $runner]"
    return
  fi
  pass "$id" "$label  [$(head -1 "$file")]"
}

# consecutive_clean <id> <label> -- S1 wants three clean runs in a row, not one. Reads the
# line-per-run history verify_workspace.sh appends. Distinguishes "never run" from "ran
# and failed": both are red, but telling someone a gate was never measured when it was
# measured and refused sends them to the wrong place.
consecutive_clean() {
  local id=$1 label=$2
  local history=$HOME/PX4_ROS2/gate_logs/sim_closeout/S1.history
  if [ ! -f "$history" ]; then
    fail "$id" "$label  [NEVER RUN -- bash scripts/verify_workspace.sh]"
    return
  fi
  local recent
  recent=$(tail -3 "$history")
  local runs clean oldest
  runs=$(printf '%s\n' "$recent" | grep -c .)
  clean=$(printf '%s\n' "$recent" | grep -c ' PASS ')
  oldest=$(printf '%s\n' "$recent" | head -1 | cut -d' ' -f1)
  if [ "$runs" -lt 3 ]; then
    fail "$id" "$label  [only $runs run(s) recorded, need 3]"
  elif [ "$clean" -ne 3 ]; then
    fail "$id" "$label  [$clean/3 clean -- last: $(tail -1 "$history" | cut -d' ' -f2-)]"
  elif [ "$oldest" -lt "$SUITE_EPOCH" ]; then
    fail "$id" \
      "$label  [3 clean runs, but the oldest predates src/ or the verdict tooling]"
  else
    pass "$id" "$label  [3/3 clean, $(tail -1 "$history" | cut -d' ' -f3-)]"
  fi
}

# --- S14: the boundary statement must exist, and must be signed --------------------
# Plan P12 section 6 lists this as a condition of being finished, but nothing checked it
# -- the gate mentioned the file only in a comment. A closeout criterion no line enforces
# is a criterion that gets remembered at signing time, which is the worst moment to find
# out it was never written.
#
# Being signed is part of the check on purpose. The document's own job is to stop "the
# simulation is done" being read as "the aircraft is safe", and an unsigned document has
# not been read by the person who would say that sentence.
check_boundary_statement() {
  local doc=$CANON/docs/sim-boundary-statement.md
  if [ ! -f "$doc" ]; then
    echo "docs/sim-boundary-statement.md does not exist (P12.7)"
    return 1
  fi
  # Scoped to the signature section, and matching the GLYPH rather than the whole cell.
  # The first version matched the literal '| ⬜ |', so a box reading '| ⬜ CHƯA CÓ |' passed
  # it -- annotating a box would have turned the gate off. Counting file-wide is wrong the
  # other way: sections 2 and 4 discuss the preflight checklist's own empty boxes in prose.
  local unsigned
  unsigned=$(sed -n '/^## §5\. Chữ ký/,$p' "$doc" | grep -c '⬜')
  echo "boundary statement present, $(grep -c '^| \*\*B-' "$doc") list-B entries"
  if grep -q '^# .*⬜' "$doc"; then
    echo "  the title still says unsigned"
    return 1
  fi
  if [ "$unsigned" -ne 0 ]; then
    echo "  $unsigned signature box(es) still empty -- unsigned means unread"
    return 1
  fi
  return 0
}

# --- S2: every package must be able to fail ------------------------------------------
check_all_packages_have_tests() {
  local missing=()
  for dir in "$CANON"/src/*/; do
    local pkg
    pkg=$(basename "$dir")
    if ! ls "$WORKSPACE/build/$pkg/test_results/$pkg"/*.xml > /dev/null 2>&1; then
      missing+=("$pkg")
    fi
  done
  if [ ${#missing[@]} -eq 0 ]; then
    echo "all packages have ctest results"
    return 0
  fi
  echo "packages with no ctest target: ${missing[*]}"
  return 1
}

# --- S12: the measurement must be about the tree under test --------------------------
check_source_fingerprint() {
  local rc=0
  if diff -rq --exclude=__pycache__ --exclude='*.pyc' \
       "$WORKSPACE/src" "$CANON/src" > /dev/null 2>&1; then
    echo "src trees identical (WSL <-> Windows)"
  else
    echo "SOURCE TREES DIVERGED -- the thing tested is not the thing edited:"
    diff -rq --exclude=__pycache__ --exclude='*.pyc' "$WORKSPACE/src" "$CANON/src" 2>&1 | head -10
    rc=1
  fi
  # The other half of S12 -- proving install/ was built from the tree under test --
  # needs version control. Blocked on Q1; it is not silently assumed to hold.
  echo "install/ provenance: NOT CHECKED (needs Q1: git init)"
  return $((rc + 1))
}

echo "=================================================================="
echo " G-SIM -- simulation closeout gate"
echo " freeze reference: newest source mtime = $(date -d "@$SOURCE_EPOCH" '+%F %T')"
echo "=================================================================="
echo

live     S2  "12/12 packages have a ctest target" check_all_packages_have_tests
live     S4  "px4_msgs boundary (R1)          [selftest_px4_msgs_boundary.sh 11/11]" \
             bash "$CANON/scripts/check_px4_msgs_boundary.sh"
live     S5  "DDS profile stays sim-only" \
             bash "$CANON/scripts/check_dds_profile_sim_only.sh"
# Through a login shell that sources ROS first. check_sim_real_parity.py BUILDS both
# LaunchDescriptions, so it needs `launch` and `launch_ros` importable -- run bare it died
# on `ModuleNotFoundError: No module named 'launch'` and this line had been red for that
# reason alone while the check itself passed by hand. A gate that fails for an environment
# reason is a gate people learn to ignore, which is worse than not having it.
live     S6  "sim/real parity (R7)            [selftest_sim_real_parity.sh 10/10]" \
             bash -c "source /opt/ros/humble/setup.bash \
                      && source \"$WORKSPACE/install/setup.bash\" \
                      && python3 \"$CANON/scripts/check_sim_real_parity.py\""
live     S11a "documentation links            [selftest_doc_links.sh 7/7]" \
             bash "$CANON/scripts/check_doc_links.sh"
live     S11b "code comment audit (R16)" bash "$CANON/scripts/audit_comments.sh"
# SKIP_SUITE=1 on purpose. That script's stage 3/3 is a full `colcon test` (~11 min),
# and starting a second ctest while another is live is exactly the collision that
# corrupted a measurement on 2026-08-24 (ops-playbook section 20). What runs here is the
# DECLARATION half -- every ROS-touching test target has a domain of its own. The other
# half, that those suites actually pass on those domains, is what S1 measures.
live     S11c "test domain isolation declared (R20)" \
             env SKIP_SUITE=1 bash "$CANON/scripts/check_test_domain_isolation.sh"
live     S12  "source fingerprint" check_source_fingerprint

consecutive_clean S1 \
  "workspace clean x3 consecutive [selftest_workspace_verdict.sh 14/14]"
recorded S3  "coverage measured and every uncovered line classified" "P12.3"
# Runs live rather than from a recorded file: it is pure XML parsing and finishes in
# about 0.2 s, so there is no reason to let evidence for an R0 invariant go stale.
live     S7  "collision covers visual (R0)   [selftest_sdf_invariants.sh 14/14]" \
             python3 "$CANON/src/uav_sim_gz/test/sdf_invariants.py"
recorded S8  "M5 regression 3/3 incl. /safety/state" "scripts/run_m5_regression.sh"
recorded S9  "freeze campaign: preflight C1-C9 + 3 rare-branch gates" "P12.6"
recorded S10 "P5 dynamic gates D1/D2/D3, or a signed waiver" "P12.5"
recorded S13 "a flight with require_obstacle_feed:=true and real obstacles" "P12.4"
live     S14 "boundary statement written and signed" check_boundary_statement

echo "--- board ---"
printf '%s\n' "${BOARD[@]}"
echo
total=$((green + red))
echo "G-SIM: $green/$total green"
echo
if [ "$red" -ne 0 ]; then
  echo "RESULT: NOT CLOSED. $red line(s) red."
  echo "        The simulation phase may not be called finished."
  exit 1
fi
echo "RESULT: every line green."
echo "        Claim allowed: 'the software scope inside simulation is closed'."
echo "        Claim NOT allowed: 'simulation is complete', or anything about safety."
exit 0
