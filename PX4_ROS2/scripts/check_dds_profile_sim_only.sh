#!/bin/bash
# The large-samples Fast DDS profile is SIM-ONLY until a P11 gate says otherwise.
# This turns that decision into an invariant a script checks, instead of a sentence
# in a document nobody re-reads -- the same move that closed the R1 debt.
#
# WHY THE DECISION IS "SIM-ONLY, PROVE IT AGAIN ON HARDWARE" (project owner, 2026-08-25).
# config/fastdds_large_samples.xml raises the Fast DDS shared-memory segment from the
# stock 549 408 B to 16 MiB so a 640x480 camera frame travels in one piece, and sets
# useBuiltinTransports=false. Gate R0 cleared it -- but cleared it ON THE SIMULATOR,
# against MicroXRCEAgent over the loopback of one WSL host.
#
# The awkward half of the truth: the 549 408 B ceiling belongs to the Fast DDS DEFAULT,
# not to the simulator, and a real camera driver publishes 640x480+ frames through that
# same library. So the real aircraft probably needs this too. That is an argument for
# TESTING it on hardware at P11.3, and specifically NOT an argument for letting it
# arrive there by inheritance: useBuiltinTransports=false replaces the transport of
# every participant that reads the profile, the flight-controller link included, and on
# real hardware that link crosses a physical interface instead of loopback. A profile
# that is measured on one transport substrate and shipped onto another is R31.
#
# So: no real-side file may reference this profile until P11.3 measures it there.
# When it does, add the file here WITH the gate result -- not before.
#
# Usage: bash scripts/check_dds_profile_sim_only.sh
# Docs:  .claude/plan/P11-real-flight-readiness.md S:3(a)
set -o pipefail

cd "$(dirname "$0")/.." || exit 1

# A reference is the profile filename or the Fast DDS env var that loads it. Matching
# the env var too is deliberate: a real-side file could point at a COPY under another
# name and inherit the same substrate problem without ever typing the filename.
REFERENCE='fastdds_large_samples|FASTRTPS_DEFAULT_PROFILES_FILE'

ALLOWED_FILE=scripts/dds_profile_sim_only_allowlist.txt

echo "=== files that reference the large-samples DDS profile ==="
# The allowlist is a REGISTRY of references, not a consumer of the profile: it names
# the filename on every line by construction, so scanning it would make the check
# report itself as a violation forever. Caught on this check's first run.
mapfile -t hits < <(
  grep -rElE "$REFERENCE" src scripts \
    --include='*.py' --include='*.cpp' --include='*.hpp' --include='*.sh' \
    --include='*.xml' --include='*.yaml' --include='*.txt' 2>/dev/null \
  | grep -vx "$ALLOWED_FILE" \
  | sort
)

allowed=0
violations=0
for hit in "${hits[@]}"; do
  [ -z "$hit" ] && continue
  if [ -f "$ALLOWED_FILE" ] && grep -q "^${hit}[[:space:]]" "$ALLOWED_FILE"; then
    reason=$(grep "^${hit}[[:space:]]" "$ALLOWED_FILE" | sed 's/^[^#]*# *//')
    echo "  ALLOWED   $hit"
    echo "            reason: $reason"
    allowed=$((allowed + 1))
  else
    echo "  VIOLATION $hit"
    grep -nE "$REFERENCE" "$hit" | head -3 | sed 's/^/            /'
    violations=$((violations + 1))
  fi
done
[ ${#hits[@]} -eq 0 ] && echo "  (none)"

# An allowlist entry pointing at a file that no longer references the profile is rot:
# it silently keeps permitting a path nobody re-reads.
stale=0
if [ -f "$ALLOWED_FILE" ]; then
  while read -r path _; do
    case "$path" in ''|'#'*) continue ;; esac
    if ! printf '%s\n' "${hits[@]}" | grep -qx "$path"; then
      echo "  STALE ALLOWLIST ENTRY: $path no longer references it -- remove the line"
      stale=$((stale + 1))
    fi
  done < "$ALLOWED_FILE"
fi

# The named trap. real.launch.py does not exist yet (P11.2 creates it), and the most
# likely way it acquires this profile is by being copied from sim.launch.py -- which
# carries dds_profile_actions(). Name the file explicitly so the check bites the day
# it appears, rather than depending on someone remembering this paragraph.
realside=0
for f in src/uav_bringup/launch/real.launch.py scripts/start_real.sh; do
  if [ -f "$f" ] && grep -qE "$REFERENCE" "$f"; then
    echo "  🔴 REAL-SIDE REFERENCE: $f"
    echo "     P11.3 must measure this profile on the target hardware BEFORE any"
    echo "     real-side file may load it. Gate R0's numbers were taken over WSL"
    echo "     loopback against MicroXRCEAgent and do not transfer (R31)."
    realside=$((realside + 1))
  fi
done

echo
echo "references: $((allowed + violations))  allowed: $allowed  violations: $violations  stale: $stale  real-side: $realside"
if [ "$violations" -ne 0 ] || [ "$stale" -ne 0 ] || [ "$realside" -ne 0 ]; then
  echo "RESULT: FAIL"
  exit 1
fi
echo "RESULT: PASS -- the profile is still sim-only."
