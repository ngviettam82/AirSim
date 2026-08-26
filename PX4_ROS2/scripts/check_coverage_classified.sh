#!/bin/bash
# G-SIM S3: every uncovered line is CLASSIFIED. Never a percentage.
#
# WHY NO THRESHOLD (P12.3). A percentage averages the safety branch together with
# the boilerplate, so 90% with the cut chain uncovered reads as healthy -- the same
# shape as peakAcceleration() reporting 0.897 for a real 55.7 (CLAUDE.md section 5).
# This gate asks the only question worth asking: is there a line nobody has looked at?
#
# Reads:  gate_logs/coverage/uncovered_by_file.txt   (written by measure_coverage.sh)
#         config/coverage_classification.tsv          (the declarations)
# Writes: gate_logs/sim_closeout/S3.ok                (only when zero lines are unclassified)
#
# Exit 1 on: no measurement, a measurement older than src/, any unclassified line, any
# rule that matches nothing (a stale rule is how a classification silently stops covering
# what it claims to).
#
# WHY THE MEASUREMENT'S AGE IS CHECKED HERE (2026-08-26). G-SIM only ages S3.ok, so a
# fresh classification written over an old measurement produced a green line. It nearly
# happened: the run of 2026-08-26 01:14 listed 225 unclassified lines, and the 40 test
# cases that cover part of them were written at 04:41-04:59. Classifying against that
# list would have declared reasons for lines that are now tested -- and those rules would
# then match nothing, which is the very thing rule 3 below exists to catch.
set -o pipefail

CANON=${CANON:-/mnt/c/code/PX4_ROS2}
WORKSPACE=${WORKSPACE:-$HOME/PX4_ROS2}
UNCOVERED=$WORKSPACE/gate_logs/coverage/uncovered_by_file.txt
RULES=$CANON/config/coverage_classification.tsv
EVIDENCE=$WORKSPACE/gate_logs/sim_closeout
mkdir -p "$EVIDENCE"

if [ ! -f "$UNCOVERED" ]; then
  echo "NOT MEASURED: $UNCOVERED is missing -- run scripts/measure_coverage.sh first"
  exit 1
fi
if [ ! -f "$RULES" ]; then
  echo "NO DECLARATION: $RULES is missing"
  exit 1
fi

# The epoch the binaries were compiled from, written by measure_coverage.sh before it
# built. Falling back to the output file's own mtime is strictly weaker -- it cannot see
# an edit made while a 20-minute run was in flight -- so say so rather than passing quietly.
STAMP=$(dirname "$UNCOVERED")/source_epoch.txt
if [ -f "$STAMP" ]; then
  measured=$(cat "$STAMP")
else
  echo "NO SOURCE STAMP: $STAMP is missing -- this measurement predates the stamp and"
  echo "  cannot prove which source it was built from. Rerun scripts/measure_coverage.sh."
  exit 1
fi
newest_src=$(find "$CANON/src" -type f \( -name '*.cpp' -o -name '*.hpp' \)              -printf '%T@
' 2>/dev/null | sort -rn | head -1 | cut -d. -f1)
if [ -n "$newest_src" ] && [ "$measured" -lt "$newest_src" ]; then
  echo "STALE MEASUREMENT: $UNCOVERED is from $(date -d @"$measured" '+%F %T'),"
  echo "  but src/ changed at $(date -d @"$newest_src" '+%F %T')."
  echo "  Classifying against it would declare reasons for lines that may now be tested."
  echo "  Rerun scripts/measure_coverage.sh first."
  exit 1
fi

python3 - "$UNCOVERED" "$RULES" "$EVIDENCE/S3.ok" <<'PY'
import collections, io, sys

uncovered_path, rules_path, ok_path = sys.argv[1], sys.argv[2], sys.argv[3]

VALID = {"B-FLIGHT", "B-ENTRY", "B-DIAG", "A-TESTED", "D-BOUNDARY"}

rules = []
for lineno, raw in enumerate(io.open(rules_path, encoding="utf-8"), 1):
    line = raw.rstrip("\n")
    if not line.strip() or line.lstrip().startswith("#"):
        continue
    parts = line.split("\t")
    parts = [p for p in parts if p != ""]
    if len(parts) < 4:
        print("BAD RULE line %d: need CATEGORY<TAB>path<TAB>lines<TAB>evidence" % lineno)
        sys.exit(1)
    category, suffix, spans, evidence = parts[0].strip(), parts[1].strip(), parts[2].strip(), parts[3].strip()
    if category not in VALID:
        print("BAD RULE line %d: unknown category %r" % (lineno, category))
        sys.exit(1)
    if not evidence:
        print("BAD RULE line %d: evidence may not be empty" % lineno)
        sys.exit(1)
    ranges = []
    if spans != "*":
        for span in spans.split(","):
            a, _, b = span.partition("-")
            ranges.append((int(a), int(b or a)))
    rules.append({"cat": category, "suffix": suffix, "ranges": ranges,
                  "evidence": evidence, "line": lineno, "hits": 0})

lines = []
for raw in io.open(uncovered_path, encoding="utf-8"):
    raw = raw.strip()
    if not raw:
        continue
    path, _, num = raw.rpartition(":")
    lines.append((path, int(num)))

def matches(rule, path, num):
    if not path.endswith(rule["suffix"]):
        return False
    if not rule["ranges"]:
        return True
    return any(a <= num <= b for a, b in rule["ranges"])

tally = collections.Counter()
unclassified = []
for path, num in lines:
    for rule in rules:
        if matches(rule, path, num):
            rule["hits"] += 1
            tally[rule["cat"]] += 1
            break
    else:
        unclassified.append((path, num))

dead_rules = [r for r in rules if r["hits"] == 0]

print("=== S3: uncovered lines, classified ===")
print("  measured lines : %d" % len(lines))
for cat in sorted(tally):
    print("  %-12s : %d" % (cat, tally[cat]))
print("  unclassified   : %d" % len(unclassified))
print("  rules          : %d declared, %d matched nothing" % (len(rules), len(dead_rules)))

if dead_rules:
    print()
    print("STALE DECLARATIONS -- these claim to cover lines that no longer exist:")
    for r in dead_rules[:20]:
        print("  line %d: %s %s %s" % (r["line"], r["cat"], r["suffix"], r["evidence"][:60]))

if unclassified:
    print()
    print("UNCLASSIFIED -- nobody has looked at these:")
    shown = collections.Counter(p for p, _ in unclassified)
    for path, count in shown.most_common(15):
        print("  %4d  %s" % (count, path))
    if len(shown) > 15:
        print("  ... and %d more file(s)" % (len(shown) - 15))

if unclassified or dead_rules:
    print()
    print("RESULT: S3 NOT CLOSED")
    sys.exit(1)

summary = "%d uncovered lines, all classified: " % len(lines)
summary += " ".join("%s=%d" % (c, tally[c]) for c in sorted(tally))
io.open(ok_path, "w", encoding="utf-8").write(summary + "\n")
print()
print("RESULT: PASS -- " + summary)
PY
