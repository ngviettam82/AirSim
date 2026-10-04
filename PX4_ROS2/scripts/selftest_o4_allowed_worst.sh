#!/bin/bash
# R27-3 positive/negative injection self-test for o4-gate mục (b)'s
# O4_ALLOWED_WORST (verify_observability.sh:~1583). Data-only -- no Gazebo/
# PX4/ROS graph needed, o4_report.py has zero rclpy dependency. Proves the
# widened allow-list (a) still bites on an unrelated-domain worst_item and
# (b) actually accepts the two real localization-domain values (the
# pre-existing Sub-A name and the new Sub-B race winner observed live
# 2026-08-24). Without this, "the criterion is fixed" is just a claim.
# Usage: bash scripts/selftest_o4_allowed_worst.sh
set -o pipefail

WORKSPACE_WIN=/mnt/c/code/PX4_ROS2
GATE_SCRIPT="$WORKSPACE_WIN/scripts/verify_observability.sh"
O4_REPORT="$WORKSPACE_WIN/src/uav_observability/test/o4_report.py"
TMPDIR=$(mktemp -d)

# Derive the allow-list straight off the real gate script instead of
# hand-copying it here -- R34 spirit, a stale hand-typed copy would test a
# different string than the one actually shipped.
O4_ALLOWED_WORST=$(grep -oP '^O4_ALLOWED_WORST="\K[^"]*' "$GATE_SCRIPT")
if [ -z "$O4_ALLOWED_WORST" ]; then
  echo "FATAL: could not derive O4_ALLOWED_WORST from $GATE_SCRIPT"
  exit 1
fi
echo "Derived O4_ALLOWED_WORST=[$O4_ALLOWED_WORST]"

SETTLE_SEC=25
PUBLISH_PERIOD_SEC=1.0

# make_fixture WORST_ITEM OUT_JSONL OUT_KILLTIME -- 14 GO samples spaced 1s
# apart ending 0.3s before t_kill (>= the 12-sample N7 precheck floor at
# this settle/period pair, comfortably inside the 2.0s freshness window),
# then ONE NO_GO sample 0.5s after t_kill carrying WORST_ITEM (well inside
# the 2.0s deadline, so elapsed<=deadline is never the thing under test --
# only worst_item membership is).
make_fixture() {
  local worst_item="$1" out_jsonl="$2" out_killtime="$3"
  python3 - "$worst_item" "$out_jsonl" "$out_killtime" "$SETTLE_SEC" "$PUBLISH_PERIOD_SEC" <<'PYEOF'
import json
import sys
import time

worst_item, out_jsonl, out_killtime, settle_sec, period_sec = sys.argv[1:6]
settle_sec = float(settle_sec)
period_sec = float(period_sec)

now = time.time()
t_kill = now + settle_sec

records = []
before_count = 14
last_before_age = 0.3
first_before_t = t_kill - last_before_age - (before_count - 1) * period_sec
for i in range(before_count):
    t = first_before_t + i * period_sec
    records.append({
        't_wall': t, 'go_no_go': 'GO', 'unknown_count': '0', 'error_count': '0',
        'warn_count': '0', 'stale_count': '0', 'worst_item': '', 'clock_regressions': '0',
        'gate_mode': 'preflight', 'gate_mode_source': 'measured', 'waived_count': '0',
        'waiver_unmatched': '0', 'blocking': '', 'watch_failed': 'false',
        'stamp_sec': str(t), 'wall_stamp_sec': str(t),
    })

t_after = t_kill + 0.5
records.append({
    't_wall': t_after, 'go_no_go': 'NO_GO', 'unknown_count': '0', 'error_count': '1',
    'warn_count': '0', 'stale_count': '1', 'worst_item': worst_item, 'clock_regressions': '0',
    'gate_mode': 'preflight', 'gate_mode_source': 'measured', 'waived_count': '0',
    'waiver_unmatched': '0', 'blocking': worst_item, 'watch_failed': 'false',
    'stamp_sec': str(t_after), 'wall_stamp_sec': str(t_after),
})

with open(out_jsonl, 'w') as f:
    for r in records:
        f.write(json.dumps(r) + '\n')
with open(out_killtime, 'w') as f:
    f.write('%.6f' % t_kill)
PYEOF
}

run_case() {
  local desc="$1" worst_item="$2" want_exit="$3"
  local jsonl="$TMPDIR/${4}.jsonl" killtime="$TMPDIR/${4}_kill.txt"
  make_fixture "$worst_item" "$jsonl" "$killtime"
  python3 "$O4_REPORT" ab --jsonl "$jsonl" --kill-time-file "$killtime" \
    --allowed-worst "$O4_ALLOWED_WORST" --deadline-sec 2.0 \
    --settle-sec "$SETTLE_SEC" --publish-period-sec "$PUBLISH_PERIOD_SEC"
  local got_exit=$?
  if [ "$got_exit" -eq "$want_exit" ]; then
    echo "SELFTEST OK: $desc (exit=$got_exit, wanted=$want_exit)"
    return 0
  else
    echo "SELFTEST FAIL: $desc -- exit=$got_exit, wanted=$want_exit"
    return 1
  fi
}

FAILURES=0

echo "=== negative injection: worst_item in an UNRELATED domain (task's own"
echo "    example, diagnostics/safety:*) must still FAIL -- proves the widened"
echo "    allow-list did not turn this check into a no-op (R27-3) ==="
run_case "unrelated-domain worst_item (diagnostics/safety:*) -> FAIL" \
  "diagnostics/safety:safety: OFFBOARD_UNHEALTHY" 1 neg_safety || FAILURES=$((FAILURES + 1))

echo
echo "=== positive: the REAL Sub-B value observed live 2026-08-24 must now PASS ==="
run_case "Sub-B localization content child (the actual race winner) -> PASS" \
  "diagnostics/localization:localization: fused output" 0 pos_subb || FAILURES=$((FAILURES + 1))

echo
echo "=== positive (regression guard): the pre-existing Sub-A name must still PASS ==="
run_case "Sub-A liveness name (state/odometry_fused) -> PASS" \
  "state/odometry_fused" 0 pos_suba || FAILURES=$((FAILURES + 1))

echo
echo "=== negative injection: a second unrelated domain (camera/perception)"
echo "    must still FAIL too, not just safety ==="
run_case "unrelated-domain worst_item (diagnostics/perception:*) -> FAIL" \
  "diagnostics/perception:camera: front/rgb" 1 neg_camera || FAILURES=$((FAILURES + 1))

rm -rf "$TMPDIR"
echo
if [ "$FAILURES" -gt 0 ]; then
  echo "SELFTEST RESULT: $FAILURES check(s) FAILED"
  exit 1
fi
echo "SELFTEST RESULT: all checks PASS"
exit 0
