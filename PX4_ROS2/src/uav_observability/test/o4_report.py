#!/usr/bin/env python3
"""P10.9a G-O4 report/verdict helper for scripts/verify_observability.sh's o4-gate round.

Consumes JSONL produced by o4_system_health_sampler.py -- one line per REAL
/state/system_health publish (R21: event-driven off the wire, not a fixed poll
interval, so the [b] transition timing below is a real latency measurement, not an
artifact of how often this script happened to look).

Exit codes match verify_observability.sh's convention: 0=PASS 1=FAIL 2=FAILED TO
MEASURE (R27-1: a round with zero usable samples is not a silent PASS).
"""
import argparse
import json
import os
import sys
import time

# N7 (P10-gate-debt review round 2, 2026-08-24): shared with
# scripts/preflight_light.py (N8) so the two readers of
# /state/system_health cannot silently diverge on what "fresh" means.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..',
                                 'scripts'))
from gate_freshness import is_fresh, min_expected_samples  # noqa: E402


def load(path):
    records = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            records.append(json.loads(line))
    return records


def fnum(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return float('nan')


def cmd_go(args):
    records = load(args.jsonl)
    if not records:
        print('FAILED TO MEASURE [%s]: %s has zero samples' % (args.label, args.jsonl))
        return 2

    # N7 (P10-gate-debt review round 2, 2026-08-24): records[-1] alone was
    # never enough -- /state/system_health is TransientLocal, so a sampler
    # that only ever caught a stuck node's ONE latched sample still has a
    # non-empty `records` list. A stalled /clock freezes diagnostics_node's
    # eval timer AND publish timer TOGETHER (they are driven off the same
    # clock), so a genuinely stuck node produces far FEWER samples over
    # --duration-sec than a healthy one, not just a stale last one -- check
    # sample COUNT before trusting anything about the content of the last one.
    min_samples = min_expected_samples(args.duration_sec, args.publish_period_sec)
    if len(records) < min_samples:
        print('FAILED TO MEASURE [%s]: only %d sample(s) over %.1fs (need >=%d at %.1fs '
              'cadence) -- a stalled /clock freezes diagnostics_node\'s eval+publish timers '
              'TOGETHER, not just the last sample (N7)' %
              (args.label, len(records), args.duration_sec, min_samples,
               args.publish_period_sec))
        return 2

    last = records[-1]

    # Then check the LAST sample is actually fresh, not just latched -- same
    # is_fresh() as preflight_light.py's own age check (N8: fails closed on
    # NaN/negative too), same underlying reason: TransientLocal means
    # "received" and "current" are different questions.
    max_age_sec = args.max_age_sec if args.max_age_sec is not None else 2.0 * args.publish_period_sec
    age_sec = time.time() - fnum(last.get('wall_stamp_sec'))
    if not is_fresh(age_sec, max_age_sec):
        print('FAILED TO MEASURE [%s]: last sample age=%.3fs (need 0<=age<=%.1fs) -- '
              'TransientLocal QoS means a dead/frozen node\'s last go_no_go=%r stays latched '
              'forever; this sample is not trustworthy (N7)' %
              (args.label, age_sec, max_age_sec, last.get('go_no_go')))
        return 2

    want = args.want
    ok = last.get('go_no_go') == want
    # P10.9b (D4, S:6's "(c) SỬA KỲ VỌNG, CẤM NỚI"): perception:=true is now
    # EXPECTED NO_GO (nợ #10, camera bridge drops frames) -- worst_item must
    # actually point at diagnostics/perception, not just "some NO_GO reason
    # or other" (a script that only checked go_no_go could pass on a
    # completely unrelated fault and never notice this one closed).
    if ok and args.worst_contains:
        ok = args.worst_contains in (last.get('worst_item') or '')
    print('[%s] samples=%d last: go_no_go=%s unknown_count=%s error_count=%s stale_count=%s '
          'worst_item=%r gate_mode=%s gate_mode_source=%s waived_count=%s watch_failed=%s' %
          (args.label, len(records), last.get('go_no_go'), last.get('unknown_count'),
           last.get('error_count'), last.get('stale_count'), last.get('worst_item'),
           last.get('gate_mode'), last.get('gate_mode_source'), last.get('waived_count'),
           last.get('watch_failed')))
    print('%s [%s]: expected go_no_go=%s%s, got go_no_go=%s worst_item=%r' %
          ('PASS' if ok else 'FAIL', args.label, want,
           (' with worst_item containing %r' % args.worst_contains) if args.worst_contains else '',
           last.get('go_no_go'), last.get('worst_item')))
    return 0 if ok else 1


def cmd_ab(args):
    records = load(args.jsonl)
    if not records:
        print('FAILED TO MEASURE [a/b]: %s has zero samples' % args.jsonl)
        return 2
    with open(args.kill_time_file) as f:
        t_kill = float(f.read().strip())

    before = [r for r in records if fnum(r.get('t_wall')) < t_kill]
    after = [r for r in records if fnum(r.get('t_wall')) >= t_kill]

    verdict = 0
    baseline_was_go = False

    # N7, same reasoning as cmd_go above: a baseline built from too FEW
    # samples (stalled clock) or a stale last sample (TransientLocal latch)
    # is not trustworthy either -- check both before reading go_no_go off it.
    min_samples_before = min_expected_samples(args.settle_sec, args.publish_period_sec)
    if not before:
        print('FAILED TO MEASURE [a]: no samples arrived before t_kill=%.3f' % t_kill)
        verdict = 2
    elif len(before) < min_samples_before:
        print('FAILED TO MEASURE [a]: only %d sample(s) before t_kill (need >=%d over '
              'settle=%.1fs at %.1fs cadence) -- N7' %
              (len(before), min_samples_before, args.settle_sec, args.publish_period_sec))
        verdict = 2
    else:
        baseline = before[-1]
        max_age_sec = (args.max_age_sec if args.max_age_sec is not None
                        else 2.0 * args.publish_period_sec)
        # Anchor freshness AT t_kill, not "now" (report time): the report
        # runs after O4_KILL_WAIT_SEC of deliberate post-kill waiting, so
        # comparing against "now" would spuriously stale-out every baseline.
        age_sec = t_kill - fnum(baseline.get('wall_stamp_sec'))
        if not is_fresh(age_sec, max_age_sec):
            print('FAILED TO MEASURE [a]: baseline sample age at t_kill=%.3fs (need '
                  '0<=age<=%.1fs) -- not trustworthy (N7)' % (age_sec, max_age_sec))
            verdict = 2
        else:
            a_ok = baseline.get('go_no_go') == 'GO'
            baseline_was_go = a_ok
            print('[a] baseline (last sample before kill): go_no_go=%s unknown_count=%s '
                  'error_count=%s stale_count=%s worst_item=%r' %
                  (baseline.get('go_no_go'), baseline.get('unknown_count'),
                   baseline.get('error_count'), baseline.get('stale_count'),
                   baseline.get('worst_item')))
            print('%s [a]: expected go_no_go=GO before kill' % ('PASS' if a_ok else 'FAIL'))
            if not a_ok:
                verdict = max(verdict, 1)

    if not after:
        print('FAILED TO MEASURE [b]: zero samples arrived after t_kill=%.3f '
              '(node may have died silently, or discovery lag ate the whole window)' % t_kill)
        return max(verdict, 2)

    if not baseline_was_go:
        # [b] tests a GO->NO_GO TRANSITION caused by the kill -- if the
        # system was already NO_GO before the kill (a's own failure above),
        # there is nothing left to attribute to the kill: any NO_GO sample
        # after t_kill is indistinguishable from the pre-existing condition
        # that already broke [a]. Not a valid measurement either way (R27-1).
        print('FAILED TO MEASURE [b]: baseline was not GO (see [a] above) -- a GO->NO_GO '
              'transition cannot be attributed to the kill when it was already NO_GO before it')
        return max(verdict, 2)

    no_go_after = [r for r in after if r.get('go_no_go') == 'NO_GO']
    if not no_go_after:
        print('FAIL [b]: go_no_go never reached NO_GO within the sampling window after kill '
              '(last seen=%s, %d sample(s) after kill)' % (after[-1].get('go_no_go'), len(after)))
        return max(verdict, 1)

    first_no_go = no_go_after[0]
    elapsed = fnum(first_no_go['t_wall']) - t_kill
    allowed_worst = set(args.allowed_worst.split(',')) if args.allowed_worst else None
    worst = first_no_go.get('worst_item')
    worst_ok = allowed_worst is None or worst in allowed_worst

    b_ok = elapsed <= args.deadline_sec and worst_ok
    print('[b] transition: t_kill=%.3f first_NO_GO_wall=%.3f elapsed=%.3fs (deadline=%.1fs) '
          'worst_item=%r (allowed=%s) unknown_count=%s stale_count=%s' %
          (t_kill, first_no_go['t_wall'], elapsed, args.deadline_sec, worst, allowed_worst,
           first_no_go.get('unknown_count'), first_no_go.get('stale_count')))
    print('%s [b]: elapsed<=deadline=%s worst_item_ok=%s' %
          ('PASS' if b_ok else 'FAIL', elapsed <= args.deadline_sec, worst_ok))
    if not b_ok:
        verdict = max(verdict, 1)
    return verdict


def cmd_monotonic(args):
    # Each --jsonl file is its OWN sim boot (a fresh /clock origin) -- G-O4's
    # (a)/(b), (c), (d) are 3 SEPARATE bringups. Monotonicity only means
    # anything WITHIN one continuous node lifetime; concatenating files and
    # checking across the boundary between two unrelated sim clocks would
    # manufacture a fake "violation" out of nothing but two runs whose sim-
    # time-since-boot ranges happen to overlap (found live running this
    # gate: exactly 1 non-increasing step, at the (a)/(b)->(c) boundary).
    total_valid = 0
    total_stamp_viol = 0
    total_wall_viol = 0
    any_file_ok = False
    for path in args.jsonl:
        records = load(path)
        pairs = [(fnum(r.get('stamp_sec')), fnum(r.get('wall_stamp_sec'))) for r in records]
        # R30/C6: stamp_sec/wall_stamp_sec are NaN before the first real
        # evaluation -- drop those, they are "not yet measured", not violations.
        pairs = [(s, w) for s, w in pairs if s == s and w == w]
        total_valid += len(pairs)
        if len(pairs) < 2:
            print('[e] %s: only %d valid sample(s) -- too few to check monotonicity on its own'
                  % (path, len(pairs)))
            continue
        stamp_viol = sum(1 for i in range(1, len(pairs)) if pairs[i][0] <= pairs[i - 1][0])
        wall_viol = sum(1 for i in range(1, len(pairs)) if pairs[i][1] <= pairs[i - 1][1])
        total_stamp_viol += stamp_viol
        total_wall_viol += wall_viol
        any_file_ok = True
        print('[e] %s: valid_samples=%d stamp_sec=[%.3f..%.3f] wall_stamp_sec=[%.3f..%.3f] '
              'stamp_sec non-increasing steps=%d wall_stamp_sec non-increasing steps=%d' %
              (path, len(pairs), pairs[0][0], pairs[-1][0], pairs[0][1], pairs[-1][1],
               stamp_viol, wall_viol))

    if not any_file_ok or total_valid < args.min_samples:
        print('FAILED TO MEASURE [e]: only %d valid (stamp_sec,wall_stamp_sec) sample(s) total '
              'across %s (need >=%d, and >=2 per file to check monotonicity)' %
              (total_valid, args.jsonl, args.min_samples))
        return 2
    ok = total_stamp_viol == 0 and total_wall_viol == 0
    print('%s [e]: stamp_sec and wall_stamp_sec both present and strictly increasing WITHIN '
          'each bringup (total violations: stamp=%d wall=%d)' %
          ('PASS' if ok else 'FAIL', total_stamp_viol, total_wall_viol))
    return 0 if ok else 1


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='cmd', required=True)

    p_go = sub.add_parser('go', help='assert the LAST sample in a JSONL file matches --want')
    p_go.add_argument('--jsonl', required=True)
    p_go.add_argument('--label', required=True)
    p_go.add_argument('--want', default='GO', choices=['GO', 'NO_GO', 'UNKNOWN'])
    p_go.add_argument(
        '--worst-contains', default=None,
        help='when set, ALSO require this substring to appear in worst_item')
    p_go.add_argument(
        '--duration-sec', type=float, required=True,
        help='N7: the --duration passed to o4_system_health_sampler.py for this JSONL, used '
             'to derive the minimum expected sample count (a stalled /clock produces far too '
             'few samples, not just a stale last one)')
    p_go.add_argument(
        '--publish-period-sec', type=float, default=1.0,
        help='N7/N8: diagnostics_node publish_period_sec -- default matches '
             'config/observability_params.yaml DEFAULT (1.0s); pass explicitly if a bringup '
             'overrides it')
    p_go.add_argument(
        '--max-age-sec', type=float, default=None,
        help='N7: override freshness threshold for the last sample; default 2x '
             '--publish-period-sec (R32)')
    p_go.set_defaults(func=cmd_go)

    p_ab = sub.add_parser('ab', help='assert GO before t_kill, NO_GO within deadline after')
    p_ab.add_argument('--jsonl', required=True)
    p_ab.add_argument('--kill-time-file', required=True)
    p_ab.add_argument('--allowed-worst', default=None)
    p_ab.add_argument('--deadline-sec', type=float, default=2.0)
    p_ab.add_argument(
        '--settle-sec', type=float, required=True,
        help='N7: elapsed time the caller waited before reading the [a] baseline (e.g. '
             "verify_observability.sh's O4_SETTLE_SEC) -- used to derive the minimum expected "
             'sample count before t_kill')
    p_ab.add_argument(
        '--publish-period-sec', type=float, default=1.0,
        help='N7/N8: diagnostics_node publish_period_sec -- default matches '
             'config/observability_params.yaml DEFAULT (1.0s); pass explicitly if a bringup '
             'overrides it')
    p_ab.add_argument(
        '--max-age-sec', type=float, default=None,
        help='N7: override freshness threshold for the [a] baseline; default 2x '
             '--publish-period-sec (R32)')
    p_ab.set_defaults(func=cmd_ab)

    p_mono = sub.add_parser('monotonic', help='assert stamp_sec/wall_stamp_sec both increase')
    p_mono.add_argument('--jsonl', nargs='+', required=True)
    p_mono.add_argument('--min-samples', type=int, default=8)
    p_mono.set_defaults(func=cmd_monotonic)

    args = parser.parse_args()
    sys.exit(args.func(args))


if __name__ == '__main__':
    main()
