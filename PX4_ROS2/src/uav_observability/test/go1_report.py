#!/usr/bin/env python3
"""G-O1 (P10.3) aggregator: reads scripts/verify_observability.sh's
go1_results.csv, prints the 3-config comparison table + the positive
control check, and picks PASS/FAIL/FAILED-TO-MEASURE per the gate's
criteria (task brief, R27 measurement-integrity, Q-P10-4 default flip).

CSV columns (no header, one row per round):
config,iter,do_kill,storage_id,t_start,t_kill,info_direct,info_reindex,
msg_count,published,seconds_lost

Usage: go1_report.py <results_csv>
Exit 0 PASS / 1 FAIL / 2 FAILED TO MEASURE (verify_mission.sh convention).
"""
import csv
import statistics
import sys

KILL_CONFIGS = ['mcap_zstdfile', 'mcap_none', 'sqlite3_none']
STORAGE_OF = {'mcap_zstdfile': 'mcap', 'mcap_none': 'mcap', 'sqlite3_none': 'sqlite3'}
READABLE_RATIO_THRESHOLD = 0.9  # 90% of MEASURED kill rounds -- same bar as the old 9/10
MIN_MEASURED_ROUNDS = 8  # out of nominal 10 -- below this the sample itself is not trustworthy


def parse_float(value):
    """R27-2: NaN must collapse to None at the one parse choke point, same
    as an unparsable value -- otherwise 'NAN' (the sentinel this gate's own
    shell code writes for "not measured") silently survives every
    downstream `is not None` filter as a live float and corrupts medians/
    counts (VÀNG#5)."""
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    return None if f != f else f


def load(csv_path):
    rows = []
    with open(csv_path, newline='') as handle:
        for row in csv.reader(handle):
            if not row:
                continue
            (config, iteration, do_kill, storage_id, t_start, t_kill,
             info_direct, info_reindex, msg_count, published, seconds_lost) = row
            rows.append({
                'config': config, 'iter': int(iteration), 'do_kill': do_kill == 'true',
                'storage_id': storage_id, 't_start': parse_float(t_start),
                't_kill': parse_float(t_kill),
                'info_direct': int(info_direct), 'info_reindex': info_reindex,
                'msg_count': int(msg_count), 'published': int(published),
                'seconds_lost': parse_float(seconds_lost),
            })
    return rows


def summarize_kill_config(rows, config):
    # VÀNG#5: do_kill=false rows for this config are startup FTMs (node
    # never came up -- nothing was actually killed), NOT measured kill
    # rounds. They must not inflate n, and the readable-count threshold
    # below is checked as a RATIO of MEASURED rounds, not an absolute count
    # against a denominator that silently grew/shrank with retries.
    all_attempts = [r for r in rows if r['config'] == config]
    subset = [r for r in all_attempts if r['do_kill']]
    n = len(subset)
    startup_ftm = len(all_attempts) - n
    readable_direct = sum(1 for r in subset if r['info_direct'] == 1)
    readable_after_reindex = sum(
        1 for r in subset if r['info_direct'] == 1 or r['info_reindex'] == '1')
    reindex_attempted = [r for r in subset if r['info_reindex'] in ('0', '1')]
    reindex_recovered = sum(1 for r in reindex_attempted if r['info_reindex'] == '1')

    losses = [r['published'] - r['msg_count'] for r in subset]
    loss_pcts = [
        100.0 * loss / r['published'] for loss, r in zip(losses, subset) if r['published'] > 0]
    # parse_float already collapsed 'NAN'/unparsable into None (R27-2) --
    # 'is not None' is now a real filter, not a NaN leak.
    seconds_lost = [r['seconds_lost'] for r in subset if r['seconds_lost'] is not None]

    return {
        'n': n,
        'startup_ftm': startup_ftm,
        'total_attempts': len(all_attempts),
        'readable_direct_rate': f'{readable_direct}/{n}',
        'readable_after_reindex_rate': f'{readable_after_reindex}/{n}',
        'readable_after_reindex_ratio': (readable_after_reindex / n) if n else float('nan'),
        'reindex_attempts': len(reindex_attempted),
        'reindex_recovered': reindex_recovered,
        'median_msg_loss': statistics.median(losses) if losses else float('nan'),
        'median_msg_loss_pct': statistics.median(loss_pcts) if loss_pcts else float('nan'),
        'median_seconds_lost': statistics.median(seconds_lost) if seconds_lost else float('nan'),
        'seconds_lost_measured_n': f'{len(seconds_lost)}/{n}',
        'readable_after_reindex_count': readable_after_reindex,
    }


# The 2 clean-SIGINT positive-control rounds (round_o1_control) also write
# do_kill='false' -- but so do startup-FTM rows now (VÀNG#5 fix above).
# Must key off config name, not do_kill, or a kill-config startup FTM gets
# folded into "the measurement method is broken" (R27-3).
CONTROL_CONFIGS = ('ctrl_mcap', 'ctrl_sqlite3')


def check_positive_control(rows):
    subset = [r for r in rows if r['config'] in CONTROL_CONFIGS]
    ok = True
    lines = []
    for r in subset:
        readable = r['info_direct'] == 1
        if r['published'] > 0:
            pct_diff = 100.0 * abs(r['published'] - r['msg_count']) / r['published']
        else:
            pct_diff = float('inf')
        within_tolerance = readable and pct_diff <= 1.0
        ok = ok and within_tolerance
        lines.append(
            f"  [{r['config']} iter={r['iter']}] readable={readable} "
            f"published={r['published']} msg_count={r['msg_count']} "
            f"diff={pct_diff:.2f}% -> {'OK' if within_tolerance else 'FAIL'}")
    return ok, lines, len(subset)


def main() -> int:
    csv_path = sys.argv[1]
    rows = load(csv_path)

    control_ok, control_lines, control_n = check_positive_control(rows)
    print('=== Positive control (clean SIGINT, no kill) ===')
    print(f'  rounds: {control_n} (expect 2, one mcap one sqlite3)')
    for line in control_lines:
        print(line)
    if control_n == 0:
        print('FAILED TO MEASURE: no positive-control rows in CSV -- run round o1-control first')
        return 2
    if not control_ok:
        print('FAILED TO MEASURE: positive control failed -- the measurement method itself '
              'is not trustworthy (R27-3: a broken positive control invalidates the gate)')
        return 2
    print('  positive control: PASS (measurement method verified alive)')

    print()
    print('=== Kill (-9) rounds, 3 configs x 10 (nominal) ===')
    header = (
        f"{'config':16} {'measured':9} {'startup_ftm':11} {'readable_direct':16} "
        f"{'readable(+reindex)':20} {'msg_loss_med':13} {'loss%_med':10} "
        f"{'sec_lost_med':13} {'sec_lost_n':11}")
    print(header)
    summaries = {}
    ftm_configs = []
    for config in KILL_CONFIGS:
        summary = summarize_kill_config(rows, config)
        summaries[config] = summary
        if summary['total_attempts'] == 0:
            print(f'FAILED TO MEASURE: no rows for {config} -- run that round first')
            return 2
        print(
            f"{config:16} {summary['n']:<9} {summary['startup_ftm']:<11} "
            f"{summary['readable_direct_rate']:16} "
            f"{summary['readable_after_reindex_rate']:20} "
            f"{summary['median_msg_loss']:<13.1f} {summary['median_msg_loss_pct']:<10.2f} "
            f"{summary['median_seconds_lost']:<13.3f} {summary['seconds_lost_measured_n']:11}")
        if config == 'sqlite3_none':
            print(
                f"    (reindex attempted on {summary['reindex_attempts']} round(s) that failed "
                f"direct info; recovered {summary['reindex_recovered']})")
        # VÀNG#5: too many startup FTMs means the SAMPLE is not trustworthy,
        # independent of how readable the few that did measure turned out
        # to be -- a config that only got 3 real kill measurements out of
        # 10 attempts cannot back a default-storage decision.
        if summary['n'] < MIN_MEASURED_ROUNDS:
            ftm_configs.append(config)

    if ftm_configs:
        print()
        for config in ftm_configs:
            s = summaries[config]
            print(
                f'FAILED TO MEASURE: {config} only reached {s["n"]}/{MIN_MEASURED_ROUNDS} minimum '
                f'measured kill rounds (startup_ftm={s["startup_ftm"]}, node kept failing to '
                'come up -- that is a startup bug, not a durability result)')
        return 2

    print()
    print('=== Verdict ===')
    candidates = [
        c for c in KILL_CONFIGS
        if summaries[c]['readable_after_reindex_ratio'] >= READABLE_RATIO_THRESHOLD
    ]
    if not candidates:
        print(
            f'FAIL: no config reached the {READABLE_RATIO_THRESHOLD:.0%} readable-of-measured '
            'threshold -- cannot pick a default storage_id from this experiment')
        for config in KILL_CONFIGS:
            s = summaries[config]
            print(
                f"  {config}: readable {s['readable_after_reindex_rate']} "
                f"({s['readable_after_reindex_ratio']:.0%} of {s['n']} measured)")
        return 1

    def sort_key(config):
        s = summaries[config]
        loss = s['median_msg_loss'] if s['median_msg_loss'] == s['median_msg_loss'] else 1e9
        sec = s['median_seconds_lost'] if s['median_seconds_lost'] == s['median_seconds_lost'] else 1e9
        return (-s['readable_after_reindex_count'], loss, sec)

    best = sorted(candidates, key=sort_key)[0]
    default_stays_mcap = STORAGE_OF[best] == 'mcap' and best.startswith('mcap')
    mcap_candidates = [c for c in candidates if STORAGE_OF[c] == 'mcap']
    print(f"chosen storage_id default: {STORAGE_OF[best]}  (winning config: {best})")
    print(
        f"  readable={summaries[best]['readable_after_reindex_rate']} "
        f"median_msg_loss={summaries[best]['median_msg_loss']:.1f} "
        f"median_seconds_lost={summaries[best]['median_seconds_lost']:.3f}")
    if STORAGE_OF[best] == 'mcap':
        print('  Q-P10-4: mcap default KEPT (no re-sign needed).')
    else:
        print(
            '  Q-P10-4: FLIP recommended -- mcap candidate(s) did not beat sqlite3_none on '
            f'readability/loss ({mcap_candidates or "none reached threshold"}).')
    print(
        'RESIDUAL RISK (must stay in README): kill -9 tests process death only -- OS page '
        'cache is still flushed by the kernel on process exit, this is NOT equivalent to a '
        'real power-loss event. No claim here covers power loss.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
