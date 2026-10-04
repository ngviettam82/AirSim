#!/usr/bin/env python3
"""P10.7 G-O2: %%CPU sampler for a set of PIDs via /proc/<pid>/stat.

No apt dependency (pidstat/sysstat not installed) -- reads utime+stime
jiffies deltas directly, the same field rosbag2's own thread accounting
would use. One sample column per PID per --interval tick for --duration
seconds; --output gets a JSON summary (avg/peak %% of ONE core).

Usage: o2_cpu_sampler.py PID [PID ...] --duration 90 --output /tmp/cpu.json
"""
import argparse
import json
import os
import signal
import time

CLK_TCK = os.sysconf('SC_CLK_TCK')
_STOP = {'flag': False}


def _handle_signal(_signum, _frame):
    _STOP['flag'] = True


signal.signal(signal.SIGINT, _handle_signal)
signal.signal(signal.SIGTERM, _handle_signal)


def read_ticks(pid):
    with open('/proc/%d/stat' % pid) as f:
        # comm field (2nd) can contain spaces/parens -- split after the
        # LAST ')' to stay robust, per proc(5).
        raw = f.read()
    after_comm = raw.rsplit(')', 1)[1].split()
    # After the comm field, proc(5) field 3 is state (index 0 here);
    # utime is field 14 overall -> index 11 here; stime is field 15 -> index 12.
    utime = int(after_comm[11])
    stime = int(after_comm[12])
    return utime + stime


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('pids', nargs='+', type=int)
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--interval', type=float, default=1.0)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()

    last_ticks = {}
    last_time = {}
    samples = {pid: [] for pid in args.pids}
    dead = set()

    for pid in args.pids:
        try:
            last_ticks[pid] = read_ticks(pid)
            last_time[pid] = time.monotonic()
        except (FileNotFoundError, ProcessLookupError, IndexError):
            dead.add(pid)

    end = time.monotonic() + args.duration
    while time.monotonic() < end and not _STOP['flag']:
        time.sleep(args.interval)
        now = time.monotonic()
        for pid in args.pids:
            if pid in dead:
                continue
            try:
                ticks = read_ticks(pid)
            except (FileNotFoundError, ProcessLookupError, IndexError):
                dead.add(pid)
                continue
            dt = now - last_time[pid]
            dticks = ticks - last_ticks[pid]
            pct = (dticks / CLK_TCK) / dt * 100.0 if dt > 0 else 0.0
            samples[pid].append(pct)
            last_ticks[pid] = ticks
            last_time[pid] = now

    report = {}
    for pid in args.pids:
        vals = samples[pid]
        report[str(pid)] = {
            'died_mid_sample': pid in dead,
            'n_samples': len(vals),
            'avg_pct': (sum(vals) / len(vals)) if vals else None,
            'peak_pct': max(vals) if vals else None,
        }
    with open(args.output, 'w') as f:
        json.dump(report, f, indent=2)

    for pid, r in report.items():
        avg = r['avg_pct'] if r['avg_pct'] is not None else -1.0
        peak = r['peak_pct'] if r['peak_pct'] is not None else -1.0
        print('CPU pid=%s avg=%.1f peak=%.1f n=%d died=%s' % (
            pid, avg, peak, r['n_samples'], r['died_mid_sample']))


if __name__ == '__main__':
    main()
