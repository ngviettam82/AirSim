"""Report which sources EKF2 is actually aiding on, and when it reset to them.

Usage: python3 probe_estimator_aiding.py <log.ulg>

Read the EVENT TIMESTAMPS, not just the counts: a control-status flag that is
"80% true" is usually just the window before the ROS stack came up. That
mistake was made once already - see docs/package-status.md section 2.
"""
import sys

import numpy as np
from pyulog import ULog

CONTROL_FLAGS = ['cs_ev_pos', 'cs_ev_hgt', 'cs_ev_vel', 'cs_ev_yaw',
                 'cs_baro_hgt', 'cs_gps_hgt', 'cs_rng_hgt', 'cs_gps',
                 'cs_fake_pos', 'cs_fake_hgt']


def main(path):
    ulog = ULog(path)
    data = {series.name: series for series in ulog.data_list}

    flags = data.get('estimator_status_flags')
    if flags is not None:
        stamps = np.asarray(flags.data['timestamp'], dtype=float) / 1e6
        print(f'control status flags (published every '
              f'{np.median(np.diff(stamps)):.2f}s, span {stamps[0]:.1f}..{stamps[-1]:.1f}s)')
        for field in CONTROL_FLAGS:
            if field not in flags.data:
                continue
            values = np.asarray(flags.data[field])
            true_at = stamps[values != 0]
            summary = ('always false' if len(true_at) == 0 else
                       'always true' if len(true_at) == len(values) else
                       f'true {true_at.min():.1f}..{true_at.max():.1f}s')
            print(f'  {field:14s} {100 * np.mean(values != 0):5.1f}% true   {summary}')

    events = data.get('estimator_event_flags')
    if events is not None:
        stamps = np.asarray(events.data['timestamp'], dtype=float) / 1e6
        print('\nreset / start events')
        for field in sorted(events.data):
            if field in ('timestamp', 'timestamp_sample') or 'changes' in field:
                continue
            fired = stamps[np.asarray(events.data[field]) != 0]
            if len(fired) == 0:
                continue
            cadence = f'every {np.median(np.diff(fired)):.2f}s' if len(fired) > 1 else 'once'
            print(f'  {field:32s} n={len(fired):4d}  '
                  f'{fired.min():6.1f}..{fired.max():6.1f}s  {cadence}')

    local = data.get('vehicle_local_position')
    vision = data.get('vehicle_visual_odometry')
    if local is not None and vision is not None:
        lt = np.asarray(local.data['timestamp'], dtype=float) / 1e6
        lz = np.asarray(local.data['z'], dtype=float)
        vt = np.asarray(vision.data['timestamp'], dtype=float) / 1e6
        vz = np.asarray(vision.data['position[2]'], dtype=float)
        window = vt >= vt[-1] - 20.0
        residual = np.interp(vt[window], lt, lz) - vz[window]
        print(f'\nEKF2_z - vision_z (NED, last 20 s, n={int(window.sum())}): '
              f'{np.median(residual):+.4f} m')
        print('  = what odometry_fused minus odometry_raw will read on z')

    for topic in ('estimator_baro_bias', 'estimator_ev_pos_bias'):
        series = data.get(topic)
        if series is None:
            print(f'{topic}: never moved more than its publish threshold')
            continue
        keys = [k for k in series.data if k.startswith('bias') and 'var' not in k]
        for key in sorted(keys):
            values = np.asarray(series.data[key], dtype=float)
            print(f'{topic}.{key}: median={np.median(values):+.4f} last={values[-1]:+.4f}')


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    main(sys.argv[1])
