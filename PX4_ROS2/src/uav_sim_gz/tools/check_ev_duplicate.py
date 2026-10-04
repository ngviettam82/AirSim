"""Detect a second, unintended publisher on PX4's vehicle_visual_odometry.

Usage: python3 check_ev_duplicate.py <log.ulg> [more.ulg ...]

Two publishers interleave on one uORB topic and look like a single healthy
stream in `ros2 topic hz`; only the value histogram gives them away. See
docs/package-status.md section 2, "PX4 tu tiem ground-truth vao EKF2".
"""
import sys

import numpy as np
from pyulog import ULog

TOPIC = 'vehicle_visual_odometry'


def inspect(path):
    try:
        ulog = ULog(path, [TOPIC])
    except Exception as error:
        print(f'{path}: cannot read ({error})')
        return
    matched = {series.name: series for series in ulog.data_list}.get(TOPIC)
    name = path.split('/')[-1]
    if matched is None:
        print(f'{name}: no {TOPIC} - nothing feeds EKF2 as vision')
        return

    stamps = np.asarray(matched.data['timestamp'], dtype=float) / 1e6
    span = stamps[-1] - stamps[0]
    rate = len(stamps) / span if span > 0 else 0.0
    axes = [np.asarray(matched.data[f'position[{i}]'], dtype=float) for i in range(3)]
    levels = [np.unique(np.round(axis, 3)) for axis in axes]

    # Stationary horizontally but not vertically is the duplicate-publisher shape.
    stationary_xy = all(len(levels[i]) <= 2 for i in (0, 1))
    spread_z = levels[2].max() - levels[2].min()
    suspect = stationary_xy and spread_z > 0.05

    print(f'{name}: n={len(stamps)} span={span:.1f}s rate={rate:.1f}Hz')
    for i, level in enumerate(levels):
        shown = np.round(level, 4) if len(level) <= 4 else \
            f'[{level.min():+.4f} .. {level.max():+.4f}]'
        print(f'   position[{i}] distinct={len(level):4d} {shown}')
    if suspect:
        print(f'   <-- SUSPECT: z spans {spread_z:.4f} m while x/y are fixed;'
              f' expect a second publisher offset by a lever arm')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    for argument in sys.argv[1:]:
        inspect(argument)
