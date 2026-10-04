#!/usr/bin/env python3
"""Zoom in on a marker; report why it fails decoding."""

import argparse
import os
import sys

import cv2  # run with PYTHONNOUSERSITE=1, README pitfall 9
import numpy as np

MARKER_CELLS = 6  # DICT_4X4_50: 4 data cells plus one border cell each side


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--frame', required=True)
    parser.add_argument('--out-dir', default=os.path.expanduser('~/marker_debug'))
    parser.add_argument('--zoom', type=int, default=6)
    args = parser.parse_args()

    image = cv2.imread(args.frame, cv2.IMREAD_GRAYSCALE)
    if image is None:
        print('cannot read %s' % args.frame)
        return 1
    os.makedirs(args.out_dir, exist_ok=True)

    dark = (image < 60).astype(np.uint8)
    if not dark.any():
        print('no dark pixels: nothing printed is in view')
        return 1
    ys, xs = np.nonzero(dark)
    x0, x1 = int(xs.min()), int(xs.max())
    y0, y1 = int(ys.min()), int(ys.max())
    span = max(x1 - x0, y1 - y0)

    pad = 25
    crop = image[max(0, y0 - pad):y1 + pad, max(0, x0 - pad):x1 + pad]
    name = os.path.splitext(os.path.basename(args.frame))[0]
    zoom_path = os.path.join(args.out_dir, name + '_zoom.png')
    cv2.imwrite(
        zoom_path,
        cv2.resize(crop, (crop.shape[1] * args.zoom, crop.shape[0] * args.zoom),
                   interpolation=cv2.INTER_NEAREST))

    inner = image[y0:y1 + 1, x0:x1 + 1]
    print('marker region   %d x %d px at (%d, %d)' % (x1 - x0, y1 - y0, x0, y0))
    print('pixels per cell %.1f  (needs roughly 4 to decode reliably)'
          % (span / MARKER_CELLS))
    print('contrast        min %d  max %d  spread %d'
          % (inner.min(), inner.max(), int(inner.max()) - int(inner.min())))
    # A crisp pattern is bimodal; mid-grey means blurred cells.
    mid = float(((inner > 80) & (inner < 170)).mean())
    print('mid-grey pixels %.1f%%  (high means the cells blurred together)'
          % (100 * mid))
    print('saved zoom      %s' % zoom_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
