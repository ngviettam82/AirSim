#!/usr/bin/env python3
"""Measure frame-latched dual-rate recording rates and image/snapshot deltas."""
from __future__ import annotations

import os
import statistics
import time
from pathlib import Path

import cosysairsim as airsim

DOC = Path(os.environ.get("AIRSIM_RECORDING_DIR", os.path.expanduser("~/Documents/AirSim")))


def wait_api(timeout=120):
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        try:
            c = airsim.MultirotorClient()
            c.confirmConnection()
            return c
        except Exception as e:
            last = e
            time.sleep(2)
    raise RuntimeError(f"API not ready: {last}")


def newest_session(not_before=0.0):
    if not DOC.is_dir():
        raise FileNotFoundError(f"recording directory does not exist: {DOC}")
    cands = [p for p in DOC.iterdir() if p.is_dir() and (p / "airsim_rec.txt").is_file()]
    cands = [p for p in cands if (p / "airsim_rec.txt").stat().st_mtime >= not_before]
    cands.sort(key=lambda d: (d / "airsim_rec.txt").stat().st_mtime, reverse=True)
    if not cands:
        raise FileNotFoundError(f"no new sessions under {DOC}")
    return cands[0]


def wait_stable_session(not_before, timeout=30.0):
    deadline = time.time() + timeout
    last_path = None
    last_size = None
    stable_count = 0
    while time.time() < deadline:
        try:
            path = newest_session(not_before) / "airsim_rec.txt"
            size = path.stat().st_size
            if path == last_path and size == last_size and size > 0:
                stable_count += 1
                if stable_count >= 3:
                    return path.parent
            else:
                stable_count = 0
            last_path, last_size = path, size
        except FileNotFoundError:
            pass
        time.sleep(0.2)
    raise TimeoutError("recording file did not become stable")


def analyze(sess: Path):
    lines = (sess / "airsim_rec.txt").read_text(errors="replace").replace("\r\n", "\n").split("\n")
    lines = [l for l in lines if l.strip()]
    h = lines[0].split("\t")
    idx = {n: i for i, n in enumerate(h)}
    rows = [l.split("\t") for l in lines[1:] if len(l.split("\t")) == len(h)]

    lower_idx = {name.lower(): column for name, column in idx.items()}
    imu_present_col = lower_idx.get("imu_present")
    imu_timestamp_col = lower_idx.get("imu_timestamp")
    imu_age_col = lower_idx.get("imu_age")

    sensor_rows = [
        r for r in rows
        if "AssociationMode" not in idx or r[idx["AssociationMode"]].strip() == "sensor_only"
    ]
    img_rows = [r for r in rows if "ImageFile" in idx and r[idx["ImageFile"]].strip()]

    def intervals_ms(ts):
        return [(ts[i] - ts[i - 1]) / 1e6 for i in range(1, len(ts)) if ts[i] > ts[i - 1]]

    def rate(ts):
        if len(ts) < 2:
            return None
        duration = (ts[-1] - ts[0]) / 1e9
        return (len(ts) - 1) / duration if duration > 0 else None

    def percentile(values, fraction):
        if not values:
            return None
        values = sorted(values)
        return values[round((len(values) - 1) * fraction)]

    sensor_fts = [int(r[idx["FrameTimeStamp"]]) for r in sensor_rows]
    sensor_dts = intervals_ms(sensor_fts)
    imu_ts = []
    ages = []
    if imu_present_col is not None and imu_timestamp_col is not None:
        imu_ts = [int(r[imu_timestamp_col]) for r in rows if r[imu_present_col].strip() == "1"]
        if imu_age_col is not None:
            ages = [int(r[imu_age_col]) for r in rows if r[imu_present_col].strip() == "1"]

    unique_imu_ts = list(dict.fromkeys(imu_ts))
    image_ts = []
    delays_ms = []
    sync_flags = []
    for row in img_rows:
        if "ImageTimeStamp" in idx:
            stamps = [v for v in row[idx["ImageTimeStamp"]].split(";") if v]
            if stamps:
                image_ts.append(int(stamps[0]))
        if "ImageDelayMs" in idx:
            delays_ms.extend(float(v) for v in row[idx["ImageDelayMs"]].split(";") if v)
        if "ImageSyncWithinTolerance" in idx:
            sync_flags.extend(v == "1" for v in row[idx["ImageSyncWithinTolerance"]].split(";") if v)

    image_dts = intervals_ms(image_ts)

    print(f"session={sess.name}")
    print(f"rows_total={len(rows)} rows_with_image={len(img_rows)} rows_sensor_only={len(sensor_rows)}")
    if sensor_dts:
        print(
            f"sensor_row_rate_Hz_avg={rate(sensor_fts):.3f} "
            f"dt_ms min={min(sensor_dts):.3f} med={statistics.median(sensor_dts):.3f} "
            f"p95={percentile(sensor_dts, 0.95):.3f} max={max(sensor_dts):.3f}"
        )
    if len(unique_imu_ts) >= 2:
        print(f"imu_unique_timestamp_rate_Hz_avg={rate(unique_imu_ts):.3f}")
    if image_dts:
        print(
            f"camera_rate_Hz_avg={rate(image_ts):.3f} "
            f"dt_ms med={statistics.median(image_dts):.3f} max={max(image_dts):.3f}"
        )
    if ages:
        print(
            f"imu_Age_ms min={min(ages)/1e6:.4f} med={statistics.median(ages)/1e6:.4f} "
            f"max={max(ages)/1e6:.4f}"
        )
    if delays_ms:
        print(
            f"image_minus_snapshot_ms min={min(delays_ms):.4f} "
            f"med={statistics.median(delays_ms):.4f} "
            f"p95={percentile(delays_ms, 0.95):.4f} max={max(delays_ms):.4f}"
        )
    if sync_flags:
        print(f"images_within_tolerance={sum(sync_flags)}/{len(sync_flags)}")
    return 0


def main():
    c = wait_api()
    print("Connected")
    if c.isRecording():
        c.stopRecording()
        time.sleep(0.5)

    duration = 3.0
    started = time.time()
    print(f"Recording {duration}s (frame-latched dual-rate)...")
    c.startRecording()
    time.sleep(duration)
    c.stopRecording()
    return analyze(wait_stable_session(started - 1.0))


if __name__ == "__main__":
    raise SystemExit(main())
