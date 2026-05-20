#!/usr/bin/env python3
"""Runtime validation for CosysAirSim equirectangular captures inside the Unreal project."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from collections import Counter
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
PYTHON_CLIENT = REPO_ROOT / "PythonClient"
if str(PYTHON_CLIENT) not in sys.path:
    sys.path.insert(0, str(PYTHON_CLIENT))

import cosysairsim as airsim  # noqa: E402


IMAGE_TYPES = [
    ("Scene", airsim.ImageType.Scene, False, False, ""),
    ("DepthPerspective", airsim.ImageType.DepthPerspective, True, False, ""),
    ("Segmentation", airsim.ImageType.Segmentation, False, False, ""),
    ("Infrared", airsim.ImageType.Infrared, False, False, ""),
    ("Lighting", airsim.ImageType.Lighting, False, False, ""),
    ("Annotation", airsim.ImageType.Annotation, False, False, "runtime_equirectangular_annotation"),
]

DISCRETE_TYPES = {"Segmentation", "Infrared", "Annotation"}


def response_u8_array(response):
    data = response.image_data_uint8
    if isinstance(data, str):
        raw = data.encode("latin1")
        arr = np.frombuffer(raw, dtype=np.uint8)
    elif isinstance(data, (bytes, bytearray, memoryview)):
        arr = np.frombuffer(data, dtype=np.uint8)
    else:
        arr = np.asarray(data, dtype=np.uint8)

    if response.width <= 0 or response.height <= 0:
        raise ValueError("response has invalid dimensions")

    pixels = response.width * response.height
    if arr.size == pixels * 4:
        return arr.reshape((response.height, response.width, 4))
    if arr.size == pixels * 3:
        return arr.reshape((response.height, response.width, 3))
    raise ValueError(
        f"unexpected uint8 payload length {arr.size}; expected {pixels * 3} or {pixels * 4}"
    )


def response_float_array(response):
    arr = np.asarray(response.image_data_float, dtype=np.float32)
    expected = response.width * response.height
    if arr.size != expected:
        raise ValueError(f"unexpected float payload length {arr.size}; expected {expected}")
    return arr.reshape((response.height, response.width))


def center_patch(arr, radius=3):
    h, w = arr.shape[:2]
    cy = h // 2
    cx = w // 2
    y0 = max(0, cy - radius)
    y1 = min(h, cy + radius + 1)
    x0 = max(0, cx - radius)
    x1 = min(w, cx + radius + 1)
    return arr[y0:y1, x0:x1]


def mode_color(arr):
    flat = arr[..., :3].reshape((-1, 3))
    values = Counter(map(tuple, flat.tolist()))
    color, count = values.most_common(1)[0]
    return list(color), count / max(1, flat.shape[0])


def compare_uint8(name, equirectangular, ref):
    equirectangular_arr = response_u8_array(equirectangular)
    ref_arr = response_u8_array(ref)
    equirectangular_patch = center_patch(equirectangular_arr)
    ref_patch = center_patch(ref_arr)

    if name in DISCRETE_TYPES:
        equirectangular_mode, equirectangular_frac = mode_color(equirectangular_patch)
        ref_mode, ref_frac = mode_color(ref_patch)
        center_exact = equirectangular_mode == ref_mode
        center_metric = {
            "equirectangular_mode_rgb": equirectangular_mode,
            "equirectangular_mode_fraction": equirectangular_frac,
            "ref_mode_rgb": ref_mode,
            "ref_mode_fraction": ref_frac,
            "center_mode_exact": center_exact,
        }
    else:
        diff = np.abs(
            equirectangular_patch[..., :3].astype(np.int16) - ref_patch[..., :3].astype(np.int16)
        )
        center_metric = {
            "center_rgb_mean_abs_diff": float(diff.mean()),
            "center_rgb_max_abs_diff": int(diff.max()),
        }

    left = equirectangular_arr[:, 0, :3].astype(np.int16)
    right = equirectangular_arr[:, -1, :3].astype(np.int16)
    seam_diff = np.abs(left - right)
    edge_rgb = np.concatenate([left, right], axis=0)
    black_fraction = float(np.all(edge_rgb == 0, axis=1).mean())
    magenta_fraction = float(np.all(edge_rgb == np.array([255, 0, 255]), axis=1).mean())

    return {
        **center_metric,
        "wrap_column_rgb_mean_abs_diff": float(seam_diff.mean()),
        "wrap_column_rgb_max_abs_diff": int(seam_diff.max()),
        "edge_exact_black_fraction": black_fraction,
        "edge_exact_magenta_fraction": magenta_fraction,
        "equirectangular_payload_bytes": int(np.asarray(equirectangular_arr).size),
        "ref_payload_bytes": int(np.asarray(ref_arr).size),
    }


def compare_float(equirectangular, ref):
    equirectangular_arr = response_float_array(equirectangular)
    ref_arr = response_float_array(ref)
    equirectangular_patch = center_patch(equirectangular_arr)
    ref_patch = center_patch(ref_arr)
    diff = np.abs(equirectangular_patch - ref_patch)
    finite = np.isfinite(diff)
    if not finite.any():
        mean_abs = math.nan
        max_abs = math.nan
    else:
        mean_abs = float(diff[finite].mean())
        max_abs = float(diff[finite].max())

    left = equirectangular_arr[:, 0]
    right = equirectangular_arr[:, -1]
    seam = np.abs(left - right)
    seam_finite = np.isfinite(seam)
    return {
        "center_depth_mean_abs_diff": mean_abs,
        "center_depth_max_abs_diff": max_abs,
        "center_equirectangular_depth_mean": float(np.nanmean(equirectangular_patch)),
        "center_ref_depth_mean": float(np.nanmean(ref_patch)),
        "wrap_column_depth_mean_abs_diff": float(np.nanmean(seam[seam_finite]))
        if seam_finite.any()
        else math.nan,
        "wrap_column_depth_max_abs_diff": float(np.nanmax(seam[seam_finite]))
        if seam_finite.any()
        else math.nan,
        "equirectangular_payload_floats": int(equirectangular_arr.size),
        "ref_payload_floats": int(ref_arr.size),
    }


def wait_for_server(ip, port, timeout_s):
    deadline = time.time() + timeout_s
    last_error = None
    while time.time() < deadline:
        client = airsim.MultirotorClient(ip=ip, port=port, timeout_value=2)
        try:
            if client.ping():
                return airsim.MultirotorClient(ip=ip, port=port, timeout_value=max(60, timeout_s))
        except Exception as exc:  # noqa: BLE001
            last_error = exc
            time.sleep(1)
    raise TimeoutError(f"AirSim RPC server did not respond within {timeout_s}s: {last_error}")


def save_png_bytes(path, payload):
    if isinstance(payload, str):
        payload = payload.encode("latin1")
    with open(path, "wb") as handle:
        handle.write(payload)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=41451)
    parser.add_argument("--vehicle", default="drone_1")
    parser.add_argument("--equirectangular-camera", default="front_center")
    parser.add_argument("--ref-camera", default="front_center_ref")
    parser.add_argument("--expected-height", type=int, default=256)
    parser.add_argument("--timeout", type=int, default=180)
    parser.add_argument("--pause", action="store_true")
    parser.add_argument(
        "--out-dir",
        default=str(Path.home() / "Documents" / "AirSim" / "equirectangular_runtime_validation"),
    )
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    client = wait_for_server(args.ip, args.port, args.timeout)
    if args.pause:
        client.simPause(True)

    raw_requests = []
    for _, image_type, pixels_as_float, compress, annotation_name in IMAGE_TYPES:
        raw_requests.append(
            airsim.ImageRequest(
                args.equirectangular_camera, image_type, pixels_as_float, compress, annotation_name
            )
        )
        raw_requests.append(
            airsim.ImageRequest(
                args.ref_camera, image_type, pixels_as_float, compress, annotation_name
            )
        )

    responses = client.simGetImages(raw_requests, vehicle_name=args.vehicle)
    if len(responses) != len(raw_requests):
        raise RuntimeError(f"expected {len(raw_requests)} responses, got {len(responses)}")

    expected_width = args.expected_height * 2
    summary = {
        "expected_equirectangular_width": expected_width,
        "expected_equirectangular_height": args.expected_height,
        "image_types": {},
    }
    failures = []

    for index, (name, image_type, pixels_as_float, _, annotation_name) in enumerate(IMAGE_TYPES):
        equirectangular = responses[index * 2]
        ref = responses[index * 2 + 1]
        item = {
            "image_type": int(image_type),
            "annotation_name": annotation_name,
            "equirectangular_width": int(equirectangular.width),
            "equirectangular_height": int(equirectangular.height),
            "ref_width": int(ref.width),
            "ref_height": int(ref.height),
            "equirectangular_message": equirectangular.message,
            "ref_message": ref.message,
        }

        if equirectangular.width != expected_width or equirectangular.height != args.expected_height:
            failures.append(
                f"{name}: equirectangular dimensions were {equirectangular.width}x{equirectangular.height}, "
                f"expected {expected_width}x{args.expected_height}"
            )
        if ref.width <= 0 or ref.height <= 0:
            failures.append(f"{name}: reference camera returned invalid dimensions")

        try:
            if pixels_as_float:
                item.update(compare_float(equirectangular, ref))
                np.save(out_dir / f"{name}_equirectangular_float.npy", response_float_array(equirectangular))
                np.save(out_dir / f"{name}_reference_float.npy", response_float_array(ref))
            else:
                item.update(compare_uint8(name, equirectangular, ref))
                np.save(out_dir / f"{name}_equirectangular_uint8.npy", response_u8_array(equirectangular))
                np.save(out_dir / f"{name}_reference_uint8.npy", response_u8_array(ref))
        except Exception as exc:  # noqa: BLE001
            failures.append(f"{name}: comparison failed: {exc}")

        summary["image_types"][name] = item

    compressed_requests = [
        airsim.ImageRequest(args.equirectangular_camera, image_type, False, True, annotation_name)
        for name, image_type, pixels_as_float, _, annotation_name in IMAGE_TYPES
        if not pixels_as_float
    ]
    compressed_responses = client.simGetImages(compressed_requests, vehicle_name=args.vehicle)
    compressed_names = [name for name, _, pixels_as_float, _, _ in IMAGE_TYPES if not pixels_as_float]
    for name, response in zip(compressed_names, compressed_responses):
        if response.width == expected_width and response.height == args.expected_height:
            save_png_bytes(out_dir / f"{name}_equirectangular.png", response.image_data_uint8)

    summary_path = out_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print(json.dumps(summary, indent=2))
    if failures:
        print("\nFAILURES:")
        for failure in failures:
            print(f"- {failure}")
        return 1

    print(f"\nRuntime equirectangular validation artifacts written to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
