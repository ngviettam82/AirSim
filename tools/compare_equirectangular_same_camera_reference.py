#!/usr/bin/env python3
"""Compare a saved equirectangular pass against a live perspective pass from the same camera."""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
PYTHON_CLIENT = REPO_ROOT / "PythonClient"
if str(PYTHON_CLIENT) not in sys.path:
    sys.path.insert(0, str(PYTHON_CLIENT))

import cosysairsim as airsim  # noqa: E402


IMAGE_TYPES = [
    ("Scene", airsim.ImageType.Scene, False, ""),
    ("DepthPerspective", airsim.ImageType.DepthPerspective, True, ""),
    ("Segmentation", airsim.ImageType.Segmentation, False, ""),
    ("Infrared", airsim.ImageType.Infrared, False, ""),
    ("Lighting", airsim.ImageType.Lighting, False, ""),
    ("Annotation", airsim.ImageType.Annotation, False, "runtime_equirectangular_annotation"),
]

DISCRETE_TYPES = {"Segmentation", "Infrared", "Annotation"}


def u8_response_array(response):
    data = response.image_data_uint8
    if isinstance(data, str):
        data = data.encode("latin1")
    arr = np.frombuffer(data, dtype=np.uint8)
    return arr.reshape((response.height, response.width, 3))


def patch(arr, radius=3):
    h, w = arr.shape[:2]
    cy, cx = h // 2, w // 2
    return arr[cy - radius : cy + radius + 1, cx - radius : cx + radius + 1]


def mode_rgb(arr):
    flat = arr[..., :3].reshape((-1, 3))
    color, count = Counter(map(tuple, flat.tolist())).most_common(1)[0]
    return list(color), count / max(1, flat.shape[0])


def compare_u8(name, equirectangular, ref):
    equirectangular_patch = patch(equirectangular)
    ref_patch = patch(ref)
    if name in DISCRETE_TYPES:
        equirectangular_mode, equirectangular_fraction = mode_rgb(equirectangular_patch)
        ref_mode, ref_fraction = mode_rgb(ref_patch)
        return {
            "equirectangular_center_mode_rgb": equirectangular_mode,
            "equirectangular_center_mode_fraction": equirectangular_fraction,
            "ref_center_mode_rgb": ref_mode,
            "ref_center_mode_fraction": ref_fraction,
            "center_mode_exact": equirectangular_mode == ref_mode,
        }

    diff = np.abs(equirectangular_patch.astype(np.int16) - ref_patch.astype(np.int16))
    return {
        "center_rgb_mean_abs_diff": float(diff.mean()),
        "center_rgb_max_abs_diff": int(diff.max()),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--equirectangular-dir", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--ip", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=41451)
    parser.add_argument("--vehicle", default="drone_1")
    parser.add_argument("--camera", default="front_center")
    parser.add_argument("--timeout", type=int, default=120)
    parser.add_argument("--pause", action="store_true")
    args = parser.parse_args()

    equirectangular_dir = Path(args.equirectangular_dir)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    client = airsim.MultirotorClient(ip=args.ip, port=args.port, timeout_value=args.timeout)
    assert client.ping()
    if args.pause:
        client.simPause(True)

    requests = [
        airsim.ImageRequest(args.camera, image_type, pixels_as_float, False, annotation_name)
        for _, image_type, pixels_as_float, annotation_name in IMAGE_TYPES
    ]
    responses = client.simGetImages(requests, vehicle_name=args.vehicle)

    summary = {"camera": args.camera, "vehicle": args.vehicle, "image_types": {}}
    failures = []

    for (name, _, pixels_as_float, _), response in zip(IMAGE_TYPES, responses):
        item = {
            "reference_width": int(response.width),
            "reference_height": int(response.height),
            "reference_message": response.message,
        }

        try:
            if pixels_as_float:
                equirectangular = np.load(equirectangular_dir / f"{name}_equirectangular_float.npy")
                ref = np.asarray(response.image_data_float, dtype=np.float32).reshape(
                    (response.height, response.width)
                )
                np.save(out_dir / f"{name}_same_camera_reference_float.npy", ref)
                diff = np.abs(patch(equirectangular) - patch(ref))
                item.update(
                    {
                        "equirectangular_center_mean": float(np.nanmean(patch(equirectangular))),
                        "ref_center_mean": float(np.nanmean(patch(ref))),
                        "center_depth_mean_abs_diff": float(np.nanmean(diff)),
                        "center_depth_max_abs_diff": float(np.nanmax(diff)),
                    }
                )
            else:
                equirectangular = np.load(equirectangular_dir / f"{name}_equirectangular_uint8.npy")
                ref = u8_response_array(response)
                np.save(out_dir / f"{name}_same_camera_reference_uint8.npy", ref)
                item.update(compare_u8(name, equirectangular, ref))
        except Exception as exc:  # noqa: BLE001
            failures.append(f"{name}: {exc}")

        summary["image_types"][name] = item

    summary_path = out_dir / "same_camera_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))

    if failures:
        print("\nFAILURES:")
        for failure in failures:
            print(f"- {failure}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
