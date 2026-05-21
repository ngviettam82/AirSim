#!/usr/bin/env python3
"""Capture the three common equirectangular outputs from a configured AirSim camera.

The script does not change settings.json. Configure the requested camera's Scene,
DepthPerspective, and Segmentation capture settings with
ProjectionMode="Equirectangular" before running it.
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

REPO_ROOT = Path(__file__).resolve().parents[1]
PYTHON_CLIENT = REPO_ROOT / "PythonClient"
if str(PYTHON_CLIENT) not in sys.path:
    sys.path.insert(0, str(PYTHON_CLIENT))

import cosysairsim as airsim  # noqa: E402


CAPTURES = [
    ("Scene", airsim.ImageType.Scene, False),
    ("DepthPerspective", airsim.ImageType.DepthPerspective, True),
    ("Segmentation", airsim.ImageType.Segmentation, False),
]


def default_output_dir() -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path.home() / "Documents" / "AirSim" / f"equirectangular_3type_capture_{timestamp}"


def response_u8_array(response: Any) -> np.ndarray:
    payload = response.image_data_uint8
    if isinstance(payload, str):
        payload = payload.encode("latin1")

    arr = np.frombuffer(payload, dtype=np.uint8)
    pixels = int(response.width) * int(response.height)
    if pixels <= 0:
        raise ValueError(response.message or "response has invalid dimensions")

    if arr.size == pixels * 4:
        return arr.reshape((response.height, response.width, 4))[..., :3].copy()
    if arr.size == pixels * 3:
        return arr.reshape((response.height, response.width, 3)).copy()

    raise ValueError(f"unexpected uint8 payload length {arr.size}; expected {pixels * 3} or {pixels * 4}")


def response_float_array(response: Any) -> np.ndarray:
    arr = np.asarray(response.image_data_float, dtype=np.float32)
    expected = int(response.width) * int(response.height)
    if expected <= 0:
        raise ValueError(response.message or "response has invalid dimensions")
    if arr.size != expected:
        raise ValueError(f"unexpected float payload length {arr.size}; expected {expected}")
    return arr.reshape((response.height, response.width))


def save_depth_preview(depth: np.ndarray, path: Path, preview_max: float) -> None:
    finite = np.isfinite(depth)
    if not finite.any():
        preview = np.zeros(depth.shape, dtype=np.uint8)
    else:
        clean = np.where(finite, depth, 0.0)
        scale_max = preview_max if preview_max > 0 else float(np.nanpercentile(clean[finite], 99))
        scale_max = max(scale_max, 1.0e-6)
        preview = (np.clip(clean / scale_max, 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8)

    Image.fromarray(preview).save(path)


def summarize_u8(arr: np.ndarray) -> dict[str, Any]:
    flat = arr.reshape((-1, arr.shape[-1]))
    return {
        "mean_rgb": [float(v) for v in flat[:, :3].mean(axis=0)],
        "min_rgb": [int(v) for v in flat[:, :3].min(axis=0)],
        "max_rgb": [int(v) for v in flat[:, :3].max(axis=0)],
        "unique_rgb_count": int(np.unique(flat[:, :3], axis=0).shape[0]),
    }


def summarize_float(arr: np.ndarray) -> dict[str, Any]:
    finite = arr[np.isfinite(arr)]
    if finite.size == 0:
        return {"finite_fraction": 0.0}

    return {
        "finite_fraction": float(finite.size / arr.size),
        "min": float(np.min(finite)),
        "p50": float(np.percentile(finite, 50)),
        "p95": float(np.percentile(finite, 95)),
        "p99": float(np.percentile(finite, 99)),
        "max": float(np.max(finite)),
        "mean": float(np.mean(finite)),
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Capture Scene, DepthPerspective, and Segmentation from an AirSim camera. "
            "The camera must already be configured with ProjectionMode=Equirectangular in settings.json."
        )
    )
    parser.add_argument("--ip", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=41451)
    parser.add_argument("--vehicle", default="drone_1")
    parser.add_argument("--camera", default="front_center")
    parser.add_argument("--timeout", type=int, default=120)
    parser.add_argument("--out-dir", type=Path, default=None)
    parser.add_argument("--depth-preview-max", type=float, default=40.0)
    parser.add_argument("--pause", action="store_true", help="pause the sim before capture")
    args = parser.parse_args()

    out_dir = args.out_dir or default_output_dir()
    out_dir.mkdir(parents=True, exist_ok=True)

    client = airsim.MultirotorClient(ip=args.ip, port=args.port, timeout_value=args.timeout)
    client.confirmConnection()
    if args.pause:
        client.simPause(True)

    requests = [
        airsim.ImageRequest(args.camera, image_type, pixels_as_float, False)
        for _, image_type, pixels_as_float in CAPTURES
    ]
    responses = client.simGetImages(requests, vehicle_name=args.vehicle)

    summary: dict[str, Any] = {
        "vehicle": args.vehicle,
        "camera": args.camera,
        "output_dir": str(out_dir),
        "depth_preview_max": args.depth_preview_max,
        "images": {},
    }

    for (name, image_type, pixels_as_float), response in zip(CAPTURES, responses):
        if response.width <= 0 or response.height <= 0:
            raise RuntimeError(f"{name} capture failed: {response.message}")

        item: dict[str, Any] = {
            "image_type": int(image_type),
            "width": int(response.width),
            "height": int(response.height),
            "message": response.message,
            "looks_equirectangular": int(response.width) == int(response.height) * 2,
        }

        if pixels_as_float:
            arr = response_float_array(response)
            npy_path = out_dir / f"{name}_equirectangular_float.npy"
            preview_path = out_dir / f"{name}_equirectangular_preview.png"
            np.save(npy_path, arr)
            save_depth_preview(arr, preview_path, args.depth_preview_max)
            item.update(summarize_float(arr))
            item["float_npy"] = str(npy_path)
            item["preview_png"] = str(preview_path)
        else:
            arr = response_u8_array(response)
            npy_path = out_dir / f"{name}_equirectangular_uint8.npy"
            png_path = out_dir / f"{name}_equirectangular.png"
            np.save(npy_path, arr)
            Image.fromarray(arr[:, :, :3]).save(png_path)
            item.update(summarize_u8(arr))
            item["uint8_npy"] = str(npy_path)
            item["png"] = str(png_path)

        if not item["looks_equirectangular"]:
            print(
                f"WARNING: {name} returned {response.width}x{response.height}, "
                "not a 2:1 equirectangular image. Check ProjectionMode in settings.json."
            )

        summary["images"][name] = item
        print(f"{name}: {response.width}x{response.height}")

    summary_path = out_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"\nSaved capture to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
