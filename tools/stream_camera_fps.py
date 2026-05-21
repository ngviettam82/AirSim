#!/usr/bin/env python3
"""Display a live AirSim camera stream and report capture FPS.

Use the same command for normal perspective/orthographic and equirectangular
captures. Projection is selected by the camera's settings.json entry.
"""

from __future__ import annotations

import argparse
import math
import queue
import sys
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
import tkinter as tk

REPO_ROOT = Path(__file__).resolve().parents[1]
PYTHON_CLIENT = REPO_ROOT / "PythonClient"
if str(PYTHON_CLIENT) not in sys.path:
    sys.path.insert(0, str(PYTHON_CLIENT))

import cosysairsim as airsim  # noqa: E402

IMAGE_TYPES = {
    "scene": airsim.ImageType.Scene,
    "depthplanar": airsim.ImageType.DepthPlanar,
    "depthperspective": airsim.ImageType.DepthPerspective,
    "depthvis": airsim.ImageType.DepthVis,
    "disparitynormalized": airsim.ImageType.DisparityNormalized,
    "segmentation": airsim.ImageType.Segmentation,
    "surfacenormals": airsim.ImageType.SurfaceNormals,
    "infrared": airsim.ImageType.Infrared,
    "opticalflow": airsim.ImageType.OpticalFlow,
    "opticalflowvis": airsim.ImageType.OpticalFlowVis,
    "lighting": airsim.ImageType.Lighting,
    "annotation": airsim.ImageType.Annotation,
}

FLOAT_IMAGE_TYPES = {
    airsim.ImageType.DepthPlanar,
    airsim.ImageType.DepthPerspective,
    airsim.ImageType.DisparityNormalized,
    airsim.ImageType.OpticalFlow,
}


def parse_image_type(value: str) -> int:
    key = value.replace("_", "").replace("-", "").lower()
    if key in IMAGE_TYPES:
        return int(IMAGE_TYPES[key])
    try:
        return int(value)
    except ValueError as exc:
        names = ", ".join(sorted(IMAGE_TYPES))
        raise argparse.ArgumentTypeError(f"unknown image type {value!r}; use one of: {names}") from exc


def euler_degrees_to_quaternion(pitch: float, roll: float, yaw: float) -> Any:
    pitch_rad = math.radians(pitch)
    roll_rad = math.radians(roll)
    yaw_rad = math.radians(yaw)

    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)

    return airsim.Quaternionr(
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def response_to_uint8_rgb(response: Any) -> np.ndarray:
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


def response_to_float_preview(response: Any, depth_max: float, invert: bool) -> np.ndarray:
    arr = np.asarray(response.image_data_float, dtype=np.float32)
    expected = int(response.width) * int(response.height)
    if expected <= 0:
        raise ValueError(response.message or "response has invalid dimensions")
    if arr.size != expected:
        raise ValueError(f"unexpected float payload length {arr.size}; expected {expected}")

    image = arr.reshape((response.height, response.width))
    finite = np.isfinite(image)
    if not finite.any():
        gray = np.zeros(image.shape, dtype=np.uint8)
    else:
        clean = np.where(finite, image, 0.0)
        if depth_max > 0:
            scale_max = depth_max
        else:
            scale_max = float(np.nanpercentile(clean[finite], 99))
        scale_max = max(scale_max, 1.0e-6)
        normalized = np.clip(clean / scale_max, 0.0, 1.0)
        if invert:
            normalized = 1.0 - normalized
        gray = (normalized * 255.0 + 0.5).astype(np.uint8)
    return np.repeat(gray[..., None], 3, axis=2)


def resize_nearest(rgb: np.ndarray, width: int, height: int) -> np.ndarray:
    source_height, source_width = rgb.shape[:2]
    if source_width == width and source_height == height:
        return rgb

    x_indices = np.linspace(0, source_width - 1, width).astype(np.int32)
    y_indices = np.linspace(0, source_height - 1, height).astype(np.int32)
    return rgb[y_indices][:, x_indices, :]


def fit_for_display(rgb: np.ndarray, max_width: int, max_height: int, exact_size: bool) -> np.ndarray:
    if exact_size:
        return resize_nearest(rgb, max_width, max_height)

    height, width = rgb.shape[:2]
    scale = max(width / max_width, height / max_height, 1.0)
    step = max(1, int(math.ceil(scale)))
    return rgb[::step, ::step, :]


def rgb_to_ppm(rgb: np.ndarray) -> bytes:
    rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
    height, width = rgb.shape[:2]
    return f"P6\n{width} {height}\n255\n".encode("ascii") + rgb.tobytes()


def latest_put(frame_queue: "queue.Queue[dict[str, Any]]", item: dict[str, Any]) -> None:
    while True:
        try:
            frame_queue.get_nowait()
        except queue.Empty:
            break
    frame_queue.put_nowait(item)


def capture_loop(args: argparse.Namespace, frame_queue: "queue.Queue[dict[str, Any]]", stop_event: threading.Event) -> None:
    client = airsim.MultirotorClient(ip=args.ip, port=args.port, timeout_value=args.timeout)
    latest_put(frame_queue, {"status": "connecting to AirSim RPC..."})
    client.ping()
    latest_put(frame_queue, {"status": "connected; configuring camera..."})

    if args.pose is not None:
        x, y, z = args.pose
        pose = airsim.Pose(
            airsim.Vector3r(x, y, z),
            euler_degrees_to_quaternion(args.pitch, args.roll, args.yaw),
        )
        client.simSetCameraPose(args.camera, pose, vehicle_name=args.vehicle)
        time.sleep(args.pose_settle_seconds)

    image_type = parse_image_type(args.image_type)
    pixels_as_float = args.pixels_as_float or image_type in FLOAT_IMAGE_TYPES
    request = airsim.ImageRequest(
        args.camera,
        image_type,
        pixels_as_float,
        False,
        args.annotation_name,
    )

    frame_count = 0
    window_count = 0
    started_at = time.perf_counter()
    window_started_at = started_at
    fps = 0.0
    rpc_ms_ewma = None

    latest_put(
        frame_queue,
        {
            "status": (
                f"waiting for first frame from camera={args.camera} image_type={args.image_type}; "
                "high-res equirectangular float depth can take many seconds"
            )
        },
    )
    print(
        f"Streaming camera={args.camera} image_type={args.image_type} "
        f"pixels_as_float={pixels_as_float} display={args.display_width}x{args.display_height}",
        flush=True,
    )

    while not stop_event.is_set():
        frame_started_at = time.perf_counter()
        try:
            response = client.simGetImages([request], vehicle_name=args.vehicle)[0]
            rpc_ms = (time.perf_counter() - frame_started_at) * 1000.0
            rpc_ms_ewma = rpc_ms if rpc_ms_ewma is None else rpc_ms_ewma * 0.9 + rpc_ms * 0.1
            if pixels_as_float:
                rgb = response_to_float_preview(response, args.float_preview_max, args.invert_float_preview)
            else:
                rgb = response_to_uint8_rgb(response)

            frame_count += 1
            window_count += 1
            now = time.perf_counter()
            elapsed = now - window_started_at
            if elapsed >= 1.0:
                fps = window_count / elapsed
                window_count = 0
                window_started_at = now
                print(
                    f"{args.camera} type={args.image_type} {response.width}x{response.height} "
                    f"fps={fps:.2f} rpc_ms={rpc_ms_ewma:.1f}",
                    flush=True,
                )

            latest_put(
                frame_queue,
                {
                    "rgb": rgb,
                    "width": int(response.width),
                    "height": int(response.height),
                    "fps": fps,
                    "rpc_ms": rpc_ms_ewma or rpc_ms,
                    "frames": frame_count,
                    "elapsed": now - started_at,
                },
            )

            if args.max_fps > 0:
                sleep_s = max(0.0, (1.0 / args.max_fps) - (time.perf_counter() - frame_started_at))
                if sleep_s > 0:
                    time.sleep(sleep_s)
        except Exception as exc:  # noqa: BLE001
            latest_put(frame_queue, {"error": str(exc)})
            time.sleep(0.25)


def run_window(args: argparse.Namespace) -> None:
    frame_queue: "queue.Queue[dict[str, Any]]" = queue.Queue(maxsize=1)
    stop_event = threading.Event()
    worker = threading.Thread(target=capture_loop, args=(args, frame_queue, stop_event), daemon=True)
    worker.start()

    root = tk.Tk()
    root.title("AirSim camera stream")
    canvas = tk.Canvas(root, width=args.display_width, height=args.display_height, highlightthickness=0)
    canvas.pack()
    placeholder = np.zeros((args.display_height, args.display_width, 3), dtype=np.uint8)
    placeholder_photo = tk.PhotoImage(data=rgb_to_ppm(placeholder), format="PPM")
    image_id = canvas.create_image(0, 0, anchor="nw", image=placeholder_photo)
    status_label = None
    if args.status_bar:
        status_label = tk.Label(root, anchor="w", justify="left", font=("Consolas", 10))
        status_label.pack(fill="x")
    photo_ref: dict[str, Any] = {"photo": None}

    def close() -> None:
        stop_event.set()
        root.destroy()

    def poll() -> None:
        try:
            item = frame_queue.get_nowait()
        except queue.Empty:
            root.after(args.ui_interval_ms, poll)
            return

        if "error" in item:
            root.title("AirSim camera stream - ERROR")
            if status_label is not None:
                status_label.configure(text=f"ERROR: {item['error']}")
        elif "status" in item:
            root.title(f"AirSim camera stream - {item['status']}")
            if status_label is not None:
                status_label.configure(text=item["status"])
        else:
            rgb = fit_for_display(item["rgb"], args.display_width, args.display_height, args.exact_display_size)
            photo = tk.PhotoImage(data=rgb_to_ppm(rgb), format="PPM")
            photo_ref["photo"] = photo
            canvas.itemconfigure(image_id, image=photo)
            stats = (
                f"camera={args.camera} image_type={args.image_type} "
                f"source={item['width']}x{item['height']} display={rgb.shape[1]}x{rgb.shape[0]} "
                f"fps={item['fps']:.2f} rpc_ms={item['rpc_ms']:.1f} frames={item['frames']}"
            )
            root.title(f"AirSim camera stream - {stats}")
            if status_label is not None:
                status_label.configure(text=stats)

        root.after(args.ui_interval_ms, poll)

    root.protocol("WM_DELETE_WINDOW", close)
    root.after(args.ui_interval_ms, poll)
    root.mainloop()
    stop_event.set()
    worker.join(timeout=2.0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=41451)
    parser.add_argument("--timeout", type=int, default=180)
    parser.add_argument("--vehicle", default="drone_1")
    parser.add_argument("--camera", default="front_center")
    parser.add_argument("--image-type", default="Scene", type=str)
    parser.add_argument("--annotation-name", default="")
    parser.add_argument("--pixels-as-float", action="store_true", help="Force float response mode.")
    parser.add_argument("--float-preview-max", type=float, default=20.0, help="Meters/value mapped to white for float previews. Use 0 for p99 autoscale.")
    parser.add_argument("--invert-float-preview", action="store_true", help="Invert float preview so near/small values are bright and far/large values are dark.")
    parser.add_argument("--display-width", type=int, default=1440)
    parser.add_argument("--display-height", type=int, default=720)
    parser.add_argument("--exact-display-size", action="store_true", help="Resize every frame to display-width x display-height.")
    parser.add_argument("--status-bar", action="store_true", help="Show a text status bar below the image. By default stats are shown in the window title and console only.")
    parser.add_argument("--ui-interval-ms", type=int, default=15)
    parser.add_argument("--max-fps", type=float, default=0.0, help="Optional client-side FPS cap. 0 means uncapped.")
    parser.add_argument("--pose", nargs=3, type=float, metavar=("X", "Y", "Z"), help="Optional camera pose in AirSim/NED relative coordinates.")
    parser.add_argument("--pitch", type=float, default=0.0, help="Optional pose pitch in degrees.")
    parser.add_argument("--roll", type=float, default=0.0, help="Optional pose roll in degrees.")
    parser.add_argument("--yaw", type=float, default=0.0, help="Optional pose yaw in degrees.")
    parser.add_argument("--pose-settle-seconds", type=float, default=0.2)
    args = parser.parse_args()

    run_window(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
