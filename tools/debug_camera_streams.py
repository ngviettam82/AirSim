#!/usr/bin/env python3
"""Show live AirSim camera streams for debugging.

Left panel:
  AirSim RPC camera stream, using simGetImage by default.

Right panel:
  Optional subwindow/viewport stream, either from a video URL/device that exposes
  the subwindow, or from a screen rectangle over the Unreal subwindow.
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
PYTHON_CLIENT = REPO_ROOT / "PythonClient"
if str(PYTHON_CLIENT) not in sys.path:
    sys.path.insert(0, str(PYTHON_CLIENT))

try:
    import cv2
except ImportError as exc:
    raise SystemExit("OpenCV is required for this viewer. Install it with: pip install opencv-python") from exc

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
}


@dataclass
class FrameState:
    frame: Optional[np.ndarray] = None
    status: str = "starting"
    error: str = ""
    fps: float = 0.0
    frames: int = 0
    latency_ms: float = 0.0


class LatestFrame:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state = FrameState()

    def set_status(self, status: str) -> None:
        with self._lock:
            self._state.status = status
            self._state.error = ""

    def set_error(self, error: str) -> None:
        with self._lock:
            self._state.error = error

    def set_frame(self, frame: np.ndarray, fps: float, frames: int, latency_ms: float) -> None:
        with self._lock:
            self._state.frame = frame
            self._state.status = "streaming"
            self._state.error = ""
            self._state.fps = fps
            self._state.frames = frames
            self._state.latency_ms = latency_ms

    def snapshot(self) -> FrameState:
        with self._lock:
            frame = None if self._state.frame is None else self._state.frame.copy()
            return FrameState(
                frame=frame,
                status=self._state.status,
                error=self._state.error,
                fps=self._state.fps,
                frames=self._state.frames,
                latency_ms=self._state.latency_ms,
            )


def parse_image_type(value: str) -> int:
    key = value.replace("_", "").replace("-", "").lower()
    if key in IMAGE_TYPES:
        return int(IMAGE_TYPES[key])
    try:
        return int(value)
    except ValueError as exc:
        names = ", ".join(sorted(IMAGE_TYPES))
        raise argparse.ArgumentTypeError(f"unknown image type {value!r}; use one of: {names}") from exc


def payload_to_bytes(payload: Any) -> bytes:
    if payload is None:
        return b""
    if isinstance(payload, bytes):
        return payload
    if isinstance(payload, str):
        return payload.encode("latin1")
    if isinstance(payload, bytearray):
        return bytes(payload)
    if isinstance(payload, memoryview):
        return payload.tobytes()
    return bytes(bytearray(payload))


def ensure_bgr(frame: np.ndarray) -> np.ndarray:
    if frame.ndim == 2:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    if frame.shape[2] == 4:
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    if frame.shape[2] == 3:
        return frame
    raise ValueError(f"unsupported frame shape {frame.shape}")


def decode_compressed_payload(payload: Any) -> np.ndarray:
    raw = payload_to_bytes(payload)
    if not raw:
        raise ValueError("empty image payload")
    arr = np.frombuffer(raw, dtype=np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
    if frame is None:
        raise ValueError(f"OpenCV could not decode compressed JPEG payload of {len(raw)} bytes")
    return ensure_bgr(frame)


def image_response_to_bgr(response: Any, float_preview_max: float, invert_float_preview: bool) -> np.ndarray:
    width = int(response.width)
    height = int(response.height)
    pixels = width * height
    if pixels <= 0:
        raise ValueError(response.message or "response has invalid dimensions")

    if response.pixels_as_float:
        image = airsim.response_to_2d_float_array(response)
        finite = np.isfinite(image)
        if finite.any():
            clean = np.where(finite, image, 0.0)
            if float_preview_max > 0:
                scale_max = float_preview_max
            else:
                scale_max = float(np.nanpercentile(clean[finite], 99))
            scale_max = max(scale_max, 1.0e-6)
            normalized = np.clip(clean / scale_max, 0.0, 1.0)
            if invert_float_preview:
                normalized = 1.0 - normalized
            gray = (normalized * 255.0 + 0.5).astype(np.uint8)
        else:
            gray = np.zeros((height, width), dtype=np.uint8)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    payload = payload_to_bytes(response.image_data_uint8)
    arr = np.frombuffer(payload, dtype=np.uint8)
    if response.compress:
        return decode_compressed_payload(payload)
    if arr.size == pixels * 4:
        rgba = arr.reshape((height, width, 4))
        return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
    if arr.size == pixels * 3:
        rgb = arr.reshape((height, width, 3))
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    raise ValueError(f"unexpected uint8 payload length {arr.size}; expected {pixels * 3} or {pixels * 4}")


def resize_letterbox(frame: np.ndarray, panel_width: int, panel_height: int) -> np.ndarray:
    if frame is None:
        return np.zeros((panel_height, panel_width, 3), dtype=np.uint8)

    height, width = frame.shape[:2]
    if height <= 0 or width <= 0:
        return np.zeros((panel_height, panel_width, 3), dtype=np.uint8)

    scale = min(panel_width / width, panel_height / height)
    target_width = max(1, int(width * scale))
    target_height = max(1, int(height * scale))
    resized = cv2.resize(frame, (target_width, target_height), interpolation=cv2.INTER_AREA)

    canvas = np.zeros((panel_height, panel_width, 3), dtype=np.uint8)
    y = (panel_height - target_height) // 2
    x = (panel_width - target_width) // 2
    canvas[y:y + target_height, x:x + target_width] = resized
    return canvas


def draw_text_block(panel: np.ndarray, lines: list[str], color: tuple[int, int, int]) -> None:
    y = 28
    for line in lines:
        cv2.putText(panel, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(panel, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
        y += 24


def compose_panel(state: FrameState, label: str, panel_width: int, panel_height: int) -> np.ndarray:
    panel = resize_letterbox(state.frame, panel_width, panel_height)
    header = [
        f"{label}",
        f"fps={state.fps:.2f} frames={state.frames} latency={state.latency_ms:.1f} ms",
    ]
    if state.error:
        header.append(f"ERROR: {state.error[:120]}")
        color = (70, 70, 255)
    elif state.frame is None:
        header.append(state.status)
        color = (80, 220, 255)
    else:
        color = (90, 255, 90)
    draw_text_block(panel, header, color)
    return panel


def update_fps(now: float, frame_counter: int, window_start: float, window_frames: int) -> tuple[float, float, int]:
    elapsed = now - window_start
    if elapsed >= 1.0:
        return window_frames / elapsed, now, 0
    return math.nan, window_start, window_frames


def simget_worker(args: argparse.Namespace, output: LatestFrame, stop_event: threading.Event) -> None:
    output.set_status(f"connecting to AirSim RPC {args.ip}:{args.port}")
    client = airsim.MultirotorClient(ip=args.ip, port=args.port, timeout_value=args.timeout)
    try:
        client.ping()
    except Exception as exc:  # noqa: BLE001
        output.set_error(f"RPC ping failed: {exc}")
        return

    image_type = parse_image_type(args.image_type)
    pixels_as_float = args.pixels_as_float or image_type in FLOAT_IMAGE_TYPES
    request = airsim.ImageRequest(
        args.camera,
        image_type,
        pixels_as_float,
        args.compress,
        args.annotation_name,
        args.float_as_bytes,
    )

    frames = 0
    window_frames = 0
    window_start = time.perf_counter()
    fps = 0.0
    output.set_status(f"waiting for first {args.api_method} frame")

    while not stop_event.is_set():
        started = time.perf_counter()
        try:
            if args.api_method == "simGetImage":
                payload = client.simGetImage(args.camera, image_type, args.vehicle, args.annotation_name)
                frame = decode_compressed_payload(payload)
            else:
                response = client.simGetImages([request], vehicle_name=args.vehicle)[0]
                frame = image_response_to_bgr(response, args.float_preview_max, args.invert_float_preview)

            now = time.perf_counter()
            frames += 1
            window_frames += 1
            new_fps, window_start, window_frames = update_fps(now, frames, window_start, window_frames)
            if not math.isnan(new_fps):
                fps = new_fps
            output.set_frame(frame, fps, frames, (now - started) * 1000.0)

            if args.max_fps > 0:
                sleep_s = max(0.0, (1.0 / args.max_fps) - (time.perf_counter() - started))
                if sleep_s > 0:
                    time.sleep(sleep_s)
        except Exception as exc:  # noqa: BLE001
            output.set_error(str(exc))
            time.sleep(0.25)


def parse_roi(value: str) -> tuple[int, int, int, int]:
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("ROI must be x,y,width,height")
    try:
        x, y, width, height = [int(part) for part in parts]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("ROI values must be integers") from exc
    if width <= 0 or height <= 0:
        raise argparse.ArgumentTypeError("ROI width and height must be positive")
    return x, y, width, height


def subwindow_video_worker(args: argparse.Namespace, output: LatestFrame, stop_event: threading.Event) -> None:
    source: Any = args.subwindow_url
    if source is None:
        output.set_status("no subwindow stream URL configured")
        return
    if isinstance(source, str) and source.isdigit():
        source = int(source)

    output.set_status(f"opening subwindow stream {source}")
    capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        output.set_error(f"could not open subwindow stream: {source}")
        return

    frames = 0
    window_frames = 0
    window_start = time.perf_counter()
    fps = 0.0
    try:
        while not stop_event.is_set():
            started = time.perf_counter()
            ok, frame = capture.read()
            if not ok or frame is None:
                output.set_error("stream read returned no frame")
                time.sleep(0.1)
                continue

            now = time.perf_counter()
            frames += 1
            window_frames += 1
            new_fps, window_start, window_frames = update_fps(now, frames, window_start, window_frames)
            if not math.isnan(new_fps):
                fps = new_fps
            output.set_frame(ensure_bgr(frame), fps, frames, (now - started) * 1000.0)
    finally:
        capture.release()


def subwindow_screen_worker(args: argparse.Namespace, output: LatestFrame, stop_event: threading.Event) -> None:
    if args.subwindow_roi is None:
        output.set_status("no screen ROI configured; pass --subwindow-roi x,y,w,h")
        return

    try:
        from PIL import ImageGrab
    except ImportError as exc:
        output.set_error("screen ROI capture needs Pillow: pip install pillow")
        return

    x, y, width, height = args.subwindow_roi
    bbox = (x, y, x + width, y + height)
    frames = 0
    window_frames = 0
    window_start = time.perf_counter()
    fps = 0.0
    output.set_status(f"capturing screen ROI {x},{y},{width},{height}")

    while not stop_event.is_set():
        started = time.perf_counter()
        try:
            image = ImageGrab.grab(bbox=bbox)
            rgb = np.asarray(image, dtype=np.uint8)
            frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

            now = time.perf_counter()
            frames += 1
            window_frames += 1
            new_fps, window_start, window_frames = update_fps(now, frames, window_start, window_frames)
            if not math.isnan(new_fps):
                fps = new_fps
            output.set_frame(frame, fps, frames, (now - started) * 1000.0)

            if args.subwindow_max_fps > 0:
                sleep_s = max(0.0, (1.0 / args.subwindow_max_fps) - (time.perf_counter() - started))
                if sleep_s > 0:
                    time.sleep(sleep_s)
        except Exception as exc:  # noqa: BLE001
            output.set_error(str(exc))
            time.sleep(0.25)


def disabled_subwindow_worker(args: argparse.Namespace, output: LatestFrame, stop_event: threading.Event) -> None:
    unused = (args, stop_event)
    output.set_status("subwindow source disabled")
    while not stop_event.wait(0.5):
        pass


def start_thread(target: Any, args: argparse.Namespace, output: LatestFrame, stop_event: threading.Event) -> threading.Thread:
    thread = threading.Thread(target=target, args=(args, output, stop_event), daemon=True)
    thread.start()
    return thread


def resolve_subwindow_source(args: argparse.Namespace) -> str:
    if args.subwindow_source != "auto":
        return args.subwindow_source
    if args.subwindow_url:
        return "url"
    if args.subwindow_roi:
        return "screen"
    return "disabled"


def run_viewer(args: argparse.Namespace) -> int:
    api_frame = LatestFrame()
    subwindow_frame = LatestFrame()
    stop_event = threading.Event()

    api_thread = start_thread(simget_worker, args, api_frame, stop_event)
    source = resolve_subwindow_source(args)
    if source == "url":
        subwindow_thread = start_thread(subwindow_video_worker, args, subwindow_frame, stop_event)
    elif source == "screen":
        subwindow_thread = start_thread(subwindow_screen_worker, args, subwindow_frame, stop_event)
    else:
        subwindow_thread = start_thread(disabled_subwindow_worker, args, subwindow_frame, stop_event)

    cv2.namedWindow(args.window_title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(args.window_title, args.panel_width * 2, args.panel_height)

    try:
        while not stop_event.is_set():
            left = compose_panel(api_frame.snapshot(), f"RPC {args.api_method}", args.panel_width, args.panel_height)
            right = compose_panel(subwindow_frame.snapshot(), f"Subwindow {source}", args.panel_width, args.panel_height)
            combined = np.hstack((left, right))
            cv2.imshow(args.window_title, combined)
            key = cv2.waitKey(args.ui_interval_ms) & 0xFF
            if key in (27, ord("q"), ord("x")):
                break
    finally:
        stop_event.set()
        api_thread.join(timeout=2.0)
        subwindow_thread.join(timeout=2.0)
        cv2.destroyWindow(args.window_title)

    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default="127.0.0.1", help="AirSim RPC host.")
    parser.add_argument("--port", type=int, default=41451, help="AirSim RPC port.")
    parser.add_argument("--timeout", type=int, default=180, help="AirSim RPC timeout in seconds.")
    parser.add_argument("--vehicle", default="drone_1", help="Vehicle name for the camera.")
    parser.add_argument("--camera", default="front_center", help="Camera name.")
    parser.add_argument("--image-type", default="Scene", type=str, help="Image type name or integer.")
    parser.add_argument("--annotation-name", default="", help="Annotation layer name for ImageType Annotation.")
    parser.add_argument("--api-method", choices=("simGetImage", "simGetImages"), default="simGetImage")
    parser.add_argument("--compress", action="store_true", help="Use compressed JPEG for simGetImages.")
    parser.add_argument("--pixels-as-float", action="store_true", help="Force float response mode for simGetImages.")
    parser.add_argument("--float-as-bytes", action="store_true", help="Request packed float32 bytes for depth simGetImages.")
    parser.add_argument("--float-preview-max", type=float, default=40.0, help="Float value mapped to white. Use 0 for p99 autoscale.")
    parser.add_argument("--invert-float-preview", action="store_true", help="Invert float preview brightness.")
    parser.add_argument("--max-fps", type=float, default=0.0, help="Optional RPC stream FPS cap. 0 means uncapped.")

    parser.add_argument(
        "--subwindow-source",
        choices=("auto", "url", "screen", "disabled"),
        default="auto",
        help="Where to read the subwindow side from.",
    )
    parser.add_argument(
        "--subwindow-url",
        default=None,
        help="Video URL/device for a subwindow stream, e.g. http://127.0.0.1:8080/video or 0.",
    )
    parser.add_argument(
        "--subwindow-roi",
        type=parse_roi,
        default=None,
        metavar="X,Y,W,H",
        help="Screen rectangle over the UE subwindow for live screen capture.",
    )
    parser.add_argument("--subwindow-max-fps", type=float, default=30.0, help="FPS cap for screen ROI capture.")
    parser.add_argument("--panel-width", type=int, default=960)
    parser.add_argument("--panel-height", type=int, default=480)
    parser.add_argument("--ui-interval-ms", type=int, default=15)
    parser.add_argument("--window-title", default="AirSim camera stream debugger")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return run_viewer(args)


if __name__ == "__main__":
    raise SystemExit(main())
