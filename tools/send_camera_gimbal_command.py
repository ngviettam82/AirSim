#!/usr/bin/env python3
"""Send a CameraHost gimbal control command.

Example:
    python tools/send_camera_gimbal_command.py drone1 cam1 -60 30 10 60

The command body sent to CameraHost is exactly:
    vehicle camera pitch yaw roll speed
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


def finite_float(value: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{value!r} is not a number") from exc
    if not math.isfinite(parsed):
        raise argparse.ArgumentTypeError(f"{value!r} is not finite")
    return parsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send a plain-text gimbal command to the AirSim CameraHost HTTP port.",
    )
    parser.add_argument("vehicle", help="AirSim vehicle name, for example: drone1")
    parser.add_argument("camera", help="AirSim camera name, for example: cam1")
    parser.add_argument("pitch", type=finite_float, help="Target pitch in degrees, -90..90")
    parser.add_argument("yaw", type=finite_float, help="Target yaw in degrees, -180..180, relative to the vehicle")
    parser.add_argument("roll", type=finite_float, help="Target roll in degrees, -180..180")
    parser.add_argument("speed", type=finite_float, help="Angular speed in degrees per second")
    parser.add_argument("--host", default="127.0.0.1", help="CameraHost address, default: 127.0.0.1")
    parser.add_argument("--port", type=int, default=8080, help="CameraHost HTTP port, default: 8080")
    parser.add_argument("--timeout", type=float, default=10.0, help="HTTP timeout in seconds, default: 10")
    return parser.parse_args()


def compact_number(value: float) -> str:
    return f"{value:.12g}"


def pretty_json(payload: bytes) -> str:
    try:
        decoded: Any = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return payload.decode("utf-8", errors="replace")
    return json.dumps(decoded, indent=2, sort_keys=True)


def main() -> int:
    args = parse_args()
    if args.speed <= 0.0:
        print("speed must be greater than 0", file=sys.stderr)
        return 2
    if args.pitch < -90.0 or args.pitch > 90.0:
        print("pitch must be within -90..90 degrees", file=sys.stderr)
        return 2
    if args.yaw < -180.0 or args.yaw > 180.0:
        print("yaw must be within -180..180 degrees", file=sys.stderr)
        return 2
    if args.roll < -180.0 or args.roll > 180.0:
        print("roll must be within -180..180 degrees", file=sys.stderr)
        return 2
    if args.port <= 0 or args.port > 65535:
        print("port must be between 1 and 65535", file=sys.stderr)
        return 2
    if args.timeout <= 0.0:
        print("timeout must be greater than 0", file=sys.stderr)
        return 2

    command = " ".join(
        [
            args.vehicle,
            args.camera,
            compact_number(args.pitch),
            compact_number(args.yaw),
            compact_number(args.roll),
            compact_number(args.speed),
        ]
    )
    url = f"http://{args.host}:{args.port}/api/gimbal"
    request = Request(
        url,
        data=command.encode("utf-8"),
        headers={"Content-Type": "text/plain; charset=utf-8"},
        method="POST",
    )

    try:
        with urlopen(request, timeout=args.timeout) as response:
            response_body = response.read()
            print(f"POST {url}")
            print(command)
            print(pretty_json(response_body))
            return 0
    except HTTPError as exc:
        error_body = exc.read()
        print(f"POST {url}", file=sys.stderr)
        print(command, file=sys.stderr)
        print(f"HTTP {exc.code} {exc.reason}", file=sys.stderr)
        if error_body:
            print(pretty_json(error_body), file=sys.stderr)
        return 1
    except URLError as exc:
        print(f"Could not connect to {url}: {exc.reason}", file=sys.stderr)
        return 1
    except TimeoutError:
        print(f"Timed out connecting to {url}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
