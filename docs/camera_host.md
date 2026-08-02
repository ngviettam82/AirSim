# Native Camera Host

The native camera host exposes settings-enabled AirSim camera/image-type pairs directly from the Unreal process. It provides a browser dashboard, an MJPEG preview URL, a JPEG snapshot URL, and an exact raw-data URL for every hosted pair. No Python relay is required.

## Configuration

Set `Host` on each `CaptureSettings` entry that should be exposed:

```json
{
  "SettingsVersion": 2.0,
  "SimMode": "Multirotor",
  "CameraHost": {
    "BindAddress": "0.0.0.0",
    "Port": 8080,
    "TargetFps": 30,
    "JpegQuality": 85,
    "FloatPreviewMax": 100,
    "MaxConnections": 64
  },
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "Cameras": {
        "front_stream": {
          "CaptureSettings": [
            { "ImageType": 0, "Width": 1280, "Height": 720, "Host": true },
            { "ImageType": 2, "Width": 640, "Height": 360, "Host": true },
            { "ImageType": 5, "Width": 640, "Height": 360, "Host": true }
          ]
        }
      }
    }
  }
}
```

`BindAddress` is a local interface on the simulator machine. Use `127.0.0.1` for local-only access, a specific local adapter address to bind one network, or `0.0.0.0` to accept connections on every adapter. A remote client connects to the simulator machine's reachable IP address, not to `0.0.0.0`.

The server starts only when at least one capture setting has `Host: true`. All routes share the configured port, while the vehicle name, camera name, and image type make each route independently addressable.

| Setting | Default | Purpose |
| --- | ---: | --- |
| `BindAddress` | `127.0.0.1` | Local IPv4 interface on which to listen. |
| `Port` | `8080` | HTTP listen port. |
| `TargetFps` | `30` | Maximum capture rate per vehicle. |
| `JpegQuality` | `85` | Quality from 1 to 100 for dashboard, MJPEG, and snapshot previews. |
| `FloatPreviewMax` | `100` | Float value mapped to the end of the grayscale preview range. Exact raw float values are unchanged. |
| `MaxConnections` | `64` | Maximum simultaneous dashboard, stream, snapshot, raw, and API connections. Range: 1 to 1024. |

For `ImageType: 11` (Annotation), also set `HostAnnotation` to the configured annotation layer name. Each annotation layer gets its own route.

## URLs

Open `http://127.0.0.1:8080/` for the dashboard. It shows every hosted route, supports click-to-focus and filtering, and can show or hide the measured capture FPS. Grid mode subscribes to every visible card. Focus mode disconnects every other card and keeps only the focused camera subscribed; click the focused card again to restore the grid subscriptions. If the focused route belongs to a vehicle-mounted camera, the dashboard also shows gimbal controls for pitch, yaw, roll, speed, and reset.

For the example above, the scene endpoints are:

```text
http://127.0.0.1:8080/camera/Drone1/front_stream/scene
http://127.0.0.1:8080/camera/Drone1/front_stream/scene/snapshot.jpg
http://127.0.0.1:8080/camera/Drone1/front_stream/scene/raw
```

The first URL is a continuous MJPEG stream and is suitable for browsers, OpenCV, and video players. The snapshot URL returns one JPEG frame. The raw URL returns one exact capture as `application/octet-stream` for machine processing. Use `GET /api/cameras` to discover all configured URLs and `GET /api/status` for measured FPS, latency, sequence, dimensions, subscribers, and capture errors.

Snapshot and raw routes return a newly captured frame by default. They also accept `?after=N`; when supplied, the request returns an available frame whose sequence is greater than `N`, or waits up to 10 seconds for one. This supports efficient per-camera polling without returning the same frame twice.

## Gimbal control

CameraHost can control the relative orientation of hosted vehicle-mounted cameras. It does not move the camera mount: only the camera's relative rotation is changed, and the original relative location stays pinned to the vehicle. Yaw is relative to the vehicle heading.

Use `GET /api/gimbals` to discover controllable cameras and their current state. Each entry includes the vehicle name, camera name, current orientation, initial orientation, motion state, mount-relative location, current relative location, and whether the location is pinned.

Send a gimbal command with `POST /api/gimbal` and a `text/plain` body containing exactly six whitespace-separated fields:

```text
vehicle camera pitch yaw roll speed
```

For example:

```text
drone1 cam1 -60 30 10 60
```

The angles are target pitch, yaw, and roll in degrees. Pitch must be within `-90..90`, while yaw and roll must be within `-180..180`. `speed` is the angular speed in degrees per second. CameraHost computes the travel time from the largest angular axis delta and reaches the target orientation at the end of that time. The command input does not accept JSON and does not accept a duration field.

The response is JSON containing the accepted camera state and motion state:

```json
{
  "vehicle_name": "drone1",
  "camera_name": "cam1",
  "orientation": { "pitch": 0, "yaw": 0, "roll": 0, "degrees": true },
  "motion": {
    "active": true,
    "duration_seconds": 1.0,
    "elapsed_seconds": 0.0,
    "remaining_seconds": 1.0,
    "target_orientation": { "pitch": -60, "yaw": 30, "roll": 10, "degrees": true }
  },
  "mount_relative_location": { "x": 50, "y": 0, "z": 0 },
  "current_relative_location": { "x": 50, "y": 0, "z": 0 },
  "location_pinned": true
}
```

A helper script is provided for command-line control:

```powershell
python tools\send_camera_gimbal_command.py drone1 cam1 -60 30 10 60
```

For a remote simulator machine, pass the reachable host address and port:

```powershell
python tools\send_camera_gimbal_command.py --host 192.168.1.20 --port 8080 drone1 cam1 -60 30 10 60
```

## Raw frame format

Raw response metadata is provided in HTTP headers:

- `X-AirSim-Sequence` and `X-AirSim-Timestamp`
- `X-AirSim-Width`, `X-AirSim-Height`, and `X-AirSim-Channels`
- `X-AirSim-Dtype`, `X-AirSim-Color-Order`, and `X-AirSim-Endianness`
- `X-AirSim-Capture-FPS` and `X-AirSim-Latency-Ms`

Color and label image types return packed `uint8` RGB data with shape `(height, width, 3)`. `DepthPlanar`, `DepthPerspective`, and `DisparityNormalized` return little-endian `float32` data with shape `(height, width)`. JPEG and MJPEG depth views are grayscale previews; the raw endpoint retains the original float values.

Example Python consumer:

```python
import numpy as np
import requests

url = "http://192.168.1.20:8080/camera/Drone1/front_stream/depthperspective/raw"
response = requests.get(url, timeout=15)
response.raise_for_status()

height = int(response.headers["X-AirSim-Height"])
width = int(response.headers["X-AirSim-Width"])
dtype = np.dtype("<f4" if response.headers["X-AirSim-Dtype"] == "float32" else "u1")
frame = np.frombuffer(response.content, dtype=dtype)
frame = frame.reshape(height, width) if dtype.itemsize == 4 else frame.reshape(height, width, 3)
```

Capture work is lazy: a hosted route renders only while an MJPEG client is connected or a snapshot/raw request is waiting. Requests for active routes belonging to the same vehicle are captured in one AirSim render batch, and multiple clients share the resulting frame. Raw-only clients receive the exact RGB or float payload without the preview RGB-to-RGBA conversion or JPEG encoding; that preview work begins only while an MJPEG or JPEG snapshot client is subscribed. Completed connection threads are reclaimed during operation, and `MaxConnections` bounds simultaneous connection resources.

## Network safety

The host has no authentication or TLS. Keep the default loopback binding unless remote access is required. For LAN access, restrict the port with the host firewall or a trusted reverse proxy, especially when gimbal control is enabled through hosted vehicle cameras. Do not expose it directly to the public internet.
