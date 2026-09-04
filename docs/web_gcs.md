# Web Companion Ground Control Station

`px4_airsim_gcs` is a production-grade, browser-based Web Companion Ground Control Station (GCS) designed for autonomous multirotors running PX4 Autopilot in AirSim simulation and real-world companion computers.

The architecture decouples the flight stack from heavy native desktop installations by providing an asynchronous **Python/ROS 2 gateway (`aiohttp`)** that streams 20 Hz telemetry over **WebSockets** and serves an **aviation-grade glass cockpit Single Page Application (SPA)** optimized for touch tablets (iPads, Android tablets) and desktop web browsers.

---

## 1. System Architecture

The gateway bridges ROS 2 topics/services/actions to standard web protocols:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      Client Browser / Mobile Tablet                     │
│  ┌───────────────────┐  ┌───────────────────┐  ┌─────────────────────┐  │
│  │ Primary Flight    │  │ 3x3 Obstacle      │  │ Multi-Layer Map     │  │
│  │ Display (PFD HUD) │  │ Clearance Radar   │  │ (Google/ESRI/OSM)   │  │
│  └─────────┬─────────┘  └─────────┬─────────┘  └──────────┬──────────┘  │
│            │                      │                       │             │
│            └───────────────┬──────┴───────────────────────┘             │
│                            │ JSON over WebSocket (/ws/telemetry @ 20Hz) │
│                            ▼ REST API (/api/flight, /api/autonomy)      │
└────────────────────────────┼────────────────────────────────────────────┘
                             │ HTTP / WebSocket (TCP Port 8080)
┌────────────────────────────┼────────────────────────────────────────────┐
│                            ▼                                            │
│             ROS 2 Gateway Node (`web_gcs_node.py`)                      │
│   - aiohttp Async Web Server (dedicated event loop in background thread)│
│   - 20 Hz WebSocket Telemetry Broadcaster (asyncio.Queue)               │
│   - REST API Command Handlers (Action mapping & parameter validation)   │
│   - Rotating Calipers Boustrophedon Photogrammetry Planner              │
│                            ▲                                            │
│                            │ ROS 2 Messages & Client Callbacks          │
└────────────────────────────┼────────────────────────────────────────────┘
                             │
     ┌───────────────────────┴───────────────────────┐
     │                                               │
     ▼                                               ▼
┌──────────────────────────────┐   ┌─────────────────────────────────────┐
│   `px4_airsim_autonomy`      │   │ AirSim Simulation / PX4 Autopilot   │
│   Enterprise Supervisor      │   │ - `/airsim_node/drone1/.../image`   │
│   - 50 Hz Control Loop       │   │ - `/fmu/out/vehicle_status`         │
│   - Obstacle Avoidance       │   │ - `/fmu/out/vehicle_odometry`       │
│   - Survey Photogrammetry    │   │ - `/fmu/out/battery_status`         │
│   - Standoff Visual Guiding  │   │ - `/fmu/in/trajectory_setpoint`     │
└──────────────────────────────┘   └─────────────────────────────────────┘
```

### Threading & Concurrency Model
* **ROS 2 Executor**: The node runs the standard `rclpy.spin()` executor on the main thread, handling subscription callbacks (`VehicleStatus`, `VehicleOdometry`, `BatteryStatus`, and sensor feeds).
* **Aiohttp Event Loop**: The web server and WebSocket connection pool execute in a dedicated daemon thread with its own `asyncio` event loop.
* **Telemetry Synchronization**: State updates from ROS 2 callbacks are serialized into a thread-safe telemetry cache protected by a reentrant mutex and pushed to WebSocket client queues at a regulated 20 Hz.

---

## 2. Cockpit UI Subsystems

The user interface is an aviation glass cockpit inspired by modern avionics suites (Garmin G3000, Collins Pro Line):

```
┌─────────────────────────────────────────────────────────────────────────┐
│ [● ONLINE]  [DISARMED]  [MODE: OFFBOARD]   BAT: 98% 16.5V   GPS: 18 SATS │
├───────────────────────────┬─────────────────────────────────────────────┤
│                           │                                             │
│   PRIMARY FLIGHT DISPLAY  │             SATELLITE MAP MANAGER           │
│         (PFD HUD)         │       (Google Hybrid / Satellite / OSM)     │
│   - Pitch Ladder (±90°)   │       - Real-Time Rotatable Vehicle Marker  │
│   - Roll Dial & Indicator │       - Live Trajectory Breadcrumbs         │
│   - Speed Tape (m/s)      │       - Interactive Polygon Survey Tool     │
│   - Altitude Tape (m AGL) │                                             │
│   - Heading Ribbon        │                                             │
├───────────────────────────┼─────────────────────────────────────────────┤
│   3×3 OBSTACLE RADAR      │          CAMERA STREAM & TRACKING           │
│   - Front/Left/Right Dist │       - Onboard AirSim Video Feed           │
│   - Stopping Bubble Alert │       - Click-to-Track Visual Guidance      │
│   - Clearance Color Rungs │                                             │
└───────────────────────────┴─────────────────────────────────────────────┘
```

### 1. Primary Flight Display (PFD HUD)
* **Canvas-Rendered Horizon**: Pitch ladder markings calibrated from $-90^\circ$ to $+90^\circ$, roll angle pointer arc with tick marks at $0^\circ, 10^\circ, 20^\circ, 30^\circ, 45^\circ, 60^\circ$, and aircraft boresight reticle.
* **Tapes & Ribbons**: Left tape indicates ground speed ($m/s$), right tape indicates barometric/optical altitude ($m$), and top ribbon displays current true magnetic yaw heading ($0^\circ - 360^\circ$).
* **Frame Rate**: Driven by `requestAnimationFrame` at 60 FPS with smooth linear interpolation between 20 Hz telemetry frames.

### 2. Directional Obstacle Radar & Safety Bubble
* **Directional Clearances**: Displays distance metrics for Front, Left, Right, Up, and Down sectors computed from projected 3D depth buffers.
* **Safety Rungs**:
  * 🟢 **Green (> 5.0 m)**: Clear airspace.
  * 🟡 **Yellow (2.5 m - 5.0 m)**: Proximity advisory; velocity damping active.
  * 🔴 **Red (< 2.5 m)**: Emergency stopping bubble breach; automatic braking.

### 3. Multi-Layer Geospatial Map Manager
* **Basemap Providers**:
  * **Google Hybrid**: High-resolution satellite photography with street labels and political boundaries.
  * **Google Satellite**: Pure photographic imagery without vector labels.
  * **Google Streets**: Standard Google Maps street and topography rendering.
  * **ESRI World Imagery**: Global commercial satellite basemap.
  * **OpenStreetMap**: Standard open-source cartographic basemap.
* **Dynamic Orientation**: Vehicle marker rotates with zero-latency heading transform to visually represent the drone's actual yaw.
* **Historical Trail**: Path breadcrumbs with automatic decimation to maintain high rendering performance during long missions.

### 4. Interactive Photogrammetry Survey Planner
* Operators can click polygon vertices directly on the satellite map to enclose any area of interest.
* The backend computes the optimal Boustrophedon sweep trajectory, generates parallel flight strips, calculates photo trigger positions, and renders the planned path overlay instantly on the map.

### 5. Camera Viewport & Click-to-Track Standoff Guiding
* Displays the live video stream from AirSim's camera API.
* Operators can click directly on any feature or object in the video stream. The interface translates pixel coordinates $(u, v)$ to normalized offsets and dispatches a target acquisition command to the autonomy supervisor.

---

## 3. Configuration & Launch

### Launch Arguments (`web_gcs.launch.py`)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `port` | `int` | `8080` | TCP port for HTTP static assets and WebSocket telemetry. |
| `host` | `string` | `0.0.0.0` | Network interface bind address (`0.0.0.0` binds all interfaces). |
| `vehicle_name` | `string` | `drone1` | Target vehicle name matching AirSim `settings.json`. |
| `camera_name` | `string` | `cam1` | Onboard camera name matching AirSim `settings.json`. |
| `camera_host_port` | `int` | `8000` | Port for AirSim Unreal Native CameraHost MJPEG stream. |
| `airsim_ip` | `string` | `127.0.0.1` | IP address where AirSim Unreal is running (Windows host). |

> [!IMPORTANT]
> **Port Coordination Warning**: Both Unreal Native CameraHost and Web GCS default to `8080` if unconfigured. To prevent `EADDRINUSE` port collision when running on the same host, AirSim Native CameraHost should be configured on port `8000` in `settings.json`, leaving port `8080` dedicated to the Web GCS dashboard.

### Execution Commands
Launch in your ROS 2 environment:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Standard launch (GCS on 8080, connecting to CameraHost on 8000)
ros2 launch px4_airsim_gcs web_gcs.launch.py

# Custom port or external AirSim IP launch
ros2 launch px4_airsim_gcs web_gcs.launch.py port:=8080 camera_host_port:=8000 airsim_ip:=192.168.1.100
```

---

## 4. Network & Field Tablet Operations

### Browser Access URLs

| Client Device | Connection URL | Notes |
|---|---|---|
| **Windows Host (Browser)** | **`http://localhost:8080`** | Windows 11 forwards localhost directly into WSL2. |
| **Windows Host (Loopback IP)** | **`http://127.0.0.1:8080`** | Explicit IPv4 loopback. |
| **Direct WSL2 Virtual IP** | **`http://<WSL_IP>:8080`** | Obtain via `wsl ip addr show eth0` (e.g. `172.27.16.75:8080`). |
| **Field Tablet (iPad / Android)** | **`http://<HOST_LAN_IP>:8080`** | Connects over local Wi-Fi. Camera streams auto-adapt to host IP. |

### Networking Caveats & Safety
* **Do not use `0.0.0.0` in browsers**: `0.0.0.0` is an `INADDR_ANY` bind wildcard for the server. Modern browsers (Chrome, Edge, Safari) reject `http://0.0.0.0:8080` with `ERR_ADDRESS_INVALID`.
* **Do not use Hyper-V Switch IP (`172.27.16.1`)**: In WSL2, `172.27.16.1` is the virtual gateway switch on Windows. The actual WSL2 IP is `172.27.16.x`.
* **Field Tablet LAN Access from WSL2**:
  If running Web GCS in WSL2 and accessing from a physical iPad/tablet on local Wi-Fi, enable Windows port forwarding in elevated PowerShell:
  ```powershell
  netsh interface portproxy add v4tov4 listenaddress=0.0.0.0 listenport=8080 connectaddress=<WSL_IP> connectport=8080
  netsh interface portproxy add v4tov4 listenaddress=0.0.0.0 listenport=8000 connectaddress=127.0.0.1 connectport=8000
  ```
  *(Or enable `networkingMode=mirrored` in `%USERPROFILE%\.wslconfig` on Windows 11 23H2+).*
* **Browser Insecure Context Notice**: Connecting via `http://<LAN_IP>:8080` is classified as an Insecure Context by browsers. Features requiring SSL (such as GPS device geolocation `navigator.geolocation`) are disabled unless running on `localhost` or behind an HTTPS reverse proxy.

---

## 5. REST API Reference

All requests and responses use `application/json` payloads.

### System Query Endpoint

#### `GET /api/system/status`
Returns the complete 37-field real-time vehicle telemetry state.
```http
GET /api/system/status HTTP/1.1
```
```json
{
  "connected": true,
  "armed": false,
  "px4_nav_state": "OFFBOARD",
  "flight_mode": "AUTONOMOUS",
  "battery_pct": 98.4,
  "battery_voltage": 16.52,
  "battery_current": 12.4,
  "lat": 47.641468,
  "lon": -122.140165,
  "alt_msl": 120.0,
  "alt_agl": 10.0,
  "groundspeed": 0.0,
  "heading_deg": 88.5,
  "active_algorithm": "",
  "navigation_tier": "Tier0_RtkFixed"
}
```

### Flight Commands

#### Arm Motors (`POST /api/flight/arm`)
```json
// Response
{
  "success": true,
  "action": "arm"
}
```

#### Disarm Motors (`POST /api/flight/disarm`)
```json
// Response
{
  "success": true,
  "action": "disarm"
}
```

#### Automated Takeoff (`POST /api/flight/takeoff`)
```json
// Request
{
  "altitude": 10.0
}
// Response
{
  "success": true,
  "action": "takeoff",
  "altitude": 10.0
}
```

#### Autonomous Land (`POST /api/flight/land`)
```json
// Response
{
  "success": true,
  "action": "land"
}
```

#### Return to Home (`POST /api/flight/rth`)
```json
// Response
{
  "success": true,
  "action": "rth"
}
```

#### Position Hold / Loiter (`POST /api/flight/hold`)
```json
// Response
{
  "success": true,
  "action": "hold"
}
```

---

### Autonomy Commands

#### Switch Active Autonomy Algorithm (`POST /api/autonomy/set_algorithm`)
```json
// Request
{
  "algorithm": "photogrammetry_survey"
}
// Response
{
  "success": true,
  "algorithm": "photogrammetry_survey"
}
```
Supported algorithms:
* `""` (Empty string: default standby hover-hold under 50 Hz supervisor protection)
* `"photogrammetry_survey"` (Aliases: `"survey"`, `"scanning_patrol"`, `"scanning"`)
* `"dynamic_avoidance"` (Aliases: `"avoidance"`, `"obstacle_avoidance"`)

#### Target Acquisition / Standoff Guiding (`POST /api/autonomy/target`)
Dispatches 3D Cartesian coordinates in the specified frame (e.g. Body FLU):
```json
// Request
{
  "x": 15.0,
  "y": 0.0,
  "z": 0.0,
  "frame_id": "body"
}
// Response
{
  "success": true,
  "target": [15.0, 0.0, 0.0]
}
```

---

### Mission Planning Endpoint

#### Plan Boustrophedon Survey (`POST /api/mission/plan_survey`)
Generates optimal Boustrophedon sweeps, waypoints, camera triggers, and GSD using Rotating Calipers.

```json
// Request
{
  "polygon": [
    {"lat": 47.641468, "lon": -122.140165},
    {"lat": 47.642500, "lon": -122.140165},
    {"lat": 47.642500, "lon": -122.138500},
    {"lat": 47.641468, "lon": -122.138500}
  ],
  "altitude": 40.0,
  "forward_overlap": 0.75,
  "side_overlap": 0.65,
  "flight_speed": 5.0
}
```

**Response Payload**:
```json
{
  "status": "success",
  "metrics": {
    "area_m2": 17042.8,
    "area_acres": 4.21,
    "optimal_angle_deg": 90.0,
    "strip_count": 8,
    "total_waypoints": 16,
    "estimated_photos": 160,
    "gsd_cm_per_px": 0.96,
    "flight_distance_m": 1280.5,
    "flight_time_s": 256.1
  },
  "waypoints": [
    {"lat": 47.641500, "lon": -122.140100, "alt": 40.0},
    {"lat": 47.642450, "lon": -122.140100, "alt": 40.0}
  ],
  "photo_triggers": [
    {"lat": 47.641520, "lon": -122.140100, "alt": 40.0}
  ]
}
```

---

## 6. WebSocket Telemetry Protocol

Clients establish a persistent WebSocket connection to `ws://<host>:<port>/ws/telemetry`. Telemetry frames are pushed at a steady **20 Hz**.

### Complete 37-Field Packet Schema
```json
{
  "timestamp": 1788511250.452,
  "connected": true,
  "armed": true,
  "nav_state": "OFFBOARD",
  "flight_mode": "AUTONOMOUS",
  "battery_pct": 98.4,
  "battery_v": 16.52,
  "battery_current_a": 12.45,
  "gps": {
    "lat": 47.641468,
    "lon": -122.140165,
    "alt": 125.42,
    "satellites": 18,
    "fix_type": 3,
    "eph": 0.35,
    "epv": 0.52
  },
  "attitude": {
    "roll": 1.25,
    "pitch": -0.84,
    "yaw": 88.52,
    "roll_rate_dps": 0.05,
    "pitch_rate_dps": -0.02,
    "yaw_rate_dps": 0.12
  },
  "velocity": {
    "vx": 2.41,
    "vy": 0.08,
    "vz": -0.05,
    "speed_mps": 2.41,
    "climb_mps": 0.05
  },
  "obstacle_sectors": {
    "front": 8.24,
    "left": 12.05,
    "right": 6.51,
    "up": 15.00,
    "down": 3.82,
    "min_clearance": 3.82
  },
  "bubble_penetrated": false,
  "autonomy_algorithm": "scanning_patrol",
  "autonomy_status": "Lawnmower sweep pass 2 of 6",
  "supervisor_state": "ACTIVE"
}
```

---

## 7. Photogrammetry Survey Math

The backend survey planner (`boustrophedon_survey_helper.py`) executes automated coverage path planning using rigorous optical photogrammetry formulas:

### 1. Ground Sampling Distance (GSD)
Calculated from the sensor dimensions and flight altitude:
$$GSD_{horiz} = \frac{H \cdot S_w}{f \cdot I_w}, \quad GSD_{vert} = \frac{H \cdot S_h}{f \cdot I_h}$$

Where:
* $H$: Flight altitude above ground level ($m$).
* $S_w, S_h$: Physical sensor width and height ($mm$).
* $f$: Optical focal length ($mm$).
* $I_w, I_h$: Image sensor resolution in pixels ($px$).

### 2. Overlap & Trigger Spacing
Given desired forward overlap $O_f \in [0.70, 0.85]$ and side overlap $O_s \in [0.60, 0.80]$:
$$D_{trigger} = I_h \cdot GSD \cdot (1 - O_f)$$
$$S_{strip} = I_w \cdot GSD \cdot (1 - O_s)$$

### 3. Rotating Calipers Minimum-Turn Optimization
To maximize battery efficiency, the algorithm finds the flight orientation $\theta^*$ that minimizes the total number of turns:
$$\theta^* = \arg\min_{\theta \in [0^\circ, 180^\circ)} \left\lceil \frac{W_{proj}(\theta)}{S_{strip}} \right\rceil$$

Where $W_{proj}(\theta)$ is the projected caliper width of the polygon's convex hull along normal vector $\mathbf{n}(\theta + 90^\circ)$.

---

## 8. Verification & Diagnostics

To verify the GCS gateway independently of the browser:
```bash
# Verify static assets delivery
curl -I http://localhost:8080/static/index.html

# Query system status API
curl http://localhost:8080/api/system/status

# Test survey calculation via curl
curl -X POST http://localhost:8080/api/mission/plan_survey \
  -H "Content-Type: application/json" \
  -d '{"polygon":[{"lat":47.641,"lon":-122.140},{"lat":47.642,"lon":-122.140},{"lat":47.642,"lon":-122.138},{"lat":47.641,"lon":-122.138}],"altitude":40.0}'
```

