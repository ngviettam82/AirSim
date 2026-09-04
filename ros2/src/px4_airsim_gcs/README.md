# PX4 + AirSim Web Companion Ground Control Station (`px4_airsim_gcs`)

A modern, browser-based, high-performance Web Companion Ground Control Station (GCS) designed for autonomous PX4 multirotors in AirSim simulation and real-world companion computers.

Built with an asynchronous **Python/ROS 2 gateway (`aiohttp`)** streaming 20 Hz telemetry over **WebSockets** and an **aviation-grade glass cockpit Single Page Application (SPA)** supporting touch tablets (iPads, Android tablets) and desktop web browsers.

---

## 1. System Architecture

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
                             │ HTTP / WebSocket (Port 8080)
┌────────────────────────────┼────────────────────────────────────────────┐
│                            ▼                                            │
│             ROS 2 Gateway Node (`web_gcs_node.py`)                      │
│   - aiohttp Async Web Server (Asyncio event loop in dedicated thread)   │
│   - 20 Hz WebSocket Telemetry Broadcaster                               │
│   - REST API Command Handlers                                           │
│   - Rotating Calipers Boustrophedon Photogrammetry Planner              │
│                            ▲                                            │
│                            │ ROS 2 Topics / Services / Actions          │
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

---

## 2. Cockpit UI Subsystems

### Primary Flight Display (PFD HUD)
* **Artificial Horizon & Pitch Ladder**: Dynamic pitch ladder (±90°) and roll angle arc with center boresight indicator rendered at 60 FPS on HTML5 Canvas.
* **Airspeed & Altitude Tapes**: Moving tape displays with digital readouts and vertical speed indication (VSI).
* **Compass Heading Ribbon**: Top-mounted heading ribbon displaying real-time magnetic/true yaw heading.

### Directional Obstacle Radar & Safety Bubble
* **3×3 Directional Sectors**: Visualizes real-time clearance distances (Front, Left, Right, Up, Down) derived from depth perception.
* **Dynamic Clearance Coloring**:
  - 🟢 **Green**: Safe (> 5.0 m)
  - 🟡 **Yellow**: Caution (2.5 m - 5.0 m)
  - 🔴 **Red**: Emergency Braking Alert (< 2.5 m)
* **Stopping Bubble Status**: Indicates if the velocity-aligned dynamic stopping ellipsoid has been penetrated.

### Multi-Layer Geospatial Map Manager
* **Supported Tile Providers**:
  - **Google Hybrid** (Satellite imagery with road and label overlays)
  - **Google Satellite** (High-resolution aerial photography)
  - **Google Streets** (Standard cartographic vector street map)
  - **ESRI World Imagery** (High-resolution global satellite basemap)
  - **OpenStreetMap** (Open-source standard street tiles)
* **Real-Time Drone Marker**: Rotates dynamically to match the vehicle's true yaw heading.
* **Breadcrumb Flight Path**: Live polyline rendering historical flight trajectory.
* **Interactive Survey Polygon Drawer**: Click-to-draw polygonal boundaries for automated survey calculation.

### Live Camera Stream & Click-to-Track Standoff Guiding
* **Low-Latency Video Viewport**: Displays AirSim onboard camera feed via MJPEG/RTSP or periodic snapshot refresh.
* **Click-to-Track Targeting**: Click anywhere on the video feed to extract image coordinates $(u, v)$ and transmit visual guidance setpoints to the autonomy supervisor.

---

## 3. Quickstart & Launch Instructions

### Prerequisites
In your ROS 2 Humble environment (Ubuntu 22.04 or WSL2):
```bash
sudo apt-get update
sudo apt-get install -y python3-aiohttp ros-humble-px4-msgs
```

### Building the Package
```bash
cd ~/ros2_ws
colcon build --packages-select px4_airsim_gcs
source install/setup.bash
```

### Launching the Web GCS
Run via the project launch file:
```bash
ros2 launch px4_airsim_gcs web_gcs.launch.py
```

Optional launch arguments:
* `port`: Web server TCP port (default: `8080`)
* `host`: Network bind address (default: `0.0.0.0` for all adapters)
* `vehicle_name`: Vehicle namespace (default: `drone1`)
* `camera_name`: Camera stream source name (default: `cam1`)
* `camera_host_port`: AirSim CameraHost streaming port (default: `8000`)
* `airsim_ip`: AirSim host IP address (default: `127.0.0.1`)

> [!IMPORTANT]
> **Port Coordination**: Set `"Port": 8000` under `CameraHost` in AirSim's `settings.json` to avoid collision with Web GCS port `8080`.

Example with custom port:
```bash
ros2 launch px4_airsim_gcs web_gcs.launch.py port:=8080 camera_host_port:=8000
```

---

## 4. Connecting from Your Browser

Once launched, the GCS server is available across multiple network addresses:

| Client Device / Location | Target URL |
| :--- | :--- |
| **Local Machine (Windows Host / Linux Desktop)** | **`http://localhost:8080`** or **`http://127.0.0.1:8080`** |
| **WSL2 Direct VM IP** | **`http://<WSL_IP>:8080`** (e.g. `http://172.27.16.75:8080`) |
| **Local WiFi Network (iPad, iPhone, Android, Laptop)** | **`http://<YOUR_PC_LAN_IP>:8080`** (e.g. `http://192.168.1.50:8080`) |

> **Note on Windows / WSL2 Networking:**
> * Do **not** use `http://0.0.0.0:8080` in web browsers (`0.0.0.0` is a server bind address; browsers reject it as an invalid destination).
> * Do **not** use `172.27.16.1` (this is the Windows-side Hyper-V virtual switch gateway, not the server).
> * Windows 11 automatically forwards `localhost:8080` directly into WSL2.

---

## 5. REST API Specification

All REST API endpoints accept and return `application/json`.

### System Query
* **`GET /api/system/status`**: Returns the complete 37-field vehicle state dictionary.

### Flight Control Endpoints
* **`POST /api/flight/arm`**: Arms motors $\rightarrow$ `{"success": true, "action": "arm"}`
* **`POST /api/flight/disarm`**: Disarms motors $\rightarrow$ `{"success": true, "action": "disarm"}`
* **`POST /api/flight/takeoff`**: `{"altitude": 10.0}` $\rightarrow$ `{"success": true, "action": "takeoff", "altitude": 10.0}`
* **`POST /api/flight/land`**: Automated vertical descent $\rightarrow$ `{"success": true, "action": "land"}`
* **`POST /api/flight/rth`**: Return to launch position $\rightarrow$ `{"success": true, "action": "rth"}`
* **`POST /api/flight/hold`**: Loiter / position hold $\rightarrow$ `{"success": true, "action": "hold"}`

### Autonomy Control Endpoints
* **`POST /api/autonomy/set_algorithm`**:
  ```json
  {"algorithm": "photogrammetry_survey"}
  ```
  Response: `{"success": true, "algorithm": "photogrammetry_survey"}`  
  *(Supported: `""` hover-hold standby, `"photogrammetry_survey"`, `"dynamic_avoidance"`)*

* **`POST /api/autonomy/target`**:
  ```json
  {"x": 15.0, "y": 0.0, "z": 0.0, "frame_id": "body"}
  ```
  Response: `{"success": true, "target": [15.0, 0.0, 0.0]}`

### Photogrammetry Survey Planner Endpoint
* **`POST /api/mission/plan_survey`**:
  ```json
  {
    "polygon": [{"lat": 47.641,"lon": -122.140},{"lat": 47.642,"lon": -122.140},{"lat": 47.642,"lon": -122.138},{"lat": 47.641,"lon": -122.138}],
    "altitude": 40.0,
    "speed": 5.0,
    "forward_overlap": 0.75,
    "side_overlap": 0.65
  }
  ```
  Response:
  ```json
  {
    "success": true,
    "optical": {"gsd_cm": 0.96, "footprint_width_m": 38.4, "footprint_height_m": 21.6, "trigger_interval_m": 5.4, "strip_spacing_m": 13.44, "max_speed_mps": 9.6},
    "mission": {"optimal_heading_deg": 90.0, "strip_count": 8, "waypoint_count": 16, "trigger_count": 160, "total_distance_m": 1280.5, "flight_time_s": 256.1, "area_m2": 17042.8, "area_acres": 4.21},
    "waypoints": [{"lat": 47.6415, "lon": -122.1401, "alt": 40.0}],
    "photo_triggers": [{"lat": 47.64152, "lon": -122.1401, "alt": 40.0}]
  }
  ```

---

## 6. WebSocket Real-Time Telemetry (`/ws/telemetry`)

The GCS streams complete vehicle state at **20 Hz** in JSON format over `ws://<host>:<port>/ws/telemetry`.

### Ground Truth 37-Field Packet Schema
```json
{
  "timestamp_ms": 1788511250452,
  "connected": true,
  "armed": true,
  "px4_nav_state": "OFFBOARD",
  "flight_mode": "AUTONOMOUS",
  "active_algorithm": "photogrammetry_survey",
  "navigation_tier": "Tier0_RtkFixed",
  "lat": 47.641468,
  "lon": -122.140165,
  "alt_msl": 120.0,
  "alt_agl": 10.0,
  "satellites": 18,
  "gps_fix_type": "RTK Fixed",
  "eph": 0.08,
  "roll_deg": 1.25,
  "pitch_deg": -0.84,
  "yaw_deg": 88.52,
  "heading_deg": 88.52,
  "pos_enu": [0.0, 0.0, 10.0],
  "vel_enu": [2.41, 0.08, -0.05],
  "groundspeed": 2.41,
  "vertical_speed": 0.05,
  "battery_pct": 98.4,
  "battery_voltage": 16.52,
  "battery_current": 12.45,
  "obstacle_sectors": [
    [25.0, 25.0, 25.0],
    [12.0, 8.24, 6.51],
    [25.0, 3.82, 25.0]
  ],
  "obstacle_clearance": {"front": 8.24, "left": 12.0, "right": 6.51, "upper": 25.0, "lower": 3.82, "min": 3.82},
  "min_depth_m": 3.82,
  "target_detected": false,
  "target_pos_flu": [0.0, 0.0, 0.0],
  "smart_rth_status": "Nominal",
  "smart_rth_margin_pct": 28.5,
  "home_lat": 47.641468,
  "home_lon": -122.140165,
  "home_alt": 120.0,
  "camera_stream_url": "http://127.0.0.1:8000/camera/drone1/cam1/scene"
}
```

---

## 7. Photogrammetry Math & Survey Formulas

The built-in survey planner uses the following formulas:

1. **Ground Sampling Distance (GSD)**:
   $$GSD = \frac{H \cdot S_w}{f \cdot I_w}$$
   Where $H$ is altitude AGL (m), $S_w$ is sensor width (mm), $f$ is focal length (mm), and $I_w$ is image width (px).

2. **Trigger & Strip Spacing**:
   * Forward Trigger Distance: $D_{trigger} = I_h \cdot GSD \cdot (1 - O_f)$
   * Side Strip Spacing: $S_{strip} = I_w \cdot GSD \cdot (1 - O_s)$
   Where $O_f$ is forward overlap ratio (e.g. 0.75) and $O_s$ is side overlap ratio (e.g. 0.65).

3. **Rotating Calipers Minimum-Turn Optimization**:
   Evaluates polygon bounding box width $W_{proj}(\theta)$ for angles $\theta \in [0^\circ, 180^\circ)$ to minimize total required flight strips:
   $$\theta^* = \arg\min_{\theta} \left\lceil \frac{W_{proj}(\theta)}{S_{strip}} \right\rceil$$

---

## 8. Directory Structure

```
ros2/src/px4_airsim_gcs/
├── CMakeLists.txt / package.xml     # ROS 2 package configuration
├── setup.py / setup.cfg             # Python console script & asset installation
├── launch/
│   └── web_gcs.launch.py           # Launch file (port, host, namespaces)
├── px4_airsim_gcs/
│   ├── __init__.py
│   ├── web_gcs_node.py             # Aiohttp HTTP/WS gateway & ROS 2 node
│   └── boustrophedon_survey_helper.py # Rotating Calipers photogrammetry planner
└── static/                          # Glass Cockpit SPA frontend
    ├── index.html                  # Cockpit layout & canvas containers
    ├── css/
    │   └── gcs.css                 # Dark aviation styling & responsive layout
    └── js/
        ├── app.js                  # App orchestrator & WebSocket consumer
        ├── pfd_hud.js              # Artificial horizon HUD (Canvas 60FPS)
        ├── obstacle_radar.js       # 3x3 obstacle sector radar
        └── map_manager.js          # Leaflet map manager (Google/ESRI/OSM)
```

