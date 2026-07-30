# Recording Data (Cameras + Sensors + ROS 2 MCAP)

## Synchronization model (frame-latched dual-rate)

Recording never calls the simulator pause API or changes `ClockSpeed`. In World mode, a short state snapshot holds the physics-update mutex, which can serialize one concurrent physics update; it is released before scene capture, GPU readback, compression, or file I/O. No recording path holds physics while an image is rendered or written.

1. A dedicated sampler produces **sensor / pose rows** at `SensorRecordInterval` (default **0.005 s, about 200 Hz** target).
2. A separate image worker schedules cameras at `RecordInterval` (default **0.05 s, about 20 Hz** target). Its short state snapshot can contend for the same mutex; GPU readback, compression, and image file I/O do not.
3. Immediately before `CaptureScene`, the game thread snapshots pose/sensors and updates the vehicle's rendered state from that snapshot. The mutex is released before GPU rendering, readback, compression, or file writing completes.
4. Image rows use `AssociationMode=frame_latched_freerun`. Sensor-only rows use `AssociationMode=sensor_only`.
5. Every valid image is retained even when its timestamp delta exceeds `ImageSyncToleranceMs`. Default output saves it as a file with timing metadata; Rosbag output embeds it in the MCAP file with the rendered-frame timestamp.

This is a logical vehicle/render-frame latch, not a frozen-world or continuous-time shutter guarantee. Other independently moving world actors continue normally apart from a briefly delayed physics update while the snapshot mutex is held. The recorded delta is `ImageTimeStamp - FrameTimeStamp`; use its absolute value when applying a synchronization threshold.

| Field | Meaning |
|-------|---------|
| `FrameTimeStamp` | Sim-clock ns at sensor/pose sample |
| `PhysicsStepID` | ClockFactory step count at sample |
| `SequenceID` | Monotonic id within the session (restarts each session) |
| `RenderFrameNumber` | UE frame containing the image (0 for sensor-only rows) |
| `S_TimeStamp` | Native sensor output timestamp (never rewritten) |
| `S_Age` | `FrameTimeStamp - S_TimeStamp` (ns) |
| `AssociationMode` | `sensor_only`, `frame_latched_freerun`, or a failure mode such as `frame_latched_capture_failed`, `frame_latched_capture_incomplete`, or `frame_latched_encode_failed` |
| `ImageRequestTimeStamp` | Sim-clock ns immediately before the render request is submitted |
| `ImageTimeStamp` | Sim-clock ns for the rendered UE frame, recorded before GPU readback |
| `ImageDelayNs` / `ImageDelayMs` | Signed `ImageTimeStamp - FrameTimeStamp` |
| `ImageSyncWithinTolerance` | `1` when `abs(ImageDelayMs) <= ImageSyncToleranceMs`, otherwise `0` |
| Image filename | Includes snapshot/render timestamps plus `_dtp...us_ok`, `_dtm...us_ok`, or `_..._over` |

Default multirotor sensor names are **lowercase** (`imu`, `gps`, `barometer`, `magnetometer`). Selection is **case-insensitive**.

## Settings example

```json
"Recording": {
  "Output": "Default",
  "RecordInterval": 0.05,
  "SensorRecordInterval": 0.005,
  "ImageSyncToleranceMs": 5.0,
  "Cameras": [
    { "CameraName": "0", "ImageType": 0, "VehicleName": "drone1", "Compress": true }
  ],
  "Sensors": [
    { "VehicleName": "drone1", "SensorName": "imu" },
    { "VehicleName": "drone1", "SensorName": "gps" },
    { "VehicleName": "drone1", "SensorName": "barometer" },
    { "VehicleName": "drone1", "SensorName": "magnetometer" }
  ]
}
```

- `RecordInterval` - minimum sim time between **camera** captures.
- `SensorRecordInterval` - minimum sim time between **sensor/pose** rows.
- `ImageSyncToleranceMs` - flag threshold only; images outside the threshold are still saved.
- `Output` - selects exactly one output method. `"Default"` (the default) writes `airsim_rec.txt` and `images/`. `"Rosbag"` writes only an MCAP file, so it never creates a duplicate text log or image folder.
- `Rosbag.FileName` - optional MCAP filename in that folder (default `airsim_rec.mcap` when omitted; path components are removed). An explicitly empty value is a settings error.
- Configure the `Rosbag` object only when `Output` is `"Rosbag"`; use `Output` to select the recording method.
- In Rosbag mode, `"Cameras": []` produces a sensor-only MCAP; it does not create an empty `images/` directory or a TSV file.
- `Rosbag.MaxImuBufferSamples` - bounded native history per IMU selected in `Recording.Sensors` (default `4096` when omitted; valid range `1`–`65536`). Invalid values are settings errors. Oldest samples are dropped only if this fills; each loss is recorded on an `imu_drops` metadata topic.
- `"Cameras": []` — sensors/pose only at `SensorRecordInterval`.
- Missing sensors: `S_Present=0`, empty value fields (column count fixed).
- `S_Present=1` only when `time_stamp != 0`. GPS also has `S_GPS_Valid`.
- Distance honors `UpdateLatency`, `UpdateFrequency`, `StartupDelay`, and `UncorrelatedNoiseSigma`.

## Default output columns

Base (12): `VehicleName`, `SequenceID`, `PhysicsStepID`, `RenderFrameNumber`, `FrameTimeStamp`, `POS_*`, `Q_*`

Per sensor token `S` (32): Present, TimeStamp, Age, 10 IMU, 10 GPS, 3 Baro, 3 Mag, 3 Distance (type-specific filled; others empty).

Car/Skid extras: `Throttle`, `Steering`, `Brake`, `Gear`, `Handbrake`, `RPM`, `Speed`.

Then: `AssociationMode`, `ImageRequestTimeStamp`, `ImageTimeStamp`, `ImageDelayNs`, `ImageDelayMs`, `ImageSyncWithinTolerance`, `ImageFile`.

Multiple images on one row use semicolon-separated values in the same order. Sensor-only rows leave the image timing and filename fields empty. These TSV columns and external image files exist only for `Output: "Default"`.

## ROS 2 MCAP bag

Set `Recording.Output` to `"Rosbag"` to write `airsim_rec.mcap` from the Unreal recorder's ordered writer worker. The file uses the MCAP ROS 2 profile (`ros2`), `ros2msg` schemas, CDR payloads, and per-channel ROS 2 QoS metadata. It is a direct MCAP file, not a separate RPC poller: it shares the recorder's frame-latched camera transaction. A Rosbag session contains only the MCAP file after successful finalization; it does not create `airsim_rec.txt` or `images/`.

This is the **data-gathering** path, not a live ROS transport. For real-time processing, run the ROS 2 wrapper separately; it can run while MCAP recording is active, but it does not tail or republish the MCAP file.

```json
"Recording": {
  "Output": "Rosbag",
  "RecordInterval": 0.05,
  "SensorRecordInterval": 0.005,
  "Rosbag": {
    "FileName": "airsim_rec.mcap",
    "MaxImuBufferSamples": 4096
  },
  "Cameras": [
    { "CameraName": "0", "ImageType": 0, "VehicleName": "drone1", "Compress": true }
  ],
  "Sensors": [
    { "VehicleName": "drone1", "SensorName": "imu" },
    { "VehicleName": "drone1", "SensorName": "gps" }
  ]
}
```

The direct-bag topic set is:

| Topic | ROS 2 type | Timestamp / contents |
|---|---|---|
| `/airsim_node/<vehicle>/imu/<sensor>` | `sensor_msgs/msg/Imu` | Every retained native update for an IMU explicitly selected in `Recording.Sensors`, stamped with its native IMU timestamp. Values use the same NED-to-ROS conversion as the ROS 2 wrapper. |
| `/airsim_node/<vehicle>/gps/<sensor>` | `sensor_msgs/msg/NavSatFix` | The latest unique GPS output observed by the sensor sampler, stamped with that output's native timestamp. A valid AirSim 2D/3D fix maps to ROS `STATUS_FIX`; no-fix/time-only data maps to `STATUS_NO_FIX`. Service is `SERVICE_GPS`; covariance is unknown. |
| `/airsim_node/<vehicle>/altimeter/<sensor>` | `airsim_interfaces/msg/Altimeter` | The latest unique barometer output observed by the sensor sampler, stamped with its native timestamp. It contains altitude (m), pressure (Pa), and QNH. |
| `/airsim_node/<vehicle>/magnetometer/<sensor>` | `sensor_msgs/msg/MagneticField` | The latest unique magnetometer output observed by the sensor sampler, stamped with its native timestamp. AirSim's Gauss/NED-body output is converted to Tesla/ROS-body axes; covariance is converted when the source provides all nine values, otherwise it is unknown. |
| `/airsim_node/<vehicle>/<camera>_<type>[_<annotation>]/image` | `sensor_msgs/msg/Image` | Raw RGB (`rgb8`) or float depth (`32FC1`) images, stamped with the rendered camera frame time. |
| `/airsim_node/<vehicle>/<camera>_<type>[_<annotation>]/image/compressed` | `sensor_msgs/msg/CompressedImage` | JPEG output for `Compress=true` or `ImageFormat="jpeg"`. The message `format` field is `jpeg`; the image is stamped with the rendered camera frame time. |
| `/airsim_node/<vehicle>/<camera>_<type>[_<annotation>]/camera_info` | `sensor_msgs/msg/CameraInfo` | Paired with each valid perspective image: the same timestamp, MCAP log time, and optical frame ID. It is not written for equirectangular or orthographic captures. |
| `/airsim_node/<vehicle>/recording/image_metadata` | `std_msgs/msg/String` | JSON with the latch sequence, physics step, NED pose, request/render timestamps, and signed render delay. |
| `/airsim_node/<vehicle>/recording/imu_drops` | `std_msgs/msg/String` | JSON loss report if a bounded IMU history overflows. |

`Recording.Sensors` is the complete direct-sensor selection: list each IMU, GPS, barometer, or magnetometer wanted in the bag. A missing or empty list produces an image-only MCAP bag and starts no IMU history. GPS, barometer, and magnetometer are collected by the `SensorRecordInterval` latest-value sampler and are de-duplicated by native sensor timestamp. This avoids writing the same 50 Hz output multiple times when the sampler runs at its default 200 Hz. Unlike IMU, they do not have a native source-history ring, so updates faster than the sampler can be missed; configure the interval at or below the source update period when that matters. Default output records Distance in TSV columns; Rosbag output has no direct-MCAP `sensor_msgs/msg/Range` channel.

The direct GPS, barometer, and magnetometer MCAP channels are populated by multirotor recording captures. Rosbag output rejects selected sensors on Car, SkidVehicle, and ComputerVision vehicles rather than silently writing no sensor messages. Default TSV sensor rows are unchanged.

Each `Recording.Cameras` entry can name a separate vehicle, camera, image type, and annotation layer. Multiple cameras and ImageType values `0` through `11` can therefore coexist in one bag: Scene, depth variants, disparity, segmentation, surface normals, infrared, optical flow, lighting, and annotation. `ImageType: -1` configures the camera's main component (for example gimbal behavior); it is not a renderable recording image and is rejected in `Recording.Cameras`. Use raw RGB or float output for pixel-exact labels/depth; JPEG is deliberately lossy. `CameraInfo` is emitted only when the capture has a usable perspective FOV, not for orthographic or equirectangular output.

Dynamic bag topic tokens are made valid for ROS 2. Existing ASCII letter-leading names with no repeated underscores are unchanged; a name such as the default camera `"0"` becomes `camera_0_haf63ad4c86019caf`. Invalid characters and repeated underscores are normalized, very long names are shortened with the same stable hash, and a rare normalized-name collision receives a bounded `_collision_<hash>` suffix. The live wrapper uses the same normalization rule for camera names. Use ROS 2-compatible vehicle and annotation names when matching bag and live topic paths, because nonconforming identifiers can normalize differently between the two paths. Image metadata retains the original AirSim camera and annotation names.

Images, compressed images, `CameraInfo`, IMU, GPS, altimeter, and magnetometer channels use ROS 2 `SensorDataQoS`: keep-last depth 5, best-effort, volatile. The two low-rate recording metadata channels use reliable, keep-last depth 10, volatile QoS instead.

For every direct-bag image, the ROS `Header.stamp` and MCAP log time are the valid rendered camera-frame timestamp (`ImageTimeStamp`). There is no fallback to the earlier physics latch, request time, readback completion, or writer time: if the render path cannot provide that timestamp, the writer logs a warning and omits that image from the MCAP file. `image_metadata` preserves the earlier `FrameTimeStamp` physics/vehicle-render latch and the signed `ImageTimeStamp - FrameTimeStamp` delay, so fusion can apply a stricter state-association filter. This does **not** claim a frozen-world shutter or a hard camera-to-IMU time bound: at a nominal 333 Hz IMU rate, one sample period is about 3 ms, but fusion should use timestamps and interpolation rather than assume a fixed maximum displacement.

`ImageFormat="jpeg"` captures the same raw RGB render result and encodes it only on the recorder writer worker. JPEG encode time therefore cannot change the image's source timestamp or its associated rendered frame, but JPEG is lossy and should not be used for pixel-exact labels, segmentation IDs, or depth.

For a perspective capture with a finite horizontal FOV, the paired `CameraInfo` uses `fx = fy = width / (2 * tan(horizontal_fov / 2))`, centered principal point, identity `R`, and matching `P`. Its `distortion_model` is empty and `D` is empty: Unreal post-process lens distortion is not asserted to be a ROS camera-distortion model, so no `plumb_bob` coefficients are fabricated. Equirectangular and orthographic captures remain uncalibrated.

`image_metadata.latched_pose_ned` is simulator ground-truth vehicle pose captured at `FrameTimeStamp`; it is useful for offline labels, but it is JSON metadata tied to image records, not a continuous standard `PoseStamped`, `Odometry`, or `/tf` ground-truth stream. The direct bag does not emit a standalone ground-truth pose, odometry, object-transform, or environment topic.

The source IMU history is enabled only for IMUs selected in `Recording.Sensors`, and only while direct bag recording is active. Its disabled path is one relaxed atomic branch per IMU update and does not lock, allocate, serialize, or perform I/O. While bagging, the physics update only appends to a bounded in-memory ring; CDR serialization, MCAP writing, and image compression/file I/O stay on recorder workers. Payloads are spooled to a temporary file and the writer's small **ordering** index is sorted by source timestamp at stop, so unchunked MCAP file order is chronological during rosbag2 playback. This is valid MCAP fast-write output, but it deliberately contains no MCAP `Chunk`, `MessageIndex`, or `ChunkIndex` records: `ros2 bag info` warns and uses file order, while seeking and efficient topic-filtered reads scan the bag. Rewrite a closed bag with an MCAP tool that adds chunks and indexes before workflows that need those operations. Finalization temporarily needs space for the spool plus the final bag, and retains one small ordering entry per message in memory. The writer publishes the final `.mcap` only after a successful footer/trailing-magic write; failed sessions do not leave a partial bag at the configured file name.

This is strictly an AirSim/Unreal record: its topics are under `/airsim_node/...` and it does not subscribe to or write PX4 uXRCE-DDS `/fmu/...` topics. Recording real PX4 telemetry in WSL requires a separate PX4-aware recorder and a separate MCAP file; see [actual PX4 `/fmu` recording](ros_cplusplus.md#actual-px4-fmu-recording-in-wsl). Do not append that process's output to the Unreal-written MCAP file.

After stopping recording, inspect the file with a ROS 2 installation that includes the MCAP storage plugin, for example `ros2 bag info <session>/airsim_rec.mcap --storage mcap`. MCAP-aware tools can also inspect the file directly.

## Lifecycle

- `Recording.Enabled=true` starts on the **next tick** after vehicle/physics init.
- Restart waits for the sampler, image worker, and ordered writer to finish; `SequenceID` restarts at 1.
- EndPlay stops recording before destroying `physics_world_`.
- Recording / RPC / CameraHost image captures are serialized.
