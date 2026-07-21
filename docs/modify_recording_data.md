# Recording Data (Cameras + Sensors)

## Synchronization model (frame-latched dual-rate)

Recording is **free-run** (no physics pause, no `ClockSpeed` change):

1. A dedicated sampler produces **sensor / pose rows** at `SensorRecordInterval` (default **0.005 s, about 200 Hz** target).
2. A separate image worker schedules cameras at `RecordInterval` (default **0.05 s, about 20 Hz** target). Camera work and image file I/O do not block the sensor sampler.
3. Immediately before `CaptureScene`, the game thread briefly locks physics, snapshots pose/sensors, updates the vehicle's rendered state from that snapshot, and releases the lock. Physics is not paused while GPU rendering, readback, compression, or file writing completes.
4. Image rows use `AssociationMode=frame_latched_freerun`. Sensor-only rows use `AssociationMode=sensor_only`.
5. Every image is saved even when its timestamp delta exceeds `ImageSyncToleranceMs`. Timing columns and the filename mark the result so downstream tools can filter it.

This is a logical vehicle/render-frame latch, not a frozen-world or continuous-time shutter guarantee. Other independently moving world actors continue normally. The recorded delta is `ImageTimeStamp - FrameTimeStamp`; use its absolute value when applying a synchronization threshold.

| Field | Meaning |
|-------|---------|
| `FrameTimeStamp` | Sim-clock ns at sensor/pose sample |
| `PhysicsStepID` | ClockFactory step count at sample |
| `SequenceID` | Monotonic id within the session (restarts each session) |
| `RenderFrameNumber` | UE frame containing the image (0 for sensor-only rows) |
| `S_TimeStamp` | Native sensor output timestamp (never rewritten) |
| `S_Age` | `FrameTimeStamp - S_TimeStamp` (ns) |
| `AssociationMode` | `sensor_only`, `frame_latched_freerun`, or a capture failure mode |
| `ImageRequestTimeStamp` | Sim-clock ns immediately before the render request is submitted |
| `ImageTimeStamp` | Sim-clock ns for the rendered UE frame, recorded before GPU readback |
| `ImageDelayNs` / `ImageDelayMs` | Signed `ImageTimeStamp - FrameTimeStamp` |
| `ImageSyncWithinTolerance` | `1` when `abs(ImageDelayMs) <= ImageSyncToleranceMs`, otherwise `0` |
| Image filename | Includes snapshot/render timestamps plus `_dtp...us_ok`, `_dtm...us_ok`, or `_..._over` |

Default multirotor sensor names are **lowercase** (`imu`, `gps`, `barometer`, `magnetometer`). Selection is **case-insensitive**.

## Settings example

```json
"Recording": {
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
- `"Cameras": []` — sensors/pose only at `SensorRecordInterval`.
- Missing sensors: `S_Present=0`, empty value fields (column count fixed).
- `S_Present=1` only when `time_stamp != 0`. GPS also has `S_GPS_Valid`.
- Distance honors `UpdateLatency`, `UpdateFrequency`, `StartupDelay`, and `UncorrelatedNoiseSigma`.

## Columns

Base (12): `VehicleName`, `SequenceID`, `PhysicsStepID`, `RenderFrameNumber`, `FrameTimeStamp`, `POS_*`, `Q_*`

Per sensor token `S` (32): Present, TimeStamp, Age, 10 IMU, 10 GPS, 3 Baro, 3 Mag, 3 Distance (type-specific filled; others empty).

Car/Skid extras: `Throttle`, `Steering`, `Brake`, `Gear`, `Handbrake`, `RPM`, `Speed`.

Then: `AssociationMode`, `ImageRequestTimeStamp`, `ImageTimeStamp`, `ImageDelayNs`, `ImageDelayMs`, `ImageSyncWithinTolerance`, `ImageFile`.

Multiple images on one row use semicolon-separated values in the same order. Sensor-only rows leave the image timing and filename fields empty.

## Lifecycle

- `Recording.Enabled=true` starts on the **next tick** after vehicle/physics init.
- Restart waits for the sampler, image worker, and ordered writer to finish; `SequenceID` restarts at 1.
- EndPlay stops recording before destroying `physics_world_`.
- Recording / RPC / CameraHost image captures are serialized.
