# Recording Data (Cameras + Sensors)

## Synchronization guarantee

Each sample is a **logical same-snapshot association**:

1. Physics is **paused**.
2. Pose + configured sensor outputs are snapshotted under the physics lock (`PhysicsStepID`, `FrameTimeStamp`).
3. Render pawn state is updated from that kinematics.
4. Images are captured **while physics remains paused**.
5. Physics is resumed; disk I/O is outside the pause.

This is **not** continuous-time shutter accuracy. We do **not** claim wall-clock bounds such as `<0.1 ms`.

| Field | Meaning |
|-------|---------|
| `FrameTimeStamp` | Sim-clock ns at paused snapshot |
| `PhysicsStepID` | ClockFactory step count at snapshot |
| `SequenceID` | Monotonic id within the session (restarts each session) |
| `RenderFrameNumber` | UE frame after image path (0 if no cameras) |
| `S_TimeStamp` | Native sensor output timestamp (never rewritten) |
| `S_Age` | `FrameTimeStamp - S_TimeStamp` (ns) |
| Image filename | Includes `_s{SequenceID}_p{PhysicsStepID}_{FrameTimeStamp}` |
| Image response timestamp | Native render/readback time (not forced equal to frame) |

Default multirotor sensor names are **lowercase** (`imu`, `gps`, `barometer`, `magnetometer`). Selection is **case-insensitive**.

## Settings example

```json
"Recording": {
  "RecordInterval": 0.05,
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

- `"Cameras": []` — pose/sensors only.
- Missing sensors: `S_Present=0`, empty value fields (column count fixed).
- `S_Present=1` only when `time_stamp != 0`. GPS also has `S_GPS_Valid`.
- Distance honors `UpdateLatency`, `UpdateFrequency`, `StartupDelay`, `UncorrelatedNoiseSigma`.

## Columns

Base (12): `VehicleName`, `SequenceID`, `PhysicsStepID`, `RenderFrameNumber`, `FrameTimeStamp`, `POS_*`, `Q_*`

Per sensor token `S` (32): Present, TimeStamp, Age, 10 IMU, 10 GPS, 3 Baro, 3 Mag, 3 Distance (type-specific filled; others empty).

Car/Skid extras: `Throttle`, `Steering`, `Brake`, `Gear`, `Handbrake`, `RPM`, `Speed`.

Then `ImageFile` (only successfully saved files).

## Lifecycle

- `Recording.Enabled=true` starts on the **next tick** after vehicle/physics init.
- Restart waits for the previous worker to finish; `SequenceID` restarts at 1.
- EndPlay stops recording before destroying `physics_world_`.
- Recording / RPC / CameraHost image captures are serialized.
