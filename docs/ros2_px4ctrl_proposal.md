# PX4-in-the-loop ROS 2 control architecture

## Scope

AirSim provides rendering, simulated physics, cameras, and the existing
MAVLink HIL connection. PX4 provides flight-stack state, failsafes, mission and
QGroundControl behavior, and native offboard command handling. ROS 2 connects
perception/control algorithms to those two existing systems without becoming a
second flight-command owner.

| Mode | Live data | Flight command owner |
|---|---|---|
| `AirSim` | AirSim RPC cameras and selected AirSim sensors | AirSim ROS wrapper/RPC |
| `PX4` | AirSim cameras plus PX4 DDS state and sensors | PX4 native ROS 2 inputs |

The mode is selected by `Ros2.ControlMode`. PX4 mode requires every configured
vehicle to be a `PX4Multirotor`, disables direct AirSim multirotor command
interfaces and reset, and rejects `enable_api_control:=true`.

## Runtime topology

```text
AirSim Unreal --RPC--> one airsim_node --------------------> original Image topics
      |                                                        |
      +--MAVLink HIL--> PX4 SITL --uXRCE-DDS--> /fmu/out/* ----+
                                                               |
                                          one synchronizer per camera
                                                               |
                                    state and IMU at image capture time
                                                               |
Algorithm --Px4RateSetpoint--> one rate controller per PX4 vehicle
                                                               |
                                      /fmu/in/* --> PX4 --> AirSim actuators
```

Multi-camera deployments share the single AirSim wrapper. Multi-vehicle
deployments add one namespaced PX4 pipeline per vehicle and use a distinct PX4
DDS topic prefix for every instance.

## Simulation-time synchronization

AirSim writes `ClockFactory` time into `HIL_SENSOR.time_usec`. A production
bridge must prove that the deployed PX4 instance consumes those values rather
than trusting configuration alone. In PX4 mode `airsim_node` publishes a
bounded history of successfully queued HIL timestamps; each camera
synchronizer requires at least three exact matches between that history and
live PX4 `SensorCombined.timestamp` values before it emits a pair. Only then
do camera render timestamps and PX4 source timestamps share a verified
simulation-time epoch.

The production bridge uses this direct HIL mapping only:

- no physics pause is introduced by ROS synchronization;
- no DDS companion-wall-clock offset is applied;
- missing, stale, or mismatched HIL-clock evidence blocks pairs and fences
  queued images;
- the source image payload and `Header.stamp` remain unchanged;
- the synchronizer consumes the matching `CameraInfo` header instead of a
  duplicate image payload subscription;
- delayed delivery is handled by retaining PX4 history and associating by
  source timestamp, not callback arrival time.

PX4's uXRCE-DDS timestamp synchronization must be disabled for this mode. The
PX4 v1.16 default `UXRCE_DDS_SYNCT=1` translates outgoing DDS timestamps into
the Agent clock with an asynchronously estimated offset, which is incompatible
with exact AirSim render/source-time binding. Set it in the PX4 console and
restart PX4 before launching the bridge:

```text
param set UXRCE_DDS_SYNCT 0
param save
```

`UXRCE_DDS_SYNCT` is reboot-required. Stop the complete PX4 SITL process from
the terminal that launched it, wait for it to exit, then rerun the same SITL
launcher command. The POSIX SITL console does not implement `reboot`; a full
restart recreates the uXRCE-DDS client from the saved parameter.

The bridge observes the default `/fmu/out/timesync_status` for the first three
seconds of an advancing PX4 clock and fails closed if it sees a DDS time-sync
sample, even when that sample's current offset is zero. This observation is
defense in depth; the parameter setting is the required configuration.

Normal AirSim SITL starts the PX4 Commander without its hardware-HIL `-h`
flag, so `VehicleStatus.hil_state` is normally `HIL_STATE_OFF`. The bridge uses
`VehicleStatus.system_id` for vehicle binding and deliberately does not use
that hardware-HIL flag as a clock prerequisite.

The wrapper accepts an image for this path only when Unreal establishes an
explicit `CaptureScene()`/ordered-readback transaction for it. It publishes
the image, paired `CameraInfo`, and dynamic camera body/optical TF with the
same game-thread capture-state stamp; no request, GPU-readback, or ROS-arrival
fallback is relabeled as a capture time.
The TF pose is world-parented from the image-time camera pose, which preserves
gimbal rotation without applying the vehicle's initial yaw twice.

For low-latency live processing, AirSim should use `"LockStep": false`. This
disables AirSim's actuator-wait barrier but does not change the HIL timestamps.
The three-match runtime proof establishes whether PX4 is actually using them;
it does not require the throughput-reducing AirSim lockstep barrier.
`"LockStep": true` is reserved for tests that need deterministic
actuator/physics sequencing and can accept reduced camera throughput.

Each camera synchronizer brackets and interpolates:

- `VehicleOdometry` position and velocity;
- `VehicleAttitude` orientation;
- `SensorCombined` gyro at its message timestamp;
- `SensorCombined` acceleration at
  `timestamp + accelerometer_timestamp_relative`.

The image timestamp is exact for its pixels. `SensorCombined` gyro and
acceleration vectors are integration-period measurements, so their interpolated
values are image-associated estimates rather than mathematically instantaneous
samples. Bracket timestamps and spans remain in `Px4ImageSync` for algorithms
that model this interval explicitly.

It rejects interpolation across PX4 estimator reset counters, IMU calibration
counters, clipped samples, or excessive gaps. Published odometry uses an ENU
world pose and FLU body twist, matching `header.frame_id` and
`child_frame_id`. Known PX4 variances are transformed into the published
frames. The IMU orientation covariance is populated when PX4 provides it;
unknown gyro and accelerometer covariance remains all-zero rather than
incorrectly marking present data as absent.

GPS is not interpolated or relabeled. The newest sample at or before the image
is associated causally, published with its original source stamp, and reported
with its age in `Px4ImageSync`. GPS can be optional because its physical update
rate is much lower than camera and IMU rates.

## Command safety

`Px4RateSetpoint.header.stamp` is mandatory and identifies the synchronized
source data used to compute the command. The rate bridge validates two
independent freshness dimensions:

- local monotonic receipt age, which detects a dead controller even if
  simulation time stops;
- AirSim/PX4 HIL source age, which detects delayed or replayed perception.

Publication stops immediately for invalid values, missing/repeated/regressed
or future source timestamps, stale command transport, stale source data, a PX4
clock that does not advance, loss of the primary camera's synchronization
events, or a synchronization event without direct-HIL proof. Invalid input
clears the previous command rather than allowing it to continue until a later
timeout.

When image synchronization is enabled, the command source stamp must exactly
equal an accepted primary `Px4ImageSync` image stamp. A merely older stamp is
rejected. A PX4 clock regression or `SensorCombined` receipt gap larger than
`px4_session_gap_sec` fences a new session, clearing image/state history and
commands until fresh status and synchronization arrive. Its default is 0.1
seconds, deliberately below the stale-clock watchdog but above measured normal
DDS/WSL inter-arrival jitter; choose an override only from target-load timing
data. Delayed state samples from before that session boundary are rejected,
preventing an interpolation bracket from crossing a detected PX4 process
restart.

PX4 provides no session UUID on this HIL path, so a restart with neither an
observable timestamp reset nor receipt gap remains indistinguishable from
uninterrupted operation. If the control safety case requires an absolute
restart boundary, extend PX4/uXRCE-DDS with a reliable boot-session UUID topic:
generate a new value at PX4 process start, publish it with the expected system
identity, and make both bridge nodes flush their state and require a fresh
three-match HIL proof whenever that value changes. This requires a deliberate
PX4/Agent integration; it cannot be reconstructed from the supported ROS 2
messages.

The bridge converts ROS FLU rates to PX4 FRD and upward collective thrust to
negative body-Z. It publishes only `OffboardControlMode` and
`VehicleRatesSetpoint`; it does not arm or select OFFBOARD.

## PX4 message compatibility

DDS type definitions must match the firmware. Build `px4_msgs` from the PX4
source that runs SITL with `PX4-Autopilot/Tools/copy_to_ros_ws.sh`. Both PX4
nodes then validate their explicit supported layouts before starting. The
explicitly supported set is the PX4 v1.16.x layout (including
`VehicleStatus` `MESSAGE_VERSION=1`); newer schemas fail closed. A source
match alone is not a promise that any future PX4 layout is supported: use a
supported firmware/message pair or update this bridge deliberately if startup
rejects a schema. The native command publisher rejects the legacy
`OffboardControlMode.actuator` layout and requires `thrust_and_torque`,
`direct_actuator`, and the supported versioned `VehicleRatesSetpoint` contract.
The synchronizer requires supported versioned `VehicleOdometry` and
`VehicleAttitude` layouts, and validates `SensorGps` only when GPS is enabled.

PX4 exposes `SensorCombined`, `VehicleOdometry`, `VehicleAttitude`, and GPS in
its default DDS topic list. Raw barometer and magnetometer topics require
adding `SensorBaro` and `SensorMag` to PX4 `dds_topics.yaml`, rebuilding PX4,
and regenerating the matching message overlay.

## Recording boundary

Unreal `Recording.Output: "Rosbag"` records direct AirSim cameras and selected
AirSim sensors. PX4 `/fmu/out/...` telemetry is recorded by a separate ROS 2
recorder in the DDS graph. Both data sources should be aligned by their source
timestamps; recorder arrival time is not the camera capture time.

## Acceptance checks

1. Settings reject non-PX4 vehicles in PX4 control mode.
2. Exactly one AirSim wrapper is present, with one synchronizer per camera and
   one rate controller per PX4 vehicle namespace.
3. `UXRCE_DDS_SYNCT` is saved as `0`, PX4 has restarted, and no DDS
   `TimesyncStatus` is observed by the direct-clock guard.
4. At least three exact sent `HIL_SENSOR.time_usec` / `SensorCombined.timestamp`
   matches appear on the per-vehicle HIL history path before any pair emits.
5. `Px4ImageSync` reports verified HIL proof, bounded brackets, and no
   reset/calibration crossings.
6. Synchronized odometry twist is body FLU and IMU contains real gyro and
   acceleration data.
7. Stopping the camera synchronizer, PX4 sensor stream, or algorithm command
   stream stops native setpoint publication within the configured watchdog.
8. A firmware-matched `px4_msgs` overlay accepts the native command schema; a
   known mismatched overlay fails at startup.
9. Blocks and HAWK measurements record camera FPS, IMU rate, pair/drop rate,
   bracket spans, source age, and command-watchdog response under target load.
