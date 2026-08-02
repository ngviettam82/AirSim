# PX4-owned ROS 2 live control

`airsim_px4_offboard` keeps AirSim as the camera/simulation provider while PX4
remains the only flight-command owner.

| Profile | Live data | Command owner |
|---|---|---|
| `Ros2.ControlMode: "AirSim"` | AirSim cameras and selected AirSim sensors | AirSim RPC velocity/takeoff/land interfaces |
| `Ros2.ControlMode: "PX4"` | AirSim cameras plus PX4 `/fmu/out/...` state and sensors | PX4 native ROS 2 offboard topics |

The existing AirSim-PX4 MAVLink HIL link still feeds simulated sensors into
PX4 and applies PX4 actuator output to the vehicle. ROS 2 does not replace that
link.

## Synchronization contract

AirSim writes `ClockFactory` time into `HIL_SENSOR.time_usec`. The bridge does
not merely assume that PX4 consumes it: in PX4 mode `airsim_node` publishes a
bounded sent-HIL history at
`/airsim_node/<vehicle>/px4/hil_sensor_clock`, and each synchronizer requires
at least three exact `HIL_SENSOR.time_usec`/PX4 `SensorCombined.timestamp`
matches before it emits an image/state pair. That runtime proof establishes the
shared simulation-time epoch; the pair then uses the exact
microsecond-to-nanosecond conversion without a wall-clock offset.

### Disable uXRCE-DDS timestamp translation

This direct mapping requires PX4's uXRCE-DDS timestamp synchronization to be
disabled. PX4 v1.16 enables `UXRCE_DDS_SYNCT` by default; when enabled it adds
the Agent/PX4 clock offset to outgoing DDS message timestamps. That offset is
asynchronously estimated and is not an AirSim render-time offset, so the
bridge deliberately does not compensate for it.

In the PX4 console, before launching the ROS 2 bridge:

```text
param set UXRCE_DDS_SYNCT 0
param save
```

`UXRCE_DDS_SYNCT` is reboot-required. Fully stop the PX4 SITL process from the
terminal that launched it, wait for it to exit, and run the same SITL launcher
command again before starting the bridge. Do not issue `reboot` at the POSIX
SITL console: this target does not provide that command. A full SITL restart
also recreates the uXRCE-DDS client from the saved parameter. The nodes observe the first
`px4_dds_timesync_guard_sec` seconds of `SensorCombined` (3 seconds by
default). A DDS `TimesyncStatus` during that window, including one with a zero
current estimate, is a fatal configuration error. The observation is a
defense-in-depth check, not a substitute for verifying the parameter.

The bridge supports only this `airsim_hil` clock path. It does not apply a DDS
wall-clock offset, relabel a delayed frame, or pause physics. For a real-time
vision/control latency budget, use `"ClockType": "ScalableClock"`,
`"ClockSpeed": 1`, and set the AirSim PX4 vehicle to `"LockStep": false`.
That keeps source time aligned with real elapsed time and removes the AirSim
actuator-wait barrier, while synchronization remains available only when the
live timestamp proof succeeds. A `SteppableClock`, non-unit `ClockSpeed`, or
`"LockStep": true` is useful for deterministic/offline simulation, but it is
not a real-time latency model and can reduce camera throughput.

The AirSim wrapper publishes an image and its paired `CameraInfo` only when
the Unreal render path establishes an explicit capture/readback transaction
for those pixels. Their headers, dimensions, and pinhole intrinsics are
snapshotted from the same game-thread camera state immediately before
`CaptureScene()`, not from request, GPU-readback, or ROS-callback time. The
camera body and optical TF samples use that same pose and stamp, so gimbal
rotation is represented without moving the camera mount. Orthographic and
equirectangular images explicitly report an uncalibrated `CameraInfo`
(`K[0] == 0`) rather than false pinhole intrinsics. Deploy the matching Unreal
plugin and ROS wrapper together: an older plugin cannot provide this
provenance and the newer wrapper intentionally drops its frames rather than
claiming an inexact timestamp.

For every camera frame, `px4_camera_sync`:

- keeps the original image topic and capture stamp unchanged;
- subscribes to the paired `CameraInfo`, whose header is identical to the
  AirSim image header, so it never transports or deserializes a second image
  payload;
- interpolates `VehicleOdometry`, `VehicleAttitude`, and image-associated gyro
  and accelerometer vectors from `SensorCombined` at that exact stamp;
- rejects brackets that cross estimator resets, IMU calibration changes,
  clipping, or configured time gaps;
- publishes ROS ENU/FLU odometry whose twist is correctly expressed in
  `child_frame_id`;
- associates the newest GPS sample at or before the frame, preserves the GPS
  source stamp, and reports its age instead of inventing an image-time GPS
  measurement.

It does not publish a pair while the HIL history is missing or stale, the
three-match proof is incomplete, PX4's `SensorCombined` clock is stale, or DDS
timestamp translation is observed. A lost proof fences queued frames and
requires fresh evidence before pairing resumes. This makes the data safe to
associate by timestamp; it does not claim that ROS/WSL delivery itself has zero
latency.

### Frame-gated image payloads

`CameraInfo`, the AirSim image payload, and `Px4ImageSync` are independent DDS
deliveries. Publishing `CameraInfo` before an image does not make the three
messages atomic: an application can receive a raw image whose `CameraInfo` was
rejected by the synchronizer, or receive a valid `Px4ImageSync` without the
image payload under subscriber backpressure. Do not process an AirSim image
immediately in its callback when it drives a PX4-bound command.

`px4_frame_gate` is the default bounded relay for the live-control launch and
an optional relay for lower-level launches that intentionally do not authorize
PX4 commands. It retains at most `max_pending_frames` raw `sensor_msgs/Image` or
`sensor_msgs/CompressedImage` messages, matching `CameraInfo` messages, and
synchronization events, then relays the original camera sample only after all
of the following hold:

- the image and `CameraInfo` headers exactly match an accepted
  `Px4ImageSync.image_header`, including `frame_id` (and raw-image dimensions);
- the input synchronizer currently reports `READY`;
- the event has the direct-HIL proof, configured AirSim/PX4 identity, and a
  bounded PX4-clock-to-image source age; and
- neither the synchronizer status nor the PX4 clock epoch fenced the cache.

It drops unmatched sides after `max_frame_wait_sec`, validates the direct JPEG
frame header and its encoded dimensions before accepting compressed payloads,
never decodes or re-encodes image data, and keeps the wrapper's original stream available. The relayed
`camera_info`, image payload, and `image_sync` retain their original headers.
They remain separate DDS messages, so a consumer must associate the three gated
topics by exact stamp. Receiving a gated payload is proof that this relay
observed the matching calibration and accepted source event.

## Requirements

- An AirSim `PX4Multirotor` connected to PX4 SITL through MAVLink HIL.
- PX4 SITL built with its HIL simulator scheduler enabled (the normal default).
  The runtime timestamp proof, rather than a configuration claim, verifies
  that this build is consuming AirSim HIL time. This is independent of AirSim's
  `LockStep` actuator-wait setting.
- ROS 2 Humble or a compatible ROS 2 distribution.
- A Micro XRCE-DDS Agent connected to PX4.
- `px4_msgs` generated from the exact PX4 source revision used to build SITL.
- `UXRCE_DDS_SYNCT=0`, saved and applied by a full PX4 SITL restart.

The bridge supports only the exact PX4 message layouts checked at startup,
including `VehicleStatus` `MESSAGE_VERSION=1`. A PX4 build with a different
layout fails closed rather than being guessed from overlapping field names.

Do not use an arbitrary `px4_msgs` checkout. DDS message definitions must
match the firmware. From the PX4 source tree, build a clean overlay with the
official export script:

```shell
mkdir -p ~/px4_msgs_overlay/src
cd ~/PX4-Autopilot
Tools/copy_to_ros_ws.sh ~/px4_msgs_overlay

cd ~/px4_msgs_overlay
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_msgs
source install/setup.bash
```

Both PX4 nodes validate the complete message contract before creating their
operational publishers and subscriptions. The bridge supports its explicitly
validated layouts, including `thrust_and_torque` and `direct_actuator` in
`OffboardControlMode`, the versioned `VehicleRatesSetpoint`,
`VehicleOdometry`, and `VehicleAttitude` layouts, and the decimal-degree
`SensorGps` layout when GPS is enabled. Building `px4_msgs` from the SITL
source is necessary but not by itself a compatibility guarantee: stop if
startup reports an unsupported layout, then use a supported PX4/`px4_msgs`
pair or extend the bridge deliberately. GPS is not schema-validated when this
pipeline neither publishes nor requires GPS.

PX4's default `dds_topics.yaml` exposes `SensorCombined`, `VehicleOdometry`,
`VehicleAttitude`, `VehicleStatus`, and `VehicleGpsPosition`/`SensorGps`.
Its uXRCE-DDS client appends `_v<N>` to a topic whose message has a nonzero
`MESSAGE_VERSION`; the validated `VehicleStatus` layout has version `1`, so the
default ROS 2 topic is `<px4_topic_prefix>/out/vehicle_status_v1`.
`px4_vehicle_status_suffix` defaults to `_v1` and can select another standard
suffix (or an empty suffix for an unversioned custom endpoint). For a fully
custom path, set `px4_vehicle_status_topic:=/fmu/out/vehicle_status`; that full
override takes precedence over the suffix. Changing the topic name does not
make a different `VehicleStatus` message layout compatible: the bridge still
accepts only its validated schema. Raw `SensorBaro` and `SensorMag` are not
exposed by default. To consume those raw PX4 topics, add them to PX4's
`dds_topics.yaml`, rebuild PX4, and regenerate the matching `px4_msgs` overlay.
Selected AirSim barometer and magnetometer topics remain available from
`airsim_node`, but they are AirSim sensor outputs rather than PX4 flight-stack
outputs.

## AirSim settings

Every configured vehicle must be a `PX4Multirotor` when the global ROS 2
control mode is `PX4`. Mixed SimpleFlight/PX4 ownership is rejected at settings
load time.

```json
{
  "SimMode": "Multirotor",
  "Ros2": {
    "ControlMode": "PX4",
    "Sensors": [
      { "VehicleName": "drone1", "SensorName": "imu" },
      { "VehicleName": "drone1", "SensorName": "gps" }
    ]
  },
  "Vehicles": {
    "drone1": {
      "VehicleType": "PX4Multirotor",
      "UseTcp": true,
      "LockStep": false
    }
  }
}
```

`Ros2.Sensors` is an explicit whitelist for additional AirSim sensor topics.
It does not control PX4's `/fmu/out/...` graph.

In PX4 mode, `airsim_node`:

- rejects `enable_api_control:=true`;
- does not create AirSim velocity, takeoff, land, group, or all-vehicle command
  interfaces;
- does not expose the global AirSim reset service;
- publishes transient-local `/airsim_node/control_mode` with value `PX4`.

## Build

Source the firmware-matched message overlay before building this workspace:

```shell
source /opt/ros/humble/setup.bash
source ~/px4_msgs_overlay/install/setup.bash
cd <cosys-airsim>/ros2
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## One vehicle, one or more cameras

`px4_live_control.launch.py` starts exactly one AirSim wrapper, one PX4 rate
controller, and one synchronizer per semicolon-separated camera topic.

```shell
AIRSIM_HOST_IP="$(ip route | awk '/default/ {print $3; exit}')"

ros2 launch airsim_px4_offboard px4_live_control.launch.py \
  host_ip:="$AIRSIM_HOST_IP" \
  node_namespace:=drone1 \
  px4_topic_prefix:=/fmu \
  update_airsim_img_response_every_n_sec:=0.016666667 \
  camera_topics:="/airsim_node/drone1/front_Scene/image;/airsim_node/drone1/down_Scene/image" \
  camera_output_prefixes:="front_sync;down_sync" \
  start_frame_gates:=true \
  primary_camera_index:=0
```

The first/selected camera's gated `image_sync` event gates native rate output
by default. Use the actual topic names from `ros2 topic list -t`; AirSim camera
topic tokens depend on the configured camera name and image type.

The wrapper requests camera frames every
`update_airsim_img_response_every_n_sec` seconds (20 Hz by default). Set it to
`0.016666667` to request 60 Hz, as in the example, only after measuring the
target map and resolution. It never queues an overdue RPC poll: if one capture
or delivery takes too long, that poll is dropped and the next frame retains its
own capture stamp. `update_airsim_control_every_n_sec` independently controls
the state and HIL-history poll period (100 Hz by default).

### Throughput choices

A 1280x720 raw RGB image is about 2.64 MiB. A raw frame gate retains and
relays both its input and output, which is about 105 MiB/s at 20 Hz or
316 MiB/s at 60 Hz before DDS serialization and process copies. When JPEG is
appropriate for the perception algorithm, use
`image_response_compress:=true frame_gate_image_transport:=compressed`. The
wrapper then publishes the direct Unreal JPEG and the gate validates and
relays its original bytes without decoding or re-encoding them. Keep raw for
pixel-exact imagery, labels, or float/depth data.

`Ros2.Sensors` opts into individual AirSim sensor RPC polling; it does not
control PX4 HIL or the PX4 `/fmu/out/...` graph. A PX4 pipeline that consumes
only synchronized PX4 topics can use `"Sensors": []` to avoid those extra
polls. Select only the additional AirSim sensors the application actually
uses.

AirSim serializes scene-capture transactions from live ROS, recording, and
CameraHost. They remain correct when used together, but two high-rate
consumers of the same camera compete for render/readback time. Avoid combining
a high-rate CameraHost stream or direct MCAP recording with the live wrapper
for that camera unless the target map, resolution, and rates have been
measured together.

`hil_clock_topic` defaults to the per-vehicle AirSim history topic shown
above. Keep `direct_hil_clock_matches_required:=3` for live control;
`direct_hil_clock_proof_timeout_sec:=10.0`, `hil_clock_max_age_sec:=0.5`,
`px4_clock_timeout_sec:=10.0`, and `px4_clock_max_age_sec:=0.5` are the
fail-closed startup and freshness bounds. Increase only after measuring the
actual ROS/WSL transport on the deployment target.

Frame gates start by default. When native rate control requires image synchronization,
`start_frame_gates:=true` is enforced; disabling the gate is allowed only for an
observation-only pipeline or an explicitly non-image-synchronized controller. With
`start_frame_gates:=true`, each camera pipeline also starts
`px4_frame_gate`. A camera prefix `front_sync` emits the accepted raw payload
on `/drone1/front_sync_gated/image`, its capture-time calibration on
`/drone1/front_sync_gated/camera_info`, and its matching proof on
`/drone1/front_sync_gated/image_sync`. Point the perception/controller
algorithm at those three gated topics, associate them by the identical header,
then copy the image header into `Px4RateSetpoint`.
When frame gates are enabled, the rate controller consumes the corresponding
`front_sync_gated/image_sync`, so it cannot authorize a camera-driven command
until the gate observed the payload, calibration, and accepted source event.

Use `frame_gate_image_transport:=compressed` to gate the AirSim
`/image/compressed` sibling without changing or decoding its JPEG payload; the
gated output then ends in `/image/compressed`. Pair it with
`image_response_compress:=true` to have the wrapper publish the direct JPEG
stream; otherwise the normal image-transport compressed publisher must be
available. In the standard launch, `camera_topics` still names the logical raw
AirSim image topic used by the synchronizer, while the gate subscribes to its
compressed sibling. Set
`frame_gate_max_pending_frames`, `frame_gate_max_frame_wait_sec`, and
`frame_gate_max_source_age_sec` only after measuring the deployment. The
defaults bound a 1280x720 raw cache to five images and accept no image older
than 250 ms at the synchronizer's PX4-clock observation.

`camera_output_prefixes` are relative to `node_namespace` (for example,
`front_sync;down_sync`), never absolute ROS paths. This prevents two vehicle
pipelines from aliasing the same output topics.

`world_frame_id` must be identical for the AirSim wrapper and every PX4 vehicle
pipeline whose camera TF and synchronized odometry are consumed together. The
PX4 convenience launch and `airsim_px4_wrapper.launch.py` default it to
`world_enu`; the generic `airsim_node.launch.py` retains its compatible
`world` default. In the split multi-vehicle topology, pass the same explicit
`world_frame_id` to the one wrapper and to each `px4_vehicle.launch.py`.

## Multiple PX4 vehicles

Launch the AirSim wrapper once, then one namespaced vehicle pipeline per PX4
instance. Each PX4 instance must have a distinct DDS namespace/topic prefix.
`px4_topic_prefix` must exactly match the namespace configured in that PX4
instance's uXRCE-DDS client; it is not an arbitrary ROS remap. Set
`expected_px4_system_id` to that instance's `VehicleStatus.system_id` and
verify the mapping before enabling control, for example with `ros2 topic echo
--once --qos-profile sensor_data /drone1/fmu/out/vehicle_status_v1`. Use
`px4_vehicle_status_suffix:=_v<N>` to select another standard versioned topic,
or pass the exact full path as `px4_vehicle_status_topic` for a custom DDS
endpoint. The full-path override takes precedence over the suffix.

```shell
ros2 launch airsim_px4_offboard airsim_px4_wrapper.launch.py \
  host_ip:="$AIRSIM_HOST_IP"

ros2 launch airsim_px4_offboard px4_vehicle.launch.py \
  node_namespace:=drone1 \
  px4_topic_prefix:=/drone1/fmu \
  expected_px4_system_id:=1 \
  camera_topics:=/airsim_node/drone1/front_Scene/image

ros2 launch airsim_px4_offboard px4_vehicle.launch.py \
  node_namespace:=drone2 \
  px4_topic_prefix:=/drone2/fmu \
  expected_px4_system_id:=2 \
  camera_topics:=/airsim_node/drone2/front_Scene/image
```

Do not start `px4_live_control.launch.py` once per camera or vehicle; that
would intentionally start another AirSim wrapper. The split launch topology
prevents duplicate RPC polling, TF/services, and PX4 command publishers.

## Synchronized outputs

With namespace `drone1` and camera prefix `front_sync`:

| Topic | Type | Meaning |
|---|---|---|
| `/drone1/front_sync/odometry_at_image` | `nav_msgs/msg/Odometry` | PX4 pose and body-frame twist at the exact image stamp. |
| `/drone1/front_sync/imu_at_image` | `sensor_msgs/msg/Imu` | PX4 attitude, gyro, and accelerometer data interpolated at the image stamp. |
| `/drone1/front_sync/gps_at_or_before_image` | `sensor_msgs/msg/NavSatFix` | Fresh causal GPS sample with its original source stamp. |
| `/drone1/front_sync/image_sync` | `airsim_interfaces/msg/Px4ImageSync` | Image header, all interpolation brackets/counters, and GPS age. |
| `/drone1/front_sync/status` | `std_msgs/msg/String` | `READY`, `STALE_SYNC`, waiting state, or startup error. |
| `/drone1/front_sync_gated/camera_info` | `sensor_msgs/msg/CameraInfo` | Optional unchanged calibration released with the exact gated image sample. |
| `/drone1/front_sync_gated/image` | `sensor_msgs/msg/Image` | Optional unchanged raw payload released with matching calibration and exact accepted sync. |
| `/drone1/front_sync_gated/image/compressed` | `sensor_msgs/msg/CompressedImage` | JPEG variant of the gated payload when `frame_gate_image_transport:=compressed`. |
| `/drone1/front_sync_gated/image_sync` | `airsim_interfaces/msg/Px4ImageSync` | Optional unchanged sync event paired with the gated camera sample. |
| `/airsim_node/drone1/px4/hil_sensor_clock` | `airsim_interfaces/msg/Px4HilSensorClock` | Bounded sent `HIL_SENSOR.time_usec` history used only to verify the shared clock. |

The synchronizer emits no pair until odometry, attitude, gyro, and
accelerometer histories bracket the frame within their configured maximum
gaps, the direct-HIL proof has three exact matches, and both PX4 and the HIL
history are fresh. `Px4ImageSync.direct_hil_clock_verified` is always true for
a published pair and includes the match count and most recent matched source
timestamp. GPS is optional by default (`require_gps:=false`) because its real
sensor rate is much lower. Set `require_gps:=true` only when every camera
result must also have a sufficiently recent causal GPS sample.

The normal AirSim SITL startup reports `VehicleStatus.hil_state=HIL_STATE_OFF`.
That Commander hardware-HIL flag is not the AirSim MAVLink sensor-clock
contract, so the bridge binds the configured PX4 `system_id` but does not
require `HIL_STATE_ON`. Do not add `commander start -h` merely to change this
flag.

The image header identifies the capture-state transaction that produced its pixels. PX4 gyro and accelerometer
vectors, however, are integration-period measurements; interpolation produces
an image-associated estimate, not a mathematically instantaneous IMU sample.
Use the `Px4ImageSync` bracket timestamps, spans, and calibration counters when
an algorithm needs to account for that measurement interval.

On a PX4 HIL-clock regression or a `SensorCombined` receipt gap longer than
`px4_session_gap_sec` (default `0.1`), the synchronizer starts a new epoch and
flushes history, pending images, and vehicle validation. It will not publish
again until fresh PX4 state and status arrive; delayed state samples dated
before the new session start are rejected so they cannot bracket a fresh
image. The receipt threshold is strict (`>`): it is a fail-closed availability
trade-off and must remain above measured normal DDS/WSL inter-arrival jitter.

PX4 does not expose a boot-session ID on this HIL path, so an implausibly fast
restart with neither a timestamp regression nor an observable receipt gap
cannot be distinguished from a continuous session. A safety case that requires
that absolute guarantee needs a PX4/uXRCE-DDS boot-session UUID topic and must
fence all camera state and commands whenever it changes; this AirSim-side
bridge cannot infer that identity from the supported PX4 messages.

`camera_topic` names the AirSim image stream whose sibling `CameraInfo` is
used by the synchronizer; the synchronizer intentionally does not subscribe
to that payload. It automatically derives the sibling `camera_info` topic,
including for an image-transport suffix such as `/image/compressed`. Direct
node launches can override that derivation with `camera_info_topic`. Launch
`px4_frame_gate` (enabled by default by the live-control launch) when a
downstream algorithm needs a complete camera sample that has passed the
synchronization boundary.

## Native PX4 rate control

The controller accepts `airsim_interfaces/msg/Px4RateSetpoint` on the
namespaced `rate_setpoint` topic:

```text
std_msgs/Header header  # mandatory AirSim/PX4 HIL source timestamp
float32 roll_rate       # ROS FLU, rad/s
float32 pitch_rate      # ROS FLU, rad/s
float32 yaw_rate        # ROS FLU, rad/s
float32 thrust          # normalized upward collective [0, 1]
```

The header should normally be copied from the synchronized camera frame used
by the algorithm. With `require_image_sync:=true` it must exactly equal an
accepted primary-camera `Px4ImageSync` stamp; an older arbitrary stamp is not
enough. The bridge stops publishing immediately when any of these guards fail:

- non-finite or out-of-range command values;
- missing, repeated, regressed, future, or stale source timestamps;
- stale command transport;
- a PX4 HIL clock that stops advancing or regresses;
- a DDS `TimesyncStatus` showing that PX4 timestamp translation is enabled;
- missing, stale, or unproven primary-camera synchronization;
- a PX4 session fence caused by a clock reset or delivery gap;
- AirSim reporting a control mode other than `PX4`; or
- PX4 reporting `failsafe` or `failsafe_and_user_took_over`.

After a PX4 failsafe/user-takeover event clears, the bridge fences the prior
camera authorization and requires a new gated frame plus a new command. While
all guards are healthy, the bridge converts FLU rates to PX4 FRD,
converts upward thrust to negative body-Z, and publishes
`OffboardControlMode` plus `VehicleRatesSetpoint` at 100 Hz by default. It
never arms the vehicle or requests OFFBOARD mode. Establish a healthy
continuous stream first, then use QGroundControl or a separately reviewed PX4
`VehicleCommand` publisher for arming and mode changes.

Set `start_rate_control:=false` for synchronized perception only. Set
`require_image_sync:=false` only for a controller whose command source is not a
camera; timestamp and PX4-clock watchdogs still apply.

## Validation

Before arming, verify:

```shell
# PX4 console: must print 0 after the required restart.
param show UXRCE_DDS_SYNCT

ros2 topic echo --once /airsim_node/control_mode
ros2 topic hz /fmu/out/sensor_combined
ros2 topic echo --once --qos-profile sensor_data /fmu/out/vehicle_status_v1
ros2 topic echo --once /airsim_node/drone1/px4/hil_sensor_clock
ros2 topic hz /drone1/front_sync/image_sync
ros2 topic echo --once /drone1/front_sync/status
ros2 topic echo --once /drone1/rate_control/status
```

Then stop the camera synchronizer, PX4 `SensorCombined`, and controller input
one at a time. Native setpoint publication must stop for each fault so PX4 can
apply its configured offboard-loss behavior.
