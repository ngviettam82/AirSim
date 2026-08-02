# airsim_ros_pkgs

A ROS2 wrapper over the Cosys-AirSim C++ client library. All coordinates and data are in the right-handed coordinate frame of the ROS standard and not in NED except for geo points.

This guide targets Ubuntu 22.04 WSL with ROS 2 Humble and the ROS 2 MCAP storage plugin.

## Build

- Build Cosys-AirSim as per the instructions.

- Install ROS 2 Humble and the MCAP tooling used to inspect Unreal-recorded bags:

```shell
sudo apt update
sudo apt install ros-humble-ros-base ros-humble-rosbag2 ros-humble-rosbag2-storage-mcap
```

- Make sure that you have set up the environment variables for ROS. Add the `source` command to your `.bashrc` for convenience:

```shell
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

-- Install dependencies with rosdep, if not already installed -

```shell
apt-get install python3-rosdep
sudo rosdep init
rosdep update
cd <path-to-cosys-airsim>/ros2
rosdep install --from-paths src -y --ignore-src --skip-keys pcl --skip-keys message_runtime --skip-keys message_generation --skip-keys px4_msgs
```

- Build ROS package

```shell
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Running

When AirSim runs on Windows and the wrapper runs in WSL 2, `localhost` refers to WSL, not Windows. Use the WSL default gateway as the AirSim RPC host:

```shell
source install/setup.bash
AIRSIM_HOST_IP="$(ip route | awk '/default/ {print $3; exit}')"
ros2 launch airsim_ros_pkgs airsim_node.launch.py \
  host_ip:="$AIRSIM_HOST_IP" host_port:=41451 enable_api_control:=true
```

The WSL gateway can change when WSL restarts, so derive it instead of hard-coding it. Set `enable_api_control:=true` when the ROS process must send takeoff, land, or velocity commands: at startup it enables API control **and arms every configured vehicle**, so use it only in a safe test environment. PX4 SITL/MAVLink configuration is separate from this TCP RPC connection: the wrapper connects to AirSim on Windows, while PX4 continues to use its configured MAVLink endpoints.

This is the default `Ros2.ControlMode: "AirSim"` path. It is the only mode that exposes the wrapper's direct multirotor `vel_cmd_*`, takeoff, and land interfaces.

## PX4-owned live control

Use PX4 mode when the flight stack, QGroundControl, PX4 failsafes, and native PX4 ROS 2 offboard topics must own the vehicle. The AirSim wrapper remains the live camera and explicitly selected AirSim-sensor publisher; it does not become a PX4 command transport.

Set the active AirSim settings to PX4 mode before launching ROS:

```json
"Ros2": {
  "ControlMode": "PX4",
  "Sensors": [
    { "VehicleName": "drone1", "SensorName": "imu" },
    { "VehicleName": "drone1", "SensorName": "gps" }
  ]
}
```

`PX4` mode requires `SimMode: "Multirotor"` and requires every configured vehicle to use `VehicleType: "PX4Multirotor"`. It rejects `enable_api_control:=true`, never arms a vehicle through AirSim, removes direct multirotor velocity/takeoff/land/group/all-vehicle endpoints, and does not expose the global AirSim reset service. `/airsim_node/control_mode` publishes the transient-local string `PX4` so companion nodes can fail clearly if settings and launch selection disagree.

Start PX4 SITL with the existing AirSim MAVLink simulator link, then start the Micro XRCE-DDS Agent. Generate `px4_msgs` from the exact PX4 source revision rather than using an unrelated checkout:

```shell
mkdir -p ~/px4_msgs_overlay/src
~/PX4-Autopilot/Tools/copy_to_ros_ws.sh ~/px4_msgs_overlay
cd ~/px4_msgs_overlay
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_msgs
source install/setup.bash
```

The exact camera/PX4 timestamp path supports the PX4 v1.16.x message layouts
and fails closed for newer layouts. Before launching the bridge, use the PX4
console to disable its default uXRCE-DDS timestamp translation, then restart
PX4 SITL because the parameter is reboot-required:

```text
param set UXRCE_DDS_SYNCT 0
param save
```

Fully stop the PX4 SITL process from the terminal that launched it, wait for
the process to exit, then rerun the same SITL launcher command. Do not issue
`reboot` at the POSIX SITL console: this target does not provide that command.
A full restart reconstructs the uXRCE-DDS client from the saved parameter.

When `UXRCE_DDS_SYNCT=1`, PX4 adds the Agent/PX4 clock offset to outgoing DDS
timestamps. That asynchronous companion-clock offset is not a camera offset
and must not be compensated in the synchronizer. The nodes reject any DDS
`TimesyncStatus`, including a transient zero estimate, during their initial
three-second observation window. Normal AirSim SITL reports
`VehicleStatus.hil_state=HIL_STATE_OFF`; this is expected and is not used as a
timestamp-synchronization test.

For a real-time vision/control latency budget, use `"ClockType": "ScalableClock"`, `"ClockSpeed": 1`, and set the AirSim PX4 vehicle to `"LockStep": false`. AirSim still sends `ClockFactory` time in `HIL_SENSOR.time_usec`, while disabling only the AirSim actuator-wait barrier. A `SteppableClock`, non-unit `ClockSpeed`, or `"LockStep": true` is useful for deterministic/offline simulation but does not model wall-time control latency. The bridge verifies the deployed PX4 path at runtime: `airsim_node` publishes the bounded sent-HIL history at `/airsim_node/<vehicle>/px4/hil_sensor_clock`, and each camera synchronizer requires three exact matches with live `SensorCombined.timestamp` values before it emits a pair. A PX4 build that cannot satisfy that proof is not supported by this synchronization path.

Launch one wrapper, one controller, and one synchronizer per semicolon-separated camera topic:

```shell
source /opt/ros/humble/setup.bash
source <px4_msgs-overlay>/install/setup.bash
source install/setup.bash
AIRSIM_HOST_IP="$(ip route | awk '/default/ {print $3; exit}')"

ros2 launch airsim_px4_offboard px4_live_control.launch.py \
  host_ip:="$AIRSIM_HOST_IP" \
  node_namespace:=drone1 \
  px4_topic_prefix:=/fmu \
  update_airsim_img_response_every_n_sec:=0.016666667 \
  camera_topics:="/airsim_node/drone1/front_Scene/image;/airsim_node/drone1/down_Scene/image" \
  camera_output_prefixes:="front_sync;down_sync"
```

For multiple PX4 vehicles, start `airsim_px4_wrapper.launch.py` once and start one `px4_vehicle.launch.py` per namespaced PX4 DDS prefix. `px4_topic_prefix` must equal the namespace configured in that PX4 instance's uXRCE-DDS client (for example `/drone1/fmu`), and `expected_px4_system_id` must equal the `system_id` on that instance's `VehicleStatus` topic. The default is `<prefix>/out/vehicle_status_v1`; verify it with `ros2 topic echo --once --qos-profile sensor_data <prefix>/out/vehicle_status_v1` before enabling control. Set `px4_vehicle_status_suffix:=_v<N>` for another standard versioned endpoint, or use `px4_vehicle_status_topic:=<topic>` for a fully custom path; the full-path override takes precedence. The bridge still rejects an unsupported `VehicleStatus` message schema. Do not start the convenience launch once per camera or vehicle, because each convenience launch intentionally owns one AirSim wrapper.

The PX4 launches use `world_frame_id:=world_enu` by default. Keep that argument
identical on the shared wrapper and every split vehicle launch so camera TF and
synchronized PX4 odometry have the same world frame. The generic
`airsim_node.launch.py` default remains `world` for existing non-PX4 users.

The camera synchronizer leaves the original image topic and `Header.stamp` untouched. The wrapper publishes an image and its sibling `CameraInfo` only after Unreal establishes an explicit `CaptureScene()`/ordered-readback transaction; their stamp, dimensions, and pinhole intrinsics are the game-thread simulation/camera-state snapshot immediately before that capture, while request, GPU-readback, and callback-time fallbacks are dropped. Orthographic and equirectangular images are explicitly uncalibrated (`CameraInfo.K[0] == 0`), rather than being assigned false pinhole intrinsics. The synchronizer reads that identical `CameraInfo` header without a second image-payload subscription, and retains only the header while pairing. It blocks pairs until the direct-HIL timestamp proof, PX4 clock, and HIL-history freshness checks all pass; every emitted `Px4ImageSync` carries the proof state and matching-source metadata. Dynamic camera body/optical TF uses the same frame pose and stamp, including gimbal rotation. It interpolates PX4 `VehicleOdometry`, `VehicleAttitude`, and image-associated gyro and accelerometer vectors from `SensorCombined`, rejects estimator-reset, calibration-change, clipping, excessive-gap brackets, and incompatible PX4 message schemas, publishes body-frame odometry twist with valid covariance semantics, and associates low-rate GPS causally with its original source timestamp and reported age. It does not pause physics, wait for GPU readback, or copy images into a second ROS image.

Frame gates are enabled by default for the live PX4 launch. Consume the matching `<camera_prefix>_gated/camera_info`, image (or `image/compressed`), and `image_sync` topics by their identical header stamp. The gate drops an incomplete sample and the rate bridge then accepts commands only after it has observed that gated image, calibration, and verified synchronization event. A camera-authorized rate controller cannot disable `start_frame_gates`; gate opt-out is restricted to observation-only or explicitly non-image-synchronized control. Use relative `camera_output_prefixes` such as `front_sync;down_sync`, never absolute paths, so vehicle namespaces cannot alias each other's outputs. The optional native rate bridge accepts `airsim_interfaces/msg/Px4RateSetpoint` in ROS FLU axes and forwards bounded values to PX4 `VehicleRatesSetpoint`/`OffboardControlMode` at 100 Hz. The input header is mandatory and must carry the AirSim/PX4 HIL source timestamp used by the algorithm, normally the gated camera stamp. When image synchronization is required, the header must exactly equal an accepted primary `Px4ImageSync` stamp with verified direct-HIL proof and the selected AirSim camera topic. Invalid values, replayed/future/stale source stamps, a non-advancing PX4 clock, active DDS timestamp translation, an unproven or stale image synchronization event, a PX4 restart fence, stale command transport, lost image synchronization, or PX4 failsafe/user takeover stop publication immediately. After a PX4 failsafe clears, the bridge requires a new gated frame and a new command before it resumes. The bridge does not arm the vehicle or select OFFBOARD mode. Use QGroundControl or an explicit, separately reviewed PX4 `VehicleCommand` publisher for those state changes. Complete parameters and multi-camera/multi-vehicle guidance are in [the package guide](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_px4_offboard/README.md).

The wrapper requests camera frames every `update_airsim_img_response_every_n_sec` seconds (20 Hz by default); set it to `0.016666667` to request 60 Hz after measuring the actual map, resolution, GPU, and RPC path. An overdue camera RPC poll is dropped rather than queued, so no old frame is relabeled or held behind a backlog. `update_airsim_control_every_n_sec` independently controls the AirSim state/HIL-history poll period (100 Hz by default).

Set `image_response_compress:=true` to request direct Unreal JPEG bytes for non-float camera streams. The wrapper publishes them as `sensor_msgs/CompressedImage` on the logical image topic's `/image/compressed` sibling, preserving the same capture stamp and paired `CameraInfo`; float/depth streams remain raw. Raw `sensor_msgs/Image` is the backward-compatible default. With direct JPEG and frame gates, also set `frame_gate_image_transport:=compressed`; the launch rejects the incompatible direct-JPEG/raw-gate combination rather than silently accepting no frames. For 1280x720 live perception, measure direct JPEG and raw delivery on the target map before choosing the transport.

A 1280x720 raw RGB frame is about 2.64 MiB. Raw gating keeps an input and an output payload, or about 105 MiB/s at 20 Hz and 316 MiB/s at 60 Hz before DDS serialization and process copies. If JPEG is acceptable for the algorithm, use `image_response_compress:=true frame_gate_image_transport:=compressed`; the gate validates and relays the original JPEG bytes without decoding or re-encoding them. Keep raw for pixel-exact imagery, labels, or float/depth data. `Ros2.Sensors` independently opts into individual AirSim sensor RPC polls, so a PX4-only pipeline can set `"Sensors": []` and enable only the extra AirSim sensor topics it needs. This does not disable PX4 HIL or PX4 `/fmu/out/...` telemetry.

Live ROS, direct MCAP recording, and CameraHost serialize their Unreal scene-capture transactions. They are safe to run together, but high-rate consumers of the same camera contend for render/readback capacity. Avoid combining a high-rate CameraHost stream or direct MCAP recording with the live wrapper for that camera unless the target map, resolution, and rates have been measured together. Also leave `CaptureSettings.ForceUpdate` disabled and hide the same capture type's HUD subwindow during high-rate capture: either enables an automatic Unreal capture that can render the target again in addition to the explicitly timestamped transaction. This affects GPU headroom, not timestamp provenance.

The default `state_history_sec:=15.0` keeps past PX4 samples long enough to associate delayed camera transport with the original capture stamp. It does not introduce a delay or clock offset. Increase it if the measured camera transport latency on the target exceeds that retention window.

PX4 `SensorCombined` gyro and acceleration values represent integration-period measurements. The synchronizer's vector interpolation is an image-associated estimate, not an instantaneous physical sample. Inspect the `Px4ImageSync` bracket fields when that distinction matters. A `SensorCombined` timestamp regression or receipt gap longer than `px4_session_gap_sec` (0.1 seconds by default) starts a new PX4 epoch and clears state/history/commands. The threshold must exceed observed normal DDS/WSL inter-arrival jitter. There is no PX4 session UUID here, so a restart with neither observable condition cannot be detected automatically; an absolute restart guarantee needs an explicit PX4/uXRCE-DDS boot-session UUID.

## Live processing and MCAP data gathering

The two paths are complementary and can run at the same time:

| Need | Path | Behavior |
|---|---|---|
| Dataset / replay of direct AirSim data | Unreal `Recording.Output="Rosbag"` | Writes chronological `/airsim_node/...` ROS 2-compatible MCAP directly in the Unreal recorder, with no duplicate TSV or image folder. Only sensors explicitly named in `Recording.Sensors` are written; every selected IMU native update is retained while its bounded source ring does not overflow. The MCAP output is unchunked and unindexed. |
| Real-time ROS 2 processing | `airsim_node` in WSL | Publishes configured live images, vehicle state, and only the individual sensors explicitly named in `Ros2.Sensors` over the existing AirSim RPC bridge. Camera and state/HIL poll periods default to 20 Hz and 100 Hz respectively and are launch-configurable; actual delivery depends on render, RPC, and subscriber load. |
| PX4-owned real-time control | `airsim_px4_offboard` plus PX4 uXRCE-DDS | Preserves AirSim image stamps, associates PX4 state at each frame, and sends optional body-rate setpoints through native PX4 ROS 2 topics. PX4 remains the only flight-command owner. |
| Actual PX4 uXRCE-DDS telemetry | Separate PX4-aware WSL recorder | Records the runtime `/<namespace>/fmu/out/...` graph with firmware-matched `px4_msgs`; it writes a separate MCAP file. |

The live IMU publisher is a **latest-value RPC poller**, not the native MCAP IMU stream. Its effective rate depends on RPC and callback load, so it cannot guarantee every native 333 Hz IMU update or an atomic camera/IMU transaction. Do not treat MCAP tailing as real-time transport. A strict native-rate live stream requires a separate bounded, fan-out RPC stream so the MCAP recorder and ROS consumer do not destructively drain the same IMU history.

The direct MCAP writer uses valid, chronological direct `Message` records but does not emit MCAP chunks or message/chunk indexes. `ros2 bag info` therefore warns and falls back to file-order reading. Sequential playback and dataset reading are correct; seeking and efficient topic-filtered reads require scanning the file. Rewrite a closed bag with an MCAP tool that adds chunks and indexes before workflows that rely on those operations.

### Performance and latency

Camera and live ROS 2 delivery rates depend on GPU rendering, RPC, the Windows/WSL network path, image size, and subscriber load. Verify throughput and latency on the target configuration before relying on a timing budget. An image's `Header.stamp` identifies the capture-state transaction that produced its pixels; callback arrival latency is a separate transport concern.

Camera names are normalized into valid ROS 2 topic tokens in both direct-AirSim paths. For example, the default AirSim camera name `0` appears as `camera_0_haf63ad4c86019caf` in live image topics and optical frame IDs. The two paths read different camera settings: the direct bag reads `Recording.Cameras`, while the live wrapper creates image publishers from `Vehicles.<vehicle>.Cameras.*.CaptureSettings`. Both support multiple configured vehicles/cameras and renderable ImageType values `0` through `11`; annotation adds its layer name to the topic. `ImageType: -1` configures the camera's main component and is deliberately not a recording/live image publisher. Define the camera explicitly under `Vehicles.<vehicle>.Cameras` for the wrapper to create image publishers. Use ROS 2-compatible vehicle and annotation names when matching bag and live topic paths, because nonconforming identifiers can normalize differently between the two paths.

For the direct Unreal MCAP path, camera images, paired `CameraInfo`, IMU, GPS, altimeter, and magnetometer channels use ROS 2 `SensorDataQoS` (keep-last depth 5, best-effort, volatile). Every bagged image has a valid frame-proven capture-state timestamp; the recorder skips it rather than falling back to the physics snapshot, request, or writer time. `Recording.Sensors` is the complete direct-sensor whitelist: list every IMU, GPS, barometer, or magnetometer needed in the bag; a missing or empty list records no direct sensor topics. Only selected IMUs have native history, and the direct auxiliary sensor channels are available for multirotor vehicles. See [Recording data](modify_recording_data.md#ros-2-mcap-bag) for the direct topic set and exact timestamp semantics.

Ground truth is deliberately separate from the normal live-control topics. The direct bag stores a simulator ground-truth NED vehicle pose only in each image's `recording/image_metadata` JSON; it does not expose a continuous standard ground-truth pose, odometry, or TF topic. Live `/airsim_node/<vehicle>/odom_local` is built from `kinematics_estimated`, while live `/environment` comes from `simGetGroundTruthEnvironment`. Treat those as distinct signals rather than assuming `odom_local` is a ground-truth stream.

## Actual PX4 `/fmu` recording in WSL

`Recording.Output="Rosbag"` and `airsim_node` record or publish AirSim data under `/airsim_node/...`; neither consumes PX4 uXRCE-DDS topics. In particular, the direct bag's IMU is AirSim `ImuBase` data, not PX4 `SensorCombined`, and enabling a `PX4Multirotor` MAVLink connection does not add `/fmu/...` messages to that file.

To record actual PX4 telemetry, run the Micro XRCE-DDS Agent and a separate recorder in WSL with `px4_msgs` built for the connected PX4 firmware. First discover the runtime graph and offered QoS; the namespace and topic suffixes are firmware and client-configuration dependent:

```shell
ros2 topic list -t
ros2 topic info --verbose <runtime-fmu-topic>
```

Record the selected discovered `/fmu/out/...` topics to a separate file, for example:

```shell
ros2 bag record -s mcap -o px4_fmu <runtime-fmu-topic>...
```

If MCAP log time must equal PX4's synchronized message time, use a PX4-aware subscriber/writer that writes the PX4 source `msg.timestamp * 1000`; normal bag recorders may use receive time. In AirSim HIL this source time is already the camera simulation-time domain. Do not append a WSL PX4 recorder to the Unreal-written MCAP file. Merge the two closed bags offline using source timestamps rather than recorder arrival time.

## Using Cosys-Airsim ROS wrapper

The ROS wrapper is composed of two ROS nodes - the first is a wrapper over Cosys-AirSim's multirotor C++ client library, and the second is a simple PD position controller.
Let's look at the ROS API for both nodes:

### Cosys-Airsim ROS Wrapper Node

#### Publishers:
Vehicle-state publishers are created for configured vehicles. Individual sensor publishers are created only for enabled sensors explicitly listed in `Ros2.Sensors`; no individual sensor topics are created when that list is omitted or empty.

- `/airsim_node/VEHICLE-NAME/car_state` [airsim_interfaces::CarState](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/CarState.msg)
  The state of the car if the vehicle is of this sim-mode type.

- `/airsim_node/VEHICLE-NAME/computervision_state` [airsim_interfaces::ComputerVisionState](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/ComputerVisionState.msg)
  The state of the computer vision actor if the vehicle is of this sim-mode type.

- `/airsim_node/origin_geo_point` [airsim_interfaces::GPSYaw](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/GPSYaw.msg)
  GPS coordinates corresponding to global frame. This is set in the airsim's [settings.json](https://cosys-lab.github.io/Cosys-AirSim/settings/) file under the `OriginGeopoint` key.

- `/airsim_node/VEHICLE-NAME/global_gps` [sensor_msgs::NavSatFix](https://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html)
  This the current GPS coordinates of the drone in airsim.

- `/airsim_node/VEHICLE-NAME/environment` [airsim_interfaces::Environment](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/Environment.msg)

- `/airsim_node/VEHICLE-NAME/odom_local` [nav_msgs::Odometry](https://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)
  Estimated odometry frame (default name: odom_local, launch name and frame type are configurable) wrt take-off point; it is not an explicit simulator ground-truth topic.

- `/airsim_node/VEHICLE-NAME/CAMERA-NAME_IMAGE-TYPE/camera_info` [sensor_msgs::CameraInfo](https://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)
  Optionally if the image type is annotation the annotation layer name is also included in the topic name.

- `/airsim_node/VEHICLE-NAME/CAMERA-NAME_IMAGE-TYPE/image` [sensor_msgs::Image](https://docs.ros.org/api/sensor_msgs/html/msg/Image.html)
  One publisher per configured vehicle camera and capture image type. RGB or float image depending on the requested type; annotation includes the annotation-layer name in the topic.

- `/tf` [tf2_msgs::TFMessage](https://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html)

- `/airsim_node/VEHICLE-NAME/altimeter/SENSOR_NAME` [airsim_interfaces::Altimeter](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/Altimeter.msg)
  This the current altimeter reading for altitude, pressure, and [QNH](https://en.wikipedia.org/wiki/QNH)

- `/airsim_node/VEHICLE-NAME/gps/SENSOR_NAME` [sensor_msgs::NavSatFix](https://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html)
  This is the configured GPS sensor reading. Valid AirSim 2D/3D fixes are mapped to ROS `STATUS_FIX`; no-fix/time-only values map to `STATUS_NO_FIX`.

- `/airsim_node/VEHICLE-NAME/imu/SENSOR_NAME` [sensor_msgs::Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
  IMU sensor data.

- `/airsim_node/VEHICLE-NAME/magnetometer/SENSOR_NAME` [sensor_msgs::MagneticField](http://docs.ros.org/api/sensor_msgs/html/msg/MagneticField.html)
  Measurement of magnetic field vector/compass in Tesla and ROS body axes.

- `/airsim_node/VEHICLE-NAME/distance/SENSOR_NAME` [sensor_msgs::Range](http://docs.ros.org/api/sensor_msgs/html/msg/Range.html)
  Measurement of distance from an active ranger, such as infrared or IR

- `/airsim_node/VEHICLE-NAME/lidar/points/SENSOR_NAME/` [sensor_msgs::PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)
  LIDAR pointcloud 

- `/airsim_node/VEHICLE-NAME/lidar/labels/SENSOR_NAME/` [airsim_interfaces::StringArray](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/StringArray.msg)
  Custom message type with an array of string that are the labels for each point in the pointcloud of the lidar sensor

- `/airsim_node/VEHICLE-NAME/gpulidar/points/SENSOR_NAME/` [sensor_msgs::PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)
  GPU LIDAR pointcloud. The instance segmentation/annotation color data is stored in the rgb field of the pointcloud. The intensity data is stored as well in the intensity field

- `/airsim_node/VEHICLE-NAME/echo/active/points/SENSOR_NAME/` [sensor_msgs::PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)
  Echo sensor pointcloud for active sensing

- `/airsim_node/VEHICLE-NAME/echo/passive/points/SENSOR_NAME/` [sensor_msgs::PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)
  Echo sensor pointcloud for passive sensing

- `/airsim_node/VEHICLE-NAME/echo/active/labels/SENSOR_NAME/` [airsim_interfaces::StringArray](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/StringArray.msg)
  Custom message type with an array of string that are the labels for each point in the pointcloud for the active echo pointcloud

- `/airsim_node/VEHICLE-NAME/echo/passive/labels/SENSOR_NAME/` [airsim_interfaces::StringArray](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/StringArray.msg)
  Custom message type with an array of string that are the labels for each point in the pointcloud for the passive echo pointcloud

- `/airsim_node/instance_segmentation_labels` [airsim_interfaces::InstanceSegmentationList](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/InstanceSegmentationList.msg)
  Custom message type with an array of a custom messages that are the names, color and index of the instance segmentation system for each object in the world.
   
- `/airsim_node/object_transforms` [airsim_interfaces::ObjectTransformsList](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/ObjectTransformsList.msg)
  Custom message type with an array of [geometry_msgs::TransformStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TransformStamped.html) that are the transforms of all objects in the world, each child frame ID is the object name.
   
#### Subscribers:

- `/airsim_node/VEHICLE-NAME/vel_cmd_body_frame` [airsim_interfaces::VelCmd](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/VelCmd.msg)
  
- `/airsim_node/VEHICLE-NAME/vel_cmd_world_frame` [airsim_interfaces::VelCmd](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/VelCmd.msg)
  
- `/airsim_node/all_robots/vel_cmd_body_frame` [airsim_interfaces::VelCmd](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/VelCmd.msg)
  Set velocity command for all drones.

- `/airsim_node/all_robots/vel_cmd_world_frame` [airsim_interfaces::VelCmd](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/VelCmd.msg)

- `/airsim_node/group_of_robots/vel_cmd_body_frame` [airsim_interfaces::VelCmdGroup](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/VelCmdGroup.msg)
  Set velocity command for a specific set of drones.
- 
- `/airsim_node/group_of_robots/vel_cmd_world_frame` [airsim_interfaces::VelCmdGroup](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/VelCmdGroup.msg)
  Set velocity command for a specific set of drones.

- `/gimbal_angle_euler_cmd` [airsim_interfaces::GimbalAngleEulerCmd](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/GimbalAngleEulerCmd.msg)
  Gimbal set point in euler angles.

- `/gimbal_angle_quat_cmd` [airsim_interfaces::GimbalAngleQuatCmd](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/GimbalAngleQuatCmd.msg)
  Gimbal set point in quaternion.

- `/airsim_node/VEHICLE-NAME/car_cmd` [airsim_interfaces::CarControls](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/CarControls.msg)
Throttle, brake, steering and gear selections for control. Both automatic and manual transmission control possible, see the [`car_joy.py`](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros/src/airsim_ros_pkgs/scripts/car_joy) script for use.

#### Services:

For the per-vehicle `takeoff` and `land` services, `success=true` with `wait_on_last_task: false` means only that the wrapper queued the asynchronous RPC request; it does not prove that AirSim accepted or completed the task. With `wait_on_last_task: true`, it reports the completed AirSim task result. Group and all-vehicle services aggregate the completed results when waiting.

- `/airsim_node/VEHICLE-NAME/land` [airsim_interfaces::Land](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/srv/Land.html)

- `/airsim_node/VEHICLE-NAME/takeoff` [airsim_interfaces::Takeoff](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/srv/Takeoff.html)

- `/airsim_node/all_robots/land` [airsim_interfaces::Land](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/srv/Land.html)
 land all drones

- `/airsim_node/all_robots/takeoff` [airsim_interfaces::Takeoff](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/srv/Takeoff.html)
 take-off all drones

- `/airsim_node/group_of_robots/land` [airsim_interfaces::LandGroup](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/srv/LandGroup.html)
 land a specific set of drones

- `/airsim_node/group_of_robots/takeoff` [airsim_interfaces::TakeoffGroup](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/srv/TakeoffGroup.html)
 take-off a specific set of drones

- `/airsim_node/reset` [airsim_interfaces::Reset](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/srv/Reset.html)
 Resets *all* vehicles

- `/airsim_node/instance_segmentation_refresh` [airsim_interfaces::RefreshInstanceSegmentation](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/srv/RefreshInstanceSegmentation.html)
 Refresh the instance segmentation list

- `/airsim_node/object_transforms_refresh` [airsim_interfaces::RefreshObjectTransforms](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/srv/RefreshObjectTransforms.html)
 Refresh the object transforms list

  

#### Parameters:

- `/airsim_node/host_ip` [string]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: localhost
  The IP of the machine running the airsim RPC API server.

- `/airsim_node/host_port` [string]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: 41451
  The port of the machine running the airsim RPC API server.

- `/airsim_node/enable_api_control` [string]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: false
  Set the API control and arm the drones on startup. If not set to true no control is available. 

- `/airsim_node/enable_object_transforms_list` [string]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: true
  Retrieve the object transforms list from the airsim API at the start or with the service to refresh. If disabled this is not available but can save time on startup.

- `/airsim_node/rpc_timeout_sec` [double]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: 5.0 seconds
  Per-request AirSim RPC timeout for the state, service, and sensor clients. Values must be between 0.1 and 300 seconds. The PX4 convenience launches forward the same argument.

- `/airsim_node/world_frame_id` [string]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: world

- `/airsim_node/odom_frame_id` [string]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: odom_local

- `/airsim_node/update_airsim_control_every_n_sec` [double]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: 0.01 seconds.
  Timer callback frequency for updating drone odom and state from airsim, and sending in control commands.
  IMU publication in this callback is a latest-value RPC poll, not native IMU delivery. The effective rate depends on RPC and callback load; the 0.01 s value requests 100 Hz but does not guarantee it and cannot retain every native 333 Hz update.

- `/airsim_node/update_airsim_img_response_every_n_sec` [double]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: 0.05 seconds.
  Timer callback frequency for receiving images from all cameras in airsim.
  The speed will depend on number of images requested and their resolution. An
  overdue poll is skipped rather than queued, so a later ROS message always
  retains its own AirSim capture timestamp.
  Timer callbacks in ROS run at maximum rate possible, so it's best to not touch this parameter.

- `/airsim_node/update_lidar_every_n_sec` [double]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: 0.01 seconds.
  Timer callback frequency for receiving images from all Lidar data in airsim.
  Timer callbacks in ROS run at maximum rate possible, so it's best to not touch this parameter.


- `/airsim_node/update_gpulidar_every_n_sec` [double]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: 0.01 seconds.
  Timer callback frequency for receiving images from all GPU-Lidar data in airsim.
  Timer callbacks in ROS run at maximum rate possible, so it's best to not touch this parameter.

- `/airsim_node/update_echo_every_n_sec` [double]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: 0.01 seconds.
  Timer callback frequency for receiving images from all echo sensor data in airsim.
  Timer callbacks in ROS run at maximum rate possible, so it's best to not touch this parameter.

- `/airsim_node/publish_clock` [double]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: false
  Will publish the ros /clock topic if set to true.

### Simple PID Position Controller Node

#### Parameters:

- PD controller parameters:
  * `/pd_position_node/kp_x` [double],
    `/pd_position_node/kp_y` [double],
    `/pd_position_node/kp_z` [double],
    `/pd_position_node/kp_yaw` [double]
    Proportional gains

  * `/pd_position_node/kd_x` [double],
    `/pd_position_node/kd_y` [double],
    `/pd_position_node/kd_z` [double],
    `/pd_position_node/kd_yaw` [double]
    Derivative gains

  * `/pd_position_node/reached_thresh_xyz` [double]
    Threshold euler distance (meters) from current position to setpoint position

  * `/pd_position_node/reached_yaw_degrees` [double]
    Threshold yaw distance (degrees) from current position to setpoint position

- `/pd_position_node/update_control_every_n_sec` [double]
  Default: 0.01 seconds

#### Services:

- `/airsim_node/VEHICLE-NAME/gps_goal` [Request: [airsim_interfaces::SetGPSPosition](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros/src/airsim_ros_pkgs/srv/SetGPSPosition.srv)]
  Target gps position + yaw.
  In **absolute** altitude.

- `/airsim_node/VEHICLE-NAME/local_position_goal` [Request: [airsim_interfaces::SetLocalPosition](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros/src/airsim_ros_pkgs/srv/SetLocalPosition.srv)]
  Target local position + yaw in global frame.

#### Subscribers:

- `/airsim_node/origin_geo_point` [airsim_interfaces::GPSYaw](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/GPSYaw.msg)
  Listens to home geo coordinates published by `airsim_node`.

- `/airsim_node/VEHICLE-NAME/odom_local` [nav_msgs::Odometry](https://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)
  Listens to odometry published by `airsim_node`

#### Publishers:

- `/vel_cmd_world_frame` [airsim_interfaces::VelCmd](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/VelCmd.msg)
  Sends velocity command to `airsim_node`

- `/vel_cmd_body_frame` [airsim_interfaces::VelCmd](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/VelCmd.msg)
  Sends velocity command to `airsim_node`

#### Global params

- Dynamic constraints. These can be changed in `dynamic_constraints.launch`:
    * `/max_vel_horz_abs` [double]
  Maximum horizontal velocity of the drone (meters/second)

    * `/max_vel_vert_abs` [double]
  Maximum vertical velocity of the drone (meters/second)

    * `/max_yaw_rate_degree` [double]
  Maximum yaw rate (degrees/second)
