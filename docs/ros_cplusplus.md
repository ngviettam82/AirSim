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
rosdep install --from-paths src -y --ignore-src --skip-keys pcl --skip-keys message_runtime --skip-keys message_generation
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

## Live processing and MCAP data gathering

The two paths are complementary and can run at the same time:

| Need | Path | Behavior |
|---|---|---|
| Dataset / replay of direct AirSim data | Unreal `Recording.Output="Rosbag"` | Writes chronological `/airsim_node/...` ROS 2-compatible MCAP directly in the Unreal recorder, with no duplicate TSV or image folder. Only sensors explicitly named in `Recording.Sensors` are written; every selected IMU native update is retained while its bounded source ring does not overflow. The MCAP output is unchunked and unindexed. |
| Real-time ROS 2 processing | `airsim_node` in WSL | Publishes configured live images, vehicle state, and only the individual sensors explicitly named in `Ros2.Sensors` over the existing AirSim RPC bridge. The image timer targets 20 Hz and the state/IMU timer targets 100 Hz; actual delivery depends on render, RPC, and subscriber load. |
| Actual PX4 uXRCE-DDS telemetry | Separate PX4-aware WSL recorder | Records the runtime `/<namespace>/fmu/out/...` graph with firmware-matched `px4_msgs`; it writes a separate MCAP file. |

The live IMU publisher is a **latest-value RPC poller**, not the native MCAP IMU stream. Its effective rate depends on RPC and callback load, so it cannot guarantee every native 333 Hz IMU update or an atomic camera/IMU transaction. Do not treat MCAP tailing as real-time transport. A strict native-rate live stream requires a separate bounded, fan-out RPC stream so the MCAP recorder and ROS consumer do not destructively drain the same IMU history.

The direct MCAP writer uses valid, chronological direct `Message` records but does not emit MCAP chunks or message/chunk indexes. `ros2 bag info` therefore warns and falls back to file-order reading. Sequential playback and dataset reading are correct; seeking and efficient topic-filtered reads require scanning the file. Rewrite a closed bag with an MCAP tool that adds chunks and indexes before workflows that rely on those operations.

### Performance and latency

Camera and live ROS 2 delivery rates depend on GPU rendering, RPC, the Windows/WSL network path, image size, and subscriber load. Verify throughput and latency on the target configuration before relying on a timing budget. An image's `Header.stamp` identifies its rendered frame; callback arrival latency is a separate transport concern.

Camera names are normalized into valid ROS 2 topic tokens in both direct-AirSim paths. For example, the default AirSim camera name `0` appears as `camera_0_haf63ad4c86019caf` in live image topics and optical frame IDs. The two paths read different camera settings: the direct bag reads `Recording.Cameras`, while the live wrapper creates image publishers from `Vehicles.<vehicle>.Cameras.*.CaptureSettings`. Both support multiple configured vehicles/cameras and renderable ImageType values `0` through `11`; annotation adds its layer name to the topic. `ImageType: -1` configures the camera's main component and is deliberately not a recording/live image publisher. Define the camera explicitly under `Vehicles.<vehicle>.Cameras` for the wrapper to create image publishers. Use ROS 2-compatible vehicle and annotation names when matching bag and live topic paths, because nonconforming identifiers can normalize differently between the two paths.

For the direct Unreal MCAP path, camera images, paired `CameraInfo`, IMU, GPS, altimeter, and magnetometer channels use ROS 2 `SensorDataQoS` (keep-last depth 5, best-effort, volatile). Every bagged image has a valid rendered-frame timestamp; the recorder skips it rather than falling back to the physics snapshot, request, or writer time. `Recording.Sensors` is the complete direct-sensor whitelist: list every IMU, GPS, barometer, or magnetometer needed in the bag; a missing or empty list records no direct sensor topics. Only selected IMUs have native history, and the direct auxiliary sensor channels are available for multirotor vehicles. See [Recording data](modify_recording_data.md#ros-2-mcap-bag) for the direct topic set and exact timestamp semantics.

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

If MCAP log time must equal PX4's synchronized message time, use a PX4-aware subscriber/writer that writes `msg.timestamp * 1000` and also records `TimesyncStatus`; normal bag recorders may use receive time. Do not append a WSL PX4 recorder to the Unreal-written MCAP file. Merge the two closed bags offline only after accounting for their independent time semantics.

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

- `/airsim_node/host_port` [string]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: 41451
  The port of the machine running the airsim RPC API server.

- `/airsim_node/is_vulkan` [string]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: True
  If using Vulkan, the image encoding is switched from rgb8 to bgr8. 

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
  Default: 0.01 seconds.
  Timer callback frequency for receiving images from all cameras in airsim.
  The speed will depend on number of images requested and their resolution.
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
