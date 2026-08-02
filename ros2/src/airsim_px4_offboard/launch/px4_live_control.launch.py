"""Convenience launch for one AirSim wrapper and one PX4 vehicle pipeline."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from airsim_px4_offboard.control_safety import DEFAULT_PX4_SESSION_GAP_SEC


def _argument(name, default_value, description):
    return DeclareLaunchArgument(name, default_value=default_value, description=description)


def generate_launch_description():
    package_share = get_package_share_directory('airsim_px4_offboard')
    wrapper_launch = os.path.join(package_share, 'launch', 'airsim_px4_wrapper.launch.py')
    vehicle_launch = os.path.join(package_share, 'launch', 'px4_vehicle.launch.py')

    passthrough_defaults = {
        'output': ('screen', 'Launch output destination.'),
        'host_ip': ('localhost', 'AirSim RPC host.'),
        'host_port': ('41451', 'AirSim RPC port.'),
        'rpc_timeout_sec': ('5.0', 'Per-request AirSim RPC timeout in seconds.'),
        'update_airsim_img_response_every_n_sec': (
            '0.05',
            'AirSim camera poll period in seconds; lower values request a higher frame rate.',
        ),
        'update_airsim_control_every_n_sec': (
            '0.01',
            'AirSim state/HIL-history poll period in seconds.',
        ),
        'image_response_compress': (
            'False',
            'Request and publish direct JPEG camera messages instead of raw Image messages.',
        ),
        'enable_object_transforms_list': ('True', 'Publish AirSim object transforms.'),
        'node_namespace': ('drone1', 'Unique ROS namespace for this PX4 vehicle.'),
        'airsim_vehicle_name': ('drone1', 'Exact AirSim vehicle in every camera topic.'),
        'expected_px4_system_id': ('1', 'PX4 VehicleStatus system_id bound to this vehicle.'),
        'camera_topics': ('', 'Semicolon-separated AirSim Image topics.'),
        'camera_output_prefixes': ('', 'Optional semicolon-separated sync output prefixes.'),
        'primary_camera_index': ('0', 'Camera whose synchronization gates control.'),
        'px4_topic_prefix': ('/fmu', 'PX4 DDS prefix for this vehicle.'),
        'px4_vehicle_status_suffix': (
            '_v1',
            'PX4 VehicleStatus version suffix appended to the standard topic; empty is unversioned.',
        ),
        'px4_vehicle_status_topic': (
            '',
            'Optional full PX4 VehicleStatus topic override; takes precedence over '
            'px4_vehicle_status_suffix.',
        ),
        'control_mode_topic': ('/airsim_node/control_mode', 'AirSim ownership status topic.'),
        'hil_clock_topic': (
            '',
            'AirSim HIL history topic; empty derives /airsim_node/<vehicle>/px4/hil_sensor_clock.',
        ),
        'rate_setpoint_topic': ('rate_setpoint', 'Namespaced Px4RateSetpoint input.'),
        'rate_control_output_prefix': ('rate_control', 'Namespaced controller status prefix.'),
        'start_rate_control': ('True', 'Start native PX4 rate control.'),
        'require_image_sync': ('True', 'Gate commands on fresh image synchronization.'),
        'start_frame_gates': (
            'True',
            'Relay camera payloads only after exact accepted PX4 image synchronization. '
            'Disable only for observation-only pipelines that never authorize control.',
        ),
        'frame_gate_image_transport': (
            'raw',
            "Payload transport for all frame gates: 'raw' or 'compressed'.",
        ),
        'frame_gate_max_pending_frames': (
            '5',
            'Bounded raw/compressed payload and synchronization cache depth.',
        ),
        'frame_gate_max_frame_wait_sec': (
            '0.15',
            'Maximum time one image or synchronization half waits for its exact match.',
        ),
        'frame_gate_max_source_age_sec': (
            '0.25',
            'Maximum PX4 HIL source age of a synchronized image payload.',
        ),
        'publish_gps': ('True', 'Publish causal PX4 GPS associations.'),
        'require_gps': ('False', 'Require fresh GPS for every synchronized pair.'),
        'world_frame_id': ('world_enu', 'Synchronized odometry world frame.'),
        'body_frame_id': ('', 'Body frame; empty derives it from node_namespace.'),
        'gps_frame_id': ('', 'GPS frame; empty derives it from node_namespace.'),
        'state_history_sec': ('15.0', 'PX4 history retained for image latency.'),
        'max_state_interpolation_gap_sec': ('0.05', 'Largest state bracket.'),
        'max_imu_interpolation_gap_sec': ('0.02', 'Largest IMU bracket.'),
        'sync_health_timeout_sec': ('0.5', 'Synchronizer health timeout.'),
        'max_gps_age_sec': ('1.0', 'Maximum associated GPS age.'),
        'vehicle_status_timeout_sec': ('10.0', 'Startup timeout for the bound PX4 identity.'),
        'vehicle_status_max_age_sec': ('3.0', 'Maximum age of PX4 VehicleStatus.'),
        'px4_clock_timeout_sec': ('10.0', 'Startup timeout for an advancing PX4 HIL clock.'),
        'px4_clock_max_age_sec': ('0.5', 'Maximum age of PX4 SensorCombined.'),
        'max_px4_stream_lead_sec': ('0.05', 'Maximum stream lead over PX4 SensorCombined.'),
        'px4_session_gap_sec': (
            str(DEFAULT_PX4_SESSION_GAP_SEC),
            'SensorCombined receipt gap that fences a restarted PX4 session.',
        ),
        'px4_dds_timesync_guard_sec': (
            '3.0',
            'DDS observation window that rejects PX4 timestamp translation.',
        ),
        'direct_hil_clock_matches_required': (
            '3',
            'Minimum exact HIL_SENSOR/SensorCombined matches before pairing images.',
        ),
        'direct_hil_clock_proof_timeout_sec': (
            '10.0',
            'Startup timeout for proving the direct AirSim/PX4 HIL clock.',
        ),
        'hil_clock_max_age_sec': ('0.5', 'Maximum receipt age of the AirSim HIL timestamp history.'),
        'setpoint_timeout_sec': ('0.25', 'Maximum setpoint transport age.'),
        'setpoint_source_timeout_sec': ('0.25', 'Maximum setpoint HIL source age.'),
        'image_sync_timeout_sec': ('0.5', 'Maximum image-sync receipt age.'),
        'publish_rate_hz': ('100.0', 'Native PX4 setpoint rate.'),
    }
    arguments = [
        _argument(name, default, description)
        for name, (default, description) in passthrough_defaults.items()
    ]

    wrapper_argument_names = (
        'output',
        'host_ip',
        'host_port',
        'rpc_timeout_sec',
        'update_airsim_img_response_every_n_sec',
        'update_airsim_control_every_n_sec',
        'image_response_compress',
        'enable_object_transforms_list',
        'world_frame_id',
    )
    wrapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(wrapper_launch),
        launch_arguments={
            name: LaunchConfiguration(name)
            for name in wrapper_argument_names
        }.items(),
    )
    vehicle_argument_names = tuple(
        name for name in passthrough_defaults if name not in wrapper_argument_names
    )
    vehicle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vehicle_launch),
        launch_arguments={
            'output': LaunchConfiguration('output'),
            'image_response_compress': LaunchConfiguration('image_response_compress'),
            'world_frame_id': LaunchConfiguration('world_frame_id'),
            **{name: LaunchConfiguration(name) for name in vehicle_argument_names},
        }.items(),
    )
    return LaunchDescription(arguments + [wrapper, vehicle])
