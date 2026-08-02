"""Launch one namespaced PX4 controller and one synchronizer per camera."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from airsim_px4_offboard.launch_helpers import (
    camera_info_topic,
    camera_node_name,
    camera_pipelines,
    join_topic_prefix,
    parse_bool,
    primary_camera_topic,
    primary_gated_sync_topic,
    primary_sync_topic,
    resolve_px4_vehicle_status_topic,
    validate_live_control_topology,
    validate_frame_gate_transport,
    validate_camera_vehicle,
)
from airsim_px4_offboard.control_safety import DEFAULT_PX4_SESSION_GAP_SEC


def _argument(name, default_value, description):
    return DeclareLaunchArgument(name, default_value=default_value, description=description)


def _value(context, name):
    return LaunchConfiguration(name).perform(context)


def _launch_setup(context):
    output = _value(context, 'output')
    namespace = _value(context, 'node_namespace').strip('/')
    airsim_vehicle_name = _value(context, 'airsim_vehicle_name').strip('/')
    if not airsim_vehicle_name:
        raise ValueError('airsim_vehicle_name must not be empty')
    expected_px4_system_id = int(_value(context, 'expected_px4_system_id'))
    if expected_px4_system_id < 1 or expected_px4_system_id > 255:
        raise ValueError('expected_px4_system_id must be in [1, 255]')
    start_rate_control = parse_bool(_value(context, 'start_rate_control'))
    require_image_sync = parse_bool(_value(context, 'require_image_sync'))
    start_frame_gates = parse_bool(_value(context, 'start_frame_gates'))
    frame_gate_image_transport = _value(context, 'frame_gate_image_transport').strip().lower()
    image_response_compress = parse_bool(_value(context, 'image_response_compress'))
    validate_frame_gate_transport(
        image_response_compress=image_response_compress,
        start_frame_gates=start_frame_gates,
        frame_gate_image_transport=frame_gate_image_transport,
    )
    validate_live_control_topology(
        start_rate_control=start_rate_control,
        require_image_sync=require_image_sync,
        start_frame_gates=start_frame_gates,
    )
    pipelines = camera_pipelines(
        _value(context, 'camera_topics'),
        _value(context, 'camera_output_prefixes'),
        reserve_gated_prefixes=start_frame_gates,
    )
    for camera_topic, _ in pipelines:
        validate_camera_vehicle(camera_topic, airsim_vehicle_name)
    if start_rate_control and require_image_sync and not pipelines:
        raise ValueError('rate control requires camera_topics when require_image_sync is true')
    if start_frame_gates and not pipelines:
        raise ValueError('frame gates require camera_topics')
    if frame_gate_image_transport not in ('raw', 'compressed'):
        raise ValueError("frame_gate_image_transport must be 'raw' or 'compressed'")

    px4_prefix = _value(context, 'px4_topic_prefix')
    px4_vehicle_status_topic = resolve_px4_vehicle_status_topic(
        px4_prefix,
        _value(context, 'px4_vehicle_status_topic'),
        _value(context, 'px4_vehicle_status_suffix'),
    )
    body_frame_id = _value(context, 'body_frame_id').strip()
    gps_frame_id = _value(context, 'gps_frame_id').strip()
    if not body_frame_id:
        body_frame_id = f'{namespace}/base_link_flu' if namespace else 'base_link_flu'
    if not gps_frame_id:
        gps_frame_id = f'{namespace}/gps_link' if namespace else 'gps_link'
    common_sync_parameters = {
        'airsim_vehicle_name': airsim_vehicle_name,
        'px4_topic_prefix': px4_prefix,
        'expected_px4_system_id': expected_px4_system_id,
        'control_mode_topic': _value(context, 'control_mode_topic'),
        'px4_odometry_topic': join_topic_prefix(px4_prefix, 'out/vehicle_odometry'),
        'px4_attitude_topic': join_topic_prefix(px4_prefix, 'out/vehicle_attitude'),
        'px4_sensor_combined_topic': join_topic_prefix(px4_prefix, 'out/sensor_combined'),
        'px4_gps_topic': join_topic_prefix(px4_prefix, 'out/vehicle_gps_position'),
        'px4_vehicle_status_topic': px4_vehicle_status_topic,
        'px4_timesync_status_topic': join_topic_prefix(px4_prefix, 'out/timesync_status'),
        'hil_clock_topic': _value(context, 'hil_clock_topic'),
        'publish_gps': parse_bool(_value(context, 'publish_gps')),
        'require_gps': parse_bool(_value(context, 'require_gps')),
        'world_frame_id': _value(context, 'world_frame_id'),
        'body_frame_id': body_frame_id,
        'gps_frame_id': gps_frame_id,
        'state_history_sec': float(_value(context, 'state_history_sec')),
        'max_state_interpolation_gap_sec': float(
            _value(context, 'max_state_interpolation_gap_sec')
        ),
        'max_imu_interpolation_gap_sec': float(
            _value(context, 'max_imu_interpolation_gap_sec')
        ),
        'sync_health_timeout_sec': float(_value(context, 'sync_health_timeout_sec')),
        'max_gps_age_sec': float(_value(context, 'max_gps_age_sec')),
        'vehicle_status_timeout_sec': float(_value(context, 'vehicle_status_timeout_sec')),
        'vehicle_status_max_age_sec': float(_value(context, 'vehicle_status_max_age_sec')),
        'px4_clock_timeout_sec': float(_value(context, 'px4_clock_timeout_sec')),
        'px4_clock_max_age_sec': float(_value(context, 'px4_clock_max_age_sec')),
        'max_px4_stream_lead_sec': float(_value(context, 'max_px4_stream_lead_sec')),
        'px4_session_gap_sec': float(_value(context, 'px4_session_gap_sec')),
        'px4_dds_timesync_guard_sec': float(_value(context, 'px4_dds_timesync_guard_sec')),
        'direct_hil_clock_matches_required': int(
            _value(context, 'direct_hil_clock_matches_required')
        ),
        'direct_hil_clock_proof_timeout_sec': float(
            _value(context, 'direct_hil_clock_proof_timeout_sec')
        ),
        'hil_clock_max_age_sec': float(_value(context, 'hil_clock_max_age_sec')),
    }

    actions = []
    for index, (camera_topic, output_prefix) in enumerate(pipelines):
        parameters = dict(common_sync_parameters)
        parameters.update({
            'camera_topic': camera_topic,
            'output_prefix': output_prefix,
        })
        actions.append(Node(
            package='airsim_px4_offboard',
            executable='px4_camera_sync',
            namespace=namespace,
            name=camera_node_name(index, camera_topic),
            output=output,
            parameters=[parameters],
        ))
        if start_frame_gates:
            gate_image_topic = camera_topic.rstrip('/')
            if (
                frame_gate_image_transport == 'compressed'
                and not gate_image_topic.endswith('/compressed')
            ):
                gate_image_topic += '/compressed'
            actions.append(Node(
                package='airsim_px4_offboard',
                executable='px4_frame_gate',
                namespace=namespace,
                name=f'frame_gate_{index}',
                output=output,
                parameters=[{
                    'image_topic': gate_image_topic,
                    'image_transport': frame_gate_image_transport,
                    'camera_info_topic': camera_info_topic(camera_topic),
                    'image_sync_topic': join_topic_prefix(output_prefix, 'image_sync'),
                    'sync_status_topic': join_topic_prefix(output_prefix, 'status'),
                    'output_prefix': f'{output_prefix}_gated',
                    # The synchronizer reports exactly the camera topic it
                    # was configured with, which may be raw even when this
                    # gate consumes its image_transport compressed sibling.
                    'expected_sync_image_topic': camera_topic,
                    'airsim_vehicle_name': airsim_vehicle_name,
                    'px4_topic_prefix': px4_prefix,
                    'expected_px4_system_id': expected_px4_system_id,
                    'max_pending_frames': int(_value(context, 'frame_gate_max_pending_frames')),
                    'max_frame_wait_sec': float(_value(context, 'frame_gate_max_frame_wait_sec')),
                    'max_source_age_sec': float(_value(context, 'frame_gate_max_source_age_sec')),
                }],
            ))

    if start_rate_control:
        sync_topic = ''
        expected_sync_image_topic = ''
        if require_image_sync:
            sync_topic = primary_sync_topic(
                pipelines, _value(context, 'primary_camera_index')
            )
            expected_sync_image_topic = primary_camera_topic(
                pipelines, _value(context, 'primary_camera_index')
            )
            # When a frame gate is enabled, rate commands must be authorized
            # by the relay event that proves the image payload itself was
            # observed alongside the accepted synchronizer event.
            if start_frame_gates:
                sync_topic = primary_gated_sync_topic(
                    pipelines, _value(context, 'primary_camera_index')
                )
        actions.append(Node(
            package='airsim_px4_offboard',
            executable='px4_rate_control',
            namespace=namespace,
            name='rate_control',
            output=output,
            parameters=[{
                'airsim_vehicle_name': airsim_vehicle_name,
                'px4_topic_prefix': px4_prefix,
                'expected_px4_system_id': expected_px4_system_id,
                'setpoint_topic': _value(context, 'rate_setpoint_topic'),
                'output_prefix': _value(context, 'rate_control_output_prefix'),
                'control_mode_topic': _value(context, 'control_mode_topic'),
                'px4_clock_topic': join_topic_prefix(px4_prefix, 'out/sensor_combined'),
                'px4_vehicle_status_topic': px4_vehicle_status_topic,
                'px4_timesync_status_topic': join_topic_prefix(
                    px4_prefix, 'out/timesync_status'
                ),
                'px4_clock_timeout_sec': float(_value(context, 'px4_clock_timeout_sec')),
                'px4_clock_max_age_sec': float(_value(context, 'px4_clock_max_age_sec')),
                'image_sync_topic': sync_topic,
                'expected_sync_image_topic': expected_sync_image_topic,
                'require_image_sync': require_image_sync,
                'px4_rates_setpoint_topic': join_topic_prefix(
                    px4_prefix, 'in/vehicle_rates_setpoint'
                ),
                'px4_offboard_control_mode_topic': join_topic_prefix(
                    px4_prefix, 'in/offboard_control_mode'
                ),
                'setpoint_timeout_sec': float(_value(context, 'setpoint_timeout_sec')),
                'setpoint_source_timeout_sec': float(
                    _value(context, 'setpoint_source_timeout_sec')
                ),
                'image_sync_timeout_sec': float(_value(context, 'image_sync_timeout_sec')),
                'px4_session_gap_sec': float(_value(context, 'px4_session_gap_sec')),
                'px4_dds_timesync_guard_sec': float(
                    _value(context, 'px4_dds_timesync_guard_sec')
                ),
                'vehicle_status_timeout_sec': float(
                    _value(context, 'vehicle_status_timeout_sec')
                ),
                'vehicle_status_max_age_sec': float(
                    _value(context, 'vehicle_status_max_age_sec')
                ),
                'publish_rate_hz': float(_value(context, 'publish_rate_hz')),
            }],
        ))
    return actions


def generate_launch_description():
    arguments = [
        _argument('output', 'screen', 'Launch output destination.'),
        _argument('node_namespace', 'drone1', 'Unique ROS namespace for this PX4 vehicle.'),
        _argument('airsim_vehicle_name', 'drone1', 'Exact AirSim vehicle in every camera topic.'),
        _argument('expected_px4_system_id', '1', 'PX4 VehicleStatus system_id bound to this vehicle.'),
        _argument(
            'camera_topics', '',
            'Semicolon-separated AirSim Image topics. One synchronizer is created per entry.',
        ),
        _argument(
            'camera_output_prefixes', '',
            'Optional semicolon-separated relative output prefixes matching camera_topics.',
        ),
        _argument('primary_camera_index', '0', 'Camera whose sync events gate rate control.'),
        _argument('px4_topic_prefix', '/fmu', 'PX4 DDS prefix for this vehicle.'),
        _argument(
            'px4_vehicle_status_suffix',
            '_v1',
            'PX4 VehicleStatus version suffix appended to the standard topic; empty is unversioned.',
        ),
        _argument(
            'px4_vehicle_status_topic',
            '',
            'Optional full PX4 VehicleStatus topic override; takes precedence over '
            'px4_vehicle_status_suffix.',
        ),
        _argument('control_mode_topic', '/airsim_node/control_mode', 'AirSim ownership status topic.'),
        _argument(
            'image_response_compress',
            'False',
            'Whether the AirSim wrapper publishes direct JPEG instead of raw camera payloads.',
        ),
        _argument(
            'hil_clock_topic', '',
            'AirSim HIL history topic; empty derives /airsim_node/<vehicle>/px4/hil_sensor_clock.',
        ),
        _argument('rate_setpoint_topic', 'rate_setpoint', 'Namespaced Px4RateSetpoint input.'),
        _argument('rate_control_output_prefix', 'rate_control', 'Namespaced controller status prefix.'),
        _argument('start_rate_control', 'True', 'Start the bounded PX4 rate bridge.'),
        _argument('require_image_sync', 'True', 'Stop rate output unless primary camera sync is fresh.'),
        _argument(
            'start_frame_gates',
            'True',
            'Relay camera payloads only after exact accepted PX4 image synchronization. '
            'Disable only for observation-only pipelines that never authorize control.',
        ),
        _argument(
            'frame_gate_image_transport',
            'raw',
            "Payload transport for all frame gates: 'raw' or 'compressed'.",
        ),
        _argument(
            'frame_gate_max_pending_frames',
            '5',
            'Bounded raw/compressed payload and synchronization cache depth.',
        ),
        _argument(
            'frame_gate_max_frame_wait_sec',
            '0.15',
            'Maximum time one image or synchronization half waits for its exact match.',
        ),
        _argument(
            'frame_gate_max_source_age_sec',
            '0.25',
            'Maximum PX4 HIL source age of a synchronized image payload.',
        ),
        _argument('publish_gps', 'True', 'Publish causal PX4 GPS associations when fresh.'),
        _argument('require_gps', 'False', 'Require fresh GPS before emitting a synchronized pair.'),
        _argument('world_frame_id', 'world_enu', 'Synchronized odometry world frame.'),
        _argument('body_frame_id', '', 'Body frame; empty derives <namespace>/base_link_flu.'),
        _argument('gps_frame_id', '', 'GPS frame; empty derives <namespace>/gps_link.'),
        _argument('state_history_sec', '15.0', 'PX4 state history retained for delayed image transport.'),
        _argument('max_state_interpolation_gap_sec', '0.05', 'Largest odometry/attitude bracket.'),
        _argument('max_imu_interpolation_gap_sec', '0.02', 'Largest gyro/accelerometer bracket.'),
        _argument('sync_health_timeout_sec', '0.5', 'Pair-loss timeout reported by each synchronizer.'),
        _argument('max_gps_age_sec', '1.0', 'Maximum causal GPS age associated with an image.'),
        _argument('vehicle_status_timeout_sec', '10.0', 'Startup timeout for the bound PX4 identity.'),
        _argument('vehicle_status_max_age_sec', '3.0', 'Maximum age of PX4 VehicleStatus.'),
        _argument('px4_clock_timeout_sec', '10.0', 'Startup timeout for an advancing PX4 HIL clock.'),
        _argument('px4_clock_max_age_sec', '0.5', 'Maximum age of PX4 SensorCombined.'),
        _argument('max_px4_stream_lead_sec', '0.05', 'Maximum PX4 stream lead over SensorCombined.'),
        _argument(
            'px4_session_gap_sec',
            str(DEFAULT_PX4_SESSION_GAP_SEC),
            'SensorCombined receipt gap that fences a restarted PX4 session.',
        ),
        _argument(
            'px4_dds_timesync_guard_sec',
            '3.0',
            'DDS observation window that rejects PX4 timestamp translation.',
        ),
        _argument(
            'direct_hil_clock_matches_required',
            '3',
            'Minimum exact HIL_SENSOR/SensorCombined matches before pairing images.',
        ),
        _argument(
            'direct_hil_clock_proof_timeout_sec',
            '10.0',
            'Startup timeout for proving the direct AirSim/PX4 HIL clock.',
        ),
        _argument(
            'hil_clock_max_age_sec',
            '0.5',
            'Maximum receipt age of the AirSim HIL timestamp history.',
        ),
        _argument('setpoint_timeout_sec', '0.25', 'Maximum command transport age.'),
        _argument('setpoint_source_timeout_sec', '0.25', 'Maximum command source age in HIL time.'),
        _argument('image_sync_timeout_sec', '0.5', 'Maximum primary-camera sync receipt age.'),
        _argument('publish_rate_hz', '100.0', 'Native PX4 setpoint publication rate.'),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_setup)])
