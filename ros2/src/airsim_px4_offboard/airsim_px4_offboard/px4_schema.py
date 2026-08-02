"""Exact PX4 message layouts supported by the native ROS 2 bridge.

PX4's ROS 2 messages are generated from the firmware source tree and are not
stable independently of that tree.  Native control therefore validates the
complete layouts it consumes instead of accepting a message merely because a
few field names happen to remain present.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Optional


@dataclass(frozen=True)
class Px4MessageSchema:
    fields: Mapping[str, str]
    message_version: Optional[int]


SUPPORTED_PX4_SCHEMAS = {
    'OffboardControlMode': Px4MessageSchema({
        'timestamp': 'uint64',
        'position': 'boolean',
        'velocity': 'boolean',
        'acceleration': 'boolean',
        'attitude': 'boolean',
        'body_rate': 'boolean',
        'thrust_and_torque': 'boolean',
        'direct_actuator': 'boolean',
    }, None),
    'SensorCombined': Px4MessageSchema({
        'timestamp': 'uint64',
        'gyro_rad': 'float[3]',
        'gyro_integral_dt': 'uint32',
        'accelerometer_timestamp_relative': 'int32',
        'accelerometer_m_s2': 'float[3]',
        'accelerometer_integral_dt': 'uint32',
        'accelerometer_clipping': 'uint8',
        'gyro_clipping': 'uint8',
        'accel_calibration_count': 'uint8',
        'gyro_calibration_count': 'uint8',
    }, None),
    'TimesyncStatus': Px4MessageSchema({
        'timestamp': 'uint64',
        'source_protocol': 'uint8',
        'remote_timestamp': 'uint64',
        'observed_offset': 'int64',
        'estimated_offset': 'int64',
        'round_trip_time': 'uint32',
    }, None),
    'SensorGps': Px4MessageSchema({
        'timestamp': 'uint64',
        'timestamp_sample': 'uint64',
        'device_id': 'uint32',
        'latitude_deg': 'double',
        'longitude_deg': 'double',
        'altitude_msl_m': 'double',
        'altitude_ellipsoid_m': 'double',
        's_variance_m_s': 'float',
        'c_variance_rad': 'float',
        'fix_type': 'uint8',
        'eph': 'float',
        'epv': 'float',
        'hdop': 'float',
        'vdop': 'float',
        'noise_per_ms': 'int32',
        'automatic_gain_control': 'uint16',
        'jamming_state': 'uint8',
        'jamming_indicator': 'int32',
        'spoofing_state': 'uint8',
        'authentication_state': 'uint8',
        'vel_m_s': 'float',
        'vel_n_m_s': 'float',
        'vel_e_m_s': 'float',
        'vel_d_m_s': 'float',
        'cog_rad': 'float',
        'vel_ned_valid': 'boolean',
        'timestamp_time_relative': 'int32',
        'time_utc_usec': 'uint64',
        'satellites_used': 'uint8',
        'system_error': 'uint32',
        'heading': 'float',
        'heading_offset': 'float',
        'heading_accuracy': 'float',
        'rtcm_injection_rate': 'float',
        'selected_rtcm_instance': 'uint8',
        'rtcm_crc_failed': 'boolean',
        'rtcm_msg_used': 'uint8',
    }, None),
    'VehicleAttitude': Px4MessageSchema({
        'timestamp': 'uint64',
        'timestamp_sample': 'uint64',
        'q': 'float[4]',
        'delta_q_reset': 'float[4]',
        'quat_reset_counter': 'uint8',
    }, 0),
    'VehicleOdometry': Px4MessageSchema({
        'timestamp': 'uint64',
        'timestamp_sample': 'uint64',
        'pose_frame': 'uint8',
        'position': 'float[3]',
        'q': 'float[4]',
        'velocity_frame': 'uint8',
        'velocity': 'float[3]',
        'angular_velocity': 'float[3]',
        'position_variance': 'float[3]',
        'orientation_variance': 'float[3]',
        'velocity_variance': 'float[3]',
        'reset_counter': 'uint8',
        'quality': 'int8',
    }, 0),
    'VehicleRatesSetpoint': Px4MessageSchema({
        'timestamp': 'uint64',
        'roll': 'float',
        'pitch': 'float',
        'yaw': 'float',
        'thrust_body': 'float[3]',
        'reset_integral': 'boolean',
    }, 0),
    'VehicleStatus': Px4MessageSchema({
        'timestamp': 'uint64',
        'armed_time': 'uint64',
        'takeoff_time': 'uint64',
        'arming_state': 'uint8',
        'latest_arming_reason': 'uint8',
        'latest_disarming_reason': 'uint8',
        'nav_state_timestamp': 'uint64',
        'nav_state_user_intention': 'uint8',
        'nav_state': 'uint8',
        'executor_in_charge': 'uint8',
        'valid_nav_states_mask': 'uint32',
        'can_set_nav_states_mask': 'uint32',
        'failure_detector_status': 'uint16',
        'hil_state': 'uint8',
        'vehicle_type': 'uint8',
        'failsafe': 'boolean',
        'failsafe_and_user_took_over': 'boolean',
        'failsafe_defer_state': 'uint8',
        'gcs_connection_lost': 'boolean',
        'gcs_connection_lost_counter': 'uint8',
        'high_latency_data_link_lost': 'boolean',
        'is_vtol': 'boolean',
        'is_vtol_tailsitter': 'boolean',
        'in_transition_mode': 'boolean',
        'in_transition_to_fw': 'boolean',
        'system_type': 'uint8',
        'system_id': 'uint8',
        'component_id': 'uint8',
        'safety_button_available': 'boolean',
        'safety_off': 'boolean',
        'power_input_valid': 'boolean',
        'usb_connected': 'boolean',
        'open_drone_id_system_present': 'boolean',
        'open_drone_id_system_healthy': 'boolean',
        'parachute_system_present': 'boolean',
        'parachute_system_healthy': 'boolean',
        'rc_calibration_in_progress': 'boolean',
        'calibration_enabled': 'boolean',
        'pre_flight_checks_pass': 'boolean',
    }, 1),
}


def validate_px4_message_contract(*messages) -> None:
    """Fail closed unless every message exactly matches its supported IDL."""

    for message in messages:
        message_name = type(message).__name__
        schema = SUPPORTED_PX4_SCHEMAS.get(message_name)
        if schema is None:
            raise RuntimeError(f'No supported PX4 schema is registered for {message_name}')

        get_fields = getattr(message, 'get_fields_and_field_types', None)
        if not callable(get_fields):
            raise RuntimeError(
                f'The installed px4_msgs {message_name} does not expose its ROS field layout'
            )
        actual_fields = dict(get_fields())
        expected_fields = dict(schema.fields)
        if actual_fields != expected_fields:
            missing = sorted(set(expected_fields) - set(actual_fields))
            unexpected = sorted(set(actual_fields) - set(expected_fields))
            changed = sorted(
                name for name in set(expected_fields) & set(actual_fields)
                if expected_fields[name] != actual_fields[name]
            )
            details = []
            if missing:
                details.append('missing=' + ','.join(missing))
            if unexpected:
                details.append('unexpected=' + ','.join(unexpected))
            if changed:
                details.append('changed=' + ','.join(
                    f'{name}:{actual_fields[name]}!={expected_fields[name]}' for name in changed
                ))
            raise RuntimeError(
                f'The installed px4_msgs {message_name} field layout is unsupported '
                f'({"; ".join(details)}). Generate px4_msgs from the exact PX4 source revision.'
            )

        if schema.message_version is None:
            if hasattr(message, 'MESSAGE_VERSION'):
                raise RuntimeError(
                    f'The installed px4_msgs {message_name} is unexpectedly versioned. '
                    'Generate px4_msgs from the exact PX4 source revision.'
                )
        else:
            actual_version = getattr(message, 'MESSAGE_VERSION', None)
            if actual_version != schema.message_version:
                raise RuntimeError(
                    f'The installed px4_msgs {message_name} MESSAGE_VERSION is '
                    f'{actual_version}, expected {schema.message_version}. Generate px4_msgs '
                    'from the exact PX4 source revision.'
                )
