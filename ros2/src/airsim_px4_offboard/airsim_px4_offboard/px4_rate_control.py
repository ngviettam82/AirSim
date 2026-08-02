"""Fail-closed ROS 2 body-rate bridge for PX4-owned AirSim control.

The bridge never arms a vehicle or requests OFFBOARD mode.  It publishes only
while PX4 HIL time advances, image synchronization is fresh, and a bounded
setpoint has a new source timestamp in that same simulation clock.
"""

from __future__ import annotations

import math
import time
from collections import deque
from typing import Deque, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from airsim_interfaces.msg import Px4ImageSync, Px4RateSetpoint
from std_msgs.msg import String

from .control_safety import (
    DEFAULT_PX4_SESSION_GAP_SEC,
    RateCommand,
    command_readiness,
    receipt_gap_exceeds,
    seconds_to_positive_nanoseconds,
    timestamp_progress,
    validate_rate_values,
    validate_source_timestamp,
)

try:  # Optional at ordinary AirSim build time; required when this node runs.
    from px4_msgs.msg import (
        OffboardControlMode,
        SensorCombined,
        TimesyncStatus,
        VehicleRatesSetpoint,
        VehicleStatus,
    )
    _PX4_IMPORT_ERROR: Optional[Exception] = None
except ImportError as error:  # pragma: no cover - depends on the target ROS overlay
    OffboardControlMode = None  # type: ignore[assignment,misc]
    SensorCombined = None  # type: ignore[assignment,misc]
    TimesyncStatus = None  # type: ignore[assignment,misc]
    VehicleRatesSetpoint = None  # type: ignore[assignment,misc]
    VehicleStatus = None  # type: ignore[assignment,misc]
    _PX4_IMPORT_ERROR = error

from .px4_schema import validate_px4_message_contract


_MIN_DIRECT_HIL_CLOCK_MATCHES = 3


def _parameter(node: Node, name: str, default):
    node.declare_parameter(name, default)
    return node.get_parameter(name).value


def _join_topic(prefix: str, suffix: str) -> str:
    cleaned_prefix = prefix.rstrip('/')
    cleaned_suffix = suffix.strip('/')
    if not cleaned_prefix:
        raise ValueError('topic prefix must not be empty')
    return f'{cleaned_prefix}/{cleaned_suffix}'


def _stamp_ns(header) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def _validate_px4_message_contract() -> None:
    """Reject every PX4 IDL other than the exact supported firmware layout."""

    validate_px4_message_contract(
        OffboardControlMode(),
        SensorCombined(),
        TimesyncStatus(),
        VehicleRatesSetpoint(),
        VehicleStatus(),
    )


def _configure_body_rate_mode(offboard_mode) -> None:
    offboard_mode.position = False
    offboard_mode.velocity = False
    offboard_mode.acceleration = False
    offboard_mode.attitude = False
    offboard_mode.body_rate = True
    offboard_mode.thrust_and_torque = False
    offboard_mode.direct_actuator = False


class Px4RateControl(Node):
    """Forward fresh ROS-FLU body rates to native PX4 offboard topics."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__('px4_rate_control', parameter_overrides=parameter_overrides)
        if _PX4_IMPORT_ERROR is not None:
            raise RuntimeError(
                'PX4 mode requires px4_msgs generated from the connected PX4 source revision. '
                f'Import failed: {_PX4_IMPORT_ERROR}'
            )
        _validate_px4_message_contract()

        self._setpoint_topic = str(_parameter(self, 'setpoint_topic', 'rate_setpoint'))
        self._output_prefix = str(_parameter(self, 'output_prefix', 'rate_control'))
        self._airsim_vehicle_name = str(
            _parameter(self, 'airsim_vehicle_name', 'drone1')
        ).strip('/')
        self._px4_topic_prefix = str(_parameter(self, 'px4_topic_prefix', '/fmu')).rstrip('/')
        self._expected_px4_system_id = int(_parameter(self, 'expected_px4_system_id', 1))
        self._control_mode_topic = str(
            _parameter(self, 'control_mode_topic', '/airsim_node/control_mode')
        )
        self._px4_clock_topic = str(
            _parameter(self, 'px4_clock_topic', '/fmu/out/sensor_combined')
        )
        self._image_sync_topic = str(_parameter(self, 'image_sync_topic', 'camera_sync/image_sync'))
        self._expected_sync_image_topic = str(
            _parameter(self, 'expected_sync_image_topic', '')
        ).rstrip('/')
        self._px4_rates_topic = str(
            _parameter(self, 'px4_rates_setpoint_topic', '/fmu/in/vehicle_rates_setpoint')
        )
        self._px4_offboard_mode_topic = str(
            _parameter(self, 'px4_offboard_control_mode_topic', '/fmu/in/offboard_control_mode')
        )
        self._px4_vehicle_status_topic = str(
            _parameter(self, 'px4_vehicle_status_topic', '/fmu/out/vehicle_status_v1')
        )
        self._px4_timesync_status_topic = str(
            _parameter(self, 'px4_timesync_status_topic', '/fmu/out/timesync_status')
        )

        self._require_px4_mode = bool(_parameter(self, 'require_px4_control_mode', True))
        self._require_image_sync = bool(_parameter(self, 'require_image_sync', True))
        self._control_mode_timeout_sec = float(_parameter(self, 'control_mode_timeout_sec', 10.0))
        self._px4_clock_timeout_sec = float(_parameter(self, 'px4_clock_timeout_sec', 10.0))
        self._px4_clock_max_age_sec = float(_parameter(self, 'px4_clock_max_age_sec', 0.5))
        self._vehicle_status_timeout_sec = float(
            _parameter(self, 'vehicle_status_timeout_sec', 10.0)
        )
        self._vehicle_status_max_age_sec = float(
            _parameter(self, 'vehicle_status_max_age_sec', 3.0)
        )
        self._setpoint_timeout_sec = float(_parameter(self, 'setpoint_timeout_sec', 0.25))
        self._setpoint_source_timeout_sec = float(
            _parameter(self, 'setpoint_source_timeout_sec', 0.25)
        )
        self._max_future_setpoint_sec = float(
            _parameter(self, 'max_future_setpoint_sec', 0.02)
        )
        self._image_sync_timeout_sec = float(_parameter(self, 'image_sync_timeout_sec', 0.5))
        self._px4_session_gap_sec = float(
            _parameter(self, 'px4_session_gap_sec', DEFAULT_PX4_SESSION_GAP_SEC)
        )
        self._px4_dds_timesync_guard_sec = float(
            _parameter(self, 'px4_dds_timesync_guard_sec', 3.0)
        )
        self._publish_rate_hz = float(_parameter(self, 'publish_rate_hz', 100.0))
        self._max_roll_rate = float(_parameter(self, 'max_roll_rate_rad_s', 6.0))
        self._max_pitch_rate = float(_parameter(self, 'max_pitch_rate_rad_s', 6.0))
        self._max_yaw_rate = float(_parameter(self, 'max_yaw_rate_rad_s', 3.0))

        if not self._setpoint_topic or not self._output_prefix.strip('/'):
            raise ValueError('setpoint_topic and output_prefix must not be empty')
        if not self._airsim_vehicle_name or not self._px4_topic_prefix:
            raise ValueError('airsim_vehicle_name and px4_topic_prefix must not be empty')
        if self._expected_px4_system_id < 1 or self._expected_px4_system_id > 255:
            raise ValueError('expected_px4_system_id must be in [1, 255]')
        if not self._px4_vehicle_status_topic:
            raise ValueError('px4_vehicle_status_topic must not be empty')
        if not self._px4_timesync_status_topic:
            raise ValueError('px4_timesync_status_topic must not be empty')
        if self._require_image_sync and not self._image_sync_topic:
            raise ValueError('require_image_sync needs a non-empty image_sync_topic')
        if self._require_image_sync and not self._expected_sync_image_topic:
            raise ValueError('require_image_sync needs expected_sync_image_topic')
        numeric_parameters = (
            self._control_mode_timeout_sec,
            self._px4_clock_timeout_sec,
            self._px4_clock_max_age_sec,
            self._vehicle_status_timeout_sec,
            self._vehicle_status_max_age_sec,
            self._setpoint_timeout_sec,
            self._setpoint_source_timeout_sec,
            self._max_future_setpoint_sec,
            self._image_sync_timeout_sec,
            self._px4_session_gap_sec,
            self._px4_dds_timesync_guard_sec,
            self._publish_rate_hz,
            self._max_roll_rate,
            self._max_pitch_rate,
            self._max_yaw_rate,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in numeric_parameters):
            raise ValueError('PX4 control timing and rate-limit parameters must be finite and positive')

        self._clock_max_age_ns = int(self._px4_clock_max_age_sec * 1_000_000_000.0)
        self._vehicle_status_max_age_ns = int(
            self._vehicle_status_max_age_sec * 1_000_000_000.0
        )
        self._setpoint_timeout_ns = int(self._setpoint_timeout_sec * 1_000_000_000.0)
        self._source_timeout_ns = int(self._setpoint_source_timeout_sec * 1_000_000_000.0)
        self._future_tolerance_ns = int(self._max_future_setpoint_sec * 1_000_000_000.0)
        self._sync_timeout_ns = int(self._image_sync_timeout_sec * 1_000_000_000.0)
        self._px4_session_gap_ns = seconds_to_positive_nanoseconds(
            self._px4_session_gap_sec, 'px4_session_gap_sec'
        )
        self._px4_dds_timesync_guard_ns = int(
            self._px4_dds_timesync_guard_sec * 1_000_000_000.0
        )

        self._mode_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(
            String, _join_topic(self._output_prefix, 'status'), self._mode_qos
        )
        self._rates_pub = self.create_publisher(
            VehicleRatesSetpoint, self._px4_rates_topic, qos_profile_sensor_data
        )
        self._offboard_mode_pub = self.create_publisher(
            OffboardControlMode, self._px4_offboard_mode_topic, qos_profile_sensor_data
        )

        self._mode_sub = self.create_subscription(
            String, self._control_mode_topic, self._control_mode_cb, self._mode_qos
        )
        self._px4_clock_sub = self.create_subscription(
            SensorCombined, self._px4_clock_topic, self._px4_clock_cb, qos_profile_sensor_data
        )
        self._vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            self._px4_vehicle_status_topic,
            self._vehicle_status_cb,
            qos_profile_sensor_data,
        )
        self._timesync_status_sub = self.create_subscription(
            TimesyncStatus,
            self._px4_timesync_status_topic,
            self._timesync_status_cb,
            qos_profile_sensor_data,
        )
        self._image_sync_sub = None
        if self._require_image_sync:
            self._image_sync_sub = self.create_subscription(
                Px4ImageSync, self._image_sync_topic, self._image_sync_cb, qos_profile_sensor_data
            )
        self._setpoint_sub = self.create_subscription(
            Px4RateSetpoint, self._setpoint_topic, self._setpoint_cb, qos_profile_sensor_data
        )
        self._publish_timer = self.create_timer(1.0 / self._publish_rate_hz, self._publish_timer_cb)
        self._health_timer = self.create_timer(0.1, self._health_cb)

        self._started_ns = time.monotonic_ns()
        self._last_px4_clock_receipt_ns: Optional[int] = None
        self._latest_px4_timestamp_us: Optional[int] = None
        self._px4_clock_epoch = 0
        self._dds_timesync_guard_started_ns: Optional[int] = None
        self._dds_timesync_guard_verified = False
        self._last_vehicle_status_receipt_ns: Optional[int] = None
        self._validated_px4_system_id: Optional[int] = None
        self._session_start_px4_timestamp_us: Optional[int] = None
        self._px4_failsafe_active = False
        self._latest_sync_stamp_ns: Optional[int] = None
        self._latest_sync_receipt_ns: Optional[int] = None
        self._accepted_sync_stamps: Deque[int] = deque()
        self._last_accepted_source_stamp_ns: Optional[int] = None
        self._control_mode: Optional[str] = None
        self._setpoint: Optional[RateCommand] = None
        self._fatal_error: Optional[str] = None
        self._last_status: Optional[str] = None
        self._last_warning_ns = 0
        self._was_publishing = False

        self._publish_status(
            'WAITING_FOR_PX4_CONTROL_MODE' if self._require_px4_mode else 'WAITING_FOR_PX4_CLOCK'
        )
        self.get_logger().info(
            f'PX4 rate control listens on {self._setpoint_topic}, requires '
            f'image_sync={self._image_sync_topic if self._require_image_sync else "false"}, and publishes '
            f'native PX4 setpoints to {self._px4_rates_topic}. It binds AirSim vehicle '
            f'{self._airsim_vehicle_name} to PX4 system {self._expected_px4_system_id} on '
            f'{self._px4_topic_prefix}, and never arms or changes flight mode.'
        )

    @property
    def fatal_error(self) -> Optional[str]:
        return self._fatal_error

    def _publish_status(self, status: str) -> None:
        if status == self._last_status:
            return
        self._last_status = status
        self._status_pub.publish(String(data=status))
        self.get_logger().info(f'PX4 rate control: {status}')

    def _fatal(self, message: str) -> None:
        if self._fatal_error is None:
            self._fatal_error = message
            self._stop_publishing()
            self._publish_status('ERROR: ' + message)
            self.get_logger().error(message)

    def _stop_publishing(self) -> None:
        self._was_publishing = False

    def _invalidate_setpoint(self, reason: str, message: str) -> None:
        self._setpoint = None
        self._stop_publishing()
        self._publish_status('INVALID_SETPOINT_' + reason.upper())
        self._warn(message)

    def _fence_camera_authorized_control(self, status: str, reason: str) -> None:
        """Require a fresh synchronized frame and command after a PX4 safety event."""

        self._setpoint = None
        self._last_accepted_source_stamp_ns = None
        self._latest_sync_stamp_ns = None
        self._latest_sync_receipt_ns = None
        self._accepted_sync_stamps.clear()
        self._stop_publishing()
        self._publish_status(status)
        self._warn(reason)

    def _control_mode_cb(self, message: String) -> None:
        self._control_mode = message.data.strip()
        if self._require_px4_mode and self._control_mode.lower() != 'px4':
            self._fatal(
                f"AirSim reports ROS 2 control mode '{self._control_mode}', but PX4 mode is required. "
                "Set Ros2.ControlMode to 'PX4' and restart both nodes."
            )

    def _vehicle_status_cb(self, message: VehicleStatus) -> None:
        system_id = int(getattr(message, 'system_id', 0))
        if system_id != self._expected_px4_system_id:
            self._fatal(
                f'PX4 topic {self._px4_vehicle_status_topic} reports system_id={system_id}, '
                f'but AirSim vehicle {self._airsim_vehicle_name} is bound to '
                f'system_id={self._expected_px4_system_id}'
            )
            return
        timestamp_us = int(getattr(message, 'timestamp', 0))
        if (
            self._session_start_px4_timestamp_us is not None
            and timestamp_us < self._session_start_px4_timestamp_us
        ):
            self._warn(
                f'Ignoring stale VehicleStatus timestamp {timestamp_us} us before PX4 session '
                f'start {self._session_start_px4_timestamp_us} us'
            )
            return
        self._validated_px4_system_id = system_id
        self._last_vehicle_status_receipt_ns = time.monotonic_ns()
        failsafe_active = bool(getattr(message, 'failsafe', False)) or bool(
            getattr(message, 'failsafe_and_user_took_over', False)
        )
        if failsafe_active:
            self._px4_failsafe_active = True
            self._fence_camera_authorized_control(
                'PX4_FAILSAFE',
                'PX4 reports failsafe or user takeover; stopped camera-authorized rate output',
            )
            return
        if self._px4_failsafe_active:
            self._px4_failsafe_active = False
            self._fence_camera_authorized_control(
                'PX4_FAILSAFE_RECOVERED',
                'PX4 failsafe recovered; waiting for a new synchronized frame and command',
            )

    def _timesync_status_cb(self, message: TimesyncStatus) -> None:
        """Reject the uXRCE-DDS translation mode before sending setpoints."""

        dds_source = int(getattr(message, 'SOURCE_PROTOCOL_DDS', 2))
        if int(getattr(message, 'source_protocol', 0)) != dds_source:
            return
        observed_offset_us = int(getattr(message, 'observed_offset', 0))
        estimated_offset_us = int(getattr(message, 'estimated_offset', 0))
        self._fatal(
            'PX4 uXRCE-DDS timestamp synchronization is active '
            f'(observed_offset={observed_offset_us} us, '
            f'estimated_offset={estimated_offset_us} us). Set UXRCE_DDS_SYNCT=0 '
            'and restart PX4 or the uXRCE-DDS client before using exact '
            'AirSim/PX4 control.'
        )

    def _dds_timesync_guard_ready(self, now_ns: int) -> bool:
        """Observe the independent DDS time-sync guard before live control.

        A DDS time-sync status is evidence that UXRCE_DDS_SYNCT is enabled,
        even if its current estimated offset is zero.
        """

        if self._dds_timesync_guard_verified:
            return True
        if self._dds_timesync_guard_started_ns is None:
            return False
        if now_ns - self._dds_timesync_guard_started_ns < self._px4_dds_timesync_guard_ns:
            return False
        self._dds_timesync_guard_verified = True
        self.get_logger().info(
            'No PX4 DDS time-translation sample was observed during the configured guard'
        )
        return True

    def _begin_px4_session(self, status: str, reason: str, epoch: Optional[int] = None) -> None:
        self._px4_clock_epoch = self._px4_clock_epoch + 1 if epoch is None else epoch
        self._dds_timesync_guard_started_ns = None
        self._dds_timesync_guard_verified = False
        self._setpoint = None
        self._last_accepted_source_stamp_ns = None
        self._latest_sync_stamp_ns = None
        self._latest_sync_receipt_ns = None
        self._accepted_sync_stamps.clear()
        self._validated_px4_system_id = None
        self._last_vehicle_status_receipt_ns = None
        self._session_start_px4_timestamp_us = None
        self._px4_failsafe_active = False
        self._stop_publishing()
        self._publish_status(status)
        self._warn(reason)

    def _px4_clock_cb(self, message: SensorCombined) -> None:
        timestamp_us = int(getattr(message, 'timestamp', 0))
        receipt_ns = time.monotonic_ns()
        if timestamp_us <= 0:
            self._warn('Ignoring PX4 clock sample without a positive timestamp')
            return

        new_session = False
        if receipt_gap_exceeds(
            self._last_px4_clock_receipt_ns, receipt_ns, self._px4_session_gap_ns
        ):
            self._begin_px4_session(
                'PX4_SESSION_GAP',
                f'PX4 SensorCombined receipt gap exceeded {self._px4_session_gap_ns} ns; '
                'cleared command and synchronization state',
            )
            new_session = True
        else:
            progress = timestamp_progress(self._latest_px4_timestamp_us, timestamp_us)
            if progress == 'duplicate':
                return
            if progress == 'regressed':
                self._begin_px4_session(
                    'PX4_CLOCK_RESET',
                    'PX4 HIL clock regressed; cleared command and synchronization state',
                )
                new_session = True
        self._latest_px4_timestamp_us = timestamp_us
        self._last_px4_clock_receipt_ns = receipt_ns
        if self._dds_timesync_guard_started_ns is None:
            self._dds_timesync_guard_started_ns = receipt_ns
        if new_session:
            self._session_start_px4_timestamp_us = timestamp_us

    def _image_sync_cb(self, message: Px4ImageSync) -> None:
        if not self._dds_timesync_guard_ready(time.monotonic_ns()):
            return
        if message.image_topic.rstrip('/') != self._expected_sync_image_topic:
            self._fatal(
                'Image synchronization camera does not match this rate controller: '
                f'got {message.image_topic!r}, expected {self._expected_sync_image_topic!r}'
            )
            return
        if not bool(message.direct_hil_clock_verified):
            self._warn('Ignoring image synchronization event without direct HIL clock proof')
            return
        if (
            int(message.direct_hil_clock_match_count) < _MIN_DIRECT_HIL_CLOCK_MATCHES
            or int(message.direct_hil_clock_last_matched_timestamp_us) <= 0
        ):
            self._warn('Ignoring image synchronization event with incomplete direct HIL proof')
            return
        stamp_ns = _stamp_ns(message.image_header)
        if (
            message.airsim_vehicle_name != self._airsim_vehicle_name
            or message.px4_topic_prefix.rstrip('/') != self._px4_topic_prefix
            or int(message.px4_system_id) != self._expected_px4_system_id
        ):
            self._fatal(
                'Image synchronization vehicle binding does not match this rate controller: '
                f'got AirSim={message.airsim_vehicle_name}, PX4 prefix={message.px4_topic_prefix}, '
                f'system_id={message.px4_system_id}'
            )
            return
        sync_epoch = int(message.px4_clock_epoch)
        if sync_epoch < self._px4_clock_epoch:
            self._warn(
                f'Ignoring stale image synchronization epoch {sync_epoch}; '
                f'current PX4 clock epoch is {self._px4_clock_epoch}'
            )
            return
        if sync_epoch > self._px4_clock_epoch:
            self._begin_px4_session(
                'IMAGE_SYNC_SESSION_FENCE',
                f'Adopted newer camera synchronization epoch {sync_epoch}; '
                'cleared command and synchronization state',
                sync_epoch,
            )
            return
        if self._latest_px4_timestamp_us is None:
            return
        sync_clock_ns = int(message.px4_clock_timestamp_us) * 1_000
        current_px4_ns = self._latest_px4_timestamp_us * 1_000
        if sync_clock_ns <= 0 or stamp_ns > sync_clock_ns:
            self._fatal(
                'Image synchronization event has an invalid PX4 clock/image timestamp relationship'
            )
            return
        if sync_clock_ns > current_px4_ns + self._future_tolerance_ns:
            self._warn('Ignoring image synchronization event ahead of the current PX4 clock')
            return
        progress = timestamp_progress(self._latest_sync_stamp_ns, stamp_ns)
        if progress == 'invalid':
            self._warn('Ignoring image synchronization event without a positive image timestamp')
            return
        if progress == 'duplicate':
            return
        if progress == 'regressed':
            self._setpoint = None
            self._last_accepted_source_stamp_ns = None
            self._accepted_sync_stamps.clear()
            self._stop_publishing()
            self._publish_status('IMAGE_SYNC_RESET')
            self._warn('Image synchronization timestamp regressed; cleared active command')
        self._latest_sync_stamp_ns = stamp_ns
        self._latest_sync_receipt_ns = time.monotonic_ns()
        self._accepted_sync_stamps.append(stamp_ns)
        earliest_valid_stamp_ns = stamp_ns - max(self._source_timeout_ns, self._sync_timeout_ns)
        while (
            self._accepted_sync_stamps
            and self._accepted_sync_stamps[0] < earliest_valid_stamp_ns
        ):
            self._accepted_sync_stamps.popleft()

    def _setpoint_cb(self, message: Px4RateSetpoint) -> None:
        receipt_ns = time.monotonic_ns()
        source_stamp_ns = _stamp_ns(message.header)
        latest_px4_ns = (
            self._latest_px4_timestamp_us * 1_000
            if self._latest_px4_timestamp_us is not None
            else None
        )
        try:
            roll_rate, pitch_rate, yaw_rate, thrust = validate_rate_values(
                message.roll_rate,
                message.pitch_rate,
                message.yaw_rate,
                message.thrust,
                self._max_roll_rate,
                self._max_pitch_rate,
                self._max_yaw_rate,
            )
            validate_source_timestamp(
                source_stamp_ns,
                self._last_accepted_source_stamp_ns,
                latest_px4_ns,
                self._future_tolerance_ns,
            )
        except ValueError as error:
            reason = str(error)
            self._invalidate_setpoint(reason, f'Rejecting PX4 rate setpoint: {reason}')
            return

        self._setpoint = RateCommand(
            roll_rate_flu=roll_rate,
            pitch_rate_flu=pitch_rate,
            yaw_rate_flu=yaw_rate,
            thrust_up=thrust,
            source_time_ns=source_stamp_ns,
            receipt_ns=receipt_ns,
        )
        self._last_accepted_source_stamp_ns = source_stamp_ns

    def _base_ready(self, now_ns: int) -> Optional[str]:
        if self._fatal_error is not None:
            return 'ERROR'
        if self._require_px4_mode and (
            self._control_mode is None or self._control_mode.lower() != 'px4'
        ):
            return 'WAITING_FOR_PX4_CONTROL_MODE'
        if self._validated_px4_system_id is None:
            return 'WAITING_FOR_PX4_VEHICLE_ID'
        if self._px4_failsafe_active:
            return 'PX4_FAILSAFE'
        if (
            self._last_vehicle_status_receipt_ns is None
            or now_ns - self._last_vehicle_status_receipt_ns > self._vehicle_status_max_age_ns
        ):
            return 'STALE_PX4_VEHICLE_STATUS'
        if self._last_px4_clock_receipt_ns is None or self._latest_px4_timestamp_us is None:
            return 'WAITING_FOR_PX4_CLOCK'
        if now_ns - self._last_px4_clock_receipt_ns > self._clock_max_age_ns:
            return 'STALE_PX4_CLOCK'
        if not self._dds_timesync_guard_ready(now_ns):
            return 'WAITING_FOR_PX4_DDS_TIMESYNC_GUARD'
        return None

    def _publish_timer_cb(self) -> None:
        now_ns = time.monotonic_ns()
        base_status = self._base_ready(now_ns)
        if base_status is not None:
            self._stop_publishing()
            self._publish_status(base_status)
            return

        ready, reason = command_readiness(
            command=self._setpoint,
            now_receipt_ns=now_ns,
            latest_px4_time_ns=self._latest_px4_timestamp_us * 1_000,
            latest_sync_time_ns=self._latest_sync_stamp_ns,
            latest_sync_receipt_ns=self._latest_sync_receipt_ns,
            accepted_sync_stamps=self._accepted_sync_stamps,
            receipt_timeout_ns=self._setpoint_timeout_ns,
            source_timeout_ns=self._source_timeout_ns,
            max_future_ns=self._future_tolerance_ns,
            sync_timeout_ns=self._sync_timeout_ns,
            require_image_sync=self._require_image_sync,
        )
        if not ready:
            if self._was_publishing:
                self.get_logger().warning(
                    f'PX4 rate output stopped: {reason}; PX4 can apply its configured offboard failsafe'
                )
            self._stop_publishing()
            self._publish_status(reason.upper())
            return

        if self._setpoint is None or self._latest_px4_timestamp_us is None:
            self._stop_publishing()
            return

        timestamp_us = self._latest_px4_timestamp_us
        offboard_mode = OffboardControlMode()
        offboard_mode.timestamp = timestamp_us
        _configure_body_rate_mode(offboard_mode)

        rates = VehicleRatesSetpoint()
        rates.timestamp = timestamp_us
        rates.roll = self._setpoint.roll_rate_flu
        # PX4 uses FRD axes; ROS FLU pitch and yaw invert.
        rates.pitch = -self._setpoint.pitch_rate_flu
        rates.yaw = -self._setpoint.yaw_rate_flu
        # Upward collective is negative body-Z in PX4 FRD.
        rates.thrust_body = [0.0, 0.0, -self._setpoint.thrust_up]
        rates.reset_integral = False

        self._offboard_mode_pub.publish(offboard_mode)
        self._rates_pub.publish(rates)
        self._was_publishing = True
        self._publish_status('SETPOINT_ACTIVE')

    def _health_cb(self) -> None:
        if self._fatal_error is not None:
            return
        now_ns = time.monotonic_ns()
        if self._require_px4_mode and self._control_mode is None:
            if now_ns - self._started_ns > self._control_mode_timeout_sec * 1_000_000_000:
                self._fatal(f'Timed out waiting for {self._control_mode_topic}=PX4')
            else:
                self._publish_status('WAITING_FOR_PX4_CONTROL_MODE')
            return
        if self._validated_px4_system_id is None:
            if now_ns - self._started_ns > self._vehicle_status_timeout_sec * 1_000_000_000:
                self._fatal(
                    f'Timed out waiting for PX4 system_id={self._expected_px4_system_id} '
                    f'on {self._px4_vehicle_status_topic}'
                )
            else:
                self._publish_status('WAITING_FOR_PX4_VEHICLE_ID')
            return
        if self._last_px4_clock_receipt_ns is None:
            if now_ns - self._started_ns > self._px4_clock_timeout_sec * 1_000_000_000:
                self._fatal(f'Timed out waiting for an advancing PX4 HIL clock on {self._px4_clock_topic}')
            else:
                self._publish_status('WAITING_FOR_PX4_CLOCK')
            return
        base_status = self._base_ready(now_ns)
        if base_status is not None:
            self._stop_publishing()
            self._publish_status(base_status)

    def _warn(self, message: str) -> None:
        now_ns = time.monotonic_ns()
        if now_ns - self._last_warning_ns >= 1_000_000_000:
            self._last_warning_ns = now_ns
            self.get_logger().warning(message)


def main(args=None) -> int:
    rclpy.init(args=args)
    node: Optional[Px4RateControl] = None
    exit_code = 0
    try:
        node = Px4RateControl()
        while rclpy.ok() and node.fatal_error is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.fatal_error is not None:
            exit_code = 1
    except (RuntimeError, ValueError) as error:
        print(f'px4_rate_control startup failed: {error}')
        exit_code = 1
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':  # pragma: no cover
    raise SystemExit(main())
