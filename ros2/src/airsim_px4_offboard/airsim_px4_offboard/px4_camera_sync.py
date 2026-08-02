"""Associate exact AirSim camera stamps with PX4 HIL state and sensors.

The AirSim image payload is never subscribed, republished, or modified.  Its
paired ``sensor_msgs/CameraInfo`` carries the same rendered-frame header, so
the synchronizer retains only that small header while native PX4 samples
bracket the render timestamp.  PX4 HIL timestamps and AirSim camera timestamps
share the same ``ClockFactory`` epoch, so this node never applies a
wall-clock/DDS offset and never pauses physics.
"""

from __future__ import annotations

from bisect import bisect_left
from collections import deque
from dataclasses import dataclass
import math
import time
from typing import Deque, List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from airsim_interfaces.msg import Px4HilSensorClock, Px4ImageSync
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Header, String

try:  # Optional at ordinary AirSim build time; required when this node runs.
    from px4_msgs.msg import (
        SensorCombined,
        SensorGps,
        TimesyncStatus,
        VehicleAttitude,
        VehicleOdometry,
        VehicleStatus,
    )
    _PX4_IMPORT_ERROR: Optional[Exception] = None
except ImportError as error:  # pragma: no cover - depends on the target ROS overlay
    SensorCombined = None  # type: ignore[assignment,misc]
    SensorGps = None  # type: ignore[assignment,misc]
    TimesyncStatus = None  # type: ignore[assignment,misc]
    VehicleAttitude = None  # type: ignore[assignment,misc]
    VehicleOdometry = None  # type: ignore[assignment,misc]
    VehicleStatus = None  # type: ignore[assignment,misc]
    _PX4_IMPORT_ERROR = error

from .control_safety import (
    DEFAULT_PX4_SESSION_GAP_SEC,
    receipt_gap_exceeds,
    seconds_to_positive_nanoseconds,
    timestamp_progress,
)
from .px4_schema import validate_px4_message_contract

from .sync_math import (
    FRD_TO_FLU,
    NED_TO_ENU,
    diagonal_covariance,
    frd_to_flu,
    latest_at_or_before_index,
    lerp_vector,
    ned_to_enu,
    ned_velocity_covariance_to_flu_body,
    ned_velocity_to_flu_body,
    px4_attitude_ned_frd_to_enu_flu,
    px4_us_to_airsim_ns,
    quaternion_slerp,
    transform_covariance,
)
from .launch_helpers import camera_info_topic


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


# Keep this equal to the bounded AirSim MAVLink history exposed by
# simGetHilSensorTimeHistory. It prevents an unbounded backlog when PX4 runs
# faster than the ROS executor that verifies the source clock.
_HIL_SENSOR_HISTORY_CAPACITY = 2048
_MIN_DIRECT_HIL_CLOCK_MATCHES = 3
_PAIR_HISTORY_NAMES = ('odometry', 'attitude', 'gyro', 'acceleration', 'gps')


@dataclass(frozen=True)
class OdomSample:
    remote_time_ns: int
    px4_timestamp_us: int
    reset_counter: int
    position_ned: Vector3
    quaternion_ned_frd: Quaternion
    velocity_ned: Vector3
    position_variance_ned: Optional[Vector3]
    orientation_variance_frd: Optional[Vector3]
    velocity_variance_ned: Optional[Vector3]


@dataclass(frozen=True)
class AttitudeSample:
    remote_time_ns: int
    px4_timestamp_us: int
    reset_counter: int
    quaternion_ned_frd: Quaternion


@dataclass(frozen=True)
class ImuVectorSample:
    remote_time_ns: int
    px4_timestamp_us: int
    calibration_count: int
    clipping: int
    vector_frd: Vector3


@dataclass(frozen=True)
class GpsSample:
    remote_time_ns: int
    px4_timestamp_us: int
    latitude_deg: float
    longitude_deg: float
    altitude_msl_m: float
    horizontal_accuracy_m: float
    vertical_accuracy_m: float
    fix_type: int


@dataclass(frozen=True)
class Bracket:
    before: object
    after: object
    ratio: float
    span_ns: int


@dataclass
class PendingImage:
    header: Header
    stamp_ns: int
    receipt_ns: int
    retry_history: Optional[str] = None
    retry_history_version: int = 0
    last_failure_reason: Optional[str] = None


@dataclass(frozen=True)
class SynchronizedPair:
    image: PendingImage
    position_ned: Vector3
    velocity_ned: Vector3
    gyro_frd: Vector3
    acceleration_frd: Vector3
    odometry_quaternion_ned_frd: Quaternion
    attitude_quaternion_ned_frd: Quaternion
    position_variance_ned: Optional[Vector3]
    orientation_variance_frd: Optional[Vector3]
    velocity_variance_ned: Optional[Vector3]
    odometry_bracket: Bracket
    attitude_bracket: Bracket
    gyro_bracket: Bracket
    acceleration_bracket: Bracket
    gps_sample: Optional[GpsSample]
    gps_age_ns: int
    gps_fresh: bool


def _parameter(node: Node, name: str, default):
    node.declare_parameter(name, default)
    return node.get_parameter(name).value


def _join_topic(prefix: str, suffix: str) -> str:
    cleaned_prefix = prefix.rstrip('/')
    cleaned_suffix = suffix.strip('/')
    if not cleaned_prefix:
        raise ValueError('topic prefix must not be empty')
    return f'{cleaned_prefix}/{cleaned_suffix}'


def _header_stamp_ns(header: Header) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def _source_timestamp_us(message) -> int:
    sample_timestamp = int(getattr(message, 'timestamp_sample', 0))
    if sample_timestamp > 0:
        return sample_timestamp
    return int(getattr(message, 'timestamp', 0))


def _finite_vector(values: Sequence[float], length: int) -> Optional[Tuple[float, ...]]:
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError):
        return None
    if len(result) != length or not all(math.isfinite(value) for value in result):
        return None
    return result


def _finite_variance(values: Sequence[float]) -> Optional[Vector3]:
    vector = _finite_vector(values, 3)
    if vector is None or any(value < 0.0 for value in vector):
        return None
    return vector  # type: ignore[return-value]


def _copy_header(source: Header) -> Header:
    header = Header()
    header.stamp.sec = source.stamp.sec
    header.stamp.nanosec = source.stamp.nanosec
    header.frame_id = source.frame_id
    return header


def _set_stamp_ns(header: Header, timestamp_ns: int) -> None:
    header.stamp.sec = timestamp_ns // 1_000_000_000
    header.stamp.nanosec = timestamp_ns % 1_000_000_000


def _lerp_optional_vector(
    first: Optional[Vector3], second: Optional[Vector3], ratio: float
) -> Optional[Vector3]:
    if first is None or second is None:
        return None
    return lerp_vector(first, second, ratio)


def _write_covariance_block(destination, block, row_offset: int, column_offset: int, stride: int) -> None:
    for row in range(3):
        for column in range(3):
            destination[(row + row_offset) * stride + column + column_offset] = block[row][column]


def _validate_px4_message_contract(include_gps: bool) -> None:
    messages = [
        VehicleOdometry(),
        VehicleAttitude(),
        SensorCombined(),
        TimesyncStatus(),
        VehicleStatus(),
    ]
    if include_gps:
        messages.append(SensorGps())
    validate_px4_message_contract(*messages)


class Px4CameraSync(Node):
    """Timestamp-preserving AirSim camera/PX4 HIL synchronizer."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__('px4_camera_sync', parameter_overrides=parameter_overrides)
        if _PX4_IMPORT_ERROR is not None:
            raise RuntimeError(
                'PX4 mode requires px4_msgs generated from the connected PX4 source revision. '
                f'Import failed: {_PX4_IMPORT_ERROR}'
            )
        self._camera_topic = str(_parameter(self, 'camera_topic', ''))
        self._camera_info_topic = str(_parameter(self, 'camera_info_topic', ''))
        self._output_prefix = str(_parameter(self, 'output_prefix', 'camera_sync'))
        self._airsim_vehicle_name = str(
            _parameter(self, 'airsim_vehicle_name', 'drone1')
        ).strip('/')
        self._px4_topic_prefix = str(_parameter(self, 'px4_topic_prefix', '/fmu')).rstrip('/')
        self._expected_px4_system_id = int(_parameter(self, 'expected_px4_system_id', 1))
        self._control_mode_topic = str(
            _parameter(self, 'control_mode_topic', '/airsim_node/control_mode')
        )
        self._require_px4_mode = bool(_parameter(self, 'require_px4_control_mode', True))
        self._control_mode_timeout_sec = float(_parameter(self, 'control_mode_timeout_sec', 10.0))
        self._px4_clock_timeout_sec = float(_parameter(self, 'px4_clock_timeout_sec', 10.0))
        self._px4_clock_max_age_ns = int(
            float(_parameter(self, 'px4_clock_max_age_sec', 0.5)) * 1_000_000_000.0
        )

        self._px4_odometry_topic = str(
            _parameter(self, 'px4_odometry_topic', '/fmu/out/vehicle_odometry')
        )
        self._px4_attitude_topic = str(
            _parameter(self, 'px4_attitude_topic', '/fmu/out/vehicle_attitude')
        )
        self._px4_sensor_combined_topic = str(
            _parameter(self, 'px4_sensor_combined_topic', '/fmu/out/sensor_combined')
        )
        self._px4_gps_topic = str(
            _parameter(self, 'px4_gps_topic', '/fmu/out/vehicle_gps_position')
        )
        self._px4_vehicle_status_topic = str(
            _parameter(self, 'px4_vehicle_status_topic', '/fmu/out/vehicle_status_v1')
        )
        self._px4_timesync_status_topic = str(
            _parameter(self, 'px4_timesync_status_topic', '/fmu/out/timesync_status')
        )
        self._hil_clock_topic = str(_parameter(self, 'hil_clock_topic', '')).strip()

        self._state_history_sec = float(_parameter(self, 'state_history_sec', 15.0))
        self._max_state_gap_ns = int(
            float(_parameter(self, 'max_state_interpolation_gap_sec', 0.05)) * 1_000_000_000.0
        )
        self._max_imu_gap_ns = int(
            float(_parameter(self, 'max_imu_interpolation_gap_sec', 0.02)) * 1_000_000_000.0
        )
        self._max_image_wait_ns = int(
            float(_parameter(self, 'max_image_wait_sec', 0.1)) * 1_000_000_000.0
        )
        self._sync_health_timeout_ns = int(
            float(_parameter(self, 'sync_health_timeout_sec', 0.5)) * 1_000_000_000.0
        )
        self._max_gps_age_ns = int(
            float(_parameter(self, 'max_gps_age_sec', 1.0)) * 1_000_000_000.0
        )
        self._vehicle_status_timeout_sec = float(
            _parameter(self, 'vehicle_status_timeout_sec', 10.0)
        )
        self._vehicle_status_max_age_ns = int(
            float(_parameter(self, 'vehicle_status_max_age_sec', 3.0)) * 1_000_000_000.0
        )
        self._max_px4_stream_lead_us = int(
            float(_parameter(self, 'max_px4_stream_lead_sec', 0.05)) * 1_000_000.0
        )
        px4_session_gap_sec = float(
            _parameter(self, 'px4_session_gap_sec', DEFAULT_PX4_SESSION_GAP_SEC)
        )
        self._px4_session_gap_ns = seconds_to_positive_nanoseconds(
            px4_session_gap_sec, 'px4_session_gap_sec'
        )
        self._px4_dds_timesync_guard_ns = int(
            float(_parameter(self, 'px4_dds_timesync_guard_sec', 3.0)) * 1_000_000_000.0
        )
        self._direct_hil_clock_proof_timeout_ns = int(
            float(_parameter(self, 'direct_hil_clock_proof_timeout_sec', 10.0)) *
            1_000_000_000.0
        )
        self._hil_clock_max_age_ns = int(
            float(_parameter(self, 'hil_clock_max_age_sec', 0.5)) * 1_000_000_000.0
        )
        self._direct_hil_clock_matches_required = int(
            _parameter(
                self,
                'direct_hil_clock_matches_required',
                _MIN_DIRECT_HIL_CLOCK_MATCHES,
            )
        )
        self._max_pending_images = int(_parameter(self, 'max_pending_images', 5))
        self._publish_gps = bool(_parameter(self, 'publish_gps', True))
        self._require_gps = bool(_parameter(self, 'require_gps', False))
        self._world_frame_id = str(_parameter(self, 'world_frame_id', 'world_enu'))
        self._body_frame_id = str(_parameter(self, 'body_frame_id', 'base_link_flu'))
        self._gps_frame_id = str(_parameter(self, 'gps_frame_id', 'gps_link'))

        if not self._camera_topic:
            raise ValueError('camera_topic must be supplied')
        if not self._camera_info_topic:
            self._camera_info_topic = camera_info_topic(self._camera_topic)
        if not self._output_prefix.strip('/'):
            raise ValueError('output_prefix must not be empty')
        if not self._airsim_vehicle_name or not self._px4_topic_prefix:
            raise ValueError('airsim_vehicle_name and px4_topic_prefix must not be empty')
        if not self._hil_clock_topic:
            self._hil_clock_topic = (
                f'/airsim_node/{self._airsim_vehicle_name}/px4/hil_sensor_clock'
            )
        if self._expected_px4_system_id < 1 or self._expected_px4_system_id > 255:
            raise ValueError('expected_px4_system_id must be in [1, 255]')
        if not self._px4_vehicle_status_topic:
            raise ValueError('px4_vehicle_status_topic must not be empty')
        if not self._px4_timesync_status_topic:
            raise ValueError('px4_timesync_status_topic must not be empty')
        positive_values = (
            self._control_mode_timeout_sec,
            self._px4_clock_timeout_sec,
            self._px4_clock_max_age_ns,
            self._state_history_sec,
            self._max_state_gap_ns,
            self._max_imu_gap_ns,
            self._max_image_wait_ns,
            self._sync_health_timeout_ns,
            self._max_gps_age_ns,
            self._vehicle_status_timeout_sec,
            self._vehicle_status_max_age_ns,
            self._max_px4_stream_lead_us,
            self._px4_session_gap_ns,
            self._px4_dds_timesync_guard_ns,
            self._direct_hil_clock_proof_timeout_ns,
            self._hil_clock_max_age_ns,
        )
        if not all(math.isfinite(float(value)) and value > 0 for value in positive_values):
            raise ValueError('PX4 synchronization timing parameters must be finite and positive')
        if self._max_pending_images < 1:
            raise ValueError('max_pending_images must be at least one')
        if self._direct_hil_clock_matches_required < _MIN_DIRECT_HIL_CLOCK_MATCHES:
            raise ValueError(
                'direct_hil_clock_matches_required must be at least '
                f'{_MIN_DIRECT_HIL_CLOCK_MATCHES}'
            )
        if self._require_gps and not self._px4_gps_topic:
            raise ValueError('require_gps needs a non-empty px4_gps_topic')

        # GPS is optional.  Do not reject an otherwise compatible PX4 bridge
        # for a SensorGps schema that this configured pipeline never uses.
        _validate_px4_message_contract(self._publish_gps or self._require_gps)

        self._state_history_ns = int(self._state_history_sec * 1_000_000_000.0)
        self._mode_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._hil_clock_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._status_pub = self.create_publisher(
            String, _join_topic(self._output_prefix, 'status'), self._mode_qos
        )
        self._odom_pub = self.create_publisher(
            Odometry, _join_topic(self._output_prefix, 'odometry_at_image'), qos_profile_sensor_data
        )
        self._imu_pub = self.create_publisher(
            Imu, _join_topic(self._output_prefix, 'imu_at_image'), qos_profile_sensor_data
        )
        self._gps_pub = self.create_publisher(
            NavSatFix, _join_topic(self._output_prefix, 'gps_at_or_before_image'), qos_profile_sensor_data
        )
        self._sync_pub = self.create_publisher(
            Px4ImageSync, _join_topic(self._output_prefix, 'image_sync'), qos_profile_sensor_data
        )

        self._mode_sub = self.create_subscription(
            String, self._control_mode_topic, self._control_mode_cb, self._mode_qos
        )
        self._camera_header_sub = self.create_subscription(
            CameraInfo, self._camera_info_topic, self._camera_header_cb, qos_profile_sensor_data
        )
        self._odom_sub = self.create_subscription(
            VehicleOdometry, self._px4_odometry_topic, self._odometry_cb, qos_profile_sensor_data
        )
        self._attitude_sub = self.create_subscription(
            VehicleAttitude, self._px4_attitude_topic, self._attitude_cb, qos_profile_sensor_data
        )
        self._sensor_combined_sub = self.create_subscription(
            SensorCombined,
            self._px4_sensor_combined_topic,
            self._sensor_combined_cb,
            qos_profile_sensor_data,
        )
        self._hil_clock_sub = self.create_subscription(
            Px4HilSensorClock,
            self._hil_clock_topic,
            self._hil_clock_cb,
            self._hil_clock_qos,
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
        self._gps_sub = None
        if self._px4_gps_topic and (self._publish_gps or self._require_gps):
            self._gps_sub = self.create_subscription(
                SensorGps, self._px4_gps_topic, self._gps_cb, qos_profile_sensor_data
            )
        self._health_timer = self.create_timer(0.1, self._health_cb)

        self._started_ns = time.monotonic_ns()
        self._control_mode: Optional[str] = None
        self._fatal_error: Optional[str] = None
        self._last_status: Optional[str] = None
        self._last_warning_ns = 0
        self._last_received_image_stamp_ns: Optional[int] = None
        self._last_pair_receipt_ns: Optional[int] = None
        self._last_vehicle_status_receipt_ns: Optional[int] = None
        self._validated_px4_system_id: Optional[int] = None
        self._latest_px4_timestamp_us: Optional[int] = None
        self._last_px4_clock_receipt_ns: Optional[int] = None
        self._session_start_px4_timestamp_us: Optional[int] = None
        self._px4_clock_epoch = 0
        self._direct_hil_clock_check_started_ns: Optional[int] = None
        self._direct_hil_clock_verified = False
        self._direct_hil_clock_match_count = 0
        self._last_matched_hil_timestamp_us: Optional[int] = None
        self._last_evaluated_px4_clock_timestamp_us: Optional[int] = None
        self._hil_sensor_timestamps_us: Tuple[int, ...] = ()
        # Membership is checked for every SensorCombined callback while the
        # HIL history itself is refreshed less often. Keep this sidecar so
        # clock proof does not rebuild a 2048-element set at IMU rate.
        self._hil_sensor_timestamp_set = frozenset()
        self._last_hil_clock_receipt_ns: Optional[int] = None
        self._last_hil_clock_timestamp_us: Optional[int] = None
        self._recent_px4_clock_timestamps_us: Deque[int] = deque(
            maxlen=_HIL_SENSOR_HISTORY_CAPACITY
        )
        self._dds_timesync_guard_started_ns: Optional[int] = None
        self._dds_timesync_guard_verified = False
        self._odom_samples: Deque[OdomSample] = deque()
        self._attitude_samples: Deque[AttitudeSample] = deque()
        self._gyro_samples: Deque[ImuVectorSample] = deque()
        self._acceleration_samples: Deque[ImuVectorSample] = deque()
        self._gps_samples: Deque[GpsSample] = deque()
        # Keep the timestamp indexes alongside their deques. Samples may
        # arrive out of order, so the indexes are maintained by
        # ``_insert_sample`` instead of being reconstructed for every frame.
        self._odom_sample_timestamps_ns: List[int] = []
        self._attitude_sample_timestamps_ns: List[int] = []
        self._gyro_sample_timestamps_ns: List[int] = []
        self._acceleration_sample_timestamps_ns: List[int] = []
        self._gps_sample_timestamps_ns: List[int] = []
        self._history_versions = {name: 0 for name in _PAIR_HISTORY_NAMES}
        self._pending_images: Deque[PendingImage] = deque()
        self._published_pairs = 0
        self._dropped_images = 0

        self._publish_status(
            'WAITING_FOR_PX4_CONTROL_MODE'
            if self._require_px4_mode
            else 'WAITING_FOR_CAMERA_AND_PX4_STATE'
        )
        self.get_logger().info(
            f'Synchronizing {self._camera_topic} from headers on {self._camera_info_topic} '
            f'in the AirSim HIL clock with '
            f'odometry={self._px4_odometry_topic}, attitude={self._px4_attitude_topic}, '
            f'imu={self._px4_sensor_combined_topic}, hil_clock={self._hil_clock_topic}, '
            f'gps={self._px4_gps_topic or "disabled"}; '
            f'binding AirSim vehicle {self._airsim_vehicle_name} to PX4 system '
            f'{self._expected_px4_system_id} on {self._px4_topic_prefix}'
        )

    @property
    def fatal_error(self) -> Optional[str]:
        return self._fatal_error

    def _publish_status(self, status: str) -> None:
        if status == self._last_status:
            return
        self._last_status = status
        self._status_pub.publish(String(data=status))
        self.get_logger().info(f'PX4 camera synchronizer: {status}')

    def _fatal(self, message: str) -> None:
        if self._fatal_error is None:
            self._fatal_error = message
            self._publish_status('ERROR: ' + message)
            self.get_logger().error(message)

    def _control_mode_cb(self, message: String) -> None:
        self._control_mode = message.data.strip()
        if self._require_px4_mode and self._control_mode.lower() != 'px4':
            self._fatal(
                f"AirSim reports ROS 2 control mode '{self._control_mode}', but PX4 mode is required. "
                "Set Ros2.ControlMode to 'PX4' and restart both nodes."
            )
            return
        self._process_pending_images()

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
        self._process_pending_images()

    def _timesync_status_cb(self, message: TimesyncStatus) -> None:
        """Reject the uXRCE-DDS translation mode from the direct HIL path."""

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
            'AirSim/PX4 synchronization.'
        )

    def _dds_timesync_guard_ready(self, now_ns: int) -> bool:
        """Observe the independent DDS translation guard.

        Absence of a DDS time-sync event is useful configuration evidence, but
        it is not proof that PX4 is consuming AirSim HIL time. That proof is
        established separately from exact HIL_SENSOR/SensorCombined matches.
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

    def _clear_direct_hil_clock_proof(self, reason: str) -> None:
        """Fence pairs after the direct HIL proof is lost.

        A new proof may be established in the same PX4 clock epoch, but no
        image queued under the old proof is allowed across the gap.
        """

        was_verified = self._direct_hil_clock_verified
        self._direct_hil_clock_verified = False
        self._direct_hil_clock_match_count = 0
        self._last_matched_hil_timestamp_us = None
        if was_verified:
            self._dropped_images += len(self._pending_images)
            self._pending_images.clear()
            self._direct_hil_clock_check_started_ns = time.monotonic_ns()
            self._warn('Cleared direct HIL clock proof: ' + reason)

    def _try_prove_direct_hil_clock(self) -> None:
        """Validate every PX4 timestamp covered by the current HIL history.

        A timestamp newer than the advertised history stays pending until the
        next history update. A timestamp inside that history but absent from it
        breaks the current proof streak immediately. This prevents a handful
        of old matches from authorizing later samples from another clock.
        """

        if not self._hil_sensor_timestamps_us:
            return
        sent_timestamps = self._hil_sensor_timestamp_set
        oldest_sent_timestamp_us = self._hil_sensor_timestamps_us[0]
        latest_sent_timestamp_us = self._hil_sensor_timestamps_us[-1]
        for timestamp_us in self._recent_px4_clock_timestamps_us:
            if (
                self._last_evaluated_px4_clock_timestamp_us is not None
                and timestamp_us <= self._last_evaluated_px4_clock_timestamp_us
            ):
                continue
            if timestamp_us > latest_sent_timestamp_us:
                # The ROS history has not reached this PX4 sample yet. Do not
                # treat ordinary cross-process delivery order as a mismatch.
                break
            self._last_evaluated_px4_clock_timestamp_us = timestamp_us
            if timestamp_us < oldest_sent_timestamp_us:
                # This sample has aged out of the bounded history and cannot
                # contribute to the current proof.
                continue
            if timestamp_us not in sent_timestamps:
                self._clear_direct_hil_clock_proof(
                    'PX4 SensorCombined timestamp '
                    f'{timestamp_us} us was absent from the covering AirSim HIL history'
                )
                continue
            self._last_matched_hil_timestamp_us = timestamp_us
            if self._direct_hil_clock_match_count < self._direct_hil_clock_matches_required:
                self._direct_hil_clock_match_count += 1
            if self._direct_hil_clock_match_count == self._direct_hil_clock_matches_required:
                if not self._direct_hil_clock_verified:
                    self._direct_hil_clock_verified = True
                    self.get_logger().info(
                        'Verified direct AirSim/PX4 HIL clock with '
                        f'{self._direct_hil_clock_match_count} exact '
                        'HIL_SENSOR/SensorCombined timestamp matches'
                    )

    def _hil_clock_cb(self, message: Px4HilSensorClock) -> None:
        if self._fatal_error is not None:
            return
        if message.airsim_vehicle_name != self._airsim_vehicle_name:
            self._fatal(
                f'HIL clock topic {self._hil_clock_topic} is bound to AirSim vehicle '
                f'{message.airsim_vehicle_name!r}, expected {self._airsim_vehicle_name!r}'
            )
            return

        raw_timestamps = tuple(int(value) for value in message.hil_sensor_timestamps_us)
        if not raw_timestamps:
            self._warn(f'Ignoring empty AirSim HIL clock history on {self._hil_clock_topic}')
            return
        if len(raw_timestamps) > _HIL_SENSOR_HISTORY_CAPACITY:
            self._fatal(
                'AirSim HIL clock history exceeds its bounded '
                f'{_HIL_SENSOR_HISTORY_CAPACITY}-sample contract'
            )
            return
        if any(timestamp_us <= 0 for timestamp_us in raw_timestamps) or any(
            later < earlier for earlier, later in zip(raw_timestamps, raw_timestamps[1:])
        ):
            self._fatal('AirSim HIL clock history is not positive and monotonically ordered')
            return

        timestamps = tuple(
            timestamp_us for index, timestamp_us in enumerate(raw_timestamps)
            if index == 0 or timestamp_us != raw_timestamps[index - 1]
        )
        latest_timestamp_us = timestamps[-1]
        if (
            self._last_hil_clock_timestamp_us is not None
            and latest_timestamp_us < self._last_hil_clock_timestamp_us
        ):
            self._begin_px4_session(
                'AIRSIM_HIL_CLOCK_RESET',
                'AirSim HIL sensor timestamp regressed; fenced PX4 synchronization state',
            )

        self._hil_sensor_timestamps_us = timestamps
        self._hil_sensor_timestamp_set = frozenset(timestamps)
        self._last_hil_clock_timestamp_us = latest_timestamp_us
        self._last_hil_clock_receipt_ns = time.monotonic_ns()
        self._try_prove_direct_hil_clock()
        self._process_pending_images()

    def _direct_hil_clock_ready(self, now_ns: int) -> bool:
        """Return true only for a fresh, runtime-proven direct HIL epoch."""

        if self._direct_hil_clock_check_started_ns is None:
            return False
        if self._last_px4_clock_receipt_ns is None or (
            now_ns - self._last_px4_clock_receipt_ns > self._px4_clock_max_age_ns
        ):
            self._clear_direct_hil_clock_proof('PX4 SensorCombined clock is stale')
            return False
        if not self._dds_timesync_guard_ready(now_ns):
            return False
        if self._last_hil_clock_receipt_ns is None or (
            now_ns - self._last_hil_clock_receipt_ns > self._hil_clock_max_age_ns
        ):
            self._clear_direct_hil_clock_proof('AirSim HIL history is stale')
            return False
        if (
            self._latest_px4_timestamp_us is not None
            and self._last_hil_clock_timestamp_us is not None
            and self._latest_px4_timestamp_us >
            self._last_hil_clock_timestamp_us + self._max_px4_stream_lead_us
        ):
            self._clear_direct_hil_clock_proof('PX4 clock advanced beyond the latest AirSim HIL history')
            return False
        if self._direct_hil_clock_verified:
            return True
        if now_ns - self._direct_hil_clock_check_started_ns >= self._direct_hil_clock_proof_timeout_ns:
            self._fatal(
                'Timed out proving the direct AirSim/PX4 HIL clock. Verify the '
                'PX4 simulator_mavlink target, primary HIL IMU, AirSim plugin, and '
                f'{self._hil_clock_topic}.'
            )
        return False

    def _source_time_is_current(self, timestamp_us: int, stream_name: str) -> bool:
        if self._latest_px4_timestamp_us is None:
            return False
        if (
            self._session_start_px4_timestamp_us is not None
            and timestamp_us < self._session_start_px4_timestamp_us
        ):
            # A PX4 restart can keep a monotonically increasing HIL clock. In
            # that case a late DDS state sample from the previous process is
            # not necessarily "ahead" of the new clock, so the upper-bound
            # check below cannot reject it. Never let it form an interpolation
            # bracket with samples from the new PX4 session.
            self._warn(
                f'Ignoring stale {stream_name} timestamp {timestamp_us} us before PX4 '
                f'session start {self._session_start_px4_timestamp_us} us'
            )
            return False
        if timestamp_us > self._latest_px4_timestamp_us + self._max_px4_stream_lead_us:
            self._warn(
                f'Ignoring {stream_name} timestamp {timestamp_us} us ahead of the current '
                f'PX4 clock epoch at {self._latest_px4_timestamp_us} us'
            )
            return False
        return True

    def _begin_px4_session(self, status: str, reason: str) -> None:
        self._px4_clock_epoch += 1
        self._direct_hil_clock_check_started_ns = None
        self._direct_hil_clock_verified = False
        self._direct_hil_clock_match_count = 0
        self._last_matched_hil_timestamp_us = None
        self._last_evaluated_px4_clock_timestamp_us = None
        self._hil_sensor_timestamps_us = ()
        self._hil_sensor_timestamp_set = frozenset()
        self._last_hil_clock_receipt_ns = None
        self._last_hil_clock_timestamp_us = None
        self._recent_px4_clock_timestamps_us.clear()
        self._dds_timesync_guard_started_ns = None
        self._dds_timesync_guard_verified = False
        self._dropped_images += len(self._pending_images)
        self._odom_samples.clear()
        self._attitude_samples.clear()
        self._gyro_samples.clear()
        self._acceleration_samples.clear()
        self._gps_samples.clear()
        self._odom_sample_timestamps_ns.clear()
        self._attitude_sample_timestamps_ns.clear()
        self._gyro_sample_timestamps_ns.clear()
        self._acceleration_sample_timestamps_ns.clear()
        self._gps_sample_timestamps_ns.clear()
        self._history_versions = {name: 0 for name in _PAIR_HISTORY_NAMES}
        self._pending_images.clear()
        self._last_received_image_stamp_ns = None
        self._last_pair_receipt_ns = None
        self._published_pairs = 0
        self._latest_px4_timestamp_us = None
        self._last_px4_clock_receipt_ns = None
        self._session_start_px4_timestamp_us = None
        self._last_vehicle_status_receipt_ns = None
        self._validated_px4_system_id = None
        self._publish_status(status)
        self._warn(f'{reason}; started synchronization epoch {self._px4_clock_epoch}')

    def _odometry_cb(self, message: VehicleOdometry) -> None:
        pose_frame = getattr(message, 'pose_frame', None)
        expected_pose_frame = getattr(message, 'POSE_FRAME_NED', 1)
        if pose_frame is not None and int(pose_frame) != int(expected_pose_frame):
            self._warn(f'Ignoring VehicleOdometry pose frame {pose_frame}; expected NED')
            return
        velocity_frame = getattr(message, 'velocity_frame', None)
        expected_velocity_frame = getattr(message, 'VELOCITY_FRAME_NED', 1)
        if velocity_frame is not None and int(velocity_frame) != int(expected_velocity_frame):
            self._warn(f'Ignoring VehicleOdometry velocity frame {velocity_frame}; expected NED')
            return

        timestamp_us = _source_timestamp_us(message)
        if not self._source_time_is_current(timestamp_us, 'VehicleOdometry'):
            return
        position = _finite_vector(getattr(message, 'position', ()), 3)
        quaternion = _finite_vector(getattr(message, 'q', ()), 4)
        velocity = _finite_vector(getattr(message, 'velocity', ()), 3)
        if timestamp_us <= 0 or position is None or quaternion is None or velocity is None:
            self._warn('Ignoring invalid VehicleOdometry state')
            return
        try:
            px4_attitude_ned_frd_to_enu_flu(quaternion)
        except ValueError:
            self._warn('Ignoring VehicleOdometry with an invalid quaternion')
            return

        sample = OdomSample(
            remote_time_ns=px4_us_to_airsim_ns(timestamp_us),
            px4_timestamp_us=timestamp_us,
            reset_counter=int(getattr(message, 'reset_counter', 0)),
            position_ned=position,  # type: ignore[arg-type]
            quaternion_ned_frd=quaternion,  # type: ignore[arg-type]
            velocity_ned=velocity,  # type: ignore[arg-type]
            position_variance_ned=_finite_variance(getattr(message, 'position_variance', ())),
            orientation_variance_frd=_finite_variance(getattr(message, 'orientation_variance', ())),
            velocity_variance_ned=_finite_variance(getattr(message, 'velocity_variance', ())),
        )
        self._insert_sample(
            self._odom_samples,
            self._odom_sample_timestamps_ns,
            sample,
            'odometry',
        )
        self._process_pending_images()

    def _attitude_cb(self, message: VehicleAttitude) -> None:
        timestamp_us = _source_timestamp_us(message)
        if not self._source_time_is_current(timestamp_us, 'VehicleAttitude'):
            return
        quaternion = _finite_vector(getattr(message, 'q', ()), 4)
        if timestamp_us <= 0 or quaternion is None:
            self._warn('Ignoring invalid VehicleAttitude state')
            return
        try:
            px4_attitude_ned_frd_to_enu_flu(quaternion)
        except ValueError:
            self._warn('Ignoring VehicleAttitude with an invalid quaternion')
            return

        sample = AttitudeSample(
            remote_time_ns=px4_us_to_airsim_ns(timestamp_us),
            px4_timestamp_us=timestamp_us,
            reset_counter=int(getattr(message, 'quat_reset_counter', 0)),
            quaternion_ned_frd=quaternion,  # type: ignore[arg-type]
        )
        self._insert_sample(
            self._attitude_samples,
            self._attitude_sample_timestamps_ns,
            sample,
            'attitude',
        )
        self._process_pending_images()

    def _sensor_combined_cb(self, message: SensorCombined) -> None:
        timestamp_us = int(getattr(message, 'timestamp', 0))
        receipt_ns = time.monotonic_ns()
        if timestamp_us <= 0:
            return
        new_session = False
        if receipt_gap_exceeds(
            self._last_px4_clock_receipt_ns, receipt_ns, self._px4_session_gap_ns
        ):
            self._begin_px4_session(
                'PX4_SESSION_GAP',
                f'PX4 SensorCombined receipt gap exceeded {self._px4_session_gap_ns} ns',
            )
            new_session = True
        else:
            progress = timestamp_progress(self._latest_px4_timestamp_us, timestamp_us)
            if progress == 'duplicate':
                return
            if progress == 'regressed':
                self._begin_px4_session(
                    'PX4_CLOCK_RESET',
                    'PX4 HIL clock regressed',
                )
                new_session = True
        self._latest_px4_timestamp_us = timestamp_us
        self._last_px4_clock_receipt_ns = receipt_ns
        if self._direct_hil_clock_check_started_ns is None:
            self._direct_hil_clock_check_started_ns = receipt_ns
        if self._dds_timesync_guard_started_ns is None:
            self._dds_timesync_guard_started_ns = receipt_ns
        if new_session:
            self._session_start_px4_timestamp_us = timestamp_us
        # The sample which identifies a delivery-gap or reset session cannot
        # seed a fresh clock proof. Require subsequent samples from the new
        # session so an old in-flight value cannot cross the session fence.
        if not new_session:
            self._recent_px4_clock_timestamps_us.append(timestamp_us)
        self._try_prove_direct_hil_clock()

        gyro = _finite_vector(getattr(message, 'gyro_rad', ()), 3)
        if gyro is not None:
            self._insert_sample(
                self._gyro_samples,
                self._gyro_sample_timestamps_ns,
                ImuVectorSample(
                    remote_time_ns=px4_us_to_airsim_ns(timestamp_us),
                    px4_timestamp_us=timestamp_us,
                    calibration_count=int(getattr(message, 'gyro_calibration_count', 0)),
                    clipping=int(getattr(message, 'gyro_clipping', 0)),
                    vector_frd=gyro,  # type: ignore[arg-type]
                ),
                'gyro',
            )

        relative_us = int(getattr(message, 'accelerometer_timestamp_relative', 0))
        invalid_relative = int(getattr(message, 'RELATIVE_TIMESTAMP_INVALID', 2_147_483_647))
        acceleration = _finite_vector(getattr(message, 'accelerometer_m_s2', ()), 3)
        acceleration_timestamp_us = timestamp_us + relative_us
        if (
            relative_us != invalid_relative
            and abs(relative_us) <= 1_000_000
            and acceleration_timestamp_us > 0
            and acceleration is not None
        ):
            self._insert_sample(
                self._acceleration_samples,
                self._acceleration_sample_timestamps_ns,
                ImuVectorSample(
                    remote_time_ns=px4_us_to_airsim_ns(acceleration_timestamp_us),
                    px4_timestamp_us=acceleration_timestamp_us,
                    calibration_count=int(getattr(message, 'accel_calibration_count', 0)),
                    clipping=int(getattr(message, 'accelerometer_clipping', 0)),
                    vector_frd=acceleration,  # type: ignore[arg-type]
                ),
                'acceleration',
            )
        self._process_pending_images()

    def _gps_cb(self, message: SensorGps) -> None:
        timestamp_us = _source_timestamp_us(message)
        if not self._source_time_is_current(timestamp_us, 'SensorGps'):
            return
        latitude = float(message.latitude_deg)
        longitude = float(message.longitude_deg)
        altitude = float(message.altitude_msl_m)
        eph = float(getattr(message, 'eph', math.nan))
        epv = float(getattr(message, 'epv', math.nan))
        if not all(math.isfinite(value) for value in (latitude, longitude, altitude, eph, epv)):
            self._warn('Ignoring SensorGps containing non-finite position or accuracy')
            return

        sample = GpsSample(
            remote_time_ns=px4_us_to_airsim_ns(timestamp_us),
            px4_timestamp_us=timestamp_us,
            latitude_deg=latitude,
            longitude_deg=longitude,
            altitude_msl_m=altitude,
            horizontal_accuracy_m=max(0.0, eph),
            vertical_accuracy_m=max(0.0, epv),
            fix_type=int(getattr(message, 'fix_type', 0)),
        )
        self._insert_sample(
            self._gps_samples,
            self._gps_sample_timestamps_ns,
            sample,
            'gps',
        )
        self._process_pending_images()

    def _insert_sample(
        self,
        samples: Deque,
        timestamps_ns: List[int],
        sample,
        history_name: str,
    ) -> None:
        if not timestamps_ns or sample.remote_time_ns >= timestamps_ns[-1]:
            if timestamps_ns and sample.remote_time_ns == timestamps_ns[-1]:
                samples[-1] = sample
            else:
                samples.append(sample)
                timestamps_ns.append(sample.remote_time_ns)
        else:
            index = bisect_left(timestamps_ns, sample.remote_time_ns)
            if index < len(samples) and timestamps_ns[index] == sample.remote_time_ns:
                samples[index] = sample
            else:
                samples.insert(index, sample)
                timestamps_ns.insert(index, sample.remote_time_ns)

        cutoff = timestamps_ns[-1] - self._state_history_ns
        while timestamps_ns and timestamps_ns[0] < cutoff:
            samples.popleft()
            del timestamps_ns[0]
        self._history_versions[history_name] += 1

    @staticmethod
    def _bracket(
        samples: Deque,
        timestamps_ns: Sequence[int],
        target_ns: int,
        max_gap_ns: int,
        generation_name: str,
    ) -> Tuple[Optional[Bracket], str]:
        # ``_insert_sample`` preserves this index in ascending order, which
        # avoids rebuilding and rescanning a full history for every frame.
        if len(samples) != len(timestamps_ns):
            return None, 'invalid_history'
        if not timestamps_ns:
            return None, 'no_samples'
        if target_ns < timestamps_ns[0]:
            return None, 'before_history'
        if target_ns > timestamps_ns[-1]:
            return None, 'awaiting_future_sample'

        index = bisect_left(timestamps_ns, target_ns)
        if index < len(timestamps_ns) and timestamps_ns[index] == target_ns:
            return Bracket(samples[index], samples[index], 0.0, 0), 'ready'

        before_index = index - 1
        after_index = index
        span_ns = timestamps_ns[after_index] - timestamps_ns[before_index]
        if span_ns <= 0:
            return None, 'invalid_history'
        if span_ns > max_gap_ns:
            return None, 'state_gap'
        if (
            int(getattr(samples[before_index], generation_name)) !=
            int(getattr(samples[after_index], generation_name))
        ):
            return None, 'state_reset'
        return Bracket(
            before=samples[before_index],
            after=samples[after_index],
            ratio=float(target_ns - timestamps_ns[before_index]) / float(span_ns),
            span_ns=span_ns,
        ), 'ready'

    def _camera_header_cb(self, message: CameraInfo) -> None:
        if self._fatal_error is not None:
            return
        if self._require_px4_mode and (
            self._control_mode is None or self._control_mode.lower() != 'px4'
        ):
            return
        if self._validated_px4_system_id is None:
            return
        if not self._direct_hil_clock_ready(time.monotonic_ns()):
            return

        stamp_ns = _header_stamp_ns(message.header)
        if stamp_ns <= 0:
            self._dropped_images += 1
            self._warn('Dropping camera image without a positive capture timestamp')
            return
        if self._last_received_image_stamp_ns is not None and stamp_ns <= self._last_received_image_stamp_ns:
            self._dropped_images += 1
            self._warn(f'Dropping replayed or out-of-order camera image at {stamp_ns} ns')
            return
        self._last_received_image_stamp_ns = stamp_ns

        if len(self._pending_images) >= self._max_pending_images:
            self._pending_images.popleft()
            self._dropped_images += 1
            self._warn('Dropping oldest pending image because PX4 state is not available')
        self._pending_images.append(
            PendingImage(_copy_header(message.header), stamp_ns, time.monotonic_ns())
        )
        self._process_pending_images()

    def _process_pending_images(self) -> None:
        if not self._pending_images or self._fatal_error is not None:
            return
        now_ns = time.monotonic_ns()
        if not self._direct_hil_clock_ready(now_ns):
            return
        if (
            self._validated_px4_system_id is None
            or self._last_vehicle_status_receipt_ns is None
            or now_ns - self._last_vehicle_status_receipt_ns > self._vehicle_status_max_age_ns
        ):
            return
        remaining: Deque[PendingImage] = deque()
        terminal_reason_suffixes = (
            'before_history', 'state_gap', 'state_reset', 'invalid_history', 'sensor_clipping',
        )
        while self._pending_images:
            image = self._pending_images.popleft()
            if (
                image.retry_history is not None
                and self._history_versions[image.retry_history] == image.retry_history_version
            ):
                if now_ns - image.receipt_ns > self._max_image_wait_ns:
                    self._dropped_images += 1
                    self._warn(
                        f'Dropping image at {image.stamp_ns} ns: {image.last_failure_reason}'
                    )
                    continue
                remaining.append(image)
                continue
            pair, reason = self._make_pair(image)
            if pair is not None:
                self._publish_pair(pair)
                continue
            if reason.endswith(terminal_reason_suffixes) or (
                now_ns - image.receipt_ns > self._max_image_wait_ns
            ):
                self._dropped_images += 1
                self._warn(f'Dropping image at {image.stamp_ns} ns: {reason}')
                continue
            image.last_failure_reason = reason
            image.retry_history = self._retry_history_for_reason(reason)
            if image.retry_history is not None:
                image.retry_history_version = self._history_versions[image.retry_history]
            remaining.append(image)
        self._pending_images = remaining

    @staticmethod
    def _retry_history_for_reason(reason: str) -> Optional[str]:
        for history_name in ('odometry', 'attitude', 'gyro', 'acceleration'):
            if reason in (
                f'{history_name}_no_samples',
                f'{history_name}_awaiting_future_sample',
            ):
                return history_name
        if reason in ('awaiting_gps', 'stale_gps'):
            return 'gps'
        return None

    def _make_pair(self, image: PendingImage) -> Tuple[Optional[SynchronizedPair], str]:
        target_ns = image.stamp_ns
        odom_bracket, reason = self._bracket(
            self._odom_samples,
            self._odom_sample_timestamps_ns,
            target_ns,
            self._max_state_gap_ns,
            'reset_counter',
        )
        if odom_bracket is None:
            return None, 'odometry_' + reason
        attitude_bracket, reason = self._bracket(
            self._attitude_samples,
            self._attitude_sample_timestamps_ns,
            target_ns,
            self._max_state_gap_ns,
            'reset_counter',
        )
        if attitude_bracket is None:
            return None, 'attitude_' + reason
        gyro_bracket, reason = self._bracket(
            self._gyro_samples,
            self._gyro_sample_timestamps_ns,
            target_ns,
            self._max_imu_gap_ns,
            'calibration_count',
        )
        if gyro_bracket is None:
            return None, 'gyro_' + reason
        acceleration_bracket, reason = self._bracket(
            self._acceleration_samples,
            self._acceleration_sample_timestamps_ns,
            target_ns,
            self._max_imu_gap_ns,
            'calibration_count',
        )
        if acceleration_bracket is None:
            return None, 'acceleration_' + reason

        gyro_before: ImuVectorSample = gyro_bracket.before  # type: ignore[assignment]
        gyro_after: ImuVectorSample = gyro_bracket.after  # type: ignore[assignment]
        accel_before: ImuVectorSample = acceleration_bracket.before  # type: ignore[assignment]
        accel_after: ImuVectorSample = acceleration_bracket.after  # type: ignore[assignment]
        if gyro_before.clipping or gyro_after.clipping or accel_before.clipping or accel_after.clipping:
            return None, 'sensor_clipping'

        odom_before: OdomSample = odom_bracket.before  # type: ignore[assignment]
        odom_after: OdomSample = odom_bracket.after  # type: ignore[assignment]
        attitude_before: AttitudeSample = attitude_bracket.before  # type: ignore[assignment]
        attitude_after: AttitudeSample = attitude_bracket.after  # type: ignore[assignment]

        gps_sample: Optional[GpsSample] = None
        gps_age_ns = 0
        gps_fresh = False
        if self._gps_samples:
            gps_index = latest_at_or_before_index(
                self._gps_sample_timestamps_ns, target_ns
            )
            if gps_index is not None:
                gps_sample = self._gps_samples[gps_index]
                gps_age_ns = target_ns - gps_sample.remote_time_ns
                gps_fresh = gps_age_ns <= self._max_gps_age_ns
        if self._require_gps and gps_sample is None:
            return None, 'awaiting_gps'
        if self._require_gps and not gps_fresh:
            return None, 'stale_gps'

        return SynchronizedPair(
            image=image,
            position_ned=lerp_vector(
                odom_before.position_ned, odom_after.position_ned, odom_bracket.ratio
            ),
            velocity_ned=lerp_vector(
                odom_before.velocity_ned, odom_after.velocity_ned, odom_bracket.ratio
            ),
            gyro_frd=lerp_vector(gyro_before.vector_frd, gyro_after.vector_frd, gyro_bracket.ratio),
            acceleration_frd=lerp_vector(
                accel_before.vector_frd, accel_after.vector_frd, acceleration_bracket.ratio
            ),
            odometry_quaternion_ned_frd=quaternion_slerp(
                odom_before.quaternion_ned_frd,
                odom_after.quaternion_ned_frd,
                odom_bracket.ratio,
            ),
            attitude_quaternion_ned_frd=quaternion_slerp(
                attitude_before.quaternion_ned_frd,
                attitude_after.quaternion_ned_frd,
                attitude_bracket.ratio,
            ),
            position_variance_ned=_lerp_optional_vector(
                odom_before.position_variance_ned,
                odom_after.position_variance_ned,
                odom_bracket.ratio,
            ),
            orientation_variance_frd=_lerp_optional_vector(
                odom_before.orientation_variance_frd,
                odom_after.orientation_variance_frd,
                odom_bracket.ratio,
            ),
            velocity_variance_ned=_lerp_optional_vector(
                odom_before.velocity_variance_ned,
                odom_after.velocity_variance_ned,
                odom_bracket.ratio,
            ),
            odometry_bracket=odom_bracket,
            attitude_bracket=attitude_bracket,
            gyro_bracket=gyro_bracket,
            acceleration_bracket=acceleration_bracket,
            gps_sample=gps_sample,
            gps_age_ns=gps_age_ns,
            gps_fresh=gps_fresh,
        ), 'ready'

    def _publish_pair(self, pair: SynchronizedPair) -> None:
        position_enu = ned_to_enu(pair.position_ned)
        velocity_flu = ned_velocity_to_flu_body(
            pair.velocity_ned, pair.odometry_quaternion_ned_frd
        )
        gyro_flu = frd_to_flu(pair.gyro_frd)
        acceleration_flu = frd_to_flu(pair.acceleration_frd)
        odometry_quaternion_enu_flu = px4_attitude_ned_frd_to_enu_flu(
            pair.odometry_quaternion_ned_frd
        )
        imu_quaternion_enu_flu = px4_attitude_ned_frd_to_enu_flu(
            pair.attitude_quaternion_ned_frd
        )

        odometry = Odometry()
        odometry.header = _copy_header(pair.image.header)
        odometry.header.frame_id = self._world_frame_id
        odometry.child_frame_id = self._body_frame_id
        odometry.pose.pose.position.x = position_enu[0]
        odometry.pose.pose.position.y = position_enu[1]
        odometry.pose.pose.position.z = position_enu[2]
        odometry.pose.pose.orientation.w = odometry_quaternion_enu_flu[0]
        odometry.pose.pose.orientation.x = odometry_quaternion_enu_flu[1]
        odometry.pose.pose.orientation.y = odometry_quaternion_enu_flu[2]
        odometry.pose.pose.orientation.z = odometry_quaternion_enu_flu[3]
        odometry.twist.twist.linear.x = velocity_flu[0]
        odometry.twist.twist.linear.y = velocity_flu[1]
        odometry.twist.twist.linear.z = velocity_flu[2]
        odometry.twist.twist.angular.x = gyro_flu[0]
        odometry.twist.twist.angular.y = gyro_flu[1]
        odometry.twist.twist.angular.z = gyro_flu[2]

        if pair.position_variance_ned is not None:
            _write_covariance_block(
                odometry.pose.covariance,
                transform_covariance(diagonal_covariance(pair.position_variance_ned), NED_TO_ENU),
                0,
                0,
                6,
            )
        orientation_covariance = None
        if pair.orientation_variance_frd is not None:
            orientation_covariance = transform_covariance(
                diagonal_covariance(pair.orientation_variance_frd), FRD_TO_FLU
            )
            _write_covariance_block(odometry.pose.covariance, orientation_covariance, 3, 3, 6)
        if pair.velocity_variance_ned is not None:
            _write_covariance_block(
                odometry.twist.covariance,
                ned_velocity_covariance_to_flu_body(
                    pair.velocity_variance_ned, pair.odometry_quaternion_ned_frd
                ),
                0,
                0,
                6,
            )

        imu = Imu()
        imu.header = _copy_header(pair.image.header)
        imu.header.frame_id = self._body_frame_id
        imu.orientation.w = imu_quaternion_enu_flu[0]
        imu.orientation.x = imu_quaternion_enu_flu[1]
        imu.orientation.y = imu_quaternion_enu_flu[2]
        imu.orientation.z = imu_quaternion_enu_flu[3]
        imu.angular_velocity.x = gyro_flu[0]
        imu.angular_velocity.y = gyro_flu[1]
        imu.angular_velocity.z = gyro_flu[2]
        imu.linear_acceleration.x = acceleration_flu[0]
        imu.linear_acceleration.y = acceleration_flu[1]
        imu.linear_acceleration.z = acceleration_flu[2]
        if orientation_covariance is not None:
            _write_covariance_block(
                imu.orientation_covariance, orientation_covariance, 0, 0, 3
            )
        # The all-zero angular-velocity and linear-acceleration covariance
        # arrays mean unknown covariance while those measurements remain
        # available, per sensor_msgs/Imu.

        gps = None
        if self._publish_gps and pair.gps_sample is not None and pair.gps_fresh:
            sample = pair.gps_sample
            gps = NavSatFix()
            _set_stamp_ns(gps.header, sample.remote_time_ns)
            gps.header.frame_id = self._gps_frame_id
            gps.status.service = NavSatStatus.SERVICE_GPS
            gps.status.status = (
                NavSatStatus.STATUS_FIX if sample.fix_type >= 2 else NavSatStatus.STATUS_NO_FIX
            )
            gps.latitude = sample.latitude_deg
            gps.longitude = sample.longitude_deg
            gps.altitude = sample.altitude_msl_m
            gps.position_covariance[0] = sample.horizontal_accuracy_m ** 2
            gps.position_covariance[4] = sample.horizontal_accuracy_m ** 2
            gps.position_covariance[8] = sample.vertical_accuracy_m ** 2
            gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        odom_before: OdomSample = pair.odometry_bracket.before  # type: ignore[assignment]
        odom_after: OdomSample = pair.odometry_bracket.after  # type: ignore[assignment]
        attitude_before: AttitudeSample = pair.attitude_bracket.before  # type: ignore[assignment]
        attitude_after: AttitudeSample = pair.attitude_bracket.after  # type: ignore[assignment]
        gyro_before: ImuVectorSample = pair.gyro_bracket.before  # type: ignore[assignment]
        gyro_after: ImuVectorSample = pair.gyro_bracket.after  # type: ignore[assignment]
        accel_before: ImuVectorSample = pair.acceleration_bracket.before  # type: ignore[assignment]
        accel_after: ImuVectorSample = pair.acceleration_bracket.after  # type: ignore[assignment]

        sync = Px4ImageSync()
        sync.image_header = _copy_header(pair.image.header)
        sync.image_topic = self._camera_topic
        sync.airsim_vehicle_name = self._airsim_vehicle_name
        sync.px4_topic_prefix = self._px4_topic_prefix
        sync.px4_system_id = self._validated_px4_system_id or 0
        sync.px4_clock_epoch = self._px4_clock_epoch
        sync.px4_clock_timestamp_us = self._latest_px4_timestamp_us or 0
        sync.direct_hil_clock_verified = self._direct_hil_clock_verified
        sync.direct_hil_clock_match_count = self._direct_hil_clock_match_count
        sync.direct_hil_clock_last_matched_timestamp_us = (
            self._last_matched_hil_timestamp_us or 0
        )
        sync.odometry_before_timestamp_us = odom_before.px4_timestamp_us
        sync.odometry_after_timestamp_us = odom_after.px4_timestamp_us
        sync.attitude_before_timestamp_us = attitude_before.px4_timestamp_us
        sync.attitude_after_timestamp_us = attitude_after.px4_timestamp_us
        sync.gyro_before_timestamp_us = gyro_before.px4_timestamp_us
        sync.gyro_after_timestamp_us = gyro_after.px4_timestamp_us
        sync.accelerometer_before_timestamp_us = accel_before.px4_timestamp_us
        sync.accelerometer_after_timestamp_us = accel_after.px4_timestamp_us
        sync.odometry_bracket_span_ns = pair.odometry_bracket.span_ns
        sync.attitude_bracket_span_ns = pair.attitude_bracket.span_ns
        sync.gyro_bracket_span_ns = pair.gyro_bracket.span_ns
        sync.accelerometer_bracket_span_ns = pair.acceleration_bracket.span_ns
        sync.odometry_interpolation_ratio = pair.odometry_bracket.ratio
        sync.attitude_interpolation_ratio = pair.attitude_bracket.ratio
        sync.gyro_interpolation_ratio = pair.gyro_bracket.ratio
        sync.accelerometer_interpolation_ratio = pair.acceleration_bracket.ratio
        sync.odometry_interpolated = pair.odometry_bracket.before is not pair.odometry_bracket.after
        sync.attitude_interpolated = pair.attitude_bracket.before is not pair.attitude_bracket.after
        sync.gyro_interpolated = pair.gyro_bracket.before is not pair.gyro_bracket.after
        sync.accelerometer_interpolated = (
            pair.acceleration_bracket.before is not pair.acceleration_bracket.after
        )
        sync.odometry_reset_counter = odom_before.reset_counter
        sync.attitude_reset_counter = attitude_before.reset_counter
        sync.gyro_calibration_count = gyro_before.calibration_count
        sync.accelerometer_calibration_count = accel_before.calibration_count
        sync.gps_available = pair.gps_fresh
        sync.gps_timestamp_us = pair.gps_sample.px4_timestamp_us if pair.gps_sample else 0
        sync.gps_age_ns = pair.gps_age_ns
        sync.px4_to_airsim_offset_ns = 0

        self._odom_pub.publish(odometry)
        self._imu_pub.publish(imu)
        if gps is not None:
            self._gps_pub.publish(gps)
        self._sync_pub.publish(sync)
        self._published_pairs += 1
        self._last_pair_receipt_ns = time.monotonic_ns()
        self._publish_status('READY')

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
        if (
            self._last_vehicle_status_receipt_ns is None
            or now_ns - self._last_vehicle_status_receipt_ns > self._vehicle_status_max_age_ns
        ):
            self._publish_status('STALE_PX4_VEHICLE_STATUS')
            return

        if self._last_px4_clock_receipt_ns is None:
            if now_ns - self._started_ns > self._px4_clock_timeout_sec * 1_000_000_000:
                self._fatal(
                    f'Timed out waiting for an advancing PX4 HIL clock on '
                    f'{self._px4_sensor_combined_topic}'
                )
            else:
                self._publish_status('WAITING_FOR_PX4_CLOCK')
            return
        if now_ns - self._last_px4_clock_receipt_ns > self._px4_clock_max_age_ns:
            self._clear_direct_hil_clock_proof('PX4 SensorCombined clock is stale')
            self._publish_status('STALE_PX4_CLOCK')
            return

        if not self._direct_hil_clock_ready(now_ns):
            if self._last_hil_clock_receipt_ns is None:
                self._publish_status('WAITING_FOR_AIRSIM_HIL_CLOCK')
            elif now_ns - self._last_hil_clock_receipt_ns > self._hil_clock_max_age_ns:
                self._publish_status('STALE_AIRSIM_HIL_CLOCK')
            elif not self._dds_timesync_guard_verified:
                self._publish_status('WAITING_FOR_PX4_DDS_TIMESYNC_GUARD')
            else:
                self._publish_status('WAITING_FOR_DIRECT_HIL_CLOCK_PROOF')
            return

        self._process_pending_images()
        if self._published_pairs == 0:
            self._publish_status('WAITING_FOR_CAMERA_AND_PX4_STATE')
        elif self._last_pair_receipt_ns is None or (
            now_ns - self._last_pair_receipt_ns > self._sync_health_timeout_ns
        ):
            self._publish_status('STALE_SYNC')
        else:
            self._publish_status('READY')

    def _warn(self, message: str) -> None:
        now_ns = time.monotonic_ns()
        if now_ns - self._last_warning_ns >= 1_000_000_000:
            self._last_warning_ns = now_ns
            self.get_logger().warning(message)


def main(args=None) -> int:
    rclpy.init(args=args)
    node: Optional[Px4CameraSync] = None
    exit_code = 0
    try:
        node = Px4CameraSync()
        while rclpy.ok() and node.fatal_error is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.fatal_error is not None:
            exit_code = 1
    except (RuntimeError, ValueError) as error:
        print(f'px4_camera_sync startup failed: {error}')
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
