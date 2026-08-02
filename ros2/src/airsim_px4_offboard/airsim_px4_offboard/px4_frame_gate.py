"""Relay complete AirSim camera samples only after their PX4 synchronization proof.

AirSim's raw image, CameraInfo, and :class:`Px4ImageSync` are separate DDS
deliveries.  A consumer must therefore not infer that receiving one side means
the synchronizer accepted the complete camera sample.  This node matches the
untouched image payload and CameraInfo to an accepted synchronization event by
their exact rendered-frame header stamp and relays all three only after the
synchronizer reports ``READY``.

The relay deliberately supports only the two AirSim image transport payload
types used by this pipeline: ``sensor_msgs/Image`` (raw) and
``sensor_msgs/CompressedImage`` (compressed).  It never decodes, re-encodes,
or changes the payload/header.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
import math
import time
from typing import Optional, Union

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from airsim_interfaces.msg import Px4ImageSync
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String


_MIN_DIRECT_HIL_CLOCK_MATCHES = 3
_Payload = Union[Image, CompressedImage]


@dataclass(frozen=True)
class PendingPayload:
    message: _Payload
    receipt_ns: int


@dataclass(frozen=True)
class PendingSync:
    message: Px4ImageSync
    receipt_ns: int


@dataclass(frozen=True)
class PendingCameraInfo:
    message: CameraInfo
    receipt_ns: int


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


def _default_sync_image_topic(image_topic: str, image_transport: str) -> str:
    """Return the raw topic normally recorded in ``Px4ImageSync.image_topic``."""

    if image_transport == 'compressed' and image_topic.rstrip('/').endswith('/compressed'):
        return image_topic.rstrip('/')[:-len('/compressed')]
    return image_topic.rstrip('/')


def _default_camera_info_topic(image_topic: str) -> str:
    """Return the CameraInfo topic paired with a logical AirSim image topic."""

    cleaned = image_topic.rstrip('/')
    suffix = '/image'
    if not cleaned.endswith(suffix):
        raise ValueError('expected_sync_image_topic must end in /image')
    return cleaned[:-len(suffix)] + '/camera_info'


def _jpeg_dimensions(message: CompressedImage) -> Optional[tuple[int, int]]:
    """Return JPEG dimensions only for a structurally valid baseline frame header.

    The gate deliberately never decodes or re-encodes payloads.  Parsing the
    marker segments still prevents a truncated or arbitrary SOI/EOI byte pair
    from being released as a camera sample and lets the gate verify that the
    encoded pixels agree with their paired CameraInfo.
    """

    data = message.data
    if (
        len(data) < 4
        or data[0] != 0xff
        or data[1] != 0xd8
        or data[-2] != 0xff
        or data[-1] != 0xd9
    ):
        return None

    index = 2
    dimensions: Optional[tuple[int, int]] = None
    sof_markers = {
        0xc0, 0xc1, 0xc2, 0xc3,
        0xc5, 0xc6, 0xc7,
        0xc9, 0xca, 0xcb,
        0xcd, 0xce, 0xcf,
    }
    while index < len(data) - 2:
        if data[index] != 0xff:
            return None
        while index < len(data) and data[index] == 0xff:
            index += 1
        if index >= len(data):
            return None
        marker = data[index]
        index += 1
        if marker in (0x00, 0xd8, 0xd9) or marker == 0x01 or 0xd0 <= marker <= 0xd7:
            return None
        if index + 2 > len(data):
            return None
        segment_length = (int(data[index]) << 8) | int(data[index + 1])
        if segment_length < 2 or index + segment_length > len(data):
            return None
        payload_start = index + 2
        if marker in sof_markers:
            if segment_length < 8:
                return None
            height = (int(data[payload_start + 1]) << 8) | int(data[payload_start + 2])
            width = (int(data[payload_start + 3]) << 8) | int(data[payload_start + 4])
            component_count = int(data[payload_start + 5])
            if (
                width <= 0
                or height <= 0
                or component_count <= 0
                or segment_length != 8 + component_count * 3
            ):
                return None
            dimensions = (width, height)
        elif marker == 0xda:
            if dimensions is None or segment_length < 6:
                return None
            component_count = int(data[payload_start])
            if component_count <= 0 or segment_length != 6 + component_count * 2:
                return None
            # Entropy bytes may legitimately contain marker-like values.  The
            # validated SOS plus terminal EOI above is sufficient here; image
            # decoding remains the consumer's responsibility.
            return dimensions
        index += segment_length
    return None


class Px4FrameGate(Node):
    """Bounded exact-stamp image relay guarded by ``Px4ImageSync``."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__('px4_frame_gate', parameter_overrides=parameter_overrides)

        self._image_topic = str(_parameter(self, 'image_topic', '')).rstrip('/')
        self._image_transport = str(_parameter(self, 'image_transport', 'raw')).strip().lower()
        self._camera_info_topic = str(_parameter(self, 'camera_info_topic', '')).rstrip('/')
        self._image_sync_topic = str(
            _parameter(self, 'image_sync_topic', 'camera_sync/image_sync')
        ).rstrip('/')
        self._sync_status_topic = str(_parameter(self, 'sync_status_topic', '')).rstrip('/')
        self._output_prefix = str(_parameter(self, 'output_prefix', 'camera_gate')).rstrip('/')
        self._expected_sync_image_topic = str(
            _parameter(self, 'expected_sync_image_topic', '')
        ).rstrip('/')
        self._airsim_vehicle_name = str(
            _parameter(self, 'airsim_vehicle_name', 'drone1')
        ).strip('/')
        self._px4_topic_prefix = str(_parameter(self, 'px4_topic_prefix', '/fmu')).rstrip('/')
        self._expected_px4_system_id = int(_parameter(self, 'expected_px4_system_id', 1))
        self._max_pending_frames = int(_parameter(self, 'max_pending_frames', 5))
        self._max_frame_wait_ns = int(
            float(_parameter(self, 'max_frame_wait_sec', 0.15)) * 1_000_000_000.0
        )
        self._max_source_age_ns = int(
            float(_parameter(self, 'max_source_age_sec', 0.25)) * 1_000_000_000.0
        )

        if (
            not self._image_topic
            or not self._image_sync_topic
            or not self._output_prefix
        ):
            raise ValueError('image_topic, image_sync_topic, and output_prefix must not be empty')
        if self._image_transport not in ('raw', 'compressed'):
            raise ValueError("image_transport must be 'raw' or 'compressed'")
        if not self._expected_sync_image_topic:
            self._expected_sync_image_topic = _default_sync_image_topic(
                self._image_topic, self._image_transport
            )
        if not self._camera_info_topic:
            self._camera_info_topic = _default_camera_info_topic(
                self._expected_sync_image_topic
            )
        if not self._camera_info_topic:
            raise ValueError('camera_info_topic must not be empty')
        if not self._sync_status_topic:
            suffix = '/image_sync'
            if not self._image_sync_topic.endswith(suffix):
                raise ValueError(
                    'sync_status_topic must be supplied when image_sync_topic does not end in /image_sync'
                )
            self._sync_status_topic = self._image_sync_topic[:-len(suffix)] + '/status'
        if not self._airsim_vehicle_name or not self._px4_topic_prefix:
            raise ValueError('airsim_vehicle_name and px4_topic_prefix must not be empty')
        if self._expected_px4_system_id < 1 or self._expected_px4_system_id > 255:
            raise ValueError('expected_px4_system_id must be in [1, 255]')
        if self._max_pending_frames < 1:
            raise ValueError('max_pending_frames must be at least one')
        if self._max_frame_wait_ns <= 0 or self._max_source_age_ns <= 0:
            raise ValueError('frame-gate time bounds must be finite and positive')

        self._status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._payload_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=self._max_pending_frames,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._status_pub = self.create_publisher(
            String, _join_topic(self._output_prefix, 'status'), self._status_qos
        )
        self._sync_pub = self.create_publisher(
            Px4ImageSync, _join_topic(self._output_prefix, 'image_sync'), self._payload_qos
        )
        self._camera_info_pub = self.create_publisher(
            CameraInfo, _join_topic(self._output_prefix, 'camera_info'), self._payload_qos
        )
        if self._image_transport == 'raw':
            self._payload_pub = self.create_publisher(
                Image, _join_topic(self._output_prefix, 'image'), self._payload_qos
            )
            self._payload_sub = self.create_subscription(
                Image, self._image_topic, self._payload_cb, self._payload_qos
            )
        else:
            self._payload_pub = self.create_publisher(
                CompressedImage,
                _join_topic(self._output_prefix, 'image/compressed'),
                self._payload_qos,
            )
            self._payload_sub = self.create_subscription(
                CompressedImage, self._image_topic, self._payload_cb, self._payload_qos
            )
        self._sync_sub = self.create_subscription(
            Px4ImageSync, self._image_sync_topic, self._sync_cb, qos_profile_sensor_data
        )
        self._camera_info_sub = self.create_subscription(
            CameraInfo, self._camera_info_topic, self._camera_info_cb, self._payload_qos
        )
        self._sync_status_sub = self.create_subscription(
            String, self._sync_status_topic, self._sync_status_cb, self._status_qos
        )
        self._expiry_timer = self.create_timer(0.05, self._expiry_timer_cb)

        self._input_ready = False
        self._current_epoch: Optional[int] = None
        self._fatal_error: Optional[str] = None
        self._last_status: Optional[str] = None
        self._last_warning_ns = 0
        self._last_released_stamp_ns: Optional[int] = None
        self._pending_payloads: OrderedDict[int, PendingPayload] = OrderedDict()
        self._pending_camera_infos: OrderedDict[int, PendingCameraInfo] = OrderedDict()
        self._pending_syncs: OrderedDict[int, PendingSync] = OrderedDict()
        self._published_frames = 0
        self._dropped_frames = 0
        self._publish_status('WAITING_FOR_SYNC_READY')
        self.get_logger().info(
            f'Gating {self._image_transport} payloads from {self._image_topic} and CameraInfo '
            f'from {self._camera_info_topic} against {self._image_sync_topic}; accepted camera '
            f'samples publish below {self._output_prefix}'
        )

    @property
    def fatal_error(self) -> Optional[str]:
        return self._fatal_error

    @property
    def published_frames(self) -> int:
        return self._published_frames

    @property
    def dropped_frames(self) -> int:
        return self._dropped_frames

    def _publish_status(self, status: str) -> None:
        if status == self._last_status:
            return
        self._last_status = status
        self._status_pub.publish(String(data=status))
        self.get_logger().info(f'PX4 frame gate: {status}')

    def _warn(self, message: str) -> None:
        now_ns = time.monotonic_ns()
        if now_ns - self._last_warning_ns >= 1_000_000_000:
            self._last_warning_ns = now_ns
            self.get_logger().warning(message)

    def _fence(self, status: str, reason: str) -> None:
        self._dropped_frames += (
            len(self._pending_payloads)
            + len(self._pending_camera_infos)
            + len(self._pending_syncs)
        )
        self._pending_payloads.clear()
        self._pending_camera_infos.clear()
        self._pending_syncs.clear()
        self._last_released_stamp_ns = None
        self._publish_status(status)
        self._warn(reason)

    def _fatal(self, message: str) -> None:
        if self._fatal_error is None:
            self._fatal_error = message
            self._input_ready = False
            self._fence('ERROR: ' + message, message)
            self.get_logger().error(message)

    def _sync_status_cb(self, message: String) -> None:
        if self._fatal_error is not None:
            return
        status = message.data.strip()
        if status == 'READY':
            self._input_ready = True
            if self._pending_payloads or self._pending_camera_infos or self._pending_syncs:
                self._try_all_matches()
            return
        self._input_ready = False
        self._current_epoch = None
        self._fence('WAITING_FOR_SYNC_READY', f'Input synchronizer is {status or "not ready"}')

    def _validate_sync_identity(self, message: Px4ImageSync) -> bool:
        if message.image_topic.rstrip('/') != self._expected_sync_image_topic:
            self._fatal(
                f'Px4ImageSync image_topic={message.image_topic!r} does not match '
                f'expected_sync_image_topic={self._expected_sync_image_topic!r}'
            )
            return False
        if (
            message.airsim_vehicle_name != self._airsim_vehicle_name
            or message.px4_topic_prefix.rstrip('/') != self._px4_topic_prefix
            or int(message.px4_system_id) != self._expected_px4_system_id
        ):
            self._fatal(
                'Px4ImageSync vehicle binding does not match this frame gate: '
                f'got AirSim={message.airsim_vehicle_name}, PX4 prefix={message.px4_topic_prefix}, '
                f'system_id={message.px4_system_id}'
            )
            return False
        return True

    def _sync_cb(self, message: Px4ImageSync) -> None:
        if self._fatal_error is not None:
            return
        if not self._validate_sync_identity(message):
            return
        if (
            not bool(message.direct_hil_clock_verified)
            or int(message.direct_hil_clock_match_count) < _MIN_DIRECT_HIL_CLOCK_MATCHES
            or int(message.direct_hil_clock_last_matched_timestamp_us) <= 0
        ):
            self._warn('Ignoring Px4ImageSync without complete direct HIL clock proof')
            return

        epoch = int(message.px4_clock_epoch)
        if epoch < 0:
            self._fatal('Px4ImageSync has a negative PX4 clock epoch')
            return
        if self._current_epoch is not None and epoch < self._current_epoch:
            self._warn(
                f'Ignoring stale Px4ImageSync epoch {epoch}; current epoch is {self._current_epoch}'
            )
            return
        if self._current_epoch is not None and epoch > self._current_epoch:
            self._input_ready = False
            self._fence(
                'WAITING_FOR_SYNC_READY',
                f'Px4ImageSync advanced from PX4 epoch {self._current_epoch} to {epoch}',
            )
        self._current_epoch = epoch

        if not self._input_ready:
            return
        stamp_ns = _stamp_ns(message.image_header)
        if stamp_ns <= 0:
            self._dropped_frames += 1
            self._warn('Ignoring Px4ImageSync without a positive image timestamp')
            return
        px4_clock_ns = int(message.px4_clock_timestamp_us) * 1_000
        if px4_clock_ns <= 0 or stamp_ns > px4_clock_ns:
            self._dropped_frames += 1
            self._warn('Ignoring Px4ImageSync with an invalid PX4 clock/image timestamp relationship')
            return
        if px4_clock_ns - stamp_ns > self._max_source_age_ns:
            self._dropped_frames += 1
            self._warn('Ignoring Px4ImageSync whose image source age exceeds the configured bound')
            return
        self._put_pending(
            self._pending_syncs, stamp_ns, PendingSync(message, time.monotonic_ns()), 'sync'
        )
        self._try_release(stamp_ns)

    def _payload_cb(self, message: _Payload) -> None:
        if self._fatal_error is not None or not self._input_ready:
            return
        if isinstance(message, CompressedImage) and _jpeg_dimensions(message) is None:
            self._dropped_frames += 1
            self._warn('Dropping compressed image payload without a valid JPEG frame header')
            return
        stamp_ns = _stamp_ns(message.header)
        if stamp_ns <= 0:
            self._dropped_frames += 1
            self._warn('Dropping image payload without a positive capture timestamp')
            return
        if self._last_released_stamp_ns is not None and stamp_ns <= self._last_released_stamp_ns:
            self._dropped_frames += 1
            self._warn(f'Dropping replayed or out-of-order image payload at {stamp_ns} ns')
            return
        self._put_pending(
            self._pending_payloads, stamp_ns, PendingPayload(message, time.monotonic_ns()), 'payload'
        )
        self._try_release(stamp_ns)

    def _camera_info_cb(self, message: CameraInfo) -> None:
        if self._fatal_error is not None or not self._input_ready:
            return
        stamp_ns = _stamp_ns(message.header)
        if stamp_ns <= 0:
            self._dropped_frames += 1
            self._warn('Dropping CameraInfo without a positive capture timestamp')
            return
        if self._last_released_stamp_ns is not None and stamp_ns <= self._last_released_stamp_ns:
            self._dropped_frames += 1
            self._warn(f'Dropping replayed or out-of-order CameraInfo at {stamp_ns} ns')
            return
        self._put_pending(
            self._pending_camera_infos,
            stamp_ns,
            PendingCameraInfo(message, time.monotonic_ns()),
            'CameraInfo',
        )
        self._try_release(stamp_ns)

    def _put_pending(self, cache: OrderedDict, stamp_ns: int, pending, name: str) -> None:
        if stamp_ns in cache:
            self._dropped_frames += 1
            self._warn(f'Dropping duplicate {name} at {stamp_ns} ns')
            return
        if len(cache) >= self._max_pending_frames:
            cache.popitem(last=False)
            self._dropped_frames += 1
            self._warn(f'Dropping oldest pending {name} because the frame-gate cache is full')
        cache[stamp_ns] = pending

    def _try_all_matches(self) -> None:
        for stamp_ns in tuple(self._pending_payloads):
            self._try_release(stamp_ns)

    def _try_release(self, stamp_ns: int) -> None:
        payload = self._pending_payloads.get(stamp_ns)
        camera_info = self._pending_camera_infos.get(stamp_ns)
        sync = self._pending_syncs.get(stamp_ns)
        if payload is None or camera_info is None or sync is None:
            return
        del self._pending_payloads[stamp_ns]
        del self._pending_camera_infos[stamp_ns]
        del self._pending_syncs[stamp_ns]

        payload_header = payload.message.header
        camera_info_header = camera_info.message.header
        sync_header = sync.message.image_header
        if (
            payload_header.frame_id != sync_header.frame_id
            or camera_info_header.frame_id != sync_header.frame_id
        ):
            self._dropped_frames += 1
            self._warn(
                f'Dropping frame {stamp_ns} ns because Image, CameraInfo, and Px4ImageSync '
                'frame IDs differ'
            )
            return
        if int(camera_info.message.width) <= 0 or int(camera_info.message.height) <= 0:
            self._dropped_frames += 1
            self._warn(f'Dropping frame {stamp_ns} ns because CameraInfo dimensions are invalid')
            return
        if isinstance(payload.message, Image) and (
            int(camera_info.message.width) != int(payload.message.width)
            or int(camera_info.message.height) != int(payload.message.height)
        ):
            self._dropped_frames += 1
            self._warn(
                f'Dropping frame {stamp_ns} ns because raw Image and CameraInfo dimensions differ'
            )
            return
        if isinstance(payload.message, CompressedImage):
            jpeg_dimensions = _jpeg_dimensions(payload.message)
            if jpeg_dimensions is None or jpeg_dimensions != (
                int(camera_info.message.width),
                int(camera_info.message.height),
            ):
                self._dropped_frames += 1
                self._warn(
                    f'Dropping frame {stamp_ns} ns because JPEG and CameraInfo dimensions differ'
                )
                return
        if self._last_released_stamp_ns is not None and stamp_ns <= self._last_released_stamp_ns:
            self._dropped_frames += 1
            self._warn(f'Dropping replayed synchronized frame at {stamp_ns} ns')
            return

        # Publish the complete camera sample before the synchronization event
        # that authorizes a command. DDS does not promise cross-topic delivery
        # ordering, but an algorithm can only create the stamped command after
        # receiving its gated payload and matching CameraInfo.
        self._camera_info_pub.publish(camera_info.message)
        self._payload_pub.publish(payload.message)
        self._sync_pub.publish(sync.message)
        self._last_released_stamp_ns = stamp_ns
        self._published_frames += 1
        self._publish_status('READY')

    def _expire_pending(self, now_ns: int) -> None:
        for cache in (
            self._pending_payloads,
            self._pending_camera_infos,
            self._pending_syncs,
        ):
            expired = [
                stamp_ns
                for stamp_ns, pending in cache.items()
                if now_ns - pending.receipt_ns > self._max_frame_wait_ns
            ]
            for stamp_ns in expired:
                del cache[stamp_ns]
                self._dropped_frames += 1
                self._warn(f'Dropping unmatched frame side at {stamp_ns} ns after bounded wait')

    def _expiry_timer_cb(self) -> None:
        if self._fatal_error is not None:
            return
        self._expire_pending(time.monotonic_ns())
        if not self._input_ready:
            self._publish_status('WAITING_FOR_SYNC_READY')
        elif self._published_frames == 0:
            self._publish_status('WAITING_FOR_MATCHED_FRAME')


def main(args=None) -> int:
    rclpy.init(args=args)
    node: Optional[Px4FrameGate] = None
    exit_code = 0
    try:
        node = Px4FrameGate()
        while rclpy.ok() and node.fatal_error is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.fatal_error is not None:
            exit_code = 1
    except (RuntimeError, ValueError) as error:
        print(f'px4_frame_gate startup failed: {error}')
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
