"""Pure safety rules for the PX4 body-rate command bridge."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Collection, Optional, Tuple


DEFAULT_PX4_SESSION_GAP_SEC = 0.1


@dataclass(frozen=True)
class RateCommand:
    roll_rate_flu: float
    pitch_rate_flu: float
    yaw_rate_flu: float
    thrust_up: float
    source_time_ns: int
    receipt_ns: int


def validate_rate_values(
    roll_rate: float,
    pitch_rate: float,
    yaw_rate: float,
    thrust: float,
    max_roll_rate: float,
    max_pitch_rate: float,
    max_yaw_rate: float,
) -> Tuple[float, float, float, float]:
    values = tuple(float(value) for value in (roll_rate, pitch_rate, yaw_rate, thrust))
    if not all(math.isfinite(value) for value in values):
        raise ValueError('non_finite_setpoint')
    roll, pitch, yaw, collective = values
    if (
        abs(roll) > max_roll_rate
        or abs(pitch) > max_pitch_rate
        or abs(yaw) > max_yaw_rate
        or collective < 0.0
        or collective > 1.0
    ):
        raise ValueError('setpoint_out_of_bounds')
    return roll, pitch, yaw, collective


def validate_source_timestamp(
    source_time_ns: int,
    previous_source_time_ns: Optional[int],
    latest_px4_time_ns: Optional[int],
    max_future_ns: int,
) -> None:
    if source_time_ns <= 0:
        raise ValueError('missing_source_timestamp')
    if previous_source_time_ns is not None and source_time_ns <= previous_source_time_ns:
        raise ValueError('replayed_source_timestamp')
    if latest_px4_time_ns is not None and source_time_ns > latest_px4_time_ns + max_future_ns:
        raise ValueError('source_timestamp_in_future')


def timestamp_progress(previous_timestamp: Optional[int], timestamp: int) -> str:
    if timestamp <= 0:
        return 'invalid'
    if previous_timestamp is None:
        return 'first'
    if timestamp > previous_timestamp:
        return 'advanced'
    if timestamp == previous_timestamp:
        return 'duplicate'
    return 'regressed'


def seconds_to_positive_nanoseconds(seconds: float, parameter_name: str) -> int:
    """Convert a positive finite duration without allowing sub-nanosecond truncation.

    Session fences are evaluated in monotonic receipt time. Letting a small,
    positive seconds value silently truncate to zero would defer a configuration
    error until the first live PX4 callback.
    """

    if not math.isfinite(seconds) or seconds <= 0.0:
        raise ValueError(f'{parameter_name} must be finite and positive')
    nanoseconds = seconds * 1_000_000_000.0
    if not math.isfinite(nanoseconds) or nanoseconds < 1.0:
        raise ValueError(f'{parameter_name} must resolve to at least one nanosecond')
    return int(nanoseconds)


def receipt_gap_exceeds(
    previous_receipt_ns: Optional[int],
    receipt_ns: int,
    maximum_gap_ns: int,
) -> bool:
    """Return whether a live PX4 stream has a session-sized receipt gap."""

    if receipt_ns <= 0 or maximum_gap_ns <= 0:
        raise ValueError('receipt timestamps and maximum gap must be positive')
    return (
        previous_receipt_ns is not None
        and receipt_ns > previous_receipt_ns + maximum_gap_ns
    )


def command_readiness(
    command: Optional[RateCommand],
    now_receipt_ns: int,
    latest_px4_time_ns: Optional[int],
    latest_sync_time_ns: Optional[int],
    latest_sync_receipt_ns: Optional[int],
    accepted_sync_stamps: Collection[int],
    receipt_timeout_ns: int,
    source_timeout_ns: int,
    max_future_ns: int,
    sync_timeout_ns: int,
    require_image_sync: bool,
) -> Tuple[bool, str]:
    """Evaluate independent transport, source-clock, and image-sync guards."""

    if command is None:
        return False, 'waiting_for_setpoint'
    if latest_px4_time_ns is None:
        return False, 'waiting_for_px4_clock'
    if now_receipt_ns - command.receipt_ns > receipt_timeout_ns:
        return False, 'stale_setpoint_transport'
    if command.source_time_ns > latest_px4_time_ns + max_future_ns:
        return False, 'setpoint_source_in_future'
    if latest_px4_time_ns - command.source_time_ns > source_timeout_ns:
        return False, 'stale_setpoint_source'

    if require_image_sync:
        if latest_sync_time_ns is None or latest_sync_receipt_ns is None:
            return False, 'waiting_for_image_sync'
        if now_receipt_ns - latest_sync_receipt_ns > sync_timeout_ns:
            return False, 'stale_image_sync'
        if command.source_time_ns > latest_sync_time_ns:
            return False, 'setpoint_ahead_of_image_sync'
        if command.source_time_ns not in accepted_sync_stamps:
            return False, 'setpoint_not_bound_to_image_sync'

    return True, 'ready'
