"""Pure synchronization and frame helpers for the PX4 ROS 2 bridge.

PX4 vehicle state uses NED world coordinates and FRD body coordinates.  The
bridge publishes conventional ROS ENU world and FLU body coordinates.  These
helpers stay independent of ROS so the safety-critical timestamp and frame
rules can be unit tested without a running graph.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from bisect import bisect_left, bisect_right
from typing import Iterable, Optional, Sequence, Tuple


Quaternion = Tuple[float, float, float, float]  # w, x, y, z
Vector3 = Tuple[float, float, float]
Matrix3 = Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]


NED_TO_ENU: Matrix3 = (
    (0.0, 1.0, 0.0),
    (1.0, 0.0, 0.0),
    (0.0, 0.0, -1.0),
)

FRD_TO_FLU: Matrix3 = (
    (1.0, 0.0, 0.0),
    (0.0, -1.0, 0.0),
    (0.0, 0.0, -1.0),
)


@dataclass(frozen=True)
class InterpolationBracket:
    before_index: int
    after_index: int
    ratio: float
    span_ns: int


def px4_us_to_airsim_ns(timestamp_us: int) -> int:
    """Map PX4 HIL microseconds directly into AirSim's nanosecond clock.

    Callers use this only after the synchronizer has verified exact
    ``HIL_SENSOR.time_usec``/``SensorCombined.timestamp`` matches. No DDS or
    wall-clock offset is valid on that proven path.
    """

    if timestamp_us <= 0:
        raise ValueError('PX4 timestamp must be positive')
    return timestamp_us * 1_000


def interpolation_bracket(
    timestamps_ns: Sequence[int],
    target_ns: int,
    max_gap_ns: int,
    continuity_generations: Optional[Sequence[int]] = None,
) -> Tuple[Optional[InterpolationBracket], str]:
    """Find a bounded interpolation bracket without crossing a reset.

    ``continuity_generations`` contains estimator reset or sensor calibration
    counters.  A bracket whose endpoints have different counters is rejected
    rather than interpolating across a discontinuity.
    """

    if max_gap_ns <= 0:
        raise ValueError('maximum interpolation gap must be positive')
    if continuity_generations is not None and len(continuity_generations) != len(timestamps_ns):
        raise ValueError('continuity generation count must match timestamp count')
    if not timestamps_ns:
        return None, 'no_samples'
    if any(right < left for left, right in zip(timestamps_ns, timestamps_ns[1:])):
        return None, 'invalid_history'
    if target_ns < timestamps_ns[0]:
        return None, 'before_history'
    if target_ns > timestamps_ns[-1]:
        return None, 'awaiting_future_sample'

    index = bisect_left(timestamps_ns, target_ns)
    if index < len(timestamps_ns) and timestamps_ns[index] == target_ns:
        return InterpolationBracket(index, index, 0.0, 0), 'ready'

    before_index = index - 1
    after_index = index
    span_ns = timestamps_ns[after_index] - timestamps_ns[before_index]
    if span_ns <= 0:
        return None, 'invalid_history'
    if span_ns > max_gap_ns:
        return None, 'state_gap'
    if continuity_generations is not None and (
        continuity_generations[before_index] != continuity_generations[after_index]
    ):
        return None, 'state_reset'

    ratio = float(target_ns - timestamps_ns[before_index]) / float(span_ns)
    return InterpolationBracket(before_index, after_index, ratio, span_ns), 'ready'


def latest_at_or_before_index(timestamps_ns: Sequence[int], target_ns: int) -> Optional[int]:
    """Return the newest causal sample index for a target timestamp."""

    if not timestamps_ns:
        return None
    index = bisect_right(timestamps_ns, target_ns) - 1
    return index if index >= 0 else None


def finite_vector(values: Iterable[float], length: int) -> Tuple[float, ...]:
    vector = tuple(float(value) for value in values)
    if len(vector) != length or not all(math.isfinite(value) for value in vector):
        raise ValueError('expected a finite vector of the requested length')
    return vector


def ned_to_enu(vector: Sequence[float]) -> Vector3:
    north, east, down = finite_vector(vector, 3)
    return east, north, -down


def frd_to_flu(vector: Sequence[float]) -> Vector3:
    forward, right, down = finite_vector(vector, 3)
    return forward, -right, -down


def quaternion_normalize(quaternion: Sequence[float]) -> Quaternion:
    w, x, y, z = finite_vector(quaternion, 4)  # type: ignore[misc]
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1.0e-12:
        raise ValueError('quaternion norm is zero')
    return w / norm, x / norm, y / norm, z / norm


def quaternion_slerp(first: Sequence[float], second: Sequence[float], ratio: float) -> Quaternion:
    """Shortest-path spherical interpolation of PX4-order quaternions."""

    if not math.isfinite(ratio) or ratio < 0.0 or ratio > 1.0:
        raise ValueError('interpolation ratio must be in [0, 1]')

    first_normalized = quaternion_normalize(first)
    second_normalized = quaternion_normalize(second)
    dot = sum(left * right for left, right in zip(first_normalized, second_normalized))
    if dot < 0.0:
        second_normalized = tuple(-value for value in second_normalized)  # type: ignore[assignment]
        dot = -dot
    dot = min(1.0, max(-1.0, dot))

    if dot > 0.9995:
        interpolated = tuple(
            left + ratio * (right - left)
            for left, right in zip(first_normalized, second_normalized)
        )
        return quaternion_normalize(interpolated)

    angle = math.acos(dot)
    sin_angle = math.sin(angle)
    first_weight = math.sin((1.0 - ratio) * angle) / sin_angle
    second_weight = math.sin(ratio * angle) / sin_angle
    return quaternion_normalize(tuple(
        first_weight * left + second_weight * right
        for left, right in zip(first_normalized, second_normalized)
    ))


def quaternion_to_matrix(quaternion: Sequence[float]) -> Matrix3:
    w, x, y, z = quaternion_normalize(quaternion)
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def matrix_multiply(left: Matrix3, right: Matrix3) -> Matrix3:
    return tuple(tuple(
        sum(left[row][index] * right[index][column] for index in range(3))
        for column in range(3)
    ) for row in range(3))  # type: ignore[return-value]


def matrix_transpose(matrix: Matrix3) -> Matrix3:
    return tuple(tuple(matrix[column][row] for column in range(3)) for row in range(3))  # type: ignore[return-value]


def matrix_vector_multiply(matrix: Matrix3, vector: Sequence[float]) -> Vector3:
    values = finite_vector(vector, 3)
    return tuple(
        sum(matrix[row][column] * values[column] for column in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def diagonal_covariance(variances: Sequence[float]) -> Matrix3:
    x, y, z = finite_vector(variances, 3)
    if x < 0.0 or y < 0.0 or z < 0.0:
        raise ValueError('variances must be non-negative')
    return (
        (x, 0.0, 0.0),
        (0.0, y, 0.0),
        (0.0, 0.0, z),
    )


def transform_covariance(covariance: Matrix3, transform: Matrix3) -> Matrix3:
    return matrix_multiply(matrix_multiply(transform, covariance), matrix_transpose(transform))


def matrix_to_quaternion(matrix: Matrix3) -> Quaternion:
    """Convert a proper rotation matrix into a normalized w, x, y, z quaternion."""

    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = (
            0.25 * scale,
            (matrix[2][1] - matrix[1][2]) / scale,
            (matrix[0][2] - matrix[2][0]) / scale,
            (matrix[1][0] - matrix[0][1]) / scale,
        )
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        quaternion = (
            (matrix[2][1] - matrix[1][2]) / scale,
            0.25 * scale,
            (matrix[0][1] + matrix[1][0]) / scale,
            (matrix[0][2] + matrix[2][0]) / scale,
        )
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        quaternion = (
            (matrix[0][2] - matrix[2][0]) / scale,
            (matrix[0][1] + matrix[1][0]) / scale,
            0.25 * scale,
            (matrix[1][2] + matrix[2][1]) / scale,
        )
    else:
        scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        quaternion = (
            (matrix[1][0] - matrix[0][1]) / scale,
            (matrix[0][2] + matrix[2][0]) / scale,
            (matrix[1][2] + matrix[2][1]) / scale,
            0.25 * scale,
        )
    return quaternion_normalize(quaternion)


def px4_attitude_ned_frd_to_enu_flu(quaternion: Sequence[float]) -> Quaternion:
    """Convert PX4 body-FRD-to-world-NED attitude into ROS FLU-to-ENU."""

    px4_rotation = quaternion_to_matrix(quaternion)
    return matrix_to_quaternion(matrix_multiply(
        matrix_multiply(NED_TO_ENU, px4_rotation),
        matrix_transpose(FRD_TO_FLU),
    ))


def ned_velocity_to_flu_body(
    velocity_ned: Sequence[float],
    quaternion_ned_frd: Sequence[float],
) -> Vector3:
    """Rotate a world-NED velocity into the vehicle's ROS FLU body frame."""

    body_frd_from_world_ned = matrix_transpose(quaternion_to_matrix(quaternion_ned_frd))
    velocity_frd = matrix_vector_multiply(body_frd_from_world_ned, velocity_ned)
    return frd_to_flu(velocity_frd)


def ned_velocity_covariance_to_flu_body(
    velocity_variance_ned: Sequence[float],
    quaternion_ned_frd: Sequence[float],
) -> Matrix3:
    """Rotate PX4's diagonal world-velocity variance into ROS body axes."""

    body_frd_from_world_ned = matrix_transpose(quaternion_to_matrix(quaternion_ned_frd))
    body_flu_from_world_ned = matrix_multiply(FRD_TO_FLU, body_frd_from_world_ned)
    return transform_covariance(diagonal_covariance(velocity_variance_ned), body_flu_from_world_ned)


def lerp_vector(first: Sequence[float], second: Sequence[float], ratio: float) -> Vector3:
    left = finite_vector(first, 3)
    right = finite_vector(second, 3)
    if not math.isfinite(ratio) or ratio < 0.0 or ratio > 1.0:
        raise ValueError('interpolation ratio must be in [0, 1]')
    return tuple(
        left_value + ratio * (right_value - left_value)
        for left_value, right_value in zip(left, right)
    )  # type: ignore[return-value]
