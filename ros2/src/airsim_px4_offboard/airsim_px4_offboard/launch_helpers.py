"""Pure launch-configuration helpers for PX4 vehicle pipelines."""

from __future__ import annotations

import re
from typing import List, Sequence, Tuple


_PX4_VEHICLE_STATUS_SUFFIX = re.compile(r'(?:_v[1-9][0-9]*)?')


def parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in ('1', 'true', 'yes', 'on'):
        return True
    if normalized in ('0', 'false', 'no', 'off'):
        return False
    raise ValueError(f"expected a boolean value, got '{value}'")


def join_topic_prefix(prefix: str, suffix: str) -> str:
    cleaned_prefix = prefix.rstrip('/')
    cleaned_suffix = suffix.strip('/')
    if not cleaned_prefix:
        raise ValueError('topic prefix must not be empty')
    return f'{cleaned_prefix}/{cleaned_suffix}'


def resolve_px4_vehicle_status_topic(
    px4_topic_prefix: str,
    configured_topic: str,
    configured_suffix: str = '_v1',
) -> str:
    """Resolve the PX4 VehicleStatus topic without guessing its message layout.

    A full topic override is useful for custom DDS configurations. Otherwise,
    PX4's versioned-topic suffix is appended to the standard VehicleStatus
    endpoint. The current bridge default is ``_v1``.
    """

    override = configured_topic.strip()
    if override:
        return override
    suffix = configured_suffix.strip()
    if not _PX4_VEHICLE_STATUS_SUFFIX.fullmatch(suffix):
        raise ValueError(
            "px4_vehicle_status_suffix must be empty or a PX4 version suffix such as '_v1'"
        )
    return join_topic_prefix(px4_topic_prefix, 'out/vehicle_status' + suffix)


def camera_info_topic(image_topic: str) -> str:
    """Return the CameraInfo topic paired with an AirSim image transport topic."""

    cleaned = image_topic.rstrip('/')
    absolute = cleaned.startswith('/')
    tokens = [token for token in cleaned.split('/') if token]
    image_indices = [index for index, token in enumerate(tokens) if token == 'image']
    if not image_indices:
        raise ValueError(
            'camera_topic must contain an image path component or camera_info_topic must be supplied'
        )
    prefix_tokens = tokens[:image_indices[-1]]
    resolved = '/'.join(prefix_tokens + ['camera_info'])
    return '/' + resolved if absolute else resolved


def camera_vehicle_name(image_topic: str) -> str:
    """Extract the AirSim vehicle component from a wrapper image topic."""

    tokens = [token for token in image_topic.rstrip('/').split('/') if token]
    image_indices = [index for index, token in enumerate(tokens) if token == 'image']
    if not image_indices or image_indices[-1] < 2:
        raise ValueError(
            'camera_topic must use the AirSim <vehicle>/<camera>_<type>/image layout'
        )
    return tokens[image_indices[-1] - 2]


def validate_camera_vehicle(image_topic: str, expected_vehicle_name: str) -> None:
    expected = expected_vehicle_name.strip('/')
    if not expected:
        raise ValueError('airsim_vehicle_name must not be empty')
    actual = camera_vehicle_name(image_topic)
    if actual != expected:
        raise ValueError(
            f"camera topic '{image_topic}' belongs to AirSim vehicle '{actual}', "
            f"not configured vehicle '{expected}'"
        )


def camera_pipelines(
    camera_topics_value: str,
    output_prefixes_value: str,
    *,
    reserve_gated_prefixes: bool = False,
) -> List[Tuple[str, str]]:
    """Parse semicolon-separated camera topics and unambiguous output prefixes.

    A frame gate derived from ``front`` publishes below ``front_gated``.  When
    gates are requested, reject an explicitly configured ``front_gated``
    synchronizer too: otherwise two different cameras could publish the same
    synchronization topic and authorize the wrong controller input.
    """

    topics = [item.strip() for item in camera_topics_value.split(';') if item.strip()]
    prefixes = [item.strip() for item in output_prefixes_value.split(';') if item.strip()]
    if not topics:
        if prefixes:
            raise ValueError('camera_output_prefixes was supplied without camera_topics')
        return []
    if not prefixes:
        prefixes = ['camera_sync'] if len(topics) == 1 else [
            f'camera_{index}_sync' for index in range(len(topics))
        ]
    if len(prefixes) != len(topics):
        raise ValueError('camera_output_prefixes must have one entry per camera_topics entry')
    normalized_prefixes = [prefix.rstrip('/') for prefix in prefixes]
    if any(not prefix.strip('/') for prefix in normalized_prefixes):
        raise ValueError('camera output prefixes must not be empty')
    if any(prefix.startswith('/') for prefix in normalized_prefixes):
        raise ValueError(
            'camera_output_prefixes must be relative to node_namespace; absolute prefixes '
            'can alias another vehicle pipeline'
        )
    if len(set(normalized_prefixes)) != len(normalized_prefixes):
        raise ValueError(
            'camera_output_prefixes entries must be unique after trailing-slash normalization'
        )
    if reserve_gated_prefixes:
        source_sync_topics = {
            join_topic_prefix(prefix, 'image_sync')
            for prefix in normalized_prefixes
        }
        gated_sync_topics = {
            join_topic_prefix(prefix + '_gated', 'image_sync')
            for prefix in normalized_prefixes
        }
        collisions = sorted(source_sync_topics & gated_sync_topics)
        if collisions:
            raise ValueError(
                'camera_output_prefixes collides with a generated frame-gate prefix: '
                + ', '.join(collisions)
            )
    return list(zip(topics, normalized_prefixes))


def validate_live_control_topology(
    *,
    start_rate_control: bool,
    require_image_sync: bool,
    start_frame_gates: bool,
) -> None:
    """Keep every camera-authorized PX4 command behind a complete frame gate.

    A synchronizer event by itself proves source-time association, but it does
    not prove that the matching image payload and calibration reached the
    algorithm.  The default live-control topology must therefore not allow a
    rate controller that requires image synchronization to bypass its gate.
    """

    if start_rate_control and require_image_sync and not start_frame_gates:
        raise ValueError(
            'start_frame_gates must be true when start_rate_control and '
            'require_image_sync are true; disable rate control for an '
            'observation-only pipeline or explicitly disable image-synchronized control'
        )


def _primary_pipeline(
    pipelines: Sequence[Tuple[str, str]],
    primary_camera_index_value: str,
) -> Tuple[str, str]:

    if not pipelines:
        raise ValueError('image synchronization requires at least one camera topic')
    try:
        index = int(primary_camera_index_value)
    except ValueError as error:
        raise ValueError('primary_camera_index must be an integer') from error
    if index < 0 or index >= len(pipelines):
        raise ValueError('primary_camera_index is outside the configured camera list')
    return pipelines[index]


def primary_camera_topic(
    pipelines: Sequence[Tuple[str, str]],
    primary_camera_index_value: str,
) -> str:
    """Return the logical AirSim image topic for the selected primary camera."""

    return _primary_pipeline(pipelines, primary_camera_index_value)[0].rstrip('/')


def primary_sync_topic(
    pipelines: Sequence[Tuple[str, str]],
    primary_camera_index_value: str,
) -> str:
    _, output_prefix = _primary_pipeline(pipelines, primary_camera_index_value)
    return join_topic_prefix(output_prefix, 'image_sync')


def primary_gated_sync_topic(
    pipelines: Sequence[Tuple[str, str]],
    primary_camera_index_value: str,
) -> str:
    """Return the frame-gated synchronization topic for the primary camera."""

    source_topic = primary_sync_topic(pipelines, primary_camera_index_value)
    suffix = '/image_sync'
    return source_topic[:-len(suffix)] + '_gated' + suffix


def validate_frame_gate_transport(
    *,
    image_response_compress: bool,
    start_frame_gates: bool,
    frame_gate_image_transport: str,
) -> None:
    """Reject a direct-JPEG wrapper paired with a non-existent raw gate input."""

    if not start_frame_gates:
        return
    transport = frame_gate_image_transport.strip().lower()
    if transport not in ('raw', 'compressed'):
        raise ValueError("frame_gate_image_transport must be 'raw' or 'compressed'")
    if image_response_compress and transport == 'raw':
        raise ValueError(
            'image_response_compress:=true publishes direct JPEG only; use '
            'frame_gate_image_transport:=compressed or disable direct JPEG output'
        )


def camera_node_name(index: int, topic: str) -> str:
    final_token = topic.rstrip('/').split('/')[-1] or f'camera_{index}'
    sanitized = re.sub(r'[^A-Za-z0-9_]', '_', final_token).strip('_')
    if not sanitized:
        sanitized = f'camera_{index}'
    return f'camera_sync_{index}_{sanitized}'[:120]
