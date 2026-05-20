#!/usr/bin/env python3
"""Validate CosysAirSim equirectangular projection math without Unreal runtime."""

import math


FACES = (
    ("+X", (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, -1.0)),
    ("-X", (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
    ("+Y", (0.0, 1.0, 0.0), (0.0, 0.0, -1.0), (1.0, 0.0, 0.0)),
    ("-Y", (0.0, -1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)),
    ("+Z", (0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0)),
    ("-Z", (0.0, 0.0, -1.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0)),
)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def normalize(v):
    length = math.sqrt(dot(v, v))
    return tuple(x / length for x in v)


def direction_to_cube_sample(direction, cube_size):
    face_index = max(range(6), key=lambda index: dot(direction, FACES[index][1]))
    _, face_dir, face_up, face_right = FACES[face_index]
    denom = dot(direction, face_dir)
    s = dot(direction, face_right) / denom
    t = dot(direction, face_up) / denom
    return face_index, ((s + 1.0) * 0.5) * cube_size - 0.5, ((1.0 - t) * 0.5) * cube_size - 0.5


def equirectangular_pixel_to_direction(x, y, width, height):
    u = (x + 0.5) / width
    v = (y + 0.5) / height
    yaw = (u - 0.5) * 2.0 * math.pi
    pitch = (0.5 - v) * math.pi
    cos_pitch = math.cos(pitch)
    return (cos_pitch * math.cos(yaw), cos_pitch * math.sin(yaw), math.sin(pitch))


def face_texel_to_direction(face_index, x, y, cube_size):
    _, face_dir, face_up, face_right = FACES[face_index]
    u = (x + 0.5) / cube_size
    v = (y + 0.5) / cube_size
    s = u * 2.0 - 1.0
    t = 1.0 - v * 2.0
    return normalize(tuple(face_dir[i] + face_right[i] * s + face_up[i] * t for i in range(3)))


def assert_face(direction, expected_name, cube_size=64):
    face_index, px, py = direction_to_cube_sample(normalize(direction), cube_size)
    actual_name = FACES[face_index][0]
    assert actual_name == expected_name, f"{direction} mapped to {actual_name}, expected {expected_name}"
    center = (cube_size - 1) * 0.5
    assert abs(px - center) < 1e-5, (actual_name, px, center)
    assert abs(py - center) < 1e-5, (actual_name, py, center)


def validate_axis_centers():
    assert_face((1, 0, 0), "+X")
    assert_face((0, 1, 0), "+Y")
    assert_face((-1, 0, 0), "-X")
    assert_face((0, -1, 0), "-Y")
    assert_face((0, 0, 1), "+Z")
    assert_face((0, 0, -1), "-Z")


def validate_equirectangular_locations(cube_size=128):
    width = cube_size * 2
    height = cube_size
    locations = {
        "front": (width // 2, height // 2, "+X"),
        "right": (width * 3 // 4, height // 2, "+Y"),
        "left": (width // 4, height // 2, "-Y"),
        "top": (width // 2, 0, "+Z"),
        "bottom": (width // 2, height - 1, "-Z"),
    }
    for name, (x, y, expected_face) in locations.items():
        face_index, _, _ = direction_to_cube_sample(equirectangular_pixel_to_direction(x, y, width, height), cube_size)
        assert FACES[face_index][0] == expected_face, f"{name} mapped to {FACES[face_index][0]}"

    left_dir = equirectangular_pixel_to_direction(0, height // 2, width, height)
    right_dir = equirectangular_pixel_to_direction(width - 1, height // 2, width, height)
    assert dot(left_dir, right_dir) > 0.999, "wrap columns are not sampling the same back direction"


def validate_neighbor_edge_mapping(cube_size=64):
    # UE's +X face uses +Y as image-up and -Z as image-right.
    face_index, px, py = direction_to_cube_sample(face_texel_to_direction(0, cube_size, cube_size // 2, cube_size), cube_size)
    assert FACES[face_index][0] == "-Z", (FACES[face_index][0], px, py)

    # A tap one texel above +X should reproject onto +Y, avoiding a clamp seam at the face boundary.
    face_index, px, py = direction_to_cube_sample(face_texel_to_direction(0, cube_size // 2, -1, cube_size), cube_size)
    assert FACES[face_index][0] == "+Y", (FACES[face_index][0], px, py)

    # UE source derives right = up ^ dir. These checks catch top/bottom face handedness regressions.
    face_index, px, py = direction_to_cube_sample(face_texel_to_direction(4, cube_size, cube_size // 2, cube_size), cube_size)
    assert FACES[face_index][0] == "+X", (FACES[face_index][0], px, py)

    face_index, px, py = direction_to_cube_sample(face_texel_to_direction(4, -1, cube_size // 2, cube_size), cube_size)
    assert FACES[face_index][0] == "-X", (FACES[face_index][0], px, py)

    face_index, px, py = direction_to_cube_sample(face_texel_to_direction(5, cube_size, cube_size // 2, cube_size), cube_size)
    assert FACES[face_index][0] == "-X", (FACES[face_index][0], px, py)

    face_index, px, py = direction_to_cube_sample(face_texel_to_direction(5, -1, cube_size // 2, cube_size), cube_size)
    assert FACES[face_index][0] == "+X", (FACES[face_index][0], px, py)


def main():
    validate_axis_centers()
    validate_equirectangular_locations()
    validate_neighbor_edge_mapping()
    print("Equirectangular projection math validation passed.")


if __name__ == "__main__":
    main()
