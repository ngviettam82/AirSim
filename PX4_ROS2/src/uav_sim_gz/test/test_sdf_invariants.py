"""Pin the invariant that keeps a simulated obstacle honest: collision covers visual.

The rule and its reason are in CLAUDE.md section 5: rendering sensors read <visual>,
physics reads <collision>, so a visual larger than its collision makes the drone SEE a
wall and FLY THROUGH it -- the avoidance test passes in simulation and the real aircraft
hits something. Until P12.2 nothing checked it: tools/check_world_invariants.py takes a
world AND a manifest (it exists to accept the shelved Bach Khoa world), raises on <mesh>,
never resolves <include>, and compares geometry for EQUALITY instead of containment.

Two kinds of case below, and both are needed:

  * the shipped worlds must pass, so a real regression in the world files is caught;
  * synthetic worlds must FAIL in each way the rule claims to catch, so a future edit
    cannot quietly turn the rule into something that always says yes.

test_a_generous_collision_is_not_flagged is the one the previous tool got backwards. A
collision LARGER than its visual is the SAFE direction and is common when a rough box
wraps a detailed shape; flagging it teaches people to ignore the gate.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sdf_invariants                                             # noqa: E402

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

GROUND = """  <model name="ground"><link name="g">
    <collision name="c"><geometry><plane><normal>0 0 1</normal></plane></geometry></collision>
    <visual name="v"><geometry><plane><normal>0 0 1</normal></plane></geometry></visual>
  </link></model>"""


def build(tmp_path, body, with_ground=True):
    """Write a one-world package under tmp_path and return its directory."""
    (tmp_path / "worlds").mkdir(parents=True, exist_ok=True)
    (tmp_path / "models").mkdir(parents=True, exist_ok=True)
    parts = ['<?xml version="1.0"?><sdf version="1.9"><world name="t">']
    if with_ground:
        parts.append(GROUND)
    parts.append(body)
    parts.append("</world></sdf>")
    (tmp_path / "worlds" / "t.sdf").write_text("\n".join(parts))
    return str(tmp_path)


def problems(tmp_path, body, with_ground=True):
    return sdf_invariants.check_package(build(tmp_path, body, with_ground)).problems


HONEST_WALL = """  <model name="w"><link name="wall">
    <visual name="v"><geometry><box><size>4 0.2 3</size></box></geometry></visual>
    <collision name="c"><geometry><box><size>4 0.2 3</size></box></geometry></collision>
  </link></model>"""

LYING_WALL = """  <model name="w"><link name="wall">
    <visual name="v"><geometry><box><size>4 0.2 3</size></box></geometry></visual>
    <collision name="c"><geometry><box><size>4 0.2 0.4</size></box></geometry></collision>
  </link></model>"""

GHOST_WALL = """  <model name="w"><link name="wall">
    <visual name="v"><geometry><box><size>4 0.2 3</size></box></geometry></visual>
  </link></model>"""

FLAT_ROAD = """  <model name="r"><link name="road_1">
    <visual name="v"><pose>0 0 0.02 0 0 0</pose>
      <geometry><box><size>20 3 0.04</size></box></geometry></visual>
  </link></model>"""

RAISED_ROAD = """  <model name="r"><link name="road_1">
    <visual name="v"><pose>0 0 0.50 0 0 0</pose>
      <geometry><box><size>20 3 0.04</size></box></geometry></visual>
  </link></model>"""

GENEROUS_COLLISION = """  <model name="w"><link name="wall">
    <visual name="v">
      <geometry><cylinder><radius>0.3</radius><length>2</length></cylinder></geometry></visual>
    <collision name="c"><geometry><box><size>1 1 2.4</size></box></geometry></collision>
  </link></model>"""

UNREADABLE_MESH = """  <model name="w"><link name="statue">
    <visual name="v"><geometry><mesh><uri>model://elsewhere/thing.dae</uri></mesh></geometry></visual>
    <collision name="c"><geometry><box><size>1 1 1</size></box></geometry></collision>
  </link></model>"""

ROTATED_BOX = """  <model name="w"><link name="wall">
    <visual name="v"><pose>0 0 1.5 0 0 0.7854</pose>
      <geometry><box><size>4 0.2 3</size></box></geometry></visual>
    <collision name="c"><geometry><box><size>4 0.2 3</size></box></geometry></collision>
  </link></model>"""


def test_the_shipped_worlds_have_no_uncovered_visual():
    report = sdf_invariants.check_package(PACKAGE_DIR)
    assert report.problems == [], (
        "a shipped world has a visual its collision does not cover -- a rendering sensor "
        "would see it and physics would not (CLAUDE.md section 5)"
    )
    assert report.links > 0 and report.visuals > 0, "scanned nothing -- the run proves nothing"


# Every visual the scan lets off as an aircraft part must be one of these links.
# WHICH geometry is exempted is pinned; HOW MANY TIMES it is mounted is not, for the
# same reason the ground-decal count is not pinned -- carrying an already-blessed
# sensor on one more drone variant is normal model work and exempts no new geometry.
# A raw total was pinned until 2026-08-26 and it failed exactly the way a raw total
# fails: it went red for a benign variant, was bumped 11 -> 13 from reasoning about
# "the new sensor, twice" instead of from the tool, and the measured delta was +4.
BLESSED_AIRCRAFT_PARTS = {
    ("sensor_camera_down", "sensor_camera_down/base_link"),
    ("sensor_camera_front", "sensor_camera_front/base_link"),
    ("sensor_lidar_down", "sensor_lidar_down/base_link"),
    ("sensor_rgbd_front", "sensor_rgbd_front/base_link"),
}

# Same shape as BLESSED_AIRCRAFT_PARTS and for the same reason: the raw count went
# 6 -> 7 because uav0_full_rgbd is one more model including uav0_frame, which includes
# x500 -- no new geometry left the package. The count answers "how often", the gate
# needs "which". Only <include> URIs land here; a <mesh> URI such as OakD-Lite is
# judged on the aircraft-part path instead.
BLESSED_EXTERNAL_MODELS = {"x500"}

BOGUS_AIRCRAFT_PART = """<?xml version="1.0"?><sdf version="1.9">
  <model name="sensor_bogus"><link name="sensor_bogus/base_link">
    <visual name="v"><pose>0 0 1.5 0 0 0</pose>
      <geometry><box><size>1 1 3</size></box></geometry></visual>
  </link></model></sdf>"""


def test_the_structural_exemptions_have_not_drifted():
    """Aircraft parts and external models are structural; a change means someone edited
    the airframe or added an out-of-package dependency, and that should be looked at.
    The ground-decal count is deliberately NOT pinned: adding a road is normal world work.
    """
    report = sdf_invariants.check_package(PACKAGE_DIR)
    got = set(report.exempt_aircraft_parts)
    assert got == BLESSED_AIRCRAFT_PARTS, (
        "the set of aircraft parts granted the collision exemption changed -- an airframe "
        "model gained or lost a visual without collision. newly exempted: %s -- gone: %s"
        % (sorted(got - BLESSED_AIRCRAFT_PARTS), sorted(BLESSED_AIRCRAFT_PARTS - got))
    )
    assert report.exempt_aircraft >= len(BLESSED_AIRCRAFT_PARTS), (
        "scanned no aircraft geometry -- the run proves nothing"
    )
    outside = set(report.exempt_external_models)
    assert outside == BLESSED_EXTERNAL_MODELS, (
        "the set of model:// references leaving this package changed -- that geometry "
        "cannot be checked from here. newly outside: %s -- gone: %s"
        % (sorted(outside - BLESSED_EXTERNAL_MODELS),
           sorted(BLESSED_EXTERNAL_MODELS - outside))
    )


def test_an_aircraft_part_outside_the_blessed_set_is_reported(tmp_path):
    """Without this, a scan that exempted nothing at all would satisfy the pin equally well."""
    pkg = build(tmp_path, HONEST_WALL)
    part = tmp_path / "models" / "sensor_bogus"
    part.mkdir(parents=True)
    (part / "model.sdf").write_text(BOGUS_AIRCRAFT_PART)
    report = sdf_invariants.check_package(pkg)
    assert report.problems == [], "an aircraft part is exempted, not flagged -- that is the premise"
    assert set(report.exempt_aircraft_parts) - BLESSED_AIRCRAFT_PARTS == {
        ("sensor_bogus", "sensor_bogus/base_link")
    }, "a part outside the blessed set was exempted without the pin naming it"


def test_an_honest_wall_passes(tmp_path):
    assert problems(tmp_path, HONEST_WALL) == []


def test_a_visual_taller_than_its_collision_is_caught(tmp_path):
    assert problems(tmp_path, LYING_WALL), "the drone would see this wall and fly through it"


def test_a_wall_with_no_collision_at_all_is_caught(tmp_path):
    assert problems(tmp_path, GHOST_WALL)


def test_a_generous_collision_is_not_flagged(tmp_path):
    """The safe direction must stay silent. The previous tool compared for equality and
    called this a mismatch, which is how a gate earns the right to be ignored."""
    assert problems(tmp_path, GENEROUS_COLLISION) == []


def test_a_flat_road_over_a_ground_plane_is_exempt(tmp_path):
    assert problems(tmp_path, FLAT_ROAD) == []


def test_a_raised_road_is_caught(tmp_path):
    """Above GROUND_DECAL_TOP_M it has stopped being a marking on the ground."""
    assert problems(tmp_path, RAISED_ROAD)


def test_a_road_without_a_ground_plane_loses_the_exemption(tmp_path):
    """The excuse is the ground underneath. Remove it and the excuse goes with it."""
    assert problems(tmp_path, FLAT_ROAD, with_ground=False)


def test_an_unreadable_mesh_fails_rather_than_passes(tmp_path):
    """O3: not measured is not OK. An unknown must not land on the safe side."""
    assert problems(tmp_path, UNREADABLE_MESH)


def test_a_freely_rotated_box_is_reported_not_guessed(tmp_path):
    """An AABB of a rotated box is larger than the box. Using it for the COLLISION side
    would overestimate coverage -- the unsafe direction -- so it is refused instead."""
    assert problems(tmp_path, ROTATED_BOX)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
