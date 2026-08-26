"""Pin the conventions every other package reads uav_interfaces through.

WHY THIS EXISTS (P12.2, 2026-08-25). uav_interfaces had no test at all -- understandable
for a package that is pure IDL, and wrong anyway: eleven packages build against these
definitions, and a field quietly added, renamed or dropped here reaches all of them. The
build catches a type that stops compiling. It does not catch a NEW message shipped without
a header, and that one is expensive: every staleness check in the project reads
header.stamp, so a headerless published message is one nothing downstream can age.

Drift had already happened by the time this was written. docs/package-status.md and
.claude/memory.md both recorded "17 msg + 9 srv + 9 action"; the tree held 21 msg. Nobody
had done anything wrong -- there was simply nothing that would notice.

THE RULE. Every message is exactly one of three things:
  (a) a PUBLISHED type -- leads with std_msgs/Header, and names its aircraft with uav_id;
  (b) an ELEMENT type -- appears as X[] inside a published message, and carries neither,
      because the enclosing message already stamped and named the whole batch;
  (c) a CONSTANT TABLE -- no fields at all, only shared values.

test_the_exception_list_polices_itself is the one that keeps this honest. An exception
list nobody re-derives becomes a place to put anything inconvenient, so each entry has to
prove it is still an element type or still a constant table.
"""
import os
import re
import sys

import pytest

# UAV_INTERFACES_DIR lets the positive control point these tests at a deliberately broken
# COPY. Same reason check_sim_real_parity.py takes a launch_dir: a checker that has only
# ever run on a passing tree has not been shown to fail, and mutating the real definitions
# in place is one interrupted session away from leaving a broken message in the tree.
PKG = os.environ.get("UAV_INTERFACES_DIR") or os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))
MSG_DIR = os.path.join(PKG, "msg")
SRV_DIR = os.path.join(PKG, "srv")
ACTION_DIR = os.path.join(PKG, "action")

# Recorded 2026-08-25 by counting the tree. Pinned so a definition cannot vanish in a
# refactor without someone saying so -- and so the count in the docs has one place to be
# checked against.
EXPECTED = {"msg": 21, "srv": 9, "action": 9}

# Messages that carry neither a header nor a uav_id, and why each is allowed to.
NOT_PUBLISHED = {
    "Obstacle": "element of ObstacleArray -- the array stamps and names the batch",
    "SemanticLandmark": "element of SemanticLandmarkArray",
    "TrajectoryPoint": "element of Trajectory3D",
    "ResultCode": "constants only -- the shared result table for all nine actions",
}


def definitions(directory, suffix):
    return sorted(f for f in os.listdir(directory) if f.endswith(suffix))


def fields(path):
    """Non-comment, non-blank, non-separator lines of a definition file."""
    out = []
    for line in open(path, encoding="utf-8"):
        line = line.split("#")[0].strip()
        if not line or line == "---":
            out.append("---") if line == "---" else None
            continue
        out.append(line)
    return out


def published_messages():
    for name in definitions(MSG_DIR, ".msg"):
        stem = name[:-4]
        if stem not in NOT_PUBLISHED:
            yield stem, os.path.join(MSG_DIR, name)


def test_the_interface_inventory_matches_the_recorded_counts():
    actual = {
        "msg": len(definitions(MSG_DIR, ".msg")),
        "srv": len(definitions(SRV_DIR, ".srv")),
        "action": len(definitions(ACTION_DIR, ".action")),
    }
    assert actual == EXPECTED, (
        "the interface inventory changed: %s (recorded %s). If this is intended, update "
        "EXPECTED here AND the counts in docs/package-status.md and .claude/memory.md -- "
        "they were four messages out of date on 2026-08-25 because nothing checked them."
        % (actual, EXPECTED))


@pytest.mark.parametrize("stem,path", list(published_messages()))
def test_every_published_message_leads_with_a_header(stem, path):
    body = [f for f in fields(path) if f != "---"]
    assert body, "%s.msg has no fields at all -- is it a constant table?" % stem
    assert body[0] == "std_msgs/Header header", (
        "%s.msg starts with %r. A published message leads with std_msgs/Header: every "
        "staleness and freshness check in this project reads header.stamp, so a message "
        "without one cannot be aged by anything downstream." % (stem, body[0]))


@pytest.mark.parametrize("stem,path", list(published_messages()))
def test_every_published_message_names_its_aircraft(stem, path):
    body = fields(path)
    assert "string uav_id" in body, (
        "%s.msg has no uav_id. R2 puts every topic under /uav/<id>/, and a sample that "
        "does not name its aircraft cannot be told apart once there is more than one."
        % stem)


def test_the_exception_list_polices_itself():
    """Each exception must still be an element type or a constant table.

    Without this, NOT_PUBLISHED is just a place to put a message that failed the rule.
    """
    all_text = ""
    for directory, suffix in ((MSG_DIR, ".msg"), (SRV_DIR, ".srv"), (ACTION_DIR, ".action")):
        for name in definitions(directory, suffix):
            all_text += open(os.path.join(directory, name), encoding="utf-8").read()

    for stem, why in NOT_PUBLISHED.items():
        path = os.path.join(MSG_DIR, stem + ".msg")
        assert os.path.isfile(path), "%s is exempted but does not exist" % stem
        body = [f for f in fields(path) if f != "---"]
        constants_only = all(re.match(r"^[a-z0-9_]+\s+[A-Z0-9_]+\s*=", f) for f in body)
        used_as_element = re.search(
            r"^(uav_interfaces/)?%s\[\]" % re.escape(stem), all_text, re.MULTILINE) is not None
        assert constants_only or used_as_element, (
            "%s is exempted as %r, but it is neither a constant table nor used as %s[] "
            "anywhere. Either it became a published message and needs a header, or it is "
            "dead and the exemption should go with it." % (stem, why, stem))


def test_every_exempted_message_really_lacks_a_header():
    """The other direction: an exemption that is no longer needed is stale.

    Same shape as the stale-entry check in check_px4_msgs_boundary.sh -- a permission
    nobody re-reads is a permission that outlives its reason.
    """
    for stem in NOT_PUBLISHED:
        body = fields(os.path.join(MSG_DIR, stem + ".msg"))
        assert "std_msgs/Header header" not in body, (
            "%s now has a header, so it is a published message. Remove it from "
            "NOT_PUBLISHED instead of keeping a permission it does not use." % stem)


@pytest.mark.parametrize("name", definitions(ACTION_DIR, ".action"))
def test_every_action_result_declares_result_code(name):
    """All nine actions answer with the same shared code table (ResultCode.msg)."""
    body = fields(os.path.join(ACTION_DIR, name))
    assert "uint8 result_code" in body, (
        "%s has no `uint8 result_code`. Nine actions share one result table so a caller "
        "can handle failure without knowing which action it called." % name)


def test_the_result_table_is_a_table():
    body = [f for f in fields(os.path.join(MSG_DIR, "ResultCode.msg")) if f != "---"]
    assert body, "ResultCode.msg is empty"
    for line in body:
        assert re.match(r"^uint8\s+[A-Z0-9_]+\s*=\s*\d+$", line), (
            "ResultCode.msg holds %r. It is the shared constant table for nine actions; a "
            "field here would give every action result a payload nobody asked for." % line)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
