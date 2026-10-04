"""Pin M5's safety verdict, and prove it can refuse.

WHY (S8, P12, 2026-08-25). smoke_flight.py is the regression flight this project has run
continuously since P3. It kept a list named `safety_violations` and printed
"violations : none" -- and the list was fed only by _on_offboard(), so both the name and
the line meant "the offboard link was healthy", never "the safety supervisor was quiet".
/uav/<id>/safety/state had never been read by the regression flight at all.

The first run that did read it showed the flight passing while the supervisor reported:

    worst=ERROR  codes=['OFFBOARD_UNHEALTHY', 'LOCALIZATION_JUMP']

so the criterion had to be chosen from what a healthy flight actually looks like, not
guessed. Both codes map to FailsafeAction::kReport -- the design says notice it, act on
nothing -- so failing on ERROR would fail clean flights and relitigate a signed decision
in the wrong place. The verdict is taken from recommended_action instead, which is where
the supervisor states what it wants done (contract 2.18).

These are unit tests on that decision, deliberately runnable without a simulator: a rule
that can only be exercised by flying is a rule nobody exercises. The class is instantiated
through object.__new__ because the verdict reads three plain fields and nothing else -- no
ROS node, no graph, no clock.

test_the_verdict_can_actually_refuse is the point of the file. Every gate this session
started from a checker that had never been seen refusing anything, and this one is new
enough to deserve the same suspicion.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "test"))
import smoke_flight                                                   # noqa: E402


def flight(samples=1, actions=None, codes=None):
    """A SmokeFlight carrying only the fields the verdict reads."""
    node = object.__new__(smoke_flight.SmokeFlight)
    node.safety_samples = samples
    node.safety_actions = list(actions or [])
    node.safety_codes = list(codes or [])
    node.safety_worst_level = 0
    return node


def test_a_quiet_supervisor_passes():
    assert flight(samples=100, actions=["none"]).safetyFailure() is None


def test_report_level_does_not_fail_the_flight():
    """The measured baseline: ERROR-level codes that map to kReport.

    Both of these were present on a clean three-flight run. They are informational by
    design, and a regression flight is not where that design gets revisited.
    """
    node = flight(samples=924, actions=["none", "report"],
                  codes=["OFFBOARD_UNHEALTHY", "LOCALIZATION_JUMP"])
    assert node.safetyFailure() is None


@pytest.mark.parametrize("action", [
    "hold",
    "inhibit",
    "would_hold (dry-run, enforcement_enabled=false)",
    "would_inhibit (dry-run, enforcement_enabled=false)",
])
def test_the_verdict_can_actually_refuse(action):
    """Each string describeAction() emits when the supervisor wants the aircraft.

    Contract 2.18 requires PREFIX comparison, because the dry-run forms carry a trailing
    explanation. Comparing whole strings would let both dry-run cases through -- and
    dry-run means enforcement was off, not that the hazard was absent.
    """
    node = flight(samples=50, actions=["none", action])
    reason = node.safetyFailure()
    assert reason is not None, "%r must fail a regression flight" % action
    assert "wanted the aircraft" in reason


def test_silence_fails_rather_than_reading_as_quiet():
    """O3: not measured is not OK.

    /safety/state is Reliable/TransientLocal. A subscriber with the wrong QoS receives
    nothing and no error is raised anywhere, so an empty record would otherwise look
    exactly like a flight where safety had nothing to say.
    """
    reason = flight(samples=0).safetyFailure()
    assert reason is not None
    assert "never arrived" in reason


def test_an_unexplained_code_is_named():
    """OFFBOARD_UNHEALTHY has a written reason; LOCALIZATION_JUMP does not, and is not
    quietly baselined into the known-good list to make a run green."""
    node = flight(samples=10, actions=["none"],
                  codes=["OFFBOARD_UNHEALTHY", "LOCALIZATION_JUMP"])
    assert node.safetyUnexplainedCodes() == ["LOCALIZATION_JUMP"]


def test_the_explained_list_is_not_a_dumping_ground():
    """Every entry must carry a reason someone can read and disagree with."""
    assert smoke_flight.SAFETY_EXPLAINED_CODES, "the list exists to hold reasons, not names"
    for code, why in smoke_flight.SAFETY_EXPLAINED_CODES.items():
        assert len(why) > 40, "%s is exempted with no real reason: %r" % (code, why)


def test_an_undocumented_code_blocks_the_run():
    """The rule LOCALIZATION_JUMP escaped for days: it printed a warning and passed.

    Measured 2026-08-25 on bag uav0_20260825_073235Z, the pose really did step 2.217 m in
    100 ms. Nothing about the flight was allowed to depend on someone noticing a line of
    output, so an unexplained code is now a blocker.
    """
    node = flight(samples=10, actions=["none"], codes=["SOMETHING_NOBODY_WROTE_DOWN"])
    blockers = node.safetyBlockers()
    assert blockers, "an undocumented safety code must stop a PASS"
    assert "SOMETHING_NOBODY_WROTE_DOWN" in blockers[0]


def test_a_documented_code_does_not_block_the_run():
    """The other side: the rule must not simply refuse every flight."""
    known = sorted(smoke_flight.SAFETY_EXPLAINED_CODES)[0]
    node = flight(samples=10, actions=["none"], codes=[known])
    assert node.safetyBlockers() == []


def test_an_acting_recommendation_still_blocks_the_run():
    """Adding the code rule must not have displaced the original one."""
    node = flight(samples=10, actions=["hold"], codes=[])
    blockers = node.safetyBlockers()
    assert blockers, "an acting recommendation must still stop a PASS"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
