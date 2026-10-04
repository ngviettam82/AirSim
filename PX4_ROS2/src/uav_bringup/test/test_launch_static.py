"""Pin what sim.launch.py and real.launch.py promise, without starting anything.

WHY THIS EXISTS (P12.2, 2026-08-25). uav_bringup had no ctest target at all, while owning
both bringup files -- including real.launch.py, the one that will run on an aircraft. Every
check on it lived in a shell script somebody has to remember to run:
scripts/verify_real_launch_inert.sh actually launches processes and sleeps for about 45
seconds, so it is a gate, not something the suite runs on each build.

Most of what those files promise is decidable from the description alone. Building both
LaunchDescriptions in-process and evaluating conditions against a context seeded with the
declared defaults answers "which nodes would start" in about a tenth of a second, with no
ROS graph, no Gazebo and no processes. That is why these are static tests and why they can
afford to run every time.

WHAT THIS DOES NOT REPLACE. Nothing here proves a node comes up, talks, or behaves. The
live half stays in verify_real_launch_inert.sh, and it is still the thing that has watched
the guard engage for real. These tests pin the DECLARATION; that script pins the BEHAVIOUR.

The sharp cases are the last four. check_sim_real_parity.py pins the FLAGS -- safety=true,
blackbox=false and so on. These pin the CONSEQUENCE: which executables actually end up on
the aircraft. Two independent statements about the same decision, so a change that slips
past one still has to get past the other.
"""
import os
import sys

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

# UAV_BRINGUP_LAUNCH_DIR lets the positive control point these tests at a deliberately
# broken COPY of the launch files. Same reason check_sim_real_parity.py takes a launch_dir
# argument, and the same wording applies: a checker that has only ever been run on a
# passing tree has not been shown to fail. Mutating the real files and relying on a trap to
# undo them is one interrupted session away from leaving a broken real.launch.py behind.
LAUNCH_DIR = os.environ.get("UAV_BRINGUP_LAUNCH_DIR") or os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "launch")
sys.path.insert(0, LAUNCH_DIR)

# Measured 2026-08-25 with reviewed:=true and every other argument at its shipped default.
# This is the aircraft's node list. Changing it is changing what flies, so it takes an edit
# here as well as in the launch file.
AIRCRAFT_NODES = {
    "control_authority_manager_node",
    "diagnostics_node",
    "event_logger_node",
    "gps_adapter_node",
    "local_planner_node",
    "localization_health_node",
    "localization_mux_node",
    "navigator_action_server_node",
    "offboard_session_manager_node",
    "optical_flow_adapter_node",
    "px4_command_gateway_node",
    "px4_external_odometry_node",
    "px4_frame_bridge_node",
    "px4_state_adapter_node",
    "rangefinder_adapter_node",
    "route_planner_node",
    "safety_supervisor_node",
    "vio_adapter_node",
}


def load(name):
    return AnyLaunchDescriptionSource(
        os.path.join(LAUNCH_DIR, name)).get_launch_description(LaunchContext())


def _walk(entity, out):
    if isinstance(entity, Node):
        out.append(entity)
    for sub in (getattr(entity, "entities", None) or []):
        _walk(sub, out)


def node_actions(desc):
    found = []
    for e in desc.entities:
        _walk(e, found)
    return found


def declared_defaults(desc):
    out = {}
    for e in desc.entities:
        if isinstance(e, DeclareLaunchArgument):
            try:
                out[e.name] = perform_substitutions(LaunchContext(), e.default_value) \
                    if e.default_value is not None else ""
            except Exception:
                out[e.name] = ""
    return out


def seeded_context(desc, **overrides):
    """A context carrying the declared defaults, so conditions can be evaluated."""
    ctx = LaunchContext()
    values = declared_defaults(desc)
    values.update(overrides)
    for key, value in values.items():
        ctx.launch_configurations[key] = value
    return ctx


def executable(node, ctx):
    try:
        return perform_substitutions(ctx, node._Node__node_executable)
    except Exception:
        fallback = getattr(node, "node_executable", None)
        return fallback if isinstance(fallback, str) else "<unresolved>"


def would_start(desc, **overrides):
    """Executables whose condition holds under these arguments."""
    ctx = seeded_context(desc, **overrides)
    out = set()
    for node in node_actions(desc):
        condition = getattr(node, "condition", None)
        if condition is None or condition.evaluate(ctx):
            out.add(executable(node, ctx))
    return out


def parameters(node):
    out = {}
    for block in (getattr(node, "_Node__parameters", None) or []):
        if not isinstance(block, dict):
            continue
        for key in block:
            if isinstance(key, (tuple, list)):
                try:
                    out[perform_substitutions(LaunchContext(), list(key))] = True
                except Exception:
                    pass
            elif isinstance(key, str):
                out[key] = True
    return out


@pytest.fixture(scope="module")
def sim():
    return load("sim.launch.py")


@pytest.fixture(scope="module")
def real():
    return load("real.launch.py")


def test_a_bare_real_launch_starts_nothing(real):
    """P11.2's guard, checked in a tenth of a second instead of a 45-second script.

    real.launch.py inherits three decisions that cannot be closed from a simulator, and its
    whole defence is that a bare launch does nothing until someone types reviewed:=true.
    """
    assert would_start(real) == set(), (
        "real.launch.py would start nodes without reviewed:=true -- the acknowledgement "
        "guard for the three inherited decisions is gone (P11.2)")


def test_a_reviewed_real_launch_starts_exactly_the_signed_set(real):
    started = would_start(real, reviewed="true")
    assert started == AIRCRAFT_NODES, (
        "the aircraft's node list changed.\n  added: %s\n  removed: %s"
        % (sorted(started - AIRCRAFT_NODES), sorted(AIRCRAFT_NODES - started)))


def test_the_safety_supervisor_is_on_the_aircraft(real):
    """The mutation that used to pass every gate.

    On 2026-08-25 the parity checker was shown to print "identical node sets / parity
    PASSED" with the safety supervisor disabled, because it compared declared names and
    never read conditions. Parity now checks the FLAG; this checks the CONSEQUENCE.
    """
    assert "safety_supervisor_node" in would_start(real, reviewed="true")


def test_the_blackbox_stays_off_on_the_aircraft(real):
    """P10 D-4, carried verbatim. Worth revisiting before first flight -- see BLACKBOX_NOTE
    in real.launch.py -- but revisiting it must be a decision, not a drift."""
    assert "rosbag_manager_node" not in would_start(real, reviewed="true")


def test_perception_stays_off_on_the_aircraft(real):
    """P11.3: there is no real camera driver yet. Started here these nodes would publish
    nothing while diagnostics reported them enabled."""
    started = would_start(real, reviewed="true")
    for node in ("marker_detector_node", "obstacle_extractor_node", "camera_health_node"):
        assert node not in started, "%s needs the P11.3 camera driver" % node


def test_the_mission_layer_stays_off_on_the_aircraft(real):
    assert "mission_executor_node" not in would_start(real, reviewed="true")


def test_the_recorder_is_the_first_node_action(sim, real):
    """P10: the black box is declared before anything it might have to record.

    Ordering is not a guarantee of start order, but a recorder declared last cannot be
    first, and this is the half that is decidable from the file.
    """
    for name, desc in (("sim", sim), ("real", real)):
        first = executable(node_actions(desc)[0], seeded_context(desc))
        assert first == "rosbag_manager_node", (
            "%s.launch.py declares %s before the recorder" % (name, first))


def test_every_node_is_told_the_time_source(sim, real):
    """R7: one codebase, and use_sim_time is how a node learns which clock it is on. A node
    that never receives it silently follows the wall clock."""
    for name, desc in (("sim", sim), ("real", real)):
        missing = [executable(n, seeded_context(desc)) for n in node_actions(desc)
                   if "use_sim_time" not in parameters(n)]
        assert not missing, "%s.launch.py: nodes without use_sim_time: %s" % (name, missing)


def test_every_node_is_told_which_aircraft_it_is(sim, real):
    """R2: every topic lives under /uav/<id>/. A node with no uav_id cannot build its own
    names, and multi-drone stops being possible."""
    for name, desc in (("sim", sim), ("real", real)):
        missing = [executable(n, seeded_context(desc)) for n in node_actions(desc)
                   if "uav_id" not in parameters(n)]
        assert not missing, "%s.launch.py: nodes without uav_id: %s" % (name, missing)


def test_the_arguments_differ_by_exactly_the_two_signed_exceptions(sim, real):
    """R7 from the operator's side: the two bringups are driven the same way apart from
    two differences that were argued for and written down.

    An earlier draft of this test asserted the sets were equal and failed, correctly: the
    files are not meant to match here.

      * localization_params is SIM-ONLY. In simulation it points the localization stack at
        noise-injection configs; on an aircraft, being able to switch on fake sensors is a
        scenario that must not be constructible at all (debt #9).
      * require_obstacle_feed is SIM-ONLY for the same kind of reason (added by S13,
        2026-08-25). The sim needs to flip it to fly the branch the aircraft will take;
        the aircraft must not let anyone flip a signed safety decision from the command
        line, so real.launch.py keeps it as a fixed node parameter.
      * reviewed is REAL-ONLY. Simulation has no inherited decisions to acknowledge.

    Asserting the exact difference is stronger than asserting equality: it pins the shared
    knobs AND both exceptions, so a third one cannot appear without someone deciding to.
    """
    sim_args, real_args = set(declared_defaults(sim)), set(declared_defaults(real))
    assert sim_args - real_args == {"localization_params", "require_obstacle_feed"}, (
        "sim-only arguments changed: %s" % sorted(sim_args - real_args))
    assert real_args - sim_args == {"reviewed"}, (
        "real-only arguments changed: %s" % sorted(real_args - sim_args))
    assert len(sim_args & real_args) >= 12, (
        "only %d shared arguments -- the two bringups are drifting apart"
        % len(sim_args & real_args))


def test_both_files_declare_the_same_node_actions(sim, real):
    """The parity rule itself, so it also runs in the ordinary suite and not only in the
    gate. scripts/check_sim_real_parity.py stays the richer check (conditions, signed flag
    defaults, inherited parameters); this is the floor under it."""
    ctx_sim, ctx_real = seeded_context(sim), seeded_context(real)
    sim_names = {executable(n, ctx_sim) for n in node_actions(sim)}
    real_names = {executable(n, ctx_real) for n in node_actions(real)}
    assert sim_names == real_names, (
        "sim-only: %s\nreal-only: %s"
        % (sorted(sim_names - real_names), sorted(real_names - sim_names)))


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
