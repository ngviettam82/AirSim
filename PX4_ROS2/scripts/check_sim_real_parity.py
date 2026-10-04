#!/usr/bin/env python3
"""R7 enforcement: sim.launch.py and real.launch.py must run THE SAME NODES.

R7 says the two bringups differ in CONFIG, not in which nodes exist. A node present in
sim and absent on the aircraft is the sim-to-real gap in its purest form: every gate
passes, and the thing that was protecting the aircraft is not running.

This does not compare the source text. It builds both LaunchDescriptions and collects
the executables they actually produce, so it also catches a node that is present in the
file but wired under a condition that can never be true.

It also checks the differences that are supposed to exist, because "same nodes" is not
the only promise:

  * real must NOT expose a localization_params argument. In sim that argument points the
    localization stack at noise-injection configs; on an aircraft, being able to enable
    simulated sensors is a scenario that must not be constructible (debt #9).
  * real must NOT set FASTRTPS_DEFAULT_PROFILES_FILE. The large-samples DDS profile is
    sim-only until P11.3 measures it on target hardware.
  * real's use_sim_time must default to false. There is no /clock on an aircraft.
  * real must default `reviewed` to false, so a bare launch starts nothing.

Usage: python3 scripts/check_sim_real_parity.py [launch_dir]
       The optional launch_dir exists so the positive control can point this at a
       deliberately-broken copy of the tree. A checker that has only ever been run on a
       passing tree has not been shown to fail.
Docs:  .claude/plan/P11-real-flight-readiness.md
"""
import os
import sys

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

LAUNCH_DIR = None  # filled in main()


def load(path):
    src = AnyLaunchDescriptionSource(path)
    return src.get_launch_description(LaunchContext())


def walk(entity, ctx, out):
    """Collect Node actions from a description, entering nested entities."""
    if isinstance(entity, Node):
        out.append(entity)
    for sub in (getattr(entity, "entities", None) or []):
        walk(sub, ctx, out)


def condition_is_constant_false(node):
    """True only when this node can NEVER start, whatever the arguments are.

    The docstring above has promised since P11.2 that a node "wired under a condition
    that can never be true" is caught. It was not: walk() collected every Node action
    and never looked at .condition, so `condition=IfCondition('false')` on the safety
    supervisor left the report saying "identical node sets / parity PASSED" (P12.0).

    A condition that reads a LaunchConfiguration cannot be settled here and is not the
    target -- that is the normal, intended shape in real.launch.py, and those flags are
    checked separately by signed_real_defaults(). Only a condition that resolves with an
    EMPTY context is constant, and a constant false one is a node that is present in the
    file purely for appearances.
    """
    cond = getattr(node, "condition", None)
    if cond is None:
        return False
    try:
        return not cond.evaluate(LaunchContext())
    except Exception:
        return False


def node_names(desc):
    return {executable_name(n) for n in node_actions(desc)}


def declared_args(desc):
    ctx = LaunchContext()
    out = {}
    for e in desc.entities:
        if isinstance(e, DeclareLaunchArgument):
            try:
                out[e.name] = perform_substitutions(ctx, e.default_value) \
                    if e.default_value is not None else None
            except Exception:
                out[e.name] = None
    return out


def sets_dds_profile(desc):
    ctx = LaunchContext()
    for e in desc.entities:
        if isinstance(e, SetEnvironmentVariable):
            try:
                if perform_substitutions(ctx, e.name) == "FASTRTPS_DEFAULT_PROFILES_FILE":
                    return True
            except Exception:
                return True
    return False


# WHY THIS TABLE EXISTS (P12.0, 2026-08-25). R7 parity compares the DECLARED node set,
# and real.launch.py deliberately keeps every node in the description and gates it with
# a flag -- its own module docstring says it does this "so the parity gate can do its
# job". The consequence nobody had drawn: THE FLAGS ARE THE AIRCRAFT CONFIGURATION, and
# until now nothing checked them. Flipping `safety` to 'false' left this gate printing
# "identical node sets / parity PASSED" while the aircraft flew with no safety
# supervisor.
#
# Every value here was signed by a named phase. Changing one now takes an edit in two
# files, which is the point: a signed safety decision must not be a one-line silent
# change. An argument that is NOT in this table also fails -- a new flag governing what
# runs on the aircraft is a decision, and undeclared decisions are how defaults drift.
SIGNED_REAL_DEFAULTS = {
    "uav_id": ("uav0", "memory.md S2, fixed 2026-08-24"),
    "reviewed": ("false", "P11.2: a bare launch must start nothing"),
    "use_sim_time": ("false", "there is no /clock on an aircraft"),
    "perception": ("false", "P11.3: no real camera driver exists yet"),
    "navigation": ("true", "the planner stack flies the aircraft"),
    "navigator": ("true", "action gateway the mission layer calls"),
    "control_authority": ("true", "P7: single writer on command_selected"),
    "safety": ("true", "P8: the safety supervisor runs on the aircraft"),
    "safety_enforcement": ("true", "P8.4/P8.5, preflight checklist D2"),
    "mission": ("false", "first flight is not a mission flight"),
    "blackbox": ("false", "P10 D-4 verbatim, preflight checklist D3"),
    "diagnostics": ("true", "go/no-go light must exist before flight"),
    "event_log": ("true", "the unified timeline is the post-flight record"),
}

# Decisions that live as node PARAMETERS rather than launch arguments, so the table
# above cannot see them. require_obstacle_feed is the sharpest one: with it false the
# local planner softens a Hold into a Clear when the obstacle map is stale
# (local_planner_node.cpp, "flying unguarded"). Sim runs false; the aircraft must not.
SIGNED_REAL_PARAMS = {
    "require_obstacle_feed": (True, "P6 Decision 4, preflight checklist D1"),
}


def executable_name(node):
    ctx = LaunchContext()
    try:
        return perform_substitutions(ctx, node._Node__node_executable)
    except Exception:
        exe = getattr(node, "node_executable", None)
        return exe if isinstance(exe, str) else "<unresolved>"


def node_actions(desc):
    ctx = LaunchContext()
    found = []
    for e in desc.entities:
        walk(e, ctx, found)
    return found


def check_signed_defaults(real_args):
    fails = []
    for name, (want, why) in sorted(SIGNED_REAL_DEFAULTS.items()):
        got = real_args.get(name, "<absent>")
        if got != want:
            print("  🔴 %s defaults to %r, signed value is %r (%s)" % (name, got, want, why))
            fails.append("real default %s is %r, signed %r" % (name, got, want))
        else:
            print("  ok  %-20s = %-6s (%s)" % (name, want, why))
    undeclared = sorted(set(real_args) - set(SIGNED_REAL_DEFAULTS))
    for name in undeclared:
        print("  🔴 %s is a real.launch.py argument nobody signed" % name)
        fails.append("undeclared real argument %s -- add it to SIGNED_REAL_DEFAULTS" % name)
    return fails


def resolve_param_part(part):
    """Undo launch_ros normalisation far enough to compare a name or a literal.

    Node.__init__ rewrites {'require_obstacle_feed': True} into a dict whose keys and
    values are TUPLES OF SUBSTITUTION OBJECTS. Reading it as a plain dict finds nothing,
    which is how the first version of this check reported "never set" for a parameter
    that is set on eight nodes. Anything that needs a launch argument to resolve is not
    a signed constant and is left alone.
    """
    if isinstance(part, (tuple, list)):
        try:
            return perform_substitutions(LaunchContext(), list(part))
        except Exception:
            return None
    return part


def as_flag(value):
    """Normalise True / 'True' / 'true' to one comparable form."""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, str):
        return value.strip().lower()
    return value


def check_signed_params(real):
    """Read the inherited decisions straight off the Node actions that carry them."""
    fails = []
    seen = {}
    for node in node_actions(real):
        for block in (getattr(node, "_Node__parameters", None) or []):
            if isinstance(block, dict):
                for key, value in block.items():
                    name = resolve_param_part(key)
                    if name in SIGNED_REAL_PARAMS:
                        seen.setdefault(name, []).append(resolve_param_part(value))
    for name, (want, why) in sorted(SIGNED_REAL_PARAMS.items()):
        values = seen.get(name)
        if not values:
            print("  🔴 %s is not set anywhere in real.launch.py (%s)" % (name, why))
            fails.append("real.launch.py never sets %s" % name)
            continue
        bad = [v for v in values if as_flag(v) != as_flag(want)]
        if bad:
            print("  🔴 %s is %r on real, signed value is %r (%s)" % (name, bad[0], want, why))
            fails.append("real %s is %r, signed %r" % (name, bad[0], want))
        else:
            print("  ok  %-20s = %-6s on %d node(s) (%s)" % (name, want, len(values), why))
    return fails


def main():
    global LAUNCH_DIR
    if len(sys.argv) > 1:
        LAUNCH_DIR = os.path.abspath(sys.argv[1])
    else:
        repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        LAUNCH_DIR = os.path.join(repo, "src", "uav_bringup", "launch")
    sim_path = os.path.join(LAUNCH_DIR, "sim.launch.py")
    real_path = os.path.join(LAUNCH_DIR, "real.launch.py")
    for p in (sim_path, real_path):
        if not os.path.isfile(p):
            print("FATAL: missing %s" % p)
            return 2

    # real.launch.py adds its own directory to sys.path; do the same here so both
    # descriptions load the same way they will at run time.
    sys.path.insert(0, LAUNCH_DIR)

    sim, real = load(sim_path), load(real_path)
    sim_nodes, real_nodes = node_names(sim), node_names(real)

    fails = []
    print("=== node parity (R7) ===")
    print("  sim:  %d nodes" % len(sim_nodes))
    print("  real: %d nodes" % len(real_nodes))
    only_sim = sorted(sim_nodes - real_nodes)
    only_real = sorted(real_nodes - sim_nodes)

    # The one legitimate absence. uav_sim_gz IS the simulator (CLAUDE.md S:4.11); it has
    # no aircraft counterpart. It contributes no Node to sim.launch.py today, so this
    # allowance exists for the day someone wires one in.
    ALLOWED_SIM_ONLY = set()

    for n in only_sim:
        if n in ALLOWED_SIM_ONLY:
            print("  sim-only (declared): %s" % n)
        else:
            print("  🔴 MISSING ON THE AIRCRAFT: %s" % n)
            fails.append("node %s runs in sim but not on real" % n)
    for n in only_real:
        print("  🔴 REAL-ONLY (undeclared): %s" % n)
        fails.append("node %s runs on real but not in sim -- never exercised by any gate" % n)
    if not only_sim and not only_real:
        print("  identical node sets")

    print()
    print("=== the differences that MUST exist ===")
    sim_args, real_args = declared_args(sim), declared_args(real)

    # require_obstacle_feed is sim-only for the same reason localization_params is: on an
    # aircraft, being able to switch a signed safety decision off from the command line is
    # a scenario that must not be constructible (P6 D4). real.launch.py sets it as a node
    # parameter, checked by SIGNED_REAL_PARAMS above.
    if "require_obstacle_feed" in real_args:
        print("  🔴 real exposes require_obstacle_feed -- D1 becomes flippable from the CLI")
        fails.append("real.launch.py exposes require_obstacle_feed (P6 D4 must stay fixed)")
    else:
        print("  ok  real does not expose require_obstacle_feed (P6 D4 stays fixed)")

    if "localization_params" in real_args:
        print("  🔴 real exposes localization_params -- noise injection is constructible")
        fails.append("real.launch.py exposes localization_params (debt #9)")
    else:
        print("  ok  real does not expose localization_params (debt #9)")

    if sets_dds_profile(real):
        print("  🔴 real sets FASTRTPS_DEFAULT_PROFILES_FILE -- the profile is sim-only")
        fails.append("real.launch.py loads the large-samples DDS profile")
    else:
        print("  ok  real does not load the sim-only DDS profile")

    ust = real_args.get("use_sim_time")
    if ust != "false":
        print("  🔴 real use_sim_time defaults to %r, expected 'false'" % ust)
        fails.append("real use_sim_time default is %r" % ust)
    else:
        print("  ok  real use_sim_time defaults to false")

    rev = real_args.get("reviewed")
    if rev != "false":
        print("  🔴 real 'reviewed' defaults to %r -- a bare launch would fly unreviewed" % rev)
        fails.append("real 'reviewed' default is %r" % rev)
    else:
        print("  ok  real refuses to start without reviewed:=true")

    print()
    print("=== nodes that can never start (R7, constant-false condition) ===")
    dead = sorted(
        {executable_name(n) for n in node_actions(real) if condition_is_constant_false(n)}
    )
    for n in dead:
        print("  🔴 %s is in real.launch.py but can never start" % n)
        fails.append("node %s is wired under a constant-false condition on real" % n)
    if not dead:
        print("  none")

    print()
    print("=== the flags that decide what runs on the aircraft ===")
    fails.extend(check_signed_defaults(real_args))

    print()
    print("=== inherited decisions carried as node parameters ===")
    fails.extend(check_signed_params(real))

    print()
    if fails:
        for f in fails:
            print("  FAIL: " + f)
        print()
        print("RESULT: sim/real parity FAILED")
        return 1
    print("RESULT: sim/real parity PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
