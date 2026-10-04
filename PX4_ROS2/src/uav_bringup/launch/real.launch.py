"""Bring up the autonomy stack on the REAL aircraft. Same nodes as sim, different config (R7).

🔴 THIS FILE HAS NEVER RUN ON AN AIRCRAFT. No hardware exists in this project yet. Every
node list below is verified by the static parity gate (scripts/check_sim_real_parity.sh)
against sim.launch.py; nothing below has been verified in flight, and the sim numbers do
not transfer -- that is the whole reason P11 exists.

WHY IT REFUSES TO START BY DEFAULT
-----------------------------------
`reviewed:=true` is required or this file starts NOTHING and says why. That is not
ceremony. P6, P8 and P10 each left an explicit written instruction addressed to this
file, and each one is a decision that cannot be closed from the simulator:

  1. P6 Decision 4  -- `require_obstacle_feed` must be TRUE here (sim runs it false).
                       Honoured below.
  2. P8.4/P8.5      -- `safety_enforcement` "must review this default independently
                       before first real flight -- do not assume sim's default is right
                       there."
  3. P10 D-4        -- "real.launch.py flips `blackbox` false until P11 review."

A default is a decision someone will inherit without reading. Making the file inert until
a human passes `reviewed:=true` is the only way to guarantee the three above are read by
the person who will fly it. Introspection still works with the flag off (the nodes are in
the description, merely conditioned), so the parity gate can do its job.

WHAT IS DELIBERATELY ABSENT, AND WHY
-------------------------------------
* **No `localization_params` launch argument.** In sim that argument exists to point the
  localization stack at noise-injection configs. Debt #9: "Tham số tiêm nhiễu MẶC ĐỊNH
  TẮT, `real.launch.py` không được phơi ra -- bay thật mà bật cảm biến giả là kịch bản
  không được phép tồn tại." Not exposing it is what makes that scenario unconstructible
  rather than merely discouraged.
* **No `dds_profile_actions()`.** The large-samples Fast DDS profile is sim-only until
  P11.3 measures it on the target hardware (owner, 2026-08-25).
  `scripts/check_dds_profile_sim_only.sh` fails the build if that ever changes here.
* **No `use_sim_time` default of true.** There is no /clock on an aircraft.
"""

import os
import sys

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ros2 launch executes this file without putting its own directory on sys.path, so the
# sibling module is not importable by default. __file__ is the only reliable anchor:
# the installed copy lives under share/uav_bringup/launch/ alongside node_manifest.py.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from node_manifest import (  # noqa: E402  (import must follow the sys.path fix)
    BACKEND_NODES,
    LOCALIZATION_NODES,
    MISSION_NODES,
    NAVIGATION_NODES,
    NAVIGATOR_NODE,
    PERCEPTION_NODES,
)

UNREVIEWED = (
    'real.launch.py did NOT start anything: pass reviewed:=true to confirm you have read '
    'the three decisions this file inherited -- require_obstacle_feed (P6 D4, set true '
    'here), safety_enforcement (P8 asked for an independent review before first flight), '
    'and blackbox (P10 D-4 asked for it to default false here). See the module docstring.'
)

BLACKBOX_NOTE = (
    'NOTE on blackbox default=false: this follows P10 D-4 verbatim. It is worth '
    'challenging before first flight -- a failed real flight cannot be re-run, which is '
    'the argument the evidence layer was built on, and O1 already guarantees the recorder '
    'cannot take the aircraft down. What is genuinely unknown is its CPU and disk cost on '
    'a companion computer nobody has bought yet; that is a G-O2-on-target measurement, '
    'not a judgement call to make from here.'
)


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('uav_bringup'), 'config')
    backend_params = os.path.join(config_dir, 'backend_params.yaml')
    # Loaded directly, NOT through a launch argument -- see the docstring.
    localization_params = os.path.join(config_dir, 'localization_params.yaml')
    navigation_params = os.path.join(
        get_package_share_directory('uav_navigation'), 'config', 'navigation_params.yaml')
    world_model_params = os.path.join(
        get_package_share_directory('uav_world_model'), 'config', 'world_model_params.yaml')
    control_authority_params = os.path.join(
        get_package_share_directory('uav_control_authority'),
        'config', 'control_authority_params.yaml')
    safety_params = os.path.join(
        get_package_share_directory('uav_safety'), 'config', 'safety_params.yaml')
    try:
        mission_params = os.path.join(
            get_package_share_directory('uav_mission'), 'config', 'mission_params.yaml')
    except PackageNotFoundError:
        mission_params = None
    try:
        observability_params = os.path.join(
            get_package_share_directory('uav_observability'),
            'config', 'observability_params.yaml')
        preflight_waivers_params = os.path.join(
            get_package_share_directory('uav_observability'),
            'config', 'preflight_waivers.yaml')
    except PackageNotFoundError:
        observability_params = None
        preflight_waivers_params = None

    uav_id = LaunchConfiguration('uav_id')
    use_sim_time = LaunchConfiguration('use_sim_time')
    reviewed = LaunchConfiguration('reviewed')
    perception = LaunchConfiguration('perception')
    navigation = LaunchConfiguration('navigation')
    navigator = LaunchConfiguration('navigator')
    control_authority = LaunchConfiguration('control_authority')
    safety = LaunchConfiguration('safety')
    safety_enforcement = LaunchConfiguration('safety_enforcement')
    mission = LaunchConfiguration('mission')
    blackbox = LaunchConfiguration('blackbox')
    diagnostics = LaunchConfiguration('diagnostics')
    event_log = LaunchConfiguration('event_log')

    # Every node is additionally gated on `reviewed`, so a bare `ros2 launch` is inert.
    def gate(flag=None):
        if flag is None:
            return IfCondition(reviewed)
        # Both must hold. PythonExpression keeps this readable without a custom condition.
        from launch.substitutions import PythonExpression
        return IfCondition(PythonExpression(["'", reviewed, "' == 'true' and '", flag, "' == 'true'"]))

    def make_nodes(package, names, params_file, extra=None):
        return [
            Node(
                package=package,
                executable=name,
                name=name,
                output='screen',
                condition=gate(),
                parameters=[
                    params_file,
                    {'uav_id': uav_id, 'use_sim_time': use_sim_time, **(extra or {})},
                ],
            )
            for name in names
        ]

    def make_navigation_nodes(names, flag):
        return [
            Node(
                package='uav_navigation',
                executable=name,
                name=name,
                output='screen',
                condition=gate(flag),
                parameters=[
                    navigation_params,
                    {
                        'uav_id': uav_id,
                        'use_sim_time': use_sim_time,
                        # P6 Decision 4: TRUE here. Sim may plan without an obstacle
                        # feed because the world is known; on the aircraft, planning
                        # with no obstacle input means flying blind and calling it fine.
                        'require_obstacle_feed': True,
                    },
                ],
            )
            for name in names
        ]

    def make_perception_nodes():
        return [
            Node(
                package='uav_perception',
                executable=name,
                name=name,
                output='screen',
                condition=gate(perception),
                parameters=[{
                    'uav_id': uav_id,
                    'use_sim_time': use_sim_time,
                    **overrides,
                }],
            )
            for name, overrides in PERCEPTION_NODES
        ]

    def make_observability_nodes():
        # Same ordering rule as sim: the recorder must be first in the list, or it
        # misses the last sample of every TransientLocal publisher that fires before it.
        if observability_params is None:
            return []
        specs = [
            ('rosbag_manager_node', blackbox, [], {}),
            ('diagnostics_node', diagnostics, [preflight_waivers_params], {
                'perception_enabled': perception, 'blackbox_enabled': blackbox,
            }),
            ('event_logger_node', event_log, [], {}),
        ]
        return [
            Node(
                package='uav_observability',
                executable=name,
                name=name,
                output='screen',
                condition=gate(flag),
                parameters=[
                    observability_params,
                    *extra_params,
                    {'uav_id': uav_id, 'use_sim_time': use_sim_time, **overrides},
                ],
            )
            for name, flag, extra_params, overrides in specs
        ]

    nodes = (
        make_observability_nodes() +
        make_nodes('uav_px4_backend', BACKEND_NODES, backend_params) +
        make_nodes('uav_localization', LOCALIZATION_NODES, localization_params) +
        make_navigation_nodes(NAVIGATION_NODES, navigation) +
        make_navigation_nodes([NAVIGATOR_NODE], navigator) +
        make_perception_nodes() +
        [Node(
            package='uav_world_model',
            executable='world_model_node',
            name='world_model_node',
            output='screen',
            condition=gate(perception),
            parameters=[
                world_model_params,
                {'uav_id': uav_id, 'use_sim_time': use_sim_time},
            ],
        )] +
        [Node(
            package='uav_control_authority',
            executable='control_authority_manager_node',
            name='control_authority_manager_node',
            output='screen',
            condition=gate(control_authority),
            parameters=[
                control_authority_params,
                {'uav_id': uav_id, 'use_sim_time': use_sim_time},
            ],
        )] +
        [Node(
            package='uav_safety',
            executable='safety_supervisor_node',
            name='safety_supervisor_node',
            output='screen',
            condition=gate(safety),
            parameters=[
                safety_params,
                {
                    'uav_id': uav_id,
                    'use_sim_time': use_sim_time,
                    'enforcement_enabled': safety_enforcement,
                },
            ],
        )] +
        ([Node(
            package='uav_mission',
            executable=name,
            name=name,
            output='screen',
            condition=gate(mission),
            parameters=[
                mission_params,
                {'uav_id': uav_id, 'use_sim_time': use_sim_time},
            ],
        ) for name in MISSION_NODES] if mission_params is not None else [])
    )

    return LaunchDescription([
        DeclareLaunchArgument('uav_id', default_value='uav0'),
        DeclareLaunchArgument(
            'reviewed',
            default_value='false',
            description='Confirm the three inherited decisions have been read. Nothing starts without it.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='FALSE on the aircraft: there is no /clock.',
        ),
        # OFF until P11.3 gives the aircraft a real camera driver. With no image source
        # these nodes would publish nothing while diagnostics reports them enabled.
        DeclareLaunchArgument(
            'perception',
            default_value='false',
            description='Start P5 perception + world_model. Needs the P11.3 camera driver.',
        ),
        DeclareLaunchArgument('navigation', default_value='true'),
        DeclareLaunchArgument('navigator', default_value='true'),
        DeclareLaunchArgument('control_authority', default_value='true'),
        DeclareLaunchArgument('safety', default_value='true'),
        # TRUE, matching sim, and here is the argument P8 asked for: INHIBIT is defined
        # as "latch + deliberate silence -> PX4 core failsafe + hand back to the pilot".
        # So a FALSE POSITIVE gives the aircraft to the pilot, while enforcement OFF
        # means a real hazard is only reported while the aircraft keeps flying on state
        # nobody trusts. With a pilot present at first flight, ON is the safer direction.
        # 🔴 Still requires the reviewed:=true acknowledgement -- this is an argument,
        # not a measurement, and no INHIBIT has ever fired on hardware.
        DeclareLaunchArgument(
            'safety_enforcement',
            default_value='true',
            description='safety_supervisor_node enforcement (P8.4/P8.5). See the note in this file.',
        ),
        DeclareLaunchArgument('mission', default_value='false'),
        # FALSE per P10 D-4, verbatim. See BLACKBOX_NOTE for why that is worth revisiting.
        DeclareLaunchArgument(
            'blackbox',
            default_value='false',
            description='Start rosbag_manager_node. Defaults FALSE here per P10 D-4.',
        ),
        DeclareLaunchArgument('diagnostics', default_value='true'),
        DeclareLaunchArgument('event_log', default_value='true'),
        LogInfo(msg=UNREVIEWED, condition=UnlessCondition(reviewed)),
        LogInfo(msg=BLACKBOX_NOTE, condition=IfCondition(reviewed)),
        *nodes,
    ])
