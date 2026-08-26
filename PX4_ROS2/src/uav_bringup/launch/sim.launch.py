"""Bring up the backend against PX4 SITL; start start_sim.sh first, see README."""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BACKEND_NODES = [
    'px4_state_adapter_node',
    'px4_frame_bridge_node',
    'px4_command_gateway_node',
    'offboard_session_manager_node',
    'px4_external_odometry_node',
]

LOCALIZATION_NODES = [
    'rangefinder_adapter_node',
    'gps_adapter_node',
    'vio_adapter_node',
    'optical_flow_adapter_node',
    'localization_mux_node',
    'localization_health_node',
]

# Advisors; gated by the 'navigation' flag.
NAVIGATION_NODES = [
    'route_planner_node',
    'local_planner_node',
]

# P7.4 (N13, sim half): the action gateway itself. Own flag -- it now writes
# control/cmd_mission (P7.3), a source the arbiter gates on authority, not
# on whether the advisors are running.
NAVIGATOR_NODE = 'navigator_action_server_node'

# P9.5: uav_mission is built on a parallel branch, may not be installed yet.
MISSION_NODES = ['mission_executor_node']

# P5 perception; no shared config yaml per package, overrides mirror the
# verify_*.sh scripts (marker/obstacle/world_model) so this stays in sync.
PERCEPTION_NODES = [
    ('marker_detector_node', {'camera': 'down', 'marker_size_m': 0.5}),
    ('obstacle_extractor_node', {'camera': 'front'}),
    ('target_tracker_node', {}),
    ('object_detector_node', {'camera': 'front'}),
    # P10.8b: was built but never wired -- state/camera_health and
    # diagnostics/perception had zero publishers in every bringup config.
    ('camera_health_node', {}),
]


def dds_profile_actions():
    """Put the stack on the same transport the gates measured it on (debt #10).

    start_sim.sh exports this too, but it runs in a CHILD shell, so its export dies
    with it and reaches only the agent, Gazebo and the bridge -- not this launch.
    Gate R0 and the two product gates were all measured with the profile in force for
    EVERY process, so shipping it to only some of them would be R31 exactly: a number
    valid only in the condition it was forced in, applied to a different condition.

    UAV_DDS_PROFILE=none opts out. That exists so gate R0's CONTROL arm can still run
    stock transport; without it the gate would silently measure the profile twice and
    could never be re-run. A gate that cannot be re-run is a gate that rots.

    Sim-only: the ros_gz image bridge does not exist on the real aircraft. Whether the
    real camera driver needs the same profile is an open P11 question -- the 549 408 B
    ceiling belongs to the Fast DDS default, not to the simulator.
    """
    if os.environ.get('UAV_DDS_PROFILE') == 'none':
        return []
    profile = os.path.join(
        get_package_share_directory('uav_bringup'), 'config', 'fastdds_large_samples.xml')
    if not os.path.isfile(profile):
        print('WARNING: large-samples DDS profile missing -- cameras will starve (debt #10).')
        return []
    return [SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', profile)]


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('uav_bringup'), 'config')
    backend_params = os.path.join(config_dir, 'backend_params.yaml')
    shipped_localization_params = os.path.join(config_dir, 'localization_params.yaml')
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
        print('WARNING: uav_mission not installed -- mission nodes skipped (P9.5).')
    try:
        observability_params = os.path.join(
            get_package_share_directory('uav_observability'),
            'config', 'observability_params.yaml')
        # P10.9b: sim.launch.py and (P11) real.launch.py load THIS SAME FILE
        # (R7) -- there is no per-environment waiver table.
        preflight_waivers_params = os.path.join(
            get_package_share_directory('uav_observability'),
            'config', 'preflight_waivers.yaml')
    except PackageNotFoundError:
        observability_params = None
        preflight_waivers_params = None
        print('WARNING: uav_observability not installed -- observability nodes skipped (P10.6).')

    uav_id = LaunchConfiguration('uav_id')
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization_params = LaunchConfiguration('localization_params')
    perception = LaunchConfiguration('perception')
    navigation = LaunchConfiguration('navigation')
    require_obstacle_feed = LaunchConfiguration('require_obstacle_feed')
    navigator = LaunchConfiguration('navigator')
    control_authority = LaunchConfiguration('control_authority')
    safety = LaunchConfiguration('safety')
    safety_enforcement = LaunchConfiguration('safety_enforcement')
    mission = LaunchConfiguration('mission')
    blackbox = LaunchConfiguration('blackbox')
    diagnostics = LaunchConfiguration('diagnostics')
    event_log = LaunchConfiguration('event_log')

    def make_nodes(package, names, params_file, condition=None):
        return [
            Node(
                package=package,
                executable=name,
                name=name,
                output='screen',
                condition=condition,
                parameters=[
                    params_file,
                    {'uav_id': uav_id, 'use_sim_time': use_sim_time},
                ],
            )
            for name in names
        ]

    def make_navigation_nodes(names, condition):
        return [
            Node(
                package='uav_navigation',
                executable=name,
                name=name,
                output='screen',
                condition=condition,
                parameters=[
                    navigation_params,
                    {
                        'uav_id': uav_id,
                        'use_sim_time': use_sim_time,
                        # Decision 4 (P6): real.launch.py must override this true.
                        'require_obstacle_feed': require_obstacle_feed,
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
                condition=IfCondition(perception),
                parameters=[{
                    'uav_id': uav_id,
                    'use_sim_time': use_sim_time,
                    **overrides,
                }],
            )
            for name, overrides in PERCEPTION_NODES
        ]

    def make_observability_nodes():
        # P10.6: rosbag_manager_node MUST be first in the whole node list --
        # TransientLocal race #967 (plan S:3): the recorder has to be up
        # before any latched publisher fires or it misses the last sample.
        if observability_params is None:
            return []
        specs = [
            ('rosbag_manager_node', blackbox, [], {}),
            # C2: perception_enabled_/blackbox_enabled_ tell diagnostics_node
            # which OPTIONAL subsystems this bringup even started -- without
            # this, an owned item with no publisher (e.g. world/
            # semantic_landmarks when perception:=false) reads as
            # permanently UNKNOWN and go_no_go gets stuck UNKNOWN forever,
            # never able to reflect a REAL later fault (docs/interface-
            # contract-v0.1.md S:2.20).
            # P10.9b: preflight_waivers_params is a SEPARATE yaml file (own
            # sign-off cadence, docs/interface-contract-v0.1.md S:2.20) --
            # only diagnostics_node declares the waiver_* params it fills.
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
                condition=IfCondition(flag),
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
        make_navigation_nodes(NAVIGATION_NODES, IfCondition(navigation)) +
        make_navigation_nodes([NAVIGATOR_NODE], IfCondition(navigator)) +
        make_perception_nodes() +
        [Node(
            package='uav_world_model',
            executable='world_model_node',
            name='world_model_node',
            output='screen',
            condition=IfCondition(perception),
            parameters=[
                world_model_params,
                {'uav_id': uav_id, 'use_sim_time': use_sim_time},
            ],
        )] +
        # P7.3: ON by default -- the arbiter is now command_selected's single writer.
        [Node(
            package='uav_control_authority',
            executable='control_authority_manager_node',
            name='control_authority_manager_node',
            output='screen',
            condition=IfCondition(control_authority),
            parameters=[
                control_authority_params,
                {'uav_id': uav_id, 'use_sim_time': use_sim_time},
            ],
        )] +
        # P8.4/P8.5: ON by default, ENFORCEMENT ON by default in sim
        # (safety_enforcement true) -- both INHIBIT (cuts via the arbiter's
        # SAFETY latch) and HOLD (streams a frozen pose on cmd_safety) are
        # real enforcement now, not dry-run.
        [Node(
            package='uav_safety',
            executable='safety_supervisor_node',
            name='safety_supervisor_node',
            output='screen',
            condition=IfCondition(safety),
            parameters=[
                safety_params,
                {
                    'uav_id': uav_id,
                    'use_sim_time': use_sim_time,
                    'enforcement_enabled': safety_enforcement,
                },
            ],
        )] +
        # P9.5: OFF by default; empty when uav_mission is not built yet.
        ([Node(
            package='uav_mission',
            executable=name,
            name=name,
            output='screen',
            condition=IfCondition(mission),
            parameters=[
                mission_params,
                {'uav_id': uav_id, 'use_sim_time': use_sim_time},
            ],
        ) for name in MISSION_NODES] if mission_params is not None else [])
    )

    return LaunchDescription([
        # First: every action below inherits it.
        *dds_profile_actions(),
        # SIM-ONLY, and deliberately so. real.launch.py hard-codes this TRUE (P6 D4) and
        # must keep it un-flippable: a launch argument is something an operator can pass,
        # and softening a signed safety decision must not be one keystroke away. Same
        # reasoning as localization_params, which is sim-only for the same kind of reason.
        #
        # Default false keeps every existing gate measuring what it already measured. The
        # S13 gate passes true to fly the branch the aircraft will actually take.
        DeclareLaunchArgument(
            'require_obstacle_feed',
            default_value='false',
            description='SIM ONLY. Planner refuses to plan without an obstacle feed (P6 D4).',
        ),
        DeclareLaunchArgument('uav_id', default_value='uav0'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Follow /clock. The real drone sets this false.',
        ),
        # Sim-only noise injectors; real.launch.py has no such override.
        DeclareLaunchArgument(
            'localization_params',
            default_value=shipped_localization_params,
            description='Override the localization config. Sim experiments only.',
        ),
        # OFF by default (R0): M5 baseline node set must stay unchanged.
        DeclareLaunchArgument(
            'perception',
            default_value='false',
            description='Start P5 perception + world_model nodes (adds render load).',
        ),
        # ON by default; set false to shed load for an RTF-sensitive M5 run.
        DeclareLaunchArgument(
            'navigation',
            default_value='true',
            description='Start route_planner_node + local_planner_node (P6.3/P6.4).',
        ),
        # ON by default (P7.4, N13 sim half): mission's cmd_mission source is
        # arbitrated (P7.3), so an idle navigator does not contend for authority.
        DeclareLaunchArgument(
            'navigator',
            default_value='true',
            description='Start navigator_action_server_node (P6/P7.4).',
        ),
        # ON by default (P7.3): the arbiter is command_selected's single writer.
        DeclareLaunchArgument(
            'control_authority',
            default_value='true',
            description='Start control_authority_manager_node (P7).',
        ),
        # ON by default (P8.3): safety_supervisor_node.
        DeclareLaunchArgument(
            'safety',
            default_value='true',
            description='Start safety_supervisor_node (P8.3).',
        ),
        # 🔴 R0: ON by default in SIM as of P8.4/P8.5 -- INHIBIT (policy 1 & 2)
        # really cuts command_selected via the arbiter's SAFETY latch, and
        # HOLD (policy 3) really streams a frozen pose on cmd_safety, once
        # G-S1/G-S2/G-S3 close (this file's own change gate) -- neither is
        # dry-run any more. real.launch.py (P11) must review this default
        # independently before first real flight -- do not assume sim's
        # default is right there.
        DeclareLaunchArgument(
            'safety_enforcement',
            default_value='true',
            description='safety_supervisor_node enforcement_enabled (P8.4/P8.5: INHIBIT + HOLD, real).',
        ),
        # OFF by default (P9.5): uav_mission is being built on a parallel branch.
        DeclareLaunchArgument(
            'mission',
            default_value='false',
            description='Start mission_executor_node (P9); no-op if uav_mission is not installed.',
        ),
        # ON by default (P10 kickoff decision 2): the evidence layer is not
        # optional in sim. real.launch.py flips this false until P11 review
        # (D-4, same pattern as safety_enforcement).
        DeclareLaunchArgument(
            'blackbox',
            default_value='true',
            description='Start rosbag_manager_node (P10.2/P10.6); no-op if uav_observability missing.',
        ),
        DeclareLaunchArgument(
            'diagnostics',
            default_value='true',
            description='Start diagnostics_node go/no-go (P10.4/P10.6); no-op if uav_observability missing.',
        ),
        DeclareLaunchArgument(
            'event_log',
            default_value='true',
            description='Start event_logger_node JSONL timeline (P10.5/P10.6); no-op if uav_observability missing.',
        ),
        *nodes,
    ])
