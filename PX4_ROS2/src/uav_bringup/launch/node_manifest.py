"""The node set, in one place, so sim and real cannot silently disagree about it (R7).

R7 promises sim.launch.py and real.launch.py run THE SAME NODES with different config.
That promise is only worth something if it is structural. Two 300-line launch files each
carrying their own copy of the node list drift the moment someone adds a node to one of
them -- and the way that drift is discovered is a node missing on the aircraft, in
flight, which is the failure this project exists to prevent.

real.launch.py imports these. sim.launch.py still carries its own literals: it is
load-bearing for every gate this project has ever run, and editing it to save a
duplication would mean re-verifying every one of those gates for no functional gain
(R14). scripts/check_sim_real_parity.sh closes that gap from the other side -- it builds
BOTH launch descriptions and compares the executables they actually produce, which is a
stronger check than sharing a list, because it also catches a node that exists in one
file but is wired under a condition that can never be true.

Pure data. No ROS imports, no logic -- so it can be read by a checker without launching
anything.
"""

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

# Advisors; gated by the 'navigation' flag. They publish advice, they never steer --
# navigator_action_server_node is the only writer of setpoints.
NAVIGATION_NODES = [
    'route_planner_node',
    'local_planner_node',
]

NAVIGATOR_NODE = 'navigator_action_server_node'

MISSION_NODES = ['mission_executor_node']

# Per-node overrides mirror the verify_*.sh gates so the launched configuration is the
# one those gates measured.
PERCEPTION_NODES = [
    ('marker_detector_node', {'camera': 'down', 'marker_size_m': 0.5}),
    ('obstacle_extractor_node', {'camera': 'front'}),
    ('target_tracker_node', {}),
    ('object_detector_node', {'camera': 'front'}),
    ('camera_health_node', {}),
]
