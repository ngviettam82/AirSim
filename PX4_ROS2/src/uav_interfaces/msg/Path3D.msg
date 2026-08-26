# Geometric path, no timing. Input to trajectory generator.

std_msgs/Header header          # frame_id "odom"; world-frame path
string uav_id

geometry_msgs/Pose[] waypoints

bool is_valid                   # true iff plan_state == PLAN_STATE_VALID
string planner_name

uint8 PLAN_STATE_UNKNOWN=0      # default; never explicitly set
uint8 PLAN_STATE_VALID=1        # waypoints are usable
uint8 PLAN_STATE_NO_GOAL=2      # nothing to plan for yet
uint8 PLAN_STATE_WITHDRAWN=3    # a prior valid plan was revoked
uint8 PLAN_STATE_FAILED=4       # planning was attempted and failed

uint8 plan_state
string reason                   # always filled unless plan_state is VALID

builtin_interfaces/Time goal_stamp   # header.stamp of the /planning/route_goal this answers; zero when none
