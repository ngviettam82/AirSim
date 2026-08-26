# Time-parameterized trajectory. Points ordered by time_from_start.

std_msgs/Header header          # frame_id "odom"; world-frame trajectory
string uav_id

TrajectoryPoint[] points
builtin_interfaces/Time start_time

bool is_valid                   # true iff plan_state == PLAN_STATE_VALID
uint32 sequence                 # increments each new plan

uint8 PLAN_STATE_UNKNOWN=0      # default; never explicitly set
uint8 PLAN_STATE_VALID=1        # points are usable
uint8 PLAN_STATE_NO_GOAL=2      # nothing to plan for yet
uint8 PLAN_STATE_WITHDRAWN=3    # a prior valid plan was revoked
uint8 PLAN_STATE_FAILED=4       # planning was attempted and failed

uint8 plan_state
string reason                   # always filled unless plan_state is VALID
