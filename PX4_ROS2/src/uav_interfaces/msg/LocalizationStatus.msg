# Safety-critical; see docs/interface-contract-v0.1.md

std_msgs/Header header          # frame_id: source-dependent, see contract
string uav_id

uint8 SOURCE_NONE=0
uint8 SOURCE_GPS=1
uint8 SOURCE_VIO=2              # visual inertial odometry
uint8 SOURCE_OPTICAL_FLOW=3
uint8 SOURCE_FUSED=4
uint8 active_source

uint8 QUALITY_UNKNOWN=0
uint8 QUALITY_GOOD=1
uint8 QUALITY_DEGRADED=2
uint8 QUALITY_BAD=3
uint8 quality

bool is_valid                   # false means downstream must not fly on this
float32 position_uncertainty    # 1-sigma, m
float32 heading_uncertainty     # 1-sigma, rad
float32 time_since_update_sec
string detail
