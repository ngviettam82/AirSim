std_msgs/Header header          # frame_id names the frame pose is expressed in
string uav_id

uint8 STATUS_UNKNOWN=0
uint8 STATUS_TRACKING=1
uint8 STATUS_COASTING=2         # briefly hidden, still predicted
uint8 STATUS_LOST=3

int32 track_id
uint8 status

geometry_msgs/Pose pose         # m and rad
geometry_msgs/Vector3 velocity  # m/s, header frame, relative to sensor

float32 confidence
float32 position_uncertainty    # 1-sigma, m; -1 if unknown
float32 age_sec                 # time since first detection
float32 time_since_seen_sec     # 0 while visible
