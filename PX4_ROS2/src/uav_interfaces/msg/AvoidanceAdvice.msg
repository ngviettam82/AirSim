std_msgs/Header header          # frame_id names the frame escape_point is in; always "odom"
string uav_id

uint8 ADVICE_CLEAR=0            # the checked horizon is free; keep flying
uint8 ADVICE_ESCAPE=1           # obstacle ahead; escape_point is a way around it
uint8 ADVICE_HOLD=2             # cannot advise safely; freeze and wait

uint8 advice
string reason                   # always filled, including on CLEAR

geometry_msgs/Point escape_point  # m; NaN unless advice is ADVICE_ESCAPE

float32 clearance_m             # smallest clearance found along checked_horizon_m
float32 checked_horizon_m       # how far ahead this advice actually looked

bool map_fresh                  # false once the obstacle feed is older than its timeout
float32 map_age_sec             # inf when no obstacle input has ever arrived
