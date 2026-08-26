# Backend-reported, never assumed.

std_msgs/Header header          # frame_id empty; non-spatial status
string uav_id

uint8 STATE_IDLE=0
uint8 STATE_STREAMING=1       # setpoints flowing, autopilot not yet in offboard
uint8 STATE_ENGAGING=2        # mode switch requested, awaiting confirmation
uint8 STATE_ACTIVE=3          # autopilot confirmed offboard
uint8 STATE_FAULT=4
uint8 state

float32 setpoint_rate_hz      # measured, not requested
bool command_fresh
bool offboard_confirmed       # read back from autopilot
string detail
