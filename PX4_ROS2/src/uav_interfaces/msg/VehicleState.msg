std_msgs/Header header          # frame_id empty; non-spatial status
string uav_id

bool connected
bool armed
bool in_air

uint8 FLIGHT_MODE_UNKNOWN=0
uint8 FLIGHT_MODE_MANUAL=1
uint8 FLIGHT_MODE_ALTITUDE=2
uint8 FLIGHT_MODE_POSITION=3
uint8 FLIGHT_MODE_OFFBOARD=4
uint8 FLIGHT_MODE_HOLD=5
uint8 FLIGHT_MODE_RETURN=6
uint8 FLIGHT_MODE_TAKEOFF=7
uint8 FLIGHT_MODE_LAND=8
uint8 FLIGHT_MODE_MISSION=9
uint8 flight_mode

uint8 nav_state_raw            # raw autopilot code, for log correlation
bool failsafe_active
bool pre_flight_checks_pass
