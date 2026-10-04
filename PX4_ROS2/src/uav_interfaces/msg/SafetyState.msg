# Supervision layer only. Does not replace autopilot failsafe.

std_msgs/Header header          # frame_id empty; non-spatial status
string uav_id

uint8 LEVEL_UNKNOWN=0
uint8 LEVEL_OK=1
uint8 LEVEL_WARNING=2
uint8 LEVEL_ERROR=3
uint8 LEVEL_EMERGENCY=4
uint8 level

string[] violation_codes
string[] violation_details       # same order as violation_codes

bool recovery_active
string recommended_action

bool localization_ok
bool offboard_link_ok
bool battery_ok
bool obstacle_clear
bool command_fresh
