# A discrete mission occurrence, meant for logging and replay.

std_msgs/Header header          # frame_id empty; non-spatial status
string uav_id

uint8 EVENT_UNKNOWN=0
uint8 EVENT_STARTED=1
uint8 EVENT_STEP_STARTED=2
uint8 EVENT_STEP_COMPLETED=3
uint8 EVENT_PAUSED=4
uint8 EVENT_RESUMED=5
uint8 EVENT_FAILED=6
uint8 EVENT_ABORTED=7
uint8 EVENT_COMPLETED=8
uint8 event_type

string mission_id
string step_name
string description               # always fill in on failure
uint8 result_code                # see ResultCode.msg
