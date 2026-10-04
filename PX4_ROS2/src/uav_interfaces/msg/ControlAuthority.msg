# Higher value wins arbitration.

std_msgs/Header header          # frame_id empty; non-spatial status
string uav_id

uint8 SOURCE_NONE=0
uint8 SOURCE_TEST=1
uint8 SOURCE_MISSION=2
uint8 SOURCE_OPERATOR=3
uint8 SOURCE_SAFETY=4

uint8 active_source
uint8 previous_source
string reason
