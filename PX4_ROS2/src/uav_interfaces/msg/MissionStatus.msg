# Current progress of the running mission.

std_msgs/Header header          # frame_id empty; non-spatial status
string uav_id

uint8 STATE_UNKNOWN=0
uint8 STATE_IDLE=1
uint8 STATE_RUNNING=2
uint8 STATE_PAUSED=3
uint8 STATE_ABORTED=4
uint8 STATE_COMPLETED=5
uint8 state

string mission_id
string current_step_name
uint32 current_step_index       # counts from 0
uint32 total_steps
float32 progress_percent        # 0.0 to 100.0
float32 elapsed_sec

uint8 last_result_code          # see ResultCode.msg; reason for current PAUSED/ABORTED
string last_reason              # always filled when state is PAUSED or ABORTED
string goal_id                  # ExecuteMission goal UUID hex; empty when idle
