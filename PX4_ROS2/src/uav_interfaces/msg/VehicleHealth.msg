# Feeds safety-layer failsafe decisions.

std_msgs/Header header          # frame_id empty; non-spatial status
string uav_id

# Battery fields are NaN until the autopilot reports them.
float32 battery_voltage         # V
float32 battery_current         # A
float32 battery_remaining       # 0.0 to 1.0
float32 battery_temperature     # degrees Celsius

bool imu_ok
bool gps_ok
bool barometer_ok
bool magnetometer_ok

uint8 LEVEL_UNKNOWN=0
uint8 LEVEL_OK=1
uint8 LEVEL_WARNING=2
uint8 LEVEL_CRITICAL=3
uint8 overall_level

string detail
