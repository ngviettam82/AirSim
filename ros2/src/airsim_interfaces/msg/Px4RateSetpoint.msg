# Body-rate setpoint for PX4 mode. header.stamp is mandatory and must be the
# AirSim/PX4 HIL source timestamp used to calculate this command (normally the
# synchronized camera image stamp). Replayed, future, or stale stamps fail closed.
# Rates use ROS FLU axes and radians/second. thrust is normalized upward
# collective in the closed interval [0, 1].
std_msgs/Header header
float32 roll_rate
float32 pitch_rate
float32 yaw_rate
float32 thrust
