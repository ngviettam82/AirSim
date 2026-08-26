# One timestamp covers every obstacle entry.

std_msgs/Header header          # frame_id: sensing frame; "odom" on /world topics
string uav_id

Obstacle[] obstacles
float32 sensing_range           # max range of the source sensor, m
