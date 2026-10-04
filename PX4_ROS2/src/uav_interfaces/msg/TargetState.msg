# World-frame target estimate; status values follow TargetTrack.

std_msgs/Header header          # frame_id "odom"; stamp = publish time
string uav_id

int32 track_id
uint8 status                    # values follow TargetTrack
geometry_msgs/Pose pose         # m and rad
geometry_msgs/Vector3 velocity  # m/s, odom frame
float32 position_uncertainty    # 1-sigma, m; perception + localization, never -1
float32 time_since_seen_sec     # 0 while visible
