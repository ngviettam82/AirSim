std_msgs/Header header          # frame_id names the observing camera
string uav_id

uint8 FAMILY_UNKNOWN=0
uint8 FAMILY_ARUCO=1
uint8 FAMILY_APRILTAG=2
uint8 FAMILY_QR=3

int32 marker_id
uint8 family
geometry_msgs/Pose pose         # camera frame, m and rad
float32 size                    # m
float32 confidence
float32 position_uncertainty    # 1-sigma, m; -1 if unknown
