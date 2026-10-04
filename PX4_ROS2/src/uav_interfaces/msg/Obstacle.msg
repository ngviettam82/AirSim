# Component of ObstacleArray; no header, frame/timestamp inherited.

uint8 SHAPE_UNKNOWN=0
uint8 SHAPE_BOX=1
uint8 SHAPE_SPHERE=2
uint8 SHAPE_CYLINDER=3

int32 obstacle_id               # negative when tracking is not available
uint8 shape

geometry_msgs/Point center      # m
geometry_msgs/Vector3 size      # full extent, m (box 1x1x2 -> size=(1,1,2))
float32 distance                # closest range to the vehicle, m
float32 confidence
float32 position_uncertainty    # 1-sigma, m; -1 if unknown
