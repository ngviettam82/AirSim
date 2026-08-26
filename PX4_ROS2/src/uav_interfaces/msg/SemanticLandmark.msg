# Component of SemanticLandmarkArray; no header.

int32 marker_id
uint8 family                    # values follow MarkerObservation
geometry_msgs/Pose pose         # m and rad, frame from array header
float32 position_uncertainty    # 1-sigma, m; perception + localization, never -1
float32 time_since_seen_sec     # 0 while visible
