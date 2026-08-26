# Safety-critical. See docs/interface-contract-v0.1.md before use.

std_msgs/Header header          # frame_id "odom"; world-frame setpoint
string uav_id

# control_mode selects which fields below are read.
uint8 MODE_UNKNOWN=0
uint8 MODE_POSITION=1
uint8 MODE_VELOCITY=2
uint8 MODE_ACCELERATION=3
uint8 MODE_ATTITUDE=4
uint8 MODE_BODY_RATE=5
uint8 control_mode

geometry_msgs/Point position            # ENU, m
geometry_msgs/Vector3 velocity          # ENU, m/s
geometry_msgs/Vector3 acceleration      # ENU, m/s^2
geometry_msgs/Quaternion orientation    # ENU
geometry_msgs/Vector3 body_rate         # FLU, rad/s

float32 yaw                             # ENU, rad
float32 yaw_rate                        # rad/s
float32 thrust                          # normalized 0..1

uint8 source                            # ControlAuthority SOURCE_*
