# Nested in Trajectory3D. No header by design.

builtin_interfaces/Duration time_from_start

geometry_msgs/Point position        # ENU, m
geometry_msgs/Vector3 velocity      # ENU, m/s
geometry_msgs/Vector3 acceleration  # ENU, m/s^2

float32 yaw                         # ENU, rad
float32 yaw_rate                    # rad/s
