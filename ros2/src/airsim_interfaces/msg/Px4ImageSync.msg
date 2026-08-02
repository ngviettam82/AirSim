# The original image header. Its timestamp is the AirSim rendered-frame time
# and is never changed by the synchronizer.
std_msgs/Header image_header
string image_topic
string airsim_vehicle_name
string px4_topic_prefix
uint8 px4_system_id

# Incremented whenever the synchronizer observes a PX4 HIL-clock regression.
# A rate controller accepts a sync event only from its current clock epoch.
uint64 px4_clock_epoch
uint64 px4_clock_timestamp_us

# Runtime evidence that PX4 is consuming the direct AirSim HIL clock. A
# synchronizer publishes a pair only after at least three exact matches between
# AirSim-sent HIL_SENSOR.time_usec values and PX4 SensorCombined.timestamp.
# Consumers must reject a pair when this proof is absent.
bool direct_hil_clock_verified
uint32 direct_hil_clock_match_count
uint64 direct_hil_clock_last_matched_timestamp_us

# PX4 timestamps are microseconds in the AirSim-driven PX4 HIL clock.
uint64 odometry_before_timestamp_us
uint64 odometry_after_timestamp_us
uint64 attitude_before_timestamp_us
uint64 attitude_after_timestamp_us
uint64 gyro_before_timestamp_us
uint64 gyro_after_timestamp_us
uint64 accelerometer_before_timestamp_us
uint64 accelerometer_after_timestamp_us

# Source intervals used to interpolate at image_header.stamp.
uint64 odometry_bracket_span_ns
uint64 attitude_bracket_span_ns
uint64 gyro_bracket_span_ns
uint64 accelerometer_bracket_span_ns
float32 odometry_interpolation_ratio
float32 attitude_interpolation_ratio
float32 gyro_interpolation_ratio
float32 accelerometer_interpolation_ratio
bool odometry_interpolated
bool attitude_interpolated
bool gyro_interpolated
bool accelerometer_interpolated

# A pair is never published across a PX4 estimator reset or IMU calibration
# change. These counters identify the continuous generation that was used.
uint8 odometry_reset_counter
uint8 attitude_reset_counter
uint8 gyro_calibration_count
uint8 accelerometer_calibration_count

# GPS is associated causally (newest sample at or before the image), not
# relabeled or interpolated. gps_available is false when no sufficiently fresh
# sample exists; gps_age_ns and gps_timestamp_us retain the source relationship.
bool gps_available
uint64 gps_timestamp_us
int64 gps_age_ns

# This is zero only for a runtime-verified direct AirSim HIL clock mapping.
int64 px4_to_airsim_offset_ns
