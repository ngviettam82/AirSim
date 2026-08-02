# A bounded, oldest-to-newest history of HIL_SENSOR.time_usec values that
# AirSim queued to the MAVLink transport for this vehicle. PX4 consumers use
# exact membership against SensorCombined.timestamp to prove the direct HIL
# clock path; these are not ROS receipt timestamps.
string airsim_vehicle_name
uint64[] hil_sensor_timestamps_us
