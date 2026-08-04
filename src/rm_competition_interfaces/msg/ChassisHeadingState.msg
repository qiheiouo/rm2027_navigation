std_msgs/Header header

# Chassis heading in the lower-controller world frame. Consumers must use
# heading deltas unless that world frame has been explicitly aligned to ROS.
float64 yaw_rad
float64 yaw_rate_rad_s

uint32 sample_sequence
uint32 mcu_time_ms

# Increment whenever the lower-controller heading estimator is re-zeroed or
# reset without rebooting the MCU.
uint16 reset_counter

# Copied from the competition_v2 heartbeat by the upper transport.
uint32 source_boot_id

bool online
bool valid
