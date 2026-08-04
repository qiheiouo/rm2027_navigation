uint8 COORDINATE_SYSTEM_UNKNOWN=0
uint8 COORDINATE_SYSTEM_REFEREE_FIELD=1
uint8 COORDINATE_SYSTEM_MAP=2

uint8 ALLIANCE_UNKNOWN=0
uint8 ALLIANCE_RED=1
uint8 ALLIANCE_BLUE=2

std_msgs/Header header

float32 target_x
float32 target_y
float32 target_yaw
uint32 command_id
uint8 robot_id
uint8 coordinate_system
uint8 alliance
bool has_yaw
uint32 mcu_sample_time_ms
uint16 ttl_ms

# True means only that the serial frame passed transport checks and the
# coordinates are finite. It does not authorize motion or imply a new command.
bool transport_valid
