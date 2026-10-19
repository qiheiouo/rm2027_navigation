# Software-only, bounded single-goal owner evidence. Never accepted on hardware.
# Addressed to this guard instance; no cross-instance or in-process rearm.
std_msgs/Header header
string endpoint_instance
string owner_instance
string session_id
uint64 sequence
builtin_interfaces/Time valid_until
uint8[24] command_publisher_gid
bool enabled
