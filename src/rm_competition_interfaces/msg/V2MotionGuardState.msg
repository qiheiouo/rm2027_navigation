# Host encoding decision only. This is not a TX ACK or MCU/physical stop proof.
uint8 DISABLED=0
uint8 WAITING=1
uint8 PERMITTED=2
uint8 BLOCKED=3
std_msgs/Header header
string endpoint_instance
string command_topic
string owner_instance
string session_id
uint8 phase
string reason
# Readiness observation only; never cached/applied as a chassis command.
uint8[24] observed_command_gid
uint64 observed_command_count
float64 observed_command_age_sec
