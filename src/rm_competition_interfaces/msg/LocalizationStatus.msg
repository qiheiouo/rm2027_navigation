# Periodic localization evidence; Bool topics are diagnostic snapshots only.
# header.stamp is emission time, never the time of a new measurement.
std_msgs/Header header
# Unique per process; started_at and sequence prevent old-instance/replay reuse.
string producer_id
builtin_interfaces/Time started_at
uint64 sequence
bool valid
# ROS-time upper bound from the accepted source, not extended by heartbeats.
# Consumers also require a bounded steady-clock receipt lease.
builtin_interfaces/Time valid_until
# Last gate-accepted map->base pose (may be retained while valid=false).
geometry_msgs/PoseWithCovarianceStamped pose
