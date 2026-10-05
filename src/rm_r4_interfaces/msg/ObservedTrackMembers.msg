# Private measured support from the same tracker assignment; never a full body.
uint64 track_id
builtin_interfaces/Time last_observation_stamp
uint64 association_sequence
uint32 detection_index
# Centroid in prediction.header.frame_id at last_observation_stamp.
geometry_msgs/Point centroid_at_observation
# Same-frame axes, relative to centroid_at_observation; no extrapolation here.
geometry_msgs/Point[<=4096] local_endpoints
# Original LaserScan beam indices, in one-to-one order with local_endpoints.
uint32[<=4096] source_member_ids
