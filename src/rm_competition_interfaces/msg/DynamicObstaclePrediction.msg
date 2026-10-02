uint8 STATE_TENTATIVE=1
uint8 STATE_CONFIRMED=2
uint8 STATE_COASTING=3

uint64 track_id
uint8 state

# Filtered visible-cluster anchor; not a certified physical object center.
geometry_msgs/Point position
# Estimated anchor velocity; not a certified material-point motion bound.
geometry_msgs/Vector3 velocity
# Smoothed map-axis visible endpoint extents, not a full mechanical footprint.
geometry_msgs/Vector3 size
builtin_interfaces/Time last_observation_stamp
uint32 observation_count
uint32 miss_count

# Display anchor at (i + 1) * prediction_dt after the array stamp.
# The producer profile may clip display speed or decay it; consumers that require
# CV use the public position/velocity rather than assuming this array is CV.
geometry_msgs/Point[] prediction
