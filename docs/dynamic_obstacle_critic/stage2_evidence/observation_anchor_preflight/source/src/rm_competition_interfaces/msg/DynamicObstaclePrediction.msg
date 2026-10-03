uint8 STATE_TENTATIVE=1
uint8 STATE_CONFIRMED=2
uint8 STATE_COASTING=3

uint64 track_id
uint8 state

# Schema v1: filtered visible-cluster anchor.
# Schema v2_observation_anchor: latest associated detection centroid advanced
# from last_observation_stamp to the array stamp with this velocity estimate.
# Neither is a certified physical object center or full-body geometry.
geometry_msgs/Point position
# Estimated anchor velocity; not a certified material-point motion bound.
geometry_msgs/Vector3 velocity
# Smoothed map-axis visible endpoint extents, not a full mechanical footprint.
geometry_msgs/Vector3 size
builtin_interfaces/Time last_observation_stamp
uint32 observation_count
uint32 miss_count

# Display anchor at (i + 1) * prediction_dt after the array stamp.
# v1 may clip/decay display velocity. v2 uses the same unclipped, undecayed CV
# as public position/velocity. Consumers still recompute CV and check schema.
geometry_msgs/Point[] prediction
