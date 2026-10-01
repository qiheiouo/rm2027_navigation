uint8 STATE_TENTATIVE=1
uint8 STATE_CONFIRMED=2
uint8 STATE_COASTING=3

uint64 track_id
uint8 state

geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 size
builtin_interfaces/Time last_observation_stamp
uint32 observation_count
uint32 miss_count

# prediction[i] is the center at (i + 1) * prediction_dt after the array stamp.
geometry_msgs/Point[] prediction
