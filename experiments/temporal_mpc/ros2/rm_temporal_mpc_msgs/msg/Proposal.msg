# Solver proposal; only native Controller Server plugin can accept/execute it.
std_msgs/Header header
uint64 generation
uint64 map_revision
float64 fixed_yaw
float64[4] centre_bounds
float64 period
geometry_msgs/Vector3[<=30] accelerations
bool model_feasible
bool fallback_requested
string reason
