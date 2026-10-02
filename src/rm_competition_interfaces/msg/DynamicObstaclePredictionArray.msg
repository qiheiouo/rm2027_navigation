# Input scan source epoch and world frame of the filtered public state.
std_msgs/Header header

string SCHEMA=rm_dynamic_obstacle_predictions/v1
string AUTHORITY_SHADOW_ONLY=shadow_only

string schema
string authority
builtin_interfaces/Time processing_stamp
float64 prediction_dt
uint32 prediction_steps
# False on output-budget truncation. True does not certify obstacle detection
# completeness, full object visibility, geometry bounds or correct association.
bool complete
uint32 total_track_count
rm_competition_interfaces/DynamicObstaclePrediction[] tracks
