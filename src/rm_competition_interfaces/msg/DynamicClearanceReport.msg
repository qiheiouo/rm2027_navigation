std_msgs/Header header

string SCHEMA=rm_dynamic_clearance_report/v1
string AUTHORITY_SHADOW_ONLY=shadow_only

uint8 DECISION_UNKNOWN=0
uint8 DECISION_CLEAR=1
uint8 DECISION_BLOCKED=2

string schema
string authority
uint8 decision
string reason

string path_revision
string region_set_sha256
builtin_interfaces/Time prediction_stamp
string[] region_ids
uint8 traversal_policy

float64 route_progress
float64 start_distance
float64 end_distance
float64 entry_eta
float64 prediction_horizon
uint32 sample_count
bool has_minimum_clearance
float64 minimum_clearance
uint64[] blocking_track_ids
