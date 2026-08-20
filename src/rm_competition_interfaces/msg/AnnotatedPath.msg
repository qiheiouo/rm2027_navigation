std_msgs/Header header

string SCHEMA=rm_annotated_path/v1

string schema
string path_revision
uint32 path_pose_count
float64 path_length

string region_set_id
string region_set_revision
string region_set_sha256
string map_id
string map_revision
string map_manifest_sha256

rm_competition_interfaces/PathIntentSegment[] segments
