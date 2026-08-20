uint8 REGION_SLOW_ZONE=1
uint8 REGION_NO_SPIN=2
uint8 REGION_FORBIDDEN=3
uint8 REGION_DOG_HOLE_APPROACH=4
uint8 REGION_COMMITTED_CORRIDOR=5
uint8 REGION_TEMPORARY_STRUCTURE=6

uint8 ADMISSION_NONE=0
uint8 ADMISSION_DYNAMIC_CLEARANCE=1

uint8 TRAVERSAL_PREEMPTIBLE=0
uint8 TRAVERSAL_COMMITTED=1

float64 start_distance
float64 end_distance
string[] region_ids
uint8[] region_types

bool blocked
bool has_max_linear_speed
float64 max_linear_speed
bool has_required_heading
float64 required_heading
float64 heading_tolerance
bool no_spin
uint8 admission_policy
uint8 traversal_policy
