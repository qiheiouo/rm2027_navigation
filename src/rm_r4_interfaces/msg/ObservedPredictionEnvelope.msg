string SCHEMA=rm_r4_observed_predictions/private_v1
string EVIDENCE=associated_observed_endpoints_uncertified
string schema
string producer_id
uint64 producer_generation
uint64 sequence
# The exact public value built once by the existing producer. Its schema,
# authority, source epoch, observation stamps and prediction math are unchanged.
rm_competition_interfaces/DynamicObstaclePredictionArray prediction
# Completeness of members for ALL public tracks, not physical visibility.
bool complete
string reason
rm_r4_interfaces/ObservedTrackMembers[<=256] tracks
