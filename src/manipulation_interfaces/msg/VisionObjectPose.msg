# Generation-stamped object pose estimated from RGB-D geometry.
# evidence_state is the sole source for the kind of pose evidence available.
uint8 REJECTED=0
uint8 MEASURED=1
uint8 PREDICTED=2
uint8 OCCLUDED=3
uint8 GRASP_NOT_ATTACHED=0
uint8 GRASP_HELD=1
uint8 GRASP_RELEASED=2
# Diagnostic layer for state_reason. This is orthogonal to evidence_state.
uint8 DIAGNOSTIC_NONE=0
uint8 DIAGNOSTIC_INPUT=1
uint8 DIAGNOSTIC_GEOMETRY=2
uint8 DIAGNOSTIC_ASSOCIATION=3
uint8 DIAGNOSTIC_PREDICTION=4
uint8 DIAGNOSTIC_GRASP=5
uint8 DIAGNOSTIC_LIFECYCLE=6
std_msgs/Header header
uint64 bridge_session
uint64 generation
uint64 sample_sequence
uint8 evidence_state
uint8 grasp_state
bool attachment_valid
# Sequence of the latest measured pose, if one exists in this lifecycle.
uint64 last_measurement_sequence
# Processing after exact input pairing, excluding camera rendering.
float64 processing_ms
geometry_msgs/Pose pose
# Cube orientation is a canonical representative, not uniquely observed yaw.
bool orientation_ambiguous
float64 confidence
# Geometry metrics are NaN when no current visual measurement exists.
float64 residual_m
float64 inlier_ratio
uint32 point_count
# Empty for a normal measured state; otherwise explains evidence_state.
string state_reason
uint8 diagnostic_stage
# Candidate/association fields are NaN or zero when no history/candidate exists.
uint32 candidate_count
uint32 eligible_candidate_count
bool support_prior_used
