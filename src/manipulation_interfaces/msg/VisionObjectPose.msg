# Generation-stamped object pose estimated from RGB-D geometry.
# accepted is true only for MEASURED. PREDICTED has a bounded executable pose,
# but is never evidence of grasp/placement success. Other states have no pose.
uint8 REJECTED=0
uint8 MEASURED=1
uint8 PREDICTED=2
uint8 OCCLUDED=3
uint8 GRASP_UNCONFIRMED=0
uint8 GRASP_CANDIDATE=1
# Conditional width/geometry evidence, not a force/contact guarantee.
uint8 GRASP_HELD=2
uint8 GRASP_RELEASED=3
uint8 GRASP_INVALIDATED=4
std_msgs/Header header
uint64 bridge_session
uint64 generation
uint64 sample_sequence
uint8 evidence_state
bool pose_valid
uint8 grasp_state
bool attachment_valid
builtin_interfaces/Time grasp_evidence_stamp
builtin_interfaces/Time attachment_stamp
uint64 attachment_measurement_sequence
string grasp_reason
string attachment_basis
builtin_interfaces/Time last_measurement_stamp
uint64 last_measurement_sequence
float64 prediction_age_s
# Growing heuristic position uncertainty budget, not a calibrated covariance
# or a guaranteed statistical error bound.
float64 position_uncertainty_m
# Processing after exact input pairing, excluding camera rendering.
float64 processing_ms
geometry_msgs/Pose pose
# Cube orientation is a canonical representative, not uniquely observed yaw.
bool orientation_ambiguous
bool accepted
float64 confidence
# Geometry metrics are NaN when no current visual measurement exists.
float64 residual_m
float64 inlier_ratio
uint32 point_count
string rejection_reason
