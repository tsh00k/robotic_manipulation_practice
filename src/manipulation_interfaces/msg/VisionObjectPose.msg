# Generation-stamped object pose estimated from RGB-D geometry.
# The pose is valid only when accepted is true. A rejected sample is still
# published so the task layer can distinguish a fresh rejection from a stale
# estimate.
std_msgs/Header header
uint64 bridge_session
uint64 generation
uint64 sample_sequence
geometry_msgs/Pose pose
bool accepted
float64 confidence
float64 residual_m
float64 inlier_ratio
uint32 point_count
string rejection_reason
