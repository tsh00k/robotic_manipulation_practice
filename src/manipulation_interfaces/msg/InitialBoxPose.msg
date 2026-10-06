# Initial pose of the box on the table, measured from the per-pixel mean of the last
# frames_required depth images of a static scene (Week 4.1 Stage 5, docs/adr/018).
# Published for every depth frame the estimator takes in while the box is not attached to the
# gripper; nothing is published while it is attached.
uint8 WARMING_UP=0    # window not yet full; reason is WINDOW_FILLING
uint8 NOT_MEASURED=1  # window full (or the frame unusable); reason says why
uint8 MEASURED=2      # position and yaw_rad are valid
std_msgs/Header header  # stamp of the newest frame in the window, frame_id "world"
uint64 bridge_session
uint64 generation
uint64 sample_sequence
uint8 state
# Empty when MEASURED. WINDOW_FILLING while warming up. Otherwise the detector's rejection name
# (NO_VALID_DEPTH, NO_BOX_BAND_PIXELS, NO_RECTANGLE_MATCHES_BOX, SEVERAL_BOX_CANDIDATES,
# INVALID_INPUT) or, for a frame that could not be used, INVALID_INPUT / MISSING_ROBOT_TRANSFORM.
string reason
uint32 frames_averaged
uint32 frames_required
# Box centre in the world frame and the yaw about +z folded to [-pi/4, pi/4) (a square repeats
# every 90 degrees). Not a Pose: roll and pitch are not measured, and a quaternion would
# suggest they are. NaN unless state is MEASURED.
geometry_msgs/Point position
float64 yaw_rad
uint32 valid_depth_pixels
# Every block in the height band; empty while warming up or when the frame was unusable.
BlockCandidate[] candidates
# Time spent adding the frame and detecting, excluding the camera and the robot mask.
float64 processing_ms
