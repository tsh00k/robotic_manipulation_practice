# Initial pose of the bin on the table, measured from the per-pixel mean of the last
# frames_required depth images of a static scene (Week 4.1 Stage 6). Same shape and same
# lifecycle as InitialBoxPose: published for every depth frame the estimator takes in while the
# box is not attached to the gripper, nothing while it is attached.
uint8 WARMING_UP=0    # window not yet full; reason is WINDOW_FILLING
uint8 NOT_MEASURED=1  # window full (or the frame unusable); reason says why
uint8 MEASURED=2      # position and yaw_rad are valid
std_msgs/Header header  # stamp of the newest frame in the window, frame_id "world"
uint64 bridge_session
uint64 generation
uint64 sample_sequence
uint8 state
# Empty when MEASURED. WINDOW_FILLING while warming up. Otherwise the detector's rejection name
# (NO_VALID_DEPTH, NO_BIN_BAND_PIXELS, NO_RECTANGLE_MATCHES_BIN, SEVERAL_BIN_CANDIDATES,
# NO_BIN_FLOOR_PIXELS, INVALID_INPUT) or, for a frame that could not be used, INVALID_INPUT /
# MISSING_ROBOT_TRANSFORM.
string reason
uint32 frames_averaged
uint32 frames_required
# Centre of the bin's inner floor in the world frame (z is the floor surface height) and the
# yaw about +z of the bin's long side, folded to [-pi/2, pi/2) (a rectangle repeats every 180
# degrees). The long side is the bin's own x axis. Not a Pose: roll and pitch are not measured.
# NaN unless state is MEASURED.
geometry_msgs/Point position
float64 yaw_rad
uint32 valid_depth_pixels
# Every block in the bin height band; empty while warming up or when the frame was unusable.
BlockCandidate[] candidates
# Time spent adding the frame and detecting, excluding the camera and the robot mask.
float64 processing_ms
