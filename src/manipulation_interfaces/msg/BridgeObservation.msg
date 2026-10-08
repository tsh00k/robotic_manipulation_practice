uint64 bridge_session
uint64 generation
uint64 sample_sequence
uint8 ATTACHMENT_NOT_ATTACHED=0
uint8 ATTACHMENT_ATTACHED=1
uint8 ATTACHMENT_RELEASED=2
uint8 attachment_state
sensor_msgs/JointState joint_state
geometry_msgs/PoseStamped object_pose
geometry_msgs/TransformStamped world_to_hand_tcp
bool left_finger_contact
bool right_finger_contact
# What the controller applied in the physics step this observation describes (Week 5 Stage 6,
# docs/adr/021): the arm's position-servo targets written into ctrl, joint1..joint7 in radians,
# and the gripper's target total finger opening in metres. The dataset's action. Stamped and
# sequenced with everything above, so state and action come from the same step.
float64[] arm_command
float64 gripper_command_width_m
