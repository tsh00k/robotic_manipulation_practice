#include "task_executor/keyframe_waypoint_source.hpp"

#include <gtest/gtest.h>

namespace task_executor
{
namespace
{

// This is a fixed lookup table, not an algorithm -- there is little to "test" in
// the usual sense. What is worth pinning down: (1) every phase returns a non-
// negative gripper target with a real arm pose behind it (no accidental
// fallthrough returning all-zero ctrl targets, which would snap the arm into a
// random configuration), and (2) the object_pose argument is truly ignored, since
// that is the one behavioral claim this class's docstring makes that a future
// reader could quietly break without noticing. Deliberately not asserting every
// arm_positions entry is nonzero -- kHome's own target genuinely is all zeros
// except joint4/joint6/joint7 (that IS the home keyframe), so a blanket "nonzero"
// check would be testing this specific model's numbers, not the lookup mechanism.
TEST(KeyframeWaypointSource, EveryPhaseReturnsAGripperWidthInRange)
{
  const KeyframeWaypointSource source;
  const ObjectPose pose;
  for (Phase phase : {
      Phase::kHome, Phase::kPregrasp, Phase::kGrasp, Phase::kClose, Phase::kLift,
      Phase::kPreplace, Phase::kPlace, Phase::kOpen, Phase::kRetract, Phase::kVerify,
      Phase::kDone, Phase::kRecover, Phase::kFailed})
  {
    const JointTarget target = source.jointTargetFor(phase, pose);
    EXPECT_GE(target.gripper_width_m, 0.0) << phaseName(phase);
    EXPECT_LE(target.gripper_width_m, 0.08) << phaseName(phase);
  }
}

TEST(KeyframeWaypointSource, ObjectPoseIsIgnored)
{
  const KeyframeWaypointSource source;
  ObjectPose near_origin;
  ObjectPose far_away;
  far_away.x = 99.0;
  far_away.y = -99.0;
  far_away.z = 99.0;
  EXPECT_EQ(
    source.jointTargetFor(Phase::kGrasp, near_origin).arm_positions,
    source.jointTargetFor(Phase::kGrasp, far_away).arm_positions);
}

TEST(KeyframeWaypointSource, CloseTargetsTheSameArmPoseAsGrasp)
{
  const KeyframeWaypointSource source;
  const ObjectPose pose;
  EXPECT_EQ(
    source.jointTargetFor(Phase::kGrasp, pose).arm_positions,
    source.jointTargetFor(Phase::kClose, pose).arm_positions);
  // ...but a narrower gripper target -- that is the entire point of kClose.
  EXPECT_LT(
    source.jointTargetFor(Phase::kClose, pose).gripper_width_m,
    source.jointTargetFor(Phase::kGrasp, pose).gripper_width_m);
}

}  // namespace
}  // namespace task_executor
