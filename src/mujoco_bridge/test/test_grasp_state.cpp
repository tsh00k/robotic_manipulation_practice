#include "mujoco_bridge/grasp_state.hpp"

#include <gtest/gtest.h>

namespace
{

// Chosen so each finger, at this qpos, overlaps `target` by ~1cm without the two
// fingers ever overlapping each other -- see contact_probe.xml's comment for the
// geometry this depends on.
constexpr double kTouchingQpos = 0.065;

class GraspState : public ::testing::Test
{
protected:
  void SetUp() override
  {
    api_ = &mujoco_bridge::loadMujocoApi();
    char error[1024] = {0};
    model_ = api_->loadXML(TEST_FIXTURE_DIR "/contact_probe.xml", nullptr, error, sizeof(error));
    ASSERT_NE(model_, nullptr) << error;
    data_ = api_->makeData(model_);
    ASSERT_NE(data_, nullptr);

    target_id_ = api_->name2id(model_, mjOBJ_BODY, "target");
    left_id_ = api_->name2id(model_, mjOBJ_BODY, "left_finger");
    right_id_ = api_->name2id(model_, mjOBJ_BODY, "right_finger");
    ASSERT_GE(target_id_, 0);
    ASSERT_GE(left_id_, 0);
    ASSERT_GE(right_id_, 0);

    const int left_joint = api_->name2id(model_, mjOBJ_JOINT, "left_slide");
    const int right_joint = api_->name2id(model_, mjOBJ_JOINT, "right_slide");
    ASSERT_GE(left_joint, 0);
    ASSERT_GE(right_joint, 0);
    left_qpos_adr_ = model_->jnt_qposadr[left_joint];
    right_qpos_adr_ = model_->jnt_qposadr[right_joint];
  }

  void TearDown() override
  {
    api_->deleteData(data_);
    api_->deleteModel(model_);
  }

  // Collision detection (and therefore mjData::contact/ncon) is computed as part of
  // mj_forward's position stage -- setting qpos alone does not populate it.
  void SetSlidesAndForward(double left_qpos, double right_qpos)
  {
    data_->qpos[left_qpos_adr_] = left_qpos;
    data_->qpos[right_qpos_adr_] = right_qpos;
    api_->forward(model_, data_);
  }

  const mujoco_bridge::MujocoApi * api_ = nullptr;
  mjModel * model_ = nullptr;
  mjData * data_ = nullptr;
  int target_id_ = -1;
  int left_id_ = -1;
  int right_id_ = -1;
  int left_qpos_adr_ = -1;
  int right_qpos_adr_ = -1;
};

}  // namespace

TEST_F(GraspState, GripperWidthIsSumOfBothSlides)
{
  data_->qpos[left_qpos_adr_] = 0.03;
  data_->qpos[right_qpos_adr_] = 0.02;
  EXPECT_DOUBLE_EQ(mujoco_bridge::gripperWidth(data_, left_qpos_adr_, right_qpos_adr_), 0.05);
}

TEST_F(GraspState, FingersFarFromTargetTouchNothing)
{
  SetSlidesAndForward(0.0, 0.0);
  EXPECT_FALSE(mujoco_bridge::bodiesInContact(model_, data_, left_id_, target_id_));
  EXPECT_FALSE(mujoco_bridge::bodiesInContact(model_, data_, right_id_, target_id_));
}

TEST_F(GraspState, BothFingersOverlappingTargetAreDetected)
{
  SetSlidesAndForward(kTouchingQpos, kTouchingQpos);
  EXPECT_TRUE(mujoco_bridge::bodiesInContact(model_, data_, left_id_, target_id_));
  EXPECT_TRUE(mujoco_bridge::bodiesInContact(model_, data_, right_id_, target_id_));
}

// Regression guard for the argument-order hazard: bodiesInContact must not silently
// assume which side of the pair MuJoCo assigned to geom[0].
TEST_F(GraspState, ContactDetectionIsOrderIndependent)
{
  SetSlidesAndForward(kTouchingQpos, kTouchingQpos);
  EXPECT_TRUE(mujoco_bridge::bodiesInContact(model_, data_, target_id_, left_id_));
  EXPECT_TRUE(mujoco_bridge::bodiesInContact(model_, data_, target_id_, right_id_));
}

// Both fingers touch the target without ever touching *each other* -- a naive "is
// ncon > 0" check would not catch a bug that reported contact for the wrong pair.
TEST_F(GraspState, FingersDoNotTouchEachOtherWhileBothTouchTarget)
{
  SetSlidesAndForward(kTouchingQpos, kTouchingQpos);
  EXPECT_FALSE(mujoco_bridge::bodiesInContact(model_, data_, left_id_, right_id_));
}

TEST_F(GraspState, OnlyOneFingerAdvancedTouchesAlone)
{
  SetSlidesAndForward(kTouchingQpos, 0.0);
  EXPECT_TRUE(mujoco_bridge::bodiesInContact(model_, data_, left_id_, target_id_));
  EXPECT_FALSE(mujoco_bridge::bodiesInContact(model_, data_, right_id_, target_id_));
}
