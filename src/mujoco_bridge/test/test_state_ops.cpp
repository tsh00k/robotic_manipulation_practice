// Copyright 2026 anby
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include "mujoco_bridge/state_ops.hpp"

namespace
{

class StateOps : public ::testing::Test
{
protected:
  void SetUp() override
  {
    api_ = &mujoco_bridge::loadMujocoApi();
    char error[1024] = {0};
    model_ = api_->loadXML(TEST_FIXTURE_DIR "/free_body.xml", nullptr, error, sizeof(error));
    ASSERT_NE(model_, nullptr) << error;
    data_ = api_->makeData(model_);
    ASSERT_NE(data_, nullptr);
    home_key_ = api_->name2id(model_, mjOBJ_KEY, "home");
    ASSERT_GE(home_key_, 0);
  }

  void TearDown() override
  {
    api_->deleteData(data_);
    api_->deleteModel(model_);
  }

  const mujoco_bridge::MujocoApi * api_ = nullptr;
  mjModel * model_ = nullptr;
  mjData * data_ = nullptr;
  int home_key_ = -1;
};

}  // namespace

// This fixture only makes sense with a genuine free joint: nq (8: 7 for the free
// body + 1 hinge) != nv (7: 6 for the free body + 1 hinge). On Panda alone
// (all hinge/slide joints) nq == nv and a jnt_qposadr/jnt_dofadr mixup is silently
// wrong -- see week1 4.3.4 / week2 Stage F.
TEST_F(StateOps, ModelHasMismatchedNqNv)
{
  EXPECT_EQ(model_->nq, 8);
  EXPECT_EQ(model_->nv, 7);
}

// The hinge joint is the second joint in the model (after the box's free joint),
// so its qpos_adr (7) and dof_adr (6) are different addresses. Reading qvel/qfrc_*
// with the wrong one would read the box's data instead of the hinge's.
TEST_F(StateOps, HingeJointQposAdrAndDofAdrDiffer)
{
  const int hinge_id = api_->name2id(model_, mjOBJ_JOINT, "hinge");
  ASSERT_GE(hinge_id, 0);
  EXPECT_EQ(model_->jnt_qposadr[hinge_id], 7);
  EXPECT_EQ(model_->jnt_dofadr[hinge_id], 6);
  EXPECT_NE(model_->jnt_qposadr[hinge_id], model_->jnt_dofadr[hinge_id]);
}

TEST_F(StateOps, ResetRestoresKeyframeQpos)
{
  for (int i = 0; i < model_->nq; ++i) {
    data_->qpos[i] = 999.0;
  }

  ASSERT_TRUE(mujoco_bridge::resetToKeyframe(*api_, model_, data_, home_key_));

  for (int i = 0; i < model_->nq; ++i) {
    EXPECT_DOUBLE_EQ(data_->qpos[i], model_->key_qpos[home_key_ * model_->nq + i]);
  }
}

// The keyframe's own time is 0; a reset must not rewind the clock, only the state.
TEST_F(StateOps, ResetPreservesSimTime)
{
  data_->time = 12.5;

  ASSERT_TRUE(mujoco_bridge::resetToKeyframe(*api_, model_, data_, home_key_));

  EXPECT_DOUBLE_EQ(data_->time, 12.5);
}

TEST_F(StateOps, ResetRefreshesDerivedQuantities)
{
  ASSERT_TRUE(mujoco_bridge::resetToKeyframe(*api_, model_, data_, home_key_));

  // xpos for the box body should match the keyframe's translation (qpos[0..2]),
  // not some stale pre-reset value -- this only holds if mj_forward actually ran.
  const int box_id = api_->name2id(model_, mjOBJ_BODY, "box");
  ASSERT_GE(box_id, 0);
  EXPECT_DOUBLE_EQ(data_->xpos[3 * box_id + 0], model_->key_qpos[home_key_ * model_->nq + 0]);
  EXPECT_DOUBLE_EQ(data_->xpos[3 * box_id + 1], model_->key_qpos[home_key_ * model_->nq + 1]);
  EXPECT_DOUBLE_EQ(data_->xpos[3 * box_id + 2], model_->key_qpos[home_key_ * model_->nq + 2]);
}

TEST_F(StateOps, NegativeKeyReturnsFalseAndLeavesStateUntouched)
{
  data_->time = 3.0;
  data_->qpos[0] = 42.0;

  EXPECT_FALSE(mujoco_bridge::resetToKeyframe(*api_, model_, data_, -1));

  EXPECT_DOUBLE_EQ(data_->time, 3.0);
  EXPECT_DOUBLE_EQ(data_->qpos[0], 42.0);
}
