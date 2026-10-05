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

#include <cmath>
#include <stdexcept>
#include <vector>

#include "mujoco_bridge/state_ops.hpp"
#include "scene_ops.hpp"

namespace mujoco_bridge
{
namespace
{

mjModel * load(const MujocoApi & api, const char * path)
{
  char error[1024] = {0};
  mjModel * model = api.loadXML(path, nullptr, error, sizeof(error));
  EXPECT_NE(model, nullptr) << path << ": " << error;
  return model;
}

// The bin scene (pick_place_bin_scene.xml): what the bridge loads when scene.enabled.
class SceneOps : public ::testing::Test
{
protected:
  void SetUp() override
  {
    api_ = &loadMujocoApi();
    model_ = load(*api_, PICK_PLACE_BIN_SCENE_MJCF_PATH);
    ASSERT_NE(model_, nullptr);
    data_ = api_->makeData(model_);
    ASSERT_NE(data_, nullptr);
    key_ = api_->name2id(model_, mjOBJ_KEY, "pick_place_home");
    bin_ = api_->name2id(model_, mjOBJ_BODY, "bin");
    box_ = api_->name2id(model_, mjOBJ_BODY, "box");
    ASSERT_GE(key_, 0);
    ASSERT_GE(bin_, 0);
    ASSERT_GE(box_, 0);
    box_qpos_adr_ = model_->jnt_qposadr[model_->body_jntadr[box_]];
  }

  void TearDown() override
  {
    api_->deleteData(data_);
    api_->deleteModel(model_);
  }

  std::vector<int> binGeoms() const
  {
    std::vector<int> geoms;
    for (int g = 0; g < model_->ngeom; ++g) {
      if (model_->geom_bodyid[g] == bin_) {
        geoms.push_back(g);
      }
    }
    return geoms;
  }

  // Free-joint qpos is (x, y, z, qw, qx, qy, qz); identity orientation, at rest.
  void dropBoxAt(double x, double y, double z)
  {
    const double pose[] = {x, y, z, 1.0, 0.0, 0.0, 0.0};
    for (int i = 0; i < 7; ++i) {
      data_->qpos[box_qpos_adr_ + i] = pose[i];
    }
    const int dof = model_->jnt_dofadr[model_->body_jntadr[box_]];
    for (int i = 0; i < 6; ++i) {
      data_->qvel[dof + i] = 0.0;
    }
    api_->forward(model_, data_);
  }

  void stepFor(double seconds)
  {
    const int steps = static_cast<int>(std::ceil(seconds / model_->opt.timestep));
    for (int i = 0; i < steps; ++i) {
      api_->step(model_, data_);
    }
  }

  double boxZ() const {return data_->xpos[3 * box_ + 2];}

  SceneRequest legacyRequest() const
  {
    SceneRequest request;
    request.box.x = kDefaultBoxX;
    request.box.y = kDefaultBoxY;
    request.box.z = 0.241;
    request.bin.x = kDefaultBinX;
    request.bin.y = kDefaultBinY;
    request.bin.z = 0.227;
    return request;
  }

  // The scene the node builds with no pose parameter set: legacy box position, bin at
  // its default pose.
  ScenePoses legacyPoses() const
  {
    return resolveScene(legacyRequest(), readSceneGeometry(*api_, model_));
  }

  const MujocoApi * api_ = nullptr;
  mjModel * model_ = nullptr;
  mjData * data_ = nullptr;
  int key_ = -1;
  int bin_ = -1;
  int box_ = -1;
  int box_qpos_adr_ = -1;
};

}  // namespace

TEST_F(SceneOps, ReadsTheExtentsFromTheModel)
{
  const SceneGeometry g = readSceneGeometry(*api_, model_);
  EXPECT_NEAR((g.box_half_extents - Eigen::Vector3d(0.02, 0.02, 0.02)).norm(), 0.0, 1e-12);
  // Floor 6 mm thick below the inner surface, walls 12 mm above it, outer half sizes
  // 0.076 x 0.071.
  EXPECT_NEAR((g.bin_min - Eigen::Vector3d(-0.076, -0.071, -0.006)).norm(), 0.0, 1e-12);
  EXPECT_NEAR((g.bin_max - Eigen::Vector3d(0.076, 0.071, 0.012)).norm(), 0.0, 1e-12);
  EXPECT_NEAR(g.table_top_z, 0.22, 1e-12);
  EXPECT_NEAR((g.table_min_xy - Eigen::Vector2d(0.2, -0.4)).norm(), 0.0, 1e-12);
  EXPECT_NEAR((g.table_max_xy - Eigen::Vector2d(0.8, 0.4)).norm(), 0.0, 1e-12);
}

TEST_F(SceneOps, TheBinSceneKeepsTheLegacyBoxStartPose)
{
  const double * q = model_->key_qpos + key_ * model_->nq + box_qpos_adr_;
  EXPECT_NEAR(q[0], 0.5, 1e-12);
  EXPECT_NEAR(q[1], 0.0, 1e-12);
  EXPECT_NEAR(q[2], 0.241, 1e-12);
}

TEST_F(SceneOps, TheBinIsCollidableAndVisibleWithoutAnyRuntimeSwitching)
{
  const std::vector<int> geoms = binGeoms();
  ASSERT_EQ(geoms.size(), 5U);  // floor and four walls
  for (const int g : geoms) {
    EXPECT_EQ(model_->geom_contype[g], 1) << "geom " << g;
    EXPECT_EQ(model_->geom_conaffinity[g], 1) << "geom " << g;
    EXPECT_EQ(model_->geom_group[g], 0) << "geom " << g;
  }
  EXPECT_EQ(model_->body_contype[bin_], 1);
  EXPECT_EQ(model_->body_conaffinity[bin_], 1);
}

TEST_F(SceneOps, AnEnabledBinCatchesTheBoxOnItsFloor)
{
  applyScene(*api_, model_, data_, legacyPoses());
  ASSERT_TRUE(resetToKeyframe(*api_, model_, data_, key_));
  dropBoxAt(0.5, 0.3, 0.30);
  stepFor(1.0);
  // Floor top at the bin origin, z = 0.227, plus half size 0.02: 7 mm above where the
  // same drop ends up on the bare table (DefaultScene.ADropAboveThePlaceMarkerHitsTheTable).
  EXPECT_NEAR(boxZ(), 0.247, 0.0015);
}

TEST_F(SceneOps, AppliesTheBinPose)
{
  const SceneGeometry geometry = readSceneGeometry(*api_, model_);
  SceneRequest request = legacyRequest();
  request.bin.x = 0.62;
  request.bin.y = -0.21;
  request.bin.yaw = 0.6;
  request.bin.roll = 0.1;
  request.bin.pitch = -0.05;
  request.bin.z = autoSupportZ(
    rotationFromRpy(0.1, -0.05, 0.6), geometry.bin_min, geometry.bin_max, 0.22);
  const ScenePoses poses = resolveScene(request, geometry);
  applyScene(*api_, model_, data_, poses);

  // applyScene calls mj_forward, so the world pose is available straight away.
  EXPECT_NEAR(data_->xpos[3 * bin_], 0.62, 1e-12);
  EXPECT_NEAR(data_->xpos[3 * bin_ + 1], -0.21, 1e-12);
  EXPECT_NEAR(data_->xpos[3 * bin_ + 2], request.bin.z, 1e-12);
  const Eigen::Quaterniond actual(
    data_->xquat[4 * bin_], data_->xquat[4 * bin_ + 1], data_->xquat[4 * bin_ + 2],
    data_->xquat[4 * bin_ + 3]);
  EXPECT_NEAR(actual.angularDistance(Eigen::Quaterniond(poses.bin.linear())), 0.0, 1e-9);
}

TEST_F(SceneOps, AMovedBinCatchesTheBoxWhereItNowIs)
{
  const SceneGeometry geometry = readSceneGeometry(*api_, model_);
  SceneRequest request = legacyRequest();
  request.bin.x = 0.62;
  request.bin.y = -0.21;
  applyScene(*api_, model_, data_, resolveScene(request, geometry));
  ASSERT_TRUE(resetToKeyframe(*api_, model_, data_, key_));

  dropBoxAt(0.62, -0.21, 0.30);
  stepFor(1.0);
  EXPECT_NEAR(boxZ(), 0.247, 0.0015);  // on the moved bin's floor

  dropBoxAt(0.5, 0.3, 0.30);  // where the bin used to be
  stepFor(1.0);
  EXPECT_NEAR(boxZ(), 0.24, 0.0015);  // bare table
}

TEST_F(SceneOps, ResetRestoresTheConfiguredBoxPoseEveryTime)
{
  const SceneGeometry geometry = readSceneGeometry(*api_, model_);
  SceneRequest request = legacyRequest();
  request.box.x = 0.45;
  request.box.y = -0.1;
  request.box.roll = 0.3;
  request.box.pitch = 0.2;
  request.box.yaw = 0.7;
  request.box.z = autoSupportZ(
    rotationFromRpy(0.3, 0.2, 0.7), -geometry.box_half_extents, geometry.box_half_extents, 0.22);
  const ScenePoses poses = resolveScene(request, geometry);
  applyScene(*api_, model_, data_, poses);

  const Eigen::Quaterniond expected(poses.box.linear());
  for (int round = 0; round < 3; ++round) {
    // Wander off: move the box, let it fall and the arm settle.
    dropBoxAt(0.7, 0.2, 0.5);
    stepFor(0.3);

    ASSERT_TRUE(resetToKeyframe(*api_, model_, data_, key_));
    EXPECT_NEAR(data_->xpos[3 * box_], 0.45, 1e-12) << "round " << round;
    EXPECT_NEAR(data_->xpos[3 * box_ + 1], -0.1, 1e-12) << "round " << round;
    EXPECT_NEAR(data_->xpos[3 * box_ + 2], request.box.z, 1e-12) << "round " << round;
    const Eigen::Quaterniond actual(
      data_->xquat[4 * box_], data_->xquat[4 * box_ + 1], data_->xquat[4 * box_ + 2],
      data_->xquat[4 * box_ + 3]);
    EXPECT_NEAR(actual.angularDistance(expected), 0.0, 1e-9) << "round " << round;
  }
}

TEST_F(SceneOps, EveryKeyframeCarriesTheConfiguredBoxPose)
{
  SceneRequest request = legacyRequest();
  request.box.x = 0.45;
  request.box.y = -0.1;
  applyScene(*api_, model_, data_, resolveScene(request, readSceneGeometry(*api_, model_)));
  // Includes panda's own `home` keyframe, whose box tail MuJoCo zero-padded to the
  // world origin (z = 0, below the table). A reset to it used to put the box there; with
  // the scene enabled it now lands on the configured pose like any other.
  ASSERT_GE(model_->nkey, 2);
  for (int k = 0; k < model_->nkey; ++k) {
    const double * q = model_->key_qpos + k * model_->nq + box_qpos_adr_;
    EXPECT_NEAR(q[0], 0.45, 1e-12) << "keyframe " << k;
    EXPECT_NEAR(q[1], -0.1, 1e-12) << "keyframe " << k;
    EXPECT_NEAR(q[2], 0.241, 1e-12) << "keyframe " << k;
    EXPECT_NEAR(q[3], 1.0, 1e-12) << "keyframe " << k;
  }
  EXPECT_NEAR(model_->qpos0[box_qpos_adr_], 0.45, 1e-12);
}

TEST_F(SceneOps, ApplyingTheLegacyLayoutLeavesTheResetKeyframeUnchanged)
{
  // The default scene.enabled=true layout must equal what the MJCF already has for the
  // box, so enabling the scene without parameters only adds the bin. Checked on the
  // keyframe the bridge resets to by default, for every qpos entry (arm included).
  //
  // Not checked on the vendor `home` keyframe: its box tail was zero-padded by MuJoCo to
  // a pose at the world origin, below the table top (see EveryKeyframeCarriesThe
  // ConfiguredBoxPose), and the scene overwrites that too.
  const double * row = model_->key_qpos + key_ * model_->nq;
  const std::vector<double> before(row, row + model_->nq);
  applyScene(*api_, model_, data_, legacyPoses());
  for (int i = 0; i < model_->nq; ++i) {
    EXPECT_NEAR(row[i], before[i], 1e-12) << "key_qpos[" << i << "]";
  }
}

// The default scene (pick_place_scene.xml): what the bridge loads when scene.enabled is
// false, i.e. the legacy one.
class DefaultScene : public ::testing::Test
{
protected:
  void SetUp() override
  {
    api_ = &loadMujocoApi();
    model_ = load(*api_, PICK_PLACE_SCENE_MJCF_PATH);
    ASSERT_NE(model_, nullptr);
    data_ = api_->makeData(model_);
    ASSERT_NE(data_, nullptr);
  }

  void TearDown() override
  {
    api_->deleteData(data_);
    api_->deleteModel(model_);
  }

  const MujocoApi * api_ = nullptr;
  mjModel * model_ = nullptr;
  mjData * data_ = nullptr;
};

TEST_F(DefaultScene, HasNoBinAtAll)
{
  EXPECT_LT(api_->name2id(model_, mjOBJ_BODY, "bin"), 0);
  EXPECT_THROW(readSceneGeometry(*api_, model_), std::runtime_error);
}

TEST_F(DefaultScene, ADropAboveThePlaceMarkerHitsTheTable)
{
  const int key = api_->name2id(model_, mjOBJ_KEY, "pick_place_home");
  const int box = api_->name2id(model_, mjOBJ_BODY, "box");
  ASSERT_GE(key, 0);
  ASSERT_GE(box, 0);
  ASSERT_TRUE(resetToKeyframe(*api_, model_, data_, key));
  const int adr = model_->jnt_qposadr[model_->body_jntadr[box]];
  data_->qpos[adr] = 0.5;
  data_->qpos[adr + 1] = 0.3;
  data_->qpos[adr + 2] = 0.30;
  api_->forward(model_, data_);
  const int steps = static_cast<int>(std::ceil(1.0 / model_->opt.timestep));
  for (int i = 0; i < steps; ++i) {
    api_->step(model_, data_);
  }
  EXPECT_NEAR(data_->xpos[3 * box + 2], 0.24, 0.0015);
}

// The bin scene is the default scene plus the bin and nothing else: same joints, same
// keyframes (all of qpos, so the reset state is the same), one more body, five more geoms.
TEST(SceneModels, TheBinSceneIsTheDefaultSceneWithOnlyTheBinAdded)
{
  const MujocoApi & api = loadMujocoApi();
  mjModel * base = load(api, PICK_PLACE_SCENE_MJCF_PATH);
  mjModel * bin = load(api, PICK_PLACE_BIN_SCENE_MJCF_PATH);
  ASSERT_NE(base, nullptr);
  ASSERT_NE(bin, nullptr);

  EXPECT_EQ(bin->nbody, base->nbody + 1);
  EXPECT_EQ(bin->ngeom, base->ngeom + 5);
  EXPECT_EQ(bin->nq, base->nq);
  EXPECT_EQ(bin->nv, base->nv);
  EXPECT_EQ(bin->nu, base->nu);
  EXPECT_EQ(bin->njnt, base->njnt);
  EXPECT_EQ(bin->nkey, base->nkey);
  ASSERT_EQ(bin->nq, base->nq);
  ASSERT_EQ(bin->nkey, base->nkey);
  for (int i = 0; i < base->nkey * base->nq; ++i) {
    EXPECT_EQ(bin->key_qpos[i], base->key_qpos[i]) << "key_qpos[" << i << "]";
  }
  for (int i = 0; i < base->nq; ++i) {
    EXPECT_EQ(bin->qpos0[i], base->qpos0[i]) << "qpos0[" << i << "]";
  }
  api.deleteModel(bin);
  api.deleteModel(base);
}

}  // namespace mujoco_bridge
