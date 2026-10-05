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

#include "scene_ops.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace mujoco_bridge
{

namespace
{

constexpr const char * kBoxName = "box";
constexpr const char * kBinName = "bin";
constexpr const char * kTableName = "table";
constexpr double kGeomTolerance = 1e-9;

int requireId(const MujocoApi & api, const mjModel * m, int type, const char * name)
{
  const int id = api.name2id(m, type, name);
  if (id < 0) {
    throw std::runtime_error(std::string("scene requires `") + name + "` in the model");
  }
  return id;
}

void requireAxisAlignedBox(const mjModel * m, int geom, const char * what)
{
  const mjtNum * q = m->geom_quat + 4 * geom;
  const bool identity = std::abs(q[0] - 1.0) < kGeomTolerance &&
    std::abs(q[1]) < kGeomTolerance && std::abs(q[2]) < kGeomTolerance &&
    std::abs(q[3]) < kGeomTolerance;
  if (m->geom_type[geom] != mjGEOM_BOX || !identity) {
    throw std::runtime_error(
            std::string(what) + " must be an unrotated box geom for the scene checks");
  }
}

Eigen::Vector3d vec3(const mjtNum * v)
{
  return Eigen::Vector3d(v[0], v[1], v[2]);
}

}  // namespace

SceneGeometry readSceneGeometry(const MujocoApi & api, const mjModel * m)
{
  SceneGeometry result;

  const int box_geom = requireId(api, m, mjOBJ_GEOM, kBoxName);
  requireAxisAlignedBox(m, box_geom, "`box`");
  if (std::abs(m->geom_pos[3 * box_geom]) > kGeomTolerance ||
    std::abs(m->geom_pos[3 * box_geom + 1]) > kGeomTolerance ||
    std::abs(m->geom_pos[3 * box_geom + 2]) > kGeomTolerance)
  {
    throw std::runtime_error("`box` geom must be centred on its body");
  }
  result.box_half_extents = vec3(m->geom_size + 3 * box_geom);

  const int table_geom = requireId(api, m, mjOBJ_GEOM, kTableName);
  requireAxisAlignedBox(m, table_geom, "`table`");
  if (m->geom_bodyid[table_geom] != 0) {
    throw std::runtime_error("`table` must be a world geom");
  }
  const Eigen::Vector3d table_pos = vec3(m->geom_pos + 3 * table_geom);
  const Eigen::Vector3d table_half = vec3(m->geom_size + 3 * table_geom);
  result.table_top_z = table_pos.z() + table_half.z();
  result.table_min_xy = (table_pos - table_half).head<2>();
  result.table_max_xy = (table_pos + table_half).head<2>();

  const int bin_body = requireId(api, m, mjOBJ_BODY, kBinName);
  result.bin_min = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  result.bin_max = -result.bin_min;
  int bin_geoms = 0;
  for (int g = 0; g < m->ngeom; ++g) {
    if (m->geom_bodyid[g] != bin_body) {
      continue;
    }
    requireAxisAlignedBox(m, g, "bin geoms");
    const Eigen::Vector3d center = vec3(m->geom_pos + 3 * g);
    const Eigen::Vector3d half = vec3(m->geom_size + 3 * g);
    result.bin_min = result.bin_min.cwiseMin(center - half);
    result.bin_max = result.bin_max.cwiseMax(center + half);
    ++bin_geoms;
  }
  if (bin_geoms == 0) {
    throw std::runtime_error("`bin` body has no geoms");
  }
  return result;
}

void applyScene(const MujocoApi & api, mjModel * m, mjData * d, const ScenePoses & poses)
{
  const int bin_body = requireId(api, m, mjOBJ_BODY, kBinName);
  const int box_body = requireId(api, m, mjOBJ_BODY, kBoxName);

  if (m->body_jntnum[box_body] != 1 || m->jnt_type[m->body_jntadr[box_body]] != mjJNT_FREE) {
    throw std::runtime_error("scene requires `box` to have exactly one free joint");
  }
  const int qpos_adr = m->jnt_qposadr[m->body_jntadr[box_body]];

  // Bin: a static body, so its pose is body_pos/body_quat relative to the world.
  const Eigen::Quaterniond bin_q(poses.bin.linear());
  for (int i = 0; i < 3; ++i) {
    m->body_pos[3 * bin_body + i] = poses.bin.translation()[i];
  }
  const double bin_quat[] = {bin_q.w(), bin_q.x(), bin_q.y(), bin_q.z()};
  for (int i = 0; i < 4; ++i) {
    m->body_quat[4 * bin_body + i] = bin_quat[i];
  }

  // Box: its free-joint qpos is (x, y, z, qw, qx, qy, qz). Written everywhere a reset
  // could read it from, not only the keyframe the bridge resets to by default.
  const Eigen::Quaterniond box_q(poses.box.linear());
  const double box_pose[] = {
    poses.box.translation().x(), poses.box.translation().y(), poses.box.translation().z(),
    box_q.w(), box_q.x(), box_q.y(), box_q.z()};
  for (int i = 0; i < 7; ++i) {
    m->qpos0[qpos_adr + i] = box_pose[i];
    d->qpos[qpos_adr + i] = box_pose[i];
    for (int k = 0; k < m->nkey; ++k) {
      m->key_qpos[k * m->nq + qpos_adr + i] = box_pose[i];
    }
  }

  api.forward(m, d);
}

}  // namespace mujoco_bridge
