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

#pragma once

// MuJoCo side of the scene layout: read the extents the checks need out of the model,
// and apply validated poses to it. In src/ for the same reason as scene_config.hpp.

#include "mujoco_bridge/mujoco_dl.hpp"
#include "scene_config.hpp"

namespace mujoco_bridge
{

// Reads the box, bin and table extents from the model by geom/body name ("box", "bin",
// "table"). Throws std::runtime_error naming what is missing or not supported (non-box
// geoms, rotated geoms): the checks assume axis-aligned boxes.
SceneGeometry readSceneGeometry(const MujocoApi & api, const mjModel * m);

// Moves the bin body and writes the box's start pose into qpos0, every keyframe and
// d->qpos, then calls mj_forward. A reset to any keyframe therefore restores the
// configured layout. Throws std::runtime_error if the model has no bin body or no
// free-jointed box body.
//
// Must run before the first simulation step. Only meaningful for the bin scene
// (pick_place_bin_scene.xml): the default scene has no bin, and the bridge does not
// call this for it.
void applyScene(const MujocoApi & api, mjModel * m, mjData * d, const ScenePoses & poses);

}  // namespace mujoco_bridge
