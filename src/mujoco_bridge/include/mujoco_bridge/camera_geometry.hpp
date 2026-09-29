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

#include <cmath>
#include <limits>

namespace mujoco_bridge
{

struct CameraPoint
{
  double x;
  double y;
  double z;
};

inline double metricDepth(double buffer_depth, double near_m, double far_m)
{
  if (!std::isfinite(buffer_depth) || buffer_depth < 0.0 || buffer_depth >= 1.0 ||
    near_m <= 0.0 || far_m <= near_m)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return near_m * far_m / (far_m - buffer_depth * (far_m - near_m));
}

inline CameraPoint backproject(
  double u, double v, double depth_m, double fx, double fy, double cx, double cy)
{
  return {(u - cx) * depth_m / fx, (v - cy) * depth_m / fy, depth_m};
}

}  // namespace mujoco_bridge
