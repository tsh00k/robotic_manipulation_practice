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

#include "task_executor/bin_containment.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace task_executor
{

Containment boxInBin(
  const PlanarPose & box, const PlanarPose & bin, const ContainmentParams & params)
{
  Containment result;
  result.min_clearance_m = std::numeric_limits<double>::infinity();
  const double cb = std::cos(box.yaw_rad);
  const double sb = std::sin(box.yaw_rad);
  const double cn = std::cos(bin.yaw_rad);
  const double sn = std::sin(bin.yaw_rad);
  for (const double sx : {-1.0, 1.0}) {
    for (const double sy : {-1.0, 1.0}) {
      const double corner_x = box.x + params.box_half_m * (sx * cb - sy * sb);
      const double corner_y = box.y + params.box_half_m * (sx * sb + sy * cb);
      const double dx = corner_x - bin.x;
      const double dy = corner_y - bin.y;
      const double local_x = cn * dx + sn * dy;
      const double local_y = -sn * dx + cn * dy;
      result.min_clearance_m = std::min(
        {result.min_clearance_m, params.inner_half_x_m - std::abs(local_x),
          params.inner_half_y_m - std::abs(local_y)});
    }
  }
  result.height_error_m = box.z - (bin.z + params.box_half_m);
  const bool finite = std::isfinite(result.min_clearance_m) && std::isfinite(result.height_error_m);
  result.inside = finite && result.min_clearance_m >= params.margin_m &&
    std::abs(result.height_error_m) <= params.height_tolerance_m;
  return result;
}

}  // namespace task_executor
