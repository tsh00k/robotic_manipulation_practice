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

namespace task_executor
{

// A box or bin pose in the table plane plus its height (world frame).
struct PlanarPose
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;     // box: centre; bin: inner floor surface
  double yaw_rad = 0.0;
};

// Week 4.1 Stage 11. Numbers are the scene's (MJCF); the margin was fixed before measuring.
struct ContainmentParams
{
  double inner_half_x_m = 0.070;  // inner wall faces, half extents in the bin frame
  double inner_half_y_m = 0.065;
  double box_half_m = 0.02;
  // Box detection 3 mm + bin detection 2 mm: a corner closer than this to a wall is not
  // claimed to be inside.
  double margin_m = 0.005;
  // The box centre must be within this of floor + box_half (a box on a 12 mm wall rim is not).
  double height_tolerance_m = 0.005;
};

struct Containment
{
  bool inside = false;
  // Smallest distance from a box corner to the inner opening, in the bin frame, without the
  // margin; negative when a corner is outside. inside needs it >= margin.
  double min_clearance_m = 0.0;
  // Box centre height minus (floor + box_half).
  double height_error_m = 0.0;
};

// Is the box resting on the bin floor, all four corners inside the inner opening by at least
// the margin? Anything else (outside, on a wall, too close to a wall to tell) is not inside.
Containment boxInBin(
  const PlanarPose & box, const PlanarPose & bin, const ContainmentParams & params = {});

}  // namespace task_executor
