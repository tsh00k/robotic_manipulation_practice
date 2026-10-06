// Copyright 2026 anby
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Replays recorded depth frames through the EXISTING box detector, offline.
//
// Why this exists (Week 4.1 Stage 5): the existing estimator (geometry_pipeline.cpp) is the
// "before" that any new initial-pose detector has to be compared with, on the same frames and
// with the same acceptance rules. Without this tool the comparison would rest on reading the
// code. It is a measurement tool, not part of the product: it is built only with
// BUILD_TESTING and is not installed.
//
// What it runs:   segmentDepth()  ->  estimateBoxPose() per candidate cluster,
//                 with default SegmentationConfig and BoxModel, i.e. what the node uses when
//                 scene.* is not involved (plane_z = 0.22 m, anchor_z_to_plane = true).
// What it leaves out on purpose, so that the comparison is a comparison of the geometry only:
//   - the robot mesh mask (at the HOME pose the arm is above every height we look at),
//   - the tracker (multi-candidate rule, temporal history) and the executor's quality gate:
//     the Python driver baseline_compare.py replays those two rules from the printed numbers,
//   - RGB, TF waiting and sample pairing (which cost the live node most of its frames; that
//     is measured separately in Week 4.1 appendix A).
//
// Input file (little endian, written by baseline_compare.py):
//   int32   frame_count
//   frame_count x ( 240 x 320 float32 )   depth in metres, NaN or <= 0 = invalid, row-major
//
// Output, one block per frame on stdout:
//   frame <i> seg=<RejectionReason name> clusters=<n>
//     cluster <c> points=<n> obb_mm=<e0> <e1> <e2> valid=<0|1> reason=<name>
//       pos=<x> <y> <z> yaw_deg=<yaw> conf=<c> resid=<m> inl=<ratio>
// (the "cluster" line is a single line; it is wrapped here only for the comment).
// obb_mm are the three sorted extents of the 3D oriented bounding box that estimateBoxPose()
// compares against the 40 mm box (reject when any exceeds 40 + 15 mm).
//
// Camera model: copied from robot_description/mujoco/franka_emika_panda/pick_place_scene.xml
// (body camera_link pos="0.5 -0.45 1.0" xyaxes="1 0 0 0 0.857 0.514", fovy=50, 320x240
// offscreen) and from the bridge's intrinsics (focal = height / (2 tan(fovy/2)),
// principal point (w-1)/2, (h-1)/2). If the MJCF camera changes, change the constants below
// and the same ones in initial_pose_eval.py together.

#include <pcl/features/moment_of_inertia_estimation.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <vector>

#include "sensor_msgs/msg/camera_info.hpp"
#include "mujoco_perception/geometry_pipeline.hpp"

namespace
{

constexpr int kWidth = 320;
constexpr int kHeight = 240;
constexpr double kFovyDeg = 50.0;

sensor_msgs::msg::CameraInfo cameraInfo()
{
  const double focal = kHeight / (2.0 * std::tan(kFovyDeg * M_PI / 180.0 / 2.0));
  const double cx = (kWidth - 1) / 2.0;
  const double cy = (kHeight - 1) / 2.0;
  sensor_msgs::msg::CameraInfo info;
  info.width = kWidth;
  info.height = kHeight;
  info.distortion_model = "plumb_bob";
  info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  info.k = {focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

// Optical frame (x right, y down, z forward) expressed in the world frame. MuJoCo's camera
// looks along its own -z with +y up, so the optical axes are (x, -y, -z) of the MuJoCo camera.
Eigen::Isometry3d worldFromOptical()
{
  const Eigen::Vector3d x_axis(1.0, 0.0, 0.0);
  const Eigen::Vector3d y_axis = Eigen::Vector3d(0.0, 0.857, 0.514).normalized();
  const Eigen::Vector3d z_axis = x_axis.cross(y_axis);
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear().col(0) = x_axis;
  result.linear().col(1) = -y_axis;
  result.linear().col(2) = -z_axis;
  result.translation() = Eigen::Vector3d(0.5, -0.45, 1.0);
  return result;
}

// Same quantity estimateBoxPose() uses for its size test: the three sorted side lengths of
// the minimum-inertia oriented bounding box of the cluster, in metres.
std::vector<double> obbExtents(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cluster)
{
  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> obb;
  obb.setInputCloud(cluster);
  obb.compute();
  pcl::PointXYZ min_point;
  pcl::PointXYZ max_point;
  pcl::PointXYZ position;
  Eigen::Matrix3f rotation;
  obb.getOBB(min_point, max_point, position, rotation);
  std::vector<double> extents = {
    std::abs(max_point.x - min_point.x), std::abs(max_point.y - min_point.y),
    std::abs(max_point.z - min_point.z)};
  std::sort(extents.begin(), extents.end());
  return extents;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::fprintf(stderr, "usage: %s <frames.bin>\n", argv[0]);
    return 2;
  }
  std::ifstream input(argv[1], std::ios::binary);
  int32_t frame_count = 0;
  if (!input.read(reinterpret_cast<char *>(&frame_count), sizeof(frame_count)) ||
    frame_count < 0)
  {
    std::fprintf(stderr, "cannot read the frame count from %s\n", argv[1]);
    return 2;
  }

  const auto info = cameraInfo();
  const auto world_from_optical = worldFromOptical();
  const mujoco_perception::SegmentationConfig segmentation_config;
  const mujoco_perception::BoxModel box_model;

  for (int frame = 0; frame < frame_count; ++frame) {
    std::vector<float> depth(static_cast<std::size_t>(kWidth) * kHeight);
    if (!input.read(reinterpret_cast<char *>(depth.data()), depth.size() * sizeof(float))) {
      std::fprintf(stderr, "file ended inside frame %d\n", frame);
      return 2;
    }
    const auto segmentation =
      mujoco_perception::segmentDepth(depth, info, world_from_optical, segmentation_config);
    std::printf(
      "frame %d seg=%s clusters=%zu\n", frame,
      mujoco_perception::rejectionReasonName(segmentation.rejection),
      segmentation.candidate_clusters.size());

    for (std::size_t c = 0; c < segmentation.candidate_clusters.size(); ++c) {
      const auto & cluster = segmentation.candidate_clusters[c];
      const auto extents = obbExtents(cluster);
      const auto estimate =
        mujoco_perception::estimateBoxPose(*cluster, box_model, segmentation_config.plane_z_m);
      const auto & q = estimate.orientation;
      const double yaw_deg =
        std::atan2(
        2.0 * (q.w() * q.z() + q.x() * q.y()),
        1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z())) *
        180.0 / M_PI;
      std::printf(
        "  cluster %zu points=%zu obb_mm=%.1f %.1f %.1f valid=%d reason=%s "
        "pos=%.5f %.5f %.5f yaw_deg=%.3f conf=%.4f resid=%.5f inl=%.4f\n",
        c, estimate.point_count, extents[0] * 1000.0, extents[1] * 1000.0,
        extents[2] * 1000.0, estimate.geometry_valid ? 1 : 0,
        mujoco_perception::rejectionReasonName(estimate.rejection), estimate.position.x(),
        estimate.position.y(), estimate.position.z(), yaw_deg, estimate.confidence,
        estimate.residual_m, estimate.inlier_ratio);
    }
  }
  return 0;
}
