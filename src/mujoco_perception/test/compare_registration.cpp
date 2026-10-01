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

#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <Eigen/Eigenvalues>

#include <chrono>
#include <iostream>
#include <string>

#include "mujoco_perception/geometry_pipeline.hpp"

int main(int argc, char ** argv)
{
  if (argc != 6) {
    std::cerr << "Usage: compare_registration cloud.pcd truth_x truth_y truth_z anchored\n";
    return 2;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr observed(new pcl::PointCloud<pcl::PointXYZ>());
  if (pcl::io::loadPCDFile(argv[1], *observed) != 0 || observed->size() < 20) {
    return 2;
  }
  const Eigen::Vector3d truth(std::stod(argv[2]), std::stod(argv[3]), std::stod(argv[4]));
  mujoco_perception::BoxModel box;
  box.anchor_z_to_plane = std::string(argv[5]) == "1";
  const auto start = std::chrono::steady_clock::now();
  const auto obb = mujoco_perception::estimateBoxPose(*observed, box, 0.22);
  const double obb_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto & p : *observed) {
    centroid += p.getVector3fMap().cast<double>();
  }
  centroid /= observed->size();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto & p : *observed) {
    const Eigen::Vector3d d = p.getVector3fMap().cast<double>() - centroid;
    covariance += d * d.transpose();
  }
  const auto eigenvalues = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(covariance).eigenvalues();
  const bool planar = eigenvalues[0] < eigenvalues[2] * 0.001;
  std::cout << "OBB," << obb.accepted << "," << (obb.position - truth).norm() << ","
            << obb_ms << "," << planar << "\n";

  pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>());
  for (int axis = 0; axis < 3; ++axis) {
    for (double side : {-0.02, 0.02}) {
      for (int a = 0; a <= 8; ++a) {
        for (int b = 0; b <= 8; ++b) {
          Eigen::Vector3f p;
          p[axis] = side;
          p[(axis + 1) % 3] = -0.02 + a * 0.005;
          p[(axis + 2) % 3] = -0.02 + b * 0.005;
          model->emplace_back(p.x(), p.y(), p.z());
        }
      }
    }
  }
  Eigen::Matrix4f initial = Eigen::Matrix4f::Identity();
  // The seed is derived from observation, never from the evaluation truth.
  initial.block<3, 1>(0, 3) = (obb.accepted ? obb.position : centroid).cast<float>();
  const auto run = [&](auto & registration, const char * name) {
      registration.setInputSource(model);
      registration.setInputTarget(observed);
      registration.setMaximumIterations(30);
      registration.setMaxCorrespondenceDistance(0.03);
      registration.setTransformationEpsilon(1e-8);
      pcl::PointCloud<pcl::PointXYZ> aligned;
      const auto begin = std::chrono::steady_clock::now();
      registration.align(aligned, initial);
      const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - begin).count();
      const Eigen::Vector3d center = registration.getFinalTransformation().template
        block<3, 1>(0, 3).template cast<double>();
      std::cout << name << "," << registration.hasConverged() << ","
                << (center - truth).norm() << "," << ms << "," << planar << "\n";
    };
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  run(icp, "ICP");
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
  run(gicp, "GICP");
  return 0;
}
