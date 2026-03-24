/*
 * ctlio_ros2
 * Copyright (c) 2026 Naoki Akai
 *
 * This file is part of ctlio_ros2.
 *
 * This source code is licensed under the terms described in the LICENSE file
 * located in the root directory of this repository.
 *
 * For non-commercial use only. Commercial use requires prior written permission.
 */

#pragma once

#include <boost/circular_buffer.hpp>

#include <Eigen/Dense>

#include <ctlio_ros2/types.hpp>
#include <ctlio_ros2/voxel_grid_filter.hpp>

namespace lico {

class NormalMap {
 public:
  NormalMap();

  ~NormalMap();

  void AddAlignedScanCloud(const PointCloud3f& aligned_scan_cloud);

  void FindCorrespondence(
    const Eigen::Vector3f& query,
    float max_correspondence_dist,
    Eigen::Vector3f& target,
    Eigen::Vector3f& normal,
    bool& has_correspondence,
    bool& is_normal_valid);

  void SetMaxKeyframeSize(size_t size) {
    aligned_scan_clouds_ = boost::circular_buffer<PointCloud3f>(size);
  }

  void SetFilterSize(float size) {
    filter_size_ = size;
  }

  void SetNumNormalPoints(size_t num) {
    num_normal_points_ = num;
  }

  void SetNormalEigenValThreshhold(float th) {
    normal_eigen_val_thresh_ = th;
  }

  const PointCloud3f& GetMapCloud() const {
    return filtered_map_cloud_;
  }

  void GetActiveMapCloud(PointCloud3f& cloud) const;

 private:
  boost::circular_buffer<PointCloud3f> aligned_scan_clouds_;

  PointCloud3f filtered_map_cloud_;
  std::unique_ptr<PointCloudAdaptor> adaptor_;
  std::unique_ptr<KDTree> kdtree_;

  std::vector<bool> normal_computed_flags_;
  std::vector<bool> normal_valid_flags_;
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> normals_;

  float filter_size_{0.1f};
  size_t num_normal_points_{10};
  float normal_eigen_val_thresh_{0.05f};
};

} // namespace lico
