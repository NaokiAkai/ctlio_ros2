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

#include <Eigen/Core>
#include <unordered_map>
#include <vector>
#include <cmath>

#include <ctlio_ros2/types.hpp>

namespace lico {

struct VoxelKey {
  int x{}, y{}, z{};

  bool operator==(const VoxelKey& other) const noexcept {
    return x == other.x && y == other.y && z == other.z;
  }
};

} // namespace lico

namespace std {

template <>
struct hash<lico::VoxelKey> {
  size_t operator()(const lico::VoxelKey& k) const noexcept {
    size_t h = 0;
    h ^= std::hash<int>{}(k.x) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= std::hash<int>{}(k.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= std::hash<int>{}(k.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};

} // namespace std

namespace lico {

class VoxelGridFilter {
 public:
  VoxelGridFilter(float voxel_size);

  ~VoxelGridFilter();

  PointCloud3f filter(const PointCloud3f& input) const;

  void filter(
    const PointCloud3f& input_cloud,
    const std::vector<float>& input_intensities,
    PointCloud3f& out_cloud,
    std::vector<float>& out_intensities) const;

  void filter(
    const PointCloud3f& input_cloud,
    const std::vector<float>& input_intensities,
    const std::vector<double>& input_stamps,
    PointCloud3f& out_cloud,
    std::vector<float>& out_intensities,
    std::vector<double>& out_stamps) const;

 private:
  float voxel_size_;
  float inv_size_;
};

} // namespace lico
