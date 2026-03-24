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

#include <ctlio_ros2/voxel_grid_filter.hpp>

namespace lico {

VoxelGridFilter::VoxelGridFilter(float voxel_size) {
  voxel_size_ = voxel_size;
  inv_size_ = 1.0f / voxel_size;
}

VoxelGridFilter::~VoxelGridFilter() {

}

PointCloud3f VoxelGridFilter::filter(const PointCloud3f& input) const {
  using Accum = std::pair<Point3f, int>;
  std::unordered_map<VoxelKey, Accum> voxels;

  // Assume the output cloud size is approximately one-fourth of the input size.
  voxels.reserve(input.size() / 4);

  for (const auto& p : input) {
    VoxelKey key{
      static_cast<int>(std::floor(p.x() * inv_size_)),
      static_cast<int>(std::floor(p.y() * inv_size_)),
      static_cast<int>(std::floor(p.z() * inv_size_)) };

    auto& [sum, cnt] = voxels[key];
    if (cnt == 0) {
      sum.setZero();
    }
    sum += p;
    cnt++;
  }

  PointCloud3f output;
  output.reserve(voxels.size());
  for (const auto& [k, acc] : voxels) {
    const auto& [sum, cnt] = acc;
    output.emplace_back(sum / static_cast<float>(cnt));
  }

  return output;
}

void VoxelGridFilter::filter(
  const PointCloud3f& input_cloud,
  const std::vector<float>& input_intensities,
  PointCloud3f& out_cloud,
  std::vector<float>& out_intensities) const
{
  struct Accum {
    Point3f sum;
    float intensity_sum = 0.0f;
    int cnt = 0;
  };

  std::unordered_map<VoxelKey, Accum> voxels;
  voxels.reserve(input_cloud.size() / 4);

  for (size_t i = 0; i < input_cloud.size(); ++i) {
    const auto& p = input_cloud[i];
    const float intensity = input_intensities[i];

    VoxelKey key{
      static_cast<int>(std::floor(p.x() * inv_size_)),
      static_cast<int>(std::floor(p.y() * inv_size_)),
      static_cast<int>(std::floor(p.z() * inv_size_))};

    auto& acc = voxels[key];
    if (acc.cnt == 0) {
      acc.sum.setZero();
    }
    acc.sum += p;
    acc.intensity_sum += intensity;
    acc.cnt++;
  }

  out_cloud.clear();
  out_intensities.clear();
  out_cloud.reserve(voxels.size());
  out_intensities.reserve(voxels.size());

  for (const auto& [key, acc] : voxels) {
    const float inv_cnt = 1.0f / static_cast<float>(acc.cnt);
    out_cloud.emplace_back(acc.sum * inv_cnt);
    out_intensities.emplace_back(acc.intensity_sum * inv_cnt);
  }
}

void VoxelGridFilter::filter(
  const PointCloud3f& input_cloud,
  const std::vector<float>& input_intensities,
  const std::vector<double>& input_stamps,
  PointCloud3f& out_cloud,
  std::vector<float>& out_intensities,
  std::vector<double>& out_stamps) const
{
  if (input_cloud.size() != input_intensities.size() ||
      input_cloud.size() != input_stamps.size()) {
    throw std::runtime_error("VoxelGridFilter::filter: input size mismatch");
  }

  struct Best {
    Point3f p;
    float intensity = 0.0f;
    double stamp = -1.0;
    bool valid = false;
  };

  std::unordered_map<VoxelKey, Best> voxels;
  voxels.reserve(input_cloud.size() / 4);

  for (size_t i = 0; i < input_cloud.size(); ++i) {
    const auto& p = input_cloud[i];
    const float intensity = input_intensities[i];
    const double stamp = input_stamps[i];

    VoxelKey key{
      static_cast<int>(std::floor(p.x() * inv_size_)),
      static_cast<int>(std::floor(p.y() * inv_size_)),
      static_cast<int>(std::floor(p.z() * inv_size_))};

    auto& best = voxels[key];
    if (!best.valid || stamp > best.stamp) {
      best.p = p;
      best.intensity = intensity;
      best.stamp = stamp;
      best.valid = true;
    }
  }

  out_cloud.clear();
  out_intensities.clear();
  out_stamps.clear();

  out_cloud.reserve(voxels.size());
  out_intensities.reserve(voxels.size());
  out_stamps.reserve(voxels.size());

  for (const auto& [key, best] : voxels) {
    if (!best.valid) continue;
    out_cloud.emplace_back(best.p);
    out_intensities.emplace_back(best.intensity);
    out_stamps.emplace_back(best.stamp);
  }
}

} // namespace lico
