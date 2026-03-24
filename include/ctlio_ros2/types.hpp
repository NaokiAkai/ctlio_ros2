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

#include <vector>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <iostream>
#include <numeric>
#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/se3.hpp>

#include <nanoflann.hpp>

namespace lico {

using Point3f = Eigen::Vector3f;
using PointCloud3f = std::vector<Point3f, Eigen::aligned_allocator<Point3f>>;

struct State {
  Sophus::SO3f R;
  Eigen::Vector3f p;
  Eigen::Vector3f v;
  Eigen::Vector3f w;
  Eigen::Vector3f bg;
  Eigen::Vector3f ba;

  State() {
    R = Sophus::SO3f();
    p.setZero();
    v.setZero();
    w.setZero();
    bg.setZero();
    ba.setZero();
  }
};

struct ImuMeasure {
  double stamp;
  double dt;
  Eigen::Vector3f gyro;
  Eigen::Vector3f acc;

  ImuMeasure() {
    stamp = 0.0;
    dt = 0.0;
    gyro.setZero();
    acc.setZero();
  }
};

struct ControlPoint {
  double stamp;
  Sophus::SO3f R;
  Eigen::Vector3f p;

  ControlPoint() {
    stamp = 0.0;
    R = Sophus::SO3f();
    p.setZero();
  }
};

struct PointCloudAdaptor {
  const PointCloud3f& pts;

  PointCloudAdaptor(const PointCloud3f& points)
    : pts(points) {}

  inline size_t kdtree_get_point_count() const {
    return pts.size();
  }

  inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
    return pts[idx][dim];
  }

  template <class BBOX>
  bool kdtree_get_bbox(BBOX&) const {
    return false;
  }
};

using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
  nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor>,
  PointCloudAdaptor,
  3
>;

} // namespace lico
