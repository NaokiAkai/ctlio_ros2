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

#include <Eigen/Core>

#include <ctlio_ros2/types.hpp>

namespace lico {

class ImuInitializer {
private:
  inline float SafeNorm(const Eigen::Vector3f& v) {
    const float n = v.norm();
    return std::isfinite(n) ? n : 0.0f;
  }

  Eigen::Quaternionf QuaternionFromTwoVectors(
    const Eigen::Vector3f& a_in,
    const Eigen::Vector3f& b_in);

  Eigen::Vector3f MeanOf(const std::vector<Eigen::Vector3f>& xs);

  Eigen::Vector3f StdOf(
    const std::vector<Eigen::Vector3f>& xs,
    const Eigen::Vector3f& mean);

public:
  ImuInitializer();

  ~ImuInitializer();

  void InitializeImu(
    const boost::circular_buffer<lico::ImuMeasure>& imu_buf,
    float gravity_acc,
    State& state);

}; // class ImuInitializer

} // namespace lico
