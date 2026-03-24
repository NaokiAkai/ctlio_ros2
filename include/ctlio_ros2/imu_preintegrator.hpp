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

#include <ctlio_ros2/types.hpp>
#include <ctlio_ros2/lie_algebra.hpp>

namespace lico {

class ImuPreintegrator {
 public:
  ImuPreintegrator();

  ~ImuPreintegrator();

  void Preintegration(
    const std::vector<ImuMeasure>& imu_measures,
    float gravity_acc,
    double control_points_interval,
    State& state,
    std::vector<ControlPoint>& control_points);

  void Preintegration(
    const ImuMeasure& imu_measure,
    float gravity_acc,
    State& state);

  inline void SetPrevControlPointStamp(double val) { prev_control_point_stamp_ = val; }

  inline double GetPrevControlPointStamp() const { return prev_control_point_stamp_; }

  float GetPreintegrationTime() const {
    return preint_time_;
  }

 private:
  double preint_time_{-1.0};
  double prev_control_point_stamp_{-1.0};
};

} // namespace lico
