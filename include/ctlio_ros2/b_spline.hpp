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

#include <iostream>
#include <algorithm>
#include <vector>

#include <Eigen/Core>

#include <ctlio_ros2/types.hpp>
#include <ctlio_ros2/lie_algebra.hpp>

namespace lico {

class BSpline {
private:
  double interval_{0.1};

public:
  BSpline();

  ~BSpline();

  inline void SetInterval(double val) { interval_ = val; }

  bool FindInterval(
    double t,
    const std::vector<ControlPoint>& control_points,
    int& idx);

  Eigen::Vector4f ComputeBasis(
    double t,
    int idx);

  void ComputeBasisAndDerivatives(
    double t,
    int idx,
    Eigen::Vector4f& beta,
    Eigen::Vector4f& beta_dot,
    Eigen::Vector4f& beta_ddot);

  Eigen::Vector3f BasisToLambda(const Eigen::Vector4f& beta);

  bool ComputePose(
    double t,
    const std::vector<ControlPoint>& control_points,
    Sophus::SO3f& R,
    Eigen::Vector3f& p);

  bool ComputePoseAndDerivatives(
    double t,
    const std::vector<ControlPoint>& control_points,
    Sophus::SO3f& R,
    Eigen::Vector3f& p,
    std::array<int, 4>& idx4,
    Eigen::Matrix<float, 3, 24>& dTt_dcps);

  bool ComputeMotion(
    double t,
    const std::vector<ControlPoint>& control_points,
    Sophus::SO3f& R,
    Eigen::Vector3f& omega_body,
    Eigen::Vector3f& p,
    Eigen::Vector3f& v,
    Eigen::Vector3f& a);

  bool ComputeMotionAndDerivatives(
    double t,
    const std::vector<ControlPoint>& control_points,
    Sophus::SO3f& R,
    Eigen::Vector3f& omega_body,
    Eigen::Vector3f& p,
    Eigen::Vector3f& v,
    Eigen::Vector3f& a,
    float gravity_acc,
    std::array<int, 4>& idx4,
    Eigen::Matrix<float, 6, 24>& dr_dcps);

}; // class BSpline

} // namespace lico
