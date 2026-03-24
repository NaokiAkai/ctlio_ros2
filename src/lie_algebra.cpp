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

#include <ctlio_ros2/lie_algebra.hpp>

namespace lico {

Eigen::Matrix3f Hat(const Eigen::Vector3f& v) {
  Eigen::Matrix3f m;
  m << 0.0f, -v.z(),  v.y(),
       v.z(), 0.0f, -v.x(),
      -v.y(), v.x(), 0.0f;
  return m;
}
  
Eigen::Matrix3f RightJacobianSO3(const Eigen::Vector3f& phi) {
  Eigen::Matrix3f Jr = Eigen::Matrix3f::Identity();
  const float t = phi.norm();

  const Eigen::Matrix3f K = Sophus::SO3f::hat(phi);
  if (t < 1e-5f) {
    Jr -= 0.5f * K + (1.0f / 6.0f) * K * K;
  } else {
    const float t2 = t * t;
    const float t3 = t2 * t;
    Jr -= (1.0f - std::cos(t)) / t2 * K + (t - std::sin(t)) / t3 * K * K;
  }

  return Jr;
}

Eigen::Matrix3f LeftJacobianSO3(const Eigen::Vector3f& phi) {
  return RightJacobianSO3(-phi);
}

Eigen::Matrix3f LeftJacobianInvSO3(const Eigen::Vector3f& phi) {
  const float t = phi.norm();
  const Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
  const Eigen::Matrix3f S = Sophus::SO3f::hat(phi);

  if (t < 1e-5f) {
    return I - 0.5f * S;
  }

  const float t2 = t * t;
  const float cot = 1.0f / tan(0.5f * t);

  return I - 0.5f * S + (1.0f - t * cot / 2.0f) / t2 * S * S;
}

Eigen::Matrix3f RightJacobianInvSO3(const Eigen::Vector3f& phi) {
  return LeftJacobianInvSO3(-phi);
}

} // namespace lico
