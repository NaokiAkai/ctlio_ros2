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

#include <sophus/se3.hpp>

namespace lico {

Eigen::Matrix3f Hat(const Eigen::Vector3f& v);

Eigen::Matrix3f RightJacobianSO3(const Eigen::Vector3f& phi);

Eigen::Matrix3f LeftJacobianSO3(const Eigen::Vector3f& phi);

Eigen::Matrix3f LeftJacobianInvSO3(const Eigen::Vector3f& phi);

Eigen::Matrix3f RightJacobianInvSO3(const Eigen::Vector3f& phi);

} // namespace lico
