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

#include <ctlio_ros2/imu_preintegrator.hpp>

namespace lico {

ImuPreintegrator::ImuPreintegrator() {

}

ImuPreintegrator::~ImuPreintegrator() {

}

void ImuPreintegrator::Preintegration(
  const std::vector<ImuMeasure>& imu_measures,
  float gravity_acc,
  double control_points_interval,
  State& state,
  std::vector<ControlPoint>& control_points)
{
  const Eigen::Vector3f bg = state.bg;
  const Eigen::Vector3f ba = state.ba;
  const Eigen::Vector3f gacc(0.0f, 0.0f, gravity_acc);
  preint_time_ = 0.0;

  Sophus::SO3f R = state.R;
  Eigen::Vector3f p = state.p;
  Eigen::Vector3f v = state.v;
  Eigen::Vector3f gyro_sum = Eigen::Vector3f::Zero();
  Eigen::Vector3f acc_sum = Eigen::Vector3f::Zero();

  if (prev_control_point_stamp_ < 0.0) {
    const double stamp = imu_measures[0].stamp;
    ControlPoint cp;
    cp.stamp = stamp;
    cp.R = R;
    cp.p = p;
    control_points.emplace_back(cp);
    prev_control_point_stamp_ = stamp;
  }

  for (size_t i = 0; i < imu_measures.size(); ++i) {
    const ImuMeasure& m = imu_measures[i];
    const double dt = m.dt;
    const Eigen::Vector3f gyro = m.gyro - bg;
    const Eigen::Vector3f acc = m.acc - ba;
    gyro_sum += gyro;
    acc_sum += acc;

    const Eigen::Vector3f phi = gyro * dt;
    const Sophus::SO3f dR = Sophus::SO3f::exp(phi);
    R = R * dR;
    const Eigen::Vector3f wacc = R * acc - gacc;
    const Eigen::Vector3f v_prev = v;
    v = v_prev + wacc * dt;
    p = p + v_prev * dt + 0.5f * wacc * dt * dt;
    preint_time_ += dt;

    const double stamp = m.stamp;
    if (stamp - prev_control_point_stamp_ > control_points_interval) {
      ControlPoint cp;
      cp.stamp = stamp;
      cp.R = R;
      cp.p = p;
      control_points.emplace_back(cp);
      prev_control_point_stamp_ = stamp;
    }
  }

  state.R = R;
  state.p = p;
  state.v = v;
}

void ImuPreintegrator::Preintegration(
  const ImuMeasure& imu_measure,
  float gravity_acc,
  State& state)
{
  const Eigen::Vector3f bg = state.bg;
  const Eigen::Vector3f ba = state.ba;
  const Eigen::Vector3f gacc(0.0f, 0.0f, gravity_acc);

  Sophus::SO3f R = state.R;
  Eigen::Vector3f p = state.p;
  Eigen::Vector3f v = state.v;

  const double dt = imu_measure.dt;
  const Eigen::Vector3f gyro = imu_measure.gyro - bg;
  const Eigen::Vector3f acc = imu_measure.acc - ba;

  const Eigen::Vector3f phi = gyro * dt;
  const Sophus::SO3f dR = Sophus::SO3f::exp(phi);
  R = R * dR;
  const Eigen::Vector3f wacc = R * acc - gacc;
  const Eigen::Vector3f v_prev = v;
  v = v_prev + wacc * dt;
  p = p + v_prev * dt + 0.5f * wacc * dt * dt;

  state.R = R;
  state.p = p;
  state.v = v;
}

} // namespace lico
