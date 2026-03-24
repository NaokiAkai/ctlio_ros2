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
 
#include <ctlio_ros2/imu_initializer.hpp>

namespace lico {

ImuInitializer::ImuInitializer() {

}

ImuInitializer::~ImuInitializer() {

}

Eigen::Quaternionf ImuInitializer::QuaternionFromTwoVectors(
  const Eigen::Vector3f& a_in,
  const Eigen::Vector3f& b_in)
{
  Eigen::Vector3f a = a_in;
  Eigen::Vector3f b = b_in;

  const float na = a.norm();
  const float nb = b.norm();
  if (na < 1e-8f || nb < 1e-8f) {
    return Eigen::Quaternionf::Identity();
  }
  a /= na;
  b /= nb;

  const float c = a.dot(b);
  // If vectors are nearly opposite, choose an arbitrary orthogonal axis.
  if (c < -0.999999f) {
    Eigen::Vector3f axis = Eigen::Vector3f(1.0f, 0.0f, 0.0f).cross(a);
    if (axis.norm() < 1e-6f) {
      axis = Eigen::Vector3f(0.0f, 1.0f, 0.0f).cross(a);
    }
    axis.normalize();
    return Eigen::Quaternionf(Eigen::AngleAxisf(static_cast<float>(M_PI), axis)).normalized();
  }

  Eigen::Vector3f v = a.cross(b);
  const float s = std::sqrt((1.0f + c) * 2.0f);
  const float invs = 1.0f / s;

  Eigen::Quaternionf q;
  q.w() = 0.5f * s;
  q.x() = v.x() * invs;
  q.y() = v.y() * invs;
  q.z() = v.z() * invs;
  return q.normalized();
}

Eigen::Vector3f ImuInitializer::MeanOf(const std::vector<Eigen::Vector3f>& xs) {
  if (xs.empty()) return Eigen::Vector3f::Zero();
  Eigen::Vector3f sum = Eigen::Vector3f::Zero();
  for (const auto& x : xs) sum += x;
  return sum / static_cast<float>(xs.size());
}

Eigen::Vector3f ImuInitializer::StdOf(
  const std::vector<Eigen::Vector3f>& xs,
  const Eigen::Vector3f& mean)
{
  if (xs.size() < 2) return Eigen::Vector3f::Zero();
  Eigen::Vector3f var = Eigen::Vector3f::Zero();
  for (const auto& x : xs) {
    const Eigen::Vector3f d = x - mean;
    var += d.cwiseProduct(d);
  }
  var /= static_cast<float>(xs.size() - 1);
  return var.cwiseSqrt();
}

void ImuInitializer::InitializeImu(
  const boost::circular_buffer<lico::ImuMeasure>& imu_buf,
  float gravity_acc,
  State& state)
{
  std::vector<Eigen::Vector3f> gyros;
  std::vector<Eigen::Vector3f> accs;
  gyros.reserve(imu_buf.size());
  accs.reserve(imu_buf.size());
  for (const auto& m : imu_buf) {
    gyros.push_back(m.gyro);
    accs.push_back(m.acc);
  }

  // 2) Mean gyro -> gyro bias (stationary: true omega = 0).
  const Eigen::Vector3f mean_g = MeanOf(gyros);
  state.bg = mean_g;

  // 3) Mean accel -> gravity direction (roll/pitch). (Yaw is unobservable here.)
  const Eigen::Vector3f mean_a = MeanOf(accs);
  const float mean_a_norm = SafeNorm(mean_a);
  if (mean_a_norm < 1e-3f) {
    return;
  }

  // We assume measured accel ~ R^T * g (plus bias). For direction, use mean_a.
  // World gravity vector (Z up): g_w = (0, 0, -g_mag)
  const Eigen::Vector3f g_w(0.0f, 0.0f, gravity_acc);

  // If we (temporarily) ignore accel bias, mean_a points roughly along R^T*g_w.
  // Let b-frame vector be "expected gravity in body": g_b_dir ~ mean_a.normalized().
  // We want q_wb such that: g_b = R_wb^T * g_w  (in body)
  // Equivalent: R_wb maps body->world, so R_wb * g_b \simeq g_w.
  // So rotate mean_a_dir (body) to g_w_dir (world) using q_wb.
  const Eigen::Vector3f mean_a_dir = mean_a / mean_a_norm;
  const Eigen::Vector3f g_w_dir = g_w.normalized();

  // q_wb rotates mean_a_dir (in body) to g_w_dir (in world).
  const Eigen::Quaternionf q_wb = QuaternionFromTwoVectors(mean_a_dir, g_w_dir);

  // 4) Accel bias initial guess.
  // If estimate_accel_bias = true:
  //   mean_a \simeq R_wb^T * g_w + b_a  => b_a \simeq mean_a - R_wb^T * g_w
  // Note: stationary-only can't uniquely separate bias vs gravity, but this is a good seed.
  const Eigen::Matrix3f R_wb = q_wb.toRotationMatrix();  // body->world
  const Eigen::Vector3f g_b = R_wb.transpose() * g_w;    // gravity in body

  const Eigen::Matrix3f R_cur = state.R.matrix();
  const Eigen::Matrix3f R_imu = R_wb;

  const float yaw = std::atan2(R_cur(1, 0), R_cur(0, 0));
  const float roll  = std::atan2(R_imu(2, 1), R_imu(2, 2));
  const float pitch = std::atan2(-R_imu(2, 0),
    std::sqrt(R_imu(2, 1) * R_imu(2, 1) + R_imu(2, 2) * R_imu(2, 2)));

  const Eigen::AngleAxisf yaw_rot(yaw,   Eigen::Vector3f::UnitZ());
  const Eigen::AngleAxisf pitch_rot(pitch, Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf roll_rot(roll, Eigen::Vector3f::UnitX());

  Eigen::Matrix3f R_new = (yaw_rot * pitch_rot * roll_rot).toRotationMatrix();
  state.R = Sophus::SO3f(R_new);
  state.ba = mean_a - g_b;

  // 5) Initial velocity (stationary).
  state.v = Eigen::Vector3f::Zero();

  // 6) Rough noise stats (useful to set covariance).
  // const Eigen::Vector3f gyro_std = StdOf(gyros, mean_g);
  // const Eigen::Vector3f acc_std = StdOf(accs, mean_a);

  std::cout << "Done IMU initialization" << std::endl;
  std::cout << "R\n" << state.R.matrix() << std::endl;
  std::cout << "v:  " << state.v.transpose() << std::endl;
  std::cout << "bg: " << state.bg.transpose() << std::endl;
  std::cout << "ba: " << state.ba.transpose() << std::endl;
  std::cout << "Std" << std::endl;
  std::cout << "gyro: " << StdOf(gyros, mean_g).transpose() << std::endl;
  std::cout << "acc:  " << StdOf(accs, mean_a).transpose() << std::endl;
  std::cout << std::endl;

  return;
}

} // namespace lico
