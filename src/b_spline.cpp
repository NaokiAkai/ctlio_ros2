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
 
#include <ctlio_ros2/b_spline.hpp>

namespace lico { 

BSpline::BSpline() {

}

BSpline::~BSpline() {

}

bool BSpline::FindInterval(
  double t,
  const std::vector<ControlPoint>& control_points,
  int& idx)
{
  idx = -1;

  const double t1 = interval_ * 2.0;
  const double t2 = interval_ * static_cast<double>(control_points.size());
  if (t < t1 || t2 <= t) {
    // std::cerr << "FindInterval: Given stamp is out of range of control points." << std::endl;
    // printf("Given %lf: [%lf - %lf]\n", t, t1, t2);
    return false;
  }

  for (size_t i = 3; i < control_points.size(); ++i) {
    if (t < static_cast<double>(i * interval_)) {
      idx = static_cast<int>(i);
      return true;
    }
  }

  // std::cerr << "FindInterval: Failed to find interval against " << t << std::endl;
  return false;
}

Eigen::Vector4f BSpline::ComputeBasis(
  double t,
  int idx)
{
  const double t1 = static_cast<double>(idx) * interval_;
  const double t0 = static_cast<double>(idx - 1) * interval_;
  const float dt = static_cast<float>(t1 - t0);
  float u = static_cast<float>(t - t0) / dt;
  u = std::clamp(u, 0.0f, 1.0f);

  const float u2 = u * u;
  const float u3 = u2 * u;

  Eigen::Vector4f beta;
  beta[0] = (1.0f - 3.0f * u + 3.0f * u2 - u3) / 6.0f;
  beta[1] = (4.0f - 6.0f * u2 + 3.0f * u3) / 6.0f;
  beta[2] = (1.0f + 3.0f * u + 3.0f * u2 - 3.0f * u3) / 6.0f;
  beta[3] = u3 / 6.0f;
  return beta;
}

void BSpline::ComputeBasisAndDerivatives(
  double t,
  int idx,
  Eigen::Vector4f& beta,
  Eigen::Vector4f& beta_dot,
  Eigen::Vector4f& beta_ddot)
{
  const double t1 = static_cast<double>(idx) * interval_;
  const double t0 = static_cast<double>(idx - 1) * interval_;
  const float dt = static_cast<float>(t1 - t0);
  float u = static_cast<float>(t - t0) / dt;
  u = std::clamp(u, 0.0f, 1.0f);

  const float u2 = u * u;
  const float u3 = u2 * u;
  const float inv_dt = 1.0f / dt;
  const float inv_dt2 = inv_dt * inv_dt;

  beta[0] = (1.0f - 3.0f * u + 3.0f * u2 - u3) / 6.0f;
  beta[1] = (4.0f - 6.0f * u2 + 3.0f * u3) / 6.0f;
  beta[2] = (1.0f + 3.0f * u + 3.0f * u2 - 3.0f * u3) / 6.0f;
  beta[3] = u3 / 6.0f;

  // d beta / dt
  beta_dot[0] = (-3.0f + 6.0f * u - 3.0f * u2) / 6.0f * inv_dt;
  beta_dot[1] = (-12.0f * u + 9.0f * u2) / 6.0f * inv_dt;
  beta_dot[2] = (3.0f + 6.0f * u - 9.0f * u2) / 6.0f * inv_dt;
  beta_dot[3] = (3.0f * u2) / 6.0f * inv_dt;

  // d^2 beta / dt^2
  beta_ddot[0] = (6.0f - 6.0f * u) / 6.0f * inv_dt2;
  beta_ddot[1] = (-12.0f + 18.0f * u) / 6.0f * inv_dt2;
  beta_ddot[2] = (6.0f - 18.0f * u) / 6.0f * inv_dt2;
  beta_ddot[3] = (6.0f * u) / 6.0f * inv_dt2;

  // printf("beta_dot %f %f %f %f\n", beta_dot[0], beta_dot[1], beta_dot[2], beta_dot[3]);
  // printf("beta_ddot %f %f %f %f\n", beta_ddot[0], beta_ddot[1], beta_ddot[2], beta_ddot[3]);
}

Eigen::Vector3f BSpline::BasisToLambda(const Eigen::Vector4f& beta) {
  const Eigen::Vector3f lambda(beta[1] + beta[2] + beta[3], beta[2] + beta[3], beta[3]);
  return lambda;
}

bool BSpline::ComputePose(
  double t,
  const std::vector<ControlPoint>& control_points,
  Sophus::SO3f& R,
  Eigen::Vector3f& p)
{
  const double stamp = t - control_points.front().stamp;
  int i;
  if (!FindInterval(stamp, control_points, i)) {
    // std::cerr << "ComputePose: Failed to find interval against " << t << std::endl;
    return false;
  }
  
  std::array<int, 4> idx4;
  idx4 = {i - 3, i - 2, i - 1, i};
  const Eigen::Vector4f beta = ComputeBasis(stamp, i);
  const Eigen::Vector3f lambda = BasisToLambda(beta);

  const Eigen::Vector3f& p0 = control_points[idx4[0]].p;
  const Eigen::Vector3f& p1 = control_points[idx4[1]].p;
  const Eigen::Vector3f& p2 = control_points[idx4[2]].p;
  const Eigen::Vector3f& p3 = control_points[idx4[3]].p;

  const Eigen::Vector3f d1 = p1 - p0;
  const Eigen::Vector3f d2 = p2 - p1;
  const Eigen::Vector3f d3 = p3 - p2;

  p = p0 + lambda[0] * d1 + lambda[1] * d2 + lambda[2] * d3;

  const Sophus::SO3f& R0 = control_points[idx4[0]].R;
  const Sophus::SO3f& R1 = control_points[idx4[1]].R;
  const Sophus::SO3f& R2 = control_points[idx4[2]].R;
  const Sophus::SO3f& R3 = control_points[idx4[3]].R;

  const Eigen::Vector3f phi1 = (R0.inverse() * R1).log();
  const Eigen::Vector3f phi2 = (R1.inverse() * R2).log();
  const Eigen::Vector3f phi3 = (R2.inverse() * R3).log();

  const Sophus::SO3f A1 = Sophus::SO3f::exp(lambda[0] * phi1);
  const Sophus::SO3f A2 = Sophus::SO3f::exp(lambda[1] * phi2);
  const Sophus::SO3f A3 = Sophus::SO3f::exp(lambda[2] * phi3);

  R = R0 * A1 * A2 * A3;

  return true;
}

bool BSpline::ComputePoseAndDerivatives(
  double t,
  const std::vector<ControlPoint>& control_points,
  Sophus::SO3f& R,
  Eigen::Vector3f& p,
  std::array<int, 4>& idx4,
  Eigen::Matrix<float, 3, 24>& dTt_dcps)
{
  const double stamp = t - control_points.front().stamp;
  int i;
  if (!FindInterval(stamp, control_points, i)) {
    // std::cerr << "ComputePose: Failed to find interval against " << t << std::endl;
    return false;
  }
  
  idx4 = {i - 3, i - 2, i - 1, i};
  const Eigen::Vector4f beta = ComputeBasis(stamp, i);
  const Eigen::Vector3f lambda = BasisToLambda(beta);

  const Eigen::Vector3f& p0 = control_points[idx4[0]].p;
  const Eigen::Vector3f& p1 = control_points[idx4[1]].p;
  const Eigen::Vector3f& p2 = control_points[idx4[2]].p;
  const Eigen::Vector3f& p3 = control_points[idx4[3]].p;

  const Eigen::Vector3f d1 = p1 - p0;
  const Eigen::Vector3f d2 = p2 - p1;
  const Eigen::Vector3f d3 = p3 - p2;

  p = p0 + lambda[0] * d1 + lambda[1] * d2 + lambda[2] * d3;

  const Sophus::SO3f& R0 = control_points[idx4[0]].R;
  const Sophus::SO3f& R1 = control_points[idx4[1]].R;
  const Sophus::SO3f& R2 = control_points[idx4[2]].R;
  const Sophus::SO3f& R3 = control_points[idx4[3]].R;

  const Eigen::Vector3f phi1 = (R0.inverse() * R1).log();
  const Eigen::Vector3f phi2 = (R1.inverse() * R2).log();
  const Eigen::Vector3f phi3 = (R2.inverse() * R3).log();

  const Sophus::SO3f A1 = Sophus::SO3f::exp(lambda[0] * phi1);
  const Sophus::SO3f A2 = Sophus::SO3f::exp(lambda[1] * phi2);
  const Sophus::SO3f A3 = Sophus::SO3f::exp(lambda[2] * phi3);

  R = R0 * A1 * A2 * A3;

  // Derivatives
  const Eigen::Matrix3f I   = Eigen::Matrix3f::Identity();
  const Eigen::Matrix3f R0T = R0.matrix().transpose();
  const Eigen::Matrix3f R1T = R1.matrix().transpose();
  const Eigen::Matrix3f R2T = R2.matrix().transpose();

  const Eigen::Matrix3f Jll1 = LeftJacobianSO3(lambda[0] * phi1) * lambda[0];
  const Eigen::Matrix3f Jll2 = LeftJacobianSO3(lambda[1] * phi2) * lambda[1];
  const Eigen::Matrix3f Jll3 = LeftJacobianSO3(lambda[2] * phi3) * lambda[2];

  const Eigen::Matrix3f dRt_dphi1 = R0.matrix() * Jll1;
  const Eigen::Matrix3f dRt_dphi2 = (R0 * A1).matrix() * Jll2;
  const Eigen::Matrix3f dRt_dphi3 = (R0 * A1 * A2).matrix() * Jll3;

  const Eigen::Matrix3f Jlinv1 = LeftJacobianInvSO3(phi1);
  const Eigen::Matrix3f dphi1_dR0 = -Jlinv1 * R0T;
  const Eigen::Matrix3f dphi1_dR1 =  Jlinv1 * R0T;

  const Eigen::Matrix3f Jlinv2 = LeftJacobianInvSO3(phi2);
  const Eigen::Matrix3f dphi2_dR1 = -Jlinv2 * R1T;
  const Eigen::Matrix3f dphi2_dR2 =  Jlinv2 * R1T;

  const Eigen::Matrix3f Jlinv3 = LeftJacobianInvSO3(phi3);
  const Eigen::Matrix3f dphi3_dR2 = -Jlinv3 * R2T;
  const Eigen::Matrix3f dphi3_dR3 =  Jlinv3 * R2T;

  const Eigen::Matrix3f dRt_dR0 = I                     + dRt_dphi1 * dphi1_dR0;
  const Eigen::Matrix3f dRt_dR1 = dRt_dphi1 * dphi1_dR1 + dRt_dphi2 * dphi2_dR1;
  const Eigen::Matrix3f dRt_dR2 = dRt_dphi2 * dphi2_dR2 + dRt_dphi3 * dphi3_dR2;
  const Eigen::Matrix3f dRt_dR3 = dRt_dphi3 * dphi3_dR3;
  
  const Eigen::Matrix3f dpt_dp0 = beta[0] * I;
  const Eigen::Matrix3f dpt_dp1 = beta[1] * I;
  const Eigen::Matrix3f dpt_dp2 = beta[2] * I;
  const Eigen::Matrix3f dpt_dp3 = beta[3] * I;

  // dRt/dR0, dpt/dp0, dRt/dR1, dpt/dp1, dRt/dR2, dpt/dp2, dRt/dR3, dpt/dp3
  dTt_dcps.block<3, 3>(0,  0) = dRt_dR0;
  dTt_dcps.block<3, 3>(0,  3) = dpt_dp0;
  dTt_dcps.block<3, 3>(0,  6) = dRt_dR1;
  dTt_dcps.block<3, 3>(0,  9) = dpt_dp1;
  dTt_dcps.block<3, 3>(0, 12) = dRt_dR2;
  dTt_dcps.block<3, 3>(0, 15) = dpt_dp2;
  dTt_dcps.block<3, 3>(0, 18) = dRt_dR3;
  dTt_dcps.block<3, 3>(0, 21) = dpt_dp3;

  return true;
}

bool BSpline::ComputeMotion(
  double t,
  const std::vector<ControlPoint>& control_points,
  Sophus::SO3f& R,
  Eigen::Vector3f& omega_body,
  Eigen::Vector3f& p,
  Eigen::Vector3f& v,
  Eigen::Vector3f& a)
{
  const double stamp = t - control_points.front().stamp;
  int i;
  if (!FindInterval(stamp, control_points, i)) {
    // std::cerr << "ComputeMotion: Failed to find interval against " << t << std::endl;
    return false;
  }

  std::array<int, 4> idx4;
  idx4 = {i - 3, i - 2, i - 1, i};
  Eigen::Vector4f beta, beta_dot, beta_ddot;
  ComputeBasisAndDerivatives(stamp, i, beta, beta_dot, beta_ddot);
  const Eigen::Vector3f lambda = BasisToLambda(beta);
  const Eigen::Vector3f lambda_dot = BasisToLambda(beta_dot);
  const Eigen::Vector3f lambda_ddot = BasisToLambda(beta_ddot);

  const Eigen::Vector3f& p0 = control_points[idx4[0]].p;
  const Eigen::Vector3f& p1 = control_points[idx4[1]].p;
  const Eigen::Vector3f& p2 = control_points[idx4[2]].p;
  const Eigen::Vector3f& p3 = control_points[idx4[3]].p;

  const Eigen::Vector3f d1 = p1 - p0;
  const Eigen::Vector3f d2 = p2 - p1;
  const Eigen::Vector3f d3 = p3 - p2;

  p = p0 + lambda[0] *      d1 + lambda[1] *      d2 + lambda[2] *      d3;
  v =      lambda_dot[0] *  d1 + lambda_dot[1] *  d2 + lambda_dot[2] *  d3;
  a =      lambda_ddot[0] * d1 + lambda_ddot[1] * d2 + lambda_ddot[2] * d3;

  const Sophus::SO3f& R0 = control_points[idx4[0]].R;
  const Sophus::SO3f& R1 = control_points[idx4[1]].R;
  const Sophus::SO3f& R2 = control_points[idx4[2]].R;
  const Sophus::SO3f& R3 = control_points[idx4[3]].R;

  const Eigen::Vector3f phi1 = (R0.inverse() * R1).log();
  const Eigen::Vector3f phi2 = (R1.inverse() * R2).log();
  const Eigen::Vector3f phi3 = (R2.inverse() * R3).log();

  const Sophus::SO3f A1 = Sophus::SO3f::exp(lambda[0] * phi1);
  const Sophus::SO3f A2 = Sophus::SO3f::exp(lambda[1] * phi2);
  const Sophus::SO3f A3 = Sophus::SO3f::exp(lambda[2] * phi3);

  R = R0 * A1 * A2 * A3;

  // const Sophus::SO3f A1_dot = Sophus::SO3f::exp(lambda_dot[0] * phi1);
  // const Sophus::SO3f A2_dot = Sophus::SO3f::exp(lambda_dot[1] * phi2);
  // const Sophus::SO3f A3_dot = Sophus::SO3f::exp(lambda_dot[2] * phi3);
  // const Eigen::Matrix3f R_dot = R0.matrix() *
  //   ((A1_dot * A2 * A3).matrix() + (A1 * A2_dot * A3).matrix() + 
  //   (A1 * A2 * A3_dot).matrix());

  const Eigen::Matrix3f A1_dot = Hat(lambda_dot[0] * phi1) * A1.matrix();
  const Eigen::Matrix3f A2_dot = Hat(lambda_dot[1] * phi2) * A2.matrix();
  const Eigen::Matrix3f A3_dot = Hat(lambda_dot[2] * phi3) * A3.matrix();
  const Eigen::Matrix3f R_dot = R0.matrix() *
    (A1_dot * (A2 * A3).matrix() + A1.matrix() * A2_dot * A3.matrix() + 
    (A1 * A2).matrix() * A3_dot);

  const Eigen::Matrix3f RT = R.matrix().transpose(); 
  const Eigen::Matrix3f W = RT * R_dot;
  omega_body = Sophus::SO3f::vee(W);

  return true;
}

bool BSpline::ComputeMotionAndDerivatives(
  double t,
  const std::vector<ControlPoint>& control_points,
  Sophus::SO3f& R,
  Eigen::Vector3f& omega_body,
  Eigen::Vector3f& p,
  Eigen::Vector3f& v,
  Eigen::Vector3f& a,
  float gravity_acc,
  std::array<int, 4>& idx4,
  Eigen::Matrix<float, 6, 24>& dr_dcps)
{
  const double stamp = t - control_points.front().stamp;
  int i;
  if (!FindInterval(stamp, control_points, i)) {
    // std::cerr << "ComputeMotion: Failed to find interval against " << t << std::endl;
    return false;
  }

  idx4 = {i - 3, i - 2, i - 1, i};
  Eigen::Vector4f beta, beta_dot, beta_ddot;
  ComputeBasisAndDerivatives(stamp, i, beta, beta_dot, beta_ddot);
  const Eigen::Vector3f lambda = BasisToLambda(beta);
  const Eigen::Vector3f lambda_dot = BasisToLambda(beta_dot);
  const Eigen::Vector3f lambda_ddot = BasisToLambda(beta_ddot);

  const Eigen::Vector3f& p0 = control_points[idx4[0]].p;
  const Eigen::Vector3f& p1 = control_points[idx4[1]].p;
  const Eigen::Vector3f& p2 = control_points[idx4[2]].p;
  const Eigen::Vector3f& p3 = control_points[idx4[3]].p;

  const Eigen::Vector3f d1 = p1 - p0;
  const Eigen::Vector3f d2 = p2 - p1;
  const Eigen::Vector3f d3 = p3 - p2;

  p = p0 + lambda[0] *      d1 + lambda[1] *      d2 + lambda[2] *      d3;
  v =      lambda_dot[0] *  d1 + lambda_dot[1] *  d2 + lambda_dot[2] *  d3;
  a =      lambda_ddot[0] * d1 + lambda_ddot[1] * d2 + lambda_ddot[2] * d3;

  const Sophus::SO3f& R0 = control_points[idx4[0]].R;
  const Sophus::SO3f& R1 = control_points[idx4[1]].R;
  const Sophus::SO3f& R2 = control_points[idx4[2]].R;
  const Sophus::SO3f& R3 = control_points[idx4[3]].R;

  const Eigen::Vector3f phi1 = (R0.inverse() * R1).log();
  const Eigen::Vector3f phi2 = (R1.inverse() * R2).log();
  const Eigen::Vector3f phi3 = (R2.inverse() * R3).log();

  const Sophus::SO3f A1 = Sophus::SO3f::exp(lambda[0] * phi1);
  const Sophus::SO3f A2 = Sophus::SO3f::exp(lambda[1] * phi2);
  const Sophus::SO3f A3 = Sophus::SO3f::exp(lambda[2] * phi3);

  R = R0 * A1 * A2 * A3;

  // const Sophus::SO3f A1_dot = Sophus::SO3f::exp(lambda_dot[0] * phi1);
  // const Sophus::SO3f A2_dot = Sophus::SO3f::exp(lambda_dot[1] * phi2);
  // const Sophus::SO3f A3_dot = Sophus::SO3f::exp(lambda_dot[2] * phi3);
  // const Eigen::Matrix3f R_dot = R0.matrix() *
  //   ((A1_dot * A2 * A3).matrix() + (A1 * A2_dot * A3).matrix() + 
  //   (A1 * A2 * A3_dot).matrix());

  const Eigen::Matrix3f A1_dot = Hat(lambda_dot[0] * phi1) * A1.matrix();
  const Eigen::Matrix3f A2_dot = Hat(lambda_dot[1] * phi2) * A2.matrix();
  const Eigen::Matrix3f A3_dot = Hat(lambda_dot[2] * phi3) * A3.matrix();
  const Eigen::Matrix3f R_dot = R0.matrix() *
    (A1_dot * (A2 * A3).matrix() + A1.matrix() * A2_dot * A3.matrix() + 
    (A1 * A2).matrix() * A3_dot);

  const Eigen::Matrix3f RT = R.matrix().transpose(); 
  const Eigen::Matrix3f W = RT * R_dot;
  omega_body = Sophus::SO3f::vee(W);

  // Derivatives
  const Eigen::Matrix3f I   = Eigen::Matrix3f::Identity();
  const Eigen::Matrix3f R0T = R0.matrix().transpose();
  const Eigen::Matrix3f R1T = R1.matrix().transpose();
  const Eigen::Matrix3f R2T = R2.matrix().transpose();

  const Eigen::Matrix3f Jll1 = LeftJacobianSO3(lambda[0] * phi1) * lambda[0];
  const Eigen::Matrix3f Jll2 = LeftJacobianSO3(lambda[1] * phi2) * lambda[1];
  const Eigen::Matrix3f Jll3 = LeftJacobianSO3(lambda[2] * phi3) * lambda[2];

  const Eigen::Matrix3f dRt_dphi1 = R0.matrix() * Jll1;
  const Eigen::Matrix3f dRt_dphi2 = (R0 * A1).matrix() * Jll2;
  const Eigen::Matrix3f dRt_dphi3 = (R0 * A1 * A2).matrix() * Jll3;

  const Eigen::Matrix3f Jlinv1 = LeftJacobianInvSO3(phi1);
  const Eigen::Matrix3f dphi1_dR0 = -Jlinv1 * R0T;
  const Eigen::Matrix3f dphi1_dR1 =  Jlinv1 * R0T;

  const Eigen::Matrix3f Jlinv2 = LeftJacobianInvSO3(phi2);
  const Eigen::Matrix3f dphi2_dR1 = -Jlinv2 * R1T;
  const Eigen::Matrix3f dphi2_dR2 =  Jlinv2 * R1T;

  const Eigen::Matrix3f Jlinv3 = LeftJacobianInvSO3(phi3);
  const Eigen::Matrix3f dphi3_dR2 = -Jlinv3 * R2T;
  const Eigen::Matrix3f dphi3_dR3 =  Jlinv3 * R2T;

  const Eigen::Matrix3f dRt_dR0 = I                     + dRt_dphi1 * dphi1_dR0;
  const Eigen::Matrix3f dRt_dR1 = dRt_dphi1 * dphi1_dR1 + dRt_dphi2 * dphi2_dR1;
  const Eigen::Matrix3f dRt_dR2 = dRt_dphi2 * dphi2_dR2 + dRt_dphi3 * dphi3_dR2;
  const Eigen::Matrix3f dRt_dR3 = dRt_dphi3 * dphi3_dR3;

  const Eigen::Vector3f gacc(0.0f, 0.0f, gravity_acc);
  const Eigen::Matrix3f A = RT * Hat(a + gacc);

  const Eigen::Matrix3f dra_dR0 = A * dRt_dR0;
  const Eigen::Matrix3f dra_dR1 = A * dRt_dR1;
  const Eigen::Matrix3f dra_dR2 = A * dRt_dR2;
  const Eigen::Matrix3f dra_dR3 = A * dRt_dR3;

  const Eigen::Matrix3f dra_dp0 = -lambda_ddot[0] * RT;
  const Eigen::Matrix3f dra_dp1 = (lambda_ddot[0] - lambda_ddot[1]) * RT;
  const Eigen::Matrix3f dra_dp2 = (lambda_ddot[1] - lambda_ddot[2]) * RT;
  const Eigen::Matrix3f dra_dp3 =  lambda_ddot[2] * RT;

  const Eigen::Matrix3f dw_dphi1 = (A1 * A2 * A3).matrix().transpose() *
    (Hat(lambda_dot[0] * phi1) * Jll1 + lambda_dot[0] * I);
  const Eigen::Matrix3f dw_dphi2 = (A2 * A3).matrix().transpose() *
    (Hat(A1.matrix().transpose() * lambda_dot[0] * phi1) * Jll2 +
     Hat(lambda_dot[1] * phi2) * Jll2 + lambda_dot[1] * I);
  const Eigen::Matrix3f dw_dphi3 = A3.matrix().transpose() *
    (Hat((A1 * A2).matrix().transpose() * lambda_dot[0] * phi1) * Jll3 +
     Hat(A2.matrix().transpose() * lambda_dot[1] * phi2) * Jll3 +
     Hat(lambda_ddot[2] * phi3) * Jll3 + lambda_dot[2] * I);

  const Eigen::Matrix3f drw_dR0 =                        dw_dphi1 * dphi1_dR0;
  const Eigen::Matrix3f drw_dR1 = dw_dphi1 * dphi1_dR1 + dw_dphi2 * dphi2_dR1;
  const Eigen::Matrix3f drw_dR2 = dw_dphi2 * dphi2_dR2 + dw_dphi3 * dphi3_dR2;
  const Eigen::Matrix3f drw_dR3 = dw_dphi3 * dphi3_dR3;

  // drw/dR0, drw/dp0, drw/dR1, drw/dp1, drw/dR2, drw/dp2, drw/dR3, drw/dp3
  // dra/dR0, dra/dp0, dra/dR1, dra/dp1, dra/dR2, dra/dp2, dra/dR3, dra/dp3
  dr_dcps.block<3, 3>(0,  0) = drw_dR0;
  dr_dcps.block<3, 3>(0,  3) = Eigen::Matrix3f::Zero();
  dr_dcps.block<3, 3>(0,  6) = drw_dR1;
  dr_dcps.block<3, 3>(0,  9) = Eigen::Matrix3f::Zero();
  dr_dcps.block<3, 3>(0, 12) = drw_dR2;
  dr_dcps.block<3, 3>(0, 15) = Eigen::Matrix3f::Zero();
  dr_dcps.block<3, 3>(0, 18) = drw_dR3;
  dr_dcps.block<3, 3>(0, 21) = Eigen::Matrix3f::Zero();

  dr_dcps.block<3, 3>(3,  0) = dra_dR0;
  dr_dcps.block<3, 3>(3,  3) = dra_dp0;
  dr_dcps.block<3, 3>(3,  6) = dra_dR1;
  dr_dcps.block<3, 3>(3,  9) = dra_dp1;
  dr_dcps.block<3, 3>(3, 12) = dra_dR2;
  dr_dcps.block<3, 3>(3, 15) = dra_dp2;
  dr_dcps.block<3, 3>(3, 18) = dra_dR3;
  dr_dcps.block<3, 3>(3, 21) = dra_dp3;

  return true;
}

} // namespace lico
