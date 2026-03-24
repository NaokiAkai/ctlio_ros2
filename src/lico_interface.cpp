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
 
#include <ctlio_ros2/lico_interface.hpp>

namespace lico {

LicoInterface::LicoInterface() {
  imu_buf_ = boost::circular_buffer<ImuMeasure>(1000);

  spline_.SetInterval(control_points_interval_);
}

LicoInterface::~LicoInterface() {

}

void LicoInterface::ReadParams() {
  const std::string yaml_file = param_files_dir_ + "/ctlio_params.yaml";
  const YAML::Node config = YAML::LoadFile(yaml_file);

  const auto initial_trans_vec   = config["initial_pose"]["translation"];
  const auto initial_rot_mat_vec = config["initial_pose"]["rotation_matrix"];
  const Eigen::Vector3f initial_trans(
    initial_trans_vec[0].as<float>(),
    initial_trans_vec[1].as<float>(),
    initial_trans_vec[2].as<float>());
  state_.p = initial_trans;

  Eigen::Matrix3f initial_rot_mat;
  initial_rot_mat << 
    initial_rot_mat_vec[0].as<float>(), initial_rot_mat_vec[1].as<float>(), initial_rot_mat_vec[2].as<float>(),
    initial_rot_mat_vec[3].as<float>(), initial_rot_mat_vec[4].as<float>(), initial_rot_mat_vec[5].as<float>(),
    initial_rot_mat_vec[6].as<float>(), initial_rot_mat_vec[7].as<float>(), initial_rot_mat_vec[8].as<float>();
  state_.R = Sophus::SO3f(initial_rot_mat);

  const auto T_imu_lidar_trans_vec   = config["extrinsics"]["imu_to_lidar"]["translation"];
  const auto T_imu_lidar_rot_mat_vec = config["extrinsics"]["imu_to_lidar"]["rotation_matrix"];
  const Eigen::Vector3f T_imu_lidar_trans(
    T_imu_lidar_trans_vec[0].as<float>(),
    T_imu_lidar_trans_vec[1].as<float>(),
    T_imu_lidar_trans_vec[2].as<float>());
  Eigen::Matrix3f T_imu_lidar_rot_mat;
  T_imu_lidar_rot_mat << 
    T_imu_lidar_rot_mat_vec[0].as<float>(), T_imu_lidar_rot_mat_vec[1].as<float>(), T_imu_lidar_rot_mat_vec[2].as<float>(),
    T_imu_lidar_rot_mat_vec[3].as<float>(), T_imu_lidar_rot_mat_vec[4].as<float>(), T_imu_lidar_rot_mat_vec[5].as<float>(),
    T_imu_lidar_rot_mat_vec[6].as<float>(), T_imu_lidar_rot_mat_vec[7].as<float>(), T_imu_lidar_rot_mat_vec[8].as<float>();
  Til_ = Sophus::SE3f(T_imu_lidar_rot_mat, T_imu_lidar_trans);

  stationary_sec_ = config["imu_params"]["stationary_sec"].as<double>();
  acc_scale_      = config["imu_params"]["acc_scale"].as<float>();
  gravity_acc_    = config["imu_params"]["gravity_acc"].as<float>();

  scan_cloud_clip_range_  = config["scan_cloud_preprocess"]["clip_range"].as<float>();
  scan_cloud_filter_size_ = config["scan_cloud_preprocess"]["filter_size"].as<float>();

  const size_t num_keyframes          = config["normal_map"]["num_keyframes"].as<size_t>();
  const float filter_size             = config["normal_map"]["filter_size"].as<float>();
  const size_t num_normal_points      = config["normal_map"]["num_normal_points"].as<size_t>();
  const float normal_eigen_val_thresh = config["normal_map"]["normal_eigen_val_thresh"].as<float>();
  normal_map_.SetMaxKeyframeSize(num_keyframes);
  normal_map_.SetFilterSize(filter_size);
  normal_map_.SetNumNormalPoints(num_normal_points);
  normal_map_.SetNormalEigenValThreshhold(normal_eigen_val_thresh);

  control_points_interval_ = config["control_points"]["control_points_interval"].as<double>();
  num_max_cp_              = config["control_points"]["num_max_cp"].as<int>();

  min_inlier_ratio_        = config["optimization"]["min_inlier_ratio"].as<float>();
  max_correspondence_dist_ = config["optimization"]["max_correspondence_dist"].as<float>();
  sm_huber_delta_          = config["optimization"]["sm_huber_delta"].as<float>();
  optimization_epsilon_    = config["optimization"]["optimization_epsilon"].as<float>();
  num_max_scan_points_     = config["optimization"]["num_max_scan_points"].as<int>();
  num_max_valid_points_    = config["optimization"]["num_max_valid_points"].as<int>();
  gn_damping_lambda_       = config["optimization"]["gn_damping_lambda"].as<float>();
  prior_dR_sigma_          = config["optimization"]["prior_dR_sigma"].as<float>();
  prior_dp_sigma_          = config["optimization"]["prior_dp_sigma"].as<float>();
  prior_bg_sigma_          = config["optimization"]["prior_bg_sigma"].as<float>();
  prior_ba_sigma_          = config["optimization"]["prior_ba_sigma"].as<float>();
  ct_traj_dR_sigma_        = config["optimization"]["ct_traj_dR_sigma"].as<float>();
  ct_traj_dp_sigma_        = config["optimization"]["ct_traj_dp_sigma"].as<float>();
  scan_matching_sigma_     = config["optimization"]["scan_matching_sigma"].as<float>();
  imu_mes_omega_sigma_     = config["optimization"]["imu_mes_omega_sigma"].as<float>();
  imu_mes_acc_sigma_       = config["optimization"]["imu_mes_acc_sigma"].as<float>();
  gyro_bias_rw_sigma_      = config["optimization"]["gyro_bias_rw_sigma"].as<float>();
  acc_bias_rw_sigma_       = config["optimization"]["acc_bias_rw_sigma"].as<float>();

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "===== Parameters Loaded =====" << std::endl;

  std::cout << "[Initial Pose]" << std::endl;
  std::cout << "  translation: ["
            << state_.p.x() << ", "
            << state_.p.y() << ", "
            << state_.p.z() << "]" << std::endl;

  std::cout << "  rotation:\n" << state_.R.matrix() << std::endl;

  std::cout << "[Extrinsics IMU -> LiDAR]" << std::endl;
  std::cout << "  translation: ["
            << Til_.translation().x() << ", "
            << Til_.translation().y() << ", "
            << Til_.translation().z() << "]" << std::endl;

  std::cout << "  rotation:\n" << Til_.rotationMatrix() << std::endl;

  std::cout << "[IMU Params]" << std::endl;
  std::cout << "  stationary_sec : " << stationary_sec_ << std::endl;
  std::cout << "  acc_scale      : " << acc_scale_ << std::endl;
  std::cout << "  gravity_acc    : " << gravity_acc_ << std::endl;

  std::cout << "[Scan Preprocess]" << std::endl;
  std::cout << "  clip_range : " << scan_cloud_clip_range_ << std::endl;
  std::cout << "  filter_size: " << scan_cloud_filter_size_ << std::endl;

  std::cout << "[Normal Map]" << std::endl;
  std::cout << "  num_keyframes      : " << num_keyframes << std::endl;
  std::cout << "  filter_size        : " << filter_size << std::endl;
  std::cout << "  num_normal_points  : " << num_normal_points << std::endl;
  std::cout << "  eigen_val_thresh   : " << normal_eigen_val_thresh << std::endl;

  std::cout << "[Control Points]" << std::endl;
  std::cout << "  interval: " << control_points_interval_ << std::endl;
  std::cout << "  max_cp  : " << num_max_cp_ << std::endl;

  std::cout << "[Optimization]" << std::endl;
  std::cout << "  min_inlier_ratio        : " << min_inlier_ratio_ << std::endl;
  std::cout << "  max_correspondence_dist : " << max_correspondence_dist_ << std::endl;
  std::cout << "  sm_huber_delta          : " << sm_huber_delta_ << std::endl;
  std::cout << "  num_max_scan_points     : " << num_max_scan_points_ << std::endl;
  std::cout << "  num_max_valid_points    : " << num_max_valid_points_ << std::endl;
  std::cout << "  gn_damping_lambda       : " << gn_damping_lambda_ << std::endl;

  std::cout << "  prior_dR_sigma  : " << prior_dR_sigma_ << std::endl;
  std::cout << "  prior_dp_sigma  : " << prior_dp_sigma_ << std::endl;
  std::cout << "  prior_bg_sigma  : " << prior_bg_sigma_ << std::endl;
  std::cout << "  prior_ba_sigma  : " << prior_ba_sigma_ << std::endl;

  std::cout << "  ct_traj_dR_sigma: " << ct_traj_dR_sigma_ << std::endl;
  std::cout << "  ct_traj_dp_sigma: " << ct_traj_dp_sigma_ << std::endl;

  std::cout << "  scan_matching_sigma : " << scan_matching_sigma_ << std::endl;
  std::cout << "  imu_omega_sigma     : " << imu_mes_omega_sigma_ << std::endl;
  std::cout << "  imu_acc_sigma       : " << imu_mes_acc_sigma_ << std::endl;

  std::cout << "  gyro_bias_rw_sigma : " << gyro_bias_rw_sigma_ << std::endl;
  std::cout << "  acc_bias_rw_sigma  : " << acc_bias_rw_sigma_ << std::endl;

  std::cout << "===================================" << std::endl;
}

void LicoInterface::SetImuMeasure(const ImuMeasure& measure) {
  ImuMeasure m = measure;
  m.acc *= acc_scale_;
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_buf_.push_back(m);
  }

  if (is_imu_initialized_) {
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      preintegrator_.Preintegration(m, gravity_acc_, odom_state_);
    }
  } else {
    const double dt = m.stamp - imu_buf_.front().stamp;
    if (dt > stationary_sec_) {
      ImuInitializer initializer;
      initializer.InitializeImu(imu_buf_, gravity_acc_, state_);
      is_imu_initialized_ = true;
    }
  }
}

void LicoInterface::SetScanCloud(
  const PointCloud3f& scan_cloud,
  const std::vector<float>& scan_intensities,
  const std::vector<double>& scan_stamps)
{
  if (!is_imu_initialized_) {
    // Skip scan processing: waiting for IMU initialization
    return;
  }

  // Clip close scan points and transform it to the IMU frame
  const size_t raw_cloud_size = scan_cloud.size();
  scan_cloud_.clear();
  scan_intensities_.clear();
  scan_stamps_.clear();
  scan_cloud_.reserve(raw_cloud_size);
  scan_intensities_.reserve(raw_cloud_size);
  scan_stamps_.reserve(raw_cloud_size);
  for (size_t i = 0; i < raw_cloud_size; ++i) {
    const Point3f& p = scan_cloud[i];
    if (p.norm() >= scan_cloud_clip_range_) {
      scan_cloud_.push_back(Til_ * p); // Transform to IMU frame
      scan_intensities_.push_back(scan_intensities[i]);
      scan_stamps_.push_back(scan_stamps[i]);
    }
  }

  // Register first points to local map
  if (!is_lidar_map_initialized_) {
    const Sophus::SE3f Toi(state_.R, state_.p);
    aligned_scan_cloud_.resize(scan_cloud_.size());
    for (size_t i = 0; i < scan_cloud_.size(); ++i) {
      aligned_scan_cloud_[i] = Toi * scan_cloud_[i];
    }
    normal_map_.AddAlignedScanCloud(aligned_scan_cloud_);
    is_lidar_map_initialized_ = true;
    is_lidar_map_updated_ = true;
    return;
  }

  // Do not use scan_stamps_ because it loses some scans when the scan is clipped.
  const double start_stamp = scan_stamps.front();
  const double end_stamp = scan_stamps.back();
  std::vector<ImuMeasure> relevant_imu_measures;

  // Retrieve relevant IMU measurements
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    for (size_t i = 0; i < imu_buf_.size(); ++i) {
      const auto& imu = imu_buf_[i];
      const double stamp = imu.stamp;

      if (start_stamp <= stamp && stamp <= end_stamp) {
        relevant_imu_measures.push_back(imu);
      } else if (stamp > end_stamp) {
        break;
      }
    }

    if (relevant_imu_measures.empty()) {
      std::cerr
        << "[WARN] No relevant IMU measures. "
        << "ROS2 may be dropping IMU messages because optimization or other processes are too computationally expensive. "
        << "In this case, the odometry estimation is highly likely to diverge or fail. "
        << "Please review the configuration and reduce the computational cost."
        << std::endl;
    }
  }

  // Save priors before prediction
  bg0_ = state_.bg;
  ba0_ = state_.ba;
  const Eigen::Vector3f bg_prior = bg0_;
  const Eigen::Vector3f ba_prior = ba0_;
  const std::vector<ControlPoint> control_points_priors = control_points_;

  // Prediction based on IMU preintegration
  if (relevant_imu_measures.size() > 0) {
    preintegrator_.Preintegration(relevant_imu_measures,
      gravity_acc_, control_points_interval_, state_, control_points_);
  }

  // The minimum control points number is 4
  // Do not optimize if the number of control points is not enough
  if (control_points_.size() < 4) return;

  // Add future control points if end stamp is not covered
  while (end_stamp - control_points_.front().stamp
         >= (control_points_.size() - 1 ) * control_points_interval_)
  {
    const double stamp = preintegrator_.GetPrevControlPointStamp() + control_points_interval_;
    const size_t size_cp = control_points_.size();
    const ControlPoint cp0 = control_points_[size_cp - 2];
    const ControlPoint cp1 = control_points_[size_cp - 1];
    const Sophus::SO3f dR = cp0.R.inverse() * cp1.R;
    const Eigen::Vector3f dp = cp1.p - cp0.p;
    ControlPoint cp;
    cp.stamp = stamp;
    cp.R = cp1.R * dR;
    cp.p = cp1.p + dp;
    control_points_.push_back(cp);
    preintegrator_.SetPrevControlPointStamp(stamp);
  }

  // Do not perform optimization if relevant IMU measures are not found
  // This always occurs due to heavy computational cost of SetScanCloud 
  if (relevant_imu_measures.size() == 0) return;

  // Save the prediction result
  const State pred_state = state_;
  const std::vector<ControlPoint> pred_control_points = control_points_;
  bg1_ = state_.bg;
  ba1_ = state_.ba;

  // Scan cloud downsamling
  const VoxelGridFilter vgf(scan_cloud_filter_size_);
  PointCloud3f filtered_scan_cloud;
  std::vector<float> filtered_scan_intensities;
  std::vector<double> filtered_scan_stamps;
  vgf.filter(scan_cloud_, scan_intensities_, scan_stamps_,
    filtered_scan_cloud, filtered_scan_intensities, filtered_scan_stamps);

  // Optimization
  // Variable order
  // cp[0], cp[1], ..., cp[N - 1], cp[N], bg0, ba0, bg1, ba1
  // Variable size: 6 * (N + 1) + 12
  const size_t num_cp = control_points_.size();
  const size_t num_opt_var = 6 * num_cp + 12;
  const size_t num_imu_iter = 2;
  const size_t num_min_iter = 1;
  const size_t num_max_iter = 5;
  has_converged_ = false;

  int num_used_points    = 1;
  int num_corresp_points = 1;
  int num_invalid_points = 1;
  int num_valid_points   = 1;

  int scan_step = static_cast<int>(filtered_scan_cloud.size()) / num_max_scan_points_;
  if (scan_step < 1) scan_step = 1;

  for (size_t iter_num = 0; iter_num < num_imu_iter + num_max_iter; ++iter_num) {
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> H = 
      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(num_opt_var, num_opt_var);
    Eigen::Matrix<float, Eigen::Dynamic, 1> b = 
      Eigen::Matrix<float, Eigen::Dynamic, 1>::Zero(num_opt_var, 1);

    // Control points priors
    {
      const Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
      const float dR_info = 1.0f / (prior_dR_sigma_ * prior_dR_sigma_);
      const float dp_info = 1.0f / (prior_dp_sigma_ * prior_dp_sigma_);
      const Eigen::Matrix3f Info_dR = dR_info * I;
      const Eigen::Matrix3f Info_dp = dp_info * I;

      for (size_t i = 0; i < control_points_priors.size(); ++i) {
        const Sophus::SO3f& R = control_points_[i].R;
        const Sophus::SO3f& Rp = control_points_priors[i].R;
        const Eigen::Vector3f rR = (Rp.inverse() * R).log();
        const Eigen::Matrix3f JR = Rp.matrix().transpose();
        const Eigen::Matrix3f JRT = JR.transpose();
        const size_t Ridx = 6 * i;
        H.block<3, 3>(Ridx, Ridx).noalias() += JRT * Info_dR * JR;
        b.block<3, 1>(Ridx,    0).noalias() += JRT * Info_dR * rR;

        const Eigen::Vector3f& p = control_points_[i].p;
        const Eigen::Vector3f& pp = control_points_priors[i].p;
        const Eigen::Vector3f rp = p - pp;
        const size_t tidx = 6 * i + 3;
        H.block<3, 3>(tidx, tidx).noalias() += Info_dp;
        b.block<3, 1>(tidx,    0).noalias() += Info_dp * rp;
      }
    }

    // Bias prior
    {
      const Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
      const float bg_info = 1.0f / (prior_bg_sigma_ * prior_bg_sigma_);
      const float ba_info = 1.0f / (prior_ba_sigma_ * prior_ba_sigma_);
      const Eigen::Matrix3f Info_bg = bg_info * I;
      const Eigen::Matrix3f Info_ba = ba_info * I;

      const Eigen::Vector3f rbg = bg0_ - bg_prior;
      const size_t bg_idx = 6 * num_cp;
      H.block<3, 3>(bg_idx, bg_idx).noalias() += Info_bg;
      b.block<3, 1>(bg_idx,      0).noalias() += rbg;

      const Eigen::Vector3f rba = ba0_ - ba_prior;
      const size_t ba_idx = 6 * num_cp + 3;
      H.block<3, 3>(ba_idx, ba_idx).noalias() += Info_ba;
      b.block<3, 1>(ba_idx,      0).noalias() += rba;
    }

    // Smoothness
    if (iter_num + 1 >= num_imu_iter) {
      const Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
      const float dR_info = 1.0f / (ct_traj_dR_sigma_ * ct_traj_dR_sigma_);
      const float dp_info = 1.0f / (ct_traj_dp_sigma_ * ct_traj_dp_sigma_);
      Eigen::Matrix<float, 6, 6> Info = Eigen::Matrix<float, 6, 6>::Zero();
      Info.block<3, 3>(0, 0) = dR_info * I;
      Info.block<3, 3>(3, 3) = dp_info * I;

      for (size_t i = 1; i < control_points_.size() - 1; ++i) {
        const Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
        const ControlPoint& cp0 = control_points_[i - 1];
        const ControlPoint& cp1 = control_points_[i];
        const ControlPoint& cp2 = control_points_[i + 1];

        const Eigen::Vector3f phi01 = (cp0.R.inverse() * cp1.R).log();
        const Eigen::Vector3f phi12 = (cp1.R.inverse() * cp2.R).log();

        const Eigen::Matrix3f R0T = cp0.R.matrix().transpose();
        const Eigen::Matrix3f R1T = cp1.R.matrix().transpose();
        const Eigen::Matrix3f Jl01inv = LeftJacobianInvSO3(phi01);
        const Eigen::Matrix3f Jl12inv = LeftJacobianInvSO3(phi12);

        const Eigen::Vector3f rR = phi12 - phi01;
        const Eigen::Vector3f rp = cp2.p - 2.0f * cp1.p + cp0.p;
        Eigen::Matrix<float, 6, 1> r;
        r.block<3, 1>(0, 0) = rR;
        r.block<3, 1>(3, 0) = rp;

        // rR/R0, rR/p0, rR/R1, rR/p1, rR/R2, rR/p2
        // rp/R0, rp/p0, rp/R1, rp/p1, rp/R2, rp/p2
        Eigen::Matrix<float, 6, 18> J = Eigen::Matrix<float, 6, 18>::Zero();
        J.block<3, 3>(0,  0) =  Jl01inv * R0T;
        J.block<3, 3>(0,  6) = -Jl12inv * R1T - Jl01inv * R0T;
        J.block<3, 3>(0, 12) =  Jl12inv * R1T;
        J.block<3, 3>(3,  3) =  I;
        J.block<3, 3>(3,  9) = -2.0f * I;
        J.block<3, 3>(3, 15) =  I;
        const Eigen::Matrix<float, 18, 6> JT = J.transpose();

        const size_t idx = 6 *  (i - 1);
        H.block<18, 18>(idx, idx) += JT * Info * J;
        b.block<18,  1>(idx,   0) += JT * Info * r;
      }
    }

    // Scan matching with normal
    float sum_sm_r = 0.0f;
    if (iter_num >= num_imu_iter) {
      num_used_points    = 0;
      num_corresp_points = 0;
      num_invalid_points = 0;
      num_valid_points   = 0;

      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Hsm = 
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(6 * num_cp, 6 * num_cp);
      Eigen::Matrix<float, Eigen::Dynamic, 1> bsm = 
        Eigen::Matrix<float, Eigen::Dynamic, 1>::Zero(6 * num_cp, 1);

      const float info = 1.0f / (scan_matching_sigma_ * scan_matching_sigma_);

      for (size_t i = 0; i < filtered_scan_cloud.size(); i += scan_step) {
        const Eigen::Vector3f& pi = filtered_scan_cloud[i];
        const double stamp = filtered_scan_stamps[i];

        Sophus::SO3f R;
        Eigen::Vector3f p;
        std::array<int, 4> idx4;
        Eigen::Matrix<float, 3, 24> dTt_dcps;
        if (!spline_.ComputePoseAndDerivatives(stamp,
          control_points_, R, p, idx4, dTt_dcps))
        {
          num_invalid_points++;
          continue;
        }

        num_used_points++;
        if (num_used_points >= static_cast<int>(num_max_scan_points_)) break;

        const Eigen::Vector3f query = R * pi + p;
        Eigen::Vector3f target, normal;
        bool has_correspondence, is_normal_valid;
        normal_map_.FindCorrespondence(query, max_correspondence_dist_,
          target, normal, has_correspondence, is_normal_valid);
        if (!has_correspondence) continue;

        num_corresp_points++;
        if (!is_normal_valid) continue;

        const Eigen::Matrix<float, 1, 3> nT = normal.transpose();
        const float r = nT * (target - query);
        const float abs_r = std::abs(r);
        const float w = (abs_r <= sm_huber_delta_) ? 1.0f : (sm_huber_delta_ / abs_r);
        sum_sm_r += abs_r;

        const Eigen::Matrix<float, 1, 3> dr_dRt = nT * Hat(R * pi);
        const Eigen::Matrix<float, 1, 3> dr_dpt = -nT;

        Eigen::Matrix<float, 1, 24> J;
        for (int j = 0; j < 4; ++j) {
          J.block<1, 3>(0,     6 * j) = dr_dRt * dTt_dcps.block<3, 3>(0,     6 * j);
          J.block<1, 3>(0, 6 * j + 3) = dr_dpt * dTt_dcps.block<3, 3>(0, 6 * j + 3);
        }
        const Eigen::Matrix<float, 24, 1> JT = J.transpose();
        const int idx = 6 * idx4[0];
        Hsm.block<24, 24>(idx, idx).noalias() += w * (JT * info * J);
        bsm.block<24,  1>(idx,   0).noalias() += w * (JT * info * r);

        num_valid_points++;
        if (num_valid_points == static_cast<int>(num_max_valid_points_)) break;
      }

      const float w = 1.0f;
      H.block(0, 0, 6 * num_cp, 6 * num_cp) += w * Hsm;
      b.block(0, 0, 6 * num_cp,          1) += w * bsm;
    }

    // IMU factor
    float sum_imu_r = 0.0f;
    {
      const Eigen::Vector3f gacc(0.0f, 0.0f, gravity_acc_);
      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Himu = 
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(num_opt_var, num_opt_var);
      Eigen::Matrix<float, Eigen::Dynamic, 1> bimu = 
        Eigen::Matrix<float, Eigen::Dynamic, 1>::Zero(num_opt_var, 1);

      const float inv_var_omega = 1.0f / (imu_mes_omega_sigma_ * imu_mes_omega_sigma_);
      const float inv_var_acc = 1.0f / (imu_mes_acc_sigma_ * imu_mes_acc_sigma_);
      Eigen::Matrix<float, 6, 6> Info = Eigen::Matrix<float, 6, 6>::Zero();
      Info.block<3, 3>(0, 0) = inv_var_omega * Eigen::Matrix3f::Identity();
      Info.block<3, 3>(3, 3) = inv_var_acc * Eigen::Matrix3f::Identity();

      for (size_t i = 0; i < relevant_imu_measures.size(); ++i) {
        const ImuMeasure& m = relevant_imu_measures[i];
        Sophus::SO3f R;
        Eigen::Vector3f omega_body, p, v, a;
        std::array<int, 4> idx4;
        Eigen::Matrix<float, 6, 24> dr_dcps;
        if (!spline_.ComputeMotionAndDerivatives(m.stamp, control_points_,
          R, omega_body, p, v, a, gravity_acc_, idx4, dr_dcps))
        {
          continue;
        }

        const Eigen::Matrix3f RT = R.matrix().transpose();
        const Eigen::Vector3f acc_body = RT * (a + gacc);
        const Eigen::Vector3f rw = omega_body - m.gyro + bg1_;
        const Eigen::Vector3f ra = acc_body - m.acc + ba1_;
        Eigen::Matrix<float, 6, 1> r;
        r.block<3, 1>(0, 0) = rw;
        r.block<3, 1>(3, 0) = ra;
        sum_imu_r += r.norm();

        const Eigen::Matrix<float, 6, 24>& Jx = dr_dcps;
        const Eigen::Matrix<float, 24, 6> JxT = Jx.transpose();

        // rw/bg1, rw/ba1
        // ra/bg1, ra/ba1
        const Eigen::Matrix<float, 6, 6> Jb = Eigen::Matrix<float, 6, 6>::Identity();
        const Eigen::Matrix<float, 6, 6> JbT = Jb.transpose();

        const int x_idx = 6 * idx4[0];
        const int b_idx = 6 * num_cp + 6;

        Himu.block<24, 24>(x_idx, x_idx).noalias() += JxT * Info * Jx;
        Himu.block<24,  6>(x_idx, b_idx).noalias() += JxT * Info * Jb;
        Himu.block<6,  24>(b_idx, x_idx).noalias() += JbT * Info * Jx;
        Himu.block<6,   6>(b_idx, b_idx).noalias() += JbT * Info * Jb;

        bimu.block<24, 1>(x_idx, 0).noalias() += JxT * Info * r;
        bimu.block<6,  1>(b_idx, 0).noalias() += JbT * Info * r;
      }

      const float w = 1.0f;
      H += w * Himu;
      b += w * bimu;
    }

    // Bias factor based on the random walk process
    {
      const Eigen::Vector3f rbg = bg1_ - bg0_;
      const Eigen::Vector3f rba = ba1_ - ba0_;
      Eigen::Matrix<float, 6, 1> r;
      r.block<3, 1>(0, 0) = rbg;
      r.block<3, 1>(3, 0) = rba;

      const float inv_var_bg = 1.0f / (gyro_bias_rw_sigma_ * gyro_bias_rw_sigma_);
      const float inv_var_ba = 1.0f / (acc_bias_rw_sigma_ * acc_bias_rw_sigma_);

      Eigen::Matrix<float, 6, 6> Info = Eigen::Matrix<float, 6, 6>::Zero();
      Info.block<3, 3>(0, 0) = inv_var_bg * Eigen::Matrix3f::Identity();
      Info.block<3, 3>(3, 3) = inv_var_ba * Eigen::Matrix3f::Identity();

      // rbg / bg0, rbg / ba0, rbg / bg1, rbg / ba1
      // rba / bg0, rba / ba0, rba / bg1, rba / ba1
      // -I, 0, I, 0
      // 0, -I, 0, I
      Eigen::Matrix<float, 6, 12> J = Eigen::Matrix<float, 6, 12>::Zero();
      J.block<3, 3>(0, 0) = -Eigen::Matrix3f::Identity();
      J.block<3, 3>(0, 6) =  Eigen::Matrix3f::Identity();
      J.block<3, 3>(3, 3) = -Eigen::Matrix3f::Identity();
      J.block<3, 3>(3, 9) =  Eigen::Matrix3f::Identity();
      const Eigen::Matrix<float, 12, 6> JT = J.transpose();

      const size_t idx = 6 * num_cp;
      H.block<12, 12>(idx, idx).noalias() += JT * Info * J;
      b.block<12,  1>(idx,   0).noalias() += JT * Info * r;
    }

    // Update
    H.diagonal().array() += gn_damping_lambda_;
    const Eigen::Matrix<float, Eigen::Dynamic, 1> dx = H.ldlt().solve(-b);
    for (size_t i = 0; i < num_cp; ++i) {
      const Eigen::Vector3f dth = dx.segment<3>(6 * i);
      const Eigen::Vector3f dp  = dx.segment<3>(6 * i + 3);
      control_points_[i].R = Sophus::SO3f::exp(dth) * control_points_[i].R;
      control_points_[i].p += dp;
    }
    bg0_ += dx.segment<3>(6 * num_cp);
    ba0_ += dx.segment<3>(6 * num_cp + 3);
    bg1_ += dx.segment<3>(6 * num_cp + 6);
    ba1_ += dx.segment<3>(6 * num_cp + 9);

    const float update_ave = dx.norm() / num_opt_var;
    inlier_ratio_ = static_cast<float>(num_corresp_points) / static_cast<float>(num_used_points);
    // std::cout << "iter: " << iter_num << ", update_ave: " << update_ave << std::endl;
    // std::cout << "ave_sm_r  = " << sum_sm_r / static_cast<float>(num_used_points) << std::endl;
    // std::cout << "ave_imu_r = " << sum_imu_r / static_cast<float>(relevant_imu_measures.size()) << std::endl;

    const Eigen::Matrix3f Hp = H.block<3, 3>(6 * num_cp - 3 - 6, 6 * num_cp - 3 - 6);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(Hp);
    if (solver.info() != Eigen::Success) {
      min_eigenvalue_ = 0.0f;
      break;
    }
    const Eigen::Vector3f eigenvalues = solver.eigenvalues();
    min_eigenvalue_       = eigenvalues(0) / eigenvalues.sum();
    min_eigenvalue_ratio_ = eigenvalues(0) / eigenvalues(2);

    if (iter_num > num_min_iter + num_imu_iter && update_ave < optimization_epsilon_) {
      // std::cout << "Optimization has converged." << std::endl;
      has_converged_ = true;
      break;
    }
  }

  if (!has_converged_) {
    std::cout << "Optimization has not been converged." << std::endl;
    state_ = pred_state;
    control_points_ = pred_control_points;
    return;
  }

  // Stats for optimization
  inlier_ratio_ = static_cast<float>(num_corresp_points) / static_cast<float>(num_used_points);
  // const float invalid_ratio = static_cast<float>(num_invalid_points) / static_cast<float>(filtered_scan_cloud.size());
  // const float valid_ratio = static_cast<float>(num_valid_points) / static_cast<float>(filtered_scan_cloud.size());
  // std::cout << "num_scan_points:      " << filtered_scan_cloud.size() << std::endl;
  // std::cout << "num_invalid_points:   " << num_invalid_points << std::endl;
  // std::cout << "num_used_points:      " << num_used_points << std::endl;
  // std::cout << "num_corresp_points:   " << num_corresp_points << std::endl;
  // std::cout << "inlier_ratio:         " << inlier_ratio_ << std::endl;
  // std::cout << "invalid_ratio:        " << invalid_ratio << std::endl;
  // std::cout << "valid_ratio:          " << valid_ratio << std::endl;
  // std::cout << "min_eigenvalue:       " << min_eigenvalue_ << std::endl;
  // std::cout << "min_eigenvalue_ratio: " << min_eigenvalue_ratio_ << std::endl;
  // std::cout << std::endl;
  
  // Estimate current state and compensate for the odometry state
  Sophus::SO3f Roi;
  Eigen::Vector3f omega_body, poi, vo, ao;
  if (spline_.ComputeMotion(end_stamp, control_points_, Roi, omega_body, poi, vo, ao)) {
    state_.R = Roi;
    state_.p = poi;
    state_.v = vo;
    state_.w = omega_body;
    state_.bg = bg1_;
    state_.ba = ba1_;
    std::lock_guard<std::mutex> lock(imu_mutex_);
    odom_state_ = state_;
    // imu_odom_state_cov_ = imu_state_cov_;
  } else {
    std::cout << "Failed to update the state." << std::endl;
  }

  // Print state
  // std::cout << "Optimized state" << std::endl;
  // std::cout << "R :\n" << state_.R.matrix() << std::endl;
  // std::cout << "p :  " << state_.p.transpose() << std::endl;
  // std::cout << "v :  " << state_.v.transpose() << std::endl;
  // std::cout << "a :  " << ao.transpose() << std::endl;
  // std::cout << "bg: " << state_.bg.transpose() << std::endl;
  // std::cout << "ba: " << state_.ba.transpose() << std::endl;
  // std::cout << "w : " << omega_body.transpose() << std::endl;
  // std::cout << std::endl;

  // Align scan clud
  aligned_scan_cloud_.resize(scan_cloud_.size());
  for (size_t i = 0; i < scan_cloud_.size(); ++i) {
    Sophus::SO3f R;
    Eigen::Vector3f p;
    if (!spline_.ComputePose(scan_stamps[i], control_points_, R, p)) {
      aligned_scan_cloud_[i] = Eigen::Vector3f::Zero();
    } else {
      aligned_scan_cloud_[i] = R * scan_cloud_[i] + p;
    }
  }

  // Pend new scan until marginalizing them
  pending_scan_cloud_.insert(pending_scan_cloud_.end(),
    scan_cloud_.begin(), scan_cloud_.end());
  pending_scan_intensities_.insert(pending_scan_intensities_.end(),
    scan_intensities_.begin(), scan_intensities_.end());
  pending_scan_stamps_.insert(pending_scan_stamps_.end(),
    scan_stamps_.begin(), scan_stamps_.end());

  // Marginalization
  if (control_points_.size() < num_max_cp_) return;
  const size_t erase_num = control_points_.size() - num_max_cp_;

  // Make marginalized scan before marginalizing control points
  PointCloud3f marginalized_scan_cloud;
  marginalized_scan_cloud.reserve(pending_scan_cloud_.size());
  const double commit_stamp = control_points_[erase_num + 3].stamp;
  size_t erase_until = 0;

  for (size_t i = 0; i < pending_scan_cloud_.size(); ++i) {
    const double stamp = pending_scan_stamps_[i];
    if (stamp > commit_stamp) break;

    Sophus::SO3f R;
    Eigen::Vector3f p;
    if (spline_.ComputePose(stamp, control_points_, R, p)) {
      const Eigen::Vector3f po = R * pending_scan_cloud_[i] + p;
      marginalized_scan_cloud.push_back(po);
    }
    erase_until = i + 1;
  }

  // Erase marginalized scans and add them to local map if necessary
  if (erase_until > 0) {
    pending_scan_cloud_.erase(
      pending_scan_cloud_.begin(), pending_scan_cloud_.begin() + erase_until);
    pending_scan_intensities_.erase(
      pending_scan_intensities_.begin(), pending_scan_intensities_.begin() + erase_until);
    pending_scan_stamps_.erase(
      pending_scan_stamps_.begin(), pending_scan_stamps_.begin() + erase_until);

    if (inlier_ratio_ < min_inlier_ratio_ && marginalized_scan_cloud.size() > 0
        && end_stamp - last_lidar_map_updated_stamp_ > 1.0)
    {
      const PointCloud3f filtered_scan_cloud = vgf.filter(marginalized_scan_cloud);
      normal_map_.AddAlignedScanCloud(filtered_scan_cloud);
      is_lidar_map_updated_ = true;
      last_lidar_map_updated_stamp_ = end_stamp;
      // std::cout << "Added " << filtered_scan_cloud.size() << " points to local map" << std::endl;
    }
  }

  // Marginalization
  control_points_.erase(control_points_.begin(), control_points_.begin() + erase_num);
}

std::vector<Sophus::SE3f> LicoInterface::GetControlPointsPoses() {
  std::vector<Sophus::SE3f> poses(control_points_.size());
  for (size_t i = 0; i < control_points_.size(); ++i) {
    const Sophus::SO3f& R = control_points_[i].R;
    const Eigen::Vector3f& p = control_points_[i].p;
    poses[i] = Sophus::SE3f(R, p);
  }
  return poses;
}

std::vector<Sophus::SE3f> LicoInterface::GetReconstructedPoses(size_t num) {
  std::vector<Sophus::SE3f> poses;
  if (control_points_.size() < 4) {
    return poses;
  }

  const double tmin = control_points_[2].stamp;
  const double tmax = control_points_.back().stamp - 1e-6;
  const double dt = (tmax - tmin) / num;

  poses.resize(num);
  for (size_t i = 0; i < num; ++i) {
    const double stamp = tmin + i * dt;
    Sophus::SO3f R;
    Eigen::Vector3f p;
    if (!spline_.ComputePose(stamp, control_points_, R, p))
      continue;
    poses[i] = Sophus::SE3f(R, p);
  }
  return poses;
}

} // namespace lico
