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
#include <iomanip>
#include <mutex>

#include <boost/circular_buffer.hpp>

#include <yaml-cpp/yaml.h>

#include <ctlio_ros2/types.hpp>
#include <ctlio_ros2/lie_algebra.hpp>
#include <ctlio_ros2/imu_initializer.hpp>
#include <ctlio_ros2/imu_preintegrator.hpp>
#include <ctlio_ros2/voxel_grid_filter.hpp>
#include <ctlio_ros2/normal_map.hpp>
#include <ctlio_ros2/b_spline.hpp>

namespace lico {

class LicoInterface {
private:
  std::string param_files_dir_;

  State state_;
  State odom_state_;
  Sophus::SE3f Til_; // p_imu    = Til p_lidar
  Sophus::SE3f Tcl_; // p_camera = Tcl p_lidar
  mutable std::mutex odom_mutex_;

  std::mutex imu_mutex_;
  bool is_imu_initialized_{false};
  double stationary_sec_{2.0};
  float gravity_acc_{9.80f};
  float acc_scale_{9.80f};
  boost::circular_buffer<ImuMeasure> imu_buf_;

  float scan_cloud_clip_range_{1.0f};   // m
  float scan_cloud_filter_size_{0.1f};  // m
  PointCloud3f scan_cloud_;
  std::vector<float> scan_intensities_;
  std::vector<double> scan_stamps_;
  PointCloud3f pending_scan_cloud_;
  std::vector<float> pending_scan_intensities_;
  std::vector<double> pending_scan_stamps_;
  PointCloud3f aligned_scan_cloud_;

  ImuPreintegrator preintegrator_;

  bool is_lidar_map_initialized_{false};
  bool is_lidar_map_updated_{false};
  double last_lidar_map_updated_stamp_{0};
  NormalMap normal_map_;

  BSpline spline_;

  float inlier_ratio_;
  float min_eigenvalue_;
  float min_eigenvalue_ratio_;
  bool has_converged_;

  double control_points_interval_{0.02};
  size_t num_max_cp_{8};
  std::vector<ControlPoint> control_points_;
  Eigen::Vector3f bg0_, bg1_;
  Eigen::Vector3f ba0_, ba1_;

  float min_inlier_ratio_{0.95f};       // %
  float max_correspondence_dist_{1.0f}; // m
  float sm_huber_delta_{0.1f};          // m
  int num_max_scan_points_{10000};
  int num_max_valid_points_{10000};
  float gn_damping_lambda_{0.0f};
  float prior_dR_sigma_{0.001f};
  float prior_dp_sigma_{0.001f};
  float prior_bg_sigma_{0.001f};
  float prior_ba_sigma_{0.001f};
  float ct_traj_dR_sigma_{0.00025f};
  float ct_traj_dp_sigma_{0.00025f};
  float scan_matching_sigma_{0.05f};
  float imu_mes_omega_sigma_{0.05f};
  float imu_mes_acc_sigma_{0.5f};
  float gyro_bias_rw_sigma_{0.01f};
  float acc_bias_rw_sigma_{0.01f};

public:
  LicoInterface();

  ~LicoInterface();

  // Getter and Setter
  inline void SetParamFilesDir(const std::string& dir) { param_files_dir_ = dir; }
  
  bool IsLidarMapUpdated() const { return is_lidar_map_updated_; }

  void SetLidarMapUpdated(bool flag) { is_lidar_map_updated_ = flag; }

  bool IsImuInitialized() const { return is_imu_initialized_; }

  inline Sophus::SE3f GetIMUPose() const { return Sophus::SE3f(state_.R, state_.p); }

  inline PointCloud3f GetAlignedScanCloud() const { return aligned_scan_cloud_; }

  inline std::vector<float> GetClippedScanIntensities() const { return scan_intensities_; }

  inline std::vector<double> GetClippedScanStamps() const { return scan_stamps_; }

  inline PointCloud3f GetNormalMapCloud() const { return normal_map_.GetMapCloud(); }

  std::vector<Sophus::SE3f> GetControlPointsPoses();

  std::vector<Sophus::SE3f> GetReconstructedPoses(size_t num);

  inline State GetOdomState() const {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    return odom_state_;
  }

  void ReadParams();

  void SetImuMeasure(const ImuMeasure& measure);

  void SetScanCloudCtlio(
    const PointCloud3f& scan_cloud,
    const std::vector<float>& scan_intensities,
    const std::vector<double>& scan_stamps);
}; // class LicoInterface

} // namespace lico
