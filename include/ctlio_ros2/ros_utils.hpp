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
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <ctlio_ros2/types.hpp>

void ParseLivoxCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg,
  lico::PointCloud3f& scan_cloud,
  std::vector<float>& scan_intensities,
  std::vector<double>& scan_stamps);

void ParseOusterCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg,
  double first_stamp,
  lico::PointCloud3f& scan_cloud,
  std::vector<float>& scan_intensities,
  std::vector<double>& scan_stamps);

void ParseCtlioRos2Cloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,
  lico::PointCloud3f& scan_cloud,
  std::vector<float>& scan_intensities,
  std::vector<double>& scan_stamps);

void ParseLicoRos2Cloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg,
  lico::PointCloud3f& scan_cloud);

void PublishPose(
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub,
  const std::string& frame_id,
  const builtin_interfaces::msg::Time& stamp,
  const Sophus::SE3f& T);

void PublishPoseArray(
  const rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub,
  const std::string& frame_id,
  const builtin_interfaces::msg::Time& stamp,
  const std::vector<Sophus::SE3f>& Ts);

void PublishOdometry(
  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub,
  const std::string& frame_id,
  const std::string& child_frame_id,
  const builtin_interfaces::msg::Time& stamp,
  const lico::State& state);

void BroadcastTransform(
  const std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster,
  const std::string& parent_frame,
  const std::string& child_frame,
  const builtin_interfaces::msg::Time& stamp,
  const Sophus::SE3f& T);

void PublishPointCloud(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
  const std::string& frame_id,
  const builtin_interfaces::msg::Time& stamp,
  const lico::PointCloud3f& cloud);
  
void PublishPointCloud(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
  const std::string& frame_id,
  const builtin_interfaces::msg::Time& stamp,
  const lico::PointCloud3f& cloud,
  const std::vector<float>& scan_intensities,
  const std::vector<double>& scan_stamps);

/*
void PublishePointMarkers(
  const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
  const std::string& frame_id,
  const builtin_interfaces::msg::Time& stamp,
  const std::string& ns,
  int id,
  float scale,
  float r,
  float g,
  float b,
  float a,
  const lico::PointCloud3f& nodes);
*/

/*
void PublishLineMarkers(
  const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
  const std::string& frame_id,
  const builtin_interfaces::msg::Time& stamp,
  const std::string& ns,
  int id,
  float line_width,
  float r,
  float g,
  float b,
  float a,
  const lico::PointCloud3f& start_points,
  const lico::PointCloud3f& end_points);
*/
