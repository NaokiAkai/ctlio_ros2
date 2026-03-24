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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <chrono>

#include <ctlio_ros2/lico_interface.hpp>
#include <ctlio_ros2/ros_utils.hpp>

class CtlioRos2Node : public rclcpp::Node {
public:
  CtlioRos2Node()
  : Node("ctlio_ros2_node") {
    // Get the LiDAR type to parse PointCloud2 messages
    this->declare_parameter<std::string>("lidar_type", "livox");    
    this->get_parameter("lidar_type", lidar_type_);
    if (lidar_type_ == "livox") {
      RCLCPP_INFO(this->get_logger(), "LiDAR type: Livox");
    } else if (lidar_type_ == "ouster") {
      RCLCPP_INFO(this->get_logger(), "LiDAR type: Ouster");
    } else {
      std::cout << "LiDAR type: Unknown (" << lidar_type_ << ")" << std::endl;
      RCLCPP_WARN(this->get_logger(),
        "Unknown LiDAR type '%s'.\n"
        "The point cloud will be parsed assuming the following fields:\n"
        "- x, y, z: float32\n"
        "- intensity: float32\n"
        "- timestamp: float64 (in seconds)\n",
        lidar_type_.c_str());
    }

    this->declare_parameter<std::string>("param_files_dir", "");
    std::string param_files_dir = "";
    this->get_parameter("param_files_dir", param_files_dir);
    lico_.SetParamFilesDir(param_files_dir);
    RCLCPP_INFO(this->get_logger(), "LICO parameters dir: %s", param_files_dir.c_str());

    this->declare_parameter<std::string>("pointcloud_topic", "/livox/lidar");
    std::string pointcloud_topic;
    this->get_parameter("pointcloud_topic", pointcloud_topic);
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, rclcpp::SensorDataQoS().keep_last(10),
      std::bind(&CtlioRos2Node::PointcloudCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "pointcloud_topic: %s", pointcloud_topic.c_str());

    this->declare_parameter<std::string>("imu_topic", "/livox/imu");
    std::string imu_topic;
    this->get_parameter("imu_topic", imu_topic);
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS().keep_last(10000),
      std::bind(&CtlioRos2Node::ImuCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "imu_topic: %s", imu_topic.c_str());

    this->declare_parameter<std::string>("imu_pose_topic", "/ctlio/imu_pose");
    std::string imu_pose_topic;
    this->get_parameter("imu_pose_topic", imu_pose_topic);
    imu_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      imu_pose_topic, 10);
    RCLCPP_INFO(this->get_logger(), "imu_pose_topic: %s", imu_pose_topic.c_str());

    this->declare_parameter<std::string>("imu_odom_topic", "/ctlio/imu_odom");
    std::string imu_odom_topic;
    this->get_parameter("imu_odom_topic", imu_odom_topic);
    imu_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      imu_odom_topic, 100);
    RCLCPP_INFO(this->get_logger(), "imu_odom_topic: %s", imu_odom_topic.c_str());

    this->declare_parameter<std::string>("lico_map_cloud_topic", "/ctlio/lio_map_cloud");
    std::string lico_map_cloud_topic;
    this->get_parameter("lico_map_cloud_topic", lico_map_cloud_topic);
    lico_map_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      lico_map_cloud_topic, rclcpp::QoS(1).transient_local().reliable());
    RCLCPP_INFO(this->get_logger(), "lico_map_cloud_topic: %s", lico_map_cloud_topic.c_str());

    this->declare_parameter<std::string>("aligned_scan_cloud_topic", "/ctlio/aligned_scan_cloud");
    std::string aligned_scan_cloud_topic;
    this->get_parameter("aligned_scan_cloud_topic", aligned_scan_cloud_topic);
    aligned_scan_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      aligned_scan_cloud_topic, 10);
    RCLCPP_INFO(this->get_logger(), "aligned_scan_cloud_topic: %s", aligned_scan_cloud_topic.c_str());

    this->declare_parameter<std::string>("imu_poses_topic", "/ctlio/imu_poses");
    std::string imu_poses_topic;
    this->get_parameter("imu_poses_topic", imu_poses_topic);
    imu_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      imu_poses_topic, 10);

    this->declare_parameter<std::string>("cp_poses_topic", "/ctlio/cp_poses");
    std::string cp_poses_topic;
    this->get_parameter("cp_poses_topic", cp_poses_topic);
    cp_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      cp_poses_topic, 10);

    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("imu_frame", "imu");
    this->get_parameter("odom_frame", odom_frame_);
    this->get_parameter("imu_frame", imu_frame_);
    RCLCPP_INFO(this->get_logger(), "odom_frame: %s", odom_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "imu_frame: %s", imu_frame_.c_str());

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    lico_.ReadParams();

    RCLCPP_INFO(this->get_logger(), "CTLIO ROS2 node initialized");
  }

private:
  void PointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Parse the PointCloud2 message
    lico::PointCloud3f scan_cloud;
    std::vector<float> scan_intensities;
    std::vector<double> scan_stamps;
    if (lidar_type_ == "livox") {
      ParseLivoxCloud(msg, scan_cloud, scan_intensities, scan_stamps);
    } else if (lidar_type_ == "ouster") {
      ParseOusterCloud(msg, rclcpp::Time(msg->header.stamp).seconds(),
        scan_cloud, scan_intensities, scan_stamps);
    } else {
      ParseLicoRos2Cloud(msg, scan_cloud, scan_intensities, scan_stamps);
    }

    const auto t1 = std::chrono::high_resolution_clock::now();

    // The main process of LICO
    lico_.SetScanCloudCtlio(scan_cloud, scan_intensities, scan_stamps);

    const auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "elapsed time for optimization [msec]: " 
      << std::chrono::duration_cast<std::chrono::milliseconds>
         (t2 - t1).count() << std::endl;

    // Publish ROS messages
    const Sophus::SE3f imu_pose = lico_.GetIMUPose();
    PublishPose(imu_pose_pub_, odom_frame_, msg->header.stamp, imu_pose);

    const std::vector<Sophus::SE3f> cp_poses = lico_.GetControlPointsPoses();
    PublishPoseArray(cp_poses_pub_, odom_frame_, msg->header.stamp, cp_poses);

    const std::vector<Sophus::SE3f> imu_poses = lico_.GetReconstructedPoses(1000);
    PublishPoseArray(imu_poses_pub_, odom_frame_, msg->header.stamp, imu_poses);

    BroadcastTransform(tf_broadcaster_, odom_frame_, imu_frame_,
      msg->header.stamp, imu_pose);

    const lico::PointCloud3f aligned_scan_cloud = lico_.GetAlignedScanCloud();
    const std::vector<float> clipped_scan_intensities = lico_.GetClippedScanIntensities();
    const std::vector<double> clipped_scan_stamps = lico_.GetClippedScanStamps();
    PublishPointCloud(aligned_scan_cloud_pub_, odom_frame_, msg->header.stamp,
      aligned_scan_cloud, clipped_scan_intensities, clipped_scan_stamps);

    if (lico_.IsLidarMapUpdated()) {
      const lico::PointCloud3f lico_map_cloud = lico_.GetNormalMapCloud();
      PublishPointCloud(lico_map_cloud_pub_, odom_frame_, msg->header.stamp, lico_map_cloud);
      lico_.SetLidarMapUpdated(false);
    }
  }

  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    const double stamp = rclcpp::Time(msg->header.stamp).seconds();
    double dt;
    if (is_imu_first_callback_) {
      prev_imu_stamp_ = stamp;
      dt = 1.0 / 200.0;
      is_imu_first_callback_ = false;
    } else {
      dt = stamp - prev_imu_stamp_;
      prev_imu_stamp_ = stamp;
    }

    lico::ImuMeasure measure;
    measure.stamp = stamp;
    measure.dt = dt;
    measure.acc = Eigen::Vector3f(msg->linear_acceleration.x,
      msg->linear_acceleration.y, msg->linear_acceleration.z);
    measure.gyro = Eigen::Vector3f(msg->angular_velocity.x,
      msg->angular_velocity.y, msg->angular_velocity.z);

    lico_.SetImuMeasure(measure);

    if (!lico_.IsImuInitialized()) return;

    // Publish odometry
    const lico::State odom_state = lico_.GetOdomState();
    PublishOdometry(imu_odom_pub_, odom_frame_, imu_frame_, msg->header.stamp, odom_state);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr imu_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr imu_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lico_map_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_scan_cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr imu_poses_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cp_poses_pub_;

  std::string lidar_type_;

  std::string odom_frame_;
  std::string imu_frame_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  lico::LicoInterface lico_;

  bool is_imu_first_callback_{true};
  double prev_imu_stamp_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CtlioRos2Node>());
  rclcpp::shutdown();
  return 0;
}
