cmake_minimum_required(VERSION 3.8)
project(ctlio_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(yaml-cpp REQUIRED)



# ctlio_ros2_node

add_executable(ctlio_ros2_node
  src/ctlio_ros2_node.cpp
  src/lico_interface.cpp
  src/lie_algebra.cpp
  src/imu_initializer.cpp
  src/imu_preintegrator.cpp
  src/normal_map.cpp
  src/voxel_grid_filter.cpp
  src/b_spline.cpp
  src/ros_utils.cpp)

target_include_directories(ctlio_ros2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})

target_compile_features(ctlio_ros2_node PUBLIC
  c_std_99
  cxx_std_17)

ament_target_dependencies(
  ctlio_ros2_node
  "rclcpp"
  "rclpy"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2_msgs"
  "tf2_ros"
  "tf2_eigen"
  "cv_bridge"
)

target_link_libraries(ctlio_ros2_node
  yaml-cpp)

install(TARGETS ctlio_ros2_node DESTINATION
  lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)
