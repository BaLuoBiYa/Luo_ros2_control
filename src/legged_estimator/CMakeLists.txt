cmake_minimum_required(VERSION 3.8)
project(legged_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(ocs2_legged_robot REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)

find_package(pinocchio REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/FromTopicEstimate.cpp
  src/LinearKalmanFilter.cpp
  src/StateEstimateBase.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rclcpp_lifecycle"
  "realtime_tools"
  "nav_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "ocs2_legged_robot"
  "ocs2_centroidal_model"
  "ocs2_core"
  "ocs2_pinocchio_interface"
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  ocs2_legged_robot
  ocs2_centroidal_model
  ocs2_core
  ocs2_pinocchio_interface
  pinocchio
  Eigen3
  Boost
)

ament_package()
