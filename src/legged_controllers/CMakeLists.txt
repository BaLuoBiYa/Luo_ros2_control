cmake_minimum_required(VERSION 3.8)
project(legged_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(angles REQUIRED)
find_package(std_msgs REQUIRED)

find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)

find_package(ocs2_legged_robot REQUIRED)
find_package(ocs2_legged_robot_ros REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)

find_package(legged_wbc REQUIRED)
find_package(legged_estimator REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/legged_controller.cpp
  src/legged_mrt.cpp
  src/test_tools/motorTester.cpp
  src/test_tools/contactTester.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  angles
  controller_interface
  pluginlib
  std_msgs
  hardware_interface
  ocs2_legged_robot
  ocs2_legged_robot_ros
  ocs2_ros_interfaces
  legged_wbc
  legged_estimator
)

include_directories(include)
target_include_directories(${PROJECT_NAME}
  PRIVATE
  include
)

pluginlib_export_plugin_description_file(controller_interface legged_controller_plugin.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(
  rclcpp
  angles
  controller_interface
  pluginlib
  std_msgs
  hardware_interface
  ocs2_legged_robot
  ocs2_legged_robot_ros
  ocs2_ros_interfaces
  legged_wbc
  legged_estimator
)

ament_package()
