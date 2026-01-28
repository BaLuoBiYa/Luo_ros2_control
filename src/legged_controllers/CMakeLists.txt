cmake_minimum_required(VERSION 3.8)
project(legged_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

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
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_msgs REQUIRED)

find_package(legged_wbc REQUIRED)
find_package(legged_estimator REQUIRED)
find_package(legged_interface)

find_package(pinocchio REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

add_library(${PROJECT_NAME} SHARED
  src/legged_controller.cpp
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
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_mpc
  ocs2_sqp
  ocs2_msgs
  legged_wbc
  legged_estimator
  legged_interface
)

include_directories(include)
target_include_directories(${PROJECT_NAME}
  PRIVATE
  include
)

pluginlib_export_plugin_description_file(controller_interface legged_controller_plugin.xml)

add_executable(legged_robot_sqp_mpc
  src/legged_sqp_mpc.cpp
)
target_link_libraries(legged_robot_sqp_mpc ${PROJECT_NAME})
target_compile_options(legged_robot_sqp_mpc PRIVATE ${OCS2_PINOCCHIO_FLAGS})

add_executable(legged_target
  src/legged_target.cpp
)
target_link_libraries(legged_target ${PROJECT_NAME})
target_compile_options(legged_target PRIVATE ${OCS2_PINOCCHIO_FLAGS})

install(
  TARGETS ${PROJECT_NAME}
    legged_robot_sqp_mpc
    legged_target
  DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_mpc
  ocs2_sqp
  ocs2_msgs
  legged_wbc
  legged_estimator
  legged_interface
  pinocchio
  Eigen3
  Boost
)

ament_package()
