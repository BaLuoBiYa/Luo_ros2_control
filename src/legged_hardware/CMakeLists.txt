cmake_minimum_required(VERSION 3.8)
project(legged_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)

find_package(gz-sim8 REQUIRED) 
find_package(gz-utils2 REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

include_directories(include)

include_directories(
  include
  ${gz-transport13_INCLUDE_DIRS}
  ${gz-msgs10_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/imu_sim.cpp
  src/contact_sim.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  pluginlib
  hardware_interface
  sensor_msgs
  trajectory_msgs
  ros_gz_interfaces
  gz_ros2_control
)

pluginlib_export_plugin_description_file(gz_ros2_control sim_interface.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  pluginlib
  hardware_interface
  sensor_msgs
  trajectory_msgs
  ros_gz_interfaces
  gz_ros2_control
  gz-sim8
  gz-utils2                 
  gz-transport13
  gz-msgs10
)

ament_package()
