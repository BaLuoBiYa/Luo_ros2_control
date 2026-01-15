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
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/motor_sim.cpp
  src/imu_sim.cpp
  src/contact_sim.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rclcpp_lifecycle"
  "pluginlib"
  "hardware_interface"
  "sensor_msgs"
  "trajectory_msgs"
)

pluginlib_export_plugin_description_file(hardware_interface sim_interface.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  pluginlib
  hardware_interface
  sensor_msgs
  trajectory_msgs
)

ament_package()
