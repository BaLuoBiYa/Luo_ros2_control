cmake_minimum_required(VERSION 3.8)
project(legged_estimator)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ocs2_legged_robot REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/legged_estimator.cpp
)
target_include_directories(${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "controller_interface"
  "pluginlib"
  "rclcpp_lifecycle"
  "nav_msgs"
  "ocs2_legged_robot"
)

# pluginlib_export_plugin_description_file(controller_interface legged_estimator_plugin.xml)

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
  controller_interface
  pluginlib
  rclcpp_lifecycle
  ocs2_legged_robot
  tf2_ros
)

ament_package()
