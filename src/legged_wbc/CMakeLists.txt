cmake_minimum_required(VERSION 3.8)
project(legged_wbc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ocs2_legged_robot REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(qpoases_colcon REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
    src/HierarchicalWbc.cpp
    src/HoQp.cpp
    src/WbcBase.cpp
    src/WeightedWbc.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "ocs2_legged_robot"
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
  ocs2_legged_robot
  qpoases_colcon
)

ament_package()
