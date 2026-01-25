cmake_minimum_required(VERSION 3.10)
project(legged_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)

# 依赖的 ROS2/第三方包
find_package(ocs2_legged_robot REQUIRED)
find_package(ocs2_self_collision REQUIRED)

find_package(pinocchio REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

add_library(${PROJECT_NAME}
        src/constraint/EndEffectorLinearConstraint.cpp
        src/constraint/FrictionConeConstraint.cpp
        src/constraint/ZeroForceConstraint.cpp
        src/constraint/NormalVelocityConstraintCppAd.cpp
        src/constraint/ZeroVelocityConstraintCppAd.cpp
        src/constraint/SwingTrajectoryPlanner.cpp
        src/initialization/LeggedRobotInitializer.cpp
        src/SwitchedModelReferenceManager.cpp
        src/LeggedRobotPreComputation.cpp
        src/LeggedInterface.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${pinocchio_LIBRARIES}
        ocs2_legged_robot::ocs2_legged_robot
        ocs2_self_collision::ocs2_self_collision
)

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
        pinocchio
        Eigen3
        Boost
        ocs2_legged_robot 
        ocs2_self_collision)

ament_package()
