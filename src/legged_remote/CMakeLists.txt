cmake_minimum_required(VERSION 3.8)
project(legged_remote)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(Crow REQUIRED)
include_directories(include)

add_library(${PROJECT_NAME}
        src/webServer.cpp
)

add_executable(webServer src/webServer.cpp)
ament_target_dependencies(webServer ${dependencies})
target_link_libraries(webServer PUBLIC Crow::Crow)

install(
        TARGETS
        webServer
        DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})
ament_package()
