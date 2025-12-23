cmake_minimum_required(VERSION 3.8)
project(qpoases_colcon)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
add_subdirectory(third_party/qpOASES)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
target_include_directories(qpOASES PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/qpOASES/include>
  $<INSTALL_INTERFACE:include>
)

include_directories(
  include
  third_party/qpOASES/include
)

install(
  DIRECTORY third_party/qpOASES/include/
  DESTINATION include
)
install(
  TARGETS qpOASES
  EXPORT export_qpoases_colcon
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/qpOASES/include
  include
)
ament_export_libraries(qpOASES)
ament_export_targets(export_qpoases_colcon HAS_LIBRARY_TARGET)

ament_package()
