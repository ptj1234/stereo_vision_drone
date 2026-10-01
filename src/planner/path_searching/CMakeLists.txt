cmake_minimum_required(VERSION 3.8)
project(path_searching)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plan_env REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(path_searching src/dyn_a_star.cpp)
target_include_directories(path_searching PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(path_searching rclcpp plan_env)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS path_searching EXPORT export_path_searching
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_path_searching HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp plan_env Eigen3)
ament_package()
