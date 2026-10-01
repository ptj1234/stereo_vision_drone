cmake_minimum_required(VERSION 3.8)
project(uav_utils)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(uav_utils INTERFACE)
target_include_directories(uav_utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(uav_utils INTERFACE nav_msgs geometry_msgs)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS uav_utils EXPORT export_uav_utils)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}-test src/${PROJECT_NAME}_test.cpp)
  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test uav_utils)
  endif()
endif()

ament_export_targets(export_uav_utils)
ament_export_dependencies(nav_msgs geometry_msgs Eigen3)
ament_package()
