cmake_minimum_required(VERSION 3.8)
project(global_fusion)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

add_subdirectory(./ThirdParty/GeographicLib/)

add_executable(global_fusion_node
	src/globalOptNode.cpp
	src/globalOpt.cpp)
target_include_directories(global_fusion_node PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
  ./ThirdParty/GeographicLib/include/
)
ament_target_dependencies(global_fusion_node
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs)
target_link_libraries(global_fusion_node Ceres::ceres libGeographiccc)

install(TARGETS global_fusion_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

ament_package()
