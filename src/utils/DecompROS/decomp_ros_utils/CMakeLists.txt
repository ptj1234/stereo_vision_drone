cmake_minimum_required(VERSION 3.8)
project(decomp_ros_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wno-deprecated-declarations")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(decomp_ros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

add_definitions(-DQT_NO_KEYWORDS)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

add_library(decomp_rviz_plugins SHARED
  src/bound_visual.cpp
  src/mesh_visual.cpp
  src/vector_visual.cpp
  src/ellipsoid_array_visual.cpp
  src/ellipsoid_array_display.cpp
  src/ellipsoid_array_display.h
  src/polyhedron_array_display.cpp
  src/polyhedron_array_display.h)
target_include_directories(decomp_rviz_plugins PRIVATE include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(decomp_rviz_plugins
  rclcpp rviz_common rviz_rendering pluginlib decomp_ros_msgs geometry_msgs sensor_msgs nav_msgs)
target_link_libraries(decomp_rviz_plugins Qt5::Widgets)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS decomp_rviz_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# header-only decomposition library (decomp_basis/geometry/util) + ROS conversions
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY icons DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(Eigen3 decomp_ros_msgs geometry_msgs sensor_msgs nav_msgs)
ament_package()
