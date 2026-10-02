cmake_minimum_required(VERSION 3.8)
project(odom_visualization)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pose_utils REQUIRED)
find_package(plan_env REQUIRED)

add_executable(odom_visualization src/odom_visualization.cpp)
ament_target_dependencies(odom_visualization
  rclcpp sensor_msgs nav_msgs geometry_msgs visualization_msgs quadrotor_msgs tf2_ros pose_utils plan_env)

install(TARGETS odom_visualization DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})

ament_package()
