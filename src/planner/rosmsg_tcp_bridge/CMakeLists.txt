cmake_minimum_required(VERSION 3.8)
project(rosmsg_tcp_bridge)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(traj_utils REQUIRED)
find_package(plan_env REQUIRED)

add_executable(bridge_node src/bridge_node.cpp)
ament_target_dependencies(bridge_node rclcpp std_msgs nav_msgs traj_utils plan_env)

install(TARGETS bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
