cmake_minimum_required(VERSION 3.8)
project(ego_planner)

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
find_package(quadrotor_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(traj_utils REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs
  quadrotor_msgs plan_env path_searching bspline_opt traj_utils
)

add_executable(ego_planner_node
  src/ego_planner_node.cpp
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
)
target_include_directories(ego_planner_node PRIVATE include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(ego_planner_node ${dependencies})

add_executable(traj_server src/traj_server.cpp)
target_include_directories(traj_server PRIVATE include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(traj_server ${dependencies})

install(TARGETS ego_planner_node traj_server DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
