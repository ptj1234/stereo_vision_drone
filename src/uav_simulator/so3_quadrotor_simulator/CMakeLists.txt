cmake_minimum_required(VERSION 3.8)
project(so3_quadrotor_simulator)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(uav_utils REQUIRED)
find_package(plan_env REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(quadrotor_dynamics src/dynamics/Quadrotor.cpp)
target_include_directories(quadrotor_dynamics PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ode
  ${EIGEN3_INCLUDE_DIRS})

add_executable(quadrotor_simulator_so3 src/quadrotor_simulator_so3.cpp)
ament_target_dependencies(quadrotor_simulator_so3
  rclcpp nav_msgs sensor_msgs geometry_msgs quadrotor_msgs uav_utils plan_env)
target_link_libraries(quadrotor_simulator_so3 quadrotor_dynamics)

install(TARGETS quadrotor_simulator_so3 DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
